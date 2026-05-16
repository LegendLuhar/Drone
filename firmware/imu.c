/*
 * imu.c — ICM-42688-P driver over SPI1, polled, manual CS.
 *
 * SPI bit clock: f_periph / ((CLKCTL+1) * 2). With SPI1 fed by SYSCLK = MCLK
 * = 16 MHz and CLKCTL = 1, bit clock = 4 MHz — well under the "Standard pad"
 * safe-zone (~10 MHz) and gives ~28 µs for the 14-byte sample burst, which
 * fits comfortably inside a single 100 µs scheduler tick.
 *
 * Coexistence with DShot:
 *   - GPIOB power-up is gated on RESETSTKY so we don't reset GPIOB out from
 *     under dshot_init().
 *   - IMU pins (PB6/8/14/16) are disjoint from motor pins (PB17..20).
 *   - SPI1 isn't used by DShot.
 *   - All SPI transfers are polled in thread context. The DShot 10 kHz timer
 *     ISR preempts the SPI spinloops freely.
 */

#include "imu.h"
#include "delay.h"
#include <ti/devices/msp/msp.h>

/* ---- ICM-42688-P register map (User Bank 0 — the default) ------------ */

#define ICM_REG_DEVICE_CONFIG   0x11   /* bit 0 = SOFT_RESET_CONFIG */
#define ICM_REG_TEMP_DATA1      0x1D   /* big-endian 14-byte burst starts */
#define ICM_REG_PWR_MGMT0       0x4E
#define ICM_REG_GYRO_CONFIG0    0x4F
#define ICM_REG_ACCEL_CONFIG0   0x50
#define ICM_REG_WHO_AM_I        0x75
#define ICM_WHO_AM_I_VAL        0x47

#define ICM_READ_BIT            0x80

/* PWR_MGMT0 — gyro mode = LN (11), accel mode = LN (11), TEMP enabled.
 * Bit layout: [TEMP_DIS:5][IDLE:4][GYRO_MODE:3:2][ACCEL_MODE:1:0]. */
#define ICM_PWR_MGMT0_LN_BOTH   ((3u << 2) | (3u << 0))

/* GYRO_CONFIG0 — FS_SEL=000 (±2000 dps), ODR=0110 (1 kHz). */
#define ICM_GYRO_CONFIG0_2000DPS_1KHZ  ((0u << 5) | (6u << 0))

/* ACCEL_CONFIG0 — FS_SEL=000 (±16g), ODR=0110 (1 kHz). */
#define ICM_ACCEL_CONFIG0_16G_1KHZ     ((0u << 5) | (6u << 0))

/* ---- PINCM indices for SPI1 pins on this board ----------------------- */

#define IMU_CS_PIN              6u                /* PB6 */
#define PINCM_PB6_CS            (23u - 1u)
#define PINCM_PB8_PICO          (25u - 1u)
#define PINCM_PB14_POCI         (31u - 1u)
#define PINCM_PB16_SCLK         (33u - 1u)

/* PF=1 on these PINCMs is GPIO; PF=3 is SPI1 alt function. */
#define IMU_PF_GPIO             (1u)

#define IMU_SPI_CLKCTL          (1u)              /* → 4 MHz bit clock */

/* ---- SWD-visible state ---------------------------------------------- */

volatile bool         imu_ready          = false;
volatile uint8_t      imu_who_am_i_seen  = 0;
volatile uint8_t      imu_init_status    = IMU_STATUS_BOOT;
volatile uint32_t     imu_sample_count   = 0;
volatile uint32_t     imu_read_failures  = 0;
volatile imu_sample_t imu_latest_sample  = {0};

/* ---- Low-level SPI1 + CS helpers ------------------------------------ */

static void imu_cs_assert(void)
{
    GPIOB->DOUTCLR31_0 = (1u << IMU_CS_PIN);
    delay_cycles(10);
}

static void imu_cs_deassert(void)
{
    delay_cycles(10);
    GPIOB->DOUTSET31_0 = (1u << IMU_CS_PIN);
}

static void imu_drain_rx(void)
{
    while ((SPI1->STAT & SPI_STAT_RFE_MASK) == SPI_STAT_RFE_NOT_EMPTY) {
        (void)SPI1->RXDATA;
    }
}

/* Bound on a single STAT poll. One byte at 4 MHz shifts in ~2 µs (~32 MCLK
 * cycles); 100k loop iterations is vastly more than that, so this only ever
 * trips on a genuinely stuck bus — turning a hang into an imu_read_failures
 * increment instead of a frozen main loop. */
#define IMU_XFER_SPIN_GUARD     100000u

/* Push len bytes from tx[] and capture len bytes into rx[] (rx may be NULL
 * to discard). Polled, no DMA.
 *
 * TX and RX are interleaved one byte at a time: SPI1's TX/RX FIFOs are only a
 * few entries deep, so queuing a whole multi-byte burst before draining RX
 * overflows the RX FIFO and stalls the bus. Keeping a single byte in flight
 * makes any burst length safe. Returns false on len==0 or a stuck bus. */
static bool imu_xfer(const uint8_t *tx, uint8_t *rx, uint8_t len)
{
    if (len == 0u) {
        return false;
    }

    imu_drain_rx();

    for (uint8_t i = 0u; i < len; ++i) {
        uint32_t guard = IMU_XFER_SPIN_GUARD;
        while ((SPI1->STAT & SPI_STAT_TNF_MASK) == SPI_STAT_TNF_FULL) {
            if (--guard == 0u) {
                return false;
            }
        }
        SPI1->TXDATA = tx[i];

        guard = IMU_XFER_SPIN_GUARD;
        while ((SPI1->STAT & SPI_STAT_RFE_MASK) == SPI_STAT_RFE_EMPTY) {
            if (--guard == 0u) {
                return false;
            }
        }
        uint8_t b = (uint8_t)SPI1->RXDATA;
        if (rx != 0) {
            rx[i] = b;
        }
    }

    uint32_t guard = IMU_XFER_SPIN_GUARD;
    while ((SPI1->STAT & SPI_STAT_BUSY_MASK) == SPI_STAT_BUSY_ACTIVE) {
        if (--guard == 0u) {
            return false;
        }
    }
    return true;
}

static bool imu_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { (uint8_t)(reg & 0x7Fu), val };
    imu_cs_assert();
    bool ok = imu_xfer(tx, 0, 2u);
    imu_cs_deassert();
    return ok;
}

static bool imu_read_burst(uint8_t reg, uint8_t *out, uint8_t len)
{
    /* One CS-low transaction: [reg|READ][N bytes shifted in].
     * ICM-42688 auto-increments the register pointer on burst, no extra bit. */
    uint8_t tx[16] = {0};
    uint8_t rx[16] = {0};
    if ((out == 0) || (len == 0u) || (len > 15u)) {
        return false;
    }
    tx[0] = (uint8_t)(reg | ICM_READ_BIT);
    imu_cs_assert();
    bool ok = imu_xfer(tx, rx, (uint8_t)(len + 1u));
    imu_cs_deassert();
    if (!ok) {
        return false;
    }
    for (uint8_t i = 0u; i < len; ++i) {
        out[i] = rx[i + 1u];
    }
    return true;
}

/* ---- One-time pin / peripheral bring-up ----------------------------- */

static void imu_init_gpio_cs(void)
{
    /* Idempotent GPIOB power-up — only RESETASSERT if the block has never
     * been brought out of reset this boot. Matches the dshot.c guard so the
     * two drivers can initialize in either order without clobbering each
     * other's pin state. */
    if ((GPIOB->GPRCM.STAT & GPIO_STAT_RESETSTKY_MASK) != 0u) {
        GPIOB->GPRCM.RSTCTL = (GPIO_RSTCTL_KEY_UNLOCK_W
                               | GPIO_RSTCTL_RESETSTKYCLR_CLR
                               | GPIO_RSTCTL_RESETASSERT_ASSERT);
        GPIOB->GPRCM.PWREN  = (GPIO_PWREN_KEY_UNLOCK_W
                               | GPIO_PWREN_ENABLE_ENABLE);
        delay_cycles(POWER_STARTUP_DELAY);
    }

    /* CS idle-high before we mux the pin so we never glitch the bus low. */
    GPIOB->DOUTSET31_0 = (1u << IMU_CS_PIN);
    GPIOB->DOESET31_0  = (1u << IMU_CS_PIN);
    IOMUX->SECCFG.PINCM[PINCM_PB6_CS] =
        IOMUX_PINCM_PC_CONNECTED | IMU_PF_GPIO;
}

static void imu_init_spi_pins(void)
{
    IOMUX->SECCFG.PINCM[PINCM_PB8_PICO] =
        IOMUX_PINCM_PC_CONNECTED | IOMUX_PINCM25_PF_SPI1_PICO;
    IOMUX->SECCFG.PINCM[PINCM_PB14_POCI] =
        IOMUX_PINCM_PC_CONNECTED | IOMUX_PINCM_INENA_ENABLE
        | IOMUX_PINCM31_PF_SPI1_POCI;
    IOMUX->SECCFG.PINCM[PINCM_PB16_SCLK] =
        IOMUX_PINCM_PC_CONNECTED | IOMUX_PINCM33_PF_SPI1_SCLK;
}

static void imu_init_spi_peripheral(void)
{
    SPI1->GPRCM.RSTCTL = (SPI_RSTCTL_KEY_UNLOCK_W
                          | SPI_RSTCTL_RESETSTKYCLR_CLR
                          | SPI_RSTCTL_RESETASSERT_ASSERT);
    SPI1->GPRCM.PWREN  = (SPI_PWREN_KEY_UNLOCK_W
                          | SPI_PWREN_ENABLE_ENABLE);
    delay_cycles(POWER_STARTUP_DELAY);

    SPI1->CLKSEL = (uint32_t)SPI_CLKSEL_SYSCLK_SEL_ENABLE;
    SPI1->CLKDIV = (uint32_t)SPI_CLKDIV_RATIO_DIV_BY_1;

    /* Controller, no parity, MSB first. */
    SPI1->CTL1 = SPI_CTL1_CP_ENABLE
                 | SPI_CTL1_PREN_DISABLE
                 | SPI_CTL1_PTEN_DISABLE
                 | SPI_CTL1_MSB_ENABLE;

    /* Mode 0 (SPO=0, SPH=0), Motorola 4-wire, 8-bit data. CS is driven
     * manually as GPIO, so don't enable any CSSEL bits. */
    SPI1->CTL0 = SPI_CTL0_SPO_LOW
                 | SPI_CTL0_SPH_FIRST
                 | SPI_CTL0_FRF_MOTOROLA_4WIRE
                 | SPI_CTL0_DSS_DSS_8;
    SPI1->CLKCTL = IMU_SPI_CLKCTL;
    SPI1->CTL1 |= SPI_CTL1_ENABLE_ENABLE;
}

/* ---- Public API ----------------------------------------------------- */

void imu_init(void)
{
    imu_ready = false;
    imu_init_status = IMU_STATUS_BOOT;

    imu_init_gpio_cs();
    imu_init_spi_pins();
    imu_init_spi_peripheral();

    /* Datasheet 14.6: after power-on the chip needs ~1 ms before SPI is
     * reliable. 320000 cycles @ 16 MHz = 20 ms — same value Simon_accel uses,
     * plenty of margin against any boot-rail ramp. */
    delay_cycles(320000);

    /* Soft reset → wait 1 ms (datasheet 14.36). */
    (void)imu_write_reg(ICM_REG_DEVICE_CONFIG, 0x01);
    delay_cycles(16000);

    /* WHO_AM_I — up to 4 attempts to ride out any final settle. */
    uint8_t who = 0u;
    bool found = false;
    for (int i = 0; i < 4; ++i) {
        if (imu_read_burst(ICM_REG_WHO_AM_I, &who, 1u)) {
            imu_who_am_i_seen = who;
            if (who == ICM_WHO_AM_I_VAL) {
                found = true;
                break;
            }
        }
        delay_cycles(1600);  /* ~100 µs */
    }
    if (!found) {
        imu_init_status = IMU_STATUS_WHOAMI_FAIL;
        return;
    }

    if (!imu_write_reg(ICM_REG_PWR_MGMT0, ICM_PWR_MGMT0_LN_BOTH)) {
        imu_init_status = IMU_STATUS_CONFIG_WRITE_FAIL;
        return;
    }
    /* Datasheet 14.36: must wait 200 µs after PWR_MGMT0 change before any
     * register read/write. 3200 cycles @ 16 MHz = 200 µs. */
    delay_cycles(3200);

    if (!imu_write_reg(ICM_REG_GYRO_CONFIG0,  ICM_GYRO_CONFIG0_2000DPS_1KHZ)
     || !imu_write_reg(ICM_REG_ACCEL_CONFIG0, ICM_ACCEL_CONFIG0_16G_1KHZ)) {
        imu_init_status = IMU_STATUS_CONFIG_WRITE_FAIL;
        return;
    }

    /* Let the first sample settle (one ODR period at 1 kHz = 1 ms). */
    delay_cycles(16000);

    imu_ready = true;
    imu_init_status = IMU_STATUS_OK;
}

bool imu_read_sample(imu_sample_t *out)
{
    if (!imu_ready || (out == 0)) {
        imu_read_failures++;
        return false;
    }

    uint8_t buf[14];
    if (!imu_read_burst(ICM_REG_TEMP_DATA1, buf, 14u)) {
        imu_read_failures++;
        return false;
    }

    /* Big-endian, signed 16-bit: H byte at low offset. */
    out->temp    = (int16_t)((buf[0]  << 8) | buf[1]);
    out->accel_x = (int16_t)((buf[2]  << 8) | buf[3]);
    out->accel_y = (int16_t)((buf[4]  << 8) | buf[5]);
    out->accel_z = (int16_t)((buf[6]  << 8) | buf[7]);
    out->gyro_x  = (int16_t)((buf[8]  << 8) | buf[9]);
    out->gyro_y  = (int16_t)((buf[10] << 8) | buf[11]);
    out->gyro_z  = (int16_t)((buf[12] << 8) | buf[13]);

    imu_latest_sample = *out;
    imu_sample_count++;
    return true;
}
