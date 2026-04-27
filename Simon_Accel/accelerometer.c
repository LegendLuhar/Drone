#include "accelerometer.h"
#include "delay.h"
#include <ti/devices/msp/msp.h>

#define ACCEL_WHO_AM_I_REG 0x0F
#define ACCEL_WHO_AM_I_VAL 0x41
#define ACCEL_CTRL1_REG 0x20
#define ACCEL_CTRL4_REG 0x23
#define ACCEL_OUT_X_L_REG 0x28

#define ACCEL_READ_CMD 0x80
#define ACCEL_AUTO_INC_CMD 0x40

#define ACCEL_CTRL1_100HZ 0xBF
#define ACCEL_CTRL4_4G_INC 0x26
#define ACCEL_SPI_PRESCALE 127

#define ACCEL_CS_PIN 6

// bonus: accelerometer mode 
// driver status shared with state machine debug display
static bool g_accel_available;
static bool g_manual_cs_mode;
static uint8_t g_accel_debug_status;
static uint8_t g_last_who_am_i;

// bonus: accelerometer mode (GPIO and SPI low-level)
static void InitializeGPIOB(void)
{
    GPIOB->GPRCM.RSTCTL = (GPIO_RSTCTL_KEY_UNLOCK_W |
                           GPIO_RSTCTL_RESETSTKYCLR_CLR |
                           GPIO_RSTCTL_RESETASSERT_ASSERT);
    GPIOB->GPRCM.PWREN = (GPIO_PWREN_KEY_UNLOCK_W |
                          GPIO_PWREN_ENABLE_ENABLE);
    delay_cycles(POWER_STARTUP_DELAY);
    
    GPIOB->DOE31_0 |= (1 << ACCEL_CS_PIN);
    GPIOB->DOUT31_0 |= (1 << ACCEL_CS_PIN);
}

static void AccelCSAssert(void)
{
    GPIOB->DOUT31_0 &= ~(1 << ACCEL_CS_PIN);
    delay_cycles(10);
}

static void AccelCSDeassert(void)
{
    delay_cycles(10);
    GPIOB->DOUT31_0 |= (1 << ACCEL_CS_PIN);
}

static void ConfigureSPIMode(uint32_t spo, uint32_t sph)
{
    // keep same data framing in both cs modes
    SPI1->CTL1 &= ~(SPI_CTL1_ENABLE_ENABLE);
    if (g_manual_cs_mode) {
        SPI1->CTL0 = spo | sph | SPI_CTL0_FRF_MOTOROLA_4WIRE |
                     SPI_CTL0_DSS_DSS_8;
    } else {
        SPI1->CTL0 = spo | sph | SPI_CTL0_FRF_MOTOROLA_4WIRE |
                     SPI_CTL0_DSS_DSS_8 | SPI_CTL0_CSSEL_CSSEL_0;
    }
    SPI1->CTL1 |= SPI_CTL1_ENABLE_ENABLE;
}

static void ConfigureHardwareCSMode(void)
{
    // spi drives cs pin directly
    g_manual_cs_mode = false;
    IOMUX->SECCFG.PINCM[(IOMUX_PINCM23)] =
        IOMUX_PINCM_PC_CONNECTED | IOMUX_PINCM23_PF_SPI1_CS0;
}

static void ConfigureManualCSMode(void)
{
    // gpio drives cs pin directly
    g_manual_cs_mode = true;
    IOMUX->SECCFG.PINCM[(IOMUX_PINCM23)] =
        IOMUX_PINCM_PC_CONNECTED | IOMUX_PINCM23_PF_GPIOB_DIO06;
    GPIOB->DOE31_0 |= (1 << ACCEL_CS_PIN);
    GPIOB->DOUT31_0 |= (1 << ACCEL_CS_PIN);
}

static void ConfigureAccelPins(void)
{
    IOMUX->SECCFG.PINCM[(IOMUX_PINCM25)] =
        IOMUX_PINCM_PC_CONNECTED | IOMUX_PINCM25_PF_SPI1_PICO;
    IOMUX->SECCFG.PINCM[(IOMUX_PINCM31)] =
        IOMUX_PINCM_PC_CONNECTED | IOMUX_PINCM_INENA_ENABLE |
        IOMUX_PINCM31_PF_SPI1_POCI;

    IOMUX->SECCFG.PINCM[(IOMUX_PINCM33)] =
        IOMUX_PINCM_PC_CONNECTED | IOMUX_PINCM33_PF_SPI1_SCLK;
}

static void ClearRXFIFO(void)
{
    // clear stale bytes before a new transfer
    while ((SPI1->STAT & SPI_STAT_RFE_MASK) == SPI_STAT_RFE_NOT_EMPTY) {
        (void)SPI1->RXDATA;
    }
}

static bool TransferTransaction(
    const uint8_t *tx_buffer,
    uint8_t *rx_buffer,
    uint8_t length)
{
    if (length == 0) {
        return false;
    }

    ClearRXFIFO();

    for (uint8_t i = 0; i < length; i++) {
        while ((SPI1->STAT & SPI_STAT_TNF_MASK) == SPI_STAT_TNF_FULL) {
        }
        SPI1->TXDATA = tx_buffer[i];
    }

    for (uint8_t i = 0; i < length; i++) {
        while ((SPI1->STAT & SPI_STAT_RFE_MASK) == SPI_STAT_RFE_EMPTY) {
        }

        if (rx_buffer != 0) {
            rx_buffer[i] = (uint8_t)SPI1->RXDATA;
        } else {
            (void)SPI1->RXDATA;
        }
    }

    while ((SPI1->STAT & SPI_STAT_BUSY_MASK) == SPI_STAT_BUSY_ACTIVE) {
    }

    return true;
}

static bool WriteRegister(uint8_t reg, uint8_t value)
{
    uint8_t tx_buffer[2];
    bool result;

    tx_buffer[0] = reg;
    tx_buffer[1] = value;
    if (g_manual_cs_mode) {
        AccelCSAssert();
    }
    result = TransferTransaction(tx_buffer, 0, 2);
    if (g_manual_cs_mode) {
        AccelCSDeassert();
    }
    return result;
}

// ai assisted
static bool ReadRegisters(uint8_t reg, uint8_t *buffer, uint8_t len)
{
    uint8_t tx_buffer[7] = {0};
    uint8_t rx_buffer[7] = {0};
    bool result;

    if ((buffer == 0) || (len == 0) || (len > 6)) {
        return false;
    }

    tx_buffer[0] = (uint8_t)(reg | ACCEL_READ_CMD);
    if (len > 1) {
        // lis2hh12 burst read uses auto-inc bit
        tx_buffer[0] |= ACCEL_AUTO_INC_CMD;
    }
    if (g_manual_cs_mode) {
        AccelCSAssert();
    }
    result = TransferTransaction(tx_buffer, rx_buffer, (uint8_t)(len + 1));
    if (g_manual_cs_mode) {
        AccelCSDeassert();
    }

    for (uint8_t i = 0; i < len; i++) {
        buffer[i] = rx_buffer[i + 1];
    }

    return result;
}

static bool ReadRegister(uint8_t reg, uint8_t *value)
{
    return ReadRegisters(reg, value, 1);
}

static bool CheckWhoAmI(void)
{
    uint8_t who_am_i = 0;

    for (int attempt = 0; attempt < 4; attempt++) {
        // retry handles startup timing + spi mode settle
        ReadRegisters(ACCEL_WHO_AM_I_REG, &who_am_i, 1);
        g_last_who_am_i = who_am_i;
        if (who_am_i == ACCEL_WHO_AM_I_VAL) {
            return true;
        }
        delay_cycles(64);
    }

    return false;
}

static bool TrySPIWhoAmI(uint32_t spo, uint32_t sph)
{
    ConfigureSPIMode(spo, sph);
    return CheckWhoAmI();
}

static bool TryWhoAmIWithCurrentCSMode(void)
{
    // try all 4 cpol/cpha combos once
    return TrySPIWhoAmI(SPI_CTL0_SPO_HIGH, SPI_CTL0_SPH_SECOND) ||
           TrySPIWhoAmI(SPI_CTL0_SPO_LOW, SPI_CTL0_SPH_FIRST) ||
           TrySPIWhoAmI(SPI_CTL0_SPO_LOW, SPI_CTL0_SPH_SECOND) ||
           TrySPIWhoAmI(SPI_CTL0_SPO_HIGH, SPI_CTL0_SPH_FIRST);
}

static bool TryDetectWithCurrentPins(void)
{
    // manual cs first, then hw cs fallback
    ConfigureManualCSMode();
    if (TryWhoAmIWithCurrentCSMode()) {
        return true;
    }

    ConfigureHardwareCSMode();
    if (TryWhoAmIWithCurrentCSMode()) {
        return true;
    }

    return false;
}

// ai assisted 
void InitializeAccelerometer(void)
{
    // bonus: accelerometer mode (full sensor configure path)
    uint8_t ctrl1_read = 0;
    uint8_t ctrl4_read = 0;

    g_accel_available = false;
    g_manual_cs_mode = false;
    g_accel_debug_status = ACCEL_DEBUG_INIT_START;
    g_last_who_am_i = 0;

    InitializeGPIOB();

    ConfigureAccelPins();

    SPI1->GPRCM.RSTCTL = (SPI_RSTCTL_KEY_UNLOCK_W |
                          SPI_RSTCTL_RESETSTKYCLR_CLR |
                          SPI_RSTCTL_RESETASSERT_ASSERT);
    SPI1->GPRCM.PWREN = (SPI_PWREN_KEY_UNLOCK_W |
                         SPI_PWREN_ENABLE_ENABLE);
    delay_cycles(POWER_STARTUP_DELAY);

    SPI1->CLKSEL = (uint32_t)SPI_CLKSEL_SYSCLK_SEL_ENABLE;
    SPI1->CLKDIV = (uint32_t)SPI_CLKDIV_RATIO_DIV_BY_1;
    SPI1->CTL1 = SPI_CTL1_CP_ENABLE |
                 SPI_CTL1_PREN_DISABLE |
                 SPI_CTL1_PTEN_DISABLE |
                 SPI_CTL1_MSB_ENABLE;
    SPI1->CLKCTL = ACCEL_SPI_PRESCALE;
    
    delay_cycles(320000);

    if (!TryDetectWithCurrentPins()) {
        g_accel_debug_status = ACCEL_DEBUG_WHOAMI_FAIL;
        return;
    }

    if (!WriteRegister(ACCEL_CTRL1_REG, ACCEL_CTRL1_100HZ)) {
        g_accel_debug_status = ACCEL_DEBUG_CONFIG_WRITE_FAIL;
        return;
    }
    delay_cycles(1000);

    if (!WriteRegister(ACCEL_CTRL4_REG, ACCEL_CTRL4_4G_INC)) {
        g_accel_debug_status = ACCEL_DEBUG_CONFIG_WRITE_FAIL;
        return;
    }
    delay_cycles(1000);

    if (!ReadRegisters(ACCEL_CTRL1_REG, &ctrl1_read, 1) ||
        !ReadRegisters(ACCEL_CTRL4_REG, &ctrl4_read, 1)) {
        g_accel_debug_status = ACCEL_DEBUG_CONFIG_READ_FAIL;
        return;
    }
    (void)ctrl1_read;
    (void)ctrl4_read;

    g_accel_available = true;
    g_accel_debug_status = ACCEL_DEBUG_OK;
}

bool AccelerometerAvailable(void)
{
    return g_accel_available;
}

// ai assisted 
bool AccelerometerReadSample(accelerometer_sample_t *sample)
{
    // bonus: accelerometer mode (per-axis reads for stable samples)
    uint8_t x_l;
    uint8_t x_h;
    uint8_t y_l;
    uint8_t y_h;
    uint8_t z_l;
    uint8_t z_h;

    if ((!g_accel_available) || (sample == 0)) {
        return false;
    }

    // read each axis register directly for stable data
    if (!ReadRegister(ACCEL_OUT_X_L_REG, &x_l) ||
        !ReadRegister((uint8_t)(ACCEL_OUT_X_L_REG + 1), &x_h) ||
        !ReadRegister((uint8_t)(ACCEL_OUT_X_L_REG + 2), &y_l) ||
        !ReadRegister((uint8_t)(ACCEL_OUT_X_L_REG + 3), &y_h) ||
        !ReadRegister((uint8_t)(ACCEL_OUT_X_L_REG + 4), &z_l) ||
        !ReadRegister((uint8_t)(ACCEL_OUT_X_L_REG + 5), &z_h)) {
        g_accel_debug_status = ACCEL_DEBUG_SAMPLE_READ_FAIL;
        return false;
    }

    sample->x = (int16_t)(((int16_t)x_h << 8) | x_l);
    sample->y = (int16_t)(((int16_t)y_h << 8) | y_l);
    sample->z = (int16_t)(((int16_t)z_h << 8) | z_l);

    return true;
}

uint8_t AccelerometerGetDebugStatus(void)
{
    return g_accel_debug_status;
}

uint8_t AccelerometerGetLastWhoAmI(void)
{
    return g_last_who_am_i;
}
