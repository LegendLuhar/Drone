/*
 * dshot.c — DShot300 on TIMA1 (M1+M2), TIMG8 (M3), TIMA0 (M4) with DMA.
 *
 * BIT TIMING @ MCLK = 16 MHz:
 *   DShot300 nominal bit period 3.333 us. We divide TIMCLK by 1, so 1 tick
 *   = 62.5 ns. LOAD = 53 - 1 gives a 53-tick = 3.3125 us period (~0.6 %
 *   short of nominal — well inside the +/-25 % tolerance the protocol
 *   spec allows).
 *     - "1" bit  high time  ~75 % -> 40 ticks high -> CC = 53 - 40 = 13
 *     - "0" bit  high time  ~37 % -> 20 ticks high -> CC = 53 - 20 = 33
 *     - quiet                            ->        CC = 53 (high time = 0)
 *
 * COUNTER MODE: down-counter. At LOAD the CCP goes high (LACT = HIGH).
 * When the counter passes CC the CCP goes low (CDACT = LOW). The CC value
 * is therefore "low duration in ticks" measured from end of period.
 *
 * FRAME LAYOUT: 17 entries per motor, [0..15] = bit values MSB-first,
 * [16] = quiet (CC = LOAD). The trailing quiet entry ensures the line
 * latches low at end-of-frame instead of holding the last bit's CC value.
 *
 * EVENT FABRIC ROUTING:
 *   TIMA1 publishes Z (zero) event on chanID 1 -> DMA_CH0 (M1) + DMA_CH1 (M2)
 *   TIMG8 publishes Z (zero) event on chanID 2 -> DMA_CH2 (M3)
 *   TIMA0 publishes Z (zero) event on chanID 3 -> DMA_CH3 (M4)
 *
 * Each DMA channel does a 17-halfword block transfer per frame, sourced
 * from its motor's frame buffer, sinking into the timer's CC_01[ch]
 * register. The channel is re-armed by the CPU at the start of every
 * frame (dshot_transmit()).
 *
 * START SYNC: timers are enabled once at end of dshot_init() and run
 * forever. Between frames the CC value last written is the trailing
 * "quiet" entry, so all CCPs hold low. DShot doesn't require phase-locked
 * channels — bit timing on each pin is what the ESC sees.
 */

#include <ti/devices/msp/msp.h>
#include <stdint.h>
#include <stdbool.h>
#include "dshot.h"
#include "delay.h"

/* ---- Bit-timing constants for 16 MHz MCLK / DShot300 ---- */
#define DSHOT_LOAD              (53u - 1u)   /* down-counter LOAD value */
#define DSHOT_CC_BIT1           (53u - 40u)  /* CC value for "1" bit */
#define DSHOT_CC_BIT0           (53u - 20u)  /* CC value for "0" bit */
#define DSHOT_CC_QUIET          (53u)        /* CC = LOAD+1 keeps line low */

/* ---- Frame buffers (one per motor, 17 halfwords) ---- */
#define DSHOT_FRAME_LEN         (17u)
static volatile uint16_t s_frame[DSHOT_NUM_MOTORS][DSHOT_FRAME_LEN];

/* ---- IOMUX pin assignments ---- */
#define PINCM_M1                (43u - 1u)   /* PB17 */
#define PINCM_M2                (44u - 1u)   /* PB18 */
#define PINCM_M3                (45u - 1u)   /* PB19 */
#define PINCM_M4                (48u - 1u)   /* PB20 */

#define PF_M1_TIMA1_CCP0        (5u)
#define PF_M2_TIMA1_CCP1        (5u)
#define PF_M3_TIMG8_CCP1        (4u)
#define PF_M4_TIMA0_CCP1        (7u)

/* ---- Event-fabric channel IDs (1..15) ---- */
#define EVT_CH_TIMA1            (1u)
#define EVT_CH_TIMG8            (2u)
#define EVT_CH_TIMA0            (3u)

/* ---- DMA channel assignments ---- */
#define DMA_M1                  (0u)   /* TIMA1 -> CC_0  (Motor 1) */
#define DMA_M2                  (1u)   /* TIMA1 -> CC_1  (Motor 2) */
#define DMA_M3                  (2u)   /* TIMG8 -> CC_1  (Motor 3) */
#define DMA_M4                  (3u)   /* TIMA0 -> CC_1  (Motor 4) */

volatile uint32_t dshot_frames_started = 0;
volatile uint32_t dshot_frames_skipped = 0;

/* ---- Helpers ------------------------------------------------------- */

static uint16_t dshot_pack_frame(uint16_t value, bool telem_request)
{
    /* DShot frame: [11 bits payload][1 bit telem][4 bits CRC]
     * CRC = (d ^ (d>>4) ^ (d>>8)) & 0x0F where d = (payload<<1) | telem. */
    if (value > DSHOT_THROTTLE_MAX) {
        value = DSHOT_THROTTLE_MAX;
    }
    uint16_t d = (uint16_t)((value << 1) | (telem_request ? 1u : 0u));
    uint16_t crc = (uint16_t)((d ^ (d >> 4) ^ (d >> 8)) & 0x0Fu);
    return (uint16_t)((d << 4) | crc);
}

static void dshot_encode_into_buffer(dshot_motor_t m, uint16_t frame16)
{
    /* MSB-first: bit 15 first, bit 0 last. */
    for (uint32_t i = 0; i < 16u; ++i) {
        bool bit = (frame16 & (1u << (15u - i))) != 0;
        s_frame[m][i] = (uint16_t)(bit ? DSHOT_CC_BIT1 : DSHOT_CC_BIT0);
    }
    s_frame[m][16] = (uint16_t)DSHOT_CC_QUIET;
}

/* ---- Bring-up steps ------------------------------------------------ */

static void dshot_init_gpio(void)
{
    /* Power up GPIOB (motors are all on port B). */
    if ((GPIOB->GPRCM.STAT & GPIO_STAT_RESETSTKY_MASK) != 0u) {
        GPIOB->GPRCM.RSTCTL = (GPIO_RSTCTL_KEY_UNLOCK_W
                               | GPIO_RSTCTL_RESETSTKYCLR_CLR
                               | GPIO_RSTCTL_RESETASSERT_ASSERT);
        GPIOB->GPRCM.PWREN  = (GPIO_PWREN_KEY_UNLOCK_W
                               | GPIO_PWREN_ENABLE_ENABLE);
        delay_cycles(POWER_STARTUP_DELAY);
    }

    IOMUX->SECCFG.PINCM[PINCM_M1] = IOMUX_PINCM_PC_CONNECTED | PF_M1_TIMA1_CCP0;
    IOMUX->SECCFG.PINCM[PINCM_M2] = IOMUX_PINCM_PC_CONNECTED | PF_M2_TIMA1_CCP1;
    IOMUX->SECCFG.PINCM[PINCM_M3] = IOMUX_PINCM_PC_CONNECTED | PF_M3_TIMG8_CCP1;
    IOMUX->SECCFG.PINCM[PINCM_M4] = IOMUX_PINCM_PC_CONNECTED | PF_M4_TIMA0_CCP1;
}

static void dshot_init_one_timer(GPTIMER_Regs *t,
                                 uint32_t enable_ch0,
                                 uint32_t enable_ch1,
                                 uint32_t pub_chan_id)
{
    /* Standard GPRCM dance. */
    t->GPRCM.RSTCTL = (GPTIMER_RSTCTL_KEY_UNLOCK_W
                       | GPTIMER_RSTCTL_RESETSTKYCLR_CLR
                       | GPTIMER_RSTCTL_RESETASSERT_ASSERT);
    t->GPRCM.PWREN  = (GPTIMER_PWREN_KEY_UNLOCK_W
                       | GPTIMER_PWREN_ENABLE_ENABLE);
    delay_cycles(POWER_STARTUP_DELAY);

    /* BUSCLK = MCLK = 16 MHz. No prescale: 1 tick = 62.5 ns. */
    t->CLKSEL                 = GPTIMER_CLKSEL_BUSCLK_SEL_ENABLE;
    t->CLKDIV                 = 0u;
    t->COMMONREGS.CCLKCTL     = GPTIMER_CCLKCTL_CLKEN_ENABLED;

    /* Down-counter, repeating, counter loaded with LOAD on enable. */
    t->COUNTERREGS.LOAD       = DSHOT_LOAD;
    t->COUNTERREGS.CTRCTL     = (GPTIMER_CTRCTL_CM_DOWN
                                 | GPTIMER_CTRCTL_CVAE_LDVAL
                                 | GPTIMER_CTRCTL_REPEAT_REPEAT_1);

    /* Per-channel PWM config. enable_chN selects which CC channels we
     * configure for PWM output (Motor1 uses CC_0 on TIMA1, all others use
     * CC_1). The unused channel on TIMA1 stays dormant. */
    if (enable_ch0) {
        t->COUNTERREGS.CC_01[0]    = DSHOT_CC_QUIET;
        t->COUNTERREGS.CCCTL_01[0] = (GPTIMER_CCCTL_01_COC_COMPARE
                                      | GPTIMER_CCCTL_01_CCUPD_ZERO_EVT);
        t->COUNTERREGS.CCACT_01[0] = (GPTIMER_CCACT_01_LACT_CCP_HIGH
                                      | GPTIMER_CCACT_01_CDACT_CCP_LOW);
        t->COUNTERREGS.OCTL_01[0]  = (GPTIMER_OCTL_01_CCPO_FUNCVAL
                                      | GPTIMER_OCTL_01_CCPIV_LOW);
    }
    if (enable_ch1) {
        t->COUNTERREGS.CC_01[1]    = DSHOT_CC_QUIET;
        t->COUNTERREGS.CCCTL_01[1] = (GPTIMER_CCCTL_01_COC_COMPARE
                                      | GPTIMER_CCCTL_01_CCUPD_ZERO_EVT);
        t->COUNTERREGS.CCACT_01[1] = (GPTIMER_CCACT_01_LACT_CCP_HIGH
                                      | GPTIMER_CCACT_01_CDACT_CCP_LOW);
        t->COUNTERREGS.OCTL_01[1]  = (GPTIMER_OCTL_01_CCPO_FUNCVAL
                                      | GPTIMER_OCTL_01_CCPIV_LOW);
    }

    /* Publish the zero-event onto the chosen event-fabric channel so the
     * DMA channels can subscribe. */
    t->GEN_EVENT0.IMASK = GPTIMER_GEN_EVENT0_IMASK_Z_SET;
    t->FPUB_0           = (pub_chan_id & GPTIMER_FPUB_0_CHANID_MASK);
}

static void dshot_init_dma_channel(uint32_t ch,
                                   const volatile uint16_t *src,
                                   volatile uint32_t *dst,
                                   uint32_t event_chan_id)
{
    /* DMA module has no GPRCM — it is always-on. */
    DMA->DMACHAN[ch].DMASA  = (uint32_t)src;
    DMA->DMACHAN[ch].DMADA  = (uint32_t)dst;
    DMA->DMACHAN[ch].DMASZ  = 0;  /* set per-frame in dshot_transmit() */

    /* Block transfer: one trigger pulls the entire 17-halfword frame.
     * Source: 16-bit halfword, increment by +1 element per transfer.
     * Destination: 32-bit word write to CC register, address fixed. */
    DMA->DMACHAN[ch].DMACTL =
          DMA_DMACTL_DMATM_BLOCK
        | DMA_DMACTL_DMASRCWDTH_HALF
        | DMA_DMACTL_DMADSTWDTH_WORD
        | DMA_DMACTL_DMASRCINCR_INCREMENT
        | DMA_DMACTL_DMADSTINCR_UNCHANGED;

    /* Subscribe this channel to the timer's published event channel.
     * DMATINT_EXTERNAL = 0 selects the event-fabric path. */
    DMA->DMATRIG[ch].DMATCTL = (event_chan_id & DMA_DMATCTL_DMATSEL_MASK)
                             | DMA_DMATCTL_DMATINT_EXTERNAL;
}

void dshot_init(void)
{
    /* Pre-stage all buffers as MOTOR_STOP so the very first transmit is
     * benign even if the caller forgets to set values. */
    dshot_set_all_motor_stop();

    dshot_init_gpio();

    /* Order: configure timers first (publishers), then DMA (subscribers),
     * then enable timer counters. */
    dshot_init_one_timer(TIMA1, /*ch0*/1, /*ch1*/1, EVT_CH_TIMA1);
    dshot_init_one_timer(TIMG8, /*ch0*/0, /*ch1*/1, EVT_CH_TIMG8);
    dshot_init_one_timer(TIMA0, /*ch0*/0, /*ch1*/1, EVT_CH_TIMA0);

    dshot_init_dma_channel(DMA_M1, s_frame[DSHOT_MOTOR_1],
                           &TIMA1->COUNTERREGS.CC_01[0], EVT_CH_TIMA1);
    dshot_init_dma_channel(DMA_M2, s_frame[DSHOT_MOTOR_2],
                           &TIMA1->COUNTERREGS.CC_01[1], EVT_CH_TIMA1);
    dshot_init_dma_channel(DMA_M3, s_frame[DSHOT_MOTOR_3],
                           &TIMG8->COUNTERREGS.CC_01[1], EVT_CH_TIMG8);
    dshot_init_dma_channel(DMA_M4, s_frame[DSHOT_MOTOR_4],
                           &TIMA0->COUNTERREGS.CC_01[1], EVT_CH_TIMA0);

    /* Enable counters. CCs already pre-loaded to QUIET so outputs idle low. */
    TIMA1->COUNTERREGS.CTRCTL |= GPTIMER_CTRCTL_EN_ENABLED;
    TIMG8->COUNTERREGS.CTRCTL |= GPTIMER_CTRCTL_EN_ENABLED;
    TIMA0->COUNTERREGS.CTRCTL |= GPTIMER_CTRCTL_EN_ENABLED;
}

void dshot_set_value(dshot_motor_t m, uint16_t value, bool telem_request)
{
    uint16_t frame = dshot_pack_frame(value, telem_request);
    dshot_encode_into_buffer(m, frame);
}

void dshot_set_all_motor_stop(void)
{
    /* MOTOR_STOP frame: payload 0, telem 0, CRC 0 -> all 16 bits are 0. */
    for (uint32_t m = 0; m < DSHOT_NUM_MOTORS; ++m) {
        for (uint32_t i = 0; i < 16u; ++i) {
            s_frame[m][i] = (uint16_t)DSHOT_CC_BIT0;
        }
        s_frame[m][16] = (uint16_t)DSHOT_CC_QUIET;
    }
}

bool dshot_transmit(void)
{
    /* If any channel still has DMASZ != 0 the previous frame is mid-flight.
     * Skip rather than corrupt timing. */
    if (DMA->DMACHAN[DMA_M1].DMASZ != 0
     || DMA->DMACHAN[DMA_M2].DMASZ != 0
     || DMA->DMACHAN[DMA_M3].DMASZ != 0
     || DMA->DMACHAN[DMA_M4].DMASZ != 0) {
        dshot_frames_skipped++;
        return false;
    }

    /* Re-load source addresses (DMASA auto-advances during a block xfer
     * and ends pointing past the buffer — must be reset every frame). */
    DMA->DMACHAN[DMA_M1].DMASA = (uint32_t)s_frame[DSHOT_MOTOR_1];
    DMA->DMACHAN[DMA_M2].DMASA = (uint32_t)s_frame[DSHOT_MOTOR_2];
    DMA->DMACHAN[DMA_M3].DMASA = (uint32_t)s_frame[DSHOT_MOTOR_3];
    DMA->DMACHAN[DMA_M4].DMASA = (uint32_t)s_frame[DSHOT_MOTOR_4];

    DMA->DMACHAN[DMA_M1].DMASZ = DSHOT_FRAME_LEN;
    DMA->DMACHAN[DMA_M2].DMASZ = DSHOT_FRAME_LEN;
    DMA->DMACHAN[DMA_M3].DMASZ = DSHOT_FRAME_LEN;
    DMA->DMACHAN[DMA_M4].DMASZ = DSHOT_FRAME_LEN;

    DMA->DMACHAN[DMA_M1].DMACTL |= DMA_DMACTL_DMAEN_ENABLE;
    DMA->DMACHAN[DMA_M2].DMACTL |= DMA_DMACTL_DMAEN_ENABLE;
    DMA->DMACHAN[DMA_M3].DMACTL |= DMA_DMACTL_DMAEN_ENABLE;
    DMA->DMACHAN[DMA_M4].DMACTL |= DMA_DMACTL_DMAEN_ENABLE;

    dshot_frames_started++;
    return true;
}
