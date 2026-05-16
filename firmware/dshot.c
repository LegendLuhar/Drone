/*
 * dshot.c — DShot300 on TIMA1 (M1+M2), TIMG8 (M3), TIMA0 (M4) with DMA.
 *
 * BIT TIMING @ MCLK = 16 MHz:
 *   DShot300 nominal bit period 3.333 us. TIMCLK = MCLK / 1, so 1 tick =
 *   62.5 ns. LOAD = 52 (53 ticks per period = 3.3125 us, ~0.6 % short of
 *   nominal — well inside DShot's +/-25 % tolerance).
 *     - "1" bit  ~75 % HIGH -> 40 ticks HIGH -> CC = 13
 *     - "0" bit  ~37 % HIGH -> 20 ticks HIGH -> CC = 33
 *     - quiet    0 % HIGH   ->                  CC = 53 (CDACT never fires)
 *
 * COUNTER MODE: down-counter, LACT=CCP_HIGH at load, CDACT=CCP_LOW at
 * compare-match. CC value is "ticks of LOW measured from end of period".
 *
 * FRAME LAYOUT: 17 entries per motor, [0..15] = bit values MSB-first,
 * [16] = quiet. The trailing quiet entry holds the timer's CCP HIGH after
 * the 16 data bits (because CC > LOAD => no CDACT) — between frames we
 * rely on the GPIO-override below to actually pull the pin LOW, since the
 * timer alone can't park a CCP low while LACT_HIGH still fires every
 * period.
 *
 * INTER-FRAME IDLE: timers run forever; the motor pins are routed through
 * IOMUX PINCM and we flip each PINCM between the timer alt-function
 * (during the ~57 us frame) and PF=1 GPIO (between frames). The GPIO is
 * pre-configured as output-LOW, so flipping PINCM to GPIO immediately
 * pulls the line LOW. That gives the ESC the clean ~940 us LOW idle gap
 * it needs to align to bit 15 of the next frame.
 *
 * EVENT FABRIC ROUTING:
 *   TIMA1 publishes its Z event on event-fabric channel 1.
 *   DMA's FSUB_0 subscribes to channel 1.
 *   All 4 DMA channels trigger off FSUB_0 (DMATSEL = DMA_GENERIC_SUB0_TRIG = 1).
 *   TIMG8 and TIMA0 don't publish — they're slaved by running on the
 *   same MCLK with the same LOAD; few-cycle drift is harmless because
 *   the DMA writes complete well before any timer needs the new CC.
 *
 * Each DMA channel does a 17-halfword SINGLE-mode transfer per frame
 * (one halfword per timer Z event). The channel is re-armed by the CPU
 * at the start of every frame in dshot_transmit().
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
#define DSHOT_CC_QUIET          (53u)        /* CC > LOAD: CDACT never fires.
                                              * Pin would hold HIGH if timer
                                              * drove it — we use GPIO override
                                              * (PF=1) between frames to actually
                                              * idle the line LOW. */

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

#define PF_GPIO                 (1u)   /* PF=1 is GPIO on all four PINCMs */

/* GPIOB mask for the four motor DIOs. */
#define MOTOR_PIN_MASK          ((1u << 17) | (1u << 18) | (1u << 19) | (1u << 20))

/* ---- Event-fabric channel IDs (1..15) ---- *
 *
 * MSPM0G3507 only exposes two external DMA trigger lines via the event
 * fabric: DMA_GENERIC_SUB0_TRIG (=1, DMA's own FSUB_0) and
 * DMA_GENERIC_SUB1_TRIG (=2, DMA's own FSUB_1). The timers do not have
 * dedicated DMA-trigger entries — everything has to go through those two
 * subscriber ports.
 *
 * We have three timers (TIMA1, TIMG8, TIMA0) but only two ports, so we
 * trigger all four DMA channels from a single timer (TIMA1). The other
 * two timers run with the same MCLK and LOAD value; they'll drift only
 * by the few cycles between their consecutive enable writes, which is
 * negligible compared to the 53-tick DShot bit period. The DMA writes
 * to each timer's CC register complete within ~10 cycles of TIMA1's
 * zero event, well before any timer needs the new value. */
#define EVT_FABRIC_CH           (1u)            /* TIMA1 publishes here, DMA subscribes here */
#define DMA_TRIG_SEL_SUB0       (1u)            /* DMA_GENERIC_SUB0_TRIG from mspm0g350x.h */

/* ---- DMA channel assignments ---- */
#define DMA_M1                  (0u)   /* TIMA1 -> CC_0  (Motor 1) */
#define DMA_M2                  (1u)   /* TIMA1 -> CC_1  (Motor 2) */
#define DMA_M3                  (2u)   /* TIMG8 -> CC_1  (Motor 3) */
#define DMA_M4                  (3u)   /* TIMA0 -> CC_1  (Motor 4) */

volatile uint32_t dshot_frames_started = 0;
volatile uint32_t dshot_frames_skipped = 0;

/* SWD-readable diagnostics. Pause the target via the debugger and inspect
 * these to figure out where DShot is failing without a logic analyzer.
 *
 *   dbg_dmasz_after_tx[]  : DMASZ for each motor sampled at the end of
 *                           dshot_transmit() *before* the next frame's
 *                           re-arm. Healthy value = 0 (DMA drained 17
 *                           halfwords). Stays at 17 = DMA never fired;
 *                           the event-fabric route is broken.
 *   dbg_dmasa_after_tx[]  : DMASA for each motor at the same moment.
 *                           Healthy value = &s_frame[m][17] (one past end).
 *                           Equal to &s_frame[m][0] = DMA never advanced.
 *   dbg_z_seen_a1/g8/a0   : Latched RIS.Z for each timer, captured once
 *                           per second from main loop. Non-zero = the
 *                           timer's Z event has fired since last read,
 *                           confirming the timer is actually counting.
 *   dbg_active_cc_*       : Snapshot of each timer's CC_01 register so
 *                           you can see what value the comparator is
 *                           using. Should equal the last frame entry
 *                           (DSHOT_CC_QUIET = 53) between frames. */
volatile uint32_t dshot_dbg_dmasz_after_tx[DSHOT_NUM_MOTORS];
volatile uint32_t dshot_dbg_dmasa_after_tx[DSHOT_NUM_MOTORS];
volatile uint32_t dshot_dbg_active_cc_m1;
volatile uint32_t dshot_dbg_active_cc_m2;
volatile uint32_t dshot_dbg_active_cc_m3;
volatile uint32_t dshot_dbg_active_cc_m4;

/* Deeper diagnostics for routing issues. Sample every transmit. */
volatile uint32_t dshot_dbg_tima1_ctr;       /* counter value; should change between calls */
volatile uint32_t dshot_dbg_timg8_ctr;
volatile uint32_t dshot_dbg_tima0_ctr;
volatile uint32_t dshot_dbg_tima1_ctrctl;    /* EN bit at bit 0 — should read 1 if timer running */
volatile uint32_t dshot_dbg_tima1_rise_z;    /* RIS.Z bit — should be 1 if Z fired since last read/clear */
volatile uint32_t dshot_dbg_tima1_fpub0;     /* should read back the configured channel id (1) */
volatile uint32_t dshot_dbg_tima1_gen_imask; /* GEN_EVENT0.IMASK — should be 1 */
volatile uint32_t dshot_dbg_dmach0_ctl;      /* DMACTL of motor1 channel — bit1 (DMAEN) should be 1 after first frame */
volatile uint32_t dshot_dbg_dmach0_tctl;    /* DMATCTL of motor1 channel — should be 1 (TSEL=1, TINT=external) */
volatile uint32_t dshot_dbg_dma_fsub0;       /* DMA->FSUB_0 — should be EVT_FABRIC_CH (1). 0 = nothing subscribed */

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

    /* Configure the four motor DIOs as GPIO outputs, driving LOW. This is
     * the "parked" state between DShot frames. When dshot_transmit() runs
     * it flips each PINCM back to the timer alt-function for the ~57 µs
     * frame, then dshot_park() flips them back here so the ESC sees a
     * clean LOW idle gap and can lock onto the next frame's rising edge. */
    GPIOB->DOUTCLR31_0 = MOTOR_PIN_MASK;     /* DOUT = 0 (LOW) */
    GPIOB->DOESET31_0  = MOTOR_PIN_MASK;     /* DOE = 1 (output) */

    /* Park the pins LOW immediately so they don't sit in whatever state
     * the timer left them. */
    IOMUX->SECCFG.PINCM[PINCM_M1] = IOMUX_PINCM_PC_CONNECTED | PF_GPIO;
    IOMUX->SECCFG.PINCM[PINCM_M2] = IOMUX_PINCM_PC_CONNECTED | PF_GPIO;
    IOMUX->SECCFG.PINCM[PINCM_M3] = IOMUX_PINCM_PC_CONNECTED | PF_GPIO;
    IOMUX->SECCFG.PINCM[PINCM_M4] = IOMUX_PINCM_PC_CONNECTED | PF_GPIO;
}

static void dshot_pins_to_timer(void)
{
    IOMUX->SECCFG.PINCM[PINCM_M1] = IOMUX_PINCM_PC_CONNECTED | PF_M1_TIMA1_CCP0;
    IOMUX->SECCFG.PINCM[PINCM_M2] = IOMUX_PINCM_PC_CONNECTED | PF_M2_TIMA1_CCP1;
    IOMUX->SECCFG.PINCM[PINCM_M3] = IOMUX_PINCM_PC_CONNECTED | PF_M3_TIMG8_CCP1;
    IOMUX->SECCFG.PINCM[PINCM_M4] = IOMUX_PINCM_PC_CONNECTED | PF_M4_TIMA0_CCP1;
}

static void dshot_pins_to_gpio_low(void)
{
    /* GPIO DOUT was already cleared in dshot_init_gpio and never written
     * since, so the pin reads LOW the instant PINCM PF flips to GPIO. */
    IOMUX->SECCFG.PINCM[PINCM_M1] = IOMUX_PINCM_PC_CONNECTED | PF_GPIO;
    IOMUX->SECCFG.PINCM[PINCM_M2] = IOMUX_PINCM_PC_CONNECTED | PF_GPIO;
    IOMUX->SECCFG.PINCM[PINCM_M3] = IOMUX_PINCM_PC_CONNECTED | PF_GPIO;
    IOMUX->SECCFG.PINCM[PINCM_M4] = IOMUX_PINCM_PC_CONNECTED | PF_GPIO;
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
     * CC_1). The unused channel on TIMA1 stays dormant.
     *
     * CCUPD = IMMEDIATELY: DMA writes land in the active CC register
     * directly. ZERO_EVT (shadowed) would seem cleaner, but in practice it
     * races with the DMA write — shadow->active happens "the TIMCLK cycle
     * following CTR=0" while DMA arbitration takes a few cycles, so the
     * first frame's bit 15 (MSB) lands one period late and the shadow
     * register's reset value (undefined) bleeds in. With IMMEDIATELY, the
     * DMA write arrives at active CC ~5-10 cycles into the new period,
     * which is well before the counter reaches even the shortest CC value
     * (CC=13 for a "1" bit, 39 ticks of slack). No glitch. */
    const uint32_t ccctl =
          GPTIMER_CCCTL_01_COC_COMPARE
        | GPTIMER_CCCTL_01_CCUPD_IMMEDIATELY;
    const uint32_t ccact =
          GPTIMER_CCACT_01_LACT_CCP_HIGH
        | GPTIMER_CCACT_01_CDACT_CCP_LOW;
    const uint32_t octl =
          GPTIMER_OCTL_01_CCPO_FUNCVAL
        | GPTIMER_OCTL_01_CCPIV_LOW;
    if (enable_ch0) {
        t->COUNTERREGS.CCCTL_01[0] = ccctl;
        t->COUNTERREGS.CCACT_01[0] = ccact;
        t->COUNTERREGS.OCTL_01[0]  = octl;
        t->COUNTERREGS.CC_01[0]    = DSHOT_CC_QUIET;
    }
    if (enable_ch1) {
        t->COUNTERREGS.CCCTL_01[1] = ccctl;
        t->COUNTERREGS.CCACT_01[1] = ccact;
        t->COUNTERREGS.OCTL_01[1]  = octl;
        t->COUNTERREGS.CC_01[1]    = DSHOT_CC_QUIET;
    }

    /* Publish the zero-event onto the chosen event-fabric channel so the
     * DMA's FSUB_0 can subscribe. pub_chan_id == 0 means "do not publish"
     * — we only publish from TIMA1 since that's the single trigger source
     * for all 4 DMA channels. */
    if (pub_chan_id != 0u) {
        t->GEN_EVENT0.IMASK = GPTIMER_GEN_EVENT0_IMASK_Z_SET;
        t->FPUB_0           = (pub_chan_id & GPTIMER_FPUB_0_CHANID_MASK);
    }
}

static void dshot_init_dma_channel(uint32_t ch,
                                   const volatile uint16_t *src,
                                   volatile uint32_t *dst)
{
    /* DMA module has no GPRCM — it is always-on. */
    DMA->DMACHAN[ch].DMASA  = (uint32_t)src;
    DMA->DMACHAN[ch].DMADA  = (uint32_t)dst;
    DMA->DMACHAN[ch].DMASZ  = 0;  /* set per-frame in dshot_transmit() */

    /* Single transfer: each trigger moves one halfword. The timer's zero
     * event triggers the channel once per bit period, so the 17-halfword
     * frame takes 17 timer periods to drain. BLOCK mode would burst all
     * 17 writes into the same CC register in a few cycles (DST is fixed)
     * — only the last value would survive, and no bit pattern reaches
     * the pin.
     * Source: 16-bit halfword, increment by +1 element per transfer.
     * Destination: 32-bit word write to CC register, address fixed. */
    DMA->DMACHAN[ch].DMACTL =
          DMA_DMACTL_DMATM_SINGLE
        | DMA_DMACTL_DMASRCWDTH_HALF
        | DMA_DMACTL_DMADSTWDTH_WORD
        | DMA_DMACTL_DMASRCINCR_INCREMENT
        | DMA_DMACTL_DMADSTINCR_UNCHANGED;

    /* Trigger from DMA's FSUB_0 line (see DMA_GENERIC_SUB0_TRIG in the
     * device header). DMATINT_EXTERNAL = 0 selects the external-trigger
     * path. The actual fabric-channel ID is configured globally via
     * DMA->FSUB_0 in dshot_init(). */
    DMA->DMATRIG[ch].DMATCTL = DMA_TRIG_SEL_SUB0
                             | DMA_DMATCTL_DMATINT_EXTERNAL;
}

void dshot_init(void)
{
    /* Pre-stage all buffers as MOTOR_STOP so the very first transmit is
     * benign even if the caller forgets to set values. */
    dshot_set_all_motor_stop();

    dshot_init_gpio();

    /* Wire the DMA peripheral's FSUB_0 subscriber port to event-fabric
     * channel EVT_FABRIC_CH. This is the missing global config step — every
     * DMA-channel-level DMATCTL=1 (DMA_GENERIC_SUB0_TRIG) refers to *this*
     * line, which is meaningless until FSUB_0 has a channel ID. */
    DMA->FSUB_0 = EVT_FABRIC_CH;

    /* Configure timers. Only TIMA1 publishes its Z event onto the fabric —
     * TIMG8 and TIMA0 just need to run with the same period; their Z events
     * are unused. */
    dshot_init_one_timer(TIMA1, /*ch0*/1, /*ch1*/1, /*pub*/EVT_FABRIC_CH);
    dshot_init_one_timer(TIMG8, /*ch0*/0, /*ch1*/1, /*pub*/0u);
    dshot_init_one_timer(TIMA0, /*ch0*/0, /*ch1*/1, /*pub*/0u);

    /* All four DMA channels trigger off the same line (DMA->FSUB_0) and so
     * fire simultaneously on each TIMA1 zero event. Each channel still has
     * its own source buffer and destination CC register. */
    dshot_init_dma_channel(DMA_M1, s_frame[DSHOT_MOTOR_1],
                           &TIMA1->COUNTERREGS.CC_01[0]);
    dshot_init_dma_channel(DMA_M2, s_frame[DSHOT_MOTOR_2],
                           &TIMA1->COUNTERREGS.CC_01[1]);
    dshot_init_dma_channel(DMA_M3, s_frame[DSHOT_MOTOR_3],
                           &TIMG8->COUNTERREGS.CC_01[1]);
    dshot_init_dma_channel(DMA_M4, s_frame[DSHOT_MOTOR_4],
                           &TIMA0->COUNTERREGS.CC_01[1]);

    /* Enable counters back-to-back. Small (~few-cycle) skew between them
     * is fine: the DMA writes are triggered only by TIMA1's Z and complete
     * within ~10 cycles, before any timer needs the new CC value for the
     * next period. */
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
    /* Snapshot DMA state from the previous frame for SWD inspection.
     * Healthy values: DMASZ == 0, DMASA == &s_frame[m][17]. */
    dshot_dbg_dmasz_after_tx[DSHOT_MOTOR_1] = DMA->DMACHAN[DMA_M1].DMASZ;
    dshot_dbg_dmasz_after_tx[DSHOT_MOTOR_2] = DMA->DMACHAN[DMA_M2].DMASZ;
    dshot_dbg_dmasz_after_tx[DSHOT_MOTOR_3] = DMA->DMACHAN[DMA_M3].DMASZ;
    dshot_dbg_dmasz_after_tx[DSHOT_MOTOR_4] = DMA->DMACHAN[DMA_M4].DMASZ;
    dshot_dbg_dmasa_after_tx[DSHOT_MOTOR_1] = DMA->DMACHAN[DMA_M1].DMASA;
    dshot_dbg_dmasa_after_tx[DSHOT_MOTOR_2] = DMA->DMACHAN[DMA_M2].DMASA;
    dshot_dbg_dmasa_after_tx[DSHOT_MOTOR_3] = DMA->DMACHAN[DMA_M3].DMASA;
    dshot_dbg_dmasa_after_tx[DSHOT_MOTOR_4] = DMA->DMACHAN[DMA_M4].DMASA;
    dshot_dbg_active_cc_m1 = TIMA1->COUNTERREGS.CC_01[0];
    dshot_dbg_active_cc_m2 = TIMA1->COUNTERREGS.CC_01[1];
    dshot_dbg_active_cc_m3 = TIMG8->COUNTERREGS.CC_01[1];
    dshot_dbg_active_cc_m4 = TIMA0->COUNTERREGS.CC_01[1];
    dshot_dbg_tima1_ctr       = TIMA1->COUNTERREGS.CTR;
    dshot_dbg_timg8_ctr       = TIMG8->COUNTERREGS.CTR;
    dshot_dbg_tima0_ctr       = TIMA0->COUNTERREGS.CTR;
    dshot_dbg_tima1_ctrctl    = TIMA1->COUNTERREGS.CTRCTL;
    dshot_dbg_tima1_rise_z    = TIMA1->GEN_EVENT0.RIS;
    dshot_dbg_tima1_fpub0     = TIMA1->FPUB_0;
    dshot_dbg_tima1_gen_imask = TIMA1->GEN_EVENT0.IMASK;
    dshot_dbg_dmach0_ctl      = DMA->DMACHAN[DMA_M1].DMACTL;
    dshot_dbg_dmach0_tctl     = DMA->DMATRIG[DMA_M1].DMATCTL;
    dshot_dbg_dma_fsub0       = DMA->FSUB_0;

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

    /* Sync the PINCM flip to a TIMA1 zero event so the line's first
     * rising edge after the LOW idle gap lands exactly at the start of
     * a bit period. Without this, the flip happens at a random point in
     * the 53-tick period; the leading-edge-to-falling-edge HIGH stretch
     * the ESC sees for bit 15 is then anywhere from 0 to ~5 µs and
     * intolerant decoders mis-decode the MSB.
     *
     * Worst-case spin: 3.3 µs (one full period). The 10 kHz scheduler
     * tick gives us 100 µs, so this is ~3 % of the budget.
     *
     * We clear RIS.Z first so we wait for a *new* zero event, not a
     * stale latched one. RIS bits aren't auto-cleared on read. */
    TIMA1->GEN_EVENT0.ICLR = GPTIMER_GEN_EVENT0_RIS_Z_SET;
    while ((TIMA1->GEN_EVENT0.RIS
            & GPTIMER_GEN_EVENT0_RIS_Z_MASK) == 0u) {
        /* spin — at most 53 cycles = 3.3 µs */
    }

    /* Counter has just reloaded to LOAD; we're at the very top of a
     * fresh period. Flip the pins now — the timer is about to drive its
     * CCP outputs HIGH (LACT) for the start of this period. The first
     * DMA write (bit 15) lands within ~5 cycles, well before the
     * earliest CDACT (39 cycles into the period for a "1" bit). */
    dshot_pins_to_timer();

    /* Block here until the DMA has streamed all 17 CC values onto the wire
     * (~56 us = 17 bit periods), then immediately park the pins LOW. This
     * ends the frame with a clean falling edge right after bit 0, instead
     * of leaving the timer's quiet-period HIGH on the line for ~43 us until
     * the next scheduler tick. A trailing HIGH that long is not valid DShot
     * idle and can stop BLHeli_S from locking onto / auto-detecting the
     * protocol. The wait fits inside the caller's 100 us tick budget; the
     * guard count is a generous fallback in case the DMA never completes. */
    {
        uint32_t guard = 20000u;
        while (((DMA->DMACHAN[DMA_M1].DMASZ
               | DMA->DMACHAN[DMA_M2].DMASZ
               | DMA->DMACHAN[DMA_M3].DMASZ
               | DMA->DMACHAN[DMA_M4].DMASZ) != 0u)
               && (--guard != 0u)) {
            /* spin */
        }
    }
    dshot_pins_to_gpio_low();

    dshot_frames_started++;
    return true;
}

void dshot_park(void)
{
    dshot_pins_to_gpio_low();
}
