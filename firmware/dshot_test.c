/*
 * dshot_test.c — motor-actuation diagnostic main.
 *
 * Background: the DShot300 driver makes the ESC arm (it plays its arming
 * tune), but motors weren't spinning when the old test jumped straight to a
 * fixed throttle. Arming only ever sends value 0 — whose CRC is trivially 0 —
 * so it proves bit timing + frame alignment but says nothing about non-zero
 * values or which of the four channels is actually wired. This main runs a
 * staged sweep that isolates each of those unknowns.
 *
 * SEQUENCE (all four motors share the 1 kHz DShot frame stream throughout):
 *
 *   [0 s, 3 s)      ARM        — MOTOR_STOP on all four; ESC arms here.
 *   [3 s, 5.4 s)    BEEP SWEEP — each motor in turn gets a DShot BEEP3
 *                                command (others MOTOR_STOP). You should
 *                                hear four beeps, one per motor, in order
 *                                M1, M2, M3, M4. A beep is a non-zero DShot
 *                                command, so hearing it proves the signal
 *                                path AND the CRC are good for that channel —
 *                                without needing the motor to mechanically
 *                                spin. A silent channel = no signal reaching
 *                                that ESC input (wiring / pin-map).
 *   [5.4 s, 15.4 s) SPIN SWEEP — each motor in turn ramps 48→600 over 2 s
 *                                (others MOTOR_STOP), then 0.5 s rest. Only
 *                                the active motor should spin up. This also
 *                                nails down the PB-pin → physical-motor map.
 *   [15.4 s, inf)   ALL SPIN   — all four held at 600 (~28 %).
 *
 * Interpreting the result:
 *   - beeps on all 4, spins on all 4   → driver fine; earlier fixed-throttle
 *                                        test just had a stale build/wiring.
 *   - beeps on all 4, spins on none    → signal+CRC are good; the problem is
 *                                        ESC config (BLHeliSuite) or motors.
 *   - beeps on some, silent on others  → those channels have no signal —
 *                                        wiring or PB-pin/PINCM mismatch.
 *
 * SWD WATCH:
 *   - diag_phase: 0=arm 1=beep 2=spin-sweep 3=all-spin.
 *   - diag_active_motor: motor index being exercised (0xFF = all/none).
 *   - dshot_frames_started: should climb at ~1000/sec.
 *   - dshot_frames_skipped: should stay 0.
 *   - tick_count_10khz / boot_seconds: scheduler tick.
 *   - imu_*: IMU still polled in the background (see imu.h).
 */

#include <ti/devices/msp/msp.h>
#include <stdint.h>
#include <stdbool.h>
#include "dshot.h"
#include "delay.h"
#include "imu.h"

volatile uint32_t tick_count_10khz = 0;
volatile uint32_t boot_seconds     = 0;

/* Set by the 10 kHz ISR, cleared by the main loop. The IMU SPI burst runs in
 * thread context (not the ISR) so it never extends the tick handler. */
volatile bool imu_poll_due = false;

/* Diagnostic state, mirrored for SWD inspection. */
volatile uint32_t diag_phase        = 0;     /* 0=arm 1=beep 2=spin 3=all */
volatile uint32_t diag_active_motor = 0xFFu; /* motor under test, 0xFF=n/a */

/* ---- Physical pin self-test (SWD-readable, no multimeter needed) ------ *
 * Set PIN_WALK_TEST to 1 to bypass the entire DShot / IMU stack and instead
 * exercise each motor pin (PB17..PB20) as a plain GPIO. For every pin the
 * test drives it HIGH then LOW and reads the *pad voltage back* through the
 * GPIO input buffer (PINCM.INENA enabled), so the result is visible purely
 * over SWD — no probing of the tiny QFN pins required.
 *
 * Watch these in the Expressions view:
 *   pinwalk_pass         — loop counter; must be climbing (test is running).
 *   pinwalk_high_rb[0..3]— pad level read back while driven HIGH; want 1.
 *   pinwalk_low_rb[0..3] — pad level read back while driven LOW;  want 0.
 *   pinwalk_ok[0..3]     — 1 = that pin toggles cleanly (high_rb=1, low_rb=0).
 * Index order is PB17, PB18, PB19, PB20.
 *
 * Interpreting pinwalk_ok / readbacks:
 *   all ok[i]==1                  -> all four MCU pins drive fine; the signal
 *                                    IS produced at the MCU. Fault is
 *                                    downstream: J1 connector, cable, or ESC.
 *   ok[i]==0 with high_rb[i]==0   -> that pin can't reach HIGH: net shorted to
 *                                    GND, or a broken/cold pad solder joint.
 *   ok[i]==0 with low_rb[i]==1    -> that pin stuck HIGH: net shorted to 3V3.
 * (An open trace *downstream* of the pin still reads ok==1 — the readback
 * only proves the MCU pad itself works; it can't see past a broken wire.)
 *
 * Set PIN_WALK_TEST back to 0 afterwards for the normal motor diagnostic. */
#define PIN_WALK_TEST      (0)

volatile uint32_t pinwalk_active_pin = 0;     /* SWD: PB pin under test now */
volatile uint32_t pinwalk_pass       = 0;     /* SWD: loop counter          */
volatile uint8_t  pinwalk_high_rb[4];         /* SWD: pad readback, driven HIGH */
volatile uint8_t  pinwalk_low_rb[4];          /* SWD: pad readback, driven LOW  */
volatile uint8_t  pinwalk_ok[4];              /* SWD: 1 = pin toggles cleanly   */

#if PIN_WALK_TEST
static void pin_walk_test(void)
{
    /* GPIOB power-up (idempotent guard, same pattern the drivers use). */
    if ((GPIOB->GPRCM.STAT & GPIO_STAT_RESETSTKY_MASK) != 0u) {
        GPIOB->GPRCM.RSTCTL = (GPIO_RSTCTL_KEY_UNLOCK_W
                               | GPIO_RSTCTL_RESETSTKYCLR_CLR
                               | GPIO_RSTCTL_RESETASSERT_ASSERT);
        GPIOB->GPRCM.PWREN  = (GPIO_PWREN_KEY_UNLOCK_W
                               | GPIO_PWREN_ENABLE_ENABLE);
        delay_cycles(POWER_STARTUP_DELAY);
    }

    static const uint32_t pin[4]   = { 17u, 18u, 19u, 20u };
    /* PINCM index = datasheet PINCM number - 1 (verified: IOMUX_PINCM43 = 42).
     * PB17=43, PB18=44, PB19=45, PB20=48. */
    static const uint32_t pincm[4] = { 43u - 1u, 44u - 1u, 45u - 1u, 48u - 1u };
    uint32_t mask = 0u;
    for (uint32_t i = 0; i < 4u; ++i) {
        mask |= (1u << pin[i]);
        /* GPIO function (PF=1), pin connected, input buffer ENABLED so the
         * pad voltage can be read back via GPIOB->DIN31_0. */
        IOMUX->SECCFG.PINCM[pincm[i]] =
            IOMUX_PINCM_PC_CONNECTED | IOMUX_PINCM_INENA_ENABLE | 1u;
    }
    GPIOB->DOUTCLR31_0 = mask;   /* all LOW first */
    GPIOB->DOESET31_0  = mask;   /* all outputs   */

    for (;;) {
        for (uint32_t i = 0; i < 4u; ++i) {
            pinwalk_active_pin = pin[i];

            GPIOB->DOUTSET31_0 = (1u << pin[i]);   /* drive HIGH */
            delay_cycles(16000u);                  /* ~1 ms settle */
            pinwalk_high_rb[i] =
                (uint8_t)((GPIOB->DIN31_0 >> pin[i]) & 1u);

            GPIOB->DOUTCLR31_0 = (1u << pin[i]);   /* drive LOW */
            delay_cycles(16000u);
            pinwalk_low_rb[i] =
                (uint8_t)((GPIOB->DIN31_0 >> pin[i]) & 1u);

            pinwalk_ok[i] = (uint8_t)((pinwalk_high_rb[i] == 1u)
                                   && (pinwalk_low_rb[i] == 0u));
        }
        pinwalk_active_pin = 0u;
        pinwalk_pass++;
    }
}
#endif

#define TIMG0_PREDIV       (8u)
#define TIMG0_RELOAD       (200u)

/* ---- Diagnostic schedule. 1 DShot frame = 1 ms, so all times are in ms. -- */
#define MS_ARM_END         (3000u)
#define BEEP_SLOT_MS       (600u)                       /* per motor */
#define BEEP_ON_MS         (200u)                       /* command sent */
#define MS_BEEP_END        (MS_ARM_END + 4u * BEEP_SLOT_MS)   /* 5400 */
#define SPIN_SLOT_MS       (2500u)                      /* per motor */
#define SPIN_ON_MS         (2000u)                      /* ramp window */
#define MS_SPIN_END        (MS_BEEP_END + 4u * SPIN_SLOT_MS)  /* 15400 */

#define DIAG_BEEP_CMD      (3u)    /* DShot BEEP3 — mid-frequency, audible */
#define DIAG_SPIN_MIN      (48u)   /* lowest throttle value (0 % throttle) */
#define DIAG_SPIN_MAX      (600u)  /* ramp top / hold value (~28 % throttle) */

/* Stage `value` on motor `active`, MOTOR_STOP on the other three. */
static void diag_stage_one(uint32_t active, uint16_t value, bool telem)
{
    for (uint32_t m = 0; m < DSHOT_NUM_MOTORS; ++m) {
        if (m == active) {
            dshot_set_value((dshot_motor_t)m, value, telem);
        } else {
            dshot_set_value((dshot_motor_t)m, DSHOT_CMD_MOTOR_STOP, false);
        }
    }
}

/* Decide what every motor should send this frame, given elapsed ms. */
static void diag_stage(uint32_t ms)
{
    if (ms < MS_ARM_END) {
        diag_phase = 0u;
        diag_active_motor = 0xFFu;
        dshot_set_all_motor_stop();
        return;
    }

    if (ms < MS_BEEP_END) {
        uint32_t t     = ms - MS_ARM_END;
        uint32_t motor = (t / BEEP_SLOT_MS) % DSHOT_NUM_MOTORS;
        uint32_t phase = t % BEEP_SLOT_MS;
        diag_phase = 1u;
        diag_active_motor = motor;
        if (phase < BEEP_ON_MS) {
            /* Telemetry bit set marks this as a DShot command frame. */
            diag_stage_one(motor, DIAG_BEEP_CMD, true);
        } else {
            dshot_set_all_motor_stop();
        }
        return;
    }

    if (ms < MS_SPIN_END) {
        uint32_t t     = ms - MS_BEEP_END;
        uint32_t motor = (t / SPIN_SLOT_MS) % DSHOT_NUM_MOTORS;
        uint32_t phase = t % SPIN_SLOT_MS;
        diag_phase = 2u;
        diag_active_motor = motor;
        if (phase < SPIN_ON_MS) {
            /* Linear ramp 48 → DIAG_SPIN_MAX across the 2 s window so a
             * cogging-stiff motor is eased past its break-away point. */
            uint32_t span = DIAG_SPIN_MAX - DIAG_SPIN_MIN;
            uint16_t thr  = (uint16_t)(DIAG_SPIN_MIN + (span * phase) / SPIN_ON_MS);
            diag_stage_one(motor, thr, false);
        } else {
            dshot_set_all_motor_stop();
        }
        return;
    }

    /* Steady state: all four motors held at the ramp-top value. */
    diag_phase = 3u;
    diag_active_motor = 0xFFu;
    dshot_set_value(DSHOT_MOTOR_1, DIAG_SPIN_MAX, false);
    dshot_set_value(DSHOT_MOTOR_2, DIAG_SPIN_MAX, false);
    dshot_set_value(DSHOT_MOTOR_3, DIAG_SPIN_MAX, false);
    dshot_set_value(DSHOT_MOTOR_4, DIAG_SPIN_MAX, false);
}

static void ConfigureClocks(void)
{
    SYSCTL->SOCLOCK.MCLKCFG =
        (1u << SYSCTL_MCLKCFG_MDIV_OFS) | SYSCTL_MCLKCFG_UDIV_DIVIDE2;
    delay_cycles(POWER_STARTUP_DELAY * 4);
}

static void InitializeTimerG0_OnMCLK(void)
{
    TIMG0->GPRCM.RSTCTL = (GPTIMER_RSTCTL_KEY_UNLOCK_W
                           | GPTIMER_RSTCTL_RESETSTKYCLR_CLR
                           | GPTIMER_RSTCTL_RESETASSERT_ASSERT);
    TIMG0->GPRCM.PWREN  = (GPTIMER_PWREN_KEY_UNLOCK_W
                           | GPTIMER_PWREN_ENABLE_ENABLE);
    delay_cycles(POWER_STARTUP_DELAY);

    TIMG0->CLKSEL                  = GPTIMER_CLKSEL_BUSCLK_SEL_ENABLE;
    TIMG0->CLKDIV                  = (TIMG0_PREDIV - 1u);
    TIMG0->COUNTERREGS.LOAD        = (TIMG0_RELOAD - 1u);
    TIMG0->COUNTERREGS.CTRCTL      = GPTIMER_CTRCTL_REPEAT_REPEAT_1;
    TIMG0->CPU_INT.IMASK           = GPTIMER_CPU_INT_IMASK_Z_SET;
    TIMG0->COMMONREGS.CCLKCTL      = GPTIMER_CCLKCTL_CLKEN_ENABLED;

    NVIC_EnableIRQ(TIMG0_INT_IRQn);
    TIMG0->COUNTERREGS.CTRCTL |= GPTIMER_CTRCTL_EN_ENABLED;
}

void TIMG0_IRQHandler(void)
{
    switch (TIMG0->CPU_INT.IIDX) {
        case GPTIMER_CPU_INT_IIDX_STAT_Z:
            tick_count_10khz++;
            if ((tick_count_10khz % 10000u) == 0u) {
                boot_seconds++;
            }
            /* 10 kHz tick / 10 = 1 kHz DShot frame rate.
             * tick%10==0 starts the frame; tick%10==1 parks the pins LOW. */
            if ((tick_count_10khz % 10u) == 0u) {
                diag_stage(tick_count_10khz / 10u);   /* frame index = ms */
                (void)dshot_transmit();
            } else if ((tick_count_10khz % 10u) == 1u) {
                /* Frame is ~57 µs; we're now 100 µs past frame start. */
                dshot_park();
            } else if ((tick_count_10khz % 10u) == 5u) {
                imu_poll_due = true;
            }
            break;
        default:
            break;
    }
}

int main(void)
{
    ConfigureClocks();

#if PIN_WALK_TEST
    pin_walk_test();   /* never returns — physical signal-path check */
#endif

    dshot_init();
    InitializeTimerG0_OnMCLK();

    __enable_irq();

    /* IMU bring-up after IRQs are live, so DShot frames are already
     * streaming to the ESCs during the IMU's blocking settle delays. */
    imu_init();

    imu_sample_t sample;

    for (;;) {
        if (imu_poll_due) {
            imu_poll_due = false;
            (void)imu_read_sample(&sample);
        }
        __WFI();
    }
}
