/*
 * pwm_test.c — standalone PWM fallback diagnostic main.
 *
 * PURPOSE: split the "motors won't spin" problem in two. The DShot path is
 * verified correct up to the MCU pins, yet the ESC never responds. This main
 * drives the four motor pins with plain 50 Hz servo PWM (1000-2000 us) using
 * the self-contained pwm.c driver — the protocol every BLHeli / AM32 ESC
 * auto-detects on power-up.
 *
 *   - If PWM spins the motors -> the ESC, motors and wiring are all good,
 *     and the fault is entirely in the DShot layer.
 *   - If PWM also does nothing -> the fault is the ESC, its config, or the
 *     wiring/power — not the MCU firmware.
 *
 * BUILD NOTE: this file and dshot_test.c both define main(). CCS compiles
 * every .c in the project, so exactly one must be excluded from the build:
 *   - To run this PWM test:  right-click dshot_test.c -> Exclude from Build.
 *   - To go back to DShot:   un-exclude dshot_test.c, exclude pwm_test.c.
 *
 * SEQUENCE (no scheduler needed — PWM is generated in hardware by the timers;
 * the CPU only writes CC registers, so the whole test is a blocking loop):
 *   [0 s, 5 s)        ARM      — all four motors held at 1000 us. Power the
 *                               ESC during this window so it arms on PWM.
 *   [5 s, ~17 s)      SWEEP    — each motor in turn ramps 1000 -> 1200 us
 *                               over 2 s, holds 1 s, returns to 1000 us,
 *                               1 s gap. Only the active motor should spin.
 *   [~17 s, inf)      ALL      — all four held at 1150 us (~15 % throttle).
 *
 * SWD WATCH: pwm_phase (0=arm 1=sweep 2=all), pwm_active_motor (0xFF=n/a),
 *            pwm_pulse_us (current pulse width on the active motor).
 *
 * SAFETY: remove propellers before running — the sweep spins motors.
 */

#include <ti/devices/msp/msp.h>
#include <stdint.h>
#include "pwm.h"
#include "delay.h"

/* SWD-visible state. */
volatile uint32_t pwm_phase        = 0;       /* 0=arm 1=sweep 2=all */
volatile uint32_t pwm_active_motor = 0xFFu;   /* motor under test, 0xFF=n/a */
volatile uint32_t pwm_pulse_us     = PWM_PULSE_MIN_US;

/* Timing helpers — MCLK = 16 MHz, so 1 ms = 16000 cycles. */
#define CYCLES_PER_MS   (16000u)
#define ARM_HOLD_MS     (5000u)
#define RAMP_MS         (2000u)   /* 1000 -> 1200 us ramp duration */
#define HOLD_MS         (1000u)   /* hold at top of ramp */
#define GAP_MS          (1000u)   /* rest between motors */

#define SPIN_TOP_US     (1200u)   /* sweep ramp top (~20 % throttle) */
#define ALL_SPIN_US     (1150u)   /* final all-motor hold */

static void delay_ms(uint32_t ms)
{
    /* delay_cycles takes a uint32_t; 5000 ms * 16000 = 8e7, well in range. */
    delay_cycles(ms * CYCLES_PER_MS);
}

static void ConfigureClocks(void)
{
    SYSCTL->SOCLOCK.MCLKCFG =
        (1u << SYSCTL_MCLKCFG_MDIV_OFS) | SYSCTL_MCLKCFG_UDIV_DIVIDE2;
    delay_cycles(POWER_STARTUP_DELAY * 4);
}

/* Ramp one motor from 1000 us to SPIN_TOP_US over RAMP_MS, the other three
 * held at idle. 1 us per step keeps the ramp smooth. */
static void sweep_one_motor(uint32_t m)
{
    pwm_active_motor = m;

    uint32_t span     = SPIN_TOP_US - PWM_PULSE_MIN_US;          /* 200 */
    uint32_t step_ms  = RAMP_MS / span;                          /* 10 ms */
    for (uint32_t us = PWM_PULSE_MIN_US; us <= SPIN_TOP_US; ++us) {
        pwm_set_value((pwm_motor_t)m, (uint16_t)us);
        pwm_pulse_us = us;
        delay_ms(step_ms);
    }

    delay_ms(HOLD_MS);                       /* hold at the top */

    pwm_set_value((pwm_motor_t)m, PWM_PULSE_MIN_US);
    pwm_pulse_us = PWM_PULSE_MIN_US;
    delay_ms(GAP_MS);                        /* rest before next motor */
}

int main(void)
{
    ConfigureClocks();

    /* pwm_init() configures the timers and brings all four motors up at
     * PWM_PULSE_MIN_US (1000 us) immediately. */
    pwm_init();

    /* ARM: hold 1000 us so the ESC sees valid min-throttle PWM and arms.
     * Power the ESC during this 5 s window. */
    pwm_phase = 0u;
    pwm_active_motor = 0xFFu;
    pwm_set_all_min();
    delay_ms(ARM_HOLD_MS);

    /* SWEEP: each motor ramps up alone. */
    pwm_phase = 1u;
    for (uint32_t m = 0; m < PWM_NUM_MOTORS; ++m) {
        sweep_one_motor(m);
    }

    /* ALL: hold every motor at a low steady throttle. */
    pwm_phase = 2u;
    pwm_active_motor = 0xFFu;
    for (uint32_t m = 0; m < PWM_NUM_MOTORS; ++m) {
        pwm_set_value((pwm_motor_t)m, ALL_SPIN_US);
    }
    pwm_pulse_us = ALL_SPIN_US;

    for (;;) {
        /* PWM runs in hardware; nothing left to do. */
    }
}
