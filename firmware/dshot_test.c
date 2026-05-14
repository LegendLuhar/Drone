/*
 * dshot_test.c — bring-up main for the DShot driver.
 *
 * Sequence:
 *   - For the first ARM_HOLD_SECONDS the firmware streams MOTOR_STOP on
 *     all four motors so the ESCs see a valid DShot signal and arm.
 *   - After the arm hold, all four motors are commanded to SPIN_THROTTLE
 *     and stay there until power-off.
 *
 * SAFETY:
 *   - SPIN_THROTTLE = 150 is a low spin (~7 % of full range above the
 *     48-count idle floor). Enough to confirm rotation, not enough to
 *     produce meaningful thrust.
 *   - The ARM_HOLD gives you a 3-second window to kill power if anything
 *     looks wrong (waveform, motor direction, mechanical bind).
 *   - Keep the airframe restrained. Hands clear before MOTOR_STOP ends.
 *
 * SWD WATCH:
 *   - dshot_frames_started: should climb at ~1000/sec.
 *   - dshot_frames_skipped: should stay 0. Any non-zero means a DMA
 *     channel didn't drain in time -> event routing or trigger bug.
 *   - tick_count_10khz / boot_seconds: scheduler tick.
 */

#include <ti/devices/msp/msp.h>
#include <stdint.h>
#include <stdbool.h>
#include "dshot.h"
#include "delay.h"

volatile uint32_t tick_count_10khz = 0;
volatile uint32_t boot_seconds     = 0;

#define TIMG0_PREDIV       (8u)
#define TIMG0_RELOAD       (200u)

/* Arming behaviour. ARM_HOLD_SECONDS of MOTOR_STOP, then constant low
 * spin on every motor. */
#define ARM_HOLD_SECONDS   (3u)
#define ARM_HOLD_TICKS     (ARM_HOLD_SECONDS * 10000u)
#define SPIN_THROTTLE      (150u)   /* range 48..2047; 150 is gentle */

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
            /* 10 kHz tick / 10 = 1 kHz DShot frame rate. */
            if ((tick_count_10khz % 10u) == 0u) {
                if (tick_count_10khz < ARM_HOLD_TICKS) {
                    dshot_set_all_motor_stop();
                } else {
                    dshot_set_value(DSHOT_MOTOR_1, SPIN_THROTTLE, false);
                    dshot_set_value(DSHOT_MOTOR_2, SPIN_THROTTLE, false);
                    dshot_set_value(DSHOT_MOTOR_3, SPIN_THROTTLE, false);
                    dshot_set_value(DSHOT_MOTOR_4, SPIN_THROTTLE, false);
                }
                (void)dshot_transmit();
            }
            break;
        default:
            break;
    }
}

int main(void)
{
    ConfigureClocks();
    dshot_init();
    InitializeTimerG0_OnMCLK();

    __enable_irq();

    for (;;) {
        __WFI();
    }
}
