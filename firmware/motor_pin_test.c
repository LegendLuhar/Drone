/*
 * motor_pin_test.c — verify the Motor1 signal pin-path
 *
 * Toggles PB17 at ~1 kHz forever. PB17 is the MCU-side end of the Motor1
 * ESC signal trace; it lands on J1 pin 3.
 *
 * SCOPE setup:
 *   - Power the FC via SWD only (no battery, ESC unplugged).
 *   - Probe J1 pin 3 (Motor1) against GND.
 *   - Flash and run this firmware.
 *
 * EXPECTED:
 *   ~1 kHz square wave, 50% duty, 0V↔3.3V.
 *
 * IF YOU SEE:
 *   - 1 kHz clean square wave  → pin path, clock, GPIO module all good.
 *                                Ready to write the DShot driver next.
 *   - Different frequency      → MCLK is not what we assumed; tells us
 *                                the actual MCLK by ratio.
 *   - Nothing / floating       → either the IOMUX isn't taking, the trace
 *                                is broken, or GPIOB isn't powering up.
 *                                Halt over SWD and inspect GPIOB->GPRCM.STAT.
 *
 * Build: replace main.c in the Drone/firmware project with this file
 *        (or rename main.c aside and rebuild).
 */

#include <ti/devices/msp/msp.h>
#include <stdint.h>
#include "delay.h"

/* PB17 = Motor1 signal pin. PINCM index 43 per datasheet Table 6-2. */
#define MOTOR1_PIN_BIT       (17u)
#define MOTOR1_PIN_MASK      (1u << MOTOR1_PIN_BIT)
#define MOTOR1_PINCM_INDEX   (43u - 1u)   /* PINCM[] array is 0-indexed */
#define IOMUX_PF_GPIO        (1u)         /* PF=1 is the digital-GPIO function */

/*
 * Half-period delay in CPU cycles for ~1 kHz toggle.
 * Assumes MCLK = 16 MHz (current Simon_accel-style boot default per
 * project_drone_hardware.md). 1 kHz → 500 us per half-period →
 * 500e-6 * 16e6 = 8000 cycles. If actual MCLK is 32 MHz the observed
 * frequency will be ~2 kHz, which still tells us the pin path works.
 */
#define HALF_PERIOD_CYCLES   (8000u)

int main(void)
{
    /* Power-on GPIOB. Canonical GPRCM dance (matches Simon_accel pattern). */
    GPIOB->GPRCM.RSTCTL = (GPIO_RSTCTL_KEY_UNLOCK_W
                           | GPIO_RSTCTL_RESETSTKYCLR_CLR
                           | GPIO_RSTCTL_RESETASSERT_ASSERT);
    GPIOB->GPRCM.PWREN  = (GPIO_PWREN_KEY_UNLOCK_W
                           | GPIO_PWREN_ENABLE_ENABLE);
    delay_cycles(POWER_STARTUP_DELAY);

    /* IOMUX: route PB17 to digital GPIO. */
    IOMUX->SECCFG.PINCM[MOTOR1_PINCM_INDEX] =
        (IOMUX_PINCM_PC_CONNECTED | IOMUX_PF_GPIO);

    /* Set PB17 as an output, start low. */
    GPIOB->DOUT31_0 &= ~MOTOR1_PIN_MASK;
    GPIOB->DOE31_0  |=  MOTOR1_PIN_MASK;

    for (;;) {
        GPIOB->DOUT31_0 |=  MOTOR1_PIN_MASK;
        delay_cycles(HALF_PERIOD_CYCLES);
        GPIOB->DOUT31_0 &= ~MOTOR1_PIN_MASK;
        delay_cycles(HALF_PERIOD_CYCLES);
    }
}
