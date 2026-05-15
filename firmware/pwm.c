/*
 * pwm.c — Standard 50 Hz servo PWM on 4 motors, no DMA.
 *
 * CLOCK PLAN @ MCLK = 16 MHz:
 *   CLKDIV = /8        -> TIMCLK = 2 MHz   (1 tick = 0.5 µs)
 *   LOAD   = 40000-1   -> period = 20 ms   (50 Hz)
 *
 * BIT TIMING:
 *   Each timer runs down-counter, edge-aligned PWM. At LOAD (start of
 *   period) the CCP goes high (LACT = HIGH). When the counter passes CC
 *   the CCP goes low (CDACT = LOW). Pulse width = (LOAD - CC) ticks =
 *   (LOAD - CC) * 0.5 µs.
 *
 *   CC = LOAD - (pulse_us * 2)
 *
 *   1000 µs -> 2000 ticks high -> CC = 37999
 *   2000 µs -> 4000 ticks high -> CC = 35999
 *
 * Why CCUPD_IMMEDIATELY: there's no DMA racing with the shadow update,
 * but writes happen at arbitrary moments from the scheduler. Using the
 * immediate-update path means the new pulse width might glitch by a few
 * ticks if the CPU write lands mid-pulse — but for servo PWM with 1000+
 * µs pulses, a sub-µs glitch is invisible to the ESC.
 */

#include <ti/devices/msp/msp.h>
#include <stdint.h>
#include "pwm.h"
#include "delay.h"

/* ---- Period / pulse-width math (2 MHz timer clock) ---------------- */
#define PWM_LOAD            (40000u - 1u)   /* 20 ms @ 2 MHz */

static inline uint32_t pwm_cc_for_us(uint32_t us)
{
    /* CC = LOAD - high_ticks, where high_ticks = us * 2 (2 MHz timer). */
    if (us < PWM_PULSE_MIN_US) us = PWM_PULSE_MIN_US;
    if (us > PWM_PULSE_MAX_US) us = PWM_PULSE_MAX_US;
    return PWM_LOAD - (us * 2u);
}

/* ---- IOMUX pin assignments (same as dshot.c) ---------------------- */
#define PINCM_M1            (43u - 1u)   /* PB17 */
#define PINCM_M2            (44u - 1u)   /* PB18 */
#define PINCM_M3            (45u - 1u)   /* PB19 */
#define PINCM_M4            (48u - 1u)   /* PB20 */

#define PF_M1_TIMA1_CCP0    (5u)
#define PF_M2_TIMA1_CCP1    (5u)
#define PF_M3_TIMG8_CCP1    (4u)
#define PF_M4_TIMA0_CCP1    (7u)

/* ---- Bring-up ----------------------------------------------------- */

static void pwm_init_gpio(void)
{
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

static void pwm_init_one_timer(GPTIMER_Regs *t,
                               uint32_t enable_ch0,
                               uint32_t enable_ch1)
{
    /* Standard GPRCM dance. */
    t->GPRCM.RSTCTL = (GPTIMER_RSTCTL_KEY_UNLOCK_W
                       | GPTIMER_RSTCTL_RESETSTKYCLR_CLR
                       | GPTIMER_RSTCTL_RESETASSERT_ASSERT);
    t->GPRCM.PWREN  = (GPTIMER_PWREN_KEY_UNLOCK_W
                       | GPTIMER_PWREN_ENABLE_ENABLE);
    delay_cycles(POWER_STARTUP_DELAY);

    /* BUSCLK = MCLK = 16 MHz. CLKDIV = /8 -> TIMCLK = 2 MHz. */
    t->CLKSEL                 = GPTIMER_CLKSEL_BUSCLK_SEL_ENABLE;
    t->CLKDIV                 = GPTIMER_CLKDIV_RATIO_DIV_BY_8;
    t->COMMONREGS.CCLKCTL     = GPTIMER_CCLKCTL_CLKEN_ENABLED;

    /* Down-counter, repeating, counter loaded with LOAD on enable. */
    t->COUNTERREGS.LOAD       = PWM_LOAD;
    t->COUNTERREGS.CTRCTL     = (GPTIMER_CTRCTL_CM_DOWN
                                 | GPTIMER_CTRCTL_CVAE_LDVAL
                                 | GPTIMER_CTRCTL_REPEAT_REPEAT_1);

    const uint32_t ccctl =
          GPTIMER_CCCTL_01_COC_COMPARE
        | GPTIMER_CCCTL_01_CCUPD_IMMEDIATELY;
    const uint32_t ccact =
          GPTIMER_CCACT_01_LACT_CCP_HIGH
        | GPTIMER_CCACT_01_CDACT_CCP_LOW;
    const uint32_t octl =
          GPTIMER_OCTL_01_CCPO_FUNCVAL
        | GPTIMER_OCTL_01_CCPIV_LOW;

    const uint32_t idle_cc = pwm_cc_for_us(PWM_PULSE_MIN_US);

    if (enable_ch0) {
        t->COUNTERREGS.CCCTL_01[0] = ccctl;
        t->COUNTERREGS.CCACT_01[0] = ccact;
        t->COUNTERREGS.OCTL_01[0]  = octl;
        t->COUNTERREGS.CC_01[0]    = idle_cc;
    }
    if (enable_ch1) {
        t->COUNTERREGS.CCCTL_01[1] = ccctl;
        t->COUNTERREGS.CCACT_01[1] = ccact;
        t->COUNTERREGS.OCTL_01[1]  = octl;
        t->COUNTERREGS.CC_01[1]    = idle_cc;
    }
}

void pwm_init(void)
{
    pwm_init_gpio();

    /* TIMA1 drives Motor 1 (CC_0) and Motor 2 (CC_1).
     * TIMG8 drives Motor 3 (CC_1 only).
     * TIMA0 drives Motor 4 (CC_1 only). */
    pwm_init_one_timer(TIMA1, /*ch0*/1, /*ch1*/1);
    pwm_init_one_timer(TIMG8, /*ch0*/0, /*ch1*/1);
    pwm_init_one_timer(TIMA0, /*ch0*/0, /*ch1*/1);

    /* Start the counters. Each timer runs independently; they don't need
     * to be phase-aligned because each ESC sees only its own pin. */
    TIMA1->COUNTERREGS.CTRCTL |= GPTIMER_CTRCTL_EN_ENABLED;
    TIMG8->COUNTERREGS.CTRCTL |= GPTIMER_CTRCTL_EN_ENABLED;
    TIMA0->COUNTERREGS.CTRCTL |= GPTIMER_CTRCTL_EN_ENABLED;
}

void pwm_set_value(pwm_motor_t m, uint16_t pulse_us)
{
    const uint32_t cc = pwm_cc_for_us((uint32_t)pulse_us);

    switch (m) {
        case PWM_MOTOR_1: TIMA1->COUNTERREGS.CC_01[0] = cc; break;
        case PWM_MOTOR_2: TIMA1->COUNTERREGS.CC_01[1] = cc; break;
        case PWM_MOTOR_3: TIMG8->COUNTERREGS.CC_01[1] = cc; break;
        case PWM_MOTOR_4: TIMA0->COUNTERREGS.CC_01[1] = cc; break;
        default: break;
    }
}

void pwm_set_all_min(void)
{
    pwm_set_value(PWM_MOTOR_1, PWM_PULSE_MIN_US);
    pwm_set_value(PWM_MOTOR_2, PWM_PULSE_MIN_US);
    pwm_set_value(PWM_MOTOR_3, PWM_PULSE_MIN_US);
    pwm_set_value(PWM_MOTOR_4, PWM_PULSE_MIN_US);
}
