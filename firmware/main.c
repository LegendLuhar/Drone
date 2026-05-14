/*
 * Drone firmware — Milestone 1: clock-tree skeleton
 *
 * Brings up MCLK = 16 MHz from SYSOSC and runs a free-running TIMG0 tick
 * counter so the clock rate can be verified over SWD without a scope or
 * logic analyzer. Motor outputs (PB17-PB20) are intentionally left
 * tri-stated until Milestone 2 so the firmware cannot accidentally drive
 * ESC inputs during bring-up.
 *
 * SWD verification recipe:
 *   1. Halt at the breakpoint at the end of ConfigureClocks(). Inspect
 *      SYSCTL.SOCLOCK status registers in the memory window.
 *   2. Continue. Wait ~5 wall-clock seconds. Halt. Read tick_count_10khz.
 *      Expected ~50000. If the count is far off, MCLK is not 16 MHz.
 */

#include <ti/devices/msp/msp.h>
#include <stdint.h>
#include <stdbool.h>
#include "delay.h"

/* SWD-readable verification globals. `volatile` so the compiler keeps the
 * stores; the SWD watch window reads them live. */
volatile uint32_t tick_count_10khz = 0;
volatile uint32_t boot_seconds     = 0;  /* derived: tick_count_10khz / 10000 */

/* Target: TIMG0 fires once every 100 us → 10 kHz interrupt rate.
 * MCLK = 16 MHz. Pre-divide by 8 → 2 MHz. Reload value 200 → 100 us. */
#define TIMG0_PREDIV       (8u)
#define TIMG0_RELOAD       (200u)

static void ConfigureClocks(void)
{
    /*
     * Bring MCLK to 16 MHz from SYSOSC. The MSPM0G3507 BCR boots with
     * SYSOSC at 32 MHz feeding HSCLK; we set MDIV = /2 to land at 16 MHz.
     * If a future revision needs HFXT/SYSPLL for tighter ppm, this is
     * the place to do it.
     *
     * Register names follow the MSPM0 device header (ti/devices/msp/msp.h).
     * The exact bitfields here may need a small tweak when this first
     * compiles against the SDK — the intent is documented inline.
     */

    /*
     * Intent (verify exact field names against ti/devices/msp/msp.h on
     * first compile — the MSPM0 SDK has occasionally renamed these):
     *
     *   - SYSOSC stays at its 32 MHz base frequency (BCR default).
     *   - MCLKCFG: source = SYSOSC (HSCLK), MDIV configured so MCLK = 16 MHz.
     *     Most MSPM0G3507 SDK headers expose MDIV as a 4-bit field where
     *     the divisor = MDIV + 1, so MDIV = 1 → /2 → 16 MHz from a 32 MHz
     *     SYSOSC.
     *   - UDIV similarly placed for ULPCLK ≤ MCLK constraint.
     *
     * If the symbols below don't resolve, the closest replacements in the
     * SDK are typically named SYSCTL_MCLKCFG_MDIV_DIVIDE_BY_2 or similar.
     */
    SYSCTL->SOCLOCK.MCLKCFG =
        (1u << SYSCTL_MCLKCFG_MDIV_OFS) |
        SYSCTL_MCLKCFG_UDIV_DIVIDE2;

    /* Let the dividers settle before downstream peripherals start clocking.
     * delay_cycles uses the new MCLK so this wait is short but adequate. */
    delay_cycles(POWER_STARTUP_DELAY * 4);
}

static void InitializeTimerG0_OnMCLK(void)
{
    /* Canonical GPRCM dance — same shape as Simon_accel/timing.c, but we
     * pick BUSCLK (= MCLK) instead of LFCLK so the count rate is a direct
     * function of MCLK. That's the whole point in this milestone. */
    TIMG0->GPRCM.RSTCTL = (GPTIMER_RSTCTL_KEY_UNLOCK_W
                           | GPTIMER_RSTCTL_RESETSTKYCLR_CLR
                           | GPTIMER_RSTCTL_RESETASSERT_ASSERT);
    TIMG0->GPRCM.PWREN  = (GPTIMER_PWREN_KEY_UNLOCK_W
                           | GPTIMER_PWREN_ENABLE_ENABLE);
    delay_cycles(POWER_STARTUP_DELAY);

    TIMG0->CLKSEL = GPTIMER_CLKSEL_BUSCLK_SEL_ENABLE;
    /* CLKDIV: divide BUSCLK by TIMG0_PREDIV (registers store DIV-1). */
    TIMG0->CLKDIV = (TIMG0_PREDIV - 1u);

    /* Down-counter, repeating, reload TIMG0_RELOAD-1 (zero-based). */
    TIMG0->COUNTERREGS.LOAD   = (TIMG0_RELOAD - 1u);
    TIMG0->COUNTERREGS.CTRCTL = GPTIMER_CTRCTL_REPEAT_REPEAT_1;

    TIMG0->CPU_INT.IMASK         = GPTIMER_CPU_INT_IMASK_Z_SET;
    TIMG0->COMMONREGS.CCLKCTL    = GPTIMER_CCLKCTL_CLKEN_ENABLED;

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
            break;
        default:
            break;
    }
}

int main(void)
{
    ConfigureClocks();
    InitializeTimerG0_OnMCLK();

    __enable_irq();

    /* Idle loop. Every wake comes from TIMG0; we have nothing else to do
     * in this milestone. SWD halt + watch on tick_count_10khz / boot_seconds
     * is the verification path. */
    for (;;) {
        __WFI();
    }
}
