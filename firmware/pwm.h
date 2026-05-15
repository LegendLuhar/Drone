/*
 * pwm.h — 50 Hz servo PWM driver for 4 motors.
 *
 * Same pin assignment as dshot.h:
 *   Motor1  PB17  PINCM43  TIMA1_CCP0 (PF=5)
 *   Motor2  PB18  PINCM44  TIMA1_CCP1 (PF=5)
 *   Motor3  PB19  PINCM45  TIMG8_CCP1 (PF=4)
 *   Motor4  PB20  PINCM48  TIMA0_CCP1 (PF=7)
 *
 * Each timer is in standalone edge-aligned PWM mode — no DMA, no event
 * fabric, no inter-timer synchronization. To change a motor's pulse width
 * you just write the timer's CC register; the change takes effect on the
 * next period boundary.
 *
 * Output: 50 Hz period (20 ms) with 1000-2000 µs pulse width. The pin is
 * HIGH for the pulse duration at the start of each 20 ms period, then LOW.
 * This is the standard servo PWM that every BLHeli / AM32 ESC auto-detects
 * on power-up.
 *
 * Arming procedure: drive 1000 µs (PWM_PULSE_MIN_US) for ~2 seconds after
 * ESC power-up. The ESC plays its arm tone, then accepts 1000-2000 µs
 * throttle values.
 */

#ifndef PWM_H
#define PWM_H

#include <stdint.h>

#define PWM_NUM_MOTORS  (4u)

typedef enum {
    PWM_MOTOR_1 = 0,
    PWM_MOTOR_2 = 1,
    PWM_MOTOR_3 = 2,
    PWM_MOTOR_4 = 3,
} pwm_motor_t;

/* Standard servo PWM pulse widths in microseconds. */
#define PWM_PULSE_MIN_US  (1000u)   /* idle / armed-min throttle */
#define PWM_PULSE_MAX_US  (2000u)   /* full throttle */

/* Configure GPIO, all three timers (TIMA1, TIMG8, TIMA0), and start them
 * counting. All four motors come up at PWM_PULSE_MIN_US (idle). Call once
 * during boot, before __enable_irq(). */
void pwm_init(void);

/* Update one motor's pulse width. Clamped to [PWM_PULSE_MIN_US,
 * PWM_PULSE_MAX_US]. Takes effect on the next 20 ms period boundary.
 * Safe to call from an ISR. */
void pwm_set_value(pwm_motor_t m, uint16_t pulse_us);

/* Convenience: drive all four motors to PWM_PULSE_MIN_US (1000 µs). Use
 * during the arm-hold window. */
void pwm_set_all_min(void);

#endif /* PWM_H */
