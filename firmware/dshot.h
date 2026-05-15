/*
 * dshot.h — DShot300 driver, timer + DMA, 4 motors.
 *
 * Pin / timer assignment (from project_drone_hardware.md):
 *   Motor1  PB17  PINCM43  TIMA1_CCP0 (PF=5)
 *   Motor2  PB18  PINCM44  TIMA1_CCP1 (PF=5)
 *   Motor3  PB19  PINCM45  TIMG8_CCP1 (PF=4)
 *   Motor4  PB20  PINCM48  TIMA0_CCP1 (PF=7)
 *
 * One DMA channel per motor (4 total). Each timer publishes its zero-event
 * onto an event-fabric channel; the DMA subscribes and writes the next CC
 * value into the timer's CC register before the next bit period starts.
 *
 * Bit timing assumes MCLK = 16 MHz. See dshot.c for the period math.
 */

#ifndef DSHOT_H
#define DSHOT_H

#include <stdint.h>
#include <stdbool.h>

#define DSHOT_NUM_MOTORS  (4u)

typedef enum {
    DSHOT_MOTOR_1 = 0,
    DSHOT_MOTOR_2 = 1,
    DSHOT_MOTOR_3 = 2,
    DSHOT_MOTOR_4 = 3,
} dshot_motor_t;

/* Standard DShot special commands (subset). 0 = MOTOR_STOP. */
#define DSHOT_CMD_MOTOR_STOP        (0u)
#define DSHOT_CMD_BEEP1             (1u)
#define DSHOT_CMD_SPIN_DIRECTION_1  (7u)
#define DSHOT_CMD_SPIN_DIRECTION_2  (8u)
#define DSHOT_CMD_SAVE_SETTINGS     (12u)

/* Throttle range used by mixer once armed: 48..2047 (0..47 are commands). */
#define DSHOT_THROTTLE_MIN          (48u)
#define DSHOT_THROTTLE_MAX          (2047u)

/* Configure pins, timers and DMA. After init, motors hold low (idle). */
void dshot_init(void);

/* Stage one frame for one motor. value is the raw 11-bit DShot payload:
 * 0..47 are special commands, 48..2047 are throttle, anything > 2047 is
 * clamped. telem_request adds the telemetry-request bit. */
void dshot_set_value(dshot_motor_t m, uint16_t value, bool telem_request);

/* Convenience: stage MOTOR_STOP (cmd 0) on all four motors. */
void dshot_set_all_motor_stop(void);

/* Kick off transmission of the staged frames on all four motors.
 * Call from a periodic task at 1..8 kHz. Returns false if a previous frame
 * is still in flight (DMA not done yet) — in that case the caller should
 * skip and try again next tick.
 *
 * Internally this flips the four motor pins from GPIO-output-LOW back to
 * timer-driven before arming the DMA, so the ESC sees a rising edge at
 * frame start instead of staying HIGH the whole inter-frame gap. */
bool dshot_transmit(void);

/* Park the motor pins LOW between frames. Call from the scheduler tick
 * *after* dshot_transmit() — typically 100 µs later for the existing
 * 10 kHz / 1 kHz-frame layout. Switches each PINCM back to GPIO so the
 * line idles LOW for the remaining ~900 µs of inter-frame gap (the ESC's
 * frame-alignment cue). */
void dshot_park(void);

/* Diagnostics for SWD. Read these over the debugger to verify activity. */
extern volatile uint32_t dshot_frames_started;
extern volatile uint32_t dshot_frames_skipped;

#endif /* DSHOT_H */
