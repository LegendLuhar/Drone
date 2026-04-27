#ifndef drone_control_include
#define drone_control_include

#include <stdbool.h>
#include <stdint.h>
#include "leds.h"

#define NUM_MOTORS          4

#define TICK_PERIOD         320

#define CALIBRATION_CYCLES  32
#define AXIS_FILTER_COEFF   4
#define OUTPUT_BLEND_COEFF  3

#define HOVER_DUTY          64
#define TILT_SCALE          32
#define DEAD_BAND           2
#define DUTY_MIN            0
#define DUTY_MAX            255
#define FRAME_BRIGHTNESS    14

typedef enum {
    DRONE_CALIBRATING = 0,
    DRONE_RUNNING,
    DRONE_FAULT
} drone_phase_t;

typedef struct {
    drone_phase_t phase;
    int32_t filt_pitch;
    int32_t filt_roll;
    int32_t offset_pitch;
    int32_t offset_roll;
    int32_t cal_sum_pitch;
    int32_t cal_sum_roll;
    uint16_t cal_samples;
    uint8_t target_duty[NUM_MOTORS];
    int32_t current_duty[NUM_MOTORS];
    leds_message_t led_frame;
} drone_state_t;

drone_state_t DroneInitialize(void);
void DroneUpdate(drone_state_t *craft);

#endif /* drone_control_include */
