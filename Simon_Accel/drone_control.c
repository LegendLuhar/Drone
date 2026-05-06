#include "drone_control.h"
#include "accelerometer.h"
#include "colors.h"

/*
 * Quadcopter X-configuration motor layout (top-down view):
 *
 *     M0 [FL]         M1 [FR]
 *         \           /
 *          \  [PCB]  /
 *           ---------
 *          /         \
 *         /           \
 *     M2 [RL]         M3 [RR]
 *
 * Pitch axis (X): positive → nose dips, front motors compensate
 * Roll  axis (Y): positive → right dips, right motors compensate
 */

static const int8_t mix_pitch[NUM_MOTORS] = { -1, -1, +1, +1 };
static const int8_t mix_roll[NUM_MOTORS]  = { +1, -1, +1, -1 };

static int32_t BoundValue(int32_t v, int32_t floor, int32_t ceil)
{
    return (v < floor) ? floor : ((v > ceil) ? ceil : v);
}

drone_state_t DroneInitialize(void)
{
    drone_state_t craft;
    int m;

    craft.phase         = DRONE_CALIBRATING;
    craft.filt_pitch    = 0;
    craft.filt_roll     = 0;
    craft.offset_pitch  = 0;
    craft.offset_roll   = 0;
    craft.cal_sum_pitch = 0;
    craft.cal_sum_roll  = 0;
    craft.cal_samples   = 0;
    craft.led_frame     = leds_off;

    for (m = 0; m < NUM_MOTORS; m++) {
        craft.target_duty[m]  = 0;
        craft.current_duty[m] = 0;
    }

    return craft;
}

static void ComputeMotorMix(drone_state_t *craft, int32_t pitch_cmd, int32_t roll_cmd)
{
    int m;
    for (m = 0; m < NUM_MOTORS; m++) {
        int32_t thrust = (int32_t)HOVER_DUTY
                       + (pitch_cmd * mix_pitch[m])
                       + (roll_cmd  * mix_roll[m]);
        craft->target_duty[m] = (uint8_t)BoundValue(thrust, DUTY_MIN, DUTY_MAX);
    }
}

static void BlendOutputs(drone_state_t *craft)
{
    int m;
    for (m = 0; m < NUM_MOTORS; m++) {
        craft->current_duty[m] =
            ((craft->current_duty[m] * (OUTPUT_BLEND_COEFF - 1))
             + (int32_t)craft->target_duty[m])
            / OUTPUT_BLEND_COEFF;
    }
}

static void RenderMotors(drone_state_t *craft)
{
    /* Motor FL/FR (indices 0/1) are swapped vs physical SW1/SW2 on the APA102 chain. */
    int32_t duty_for_led[4];

    duty_for_led[0] = craft->current_duty[1];
    duty_for_led[1] = craft->current_duty[0];
    duty_for_led[2] = craft->current_duty[2];
    duty_for_led[3] = craft->current_duty[3];

    BuildDroneFrame(&craft->led_frame, duty_for_led, FRAME_BRIGHTNESS);
}

static void RenderFault(drone_state_t *craft)
{
    uint8_t status = AccelerometerGetDebugStatus();
    uint8_t r = 0x60, g = 0x00, b = 0x00;

    if (status == ACCEL_DEBUG_WHOAMI_FAIL) {
        r = 0xFF; g = 0x00; b = 0x00;
    } else if (status == ACCEL_DEBUG_CONFIG_WRITE_FAIL) {
        r = 0xFF; g = 0x60; b = 0x00;
    } else if (status == ACCEL_DEBUG_CONFIG_READ_FAIL) {
        r = 0x00; g = 0xFF; b = 0x00;
    } else if (status == ACCEL_DEBUG_SAMPLE_READ_FAIL) {
        r = 0x00; g = 0x00; b = 0xFF;
    }

    BuildUniformFrame(&craft->led_frame, r, g, b, FRAME_BRIGHTNESS);
}

void DroneUpdate(drone_state_t *craft)
{
    accelerometer_sample_t reading;
    int32_t pitch_cmd;
    int32_t roll_cmd;
    int m;

    if (!AccelerometerAvailable()) {
        InitializeAccelerometer();
    }

    if (!AccelerometerReadSample(&reading)) {
        craft->phase = DRONE_FAULT;
        RenderFault(craft);
        return;
    }

    if (craft->phase == DRONE_FAULT) {
        craft->phase     = DRONE_CALIBRATING;
        craft->cal_samples   = 0;
        craft->cal_sum_pitch = 0;
        craft->cal_sum_roll  = 0;
    }

    craft->filt_pitch =
        ((craft->filt_pitch * (AXIS_FILTER_COEFF - 1)) + (int32_t)reading.x)
        / AXIS_FILTER_COEFF;
    craft->filt_roll =
        ((craft->filt_roll * (AXIS_FILTER_COEFF - 1)) + (int32_t)reading.y)
        / AXIS_FILTER_COEFF;

    if (craft->phase == DRONE_CALIBRATING) {
        craft->cal_sum_pitch += craft->filt_pitch;
        craft->cal_sum_roll  += craft->filt_roll;
        craft->cal_samples++;

        if (craft->cal_samples >= CALIBRATION_CYCLES) {
            craft->offset_pitch = craft->cal_sum_pitch / (int32_t)craft->cal_samples;
            craft->offset_roll  = craft->cal_sum_roll  / (int32_t)craft->cal_samples;
            craft->phase = DRONE_RUNNING;

            for (m = 0; m < NUM_MOTORS; m++) {
                craft->current_duty[m] = HOVER_DUTY;
            }
        }

        for (m = 0; m < NUM_MOTORS; m++) {
            craft->target_duty[m]  = 28;
            craft->current_duty[m] = 28;
        }
        RenderMotors(craft);
        return;
    }

    pitch_cmd = (craft->filt_pitch - craft->offset_pitch) / TILT_SCALE;
    roll_cmd  = (craft->filt_roll  - craft->offset_roll)  / TILT_SCALE;

    if ((pitch_cmd > -DEAD_BAND) && (pitch_cmd < DEAD_BAND)) {
        pitch_cmd = 0;
    }
    if ((roll_cmd > -DEAD_BAND) && (roll_cmd < DEAD_BAND)) {
        roll_cmd = 0;
    }

    ComputeMotorMix(craft, pitch_cmd, roll_cmd);
    BlendOutputs(craft);
    RenderMotors(craft);
}
