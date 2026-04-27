
/*
 * Drone PWM Visualization — Accelerometer-driven motor brightness
 *
 * Reads tilt from LIS2HH12 accelerometer and maps pitch/roll
 * to four independent LED brightness levels representing
 * quadcopter motor thrust compensation.
 */

#include <ti/devices/msp/msp.h>
#include "delay.h"
#include "buttons.h"
#include "timing.h"
#include "leds.h"
#include "accelerometer.h"
#include "drone_control.h"

int spi_frame_words = sizeof(leds_message_t) / sizeof(uint16_t);

int main(void)
{
    InitializeButtonGPIO();
    InitializeLEDInterface();
    InitializeAccelerometer();
    InitializeTimerG0();

    drone_state_t craft = DroneInitialize();

    SetTimerG0Delay(TICK_PERIOD);
    EnableTimerG0();

    while (1) {
        if (timer_wakeup) {
            timer_wakeup = false;

            DroneUpdate(&craft);

            while (!SendSPIMessage((uint16_t *)&craft.led_frame, spi_frame_words)) {
            }
        }

        __WFI();
    }
}
