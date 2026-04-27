#include "colors.h"
#include "leds.h"

const leds_message_t leds_off = {
    .start = {0x0000, 0x0000},
    .led = {
        {.brightness = 0, ._header = 7},
        {.brightness = 0, ._header = 7},
        {.brightness = 0, ._header = 7},
        {.brightness = 0, ._header = 7},
    },
    .end = {0xFFFF, 0xFFFF},
};

void BuildDroneFrame(
    leds_message_t *frame,
    const int32_t *duty,
    uint8_t brightness)
{
    int i;

    frame->start[0] = 0x0000;
    frame->start[1] = 0x0000;

    for (i = 0; i < 4; i++) {
        int32_t val = duty[i];
        uint8_t level = (uint8_t)((val < 0) ? 0 : ((val > 255) ? 255 : val));

        frame->led[i]._header    = 7;
        frame->led[i].brightness = brightness;
        frame->led[i].red        = level;
        frame->led[i].green      = level;
        frame->led[i].blue       = level;
    }

    frame->end[0] = 0xFFFF;
    frame->end[1] = 0xFFFF;
}

void BuildUniformFrame(
    leds_message_t *frame,
    uint8_t red,
    uint8_t green,
    uint8_t blue,
    uint8_t brightness)
{
    int i;

    frame->start[0] = 0x0000;
    frame->start[1] = 0x0000;

    for (i = 0; i < 4; i++) {
        frame->led[i]._header    = 7;
        frame->led[i].brightness = brightness;
        frame->led[i].red        = red;
        frame->led[i].green      = green;
        frame->led[i].blue       = blue;
    }

    frame->end[0] = 0xFFFF;
    frame->end[1] = 0xFFFF;
}
