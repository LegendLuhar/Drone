#ifndef colors_include
#define colors_include

#include <stdint.h>
#include "leds.h"

extern const leds_message_t leds_off;

void BuildDroneFrame(
    leds_message_t *frame,
    const int32_t *duty,
    uint8_t brightness);

void BuildUniformFrame(
    leds_message_t *frame,
    uint8_t red,
    uint8_t green,
    uint8_t blue,
    uint8_t brightness);

#endif /* colors_include */
