#ifndef accelerometer_include
#define accelerometer_include

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} accelerometer_sample_t;

// accel debug states used by accel mode error leds
#define ACCEL_DEBUG_OK 0
#define ACCEL_DEBUG_INIT_START 1
#define ACCEL_DEBUG_WHOAMI_FAIL 2
#define ACCEL_DEBUG_CONFIG_WRITE_FAIL 3
#define ACCEL_DEBUG_CONFIG_READ_FAIL 4
#define ACCEL_DEBUG_SAMPLE_READ_FAIL 5

// returns true only after who_am_i and cfg steps pass
void InitializeAccelerometer(void);
bool AccelerometerAvailable(void);
bool AccelerometerReadSample(accelerometer_sample_t *sample);
uint8_t AccelerometerGetDebugStatus(void);
uint8_t AccelerometerGetLastWhoAmI(void);

#endif // accelerometer_include
