/*
 * imu.h — ICM-42688-P 6-axis IMU driver, SPI1, polled.
 *
 * Pin / mux assignment (project_drone_hardware.md):
 *   PB6  PINCM23  SPI1_CS0   (PF=3)  — driven as GPIO output, manual CS
 *   PB8  PINCM25  SPI1_PICO  (PF=3)
 *   PB14 PINCM31  SPI1_POCI  (PF=3)
 *   PB16 PINCM33  SPI1_SCLK  (PF=3)
 *
 * SPI1 isn't used by the DShot path (which lives on TIMA0/TIMA1/TIMG8 + DMA),
 * and the IMU pins (PB6/8/14/16) are disjoint from the motor pins
 * (PB17..PB20), so this driver coexists with the DShot stack on the same
 * GPIOB block — provided the GPIOB power-up is idempotent (it is).
 *
 * INT1 is no-connect and INT2 is tied to GND on this board, so there is no
 * data-ready IRQ available. Samples are polled from the scheduler tick.
 *
 * Bring-up milestone: imu_who_am_i_seen == 0x47 over SWD.
 */

#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    int16_t temp;
} imu_sample_t;

/* Configure SPI1 + CS pin, soft-reset the chip, verify WHO_AM_I, enable
 * gyro+accel in low-noise mode, set ODRs. Safe to call after dshot_init() and
 * after __enable_irq() — internal blocking delays are spinloops in thread
 * context and don't block the DShot ISR. Sets imu_ready on success. */
void imu_init(void);

/* Read the 14-byte temp+accel+gyro burst into *out. Returns false if the IMU
 * never initialized, or *out is NULL, or the SPI transfer aborted early.
 * ~30 µs at 4 MHz SPI bit clock, runs in thread context. */
bool imu_read_sample(imu_sample_t *out);

/* Diagnostic counters and last-seen state. All volatile so a SWD watch
 * window can see them updating live. */
extern volatile bool      imu_ready;            /* WHO_AM_I + config OK */
extern volatile uint8_t   imu_who_am_i_seen;    /* last raw WHO_AM_I byte */
extern volatile uint8_t   imu_init_status;      /* see imu_init_status_t */
extern volatile uint32_t  imu_sample_count;     /* successful reads */
extern volatile uint32_t  imu_read_failures;    /* failed reads */
extern volatile imu_sample_t imu_latest_sample; /* most recent good sample */

typedef enum {
    IMU_STATUS_BOOT             = 0,
    IMU_STATUS_WHOAMI_FAIL      = 1,
    IMU_STATUS_CONFIG_WRITE_FAIL= 2,
    IMU_STATUS_OK               = 3,
} imu_init_status_t;

#endif /* IMU_H */
