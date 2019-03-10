#ifndef SENSOR_NOTIFY_H_
#define SENSOR_NOTIFY_H_

#include <stdint.h>

uint32_t create_sensor_timer(void);
uint32_t accel_gyro_timer_start(void);
uint32_t tempe_press_light_timer_start(void);
uint32_t close_all_sensor_timer(void);

void sensor_timer_start(void);

#endif
