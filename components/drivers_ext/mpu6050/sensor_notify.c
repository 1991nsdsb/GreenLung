#include "app_timer.h"
#include "ble_nus.h"
#include "mpu6050.h"


extern ble_nus_t                        m_nus;                                      /**< Structure to identify the Nordic UART Service. */
extern uint8_t accel[];
extern uint8_t gyro[];




static app_timer_id_t accel_gyro_timer;
static app_timer_id_t tempe_press_light_timer;


static void mpu6050_timeout_handler(){

	get_mpu6050data();
	ble_nus_string_send(&m_nus, accel, 6, m_nus.accel_handles.value_handle);
	ble_nus_string_send(&m_nus, gyro, 6, m_nus.gyro_handles.value_handle);
}

static void bmp180_timeout_handler(){
	static uint8_t tempe[2];
	static uint8_t press[4];
	long true_tempe,true_press;


	tempe[0] = (true_tempe >> 8)&0xff;
	tempe[1] = (true_tempe &0xff);
	
	press[0] = (true_press >> 24)&0xff;
	press[1] = (true_press >> 16)&0xff;
	press[2] = (true_press >> 8)&0xff;
	press[3] = (true_press >> 0)&0xff;
	ble_nus_string_send(&m_nus, tempe, 2, m_nus.tempe_handles.value_handle);
	ble_nus_string_send(&m_nus, press, 4, m_nus.press_handles.value_handle);
	
}

static void ap3216c_timeout_handler(){
	uint16_t temp;
	static uint8_t al_data[2];

	al_data[0] = (temp>>8)&0xff;
	al_data[1] = temp&0xff;
	ble_nus_string_send(&m_nus, al_data, 2, m_nus.light_handles.value_handle);
}


void accel_gyro_timeout_handler(void *p_context){
	mpu6050_timeout_handler();
}

void tempe_press_light_timeout_handler(void *p_context){
	bmp180_timeout_handler();
	ap3216c_timeout_handler();
}

uint32_t create_sensor_timer(void){
	uint32_t err_code;
	err_code = app_timer_create(&accel_gyro_timer, APP_TIMER_MODE_REPEATED,accel_gyro_timeout_handler);
	err_code |= app_timer_create(&tempe_press_light_timer, APP_TIMER_MODE_REPEATED,tempe_press_light_timeout_handler);
	
	return err_code;
}


uint32_t accel_gyro_timer_start(void){
	return app_timer_start(accel_gyro_timer, APP_TIMER_TICKS(150, 0), NULL);
}
uint32_t tempe_press_light_timer_start(void){
	return app_timer_start(tempe_press_light_timer, APP_TIMER_TICKS(1500, 0), NULL);
}

void sensor_timer_start(void){
	tempe_press_light_timer_start();
	accel_gyro_timer_start();
}

uint32_t close_all_sensor_timer(void){
	uint32_t err;
	err = app_timer_stop_all();
	
	return err;
}

