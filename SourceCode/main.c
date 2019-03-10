/**
 * Copyright (c) 2014 - 2017, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
/** @example examples/ble_peripheral/ble_app_hrs/main.c
 *
 * @brief Heart Rate Service Sample Application main file.
 *
 * This file contains the source code for a sample application using the Heart Rate service
 * (and also Battery and Device Information services). This application uses the
 * @ref srvlib_conn_params module.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_bas.h"
#include "ble_hrs.h"
#include "ble_dis.h"
#include "ble_conn_params.h"
#include "boards.h"
//#include "sensorsim.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "bsp.h"
#include "nrf_delay.h"
#include "bsp_btn_ble.h"
#include "peer_manager.h"
#include "fds.h"
#include "fstorage.h"
#include "nrf_ble_gatt.h"
#include "ble_conn_state.h"
#include "SDP3x.h"
#include "mpu_reg.h"
#include "twi_master.h"
#include "bsp_config.h"
#include "m_batt_meas.h"
#include "ble_dfu.h"
#include "app_pwm.h"
#include "bma2x2.h"
#include "nrf_drv_spi.h"
#include "w25x40.h"
#include "nrf_drv_rtc.h"

#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#define IS_SRVC_CHANGED_CHARACT_PRESENT  1                                           /**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/

#define DEVICE_NAME                      "GreenLung"                     	           /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME                "Elecomed"            							           /**< Manufacturer. Will be passed to Device Information Service. */
#define HARDWARE_VER										 "V1.0.2"
#define	FIRMWARE_VER										 "V1.0.9"
#define	MODE_NUMBER											 "PVT"
#define	SERIAL_NUMBER										 "12345678"
#define APP_ADV_INTERVAL                 300                                         /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS       60                                         /**< The advertising timeout in units of seconds. */

#define APP_TIMER_PRESCALER              0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE          4                                           /**< Size of timer operation queues. */

#define BATTERY_LEVEL_MEAS_INTERVAL      APP_TIMER_TICKS(2000, APP_TIMER_PRESCALER)  /**< Battery level measurement interval (ticks). */

#define HEART_RATE_MEAS_INTERVAL         APP_TIMER_TICKS(75, APP_TIMER_PRESCALER)  /**< Heart rate measurement interval (ticks). */
#define HEART_RATE_OFF_LINE_MEAS_INTERVAL         APP_TIMER_TICKS(500, APP_TIMER_PRESCALER)  /**< Heart rateoffline measurement interval (ticks). */

#define BUZZER_INTERVAL             		APP_TIMER_TICKS(200, APP_TIMER_PRESCALER)   /**< RR interval interval (ticks). */
//#define MIN_RR_INTERVAL                  100                                         /**< Minimum RR interval as returned by the simulated measurement function. */
//#define MAX_RR_INTERVAL                  500                                         /**< Maximum RR interval as returned by the simulated measurement function. */
//#define RR_INTERVAL_INCREMENT            1                                           /**< Value by which the RR interval is incremented/decremented for each call to the simulated measurement function. */

#define SENSOR_CONTACT_DETECTED_INTERVAL APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Sensor Contact Detected toggle interval (ticks). */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */

#define FIRST_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY    APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT     3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                   1                                           /**< Perform bonding. */
#define SEC_PARAM_MITM                   0                                           /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                   1                                           /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS               0                                           /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES        BLE_GAP_IO_CAPS_NONE                        /**< No I/O capabilities. */
#define SEC_PARAM_OOB                    0                                           /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE           7                                           /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE           16                                          /**< Maximum encryption key size. */

#define DEAD_BEEF                        0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define APP_FEATURE_NOT_SUPPORTED        BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2        /**< Reply when unsupported features are requested. */

#define G_SENSOR_X_LEVEL_L										-6000																		/**<1G is 16384. */
#define G_SENSOR_X_LEVEL_H										-2000																		/**<1G is 16384. */
#define G_SENSOR_Y_LEVEL_L										-1000																		/**<1G is 16384. */
#define G_SENSOR_Y_LEVEL_H										6000																		/**<1G is 16384. */
#define G_SENSOR_Z_LEVEL_L										14500																		/**<1G is 16384. */
#define G_SENSOR_Z_LEVEL_H										16500																		/**<1G is 16384. */
#define STABLE_COUNT_TIME_UP									5																				/**每次的间隔是HEART_RATE_MEAS_INTERVAL */
#define STABLE_COUNT_TIME											10																			/**每次的间隔是HEART_RATE_MEAS_INTERVAL */
#define P_SENSOR_THRESHOLD										-200																			/**超过这个数值才被认为是人吸的，否则算noise */

//#define USE_BPM180
//#define USE_MPU6050
#define USE_SDP3x
#define USE_BMA253
uint8_t data_test[16] = {0x02,0xfc,0x18,0x10,0x10,0x10,0x10,0x10,0x10,0x11,0x11,0x11,0x11,0x00,0x01,0xee};

APP_PWM_INSTANCE(PWM1,1);                   // Create the instance "PWM1" using TIMER1.
const nrf_drv_rtc_t rtc = NRF_DRV_RTC_INSTANCE(0);  /**< Declaring an instance of nrf_drv_rtc for RTC2. */
static uint16_t  m_conn_handle = BLE_CONN_HANDLE_INVALID; /**< Handle of the current connection. */
static ble_bas_t m_bas;                                   /**< Structure used to identify the battery service. */
static ble_hrs_t m_hrs;                                   /**< Structure used to identify the heart rate service. */
//static bool      m_rr_interval_enabled = true;            /**< Flag for enabling and disabling the registration of new RR interval measurements (the purpose of disabling this is just to test sending HRM without RR interval data. */

static nrf_ble_gatt_t m_gatt;                             /**< Structure for gatt module*/

//static sensorsim_cfg_t   m_rr_interval_sim_cfg;           /**< RR Interval sensor simulator configuration. */
//static sensorsim_state_t m_rr_interval_sim_state;         /**< RR Interval sensor simulator state. */
#define CHARGE_TIMEOUT_MS                   APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER)

APP_TIMER_DEF(m_battery_timer_id);                        /**< Battery timer. */
APP_TIMER_DEF(m_heart_rate_timer_id);                     /**< Heart rate measurement timer. */
APP_TIMER_DEF(m_buzzer_timer_id);                    			/**< control buzzer. */
APP_TIMER_DEF(m_sensor_contact_timer_id);                 /**< Sensor contact detected timer. */
APP_TIMER_DEF(m_charge_timer_id);                         /**charge timer **/

#define NUM_OF_STORAGE_DATA                              9
//static bool		      charging_flag = false;
static bool         red_led_convert_flag = true;
bool 								rtc_timer_convert = true;    //true 500ms;flase 75ms
uint8_t             m_adv_mode = BLE_ADV_EVT_IDLE;

static bool					m_is_horizontal = false;
//static bool					m_is_Psensor_sleep = false;
static bool					m_is_Buzzer_can_on = true;
uint8_t							m_buzzer_count_expect = 0; 
uint8_t							m_LED_Color = 0; 
static bool					m_buzzer_busy = false;
static bool					m_BLE_Connect_Status = false;
static							ble_dfu_t m_dfus;                                                            /**< Structure used to identify the DFU service. */
nrf_drv_spi_t m_spi_master_0 = NRF_DRV_SPI_INSTANCE(0);

unsigned int flash_storage_addr;
unsigned int flash_sector_addr;

void pwm_ready_callback(uint32_t pwm_id)    // PWM callback function
{
	UNUSED_PARAMETER(pwm_id);
    //NRF_LOG_INFO("pwm_ready_callback\r\n");
}

static void ble_dfu_evt_handler(ble_dfu_t * p_dfu, ble_dfu_evt_t * p_evt)
{
    switch (p_evt->type)
    {
        case BLE_DFU_EVT_INDICATION_DISABLED:
            NRF_LOG_INFO("Indication for BLE_DFU is disabled\r\n");
            break;

        case BLE_DFU_EVT_INDICATION_ENABLED:
            NRF_LOG_INFO("Indication for BLE_DFU is enabled\r\n");
            break;

        case BLE_DFU_EVT_ENTERING_BOOTLOADER:
            NRF_LOG_INFO("Device is entering bootloader mode!\r\n");
            break;
        default:
            NRF_LOG_INFO("Unknown event from ble_dfu\r\n");
            break;
    }
}

static ble_uuid_t m_adv_uuids[] = {{BLE_UUID_HEART_RATE_SERVICE, BLE_UUID_TYPE_BLE},
                                   {BLE_UUID_BATTERY_SERVICE, BLE_UUID_TYPE_BLE},
                                   {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}}; /**< Universally unique service identifiers. */

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for starting advertising.
 */
void advertising_start(void)
{
    ret_code_t err_code;

    err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling File Data Storage events.
 *
 * @param[in] p_evt  Peer Manager event.
 * @param[in] cmd
 */
static void fds_evt_handler(fds_evt_t const * const p_evt)
{
    if (p_evt->id == FDS_EVT_GC)
    {
        NRF_LOG_DEBUG("GC completed\n");
    }
}


/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    ret_code_t err_code;

    switch (p_evt->evt_id)
    {
        case PM_EVT_BONDED_PEER_CONNECTED:
        {
            NRF_LOG_INFO("Connected to a previously bonded device.\r\n");
        } break;

        case PM_EVT_CONN_SEC_SUCCEEDED:
        {
            NRF_LOG_INFO("Connection secured. Role: %d. conn_handle: %d, Procedure: %d\r\n",
                         ble_conn_state_role(p_evt->conn_handle),
                         p_evt->conn_handle,
                         p_evt->params.conn_sec_succeeded.procedure);
        } break;

        case PM_EVT_CONN_SEC_FAILED:
        {
            /* Often, when securing fails, it shouldn't be restarted, for security reasons.
             * Other times, it can be restarted directly.
             * Sometimes it can be restarted, but only after changing some Security Parameters.
             * Sometimes, it cannot be restarted until the link is disconnected and reconnected.
             * Sometimes it is impossible, to secure the link, or the peer device does not support it.
             * How to handle this error is highly application dependent. */
        } break;

        case PM_EVT_CONN_SEC_CONFIG_REQ:
        {
            // Reject pairing request from an already bonded peer.
            pm_conn_sec_config_t conn_sec_config = {.allow_repairing = false};
            pm_conn_sec_config_reply(p_evt->conn_handle, &conn_sec_config);
        } break;

        case PM_EVT_STORAGE_FULL:
        {
            // Run garbage collection on the flash.
            err_code = fds_gc();
            if (err_code == FDS_ERR_BUSY || err_code == FDS_ERR_NO_SPACE_IN_QUEUES)
            {
                // Retry.
            }
            else
            {
                APP_ERROR_CHECK(err_code);
            }
        } break;

        case PM_EVT_PEERS_DELETE_SUCCEEDED:
        {
            advertising_start();
        } break;

        case PM_EVT_LOCAL_DB_CACHE_APPLY_FAILED:
        {
            // The local database has likely changed, send service changed indications.
            pm_local_database_has_changed();
        } break;

        case PM_EVT_PEER_DATA_UPDATE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peer_data_update_failed.error);
        } break;

        case PM_EVT_PEER_DELETE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peer_delete_failed.error);
        } break;

        case PM_EVT_PEERS_DELETE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peers_delete_failed_evt.error);
        } break;

        case PM_EVT_ERROR_UNEXPECTED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.error_unexpected.error);
        } break;

        case PM_EVT_CONN_SEC_START:
        case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:
        case PM_EVT_PEER_DELETE_SUCCEEDED:
        case PM_EVT_LOCAL_DB_CACHE_APPLIED:
        case PM_EVT_SERVICE_CHANGED_IND_SENT:
        case PM_EVT_SERVICE_CHANGED_IND_CONFIRMED:
        default:
            break;
    }
}

/*PWM Module init*/
void PWM_init(void)
{
		ret_code_t err_code; 
    /* 2-channel PWM, 4000Hz(), output on LED pins. */
    app_pwm_config_t pwm1_cfg = APP_PWM_DEFAULT_CONFIG_1CH(250L, BUZZER_PWM);

    /* Switch the polarity of the second channel. */
    pwm1_cfg.pin_polarity[1] = APP_PWM_POLARITY_ACTIVE_HIGH;

    /* Initialize and enable PWM. */
    err_code = app_pwm_init(&PWM1,&pwm1_cfg,pwm_ready_callback);
    APP_ERROR_CHECK(err_code);

}

/*PWM Module on*/
void PWM_on(void)
{
		ret_code_t err_code; 
//	NRF_LOG_INFO("Mute swich is %d \r\n",bsp_button_is_pressed(MUTE_SWITCH));
	if(bsp_button_is_pressed(MUTE_SWITCH)|| (m_is_Buzzer_can_on == false))
	{
		app_pwm_disable(&PWM1);
		nrf_gpio_pin_clear(BUZZER_PWM);
	}
	else
	{
		app_pwm_enable(&PWM1);
		NRF_LOG_INFO("PWM on\r\n");
		err_code = app_timer_stop(m_heart_rate_timer_id);
		APP_ERROR_CHECK(err_code);
		err_code = app_pwm_channel_duty_set(&PWM1, 0,20);
		APP_ERROR_CHECK(err_code);
	}


}

/*PWM Module off*/
void PWM_off(void)
{
	  app_pwm_disable(&PWM1);
		nrf_gpio_pin_clear(BUZZER_PWM);
		NRF_LOG_INFO("PWM off\r\n");

}

void buzzer_on(uint8_t expect_count,uint8_t led_color)
{
	  uint32_t err_code;
	  NRF_LOG_INFO("buzzer_on\r\n");
//	
//		if(!bsp_button_is_pressed(MUTE_SWITCH)&&(m_is_Buzzer_can_on == true))
//		{
			m_buzzer_count_expect = expect_count;
			m_buzzer_busy = true;
			m_LED_Color = led_color;
			err_code = app_timer_start(m_buzzer_timer_id, BUZZER_INTERVAL, NULL);
			APP_ERROR_CHECK(err_code);
//		}

}


/**@brief Function for performing battery measurement and updating the Battery Level characteristic
 *        in Battery Service.
 */
static void battery_level_update(void)
{
    uint32_t err_code;
    uint8_t  battery_level;
	
    battery_level = battery_level_read();
//		if(battery_level >= 100 && (!nrf_gpio_pin_read(INIO_1)))  //充满亮绿 灯
//		{
//				bsp_board_led_off(BSP_LED_RED);
//				bsp_board_led_on(BSP_LED_GREEN);
//		}
//		
//		if(nrf_gpio_pin_read(INIO_1))
//		{
//				bsp_board_led_off(BSP_LED_GREEN);
//		}
		
    err_code = ble_bas_battery_level_update(&m_bas, battery_level);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != BLE_ERROR_NO_TX_PACKETS) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
       )
    {
        APP_ERROR_HANDLER(err_code);
    }
}


/**@brief Function for handling the Battery measurement timer timeout.
 *
 * @details This function will be called each time the battery level measurement timer expires.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
static void battery_level_meas_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    battery_level_update();
}

/**@brief The function of judging that the sensor data reaches the horizontal attitude.
 *
 * @details 
 *          
 *
 * @param[in] 
 *      
 */
static bool GSensorDatajudgment(uint8_t * AccSensorData, uint16_t len)
{
    int16_t 		acc_data_x,acc_data_y,acc_data_z;
		static uint16_t	Stable_count_p = 0;
		static uint16_t	Stable_count_n = 0;
		uint16_t        data_tmp_U = 0;

		if(len<6)
		{
			NRF_LOG_INFO("Accel data length too short!\r\n");
			return false;
		}
		data_tmp_U = AccSensorData[5]<<8 | AccSensorData[4];
		acc_data_z = ~(data_tmp_U-1);
		data_tmp_U = AccSensorData[3]<<8 | AccSensorData[2];
		acc_data_y = ~(data_tmp_U-1);
		data_tmp_U = AccSensorData[1]<<8 | AccSensorData[0];
		acc_data_x = ~(data_tmp_U-1);
//		NRF_LOG_INFO("bma253_acceldata x is %d y is %d z is %d\r\n",acc_data_x,acc_data_y,acc_data_z);
//		NRF_LOG_INFO("m_buzzer_busy is %d\r\n",m_buzzer_busy);
//		NRF_LOG_INFO("judgment count P %d, N is %d!\r\n",Stable_count_p,Stable_count_n);
//		NRF_LOG_INFO("m_is_horizontal is %d!\r\n",m_is_horizontal);
		if((acc_data_x > G_SENSOR_X_LEVEL_L)&&(acc_data_x < G_SENSOR_X_LEVEL_H)&&
				(acc_data_y > G_SENSOR_Y_LEVEL_L)&&(acc_data_y < G_SENSOR_Y_LEVEL_H)&&
				(acc_data_z > G_SENSOR_Z_LEVEL_L)&&(acc_data_z < G_SENSOR_Z_LEVEL_H))
		{
			//bsp_board_led_on(BSP_LED_GREEN);
			Stable_count_n = 0;
			if(Stable_count_p <= STABLE_COUNT_TIME_UP)
			{
				Stable_count_p++;
			}
			
			if((m_is_horizontal == false) && (!m_buzzer_busy) && (Stable_count_p >STABLE_COUNT_TIME_UP))
			{
				NRF_LOG_INFO("Accel data ok!\r\n");
				m_is_horizontal = true;
				Stable_count_p = 0;
				buzzer_on(1,0);
				return true;
			}
			
		}
		else
		{
			//bsp_board_led_off(BSP_LED_GREEN);
			Stable_count_p = 0;
			if(Stable_count_n <= STABLE_COUNT_TIME)
				{
					Stable_count_n++;
				}
			if((m_is_horizontal == true) && (!m_buzzer_busy) && (Stable_count_n >STABLE_COUNT_TIME))
			{
				m_is_horizontal = false;
				Stable_count_n = 0;
				NRF_LOG_INFO("Accel data NG!\r\n");
				buzzer_on(2,1);
			}
			//NRF_LOG_INFO("sensor data NG!\r\n");
		}
    return false;
}
/**@brief Function for handling the Heart rate measurement timer timeout.
 *
 * @details This function will be called each time the heart rate measurement timer expires.
 *          It will exclude RR Interval data from every third measurement.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
//		NRF_LOG_INFO("Disconnected, reason %d\r\n",
//							err_code);

static void heart_rate_meas_timeout_handler(void * p_context)
{
    static uint32_t cnt = 0;
    uint32_t        err_code;
		uint8_t 				acceldata[14];
		int16_t 				PressureData = 0;
		int16_t 				PressureData_Mcu = 0;
		uint8_t					SDP3xpressureData[30];
//		uint8_t         storage_data[16];
		uint32_t 				ticks = 0;
		static uint32_t last_ticks = 0;
		static uint32_t total_time = 0;
		static uint32_t measure_count;
		uint32_t temp_time = 0;
	
    UNUSED_PARAMETER(p_context);


#ifdef USE_SDP3x
//		if(m_Psensor_start)m_is_horizontal
//		if(m_is_horizontal)
//		{
			memset(SDP3xpressureData,0x00,30);
		
			err_code = SDP3x_data_read(SDP3xpressureData,3);
			if(err_code == 0)
			{

				bsp_board_led_on(BSP_LED_ALERT);
			}
			else
			{
				bsp_board_led_off(BSP_LED_ALERT);
			}

			PressureData = (SDP3xpressureData[1]<<8|SDP3xpressureData[0]);     //big
			PressureData_Mcu = (SDP3xpressureData[0]<<8|SDP3xpressureData[1]); //little 
//			m_is_Psensor_sleep = false;
			if(PressureData_Mcu < P_SENSOR_THRESHOLD)
			{
					
				NRF_LOG_INFO("OK \r\n");
//				err_code = app_timer_stop(m_heart_rate_timer_id);
//				APP_ERROR_CHECK(err_code);
//				err_code = app_timer_start(m_heart_rate_timer_id, HEART_RATE_MEAS_INTERVAL, NULL);//less -200，75ms测量
//				APP_ERROR_CHECK(err_code);
//				m_is_Buzzer_can_on = false;
			}
			else                        /*greater -200, change timer internal time*/
			{
				m_is_Buzzer_can_on = true;
//				if(m_BLE_Connect_Status == false)
//				{
						
//				}
//				NRF_LOG_INFO("NG \r\n");
			}
	//		NRF_LOG_INFO("PressureData is %d\r\n",PressureData);
//		}
//		else if(m_is_Psensor_sleep == false)
//		{
//			
//			if(SDP3x_Enter_Sleep())
//			{
//				m_is_Psensor_sleep = true;
//				NRF_LOG_INFO("SDP3x sleep.\r\n");
//			}
//		}

#endif
			
#ifdef USE_BMA253

		err_code = BMA253_register_read(BMA2x2_X_AXIS_LSB_ADDR,acceldata,6);
		if(err_code == 0)
		{
			NRF_LOG_INFO("BMA253 Read data timeout!\r\n");
			BMA253_init();
		}

		if(!m_buzzer_busy)
		{
			GSensorDatajudgment(acceldata,6);
		}
#endif
		
		measure_count++;
		ticks = nrf_drv_rtc_counter_get(&rtc);
//		if(measure_count == 1)
//		{
//				last_ticks = ticks;
//				total_time = 0;
//		}
//		else
//		{
//				temp_time = (ticks - last_ticks)*1000/40;
//				total_time += temp_time;
//		}
		total_time = ticks * 1000 / 40;
		acceldata[6] = total_time & 0xff;
		acceldata[7] = (total_time >> 8) & 0xff;
		acceldata[8] = (total_time >> 16) & 0xff;
		acceldata[9] = (total_time >> 24) & 0xff;
		acceldata[10] = (measure_count) & 0xff;
		acceldata[11] = (measure_count >> 8) & 0xff;
		acceldata[12] = 0xee;   //end, not tranfer  
//		if(!m_BLE_Connect_Status)   //离线
//		{
//				
//		}
		
		//heart_rate = (uint16_t)sensorsim_measure(&m_heart_rate_sim_state, &m_heart_rate_sim_cfg);
		//NRF_LOG_INFO("heart rate meas!\r\n");
    cnt++;
    err_code = ble_hrs_heart_rate_measurement_send(&m_hrs,PressureData,acceldata,13);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != BLE_ERROR_NO_TX_PACKETS) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING))
    {
			NRF_LOG_INFO("BLE Data send error.\r\n");
      APP_ERROR_HANDLER(err_code);
    }

    // Disable RR Interval recording every third heart rate measurement.
    // NOTE: An application will normally not do this. It is done here just for testing generation
    // of messages without RR Interval measurements.
//    m_rr_interval_enabled = ((cnt % 3) != 0);
}


/**@brief Function for handling the RR interval timer timeout.
 *
 * @details This function will be called each time the RR interval timer expires.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
static void buzzer_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
		static uint8_t buzzer_count_current = 0;
		static bool buzzer_on_state = true;
		uint32_t        err_code;
//		NRF_LOG_INFO("buzzer_timeout count %d!\r\n",buzzer_count_current);
		if(buzzer_count_current < (m_buzzer_count_expect*2))
		{
			buzzer_count_current++;
			if(buzzer_on_state)
			{
				PWM_off();
				if(m_LED_Color == 0){
					bsp_board_led_on(BSP_LED_GREEN);
					NRF_LOG_INFO("GREEN LED on!\r\n");
				}
				else
				{
					bsp_board_led_on(BSP_LED_RED);
					NRF_LOG_INFO("RED LED on!\r\n");
				}
				buzzer_on_state = false;
			}
			else
			{
				buzzer_on_state = true;
				if(m_LED_Color == 0){
					bsp_board_led_off(BSP_LED_GREEN);
				}
				else
				{
					bsp_board_led_off(BSP_LED_RED);
				}

				PWM_on();
			}
		}
		else
		{
			app_timer_stop(m_buzzer_timer_id);
			PWM_off();
			m_buzzer_busy = false;   //buzzer 结束后，其他程序中可以使用buzzer。
			buzzer_count_current = 0;
			buzzer_on_state = true;
			bsp_board_led_off(BSP_LED_GREEN);
			bsp_board_led_off(BSP_LED_RED);
//			if(m_BLE_Connect_Status) //连接不连接都开始测量。
//			{
				err_code = app_timer_start(m_heart_rate_timer_id, HEART_RATE_OFF_LINE_MEAS_INTERVAL, NULL); //一上电就测量。500ms间隔
				APP_ERROR_CHECK(err_code);
				
				NRF_LOG_INFO("heart_rate_timer on.\r\n");
//			}
		}
		
}


/**@brief Function for handling the Sensor Contact Detected timer timeout.
 *用于检测传感器连接超时
 * @details This function will be called each time the Sensor Contact Detected timer expires.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
static void sensor_contact_detected_timeout_handler(void * p_context)
{
    static bool sensor_contact_detected = false;

    UNUSED_PARAMETER(p_context);

    sensor_contact_detected = !sensor_contact_detected;
    ble_hrs_sensor_contact_detected_update(&m_hrs, sensor_contact_detected);
}

/**@brief Function for handling the Charging timer timeout.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
static void charge_timer_handler(void * p_context)
{
		uint32_t err_code;
//		red_led_convert_flag = true;
//		uint32_t err_code = app_timer_start(m_charge_timer_id, CHARGE_TIMEOUT_MS, NULL);
//		APP_ERROR_CHECK(err_code);
		if(!nrf_gpio_pin_read(INIO_1)) //charge_goog pin low -> charging
		{
					nrf_gpio_pin_toggle(BSP_LED_2);  //red led BSP_LED_2
		}
		else
		{
					nrf_gpio_pin_write(BSP_LED_2, 1);
		}

		//charging_flag = true;
}
/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    uint32_t err_code;

    // Initialize timer module.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);

    // Create timers.
    err_code = app_timer_create(&m_battery_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                battery_level_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_heart_rate_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                heart_rate_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_buzzer_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                buzzer_timeout_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_sensor_contact_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                sensor_contact_detected_timeout_handler);
    APP_ERROR_CHECK(err_code);
		
		err_code = app_timer_create(&m_charge_timer_id,
                                        APP_TIMER_MODE_REPEATED,
                                        charge_timer_handler);
		APP_ERROR_CHECK(err_code);
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_HEART_RATE_SENSOR_HEART_RATE_BELT);
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing services that will be used by the application.
 *
 * @details Initialize the Heart Rate, Battery and Device Information services.
 */
static void services_init(void)
{
    uint32_t       err_code;
    ble_hrs_init_t hrs_init;
    ble_bas_init_t bas_init;
    ble_dis_init_t dis_init;
		ble_dfu_init_t dfus_init;
    uint8_t        body_sensor_location;
		uint32_t iserial_number[2] = {0,0};
		iserial_number[1] = NRF_UICR->CUSTOMER[1];
		iserial_number[0]	= NRF_UICR->CUSTOMER[0];
		
    // Initialize the Device Firmware Update Service.
    memset(&dfus_init, 0, sizeof(dfus_init));
		
    dfus_init.evt_handler                               = ble_dfu_evt_handler;
    dfus_init.ctrl_point_security_req_write_perm        = SEC_SIGNED;
    dfus_init.ctrl_point_security_req_cccd_write_perm   = SEC_SIGNED;

    err_code = ble_dfu_init(&m_dfus, &dfus_init);
    APP_ERROR_CHECK(err_code);
	
    // Initialize Heart Rate Service.
    body_sensor_location = BLE_HRS_BODY_SENSOR_LOCATION_FINGER;

    memset(&hrs_init, 0, sizeof(hrs_init));

    hrs_init.evt_handler                 = NULL;
    hrs_init.is_sensor_contact_supported = true;
    hrs_init.p_body_sensor_location      = &body_sensor_location;

    // Here the sec level for the Heart Rate Service can be changed/increased.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&hrs_init.hrs_hrm_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hrs_init.hrs_hrm_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hrs_init.hrs_hrm_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&hrs_init.hrs_bsl_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hrs_init.hrs_bsl_attr_md.write_perm);

    err_code = ble_hrs_init(&m_hrs, &hrs_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Battery Service.
    memset(&bas_init, 0, sizeof(bas_init));

    // Here the sec level for the Battery Service can be changed/increased.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&bas_init.battery_level_char_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_report_read_perm);

    bas_init.evt_handler          = NULL;
    bas_init.support_notification = true;
    bas_init.p_report_ref         = NULL;
    bas_init.initial_batt_level   = 100;

    err_code = ble_bas_init(&m_bas, &bas_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Device Information Service.
    memset(&dis_init, 0, sizeof(dis_init));
//		NRF_LOG_INFO("iserial_number 0 reg is %x.\r\n",iserial_number[0]);
//		NRF_LOG_INFO("iserial_number 1 reg is %x.\r\n",iserial_number[1]);
    ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, (char *)MANUFACTURER_NAME);
		ble_srv_ascii_to_utf8(&dis_init.hw_rev_str, (char *)HARDWARE_VER);
		ble_srv_ascii_to_utf8(&dis_init.fw_rev_str, (char *)FIRMWARE_VER);
		ble_srv_ascii_to_utf8(&dis_init.model_num_str, (char *)MODE_NUMBER);
		ble_srv_ascii_to_utf8(&dis_init.serial_num_str, (char *)iserial_number);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&dis_init.dis_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&dis_init.dis_attr_md.write_perm);

    err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the sensor simulators.
 */
//static void sensor_simulator_init(void)
//{
//    m_rr_interval_sim_cfg.min          = MIN_RR_INTERVAL;
//    m_rr_interval_sim_cfg.max          = MAX_RR_INTERVAL;
//    m_rr_interval_sim_cfg.incr         = RR_INTERVAL_INCREMENT;
//    m_rr_interval_sim_cfg.start_at_max = false;

//    sensorsim_init(&m_rr_interval_sim_state, &m_rr_interval_sim_cfg);
//}


/**@brief Function for starting application timers.
 */
static void application_timers_start(void)
{
    uint32_t err_code;

    // Start application timers.
//    err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
//    APP_ERROR_CHECK(err_code);

////    err_code = app_timer_start(m_heart_rate_timer_id, HEART_RATE_MEAS_INTERVAL, NULL);
////    APP_ERROR_CHECK(err_code);

//    //err_code = app_timer_start(m_buzzer_timer_id, BUZZER_INTERVAL, NULL);
//    //APP_ERROR_CHECK(err_code);

//    err_code = app_timer_start(m_sensor_contact_timer_id, SENSOR_CONTACT_DETECTED_INTERVAL, NULL);
//    APP_ERROR_CHECK(err_code);
	
		err_code = app_timer_start(m_charge_timer_id, CHARGE_TIMEOUT_MS, NULL);
		APP_ERROR_CHECK(err_code);
	
}

static void application_timers_stop(void)
{
		uint32_t err_code;
//		err_code = app_timer_stop(m_battery_timer_id);
//    APP_ERROR_CHECK(err_code);

    err_code = app_timer_stop(m_heart_rate_timer_id);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_stop(m_buzzer_timer_id);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_stop(m_sensor_contact_timer_id);
    APP_ERROR_CHECK(err_code);
		
//		err_code = app_timer_stop(m_charge_timer_id);
//		APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = m_hrs.hrm_handles.cccd_handle;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

void Enter_Sleep_IO_Setting(void)
{
		PWM_off();
		if(BMA253_stop())
		{
			NRF_LOG_INFO("BMA253 went into sleep!\r\n");
		}
		if(SDP3x_Enter_Sleep())
		{
			NRF_LOG_INFO("SDP3x went into sleep!\r\n");
		}
		//SDP3x_PowerOFF(); 
		app_timer_stop_all();
}

/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
		Enter_Sleep_IO_Setting();

    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);
		

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;
		uint16_t num_sector; //60s 存了几个sector
		uint16_t single_sector;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            NRF_LOG_INFO("Fast Advertising\r\n");
						m_adv_mode = BLE_ADV_EVT_FAST;
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_IDLE:
						NRF_LOG_INFO("Idle\r\n");
						m_adv_mode = BLE_ADV_EVT_IDLE;
            sleep_mode_enter();
//						uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
//						APP_ERROR_CHECK(err_code);
//						
//						err_code = app_timer_stop(m_heart_rate_timer_id);
//						APP_ERROR_CHECK(err_code);

//						err_code = app_timer_stop(m_buzzer_timer_id);
//						APP_ERROR_CHECK(err_code);

//						err_code = app_timer_stop(m_sensor_contact_timer_id);
//						APP_ERROR_CHECK(err_code);
//				
//						num_sector = (flash_storage_addr - flash_sector_addr)/PER_SECTOR_SIZE;
//						single_sector = (flash_storage_addr - flash_sector_addr) % PER_SECTOR_SIZE;
//						if(single_sector)
//								num_sector += 1;
//	
//						flash_sector_addr += PER_SECTOR_SIZE * num_sector; //未断电时，获取下一次离线时的falsh sector first addr
//						flash_storage_addr = 	flash_sector_addr;  //保存数据地址
            break;

        default:
            break;
    }
}


/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t err_code;
//		offline_data fl_data;
		static uint32_t addr = 0;   //按页存储，连线后离线数据发送出去，就删flash，所以每次连线都从0地址开始读。
		static uint16_t num_page = 0;
		static uint16_t i = 0;
		uint8_t buf[20] = {0};
		static uint16_t counter1 = 0;
		static uint8_t counter2 = 0;
		uint16_t presure_data = 0;
		static bool flag = true;
		uint8_t transfered_mark = 0xaa;
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
						err_code = app_timer_stop(m_heart_rate_timer_id);
						APP_ERROR_CHECK(err_code);
            NRF_LOG_INFO("Connected\r\n");
		
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
						m_BLE_Connect_Status = true;
				
						counter2++;
						data_test[3] = counter2;
						err_code = ble_hrs_connected_measurement_send(&m_hrs, data_test, NUM_OF_STORAGE_DATA);
//						NRF_LOG_INFO("err_code:%d\r\n",err_code);
						if ((err_code != NRF_SUCCESS) &&
						(err_code != NRF_ERROR_INVALID_STATE) &&
						(err_code != BLE_ERROR_NO_TX_PACKETS) &&
						(err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING))
						{
							NRF_LOG_INFO("BLE Data send error.err_code:%d\r\n",err_code);
							APP_ERROR_HANDLER(err_code);
						}
						counter2++;
						data_test[3] = counter2;
						err_code = ble_hrs_connected_measurement_send(&m_hrs, data_test, NUM_OF_STORAGE_DATA);
//						NRF_LOG_INFO("err_code:%d\r\n",err_code);
						if ((err_code != NRF_SUCCESS) &&
						(err_code != NRF_ERROR_INVALID_STATE) &&
						(err_code != BLE_ERROR_NO_TX_PACKETS) &&
						(err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING))
						{
							NRF_LOG_INFO("BLE Data send error.err_code:%d\r\n",err_code);
							APP_ERROR_HANDLER(err_code);
						}
//				do{
//						presure_data = (data_test[1] << 8 | data_test[2]);  
////						if(presure_data >= (uint16_t)0xd8f0 && flag)
////						{
//								presure_data -= 500;
////						}
////						else
////						{
////								presure_data += 500;
////								flag = false;
////						}
//						data_test[1] = (presure_data >> 8) & 0xff;
//						data_test[2] = presure_data & 0xff;
//				
//						err_code = ble_hrs_connected_measurement_send(&m_hrs, data_test, NUM_OF_STORAGE_DATA);
//					NRF_LOG_INFO("counter:%d, presure:%d\r\n",counter1++, presure_data);
//						if ((err_code != NRF_SUCCESS) &&
//						(err_code != NRF_ERROR_INVALID_STATE) &&
//						(err_code != BLE_ERROR_NO_TX_PACKETS) &&
//						(err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING))
//						{
//							NRF_LOG_INFO("BLE Data send error.err_code:%d\r\n",err_code);
//							APP_ERROR_HANDLER(err_code);
//						}
//					}while(presure_data >= 0xd8f0);

//							err_code = app_timer_start(m_heart_rate_timer_id, HEART_RATE_OFF_LINE_MEAS_INTERVAL, NULL);//连接
//							APP_ERROR_CHECK(err_code);
							
							
//						num_page = flash_storage_addr / PER_WRITE_PAGE_SIZE + 1;
//						w25x40_BufferRead(buf, addr, NUM_OF_STORAGE_DATA);
//						if(buf[4] != 0xff)
//						{
//								ble_hrs_connected_measurement_send(&m_hrs, &buf[4], NUM_OF_STORAGE_DATA);
//								addr += NUM_OF_STORAGE_DATA;
//						}
//						err_code = app_timer_start(m_heart_rate_timer_id, HEART_RATE_MEAS_INTERVAL, NULL);
//						APP_ERROR_CHECK(err_code);
						/*按*/
//						if(offline_is_tranfered(flash_storage_addr)) //true ,not tranfer.when device connected,date tranfer 
//						{
//								fl_data = w25x40_read_offline_data(flash_storage_addr);
//								for(uint16_t i = 0; i <= fl_data.total_count; i++)
//								{
//										w25x40_BufferRead(buf, fl_data.first_measure_count_addr, 16);
//										ble_hrs_connected_measurement_send(&m_hrs, &buf[4], 16);
//										w25x40_BufferWrite(&transfered_mark, fl_data.first_measure_count_addr + 15, 1);
//										fl_data.first_measure_count_addr += 16;
//								}
//						}
							/* 按sector来存*/
//						addr = w25x40_sector_read_offline_data(flash_sector_addr, NUM_OF_STORAGE_DATA);
//						do
//						{
//								w25x40_BufferRead(buf, addr, 16);
//								ble_hrs_connected_measurement_send(&m_hrs, &buf[4], 16);
//								w25x40_BufferWrite(&transfered_mark, (addr + NUM_OF_STORAGE_DATA - 1), 1);
//								addr += 16;	
//						}while(!flash_is_used(&buf[4], 8)); //不为0xff，接着读

						NRF_LOG_INFO("Sensor Timer is on! \r\n");
            break; // BLE_GAP_EVT_CONNECTED

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected, reason %d\r\n",
                          p_ble_evt->evt.gap_evt.params.disconnected.reason);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
						m_BLE_Connect_Status = false;
						err_code = app_timer_stop(m_heart_rate_timer_id);
						APP_ERROR_CHECK(err_code);
//						NRF_LOG_INFO("Sensor Timer is off! \r\n");
						if(SDP3x_Enter_Sleep())
						{
							NRF_LOG_INFO("SDP3x Enter Sleep!\r\n");
						}
//						err_code = app_pwm_channel_duty_set(&PWM1, 0,20);
//						APP_ERROR_CHECK(err_code);
            break; // BLE_GAP_EVT_DISCONNECTED

				case BLE_GAP_EVT_CONN_PARAM_UPDATE:
							NRF_LOG_INFO("CONN_PARAM_UPDATE!\r\n");
//						PWM_on();
//						num_page = flash_storage_addr / PER_WRITE_PAGE_SIZE + 1;
//						w25x40_BufferRead(buf, addr, NUM_OF_STORAGE_DATA);
//						if(buf[4] != 0xff)
//						{
//								ble_hrs_connected_measurement_send(&m_hrs, &buf[4], NUM_OF_STORAGE_DATA);
//								addr += NUM_OF_STORAGE_DATA;
//						}
						break;
				case BLE_GAP_EVT_CONN_SEC_UPDATE:
						NRF_LOG_INFO("CONN_SEC_UPDATE!\r\n");
//						PWM_on();
						break;
				
				case BLE_EVT_TX_COMPLETE:
						counter2++;
//						sd_ble_tx_packet_count_get(m_conn_handle, &counter2);
						NRF_LOG_INFO("counter2 = %d\r\n", counter2);
						data_test[3] = counter2;
							
//						NRF_LOG_INFO("err_code:%d\r\n",err_code);
//						if(counter2 < 15)
//						{		
//								presure_data = (data_test[1] << 8 | data_test[2]);
//								err_code = ble_hrs_connected_measurement_send(&m_hrs, data_test, NUM_OF_STORAGE_DATA);
//								NRF_LOG_INFO("counter:%d, presure:%d\r\n",++counter1, presure_data);
//								if((err_code != NRF_SUCCESS) &&
//								(err_code != NRF_ERROR_INVALID_STATE) &&
//								(err_code != BLE_ERROR_NO_TX_PACKETS) &&
//								(err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING))
//								{
//									NRF_LOG_INFO("BLE Data send error.err_code:%d\r\n",err_code);
//									APP_ERROR_HANDLER(err_code);
//								}
//						}
						presure_data = (data_test[1] << 8 | data_test[2]);  
						if((presure_data >= (uint16_t)0xd8f0) && flag)
						{
								presure_data -= 500;
								data_test[1] = (presure_data >> 8) & 0xff;
								data_test[2] = presure_data & 0xff;
								err_code = ble_hrs_connected_measurement_send(&m_hrs, data_test, NUM_OF_STORAGE_DATA);
								NRF_LOG_INFO("counter:%d, presure:%d\r\n",counter1++, presure_data);
								if ((err_code != NRF_SUCCESS) &&
								(err_code != NRF_ERROR_INVALID_STATE) &&
								(err_code != BLE_ERROR_NO_TX_PACKETS) &&
								(err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING))
								{
									NRF_LOG_INFO("BLE Data send error.err_code:%d\r\n",err_code);
									APP_ERROR_HANDLER(err_code);
								}
						}
						else
						{
								if(presure_data > 0xfc17)
								{
										break;
										err_code = app_timer_start(m_heart_rate_timer_id, HEART_RATE_OFF_LINE_MEAS_INTERVAL, NULL); //一上电就测量。500ms间隔
										APP_ERROR_CHECK(err_code);
								}
								flag = false;
								presure_data += 500;
								data_test[1] = (presure_data >> 8) & 0xff;
								data_test[2] = presure_data & 0xff;
								err_code = ble_hrs_connected_measurement_send(&m_hrs, data_test, NUM_OF_STORAGE_DATA);
								NRF_LOG_INFO("counter:%d, presure:%d\r\n",counter1++, presure_data);
								if ((err_code != NRF_SUCCESS) &&
								(err_code != NRF_ERROR_INVALID_STATE) &&
								(err_code != BLE_ERROR_NO_TX_PACKETS) &&
								(err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING))
								{
									NRF_LOG_INFO("BLE Data send error.err_code:%d\r\n",err_code);
									APP_ERROR_HANDLER(err_code);
								}
								
						}

//						ble_hrs_connected_measurement_send(&m_hrs, data_test, NUM_OF_STORAGE_DATA);
//						if(counter1 == 16)
//						{
//								presure_data = (data_test[1] << 8 | data_test[2]);
//								if((presure_data >= (uint16_t)0xdcd8) && flag)
//								{
//										presure_data -= 500; 
//										data_test[1] = (presure_data >> 8) & 0xff;
//										data_test[2] = presure_data & 0xff;
//										ble_hrs_connected_measurement_send(&m_hrs, data_test, NUM_OF_STORAGE_DATA);
//								}
//								else
//								{
//										if(presure_data > 0xff38)
//												break;
//										presure_data += 500;
//										flag = false;
//										data_test[1] = (presure_data >> 8) & 0xff;
//										data_test[2] = presure_data & 0xff;
//										ble_hrs_connected_measurement_send(&m_hrs, data_test, NUM_OF_STORAGE_DATA);
//								}
//								data_test[1] = (presure_data >> 8) & 0xff;
//								data_test[2] = presure_data & 0xff;
//								if(presure_data <= 0xff38)
//										ble_hrs_connected_measurement_send(&m_hrs, data_test, NUM_OF_STORAGE_DATA);
//								else
//									break;
//								counter = 0;
//				}
//						w25x40_BufferRead(buf, addr, NUM_OF_STORAGE_DATA);
//						if(buf[4] != 0xff)
//						{
//								ble_hrs_connected_measurement_send(&m_hrs, &buf[4], NUM_OF_STORAGE_DATA);
//								addr += NUM_OF_STORAGE_DATA;
//						}
//						else
//						{
//								i++;
//								if(i < num_page)
//								{
//										addr = PER_WRITE_PAGE_SIZE * i;
//										w25x40_BufferRead(buf, addr, NUM_OF_STORAGE_DATA);
//										ble_hrs_connected_measurement_send(&m_hrs, &buf[4], NUM_OF_STORAGE_DATA);
//										addr += NUM_OF_STORAGE_DATA;
//								}
//								else
//								{
////										w25x40_Simulate_PageErase(flash_storage_addr);
//										err_code = app_timer_start(m_heart_rate_timer_id, HEART_RATE_OFF_LINE_MEAS_INTERVAL, NULL);//连接
//										APP_ERROR_CHECK(err_code);
//								}
//						}
						break;
				
        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.\r\n");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTC_EVT_TIMEOUT

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.\r\n");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_TIMEOUT

        case BLE_EVT_USER_MEM_REQUEST:
            err_code = sd_ble_user_mem_reply(m_conn_handle, NULL);
            APP_ERROR_CHECK(err_code);
            break; // BLE_EVT_USER_MEM_REQUEST

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
        {
            ble_gatts_evt_rw_authorize_request_t  req;
            ble_gatts_rw_authorize_reply_params_t auth_reply;

            req = p_ble_evt->evt.gatts_evt.params.authorize_request;

            if (req.type != BLE_GATTS_AUTHORIZE_TYPE_INVALID)
            {
                if ((req.request.write.op == BLE_GATTS_OP_PREP_WRITE_REQ)     ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW) ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL))
                {
                    if (req.type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
                    }
                    else
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
                    }
                    auth_reply.params.write.gatt_status = APP_FEATURE_NOT_SUPPORTED;
                    err_code = sd_ble_gatts_rw_authorize_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                               &auth_reply);
                    APP_ERROR_CHECK(err_code);
                }
            }
        } break; // BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST


        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the BLE Stack event interrupt handler after a BLE stack
 *          event has been received.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    ble_conn_state_on_ble_evt(p_ble_evt);
    pm_on_ble_evt(p_ble_evt);
    ble_hrs_on_ble_evt(&m_hrs, p_ble_evt);
    ble_bas_on_ble_evt(&m_bas, p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);
    bsp_btn_ble_on_ble_evt(p_ble_evt);
    on_ble_evt(p_ble_evt);
    ble_advertising_on_ble_evt(p_ble_evt);
		ble_dfu_on_ble_evt(&m_dfus, p_ble_evt);
    nrf_ble_gatt_on_ble_evt(&m_gatt, p_ble_evt);
}


/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in] sys_evt  System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt)
{
    // Dispatch the system event to the fstorage module, where it will be
    // dispatched to the Flash Data Storage (FDS) module.
    fs_sys_event_handler(sys_evt);

    // Dispatch to the Advertising module last, since it will check if there are any
    // pending flash operations in fstorage. Let fstorage process system events first,
    // so that it can report correctly to the Advertising module.
    ble_advertising_on_sys_evt(sys_evt);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;

    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;

    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);

    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(NRF_BLE_CENTRAL_LINK_COUNT,
                                                    NRF_BLE_PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(NRF_BLE_CENTRAL_LINK_COUNT, NRF_BLE_PERIPHERAL_LINK_COUNT);

    // Enable BLE stack.
#if (NRF_SD_BLE_API_VERSION == 3)
    ble_enable_params.gatt_enable_params.att_mtu = NRF_BLE_GATT_MAX_MTU_SIZE;
#endif
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;

    switch (event)
    {
        case BSP_EVENT_SLEEP:
						NRF_LOG_DEBUG("BSP_EVENT_SLEEP.\r\n");
						nrf_delay_ms(500);
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist();
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break;
//				case BSP_EVENT_KEY_0:
//						if (m_conn_handle == BLE_CONN_HANDLE_INVALID && m_adv_mode == BLE_ADV_EVT_IDLE)
//            {
//								err_code = app_timer_start(m_heart_rate_timer_id, HEART_RATE_MEAS_INTERVAL, NULL);
//								APP_ERROR_CHECK(err_code);

//								err_code = app_timer_start(m_buzzer_timer_id, BUZZER_INTERVAL, NULL);
//								APP_ERROR_CHECK(err_code);

//								err_code = app_timer_start(m_sensor_contact_timer_id, SENSOR_CONTACT_DETECTED_INTERVAL, NULL);
//								APP_ERROR_CHECK(err_code);
//								advertising_start();
//						}
//				case BSP_EVENT_ADVERTISING_START:
//						if (m_conn_handle == BLE_CONN_HANDLE_INVALID && m_adv_mode == BLE_ADV_EVT_IDLE)
//            {
//								err_code = app_timer_start(m_heart_rate_timer_id, HEART_RATE_MEAS_INTERVAL, NULL);
//								APP_ERROR_CHECK(err_code);

//								err_code = app_timer_start(m_buzzer_timer_id, BUZZER_INTERVAL, NULL);
//								APP_ERROR_CHECK(err_code);

//								err_code = app_timer_start(m_sensor_contact_timer_id, SENSOR_CONTACT_DETECTED_INTERVAL, NULL);
//								APP_ERROR_CHECK(err_code);
//								advertising_start();
//						}
//						break;
        default:
            break;
    }
}


/**@brief Function for the Peer Manager initialization.
 *
 * @param[in] erase_bonds  Indicates whether bonding information should be cleared from
 *                         persistent storage during initialization of the Peer Manager.
 */
static void peer_manager_init(bool erase_bonds)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    if (erase_bonds)
    {
        err_code = pm_peers_delete();
        APP_ERROR_CHECK(err_code);
    }

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = fds_register(fds_evt_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t               err_code;
    ble_advdata_t          advdata;
    ble_adv_modes_config_t options;

    // Build advertising data struct to pass into @ref ble_advertising_init.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance      = true;
    advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    advdata.uuids_complete.p_uuids  = m_adv_uuids;

    memset(&options, 0, sizeof(options));
    options.ble_adv_fast_enabled  = true;
    options.ble_adv_fast_interval = APP_ADV_INTERVAL;
    options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = ble_advertising_init(&advdata, NULL, &options, on_adv_evt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    bsp_event_t startup_event;

    uint32_t err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS /*| BSP_INIT_OUTIO*/ | BSP_INIT_INIO,
                                 APP_TIMER_TICKS(100, APP_TIMER_PRESCALER),
                                 bsp_event_handler);

    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for the Power manager.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();

    APP_ERROR_CHECK(err_code);
}

/*GATT generic Event handler*/
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t * p_evt)
{
    ble_hrs_on_gatt_evt(&m_hrs, p_evt);
}


/*GATT Module init*/
void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);
}

/*RTC init  
* 离线保存数据，时间函数*/
static void appRTCHandler(nrf_drv_rtc_int_type_t int_type)
{
//		static uint8_t count = 0;
//   if(int_type == NRF_DRV_RTC_INT_TICK)   //25ms trigger tick event
//	{
//			//LEDS_INVERT(BSP_LED_3_MASK);
//			count++;
//			if(rtc_timer_convert)           //25*20 = 500ms
//			{
//					if(count == 20)
//					{
//							
//					}
//			}
//			else
//			{
//					if(count == 3)             //25*3 = 75ms
//					{
//							
//					}
//			}
//	}
}

void appRTCInit(void)
{
	uint32_t err_code;
	nrf_drv_rtc_config_t rtc_config = NRF_DRV_RTC_DEFAULT_CONFIG;
	//Initialize RTC instance
	err_code = nrf_drv_rtc_init(&rtc, &rtc_config, appRTCHandler);
	APP_ERROR_CHECK(err_code);

	//Enable tick event & interrupt

	//Power on RTC instance
	nrf_drv_rtc_enable(&rtc);
}

/*SPI Flash init*/
void spi_flash_init(void)
{
		nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
    spi_config.ss_pin   = SPI_SS_PIN;
    spi_config.miso_pin = SPI_MISO_PIN;
    spi_config.mosi_pin = SPI_MOSI_PIN;
    spi_config.sck_pin  = SPI_SCK_PIN;
    APP_ERROR_CHECK(nrf_drv_spi_init(&m_spi_master_0, &spi_config, NULL));
		nrf_gpio_cfg_output(SPI_WP);
		nrf_gpio_pin_set(SPI_WP);
    nrf_gpio_cfg_output(SPI_HOLD);
		nrf_gpio_pin_set(SPI_HOLD);
}

/**@brief Function for application main entry.
 */
int main(void)
{
    uint32_t err_code;
    bool     erase_bonds;

//		uint8_t data[12] = {0};
    // Initialize.
    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    timers_init();
    buttons_leds_init(&erase_bonds);
//		SDP3x_PowerOFF();
		PWM_init();
		//SDP3x_PowerON();
	//电池电量读取ADC初始化
		m_batt_meas_init(50,hal_adc_pin_disabled);
	//添加Sensors读取的动作

		if(twi_master_init()!=true){
			NRF_LOG_INFO("TWI initial error.\r\n");
			bsp_board_led_on(BSP_LED_ALERT);
			//PWM_on();                    //ok, not ring
		}

		nrf_delay_ms(10);
		NRF_LOG_INFO("reset reg is %d.\r\n",NRF_POWER->RESETREAS);
#ifdef USE_SDP3x
		if(SDP3x_init() == 0){
			bsp_board_led_on(BSP_LED_ALERT);
			NRF_LOG_INFO("SDP3x initial error.\r\n");
			//PWM_on();                  //ring  
		}
		else
		{
			bsp_board_led_off(BSP_LED_ALERT);
			NRF_LOG_INFO("SDP3x is used!\r\n");
		}
		
		if(SDP3x_Enter_Sleep())
		{
			NRF_LOG_INFO("SDP3x Enter Sleep!\r\n");
		}
#endif

#ifdef USE_BMA253
		if(BMA253_init()== 0){
			NRF_LOG_INFO("MBMA253 initial error.\r\n");
			bsp_board_led_on(BSP_LED_ALERT);                        
		}
		else
		{
			bsp_board_led_off(BSP_LED_ALERT);
			NRF_LOG_INFO("BMA253 is used!\r\n");
		}
#endif
		spi_flash_init();
		
		//PWM_on();
//		buzzer_on(1,0);
		if(w25x40_Init()== 0)  //ERROR
		{
				bsp_board_led_on(BSP_LED_ALERT);
		}
		else                   //OK
		{
				bsp_board_led_off(BSP_LED_ALERT);
		}
//		flash_storage_addr = w25x40_Storage_Init(); //上电获取flash中可以存放数据的首地址。一次写16字节
//		flash_sector_addr = flash_storage_addr;
//		for(uint16_t j = 0; j < 32; j++)
//		{
//				w25x40_BufferWrite(data_test, flash_storage_addr, sizeof(data_test));
//				w25x40_BufferRead(data, flash_storage_addr, sizeof(data));
////				nrf_delay_ms(100);
//				for(uint8_t i = 0; i < 8 ; i++)
//				{
//						if(data[i+4] == i+1)
//						{
//								bsp_board_led_on(BSP_LED_GREEN);
//						}
//						else
//						{
//								
////								bsp_board_led_off(BSP_LED_GREEN);
//								PWM_on();
//						}
//				}
//				flash_storage_addr += 8;
////				if(flash_storage_addr == 8)  
////				{
////						
////						//PWM_on();
////				}
//		}
	//添加Sensors读取的动作结束
//		w25x40_ChipErase();
    ble_stack_init();
    peer_manager_init(erase_bonds);
    if (erase_bonds == true)
    {
        NRF_LOG_INFO("Bonds erased!\r\n");
    }
    gap_params_init();
    advertising_init();
    gatt_init();
    services_init();
//    sensor_simulator_init();
    conn_params_init();

    // Start execution.
		buzzer_on(1,0);
				
    NRF_LOG_INFO("Heart Rate Sensor Start!\r\n");
//    application_timers_start();
    advertising_start();
    // Enter main loop.
    for (;;)
    {
//				if(!nrf_gpio_pin_read(INIO_1)) //charge_goog pin low -> charging
//				{
//						if(red_led_convert_flag)
//						{
//								err_code = app_timer_start(m_charge_timer_id, CHARGE_TIMEOUT_MS, NULL);
//								APP_ERROR_CHECK(err_code);
//								bsp_board_led_invert(BSP_LED_RED);  //red led
//								red_led_convert_flag = false;
//						}
//				}
	
//			if(bsp_button_is_pressed(BSP_BOARD_BUTTON_0))
//			{
////				bsp_board_led_off(BSP_LED_ALERT);

//				sleep_mode_enter();
//			}
//			else
//			{
//				bsp_board_led_on(BSP_LED_ALERT);
//			}
			
//			if(bsp_button_is_pressed(BSP_BOARD_BUTTON_1))
//			{
//				bsp_board_led_on(BSP_LED_GREEN);
//			}
//			else
//			{
//				bsp_board_led_off(BSP_LED_GREEN);
//			}
//			NRF_LOG_INFO("main loop!\r\n");
//			nrf_delay_ms(1000);
			
        if (NRF_LOG_PROCESS() == false)
        {
//					NRF_LOG_INFO("main loop no log!\r\n");
            power_manage();
        }
    }
}


