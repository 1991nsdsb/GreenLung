#ifndef SDP3X_H
#define SDP3X_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
	
//#define USE_SDP31
#define USE_SDP32
	
#define SDP3X_SLAVEADDR 				0x21
#define SDP31_PRODUCT_ID				0x03010188UL
#define SDP32_PRODUCT_ID				0x03010201UL

#define SDP31_SCALE_FACTORS_DIFFPRES 	60
#define SDP32_SCALE_FACTORS_DIFFPRES 	240
#define SDP31_SCALE_FACTORS_TEMP	 	200
#define SDP32_SCALE_FACTORS_TEMP	 	200
#define READWAITTIME								500   //count
typedef enum
{
  CON_MEAS_MASS_FLOW_AVG 				= 0x3603,
	CON_MEAS_MASS_FLOW_NONAVG 			= 0x3608,
	CON_MEAS_DIFF_PRES_AVG 				= 0x3615,
	CON_MEAS_DIFF_PRES_NONAVG 			= 0x361E,
	CON_MEAS_STOP_MEAS					= 0x3FF9,
	TRIG_MEAS_MASS_FLOW 				= 0x3624,
	TRIG_MEAS_MASS_FLOW_CLK_STRE		= 0x3726,
	TRIG_MEAS_DIFF_PRES					= 0x362F,
	TRIG_MEAS_DIFF_PRES_CLK_STRE 		= 0x372D,
	READ_PRODUCT_IDENTIFIER0 			= 0x367C,
	READ_PRODUCT_IDENTIFIER1 			= 0xE102,
	READ_DATA 										= 0xE000,
	SDP3X_ENTER_SLEEP_MODE				= 0x3677,
	SOFT_RESET 								= 0x0006,                         
} SDP3x_Commands_type_t;
typedef enum
{
	STATUS_NULL												= 0,
	CONTINUOUS_MODE_STOP_MEAS					,
  CONTINUOUS_MODE_MASS_FLOW_AVG			,
	CONTINUOUS_MODE_FLOW_NONAVG 			,
	CONTINUOUS_MODE_DIFF_PRES_AVG 		,
	CONTINUOUS_MODE_DIFF_PRES_NONAVG 	,
	TRIG_MODE_MASS_FLOW 							,
	TRIG_MODE_MASS_FLOW_CLK_STRE			,
	TRIG_MODE_DIFF_PRES								,
	TRIG_MODE_DIFF_PRES_CLK_STRE 			,
	DEVICE_SOFT_RESET 								,                         
} SDP3x_Status_type_t;

bool SDP3x_init(void);
long SDP3x_Get_BMP180UT(void);
long SDP3x_Get_BMP180UP(void);
bool SDP3x_verify_product_id(void);
//bool SDP3x_register_write(uint16_t commands);
//bool SDP3x_register_read(uint8_t * destination, uint8_t number_of_bytes);
bool SDP3x_data_read(uint8_t * destination, uint8_t number_of_bytes);
bool SDP3x_Enter_Sleep(void);

#ifdef __cplusplus
}
#endif

#endif /* SDP3X_H */
