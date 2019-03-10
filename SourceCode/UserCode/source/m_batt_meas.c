/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 * $LastChangedRevision$
 */
 
#include "m_batt_meas.h"
#include "m_batt_meas_cfg.h"

#include "app_util.h"
#include "nrf_assert.h"
#include "nrf_error.h"
#include "hal_adc.h"

static uint8_t                   m_batt_meas_prev_level       = 0;
static uint8_t                   m_batt_meas_threshold        = 0;

static uint8_t approxmiate_battery_level(uint16_t measurement)
{   
    float measured_voltage_mv;
    uint8_t level = 0;    
    
    // Calculation based on ADC settings: 10-bit resolution, 1/3 prescaled Vss compared to internal 1.2V reference.
    measured_voltage_mv = (((float)measurement) / 1024.f) * 3600.f;
    
#if ((M_BATT_MEAS_BATTERY_TYPE == M_BATT_MEAS_TYPE_ALKALINE_AAA)\
			||( M_BATT_MEAS_BATTERY_TYPE == M_BATT_MEAS_TYPE_ALKALINE_AAA_2x_SERIES)\
			||( M_BATT_MEAS_BATTERY_TYPE == M_BATT_MEAS_TYPE_LITHIUM_CR2032) )    
    // A simple linear approximation is sufficient when the power consumption is fairly low (<100mW)
    if (measured_voltage_mv >= M_BATT_MEAS_MAX_LEVEL)
    {
        level = 100;
    }
    else if (measured_voltage_mv <= M_BATT_MEAS_MIN_LEVEL)
    {
        level = 1;
    }
    else
    {
        static float a = (100.f/(M_BATT_MEAS_MAX_LEVEL - M_BATT_MEAS_MIN_LEVEL));
        static float b = ((100.f/(M_BATT_MEAS_MAX_LEVEL - M_BATT_MEAS_MIN_LEVEL)) * M_BATT_MEAS_MIN_LEVEL);
        
        level = (uint8_t) (measured_voltage_mv * a - b);
    }    
    
#elif M_BATT_MEAS_BATTERY_TYPE == M_BATT_MEAS_TYPE_ALKALINE_COINCELL
#warning Coincell battery level approximiation is not implemented.
#endif    
    
    return level;
}

uint8_t battery_level_read(void)
{    
    uint16_t val;    
    uint8_t  level;
    
    val   = hal_adc_convert_single();
    level = approxmiate_battery_level(val);
    
    if ((level < (m_batt_meas_prev_level - m_batt_meas_threshold)) || 
        (level > (m_batt_meas_prev_level + m_batt_meas_threshold)))    
    {
        m_batt_meas_prev_level = level;
    }
		
		return level;
}

uint32_t m_batt_meas_init(uint8_t threshold, hal_adc_pin_sel_t adc_input)
{
    uint32_t err_code;
    hal_adc_init_t adc_params;
    
    m_batt_meas_threshold = threshold;
   
    // Note: These settings impact how battery percentage is calculated
    adc_params.res       = hal_adc_res_10bit;             // 10-bit resolution
    adc_params.refsel    = hal_adc_ref_vbg;               // Internal 1.2 V band gap as reference
    adc_params.psel      = adc_input;                     
    adc_params.extrefsel = hal_adc_ext_none;              // Not using an external analog reference
    if(adc_input==hal_adc_pin_disabled)
    {
        adc_params.inpsel = hal_adc_inp_vdd_onethird_pre;  // VDD with 1/3 prescaling as input
    }
    else
    {
        adc_params.inpsel = hal_adc_inp_analog_onethird_pre; // Analog input from pin with 1/3 prescaling
    }
    
		err_code =  hal_adc_init(&adc_params);
    
    return err_code;
}


