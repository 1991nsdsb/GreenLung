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

/** @file 
 *
 * @defgroup modules_batt_meas
 * @{
 * @ingroup nrfready_modules
 * @brief Battery module.
 *
 * @details This module deals with battery measurements.
 *          The figure below depicts how this module operates:
 * @image html flow_m_batt_meas.png Battery measurement module flow 
 */
#ifndef __M_BATT_MEAS_H__
#define __M_BATT_MEAS_H__

#include <stdint.h>

#include "m_batt_meas_cfg.h"

#include "app_scheduler.h"

#include "hal_adc.h"

typedef enum
{
    m_batt_meas_type_alkaline_aaa
} m_batt_meas_type_t;

/**@brief Battery measurement initialization
 * 
 * @param[in] event_callback Event callback called when change in battery level exceeds threshold
 * @param[in] threshold      Required change in voltage to generate event [%]
 * @param[in] adc_input      Which input to read battery voltage from. Analog IO or internal.
 * @return 
 * @retval NRF_SUCCESS
 * @retval NRF_ERROR_INVALID_PARAM
 */
uint32_t m_batt_meas_init(uint8_t threshold, hal_adc_pin_sel_t adc_input);


uint8_t battery_level_read(void);

#endif /* __M_BATT_MEAS_H__ */

/** @} */
