/**
 * Copyright (c) 2009 - 2017, Nordic Semiconductor ASA
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

#include <stdbool.h>
#include <stdint.h>

#include "twi_master.h"
#include "bma2x2.h"
#include "nrf_delay.h"

#define NRF_LOG_MODULE_NAME "BMA"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
static uint8_t       m_bma253_device_status;          // !< Device address in bits [7:1]

bool BMA253_verify_product_id(void)
{
    uint8_t bma_chip_ID = 0;

    if (BMA253_register_read(BMA2x2_CHIP_ID_ADDR, &bma_chip_ID, 1))
    {
			NRF_LOG_INFO("bma_chip_ID is %d\r\n",bma_chip_ID);
        if (bma_chip_ID != BMA253_CHIP_ID_VALUE)
        {
            return false;
        }
        else
        {
            return true;
        }
    }
    else
    {
        return false;
    }
}

bool BMA253_register_write(uint8_t register_address, uint8_t value)
{
    uint8_t w2_data[2];

    w2_data[0] = register_address;
    w2_data[1] = value;
    return twi_master_transfer(m_bma253_device_status, w2_data, 2, TWI_ISSUE_STOP);
}

bool BMA253_register_read(uint8_t register_address, uint8_t * destination, uint8_t number_of_bytes)
{
    bool transfer_succeeded;
    transfer_succeeded  = twi_master_transfer(m_bma253_device_status, &register_address, 1, TWI_DONT_ISSUE_STOP);
    transfer_succeeded &= twi_master_transfer(m_bma253_device_status|TWI_READ_BIT, destination, number_of_bytes, TWI_ISSUE_STOP);
    return transfer_succeeded;
}
/*!
 *	@brief This API is used to set the operating
 *	modes of the sensor in the register 0x11 and 0x12
 *	@note Register 0x11 - bit from 5 to 7
 *	@note Register 0x12 - bit from 5 and 6
 *
 *
 *  @param power_mode_u8 : The value of power mode
 *	power_mode_u8         |value  |   0x11  |   0x12
 *  ------------------------- |-------| --------|--------
 *  BMA2x2_MODE_NORMAL        |  0    |  0x00   |  0x00
 *  BMA2x2_MODE_LOWPOWER1     |  1    |  0x02   |  0x00
 *  BMA2x2_MODE_SUSPEND       |  2    |  0x06   |  0x00
 *  BMA2x2_MODE_DEEP_SUSPEND  |  3    |  0x01   |  0x00
 *  BMA2x2_MODE_LOWPOWER2     |  4    |  0x02   |  0x01
 *  BMA2x2_MODE_STANDBY       |  5    |  0x04   |  0x00
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */

bool BMA253_init(void)
{
    bool transfer_succeeded = true;

    m_bma253_device_status = (uint8_t)(BMA2x2_I2C_ADDR1 << 1);

		//ÉèÖÃPower mode
		transfer_succeeded = BMA253_register_write(BMA2x2_MODE_CTRL_ADDR,BMA2x2_MODE_NORMAL);
		nrf_delay_ms(1);
    transfer_succeeded &= BMA253_verify_product_id();
		transfer_succeeded &= BMA253_register_write(BMA2x2_BW_SELECT_ADDR,BMA2x2_BW_62_50HZ);
		transfer_succeeded &= BMA253_register_write(BMA2x2_RANGE_SELECT_ADDR,BMA2x2_RANGE_2G);
    return transfer_succeeded;
}

bool BMA253_stop(void)
{
    bool transfer_succeeded = false;

		//ÉèÖÃPower mode
		transfer_succeeded = BMA253_register_write(BMA2x2_MODE_CTRL_ADDR,BMA2x2_MODE_DEEP_SUSPEND);

    return transfer_succeeded;
}
