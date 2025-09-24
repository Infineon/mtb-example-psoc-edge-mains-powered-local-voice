/*******************************************************************************
* \file mtb_ctp_ft5406.c
*
* \brief
* Provides implementation of the FT5406 touch panel driver library.
*
********************************************************************************
* \copyright
* Copyright 2025 Cypress Semiconductor Corporation
* SPDX-License-Identifier: Apache-2.0
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/


/*******************************************************************************
* Header Files
*******************************************************************************/
#include "mtb_ctp_ft5406.h"

/*******************************************************************************
* Macros
*******************************************************************************/

/* I2C slave address */
#define MTB_CTP_FT5406_I2C_ADDRESS         (0x38)

#define RETRY_COUNT                        (5U)

#define I2C_TIMEOUT                        (5U)

#define I2C_DELAY_MS                       (1U)

#define RESET_VAL                          (0U)

#define SET_VAL                            (1U)

/* Combine I2C controller error statuses in single mask  */
#define I2C_CONTROLLER_ERROR_MASK          (CY_SCB_I2C_MASTER_DATA_NAK | \
                                            CY_SCB_I2C_MASTER_ADDR_NAK | \
                                            CY_SCB_I2C_MASTER_ARB_LOST | \
                                            CY_SCB_I2C_MASTER_ABORT_START | \
                                            CY_SCB_I2C_MASTER_BUS_ERR)


#define TOUCH_POINT_GET_EVENT(T) ((mtb_ctp_touch_event_t)(uint8_t)((T).XH >> 6U))
#define TOUCH_POINT_GET_ID(T)    ((T).YH >> 4)
#define TOUCH_POINT_GET_X(T)     (int)((((uint16_t)(T).XH & 0x0fU) << 8) | \
                                      (uint16_t)(T).XL)
#define TOUCH_POINT_GET_Y(T)     (int)((((uint16_t)(T).YH & 0x0fU) << 8) | \
                                      (uint16_t)(T).YL)

/*******************************************************************************
* Global Variables
*******************************************************************************/
static mtb_ctp_ft5406_config_t* ft5406_config = NULL;


/*******************************************************************************
* Function Name: mtb_ctp_i2c_write_packet
********************************************************************************
* This function performs I2C master write operation
*
* \param *write_buffer
* Pointer to I2C write buffer
*
* \param buff_size
* I2C write buffer size
*
* \return cy_en_scb_i2c_status_t
* I2C error status
*
*******************************************************************************/
static cy_en_scb_i2c_status_t mtb_ctp_i2c_write_packet(uint8_t* write_buffer,
                                                       int buff_size)
{
    cy_stc_scb_i2c_master_xfer_config_t transfer_config;
    cy_en_scb_i2c_status_t i2c_status = CY_SCB_I2C_SUCCESS;
    uint8_t retry_count               = RETRY_COUNT;
    uint32_t i2c_controller_stat      = 0;
    uint32_t timeout_count;

    CY_ASSERT(NULL != write_buffer);

    /* I2C controller transfer configuration */
    transfer_config.slaveAddress      = MTB_CTP_FT5406_I2C_ADDRESS;
    transfer_config.buffer            = write_buffer;
    transfer_config.bufferSize        = buff_size;
    /* Generate Stop condition the end of transaction */
    transfer_config.xferPending       = false;

    do
    {
        /* Initiate write transaction */
        /* The Start condition is generated to begin this transaction */
        i2c_status = Cy_SCB_I2C_MasterWrite(ft5406_config->scb_inst,
                                            &transfer_config,
                                            ft5406_config->i2c_context);

        if (CY_SCB_I2C_SUCCESS == i2c_status)
        {
            /* Total timeout 5 ms */
            timeout_count = I2C_TIMEOUT;
            /* Wait until controller completes write transfer or time out has occurred */
            do
            {
                i2c_controller_stat = Cy_SCB_I2C_MasterGetStatus(ft5406_config->scb_inst,
                                                                 ft5406_config->i2c_context);
                Cy_SysLib_Delay(I2C_DELAY_MS);
                timeout_count--;
            } while ((CY_SCB_I2C_MASTER_BUSY & i2c_controller_stat) && timeout_count);

            if ((!(I2C_CONTROLLER_ERROR_MASK & i2c_controller_stat)) &&
                    (transfer_config.bufferSize ==
                            Cy_SCB_I2C_MasterGetTransferCount(ft5406_config->scb_inst, ft5406_config->i2c_context)))
            {
                i2c_status  = CY_SCB_I2C_SUCCESS;
                retry_count = RESET_VAL;
            }
            else
            {
                /* Timeout/error recovery */
                Cy_SCB_I2C_Disable(ft5406_config->scb_inst, ft5406_config->i2c_context);
                Cy_SCB_I2C_Enable(ft5406_config->scb_inst);
                retry_count--;
            }
        }
        else
        {
            retry_count--;
        }
    } while (retry_count);

    return i2c_status;
}


/*******************************************************************************
* Function Name: mtb_ctp_i2c_read_packet
********************************************************************************
* This function performs I2C master read operation
*
* \param *read_buffer
* Pointer to I2C read buffer
*
* \param buff_size
* I2C read buffer size
*
* \return cy_en_scb_i2c_status_t
* I2C error status
*
*******************************************************************************/
static cy_en_scb_i2c_status_t mtb_ctp_i2c_read_packet(uint8_t* read_buffer,
                                                      int buff_size)
{
    cy_stc_scb_i2c_master_xfer_config_t transfer_config;
    cy_en_scb_i2c_status_t i2c_status = CY_SCB_I2C_SUCCESS;
    uint8_t retry_count               = RETRY_COUNT;
    uint32_t i2c_controller_stat      = 0;
    uint32_t timeout_count;

    CY_ASSERT(NULL != read_buffer);

    /* I2C controller transfer configuration */
    transfer_config.slaveAddress      = MTB_CTP_FT5406_I2C_ADDRESS;
    transfer_config.buffer            = read_buffer;
    transfer_config.bufferSize        = buff_size;
    transfer_config.xferPending       = false;

    do
    {
        /* Initiate read transaction. */
        i2c_status = Cy_SCB_I2C_MasterRead(ft5406_config->scb_inst, &transfer_config,
                                           ft5406_config->i2c_context);

        if (CY_SCB_I2C_SUCCESS == i2c_status)
        {
            /* Total timeout 5 ms */
            timeout_count = I2C_TIMEOUT;
            /* Wait until controller complete read transfer or time out has occurred */
            do
            {
                i2c_controller_stat = Cy_SCB_I2C_MasterGetStatus(ft5406_config->scb_inst,
                                                     ft5406_config->i2c_context);
                Cy_SysLib_Delay(I2C_DELAY_MS);
                timeout_count--;
            } while ((CY_SCB_I2C_MASTER_BUSY & i2c_controller_stat) && timeout_count);

            if (!(I2C_CONTROLLER_ERROR_MASK & i2c_controller_stat))
            {
                i2c_status  = CY_SCB_I2C_SUCCESS;
                retry_count = RESET_VAL;
            }
            else
            {
                /* Timeout/error recovery */
                Cy_SCB_I2C_Disable(ft5406_config->scb_inst, ft5406_config->i2c_context);
                Cy_SCB_I2C_Enable(ft5406_config->scb_inst);
                retry_count--;
            }
        }
        else
        {
            retry_count--;
        }
    } while (retry_count);

    return i2c_status;
}

/*******************************************************************************
* Function Name: mtb_ctp_ft5406_init
********************************************************************************
* Performs FT5406 Touch panel driver initialization using I2C interface.
*
* \param *mtb_ft5406_config
* Pointer to FT5406 configuration structure
*
* \return cy_en_scb_i2c_status_t
* Initialization status.
*
* \func usage
* \snippet snippet/main.c mtb_ctp_ft5406_init
*
*******************************************************************************/
cy_en_scb_i2c_status_t mtb_ctp_ft5406_init(mtb_ctp_ft5406_config_t* mtb_ft5406_config)
{
    cy_en_scb_i2c_status_t i2c_status = CY_SCB_I2C_SUCCESS;
    uint8_t device_mode[] = { MTB_CTP_FT5406_DEVICE_MODE, MTB_CTP_FT5406_NORMAL_MODE } ;

    CY_ASSERT(NULL != mtb_ft5406_config);

    ft5406_config = mtb_ft5406_config;

    i2c_status = mtb_ctp_i2c_write_packet(device_mode, sizeof(device_mode));

    return i2c_status;
}


/*******************************************************************************
* Function Name: mtb_ctp_ft5406_read_raw_touch_data
********************************************************************************
* This function reads the raw x, y coordinate data from the touch sensor.
*
* \param *touch_data
* Pointer to the buffer to store touch data.
*
* \param size
* Size of the buffer to store touch data.
*
* \return cy_en_scb_i2c_status_t
* Read operation status based on I2C communication.
*
*******************************************************************************/
static cy_en_scb_i2c_status_t mtb_ctp_ft5406_read_raw_touch_data(uint8_t* touch_data,
                                                                 int size)
{
    cy_en_scb_i2c_status_t i2c_status = CY_SCB_I2C_SUCCESS;
    uint8_t get_touch_cmd = (uint8_t)MTB_CTP_REG_READ_TOUCH_DATA;

    CY_ASSERT(NULL != touch_data);

    i2c_status = mtb_ctp_i2c_write_packet(&get_touch_cmd, sizeof(get_touch_cmd));
    if (CY_SCB_I2C_SUCCESS == i2c_status)
    {
        i2c_status = mtb_ctp_i2c_read_packet(touch_data, size);
    }

    return i2c_status;
}


/*******************************************************************************
* Function Name: mtb_ctp_ft5406_get_single_touch
********************************************************************************
* Reads touch data from the FT5406 touch panel driver using I2C interface.
*
* \param *touch_x
* Pointer to the variable of X touch co-ordinate.
*
* \param *touch_y
* Pointer to the variable of Y touch co-ordinate
*
* \return cy_en_scb_i2c_status_t
* Single touch read operation status based on I2C communication.
*
*******************************************************************************/
cy_en_scb_i2c_status_t mtb_ctp_ft5406_get_single_touch(mtb_ctp_touch_event_t* touch_event,
                                                       int* touch_x, int* touch_y)
{
    cy_en_scb_i2c_status_t i2c_status = CY_SCB_I2C_SUCCESS;
    uint8_t touch_buff[MTB_CTP_FT5406_TOUCH_DATA_LEN] = {0xff};
    mtb_ctp_touch_event_t touch_event_local;

    i2c_status = mtb_ctp_ft5406_read_raw_touch_data(touch_buff, sizeof(touch_buff));

    if (CY_SCB_I2C_SUCCESS == i2c_status)
    {
        mtb_ctp_ft5406_touch_data_t* touch_data = (mtb_ctp_ft5406_touch_data_t*)(void*)(touch_buff);

        touch_event_local = TOUCH_POINT_GET_EVENT(touch_data->touch_points[0]);

        /* Update coordinates only if there is touch detected */
        if ((MTB_CTP_TOUCH_DOWN == touch_event_local) \
            || (MTB_CTP_TOUCH_CONTACT == touch_event_local))
        {
            if (NULL != touch_x)
            {
                *touch_x = TOUCH_POINT_GET_X(touch_data->touch_points[0]);
            }
            if (NULL != touch_y)
            {
                *touch_y = TOUCH_POINT_GET_Y(touch_data->touch_points[0]);
            }
        }

        if (NULL != touch_event)
        {
            *touch_event = touch_event_local;
        }
    }

    return i2c_status;
}


/* [] END OF FILE */
