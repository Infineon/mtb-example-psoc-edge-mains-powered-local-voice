/*******************************************************************************
* \file mtb_ctp_ft5406.h
*
* \brief
* Provides constants, parameter values, and API prototypes for the FT5406
* touch panel driver library.
*
********************************************************************************
* \copyright
* Copyright 2025, Cypress Semiconductor Corporation (an Infineon company)
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

#ifndef MTB_CTP_FT5406_H
#define MTB_CTP_FT5406_H

/*******************************************************************************
* Header Files
*******************************************************************************/

#include "cy_pdl.h"

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

/*******************************************************************************
* Macros
*******************************************************************************/

/*! @brief FT5406 maximum number of simultaneously detected touches. */
#define MTB_CTP_FT5406_MAX_TOUCHES            (5U)

/*! @brief FT5406 register address where touch data begin. */
#define MTB_CTP_FT5406_TOUCH_DATA_SUBADDR     (1)

/*! @brief FT5406 raw touch data length. */
#define MTB_CTP_FT5406_TOUCH_DATA_LEN         (32)

/* FT5406 registers */
#define MTB_CTP_FT5406_DEVICE_MODE            (0x00)
#define MTB_CTP_REG_READ_TOUCH_DATA           (0x01)


#define MTB_CTP_FT5406_NORMAL_MODE            (0x00)

/*******************************************************************************
* Data Structures
*******************************************************************************/
/* Touch events */
typedef enum
{
    MTB_CTP_TOUCH_DOWN,    /* The state changed to touched. */
    MTB_CTP_TOUCH_UP,      /* The state changed to not touched. */
    MTB_CTP_TOUCH_CONTACT, /* There is a continuous touch being detected. */
    MTB_CTP_TOUCH_RESERVED /* No touch information available. */
} mtb_ctp_touch_event_t;

/* Touch point definition */
typedef struct
{
    uint8_t XH;
    uint8_t XL;
    uint8_t YH;
    uint8_t YL;
    uint8_t reserved[2];
} mtb_ctp_ft5406_touch_point_t;

typedef struct
{
    uint8_t gesture_ID;                                                    /* Gesture ID */
    uint8_t touch_detection_count;                                         /* Touch detection count
                                                                            */
    mtb_ctp_ft5406_touch_point_t touch_points[MTB_CTP_FT5406_MAX_TOUCHES]; /* Touch point values */
} mtb_ctp_ft5406_touch_data_t;

/* FT5406 touch controller configuration structure */
typedef struct
{
    CySCB_Type* scb_inst;
    cy_stc_scb_i2c_context_t* i2c_context;
    volatile bool touch_event;
} mtb_ctp_ft5406_config_t;

/*******************************************************************************
* Function Prototypes
*******************************************************************************/
cy_en_scb_i2c_status_t mtb_ctp_ft5406_init(mtb_ctp_ft5406_config_t* mtb_ctp_ft5406_config);
cy_en_scb_i2c_status_t mtb_ctp_ft5406_get_single_touch(mtb_ctp_touch_event_t* touch_event,
                                                       int* touch_x, int* touch_y);

#if defined(__cplusplus)
}
#endif /* __cplusplus */

#endif /* MTB_CTP_FT5406_H */

/* [] END OF FILE */
