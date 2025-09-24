/*******************************************************************************
* File Name        : lv_port_indev.c
*
* Description      : This file provides implementation of low level input device
*                    driver for LVGL.
*
* Related Document : See README.md
*
*******************************************************************************
* Copyright 2025-2025, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
******************************************************************************/

/*******************************************************************************
* Header Files
*******************************************************************************/
#include "lv_port_indev.h"
#include "lv_port_disp.h"
#include "cy_utils.h"
#include "app_i2s.h"
#if defined(MTB_CTP_GT911)
#include "mtb_ctp_gt911.h"
#elif defined(MTB_CTP_ILI2511)
#include "mtb_ctp_ili2511.h"
#endif
#include "cybsp.h"

#include "mtb_ctp_ft5406.h"
#include "lv_port_disp.h"
#include "lv_indev_private.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "music_player_gfx_task.h"

/*****************************************************************************
* Macros
*****************************************************************************/
#if defined(MTB_CTP_ILI2511)
#define CTP_RESET_PORT       GPIO_PRT17
#define CTP_RESET_PIN        (3U)
#define CTP_IRQ_PORT         GPIO_PRT17
#define CTP_IRQ_PIN          (2U)
#endif


/*******************************************************************************
* Global Variables
*******************************************************************************/

mtb_ctp_ft5406_config_t ft5406_config =
{
    .scb_inst    = CYBSP_I2C_CONTROLLER_HW,
    .i2c_context = &disp_touch_i2c_controller_context
};


/*******************************************************************************
* Function Name: touchpad_init
********************************************************************************
* Summary:
*  Initialization function for touchpad supported by LittelvGL.
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
static void touchpad_init(void)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    xSemaphoreTake(i2c_semaphore, portMAX_DELAY);
    result = (cy_rslt_t)mtb_ctp_ft5406_init(&ft5406_config);
    xSemaphoreGive(i2c_semaphore);

    if (CY_RSLT_SUCCESS != result)
    {
        CY_ASSERT(0);
    }
}


/*******************************************************************************
* Function Name: touchpad_read
********************************************************************************
* Summary:
*  Touchpad read function called by the LVGL library.
*  Here you will find example implementation of input devices supported by
*  LittelvGL:
*   - Touchpad
*   - Mouse (with cursor support)
*   - Keypad (supports GUI usage only with key)
*   - Encoder (supports GUI usage only with: left, right, push)
*   - Button (external buttons to press points on the screen)
*
*   The `..._read()` function are only examples.
*   You should shape them according to your hardware.
*
*
* Parameters:
*  *indev_drv: Pointer to the input driver structure to be registered by HAL.
*  *data: Pointer to the data buffer holding touch coordinates.
*
* Return:
*  void
*
*******************************************************************************/
static void touchpad_read(lv_indev_t *indev_drv, lv_indev_data_t *data)
{
    static int touch_x = 0;
    static int touch_y = 0;
    mtb_ctp_touch_event_t touch_event;
    cy_rslt_t result = CY_RSLT_SUCCESS;

    data->state = LV_INDEV_STATE_REL;

    xSemaphoreTake(i2c_semaphore, portMAX_DELAY);
    result = (cy_rslt_t)mtb_ctp_ft5406_get_single_touch(&touch_event, &touch_x, &touch_y);
    xSemaphoreGive(i2c_semaphore);

    if ((CY_RSLT_SUCCESS == result) && ((touch_event == MTB_CTP_TOUCH_DOWN) || (touch_event == MTB_CTP_TOUCH_CONTACT)))
    {
        data->state = LV_INDEV_STATE_PR;
    }

    /* Set the last pressed coordinates */
    data->point.x = (MY_DISP_HOR_RES - 32 - 1) - touch_x;
    data->point.y = (MY_DISP_VER_RES - 1) - touch_y;
}


/*******************************************************************************
* Function Name: lv_port_indev_init
********************************************************************************
* Summary:
*  Initialization function for input devices supported by LittelvGL.
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void lv_port_indev_init(void)
{
    /* Initialize your touchpad if you have. */
    touchpad_init();
    /* Register a touchpad input device */
    lv_indev_t * indev = lv_indev_create();
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(indev, touchpad_read);
    lv_timer_pause(indev->read_timer);
    lv_timer_reset(indev->read_timer);
    lv_timer_set_period(indev->read_timer, 90);
    lv_timer_resume(indev->read_timer);
}


/* [] END OF FILE */
