/******************************************************************************
* File Name : usb_audio_interface.c
*
* Description :
* USB interface control code.
********************************************************************************
* Copyright 2025, Cypress Semiconductor Corporation (an Infineon company) or
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
*******************************************************************************/

/*******************************************************************************
* Header Files
*******************************************************************************/
#include "usb_audio_interface.h"
#include "cy_pdl.h"
#include "cybsp.h"
#include "cy_log.h"
#include "rtos.h"
#include "audio_app.h"
#include "app_logger.h"

/*******************************************************************************
* Macros
*******************************************************************************/

#define USB_INTERFACE_TASK_PRIORITY         (4)

/*******************************************************************************
* Global Variables
*******************************************************************************/
TaskHandle_t audio_usb_task;

/*******************************************************************************
* Function Name: cy_audio_usb_interface_init
********************************************************************************
* Summary:
* Create USB interface thread.
*
* Parameters:
*  None
*
* Return:
*  CY_RSLT_SUCCESS
*
*******************************************************************************/
cy_rslt_t usb_audio_interface_init()
{
    BaseType_t rtos_task_status;
    /* Create the RTOS tasks */

    rtos_task_status = xTaskCreate(audio_app_process, "usb_interface",
                        RTOS_STACK_DEPTH*4, NULL, USB_INTERFACE_TASK_PRIORITY,
                        &audio_usb_task);

    if (pdPASS != rtos_task_status)
    {
        app_log_print("Error in creating USB audio task \r\n");
    }

    return CY_RSLT_SUCCESS;
}

/*******************************************************************************
* Function Name: cy_audio_usb_interface_deinit
********************************************************************************
* Summary:
* Delete USB interface thread.
*
* Parameters:
*  None
*
* Return:
*  CY_RSLT_SUCCESS
*
*******************************************************************************/
cy_rslt_t usb_audio_interface_deinit() 
{
    vTaskDelete((TaskHandle_t)&audio_usb_task);
    return CY_RSLT_SUCCESS;
}
/* [] END OF FILE */
