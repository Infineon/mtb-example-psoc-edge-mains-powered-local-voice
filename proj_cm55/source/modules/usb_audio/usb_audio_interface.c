/******************************************************************************
* File Name : usb_audio_interface.c
*
* Description :
* USB interface control code.
********************************************************************************
* (c) 2025-2026, Infineon Technologies AG, or an affiliate of Infineon
* Technologies AG. All rights reserved.
* This software, associated documentation and materials ("Software") is
* owned by Infineon Technologies AG or one of its affiliates ("Infineon")
* and is protected by and subject to worldwide patent protection, worldwide
* copyright laws, and international treaty provisions. Therefore, you may use
* this Software only as provided in the license agreement accompanying the
* software package from which you obtained this Software. If no license
* agreement applies, then any use, reproduction, modification, translation, or
* compilation of this Software is prohibited without the express written
* permission of Infineon.
*
* Disclaimer: UNLESS OTHERWISE EXPRESSLY AGREED WITH INFINEON, THIS SOFTWARE
* IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
* INCLUDING, BUT NOT LIMITED TO, ALL WARRANTIES OF NON-INFRINGEMENT OF
* THIRD-PARTY RIGHTS AND IMPLIED WARRANTIES SUCH AS WARRANTIES OF FITNESS FOR A
* SPECIFIC USE/PURPOSE OR MERCHANTABILITY.
* Infineon reserves the right to make changes to the Software without notice.
* You are responsible for properly designing, programming, and testing the
* functionality and safety of your intended application of the Software, as
* well as complying with any legal requirements related to its use. Infineon
* does not guarantee that the Software will be free from intrusion, data theft
* or loss, or other breaches ("Security Breaches"), and Infineon shall have
* no liability arising out of any Security Breaches. Unless otherwise
* explicitly approved by Infineon, the Software may not be used in any
* application where a failure of the Product or any consequences of the use
* thereof can reasonably be expected to result in personal injury.
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
