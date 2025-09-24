/******************************************************************************
* File Name : audio_enhancement_interface.c
*
* Description :
* Wrapper for Audio Enhancement
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

#include "audio_enhancement_interface.h"
#include "inferencing_task.h"

#include "app_logger.h"
#include "audio_usb_send_utils.h"



/*******************************************************************************
* Macros
*******************************************************************************/


/*******************************************************************************
* Global Variables
*******************************************************************************/

extern QueueHandle_t infer_queue_handle;

/*******************************************************************************
* Function Name: license_limitation_exit
********************************************************************************
* Summary:
*  This function is blocking call after Audio Front End license timeout
*
*******************************************************************************/
void license_limitation_exit()
{
    while(1){}
}


void audio_enhancement_process_output(ae_buffer_info_t *output_buffer)
{
    
    BaseType_t ret = pdTRUE;


    int16_t *output_dgb1 = (int16_t *)output_buffer->dbg_output1;
    int16_t *output_dgb2 = (int16_t *)output_buffer->dbg_output2;
    int16_t *output_dgb3 = (int16_t *)output_buffer->dbg_output3;
    int16_t *output_dgb4 = (int16_t *)output_buffer->dbg_output4;

#if AE_APP_PROFILE
    cy_afe_profile(AFE_PROFILE_CMD_PRINT_STATS_1SEC, NULL);
    cy_afe_profile(AFE_PROFILE_CMD_RESET, NULL);
#endif /* AE_APP_PROFILE */

        usb_send_out_dbg_put(USB_CHANNEL_1,(int16_t *)output_dgb1);
        usb_send_out_dbg_put(USB_CHANNEL_2,(int16_t *)output_dgb2);
        usb_send_out_dbg_put(USB_CHANNEL_3,(int16_t *)output_dgb3);
        usb_send_out_dbg_put(USB_CHANNEL_4,(int16_t *)output_dgb4);


    if (infer_queue_handle !=NULL)
    {
        ret = xQueueSend(infer_queue_handle, (void*)output_buffer->output_buf, 0);
    }
    if (pdTRUE != ret)
    {
        app_log_print(">>> Send failed to inferencing task - Queue full \r\n");
    }
}


/*******************************************************************************
* Function Name: mains_powered_ae_init
********************************************************************************
* Summary:
* Initialize Audio Enhancement for mains powered application 
*
* Parameters:
*  channels - number of input channels
* 
* Return:
*  Result of AE initialization.
*
*******************************************************************************/

int mains_powered_ae_init(int channels)
{

    ae_rslt_t result = AE_RSLT_SUCCESS;

    result = audio_enhancement_init(channels);
    
    if (result != AE_RSLT_SUCCESS) 
    {
        app_log_print("DEEPCRAFT Audio Enhancement initialization failed \r\n");
    }
    else
    {
        app_log_print("DEEPCRAFT Audio Enhancement initialized \r\n");
    }
#if AE_APP_PROFILE
    cy_profiler_init();
    cy_afe_profile(AFE_PROFILE_CMD_ENABLE,NULL);
#endif /* AE_APP_PROFILE */
    
    return result;
}

/*******************************************************************************
* Function Name: mains_powered_ae_feed
********************************************************************************
* Summary:
*  Feed mic audio data and AEC reference to Audio Enhancement.
*
*******************************************************************************/

int mains_powered_ae_feed(void* audio_input, void* aec_buffer)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    result = audio_enhancement_feed_input((int16_t*)audio_input, (int16_t*)aec_buffer);
    
    if (AE_RSLT_LICENSE_ERROR == result)
    {
        app_log_print("CPU Halt: Audio Enhancement Restricted License Timeout - Reset the board \r\n");
        license_limitation_exit();
        return result;
    }

    if(AE_RSLT_SUCCESS != result)
    {
        app_log_print("Failed to feed data to AE \r\n");
        return result;
    }
    return result;
}
/* [] END OF FILE */
