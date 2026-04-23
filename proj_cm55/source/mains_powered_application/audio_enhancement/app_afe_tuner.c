/******************************************************************************
* File Name : app_afe_tuner.c
*
* Description :
* Source file for Audio Front End tuning.
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

#ifdef CY_AFE_ENABLE_TUNING_FEATURE

/*******************************************************************************
* Header Files
*******************************************************************************/

#include "cybsp.h"
#include "cy_pdl.h"
#include "audio_enhancement.h"
#include "cy_retarget_io.h"
#include "cy_result.h"
#include "cy_log.h"
#include "audio_usb_send_utils.h"
#include "pdm_mic_interface.h"
#include "app_logger.h"
#include "app_i2s.h"

/*******************************************************************************
* Macros
*******************************************************************************/
#define UART_READ_TIMEOUT_MS             (1u)

/*******************************************************************************
* Global Variables
*******************************************************************************/
int cmd_index = 0;

int8_t gain_change=0;

/*******************************************************************************
* Extern Variables
*******************************************************************************/

int stored_input_gain=0;

/*******************************************************************************
* Function Name: audio_enhancement_tuner_notify
********************************************************************************
* Summary:
*  Tuner notification API for AFE middleware.
*
* Parameters:
*  handle - AFE handle
*  config_setting - Configuration settings.
* Return:
*  AE_RSLT_SUCCESS
*
*******************************************************************************/

cy_rslt_t audio_enhancement_tuner_notify(cy_afe_t handle, cy_afe_config_setting_t *config_setting)
{

    int16_t input_gain_factor = 0 ;
    int* data = NULL;
    cy_afe_tuner_buffer_t response_buffer;
    float gain_db =0.0;
    char pdm_error[]= "AFERSP,Valid range is [-20dB to 40dB] on EVK\r\n";

    if(CY_AFE_READ_CONFIG == config_setting->action)
    {
        if(CY_AFE_CONFIG_INPUT_GAIN == config_setting->config_name)
        {
            data = (int*) config_setting->value;
        
            *data = stored_input_gain;

           if (stored_input_gain==0 && gain_change==0)
           {
                *data=AFE_MIC_INPUT_GAIN_DB*2;
                stored_input_gain=*data;
           }
           app_ae_log("Final gain value read is %d \r\n",*data); 
        }
    }
    else if (CY_AFE_UPDATE_CONFIG == config_setting->action)
    {
        if(CY_AFE_CONFIG_INPUT_GAIN == config_setting->config_name)
        {
            gain_change=1;
            data = (int*) config_setting->value;
            app_ae_log("Update input gain config. Input gain from AFE configurator : %d", *data);
            gain_db=*data/2;

#ifdef GAIN_CONTROL_ON  
            if (gain_db>=AFE_MIN_MIC_INPUT_HW_GAIN && gain_db<=AFE_MAX_MIC_INPUT_HW_GAIN)
#else                     
            if (gain_db>=PDM_PCM_MIN_GAIN && gain_db<=PDM_PCM_MAX_GAIN)
#endif /* GAIN_CONTROL_ON */            
            {
                app_ae_log("Setting input gain to %f \r\n",gain_db);
                input_gain_factor=convert_db_to_pdm_scale(gain_db);
                app_ae_log("Setting input scale to %d \r\n",input_gain_factor);
                set_pdm_pcm_gain((int16_t)input_gain_factor);
                stored_input_gain=*data;
            }
            else
            {
                app_log_print("PDM Gain out of bounds \r\n");
                response_buffer.buffer=(uint8_t*)pdm_error;
                response_buffer.length=strlen(pdm_error);
                
                return CY_RSLT_AFE_TUNER_HW_INPUT_GAIN_OUT_OF_RANGE;
            
            }
             
        }
        else if(CY_AFE_CONFIG_STREAM == config_setting->config_name)
        {
            data = (int*) config_setting->value;
            app_ae_log("data is %d \r\n",*data);
        }
    }
    else if (CY_AFE_NOTIFY_CONFIG == config_setting->action)
    {

        if(CY_AFE_CONFIG_BULK_DELAY_CALC_START == config_setting->config_name)
        {
                        return AE_RSLT_FAIL; /* No AEC calibration for Local Voice Music Player*/
           
        }
        else if(CY_AFE_CONFIG_BULK_DELAY_CALC_STOPPED == config_setting->config_name)
        {
            /* No AEC calibation in this CE.*/
            /* Refer to Audio Enhancement Code Example with the same file name for bulk delay calibration*/
        }
     
    }

    return AE_RSLT_SUCCESS;

}

/*******************************************************************************
* Function Name: audio_enhancement_tuner_read
********************************************************************************
* Summary:
*  Callback for AFE tuning read operation.
*
* Parameters:
*  handle - AFE handle
*  request_buffer - Buffer with read data.
* Return:
*  AE_RSLT_SUCCESS
*
*******************************************************************************/

cy_rslt_t audio_enhancement_tuner_read(cy_afe_tuner_buffer_t *request_buffer)
{
    uint32_t read_value = Cy_SCB_UART_Get(CYBSP_DEBUG_UART_HW);
    if (CY_SCB_UART_RX_NO_DATA == read_value)
    {
        vTaskDelay(pdMS_TO_TICKS(UART_READ_TIMEOUT_MS));
        read_value = Cy_SCB_UART_Get(CYBSP_DEBUG_UART_HW);
    }

    if (CY_SCB_UART_RX_NO_DATA != read_value)
    {
        request_buffer->buffer[0] = (uint8_t)read_value;
        request_buffer->length = 1;
    }
    else
    {
        request_buffer->length = 0;
        request_buffer->buffer = NULL;
    }
    return AE_RSLT_SUCCESS;
}

/*******************************************************************************
* Function Name: audio_enhancement_tuner_write
********************************************************************************
* Summary:
*  Callback for Audio Enhancment tuning write operation.
*
* Parameters:
*  response_buffer - Buffer with write data.
*  user_arg - User argument.
* Return:
*  AE_RSLT_SUCCESS
*
*******************************************************************************/

cy_rslt_t audio_enhancement_tuner_write(cy_afe_tuner_buffer_t *response_buffer)
{
    Cy_SCB_UART_PutArrayBlocking(CYBSP_DEBUG_UART_HW, response_buffer->buffer, response_buffer->length);
    return AE_RSLT_SUCCESS;
}
#endif /* CY_AFE_ENABLE_TUNING_FEATURE */

/* [] END OF FILE */
