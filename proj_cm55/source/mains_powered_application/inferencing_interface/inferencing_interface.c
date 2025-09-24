/******************************************************************************
* File Name : inferencing_interface.c
*
* Description :
* Source file for Inference processing on CM55.
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
#include "inferencing_interface.h"

#include "app_logger.h"

#ifdef VA_INFERENCING
#include "va_inferencing.h"
#endif /* VA_INFERENCING */
#include "audio_usb_send_utils.h"


/*******************************************************************************
* Macros
*******************************************************************************/
#if defined (VA_INFERENCING)
#define VA_DIGITAL_GAIN                   (1)
#define VA_AUDIO_SAMPLES                  (160)
#endif /* VA_INFERENCING */

#define WAKE_WORD_INFER_STAGE             (1)
#define ASR_INFER_STAGE                   (2)
/*******************************************************************************
* Global Variables
*******************************************************************************/
#if defined (VA_INFERENCING)
short boosted_data[VA_AUDIO_SAMPLES] __attribute__((section(".dtcm_data"), aligned(4))) = {0};
#endif /*VA_INFERENCING*/

#if defined (VA_INFERENCING)
short inferencing_stage = 0;
int inferencing_frame_count_ww = 0;
int inferencing_frame_count_asr = 0;
inferencing_callback g_callback_function;
extern void app_inferencing_callback(const char *function, char *message, char *parameter);
#endif /* VA_INFERENCING */


/*******************************************************************************
* Function Name: inferencing_engine_init
********************************************************************************
* Summary:
*    Initializes the inferencing engine.
*
*******************************************************************************/
int inferencing_engine_init(inferencing_callback callback_function)
{

#ifdef VA_INFERENCING
    va_rslt_t status = 0;
    status = voice_assistant_init(VA_MODE_WW_SINGLE_CMD);
    if (status != VA_RSLT_SUCCESS)
    {
        app_log_print("Voice Assistant Init failed \r\n");
    }
    g_callback_function=callback_function;
#endif /* VA_INFERENCING */
    return 1;

}

/*******************************************************************************
* Function Name: inferencing_engine_deinit
********************************************************************************
* Summary:
*    De-initializes the inferencing engine.
*
*******************************************************************************/
void inferencing_engine_deinit()
{

    return;

}


/*******************************************************************************
* Function Name: inference_processing
********************************************************************************
* Summary:
*    Perform inferencing on the data.
*
*******************************************************************************/
void inference_processing(short *data, int num_samples)
{

#ifdef VA_INFERENCING
    int index = 0;
    for (index = 0; index < VA_AUDIO_SAMPLES; index++)
    {
        boosted_data[index] = data[index]*VA_DIGITAL_GAIN;
    }
    
    run_voice_assistant_process(boosted_data);
#endif /* VA_INFERENCING*/
    return;

}

/*******************************************************************************
* Function Name: is_inference_license_valid
********************************************************************************
* Summary:
*    Check license of inference library.
*
*******************************************************************************/

int is_inference_license_valid()
{
    return 1;
}

/* [] END OF FILE */
