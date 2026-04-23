/******************************************************************************
* File Name : inferencing_interface.c
*
* Description :
* Source file for Inference processing on CM55.
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
#include "inferencing_interface.h"

#include "app_logger.h"

#ifdef VA_INFERENCING
#include "va_inferencing.h"
#endif /* VA_INFERENCING */
#include "audio_usb_send_utils.h"

#if INFERENCING_PROFILE
#include "cy_afe_profiler.h"
#include "cy_profiler.h"
#endif /* INFERENCING_PROFILE */


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

#if INFERENCING_PROFILE
int32_t va_profiling_count=0;
int va_frames=0;
int32_t va_cycles=0;
#endif /* INFERENCING_PROFILE */


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
#if INFERENCING_PROFILE
            cy_profiler_start();

            run_voice_assistant_process(boosted_data);

            cy_profiler_stop();
            if (va_frames<=100)
            {
                va_frames++;
                va_cycles=cy_profiler_get_cycles();
                va_profiling_count+=va_cycles;

            } else {
                app_log_print("Inference-Cycles %ld \r\n",(long)(va_profiling_count/1000000));
                va_frames=0;
                va_profiling_count=0;
            }
#else
    run_voice_assistant_process(boosted_data);
#endif /* INFERENCING_PROFILE */

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
