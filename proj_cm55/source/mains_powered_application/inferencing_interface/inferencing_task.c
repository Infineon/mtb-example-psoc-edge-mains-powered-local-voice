/******************************************************************************
* File Name : inferencing_task.c
*
* Description :
* Source file for Inference task on CM55.
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
#include "cy_pdl.h"
#include "cycfg.h"
#include "cy_log.h"
#include "cyabs_rtos.h"
#include "cybsp_types.h"
#include "inferencing_task.h"
#include "control_task.h"
#include "audio_enhancement_interface.h"

#include "app_logger.h"

/*******************************************************************************
* Macros
*******************************************************************************/

#define INFERENCING_LOG_CONTROL                         (0)

#if INFERENCING_LOG_CONTROL
#define hp_app_infer_log(format,...)                    printf(format "\r\n",##__VA_ARGS__);
#else
#define hp_app_infer_log(format,...)
#endif /* INFERENCING_LOG_CONTROL */

#define INFER_FRAME_SIZE                                (320)

/****************************************************************************
* Global variables
*****************************************************************************/
volatile bool ptt_flag = false;
extern QueueHandle_t map_id_q;

/*******************************************************************************
* Function Name: app_inferencing_callback
********************************************************************************
* Summary:
* Callback function for inferencing library.
* This function is used to print outputs from inferencing library API.
*
* Parameters:
*  function - Function from which callback is called.
*  message - Message to print.
*  parameter - Parameter passed in callback from inferencing library API.
*
* Return:
*  None
*
*******************************************************************************/

void app_inferencing_callback(const char *function, char *message, char *parameter)
{

    if (!strcmp(message,"Timeout")) 
    {
        app_log_print("Wake Word Not Detected -Time out \r\n");
    }
    return;
}

/*******************************************************************************
* Function Name: post_process_m55_only_inference
********************************************************************************
* Summary:
* Post Processing API for Inference outputs.
* Based on the inferred map id, the necessary action will be taken.
*
* Parameters:
*  map_id - Inferred Map ID.
*
* Return:
*  None
*
*******************************************************************************/
void mains_powered_post_processing(int map_id)
 {
#if AFE_INPUT_SOURCE==AFE_INPUT_SOURCE_MIC    
    BaseType_t ret = pdTRUE;
    ret = xQueueSend(map_id_q, &map_id, 0);
    if (pdTRUE != ret)
    {
        app_log_print(">>> Send failed to control task - Queue full \r\n");
    }
#endif /* AFE_INPUT_SOURCE */    
}
/*******************************************************************************
* Function Name: mains_powered_inference_feed
********************************************************************************
* Summary:
* Feeds audio frames from audio pipeline to inferencing engine.
*
* Parameters:
*  stereo - Audio data.
* 
* Return:
*  None
*
*******************************************************************************/

void mains_powered_inference_feed(short *mono)
{
    inference_processing((short *)mono, INFER_FRAME_SIZE/2);
    return;
}

/*******************************************************************************
* Function Name: mains_powered_inference_engine_init
********************************************************************************
* Summary:
* Initializes inferencing library.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/

void mains_powered_inference_engine_init()
{

    /* Initialize ASR  */
    if (!inferencing_engine_init(app_inferencing_callback)) {
        app_log_print("Error in Inference engine initialization \r\n");
    }

    app_log_print("Inference engine initialization completed. ");
 
    return;
}


/*******************************************************************************
* Function Name: mains_powered_inference_engine_deinit
********************************************************************************
* Summary:
* De-initializes inferencing library.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/

void mains_powered_inference_engine_deinit()
{
    inferencing_engine_deinit();
    return;
}

/* [] END OF FILE */
