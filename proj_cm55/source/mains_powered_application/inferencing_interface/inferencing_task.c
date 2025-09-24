/******************************************************************************
* File Name : inferencing_task.c
*
* Description :
* Source file for Inference task on CM55.
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
#include "cy_pdl.h"
#include "cycfg.h"
#include "cy_log.h"
#include "cyabs_rtos.h"
#include "cybsp_types.h"
#include "inferencing_task.h"
#include "control_task.h"
#include "audio_enhancement_interface.h"

#include "app_logger.h"
#include "rtos.h"
#if INFERENCING_PROFILE
#include "cy_afe_profiler.h"
#include "cy_profiler.h"
#endif /* INFERENCING_PROFILE */

/*******************************************************************************
* Macros
*******************************************************************************/

#define INFERENCING_LOG_CONTROL                         (0)

#if INFERENCING_LOG_CONTROL
#define hp_app_infer_log(format,...)                    printf(format "\r\n",##__VA_ARGS__);
#else
#define hp_app_infer_log(format,...)
#endif /* INFERENCING_LOG_CONTROL */

#define INFERENCING_TASK_PRIORITY                       (3)
#define INFER_QUEUE_SIZE                                (320)
#ifdef ITSI_INFERENCING
#define INFER_QUEUE_ELEMENTS                            (30)
#else
#define INFER_QUEUE_ELEMENTS                            (60)
#endif /* ITSI_INFERENCING */
#define INFER_FRAME_SIZE                                (320)
#define INFERENCING_STACK_SIZE                          (1024*10)

/****************************************************************************
* Global variables
*****************************************************************************/
volatile bool ptt_flag = false;
extern QueueHandle_t map_id_q;
TaskHandle_t rtos_infer_task;
QueueHandle_t infer_queue_handle;


#if INFERENCING_PROFILE
int32_t itsi_profiling_count=0;
int itsi_frames=0;
int32_t itsi_cycles=0;
#endif /* INFERENCING_PROFILE */

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
    BaseType_t ret = pdTRUE;
    ret = xQueueSend(map_id_q, &map_id, 0);
    if (pdTRUE != ret)
    {
        app_log_print(">>> Send failed to control task - Queue full \r\n");
    }
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

void mains_powered_inference_feed(char *mono)
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

    BaseType_t rtos_task_status;

    infer_queue_handle = xQueueCreate(INFER_QUEUE_ELEMENTS, INFER_QUEUE_SIZE);
    if (infer_queue_handle == NULL)
    {
         app_log_print("Init queue for inferencing queue failed \r\n");
         CY_ASSERT(0);
    }

    /* Initialize ASR  */
    if (!inferencing_engine_init(app_inferencing_callback)) {
        app_log_print("Error in Inference engine initialization \r\n");
    }

    app_log_print("Inference engine initialization completed. ");

    rtos_task_status = xTaskCreate(mains_powered_infer_task, "inferencing_task",
                        INFERENCING_STACK_SIZE, NULL, INFERENCING_TASK_PRIORITY,
                        &rtos_infer_task);

    if (pdPASS != rtos_task_status)
    {
        app_log_print("Inferencing task create failed \r\n");
        CY_ASSERT(0);
    }


    return;
}


/*******************************************************************************
* Function Name: mains_powered_infer_task
********************************************************************************
* Summary:
* Inferencing task.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/

void mains_powered_infer_task(void *arg)
{

   char infer_buffer[INFER_FRAME_SIZE]={0};
    while(1)
    {
        memset(infer_buffer, 0, INFER_FRAME_SIZE);
        if (pdTRUE == xQueueReceive(infer_queue_handle, (void*)infer_buffer, portMAX_DELAY))
        {
#if INFERENCING_PROFILE
            cy_profiler_start();

            mains_powered_inference_feed((char*)infer_buffer);

            cy_profiler_stop();
            if (itsi_frames<=100)
            {
                itsi_frames++;
                itsi_cycles=cy_profiler_get_cycles();
                itsi_profiling_count+=itsi_cycles;

            } else {
                app_log_print("Inference-Cycles %ld \r\n",(long)(itsi_profiling_count/1000000));
                itsi_frames=0;
                itsi_profiling_count=0;
            }
#else
            mains_powered_inference_feed(infer_buffer);
#endif /* INFERENCING_PROFILE */
        }
    }
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
