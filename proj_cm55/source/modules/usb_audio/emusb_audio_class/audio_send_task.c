/******************************************************************************
* File Name : audio_send_task.c
*
* Description :
* Code to send audio data from PSOC Edge to PC via USB audio class.
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

#include "audio_send_task.h"
#include "audio.h"
#include "audio_app.h"
#include "rtos.h"
#include "USB_Audio.h"
#include "cybsp.h"
#include "audio_usb_send_utils.h"
#include "app_logger.h"

/*******************************************************************************
* Macros
*******************************************************************************/
#ifdef DEBUG_TUNE_BATTERY_APP
#define USB_AUDIO_TX_TASK_PRIORITY        (3)
#else
#define USB_AUDIO_TX_TASK_PRIORITY        (6)
#endif /* DEBUG_TUNE_BATTERY_APP */

/*******************************************************************************
* Global Variables
*******************************************************************************/

/* PCM buffer data (16-bits) */
uint16_t audio_in_pcm_buffer_ping[MAX_AUDIO_IN_PACKET_SIZE_WORDS];
uint16_t audio_in_pcm_buffer_pong[MAX_AUDIO_IN_PACKET_SIZE_WORDS];

/* Audio IN flags */
volatile bool audio_in_is_recording    = false;
volatile bool audio_start_recording    = false;

TaskHandle_t rtos_audio_in_task;

/*******************************************************************************
* Function Name: audio_in_init
********************************************************************************
* Summary:
*   Schedules task for IN endpoint.
*
*******************************************************************************/
void audio_in_init(void)
{
    BaseType_t rtos_task_status;

    rtos_task_status = xTaskCreate(audio_in_process, "usb_audio_to_pc",
                        RTOS_STACK_DEPTH, NULL, USB_AUDIO_TX_TASK_PRIORITY,
                        &rtos_audio_in_task);

    if (pdPASS != rtos_task_status)
    {
        app_log_print("Error in creating Audio In task \r\n");
    }

}


/*******************************************************************************
* Function Name: audio_in_enable
********************************************************************************
* Summary:
*   Start a recording session.
*
*******************************************************************************/
void audio_in_enable(void)
{

    audio_start_recording = true;
}


/*******************************************************************************
* Function Name: audio_in_disable
********************************************************************************
* Summary:
*   Stop a recording session.
*
*******************************************************************************/
void audio_in_disable(void)
{
    audio_in_is_recording = false;
}


/*******************************************************************************
* Function Name: audio_in_process
********************************************************************************
* Summary:
*   Main task for the audio in endpoint. Check if should start a recording
*   session.
*
*******************************************************************************/


void audio_in_process(void *arg)
{
    (void) arg;

    USBD_AUDIO_Start_Play(usb_audioContext, NULL);

    USBD_AUDIO_Write_Task();

    while(1)
    {

    }

}

/*******************************************************************************
* Function Name: is_audio_usb_send_out_data_from_device_started
********************************************************************************
* Summary:
*   Checks if audio capture is going on.
*
*******************************************************************************/

int is_audio_usb_send_out_data_from_device_started(void)
{
    return audio_in_is_recording;
}

/*******************************************************************************
* Function Name: audio_in_endpoint_callback
********************************************************************************
* Summary:
*   Audio in endpoint callback implementation. It enables the Audio in DMA to
*   stream an audio frame.
*
*******************************************************************************/
void audio_in_endpoint_callback(void * pUserContext, const uint8_t ** ppNextBuffer, unsigned long * pNextPacketSize)
{

    static uint16_t *audio_in_pcm_buffer = NULL;
    CY_UNUSED_PARAMETER(pUserContext);
    uint16_t length = 0;

    if (audio_start_recording)
    {
        audio_start_recording = false;
        audio_in_is_recording = true;

        /* Clear Audio In buffer */
        memset(audio_in_pcm_buffer_ping, 0, (MAX_AUDIO_IN_PACKET_SIZE_BYTES));

        audio_in_pcm_buffer = audio_in_pcm_buffer_ping;

        /* Start a transfer to the Audio IN endpoint */
        *ppNextBuffer = (uint8_t *) audio_in_pcm_buffer;
        *pNextPacketSize = PACKET_SIZE_IN_MAX;
    }

    else if(audio_in_is_recording)
    {
        usb_send_out_dbg_callback((uint8_t **)ppNextBuffer, &length);
        *pNextPacketSize = length;
    }
}

/* [] END OF FILE */
