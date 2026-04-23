/******************************************************************************
* File Name : audio_send_task.c
*
* Description :
* Code to send audio data from PSOC Edge to PC via USB audio class.
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
#define USB_AUDIO_TX_TASK_PRIORITY        (6)

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
