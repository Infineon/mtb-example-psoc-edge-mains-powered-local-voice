/******************************************************************************
* File Name : audio_receive_task.c
*
* Description :
* Code to receive audio data from PC via USB audio class.
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
#include "USB_Audio.h"
#include "audio_app.h"
#include "audio.h"
#include "cycfg.h"
#include "rtos.h"
#include "audio_conv_utils.h"
#include "audio_usb_send_utils.h"
#include "audio_receive_task.h"
#include "cyabs_rtos.h"
#include "app_logger.h"
#include "audio_input_configuration.h"
/*******************************************************************************
* Macros
*******************************************************************************/
#define USB_10MS_AUDIO_SAMP            (160)
#define USB_AUDIO_RX_TASK_PRIORITY     (6)

/*******************************************************************************
* Global Variables
*******************************************************************************/

/* Audio IN flags */
volatile bool audio_out_is_streaming    = false;
volatile bool audio_start_streaming     = false;


int usb_packet_count                    = 0;
int ping_pong_buff                      = 0;
int16_t audio_mic_buffer_usb_ping[USB_10MS_AUDIO_SAMP*2] = {0};
int16_t audio_mic_buffer_usb_pong[USB_10MS_AUDIO_SAMP*2] = {0};
int16_t usb_non_interleaved_buffer[USB_10MS_AUDIO_SAMP*4]= {0};

int8_t *audio_usb_ptr                   = NULL;
int8_t *audio_usb_ref                   = NULL;
int8_t *audio_usb_mic_ptr               = NULL;

TaskHandle_t rtos_audio_out_task;
TaskHandle_t usb_audio_mic_task;


/*******************************************************************************
* Functions Prototypes
*******************************************************************************/
extern void usb_mic_data_feed(int16_t *audio_data);

/*******************************************************************************
* Function Name: audio_out_init
********************************************************************************
* Summary:
*   Initialize the audio OUT endpoint and USB mic task.
*
*******************************************************************************/
void audio_out_init(void)
{
    BaseType_t rtos_task_status;

    rtos_task_status = xTaskCreate(audio_out_process, "usb_audio_to_psoc",
                        RTOS_STACK_DEPTH, NULL, USB_AUDIO_RX_TASK_PRIORITY,
                        &rtos_audio_out_task);

    if (pdPASS != rtos_task_status)
    {
        CY_ASSERT(0);
    }

    rtos_task_status = xTaskCreate(usb_mic_task, "usb_mic_task",
                        RTOS_STACK_DEPTH*4, NULL, USB_AUDIO_RX_TASK_PRIORITY,
                        &usb_audio_mic_task);

    if (pdPASS != rtos_task_status)
    {
        CY_ASSERT(0);
    }

}

/*******************************************************************************
* Function Name: audio_out_enable
********************************************************************************
* Summary:
*   Start a playing session.
*
*******************************************************************************/
void audio_out_enable(void)
{
    USBD_AUDIO_Start_Listen(usb_audioContext, NULL);

    audio_start_streaming = true;

}

/*******************************************************************************
* Function Name: audio_out_disable
********************************************************************************
* Summary:
*   Stop a playing session.
*
*******************************************************************************/
void audio_out_disable(void)
{
    USBD_AUDIO_Stop_Listen(usb_audioContext);
    audio_out_is_streaming = false;

}

/*******************************************************************************
* Function Name: audio_out_process
********************************************************************************
* Summary:
*   Main task for the audio out endpoint. Check if it should start a playing
*   session.
*
*******************************************************************************/
void audio_out_process(void *arg)
{
    (void) arg;

    USBD_AUDIO_Start_Listen(usb_audioContext, NULL);

    app_log_print("Audio out process \r\n");
    USBD_AUDIO_Read_Task();

    while (1)
    {

    }
}

/*******************************************************************************
* Function Name: usb_mic_task
********************************************************************************
* Summary:
*   Sends audio packets received via USB to the audio pipeline.
*
*******************************************************************************/

void usb_mic_task(void *arg)
{
    int16_t usb_buffer[USB_10MS_AUDIO_SAMP*2];
    uint32_t notify_val=0;
    while (1) {
        xTaskNotifyWait(0,0,&notify_val,portMAX_DELAY);
        memcpy(usb_buffer,audio_usb_mic_ptr,USB_10MS_AUDIO_SAMP*2*2);
        convert_interleaved_to_stereo_non_interleaved((uint16_t *)usb_buffer,(uint16_t *)usb_non_interleaved_buffer);
#if AFE_INPUT_SOURCE==AFE_INPUT_SOURCE_USB       
        usb_mic_data_feed((int16_t*)usb_non_interleaved_buffer);
#endif /* AFE_INPUT_SOURCE */        
    }


}

/*******************************************************************************
* Function Name: audio_out_endpoint_callback
********************************************************************************
* Summary:
*   Audio OUT endpoint callback implementation. 
*     Populates buffers with audio data received over USB from PC.
*   Populates buffers with audio data to loopback over USB to PC.
*
*******************************************************************************/
void audio_out_endpoint_callback(void * pUserContext,
                                 int NumBytesReceived,
                                 uint8_t ** ppNextBuffer,
                                 unsigned long * pNextBufferSize)
{
    CY_UNUSED_PARAMETER(pUserContext);

    if (audio_start_streaming)
    {
        audio_start_streaming = false;
        audio_out_is_streaming = true;

        audio_usb_ptr = (int8_t*)audio_mic_buffer_usb_ping;
        ping_pong_buff= 0;
        audio_usb_ref = audio_usb_ptr;
      
        /* Start a transfer to the Audio OUT endpoint */
        *ppNextBuffer = (uint8_t *) audio_usb_ptr;

    }
    else if(audio_out_is_streaming)
    {
        if(NumBytesReceived != 0)
        {

            if (usb_packet_count<USB_10MS_AUDIO_SAMP*4) {
                usb_packet_count+=NumBytesReceived;
                audio_usb_ptr+=NumBytesReceived;
            }
            if (usb_packet_count>=USB_10MS_AUDIO_SAMP*4)
            {

/* 10ms of data is received, so notify mic task */
                audio_usb_mic_ptr=audio_usb_ref;

                xTaskNotify(usb_audio_mic_task, 0,eNoAction);

                usb_packet_count=0;

                if (ping_pong_buff==0)
                {
                    audio_usb_ptr=(int8_t*)audio_mic_buffer_usb_pong;
                    audio_usb_ref = audio_usb_ptr;
                    ping_pong_buff=1;
                }
                else
                {
                    audio_usb_ptr=(int8_t*)audio_mic_buffer_usb_ping;
                    audio_usb_ref = audio_usb_ptr;
                    ping_pong_buff=0;
                }
            }

             /* Start a transfer to OUT endpoint */
            *ppNextBuffer = (uint8_t *) audio_usb_ptr;

        }
    }

}
/* [] END OF FILE */
