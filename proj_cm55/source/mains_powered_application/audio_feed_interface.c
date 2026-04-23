/******************************************************************************
* File Name : mic_data_feed.c
*
* Description :
* Feeds data received via PDM ISR/USB ISR to the audio pipeline on CM55
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

#include "audio_conv_utils.h"
#include "audio_enhancement_interface.h"
#include "cybsp_types.h"
#include "audio_usb_send_utils.h"
#include "app_logger.h"
#include "audio_input_configuration.h"

#ifdef AUDIO_OUT
#include "app_i2s.h"
#include "music_player_task.h"
#endif /* AUDIO_OUT */
#include "inferencing_task.h"


/*******************************************************************************
* Macros
*******************************************************************************/

#define MAX_SIZE_FOR_NON_INTERLEAVE_STEREO_IN_BYTES             (160*2*2)
#define FRAME_SIZE_MS                                           (10u)
#define FRAME_SIZE                                              (320u)
#define PLAYBACK_DATA_FRAME_SIZE                                (2*FRAME_SIZE)

/*******************************************************************************
* Global Variables
*******************************************************************************/

int16_t non_interleaved_audio_ping[MAX_SIZE_FOR_NON_INTERLEAVE_STEREO_IN_BYTES] = {0};
int16_t non_interleaved_audio_pong[MAX_SIZE_FOR_NON_INTERLEAVE_STEREO_IN_BYTES] = {0};

int16_t swap_audio[MAX_SIZE_FOR_NON_INTERLEAVE_STEREO_IN_BYTES]= { 0 };

int16_t aec_ref_buffer[MONO_AUDIO_DATA_IN_BYTES] = { 0 };

int16_t* actual_aec_ref = aec_ref_buffer;

int8_t buff_toggle_flag = 0;
int16_t* non_interleaved_audio = NULL;

/*******************************************************************************
* Function Name: usb_mic_data_feed
********************************************************************************
* Summary:
* API that receives audio data from mic (USB) and sends it to M55 audio pipeline.
*
* Parameters:
*  audio_data - Audio data
*
* Return:
*  None
*
*******************************************************************************/

void usb_mic_data_feed(int16_t *audio_data)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;
    int16_t* aec_reference = NULL;

 /* Feed the data to AFE */
    result = mains_powered_ae_feed((int16_t*)audio_data, aec_reference);

    if(CY_RSLT_SUCCESS != result)
    {
        app_log_print("Failed to feed audio frame to AFE \r\n");
    }
}

/*******************************************************************************
* Function Name: audio_mic_data_feed_cm55
********************************************************************************
* Summary:
* Receive PDM mic frames and feed it to the audio pipeline.
*
* Parameters:
*  audio_data - Pointer to audio buffer.
* Return:
*  None
*
*******************************************************************************/
void audio_mic_data_feed_cm55(int16_t *audio_data)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;
    int16_t* aec_reference = NULL;
   
    uint8_t bulk_delay_equivalent_frames = 0;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

#ifndef ENABLE_IFX_AEC
    actual_aec_ref = NULL;
#else
    if (uxQueueMessagesWaiting(aec_ref_data_q) >= bulk_delay_equivalent_frames)
    {
                if (pdTRUE != xQueueReceiveFromISR(aec_ref_data_q, &aec_reference, &xHigherPriorityTaskWoken))
        {
            actual_aec_ref = NULL;
        }
        else
        {
            actual_aec_ref = aec_reference;
        }
                portYIELD_FROM_ISR(xHigherPriorityTaskWoken);    
    }
    else
    {
        actual_aec_ref = NULL;
    }
#endif /* ENABLE_IFX_AEC */

    if (buff_toggle_flag == 0)
    {
        non_interleaved_audio = non_interleaved_audio_ping;
        buff_toggle_flag = 1;

    } else if (buff_toggle_flag == 1)
    {
        non_interleaved_audio = non_interleaved_audio_pong;
        buff_toggle_flag = 0;
    }
    

#ifdef ENABLE_STEREO_INPUT_FEED
    
    convert_interleaved_to_stereo_non_interleaved((uint16_t *)audio_data, (uint16_t *)non_interleaved_audio);
    
    /* Feed the data to AFE */
    result = mains_powered_ae_feed((int16_t*)non_interleaved_audio, actual_aec_ref);
#else
/* Feed the data to AFE */
    result = mains_powered_ae_feed(audio_data, actual_aec_ref);
#endif /* ENABLE_STEREO_INPUT_FEED */

    if(CY_RSLT_SUCCESS != result)
    {
        app_log_print("Failed to feed audio frame to AFE \r\n ");
    }
}


/* [] END OF FILE */


