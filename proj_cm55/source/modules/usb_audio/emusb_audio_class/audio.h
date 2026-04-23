/******************************************************************************
* File Name : audio.h
*
* Description :
* Configuration for various sampling rates - USB audio class.
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

#ifndef AUDIO_H
#define AUDIO_H

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

/***********************************************************************
 * Macros
************************************************************************/
#define ENABLE_LARGE_FORMATS                    (0)
#define NUM_INTERFACES                          (2)
#define BUFFER_SIZE                             (1024 * 32)
#define NUM_BUFFERS                             (4)
#define MAX_TIMEOUT_COUNT                       (100)
#define READ_TIMEOUT                            (10)
#define WRITE_TIMEOUT                           (10)
#define MAX_UNHANDLED_RECEIVE                   (30)
#define MAX_SILENT_PACKET                       (2000)

/*
* Has to match the configured values in Microphone Configuration
* For a sample rate of 48000, 16 bits per sample, 2 channels:
* (48000 * ((16/8) * 2)) / 1000 = 192 bytes
* Two additional sample sizes are added to make sure we can send odd sized frames if necessary:
* 192 bytes + ((16/8) * 2) = 196

 For Microphone 16KHZ - 16000 * ((16/8) * 2) / 1000  + (16/8)*2   = 34 bytes     (2 channel)

 For Speaker 22KHZ -    16000 * ((16/8) * 2) / 1000  + (16/8)*2   = 92 bytes     (2 channels)
*/

#define PACKET_SIZE_IN_MAX                      (128)

#if ENABLE_LARGE_FORMATS
#define PACKET_SIZE_OUT_MAX                     (768 + 4)
#else
#define PACKET_SIZE_OUT_MAX                     (64)
#endif /* ENABLE_LARGE_FORMATS */

#define AUDIO_SAMPLING_RATE_48KHZ               (48000U)
#define AUDIO_SAMPLING_RATE_44KHZ               (44100U)
#define AUDIO_SAMPLING_RATE_32KHZ               (32000U)
#define AUDIO_SAMPLING_RATE_22KHZ               (22050U)
#define AUDIO_SAMPLING_RATE_16KHZ               (16000U)
#define AUDIO_SAMPLING_RATE_8KHZ                (8000U)

#define AUDIO_OUT_NUM_CHANNELS                  (2)
#define AUDIO_OUT_SUB_FRAME_SIZE                (2)    /* In bytes */
#define AUDIO_OUT_BIT_RESOLUTION                (16)
#define AUDIO_OUT_SAMPLE_FREQ                   (AUDIO_SAMPLING_RATE_16KHZ)


#define AUDIO_IN_NUM_CHANNELS                   (4) /* Quad for AE USB */


#define AUDIO_IN_SUB_FRAME_SIZE                 (2)    /* In bytes */
#define AUDIO_IN_BIT_RESOLUTION                 (16)
#define AUDIO_IN_SAMPLE_FREQ                    (AUDIO_SAMPLING_RATE_16KHZ)

/* Each report consists of 2 bytes: The report ID (0x01) and a bit mask
 containing 8 control events: */

#define AUDIO_HID_REPORT_VOLUME_UP              (0x01u)
#define AUDIO_HID_REPORT_VOLUME_DOWN            (0x02u)
#define AUDIO_HID_REPORT_PLAY_PAUSE             (0x08u)

#define ADDITIONAL_AUDIO_IN_SAMPLE_SIZE_BYTES   (((AUDIO_IN_BIT_RESOLUTION) / 8U) * (AUDIO_IN_SUB_FRAME_SIZE)) /* In bytes */

#define MAX_AUDIO_IN_PACKET_SIZE_BYTES          ((((AUDIO_IN_SAMPLE_FREQ) * (((AUDIO_IN_BIT_RESOLUTION) / 8U) * (AUDIO_IN_NUM_CHANNELS))) / 1000U) + (ADDITIONAL_AUDIO_IN_SAMPLE_SIZE_BYTES)) /* In bytes */

#define ADDITIONAL_AUDIO_IN_SAMPLE_SIZE_WORDS   ((ADDITIONAL_AUDIO_IN_SAMPLE_SIZE_BYTES) / (AUDIO_IN_SUB_FRAME_SIZE)) /* In words */

#define MAX_AUDIO_IN_PACKET_SIZE_WORDS          ((MAX_AUDIO_IN_PACKET_SIZE_BYTES) / (AUDIO_IN_SUB_FRAME_SIZE)) /* In words */


/* OUT endpoint macros */
#define ADDITIONAL_AUDIO_OUT_SAMPLE_SIZE_BYTES   (((AUDIO_OUT_BIT_RESOLUTION) / 8U) * (AUDIO_OUT_SUB_FRAME_SIZE)) /* In bytes */

#define MAX_AUDIO_OUT_PACKET_SIZE_BYTES          ((((AUDIO_OUT_SAMPLE_FREQ) * (((AUDIO_OUT_BIT_RESOLUTION) / 8U) * (AUDIO_OUT_NUM_CHANNELS))) / 1000U) + (ADDITIONAL_AUDIO_OUT_SAMPLE_SIZE_BYTES)) /* In bytes */

#define ADDITIONAL_AUDIO_OUT_SAMPLE_SIZE_WORDS   ((ADDITIONAL_AUDIO_OUT_SAMPLE_SIZE_BYTES) / (AUDIO_OUT_SUB_FRAME_SIZE)) /* In words */

#define MAX_AUDIO_OUT_PACKET_SIZE_WORDS          ((MAX_AUDIO_OUT_PACKET_SIZE_BYTES) / (AUDIO_OUT_SUB_FRAME_SIZE)) /* In words */

#if defined(__cplusplus)
}
#endif /* __cplusplus */

#endif /* AUDIO_H */

/* [] END OF FILE */

