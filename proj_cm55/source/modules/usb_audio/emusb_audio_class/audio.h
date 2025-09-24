/******************************************************************************
* File Name : audio.h
*
* Description :
* Configuration for various sampling rates - USB audio class.
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

