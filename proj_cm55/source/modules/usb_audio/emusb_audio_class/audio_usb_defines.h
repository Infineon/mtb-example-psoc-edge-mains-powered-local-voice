/******************************************************************************
* File Name : audio_usb_defines.h
*
* Description :
* USB audio class configuration macros.
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
#ifndef AUDIO_USB_DEFINES_H
#define AUDIO_USB_DEFINES_H

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

/*******************************************************************************
* Macros
*******************************************************************************/
/* Constants from USB Audio Descriptor */

#define AUDIO_OUT_ENDPOINT_SIZE             (294U)
#define AUDIO_IN_ENDPOINT_SIZE              (294U)
#define AUDIO_FEEDBACK_ENDPOINT_SIZE        (3U)

#define AUDIO_FRAME_DATA_SIZE               (64u)
#define AUDIO_DELTA_VALUE                   (4u)
#define AUDIO_MAX_DATA_SIZE                 (AUDIO_FRAME_DATA_SIZE + AUDIO_DELTA_VALUE)

#define AUDIO_CONTROL_INTERFACE             (0x00U)
#define AUDIO_CONTROL_IN_ENDPOINT           (6U)
#define AUDIO_CONTROL_FEATURE_UNIT_IDX      (0x02U)
#define AUDIO_CONTROL_FEATURE_UNIT          ((AUDIO_CONTROL_FEATURE_UNIT_IDX << 8U) | (AUDIO_CONTROL_INTERFACE))

#define AUDIO_STREAMING_OUT_INTERFACE       (1U)
#define AUDIO_STREAMING_OUT_ALTERNATE       (1U)
#define AUDIO_STREAMING_IN_INTERFACE        (2U)
#define AUDIO_STREAMING_IN_ALTERNATE        (1U)

#define AUDIO_STREAMING_OUT_ENDPOINT        (1U)
#define AUDIO_STREAMING_IN_ENDPOINT         (2U)
#define AUDIO_FEEDBACK_IN_ENDPOINT          (3U)

#define AUDIO_STREAMING_OUT_ENDPOINT_ADDR   (0x01U)
#define AUDIO_STREAMING_IN_ENDPOINT_ADDR    (0x82U)
#define AUDIO_FEEDBACK_IN_ENDPOINT_ADDR     (0x83U)

#define AUDIO_FEED_SINGLE_SAMPLE            (0x000800U)

#define AUDIO_STREAMING_EPS_NUMBER          (0x2U)
#define AUDIO_SAMPLE_FREQ_SIZE              (3U)
#define AUDIO_SAMPLE_DATA_SIZE              (2U)

#define AUDIO_FEATURE_UNIT_MASTER_CHANNEL   (0U)

#define AUDIO_HID_ENDPOINT                  (0x4u)
#define AUDIO_HID_REPORT_SIZE               (1u)
#define AUDIO_HID_REPORT_VOLUME_UP          (0x20u)
#define AUDIO_HID_REPORT_VOLUME_DOWN        (0x40u)
#define AUDIO_HID_REPORT_PLAY_PAUSE         (0x01u)
#define AUDIO_HID_REPORT_STOP               (0x08u)

#define AUDIO_VOLUME_SIZE                   (2U)

#define AUDIO_VOL_RES_MSB                   (0x00u)
#define AUDIO_VOL_RES_LSB                   (0x01u)

#define AUDIO_SAMPLING_RATE_48KHZ           (48000U)
#define AUDIO_SAMPLING_RATE_44KHZ           (44100U)
#define AUDIO_SAMPLING_RATE_32KHZ           (32000U)
#define AUDIO_SAMPLING_RATE_22KHZ           (22050U)
#define AUDIO_SAMPLING_RATE_16KHZ           (16000U)

#if defined(__cplusplus)
}
#endif /* __cplusplus */

#endif /* AUDIO_USB_DEFINES_H */

/* [] END OF FILE */
