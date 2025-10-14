/******************************************************************************
* File Name : audio_enhancement_interface.h
*
* Description :
* Header file for DEEPCRAFT Audio Enhancement wrapper interface
********************************************************************************
* (c) 2025, Infineon Technologies AG, or an affiliate of Infineon
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

#ifndef __AUDIO_ENHANCEMENT_INTERFACE_H__
#define __AUDIO_ENHANCEMENT_INTERFACE_H__

#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */
#include "audio_enhancement.h"
#include "cy_audio_front_end_error.h"
#include "cy_afe_configurator_settings.h"
#ifdef PROFILER_ENABLE
#include "cy_afe_profiler.h"
#include "cy_profiler.h"
#endif /* PROFILER_ENABLE */

/*******************************************************************************
* Macros
*******************************************************************************/

#ifdef PROFILER_ENABLE
#define AE_APP_PROFILE                                (0)
#define INFERENCING_PROFILE                           (0)
#endif /* PROFILER_ENABLE */

/******************************************************************************
 * Structures
 ******************************************************************************/
/* Output buffer structure for filtered audio data - USB */
typedef struct
{
    int16_t input_buf[MONO_AUDIO_DATA_IN_BYTES]; /* Input buffer pointer */
    int16_t input_aec_ref_buf[MONO_AUDIO_DATA_IN_BYTES]; /* AEC buffer pointer which was passed during feed_input() call */
    int16_t output_buf[MONO_AUDIO_DATA_IN_BYTES]; /* Output buffer pointer */
#ifdef CY_AFE_ENABLE_TUNING_FEATURE
    int16_t dbg_output1[MONO_AUDIO_DATA_IN_BYTES]; /* Debug output1 based on configuration */
    int16_t dbg_output2[MONO_AUDIO_DATA_IN_BYTES]; /* Debug output2 based on configuration */
    int16_t dbg_output3[MONO_AUDIO_DATA_IN_BYTES]; /* Debug output3 based on configuration */
    int16_t dbg_output4[MONO_AUDIO_DATA_IN_BYTES]; /* Debug output4 based on configuration */
#endif
 } ae_buffer_usb_t;

/*******************************************************************************
 * Function Prototypes
 *******************************************************************************/

int mains_powered_ae_feed(void* audio_input, void* aec_buffer);
int mains_powered_ae_init(int);
void mains_powered_ae_usb_task(void *arg);

#ifdef __cplusplus
} /*extern "C" */
#endif  /* __cplusplus */
#endif /* __AUDIO_ENHANCEMENT_INTERFACE_H__ */

/* [] END OF FILE */
