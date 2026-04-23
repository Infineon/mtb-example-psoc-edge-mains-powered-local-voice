/******************************************************************************
* File Name : audio_conv_utils.h
*
* Description :
* Audio conversion utilities.
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

#ifndef __AUDIO_CONV_UTILS_H__
#define __AUDIO_CONV_UTILS_H__

#include "cy_result.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif /* __cplusplus */

/****************************************************************************
* Functions Prototypes
*****************************************************************************/

void convert_stereo_non_interleaved_to_stereo_interleaved(
        uint16_t *stereo_non_interleaved,
        uint16_t *stereo_interleaved);

void convert_interleaved_to_stereo_non_interleaved(
        uint16_t *stereo_interleaved,
        uint16_t *stereo_non_interleaved);

void convert_stereo_interleaved_to_mono(
        uint16_t *stereo,
        uint16_t *mono, int16_t samples_per_10ms);

void convert_mono_to_stereo_interleaved(
        uint16_t *stereo,
        uint16_t *mono, int16_t samples_per_10ms);
        
void swap_stereo_channel(uint16_t *in, uint16_t *out);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __AUDIO_CONV_UTILS_H__ */

/* [] END OF FILE */
