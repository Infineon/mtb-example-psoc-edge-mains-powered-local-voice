/******************************************************************************
* File Name : audio_conv_utils.h
*
* Description :
* Audio conversion utilities.
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
        uint16_t *mono);

void convert_mono_to_stereo_interleaved(
        uint16_t *stereo,
        uint16_t *mono);
        
void swap_stereo_channel(uint16_t *in, uint16_t *out);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __AUDIO_CONV_UTILS_H__ */

/* [] END OF FILE */
