/******************************************************************************
* File Name : audio_enhancement_interface.h
*
* Description :
* Header file for DEEPCRAFT Audio Enhancement wrapper interface
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

/*******************************************************************************
 * Function Prototypes
 *******************************************************************************/

int mains_powered_ae_feed(void* audio_input, void* aec_buffer);
int mains_powered_ae_init(int);

#ifdef __cplusplus
} /*extern "C" */
#endif  /* __cplusplus */
#endif /* __AUDIO_ENHANCEMENT_INTERFACE_H__ */

/* [] END OF FILE */
