/******************************************************************************
* File Name : voice_assistant.h
*
* Description :
* Header for the DEEPCRAFT voice assistant (VA)
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

#ifndef _VOICE_ASSISTANT_H_
#define _VOICE_ASSISTANT_H_

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

#include "mtb_wwd_nlu_common.h"
#include "mtb_nlu.h"
#include "mtb_wwd.h"

/******************************************************************************
 * Macros
 *****************************************************************************/
#define VA_NLU_MAX_NUM_VARIABLES   4u

/******************************************************************************
 * Typedefs
 *****************************************************************************/
typedef enum
{
    VA_RSLT_SUCCESS = 0,
    VA_RSLT_FAIL = -1,
    VA_RSLT_INVALID_ARGUMENT = -2,
    VA_RSLT_LICENSE_ERROR = -3,
} va_rslt_t;

 typedef enum
{
    VA_NO_EVENT = 0,
    VA_EVENT_WW_DETECTED = 1,
    VA_EVENT_WW_NOT_DETECTED = 2,
    VA_EVENT_CMD_DETECTED = 3,
    VA_EVENT_CMD_TIMEOUT = 4,
    VA_EVENT_CMD_SILENCE_TIMEOUT = 5,
} va_event_t;

typedef enum
{
    VA_MODE_WW_SINGLE_CMD = 0,
    VA_MODE_WW_MULTI_CMD = 1,
    VA_MODE_WW_ONLY = 2,
    VA_MODE_CMD_ONLY = 3,
} va_mode_t;

typedef enum
{
    VA_RUN_WWD = 0,
    VA_RUN_CMD = 1,
} va_run_state_t;

/******************************************************************************
 * Structures
 ******************************************************************************/
typedef struct
{
    int     intent_index;
    int     num_var;
    mtb_nlu_variable_t variable[VA_NLU_MAX_NUM_VARIABLES];
} va_data_t;

/*******************************************************************************
 * Function Prototypes
 *******************************************************************************/
va_rslt_t voice_assistant_init(va_mode_t mode);
void      voice_assistant_change_state(va_run_state_t state);
va_rslt_t voice_assistant_process(int16_t *audio_frame, va_event_t *event, va_data_t *va_data);
va_rslt_t voice_assistant_get_command(char *text);

#if defined(__cplusplus)
}
#endif /* __cplusplus */

#endif /* _VOICE_ASSISTANT_H_ */

/* [] END OF FILE */
