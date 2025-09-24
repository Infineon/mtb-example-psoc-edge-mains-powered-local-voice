/******************************************************************************
* File Name : inferencing_interface.h
*
* Description :
* Header file for Inference processing on CM55.
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

#ifndef __INFERENCING_INTERFACE_H__
#define __INFERENCING_INTERFACE_H__

#ifdef __cplusplus
extern "C"
{
#endif /* __cplusplus */

#include "inferencing_task.h"

/*******************************************************************************
* Macros
*******************************************************************************/
#define OK_INFINEON_CMD_ID                              (101)
#define INFERENCING_TIMEOUT                             (128000)
#define WAKEWORD_CMD_ID                                 (101)

/*******************************************************************************
 * Type definitions
 ******************************************************************************/

typedef void (*inferencing_callback)(const char *function_name, char *message, char *parameter);

typedef enum cy_grp_2_asr_cmds_music
{
    PLAY_MUSIC_CMD_ID = 401,
    STOP_MUSIC_CMD_ID,
    NEXT_TRACK_CMD_ID,
    PREVIOUS_TRACK_CMD_ID,
    INCREASE_VOL_CMD_ID,
    DECREASE_VOL_CMD_ID,
    GOTO_HOMESCREEN_CMD_ID,
    PAUSE_MUSIC_CMD_ID,
    VOL_LEVEL_0_CMD_ID,
    VOL_LEVEL_1_CMD_ID,
    VOL_LEVEL_2_CMD_ID,
    VOL_LEVEL_3_CMD_ID, 
    VOL_LEVEL_4_CMD_ID,
    VOL_LEVEL_5_CMD_ID,   
} cy_grp_2_asr_cmds_music_t;


#define INF_WAKE_WORD                "WAKEWORD"

#define INTENT_PLAY_MUSIC            "play_music"
#define INTENT_STOP_MUSIC            "end_music"
#define INTENT_INCREASE_VOLUME       "raise_volume"
#define INTENT_DECREASE_VOLUME       "lower_volume"
#define INTENT_NEXT_TRACK            "next_track"
#define INTENT_PREVIOUS_TRACK        "previous_track"
#define INTENT_GOTO_HOMESCREEN       "goto_homescreen"
#define INTENT_PAUSE_MUSIC           "pause_music"

#define INTENT_SET_VOLUME_0          "vol_level_0"
#define INTENT_SET_VOLUME_1          "vol_level_1"
#define INTENT_SET_VOLUME_2          "vol_level_2"
#define INTENT_SET_VOLUME_3          "vol_level_3"
#define INTENT_SET_VOLUME_4          "vol_level_4"
#define INTENT_SET_VOLUME_5          "vol_level_5"
#define INTENT_SET_VOLUME_n          "vol_level_n"
#define INTENT_VOLUME_LEVEL_MAX      "raise_volume_level"
#define INTENT_VOLUME_LEVEL_MIN      "lower_volume_level"

#define INF_OK_INFINEON              "Okay Infineon"
#define INF_PLAY_MUSIC               "play music"
#define INF_STOP_MUSIC               "stop music"
#define INF_INCREASE_VOLUME          "increase volume"
#define INF_DECREASE_VOLUME          "decrease volume"
#define INF_NEXT_TRACK               "next track"
#define INF_PREVIOUS_TRACK           "previous track"
#define INF_GOTO_HOMESCREEN          "go to home screen"
#define INF_TIMEOUT                  "timeout"


/*******************************************************************************
* Functions Prototypes
*******************************************************************************/

int inferencing_engine_init(inferencing_callback callback_function);
void inferencing_engine_deinit();
void inference_processing(short *data, int num_samples);
int is_inference_license_valid();

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __INFERENCING_INTERFACE_H__ */

/* [] END OF FILE */
