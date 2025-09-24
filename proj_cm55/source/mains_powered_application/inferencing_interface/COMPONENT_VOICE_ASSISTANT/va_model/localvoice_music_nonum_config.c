/****************************************************************************
* File Name        : localvoice_music_nonum_config.c
*
* Description      : This source file contains the configuration object for WWD and NLU
*
* Related Document : See README.md
*
*****************************************************************************
* Copyright 2023-2024, Cypress Semiconductor Corporation (an Infineon company) or
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
*****************************************************************************/

#include "localvoice_music_nonum_config.h"

#include "mtb_ml.h"
#include "mtb_ml_model_16x8.h"
#include "AM_LSTM_tflm_model_int16x8.h"

#include "ifx_va_prms.h"
#include "ifx_sp_common_priv.h"

#include "localvoice_music_nonum_U55_WWmodel.h"
#include "localvoice_music_nonum_U55_CMDmodel.h"
#include "U55_NMBmodel.h"

#include "localvoice_music_nonum.h"

/* Following am_tensor_arena has been counted as part of persistent memory total size */
/* Tensor_arena buffer must be in SOCMEM and aligned by 16 which are required by U55 */
static uint8_t am_tensor_arena[AM_LSTM_ARENA_SIZE] __attribute__((aligned(16)))
                                          __attribute__((section(".cy_socmem_data")));


static int16_t data_feed_int[N_SEQ * FEATURE_BUF_SZ] __attribute__((aligned(16)));
static float mtb_ml_input_buffer[N_SEQ * FEATURE_BUF_SZ];

static float xIn[FRAME_SIZE_16K] __attribute__((section(".wwd_nlu_data3")));
static float features[FEATURE_BUF_SZ] __attribute__((section(".wwd_nlu_data4")));
static float output_scores[(N_PHONEMES + 1) * (1 + AM_LOOKBACK)] __attribute__((section(".wwd_nlu_data5")));

//common buffers
static mtb_wwd_nlu_buff_t wwd_nlu_buff =
{
    .am_model_bin = { MTB_ML_MODEL_BIN_DATA(AM_LSTM) },
    .am_model_buffer = {
        .tensor_arena = am_tensor_arena,
        .tensor_arena_size = AM_LSTM_ARENA_SIZE
    },
    .data_feed_int = data_feed_int,
    .mtb_ml_input_buffer = mtb_ml_input_buffer,
    .output_scores = output_scores,
    .xIn = xIn,
    .features = features
};

// NLU setup array
static mtb_nlu_setup_array_t nlu_setup_array =
{
    .intent_name_list = localvoice_music_nonum_intent_name_list,
    .variable_name_list = localvoice_music_nonum_variable_name_list,
    .variable_phrase_list = localvoice_music_nonum_variable_phrase_list,
    .unit_phrase_list = localvoice_music_nonum_unit_phrase_list,
    .intent_map_array = localvoice_music_nonum_intent_map_array,
    .intent_map_array_sizes = localvoice_music_nonum_intent_map_array_sizes,
    .variable_phrase_sizes = localvoice_music_nonum_variable_phrase_sizes,
    .unit_phrase_map_array = localvoice_music_nonum_unit_phrase_map_array,
    .unit_phrase_map_array_sizes = localvoice_music_nonum_unit_phrase_map_array_sizes,
    .NUM_UNIT_PHRASES = sizeof(localvoice_music_nonum_unit_phrase_list),
};

// WW config
static mtb_wwd_conf_t ww_conf = {
    .callback.cb_for_event = CY_EVENT_SOD,
    .callback.cb_function = localvoice_music_nonum_wake_word_callback
};

// NLU config
static mtb_nlu_config_t nlu_conf = {
    .nlu_pre_silence_timeout = 2000,
    .nlu_command_timeout = 5000,
};

static mtb_wwd_nlu_config_t ww_1_conf = {
    .ww_model_ptr = localvoice_music_nonum_WWmodeldata,
    .cmd_model_ptr = localvoice_music_nonum_CMDmodeldata,
    .nmb_model_ptr = NMBmodeldata,
    .wwd_nlu_buff_data = &wwd_nlu_buff,
    .ww_conf = &ww_conf,
    .nlu_conf.nlu_config = &nlu_conf,
    .nlu_conf.nlu_variable_data = &nlu_setup_array,
};

mtb_wwd_nlu_config_t *localvoice_music_nonum_ww_nlu_configs[LOCALVOICE_MUSIC_NONUM_NO_OF_WAKE_WORD] = {&ww_1_conf
};

char *localvoice_music_nonum_ww_str[LOCALVOICE_MUSIC_NONUM_NO_OF_WAKE_WORD] = {"OK Infineon"};

const char* localvoice_music_nonum_intent_name_list[LOCALVOICE_MUSIC_NONUM_NUM_INTENTS] = {
    "play_music",
    "end_music",
    "pause_music",
    "next_track",
    "previous_track",
    "raise_volume",
    "lower_volume",
    "vol_level_0",
    "vol_level_1",
    "vol_level_2",
    "vol_level_3",
    "vol_level_4",
    "vol_level_5",
    "raise_volume_level",
    "lower_volume_level",
};

const char* localvoice_music_nonum_variable_name_list[LOCALVOICE_MUSIC_NONUM_NUM_VARIABLES] = {
};

const char* localvoice_music_nonum_variable_phrase_list[LOCALVOICE_MUSIC_NONUM_NUM_VARIABLE_PHRASES] = {
};

const char* localvoice_music_nonum_unit_phrase_list[LOCALVOICE_MUSIC_NONUM_NUM_UNIT_PHRASES] = {
    "degree", "degrees", 
    "percent", 
    "level", "levels", 
    "hour", "hours", 
    "minute", "minutes", 
    "second", "seconds", 
    "day", "days", 
    "", 
    "AM", 
    "PM", 
};

const int localvoice_music_nonum_intent_map_array[LOCALVOICE_MUSIC_NONUM_INTENT_MAP_ARRAY_TOTAL_SIZE] = {
    0, 0, // play music
    0, 0, // play track
    0, 0, // play the music
    0, 0, // play the track
    0, 0, // start music
    0, 0, // start track
    0, 0, // start the music
    0, 0, // start the track
    0, 0, // can you play music
    0, 0, // can you play track
    0, 0, // can you play the music
    0, 0, // can you play the track
    0, 0, // can you start music
    0, 0, // can you start track
    0, 0, // can you start the music
    0, 0, // can you start the track
    1, 0, // end music
    1, 0, // end track
    1, 0, // end the music
    1, 0, // end the track
    1, 0, // can you end music
    1, 0, // can you end track
    1, 0, // can you end the music
    1, 0, // can you end the track
    2, 0, // pause music
    2, 0, // pause track
    2, 0, // pause the music
    2, 0, // pause the track
    2, 0, // can you pause music
    2, 0, // can you pause track
    2, 0, // can you pause the music
    2, 0, // can you pause the track
    3, 0, // next track
    3, 0, // switch to next track
    3, 0, // go to next track
    3, 0, // can you next track
    3, 0, // can you switch to next track
    3, 0, // can you go to next track
    4, 0, // previous track
    4, 0, // switch to previous track
    4, 0, // go to previous track
    4, 0, // can you previous track
    4, 0, // can you switch to previous track
    4, 0, // can you go to previous track
    5, 0, // raise volume
    5, 0, // can you raise volume
    6, 0, // lower volume
    6, 0, // can you lower volume
    7, 0, // set volume to level zero
    7, 0, // can you set volume to level zero
    8, 0, // set volume to level one
    8, 0, // can you set volume to level one
    9, 0, // set volume to level two
    9, 0, // can you set volume to level two
    10, 0, // set volume to level three
    10, 0, // can you set volume to level three
    11, 0, // set volume to level four
    11, 0, // can you set volume to level four
    12, 0, // set volume to level five
    12, 0, // can you set volume to level five
    13, 0, // I can't hear you
    13, 0, // I cannot hear you
    13, 0, // turn up the volume please
    14, 0, // Music is too loud
    14, 0, // turn down the volume please
};

const int localvoice_music_nonum_intent_map_array_sizes[LOCALVOICE_MUSIC_NONUM_NUM_COMMANDS] = {
    2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 
    2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 
    2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 
};

const int localvoice_music_nonum_variable_phrase_sizes[LOCALVOICE_MUSIC_NONUM_NUM_VARIABLES] = {
};

const int localvoice_music_nonum_unit_phrase_map_array[LOCALVOICE_MUSIC_NONUM_UNIT_PHRASE_MAP_ARRAY_TOTAL_SIZE] = {
    0, // None
    0, // None
    0, // None
    0, // None
    0, // None
    0, // None
    0, // None
    0, // None
    0, // None
    0, // None
    0, // None
    0, // None
    0, // None
    0, // None
    0, // None
    0, // None
    0, // None
    0, // None
    0, // None
    0, // None
    0, // None
    0, // None
    0, // None
    0, // None
    0, // None
    0, // None
    0, // None
    0, // None
    0, // None
    0, // None
    0, // None
    0, // None
    0, // None
    0, // None
    0, // None
    0, // None
    0, // None
    0, // None
    0, // None
    0, // None
    0, // None
    0, // None
    0, // None
    0, // None
    0, // None
    0, // None
    0, // None
    0, // None
    0, // None
    0, // None
    0, // None
    0, // None
    0, // None
    0, // None
    0, // None
    0, // None
    0, // None
    0, // None
    0, // None
    0, // None
    0, // None
    0, // None
    0, // None
    0, // None
    0, // None
};

const int localvoice_music_nonum_unit_phrase_map_array_sizes[LOCALVOICE_MUSIC_NONUM_NUM_COMMANDS] = {
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 
};



__attribute__((weak)) void localvoice_music_nonum_wake_word_callback(mtb_wwd_nlu_events_t event)
{

}
