/******************************************************************************
* File Name:   music_player_task.h
*
* Description: Header file for music player implemenation
*
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

#ifndef MUSIC_PLAYER_TASK_H_
#define MUSIC_PLAYER_TASK_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/*******************************************************************************
* Header Files
*******************************************************************************/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "cybsp.h"
#include "semphr.h"
#include "cyabs_rtos_internal.h"
#include "app_logger.h"

#ifdef AUDIO_OUT
#include "app_i2s.h"
#endif /* AUDIO_OUT */
/*******************************************************************************
* Macros
********************************************************************************/
/* Task priority and stack size for the music player task */
#define MUSIC_PLAYER_TASK_PRIORITY                 (3u)
#define MUSIC_PLAYER_TASK_STACK_SIZE               (3 * 1024u)

#define CHANNELS_MONO                               (1u)
#define CHANNELS_STEREO                             (2u)
#define PCM_BIT_WIDTH_16                            (16u)
#define I2S_FRAME_SIZE_SAMPLES(SAMPLE_RATE)         (SAMPLE_RATE / 100UL)  /* 10 ms frame size */
#define AEC_REF_SAMPLE_RATE_HZ                      (16000UL)

/* Maximum number of music files supported */
#define MUSIC_PLAYER_MAX_FILE_COUNT                (10UL)

#define GFX_SWITCH_TO_TRACK_ID                     (1001u)

#define LPF_ENABLE                                 (1)
#define HPF_ENABLE                                 (1)

/*******************************************************************************
* Data structure and enumeration
********************************************************************************/
/* Data-type for Music Player task commands */
typedef enum
{
    I2S_PLAYBACK_DEFAULT_STATE=0x00,        /* 0 */
    MUSIC_PLAYER_PRELOAD_DATA,              /* 1 */
    MUSIC_PLAYER_NEXT_FRAME_TRIGGER,        /* 2 */
    MUSIC_PLAYER_FLUSH_DATA,                /* 3 */

    I2S_PLAYBACK_PLAY_MUSIC,                /* 4 */
    I2S_PLAYBACK_RESUME_MUSIC,              /* 5 */
    I2S_PLAYBACK_SEEK_MUSIC,                /* 6 */
    I2S_PLAYBACK_STOP_MUSIC,                /* 7 */
    I2S_PLAYBACK_PAUSE_MUSIC,               /* 8 */
    I2S_PLAYBACK_NEXT_TRACK,                /* 9 */
    I2S_PLAYBACK_PREV_TRACK,                /* 10 */
    I2S_PLAYBACK_PLAY_TRACK_INDEX,          /* 11 */
    I2S_PLAYBACK_INC_VOL,                   /* 12 */
    I2S_PLAYBACK_DEC_VOL,                   /* 13 */
    I2S_PLAYBACK_WWD,                       /* 14 */
    I2S_PLAYBACK_SET_VOLUME_LEVEL,          /* 15 */

    MUSIC_PLAYER_SOURCE_FLASH_INIT,         /* 16 */
    MUSIC_PLAYER_SOURCE_FLASH_DEINIT,       /* 17 */

} music_player_cmd_t;

/* Data-type for Music Player task's queue data */
typedef struct
{
    music_player_cmd_t cmd;
    uint32_t data_len;
    uint8_t* data;
} music_player_q_data_t;

/* Data-type for Music Player's Audio Source */
typedef enum
{
    MUSIC_PLAYER_SOURCE_NOT_CONFIGURED,
    MUSIC_PLAYER_SOURCE_FLASH,
} music_player_audio_source_t;

/* Data-type for Music Player's Audio Output */
typedef enum
{
    MUSIC_PLAYER_OUTPUT_NOT_CONFIGURED,
    MUSIC_PLAYER_OUTPUT_I2S,
} music_player_audio_output_t;


/*******************************************************************************
* Extern Variables
********************************************************************************/
extern TaskHandle_t music_player_task_handle;
extern QueueHandle_t music_player_task_q;

extern bool music_player_active;
extern bool music_player_pause;
extern bool music_player_wwd_pause;
extern bool music_player_loop;
extern bool music_player_shuffle;
extern music_player_audio_source_t music_player_audio_source;
extern music_player_audio_output_t music_player_audio_output;
extern uint8_t music_player_volume;
extern int32_t playback_order[MUSIC_PLAYER_MAX_FILE_COUNT];
extern int32_t flash_music_file_index;

/*******************************************************************************
* Function Prototypes
********************************************************************************/
cy_rslt_t create_music_player_task(void);
uint8_t* get_pcm_buffer_ptr(uint32_t data_size_bytes);
uint8_t* get_aec_ref_buffer_ptr(uint32_t data_size_bytes);
void configure_music_player_audio_source(music_player_audio_source_t audio_source);
bool is_music_player_active(void);
bool is_music_player_paused(void);
bool is_music_player_paused_for_wwd(void);
void set_music_player_shuffle(bool enable_shuffle);
int32_t get_relative_index(int32_t absolute_file_index);

extern bool is_in_isr();
/*******************************************************************************
* Inline Function Definitions
********************************************************************************/
__STATIC_INLINE void enable_music_player(void)
{
    music_player_active = true;
    configure_music_player_audio_source(MUSIC_PLAYER_SOURCE_FLASH);
    music_player_audio_output = MUSIC_PLAYER_OUTPUT_I2S;
    // i2s_init(i2s_playback_sampling_rate_hz);
    start_i2s();
}

__STATIC_INLINE void disable_music_player(void)
{
    music_player_active = false;
    configure_music_player_audio_source(MUSIC_PLAYER_SOURCE_NOT_CONFIGURED);
    music_player_audio_output = MUSIC_PLAYER_OUTPUT_NOT_CONFIGURED;
    stop_i2s();
    // i2s_deinit();
}

__STATIC_INLINE void set_music_player_loop(bool enable_loop)
{
    music_player_loop = enable_loop;
    app_log_print(" [%s]: Loop = %d\n", __FUNCTION__, music_player_loop);
}

#ifdef __cplusplus
} /* extern C */
#endif /* __cplusplus */

#endif /* MUSIC_PLAYER_TASK_H_ */

/* [] END OF FILE */
