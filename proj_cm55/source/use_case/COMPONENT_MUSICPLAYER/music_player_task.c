/******************************************************************************
* File Name:   music_player_task.c
*
* Description: This file contains music player implementation
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

#include "cy_pdl.h"
#include "cybsp.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"
#include "cy_retarget_io.h"
#include "app_logger.h"

#include "music_player_task.h"
#ifdef ENABLE_GFX_UI
#include "music_player_gfx_task.h"
#endif /* ENABLE_GFX_UI */
#include "asc_wav_header_parser.h"

#include "IFX_asrc.h"
#include "app_i2s.h"


#ifdef ENABLE_MP3_PLAYBACK
    #include "cy_audio_sw_codecs.h"
    #include "cy_audio_sw_codec_utils.h"
    #include "cy_audio_sw_mp3_decoder.h"
    #include "mp3_seek.h"

    #include "borrtex_the_city_of_hope.mp3.h"
    #include "campagna_right_side_of_the_bed.mp3.h"
    #include "solar_body_about_the_memories.mp3.h"
    #include "wonderland_star_night.mp3.h"
    #include "wonderland_work.mp3.h"
#else
    #include "borrtex_the_city_of_hope.wav.h"
    #include "campagna_right_side_of_the_bed.wav.h"
    #include "wonderland_star_night.wav.h"
#endif /* ENABLE_MP3_PLAYBACK */

/******************************************************************************
* Macros
******************************************************************************/
#define WAV_HEADER_SIZE_BYTES               (44u)
#define MUSIC_PLAYER_TASK_QUEUE_LENGTH      (20u)

#define I2S_PREFILL_FRAMES_COUNT            (10u)

#define PCM_DATA_BUFFER_SIZE                ((uint32_t)(16 * 512) * (I2S_PREFILL_FRAMES_COUNT + 3u))
#define AEC_REF_BUFFER_SIZE                 ((uint32_t)(5 * 160 * sizeof(int16_t)) * (I2S_PREFILL_FRAMES_COUNT + 3u))
#define ASRC_EXPECTED_OUTPUT_SIZE(IN_SIZE,  \
                                  IN_FS,    \
                                  OUT_FS)   ((IN_SIZE) * (OUT_FS) / (IN_FS))


#define FADE_IN_TIMER_PERIOD_MS             (10u)
#define FADE_IN_TOTAL_DURATION_MS           (150u)
#define FADE_IN_STEPS                       (FADE_IN_TOTAL_DURATION_MS / FADE_IN_TIMER_PERIOD_MS)

#define FADE_OUT_TIMER_PERIOD_MS            (5u)
#define FADE_OUT_TOTAL_DURATION_MS          (50u)
#define FADE_OUT_STEPS                      (FADE_OUT_TOTAL_DURATION_MS / FADE_OUT_TIMER_PERIOD_MS)

#define FADE_OUT_TO_FADE_IN_DELAY_MS        (50u)

#ifdef ENABLE_MP3_PLAYBACK
    /* Buffer size and limits for the PCM data decoded from MP3. */
    #define MP3_DECODED_BUFFER_SIZE         (480 * sizeof(int16_t) * 30 * 2)
    #define MP3_DECODED_BUFFER_SIZE_LIMIT   (MP3_DECODED_BUFFER_SIZE - (2 * 480 * sizeof(int16_t)))

    /* Number of frames to be decoded at once */
    #define MP3_DECODER_FRAME_COUNT         (1u)

    /* Frames to be skipped during first decoding post seeking */
    #define MP3_SEEK_SKIP_FRAMES_COUNT      (75u)

#endif /* ENABLE_MP3_PLAYBACK */

/*******************************************************************************
* Global Variables
********************************************************************************/
/* Music Player task handle */
TaskHandle_t music_player_task_handle;

/* Handle of the queue holding the Music Player task commands. */
QueueHandle_t music_player_task_q;

static i2s_playback_q_data_t i2s_playback_q_data;

/* Variable to receive the queue data for this task. */

music_player_audio_source_t music_player_audio_source = MUSIC_PLAYER_SOURCE_NOT_CONFIGURED;
music_player_audio_output_t music_player_audio_output = MUSIC_PLAYER_OUTPUT_NOT_CONFIGURED;

/* I2S queue handle holding audio playback data */
extern QueueHandle_t i2s_playback_task_q;

/* Used to indicate Music player state */
bool music_player_active = false;
bool music_player_pause = false;
bool music_player_wwd_pause = false;
bool music_player_loop = false;
bool music_player_shuffle = false;

/* Flash Music Player variables */
static const uint8_t *flash_music_ptr[] =
{
#ifdef ENABLE_MP3_PLAYBACK
    (const uint8_t *) borrtex_the_city_of_hope,
    (const uint8_t *) campagna_right_side_of_the_bed,
    (const uint8_t *) solar_body_about_the_memories,
    (const uint8_t *) wonderland_star_night,
    (const uint8_t *) wonderland_work,
#else
    (const uint8_t *) borrtex_the_city_of_hope,
    (const uint8_t *) campagna_right_side_of_the_bed,
    (const uint8_t *) wonderland_star_night,
#endif /* ENABLE_MP3_PLAYBACK */
};

static unsigned int flash_music_sizes[] =
{
#ifdef ENABLE_MP3_PLAYBACK
    borrtex_the_city_of_hope_size,
    campagna_right_side_of_the_bed_size,
    solar_body_about_the_memories_size,
    wonderland_star_night_size,
    wonderland_work_size,
#else
    borrtex_the_city_of_hope_size,
    campagna_right_side_of_the_bed_size,
    wonderland_star_night_size,
#endif /* ENABLE_MP3_PLAYBACK */
};

/* Holds PCM sample index which will be sent to i2s playback task to play */
static int32_t flash_music_file_index = 0;
static int32_t flash_music_relative_file_index = 0;

/* Calculate number of music files */
static int32_t flash_music_file_index_max = sizeof(flash_music_sizes) / sizeof(flash_music_sizes[0]);

/* Static buffer for PCM data */
static uint8_t pcm_data_buffer[PCM_DATA_BUFFER_SIZE] __attribute__ ((section(".cy_socmem_data"), aligned(4)))  = {0};
static uint32_t pcm_data_buffer_index = 0;

/* Common Music Player Variables */
static uint8_t *music_player_ptr = NULL;
static int32_t music_player_index = 0;
static int32_t music_player_index_max;
static uint32_t music_player_num_channels = 0;
uint32_t music_player_sampling_rate = 0;
uint8_t music_player_volume;
static uint8_t music_player_restore_volume;

int32_t playback_order[MUSIC_PLAYER_MAX_FILE_COUNT] = { 0 };

/* Static buffer for AEC Reference Data */
static uint8_t aec_ref_buffer[AEC_REF_BUFFER_SIZE] __attribute__ ((section(".cy_socmem_data"), aligned(4)))  = {0};
static uint32_t aec_ref_buffer_index = 0;

IFX_ASRC_STRUCT_t aec_ref_asrc_obj;
IFX_ASRC_STRUCT_t playback_asrc_obj;

/* Definition of buffers for ASRC, AEC and I2S */
static int32_t asrc_in_buf_32bit[3 * 4 * MAX_FRAME_SIZE]        __attribute__ ((section(".cy_socmem_data"), aligned(4))) = { 0 };
static int32_t asrc_out_buf_32bit[ASRC_OUTPUT_BUFFER_SIZE]      __attribute__ ((section(".cy_socmem_data"), aligned(4))) = { 0 };
static int16_t asrc_in_temp_buf_16bit[3 * 4 * MAX_FRAME_SIZE]   __attribute__ ((section(".cy_socmem_data"), aligned(4))) = { 0 };

static uint32_t output_sampling_freq = 0;

static TimerHandle_t fade_out_timer = NULL;
static TimerHandle_t fade_in_timer = NULL;

static uint8_t fade_out_start_volume = 0;
static uint8_t fade_in_target_volume = 0;
static uint8_t fade_step_count = 0;
static bool fade_out_triggers_fade_in = false;
static bool fading_active = false;

#ifdef ENABLE_MP3_PLAYBACK
    /* Static buffer for decoded PCM data */
    static uint8_t mp3_decoded_buffer[MP3_DECODED_BUFFER_SIZE] __attribute__ ((section(".cy_socmem_data")));

    static cy_audio_sw_codec_handle_t mp3_decoder_handle;
    static cy_audio_sw_codec_stream_info_t mp3_stream_info;
    static cy_audio_sw_codec_decode_config_t mp3_decoder_config =
    {
        .codec_type = CY_AUDIO_CODEC_TYPE_MP3,
        .sampling_rate = CY_AUDIO_CODEC_SAMPLING_RATE_UNKNOWN,
        .channel = CY_AUDIO_CODEC_CHANNEL_UNKNOWN,
        .bitwidth = CY_AUDIO_CODEC_BITWIDTH_UNKNOWN,
        .callback_user_arg = NULL,
        .stream_info = NULL,
    };

    static uint32_t mp3_recommended_out_size;
    static uint32_t mp3_decoded_buffer_index = 0;
    static uint32_t mp3_decoded_buffer_size = 0;
    static uint32_t prev_i2s_buffer_index = 0;
    static int32_t mp3_decoded_data_in_buffer = 0;
    static uint32_t curr_mp3_file_total_samples = 0;
    static uint32_t skip_frames = 0;
#endif /* ENABLE_MP3_PLAYBACK */

/*******************************************************************************
* Function Prototypes
********************************************************************************/
static void music_player_task(void* pvParameters);
#ifndef ENABLE_MP3_PLAYBACK
static bool process_music_player_wav_file(uint8_t *wav_ptr, uint32_t wav_size);
#endif /* ENABLE_MP3_PLAYBACK */
static bool process_music_player_relative_file_index(int32_t relative_file_index);
static bool process_music_player_absolute_file_index(int32_t absolute_file_index);
#ifdef ENABLE_MP3_PLAYBACK
static uint8_t* get_mp3_decoded_data(void);
#endif /* ENABLE_MP3_PLAYBACK */
static void fade_out_timer_callback(TimerHandle_t xTimer);
static void fade_in_timer_callback(TimerHandle_t xTimer);
void create_fade_timers(void);
void start_fade_out(bool start_fade_in_after);
void start_fade_in(void);
static void read_and_send_music_player_data(uint32_t tx_frame_count);
static void notify_music_player_outputs(bool enable_fade_in);
static void process_music_player_track_change(int32_t relative_file_index);
static void initialize_asrc(void);
static uint32_t perform_asrc(int16_t* input_ptr, uint32_t input_samples, bool stereo_audio,
                             int16_t* output_ptr, bool enable_stereo_output,
                             IFX_ASRC_STRUCT_t* asrc_obj);
#ifndef ENABLE_MP3_PLAYBACK
static int32_t get_wav_seek_offset(const uint8_t *wav_data, uint32_t wav_size, uint8_t seek_percent);
#endif /*ENABLE_MP3_PLAYBACK */
/*******************************************************************************
* Function Name: create_music_player_task
********************************************************************************
* Summary:
*  Function that creates the music player task.
*
* Parameters:
*  None
*
* Return:
*  CY_RSLT_SUCCESS upon successful creation of the task, else a non-zero value
*   that indicates the error.
*
*******************************************************************************/
cy_rslt_t create_music_player_task(void)
{
    BaseType_t status;

    status = xTaskCreate(music_player_task, "Music Player Task", MUSIC_PLAYER_TASK_STACK_SIZE,
                         NULL, MUSIC_PLAYER_TASK_PRIORITY, &music_player_task_handle);

    return (status == pdPASS) ? CY_RSLT_SUCCESS : (cy_rslt_t) status;
}

/*******************************************************************************
* Function Name: music_player_task
********************************************************************************
* Summary:
*  Music player task implementation which handles:
*      - MP3 decoding and data buffering
*      - Sending data to I2S playback based on WW/ASR command
*      - Initialize music player source (Flash)
*
* Parameters:
*  void *pvParameters : Task parameter defined during task creation (unused)
*
* Return:
*  None
*
*******************************************************************************/
static void music_player_task(void* pvParameters)
{
    /* Remove warning for unused parameter */
    (void)pvParameters;

    /* Status variable to indicate the result of various operations */
#ifdef ENABLE_MP3_PLAYBACK
    cy_rslt_t result;
#endif /* ENABLE_MP3_PLAYBACK */

    music_player_q_data_t music_player_q_data;
    uint32_t tx_q_count = 0;
    uint32_t seek_percent = 0;
    uint32_t seek_offset = 0;

    music_player_volume = volume_level[DEFAULT_VOLUME_INDEX];
    music_player_restore_volume = volume_level[DEFAULT_VOLUME_INDEX];

    /* Create a Receive message queue (to i2s control) between control tasks, music player task and i2s playback task. */
    music_player_task_q = xQueueCreate(MUSIC_PLAYER_TASK_QUEUE_LENGTH, sizeof(music_player_q_data_t));

#ifdef ENABLE_MP3_PLAYBACK
    /* Initialize MP3 Decoder*/
    result = cy_audio_sw_codec_init(CY_AUDIO_CODEC_OPERATION_DECODE,
                                    &mp3_decoder_config,
                                    (cy_audio_sw_codec_t *) &mp3_decoder_handle);
    if (CY_RSLT_SUCCESS != result)
    {
        app_log_print(" >> Error: MP3 decoder init failed with error code: 0x%0X\n", result);
        CY_ASSERT(0);
    }
#endif /* ENABLE_MP3_PLAYBACK */

    create_fade_timers();

    /* Initialize the playback_order array */
    for (uint32_t i = 0; i < flash_music_file_index_max; i++)
    {
        playback_order[i] = i;
    }

    /* Set the sampling rate for I2S playback */
    output_sampling_freq = SAMPLING_RATE_48kHz;
    i2s_playback_sampling_rate_hz = output_sampling_freq;

    i2s_init(i2s_playback_sampling_rate_hz);

    for (;;)
    {
        if (pdTRUE == xQueueReceive(music_player_task_q, &music_player_q_data, portMAX_DELAY))
        {
            switch(music_player_q_data.cmd)
            {
                case I2S_PLAYBACK_PLAY_MUSIC:
                {
                    if (!is_music_player_active())
                    {
                        if (music_player_pause)
                        {
                            app_log_print(" Resuming playing file '%d'...\n", flash_music_file_index);
                            start_i2s();
                            tx_q_count = I2S_PREFILL_FRAMES_COUNT;
                            music_player_active = true;
                            music_player_pause = false;
                            music_player_wwd_pause = false;
                            // start_fade_in();
                            start_fade_out(true);
                        }
                        else
                        {
                            app_log_print(" Playing file '%d'...\n", flash_music_file_index);
                            enable_music_player();
                            // start_fade_in();
                            start_fade_out(true);
                            break;
                        }
                    }
                }

                case I2S_PLAYBACK_RESUME_MUSIC:
                {
                    if (!is_music_player_active())
                    {
                        // notify_music_player_outputs(true);
                        music_player_active = !music_player_active;
                        if(music_player_pause)
                        {
                            music_player_pause = false;
                            music_player_wwd_pause = false;
                            // start_fade_in();
                            start_fade_out(true);
                        }
                        tx_q_count = 1;
                    }
                }

                case MUSIC_PLAYER_NEXT_FRAME_TRIGGER:
                {
                    /* Request for single frame from I2S playback task */
                    if (tx_q_count == 0)
                    {
                        tx_q_count = 1;
                    }
                }

                case MUSIC_PLAYER_PRELOAD_DATA:
                {
                    /* Request for prefill number of frames */
                    if (tx_q_count == 0)
                    {
                        tx_q_count = I2S_PREFILL_FRAMES_COUNT;
                    }
                }

                /* Music Playback implementation common to Play Music, Resume Music,
                 * Next Frame Trigger and Preload Data scenarios.
                 */
                {
                    read_and_send_music_player_data(tx_q_count);
                    tx_q_count = 0;

                    break;
                }

                case I2S_PLAYBACK_WWD:
                {
                    if(music_player_active)
                    {
                        if (!fading_active)
                        {
                            music_player_restore_volume = music_player_volume;
                        }
                        start_fade_out(false);
                        music_player_pause = true;
                        music_player_wwd_pause = true;
                    }
                    music_player_active = false;
                    break;
                }

                case I2S_PLAYBACK_SEEK_MUSIC:
                {
                    if ((MUSIC_PLAYER_SOURCE_NOT_CONFIGURED == music_player_audio_source) ||
                        (!is_music_player_active()) || (NULL == music_player_q_data.data) ||
                        (sizeof(uint8_t) != music_player_q_data.data_len))
                    {
                        break;
                    }

                    seek_percent = *((uint8_t*)music_player_q_data.data);

                    if ((seek_percent >= 0) && (seek_percent <= 100))
                    {
                        /*app_log_print(" >> Received Seek percent: %d\n", seek_percent); */

#ifdef ENABLE_MP3_PLAYBACK
                        seek_offset = get_mp3_seek_offset(music_player_ptr,
                                                          music_player_index_max,
                                                          seek_percent);
#else
                        seek_offset = get_wav_seek_offset(music_player_ptr,
                                                          music_player_index_max,
                                                          seek_percent);
#endif /* ENABLE_MP3_PLAYBACK */
                        if (seek_offset <= 0)
                        {
                            app_log_print("Seek offset error: %u\n", seek_offset);
                            break;
                        }
                        /*app_log_print("    Seek offset: %u\n", seek_offset);*/

                        xQueueReset(music_player_task_q);
                        xQueueReset(i2s_playback_task_q);
                        xQueueReset(aec_ref_data_q);

                        if(music_player_pause)
                        {
                            music_player_pause = false;
                            music_player_wwd_pause = false;
                        }

#ifdef ENABLE_MP3_PLAYBACK
                        mp3_decoded_buffer_index = 0;
                        prev_i2s_buffer_index = 0;
                        mp3_decoded_data_in_buffer = 0;
                        skip_frames = MP3_SEEK_SKIP_FRAMES_COUNT;
#endif /* ENABLE_MP3_PLAYBACK */

                        music_player_index = (int32_t) seek_offset;
                        if (!fading_active)
                        {
                            music_player_restore_volume = music_player_volume;
                        }
                        start_fade_out(true);

                        tx_q_count = 0;
                        music_player_q_data.cmd = MUSIC_PLAYER_PRELOAD_DATA;
                        music_player_q_data.data_len = 0;
                        music_player_q_data.data = NULL;
                        if (pdTRUE != xQueueSend(music_player_task_q, &music_player_q_data, portMAX_DELAY))
                        {
                            app_log_print(">>> Error: Next track - Music Player Task Queue Send failed*******************\r\n");
                            break;
                        }
                    }
                    break;
                }

                case I2S_PLAYBACK_INC_VOL:
                {
                    if (music_player_q_data.data_len != -1)
                    {
                        volume_level_index ++;
                        if (volume_level_index >= NUM_VOLUME_STEPS)
                        {
                            volume_level_index = NUM_VOLUME_STEPS - 1;
                        }
                    }

                    music_player_restore_volume = volume_level[volume_level_index];

                    if (music_player_restore_volume > MAX_VOLUME)
                    {
                        music_player_restore_volume = MAX_VOLUME;
                    }
                    // music_player_restore_volume = music_player_restore_volume;

                    app_log_print("Volume index '%d', absolute value = '%d'\n",
                           volume_level_index, music_player_restore_volume);

                    if(MUSIC_PLAYER_SOURCE_NOT_CONFIGURED == music_player_audio_source)
                    {
                        music_player_volume = music_player_restore_volume;
                        i2s_playback_volume_control(music_player_volume);
                    }
                    else
                    {
                        if (music_player_wwd_pause)
                        {
                            notify_music_player_outputs(true);
                        }
                        else if (!is_music_player_active() && music_player_pause)
                        {
                            start_fade_in();
                        }
                        music_player_volume = music_player_restore_volume;
                        i2s_playback_volume_control(music_player_volume);
                    }
                    break;
                }
                case I2S_PLAYBACK_DEC_VOL:
                {
                    if (music_player_q_data.data_len != -1)
                    {
                        if (volume_level_index > 0)
                        {
                            volume_level_index --;
                        }
                    }

                    music_player_restore_volume = volume_level[volume_level_index];

                    app_log_print("Volume index '%d', absolute value = '%d'\n",
                           volume_level_index, music_player_restore_volume);

                    if (MUSIC_PLAYER_SOURCE_NOT_CONFIGURED == music_player_audio_source)
                    {
                        music_player_volume = music_player_restore_volume;
                        i2s_playback_volume_control(music_player_volume);
                    }
                    else
                    {
                        if (music_player_wwd_pause)
                        {
                            notify_music_player_outputs(true);
                        }
                        else if (!is_music_player_active() && music_player_pause)
                        {
                            start_fade_in();
                        }
                        music_player_volume = music_player_restore_volume;
                        i2s_playback_volume_control(music_player_volume);
                    }
                    break;
                }
                case I2S_PLAYBACK_SET_VOLUME_LEVEL:
                {
                    if (music_player_q_data.data_len == -1)
                    {
                        *((uint8_t*)music_player_q_data.data) = volume_level_index;
                    }
                    else if ((music_player_q_data.data_len != sizeof(uint8_t)) || (NULL == music_player_q_data.data))
                    {
                        app_log_print(">>> Error: Invalid data length or pointer for volume level command\n");
                        break;
                    }

                    uint8_t req_volume_level = *((uint8_t*)music_player_q_data.data);
                    if (req_volume_level >= NUM_VOLUME_STEPS)
                    {
                        app_log_print(">>> Error: Invalid volume level requested: %d\n", req_volume_level);
                        break;
                    }
                    volume_level_index = req_volume_level;
                    music_player_restore_volume = volume_level[volume_level_index];

                    app_log_print("Volume index '%d', absolute value = '%d'\n",
                           volume_level_index, music_player_restore_volume);

                    if(MUSIC_PLAYER_SOURCE_NOT_CONFIGURED == music_player_audio_source)
                    {
                        music_player_volume = music_player_restore_volume;
                        i2s_playback_volume_control(music_player_volume);
                    }
                    else
                    {
                        if (music_player_wwd_pause)
                        {
                            notify_music_player_outputs(true);
                        }
                        else if (!is_music_player_active() && music_player_pause)
                        {
                            start_fade_in();
                        }
                        music_player_volume = music_player_restore_volume;
                        i2s_playback_volume_control(music_player_volume);
                    }
                    break;
                }

                case MUSIC_PLAYER_FLUSH_DATA:
                {
                    break;
                }

                case I2S_PLAYBACK_PAUSE_MUSIC:
                {
                    xQueueReset(music_player_task_q);
                    xQueueReset(i2s_playback_task_q);
                    xQueueReset(aec_ref_data_q);

                    if (is_music_player_active())
                    {
                        if (!fading_active)
                        {
                            music_player_restore_volume = music_player_volume;
                        }
                        music_player_pause = true;
                        music_player_wwd_pause = false;
                    }
                    tx_q_count = 0;
                    start_fade_out(false);
                    // disable_music_player();
                    music_player_active = false;
                    stop_i2s();
                    break;
                }

                case I2S_PLAYBACK_STOP_MUSIC:
                {
                    xQueueReset(music_player_task_q);
                    xQueueReset(i2s_playback_task_q);
                    xQueueReset(aec_ref_data_q);

                    notify_music_player_outputs(false);
                    // music_player_restore_volume = music_player_volume;
                    start_fade_out(false);

                    music_player_pause = false;
                    music_player_wwd_pause = false;
                    tx_q_count = 0;
                    disable_music_player();

                    music_player_ptr = NULL;
                    music_player_index = 0;
                    music_player_index_max = 0;

                    break;
                }

                case I2S_PLAYBACK_NEXT_TRACK:
                {
                    process_music_player_track_change(1);
                    tx_q_count = 0;
                    break;
                }

                case I2S_PLAYBACK_PREV_TRACK:
                {
                    process_music_player_track_change(-1);
                    tx_q_count = 0;
                    break;
                }

                case I2S_PLAYBACK_PLAY_TRACK_INDEX:
                {
                    if ((NULL == music_player_q_data.data) ||
                        (sizeof(uint8_t) != music_player_q_data.data_len) ||
                        (*((uint8_t*)music_player_q_data.data) >= flash_music_file_index_max))
                    {
                        app_log_print(">>> Error: Invalid track index data received!\n");
                        break;
                    }

                    xQueueReset(music_player_task_q);
                    // xQueueReset(i2s_playback_task_q);

                    // if (is_music_player_active())
                    // {
                    //     if (!fading_active)
                    //     {
                    //         music_player_restore_volume = music_player_volume;
                    //     }
                    // }
                    // notify_music_player_outputs(false);
                    start_fade_out(true);

#ifdef ENABLE_MP3_PLAYBACK
                    mp3_decoded_buffer_index = 0;
                    prev_i2s_buffer_index = 0;
                    mp3_decoded_data_in_buffer = 0;
#endif /* ENABLE_MP3_PLAYBACK */

                    if(MUSIC_PLAYER_SOURCE_NOT_CONFIGURED == music_player_audio_source)
                    {
                        flash_music_file_index = *((uint8_t*)music_player_q_data.data);
                        flash_music_relative_file_index = get_relative_index(flash_music_file_index);

                        app_log_print(" Playing file '%d'...\n", flash_music_file_index);
                        enable_music_player();
                        break;
                    }
                    else
                    {
                        if (!is_music_player_active() && music_player_pause)
                        {
                            start_i2s();
                            music_player_active = true;
                            music_player_pause = false;
                            music_player_wwd_pause = false;
                        }

                        flash_music_file_index = *((uint8_t*)music_player_q_data.data);
                        flash_music_relative_file_index = get_relative_index(flash_music_file_index);
                        music_player_ptr = NULL;
                        music_player_index = 0;

                        if (!process_music_player_absolute_file_index(flash_music_file_index))
                        {
                            app_log_print(">>> Error: process_music_player_absolute_file_index failed\n");
                            break;
                        }
                        app_log_print(" Playing file '%d'...\n", flash_music_file_index);
                    }

                    tx_q_count = 0;
                    music_player_q_data.cmd = MUSIC_PLAYER_PRELOAD_DATA;
                    music_player_q_data.data_len = 0;
                    music_player_q_data.data = NULL;
                    if (pdTRUE != xQueueSend(music_player_task_q, &music_player_q_data, portMAX_DELAY))
                    {
                        app_log_print(">>> Error: Next track - Music Player Task Queue Send failed\n");
                        break;
                    }
                    break;
                }

                case MUSIC_PLAYER_SOURCE_FLASH_INIT:
                {
                    music_player_audio_source = MUSIC_PLAYER_SOURCE_FLASH;

                    flash_music_file_index_max = sizeof(flash_music_sizes) / sizeof(flash_music_sizes[0]);

                    if (!process_music_player_absolute_file_index(flash_music_file_index))
                    {
                        app_log_print(">>> Error: process_music_player_absolute_file_index failed\n");
                        break;
                    }

                    tx_q_count = 0;
                    music_player_q_data.cmd = MUSIC_PLAYER_PRELOAD_DATA;
                    music_player_q_data.data_len = 0;
                    music_player_q_data.data = NULL;
                    if (pdTRUE != xQueueSend(music_player_task_q, &music_player_q_data, portMAX_DELAY))
                    {
                        app_log_print(">>> Error: Flash Init - Music Player Task Queue Send failed*******************\r\n");
                        break;
                    }
                    break;
                }

                case MUSIC_PLAYER_SOURCE_FLASH_DEINIT:
                {
                    music_player_audio_source = MUSIC_PLAYER_SOURCE_NOT_CONFIGURED;
                    break;
                }

                case I2S_PLAYBACK_DEFAULT_STATE:
                {
                    break;
                }
            }
        }
    }
}

/* ****************************************************************************
 * Function Name: configure_music_player_audio_source
 ******************************************************************************
 * Summary:
 *  This function configures music player source (FLASH).
 *
 * Parameters:
 *  audio_source: Audio source is Flash
 *
 * Return:
 *  None
 *
 * ***************************************************************************/
void configure_music_player_audio_source(music_player_audio_source_t audio_source)
{
    music_player_q_data_t music_player_q_data;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    music_player_q_data.cmd = 0;
    music_player_q_data.data_len = 0;
    music_player_q_data.data = NULL;

    if (music_player_audio_source == audio_source)
    {
        return;
    }

    /* Deinit previous music player source */
    if (music_player_audio_source == MUSIC_PLAYER_SOURCE_FLASH)
    {
        music_player_q_data.cmd = MUSIC_PLAYER_SOURCE_FLASH_DEINIT;
    }

    if (music_player_q_data.cmd > 0)
    {
        if (is_in_isr())
        {
            if (pdTRUE != xQueueSendToFrontFromISR(music_player_task_q, &music_player_q_data, &xHigherPriorityTaskWoken))
            {
                app_log_print(">>> Error: Music Player Task Queue Send from ISR failed !\r\n");
            }
            else
            {
                portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            }
        }
        else
        {
            if (pdTRUE != xQueueSendToFront(music_player_task_q, &music_player_q_data, portMAX_DELAY))
            {
                app_log_print(">>> Error: Music Player Task Queue Send failed !\r\n");
            }
        }
    }

    /* Init required music player source */
    music_player_q_data.cmd = 0;
    music_player_q_data.data_len = 0;
    music_player_q_data.data = NULL;

    if (audio_source == MUSIC_PLAYER_SOURCE_FLASH)
    {
        music_player_q_data.cmd = MUSIC_PLAYER_SOURCE_FLASH_INIT;
    }

    if (music_player_q_data.cmd > 0)
    {
        if (is_in_isr())
        {
            if (pdTRUE != xQueueSendToFrontFromISR(music_player_task_q, &music_player_q_data, &xHigherPriorityTaskWoken))
            {
                app_log_print(">>> Error: Music Player Task Queue Send from ISR failed !\r\n");
            }
            else
            {
                portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            }
        }
        else
        {
            if (pdTRUE != xQueueSendToFront(music_player_task_q, &music_player_q_data, portMAX_DELAY))
            {
                app_log_print(">>> Error: Music Player Task Queue Send failed !\r\n");
            }
        }
    }

}

/*******************************************************************************
* Function Name: is_music_player_active
********************************************************************************
* Summary:
* Returns music player state.
*
* Parameters:
*  None
*
* Return:
*  bool : Music player status - true for active, false for inactive
*
*******************************************************************************/
bool is_music_player_active(void)
{
    return music_player_active;
}

/*******************************************************************************
* Function Name: is_music_player_paused
********************************************************************************
* Summary:
* Returns music player pause state.
*
* Parameters:
*  None
*
* Return:
*  bool : Music player pause status - true for paused, false for not paused
*
*******************************************************************************/
bool is_music_player_paused(void)
{
    return music_player_pause;
}

/*******************************************************************************
* Function Name: is_music_player_paused
********************************************************************************
* Summary:
* Returns music player WWD pause state.
*
* Parameters:
*  None
*
* Return:
*  bool : Music player WWD pause status - true for paused, false for not paused
*
*******************************************************************************/
bool is_music_player_paused_for_wwd(void)
{
    return music_player_wwd_pause;
}

#ifndef ENABLE_MP3_PLAYBACK
/* ****************************************************************************
 * Function Name: process_music_player_wav_file
 ******************************************************************************
 * Summary: Parses audio data array for wav header and audio data
 *
 * Parameters:
 *  wav_ptr: audio data array pointer
 *  wav_size: audio data size in bytes
 *
 * Return:
 *  bool: true if wav file is processed successfully, false otherwise
 *
 * ***************************************************************************/
static bool process_music_player_wav_file(uint8_t *wav_ptr, uint32_t wav_size)
{
    uint32_t        i_channel_mask;
    uint32_t        pcm_sz;
    int32_t         length;
    unsigned int    asc_stream_read_counter;

    if (wav_size < WAV_HEADER_SIZE_BYTES)
    {
        return false;
    }

    if( 0 != cy_wav_header_decode(&music_player_num_channels,
                                  &i_channel_mask,
                                  &music_player_sampling_rate,
                                  &pcm_sz,
                                  &length,
                                  wav_ptr,
                                  &asc_stream_read_counter,
                                  wav_size ) )
    {
        app_log_print("Error WAV header parsing failed\r\n");
        return false;
    }

    // app_log_print("WAV File Info: Channels: %d, Sampling Rate: %d, PCM Size: %d, Length: %d\n",
    //        music_player_num_channels, music_player_sampling_rate, pcm_sz, length);

    if ((music_player_num_channels <= 0) ||
        (music_player_num_channels > CHANNELS_STEREO) ||
        (pcm_sz != PCM_BIT_WIDTH_16) || (length <= (int32_t)0))
    {
        return false;
    }

    if ( (SAMPLING_RATE_48kHz != music_player_sampling_rate) &&
         (SAMPLING_RATE_44_1kHz != music_player_sampling_rate) &&
         (SAMPLING_RATE_16kHz != music_player_sampling_rate) )
    {
        return false;
    }

    if (MUSIC_PLAYER_SOURCE_FLASH == music_player_audio_source)
    {
        music_player_ptr = wav_ptr;
        music_player_index = asc_stream_read_counter;
        music_player_index_max = wav_size;
    }

    return true;
}

#endif /* ENABLE_MP3_PLAYBACK*/
/* ****************************************************************************
 * Function Name: process_music_player_relative_file_index
 ******************************************************************************
 * Summary:
 *  Process music file and file indexes. File index indicate next sample played over I2S.
 *
 * Parameters:
 *   relative_file_index: +1 for next song, -1 for prev song, 0 for start current song
 *   from start
 *
 * Return:
 *  None
 *
 * ***************************************************************************/
static bool process_music_player_relative_file_index(int32_t relative_file_index)
{
    const uint8_t **music_ptr;
    unsigned int *music_size;
    int32_t *music_file_index;
    int32_t *music_relative_file_index;
    int32_t music_file_index_max;
    int32_t file_index;

    int32_t parser_counter = 0;

#ifdef ENABLE_MP3_PLAYBACK
    uint32_t mp3_input_size;
    cy_rslt_t result;
#endif /* ENABLE_MP3_PLAYBACK */

    if (MUSIC_PLAYER_SOURCE_FLASH == music_player_audio_source)
    {
        music_ptr = flash_music_ptr;
        music_size = flash_music_sizes;
        music_file_index = &flash_music_file_index;
        music_relative_file_index = &flash_music_relative_file_index;
        music_file_index_max = flash_music_file_index_max;
    }
    else
    {
        return false;
    }

    file_index = (*music_relative_file_index + relative_file_index) % music_file_index_max;

    if (file_index < 0)
    {
        file_index += music_file_index_max;
    }

    do
    {
        /* app_log_print("Trying to play file index %d, ABS = %d...\n", file_index, playback_order[file_index]); */
#ifdef ENABLE_MP3_PLAYBACK
        mp3_input_size = music_size[playback_order[file_index]] > UINT16_MAX ? UINT16_MAX :
                         (uint16_t)music_size[playback_order[file_index]];

        /* De-initialize and re-initialize MP3 Decoder */
        result = cy_audio_sw_codec_deinit((cy_audio_sw_codec_t *)&mp3_decoder_handle);
        if (CY_RSLT_SUCCESS != result)
        {
            app_log_print(" >> Error: MP3 decoder deinit failed with error code: 0x%0X\n", result);
            CY_ASSERT(0);
        }

        result = cy_audio_sw_codec_init(CY_AUDIO_CODEC_OPERATION_DECODE,
                                        &mp3_decoder_config,
                                        (cy_audio_sw_codec_t *)&mp3_decoder_handle);
        if (CY_RSLT_SUCCESS != result)
        {
            app_log_print(" >> Error: MP3 decoder init failed with error code: 0x%0X\n", result);
            CY_ASSERT(0);
        }

        result = cy_audio_sw_codec_decode_stream_info_header(mp3_decoder_handle,
                                                (uint8_t*)music_ptr[playback_order[file_index]],
                                                &mp3_input_size,
                                                &mp3_stream_info);
        if (CY_RSLT_SUCCESS == result)
        {
            music_player_ptr = (uint8_t*) (music_ptr[playback_order[file_index]]);
            music_player_index = 0;
            music_player_index_max = music_size[playback_order[file_index]];

            /* Print mp3_stream_info struct */
            // app_log_print("\nMP3 File '%d' Info:", playback_order[file_index]);
            // app_log_print("  Sampling Rate: %d,", mp3_stream_info.sampling_rate);
            // app_log_print("  Channel: %d,", mp3_stream_info.channel);
            // app_log_print("  Bitwidth: %d\n", mp3_stream_info.bitwidth);

            if (((CY_AUDIO_CODEC_SAMPLING_RATE_48000HZ != mp3_stream_info.sampling_rate) &&
                 (CY_AUDIO_CODEC_SAMPLING_RATE_44100HZ != mp3_stream_info.sampling_rate) &&
                 (CY_AUDIO_CODEC_SAMPLING_RATE_16000HZ != mp3_stream_info.sampling_rate)) ||
                (CY_AUDIO_CODEC_BITWIDTH_16 != mp3_stream_info.bitwidth))
            {
                app_log_print(" >> Error: MP3 decoder stream info is not supported\n");
            }
            else
            {
                result = cy_audio_sw_codec_get_decoder_recommended_outbuf_size(
                                                    mp3_decoder_handle,
                                                    &mp3_stream_info,
                                                    &mp3_recommended_out_size);
                if (CY_RSLT_SUCCESS != result)
                {
                    app_log_print(" >> Error: MP3 decoder output size retrieval failed with error code: 0x%0X\n", result);
                    CY_ASSERT(0);
                }
                // app_log_print("MP3 decoder output size: %d\n", mp3_recommended_out_size);

                music_player_sampling_rate = (CY_AUDIO_CODEC_SAMPLING_RATE_48000HZ == mp3_stream_info.sampling_rate) ? SAMPLING_RATE_48kHz :
                                             (CY_AUDIO_CODEC_SAMPLING_RATE_44100HZ == mp3_stream_info.sampling_rate) ? SAMPLING_RATE_44_1kHz :
                                             (CY_AUDIO_CODEC_SAMPLING_RATE_16000HZ == mp3_stream_info.sampling_rate) ? SAMPLING_RATE_16kHz : 0;
                music_player_num_channels = (CY_AUDIO_CODEC_CHANNEL_MONO == mp3_stream_info.channel) ? 1 : 2;
                curr_mp3_file_total_samples = get_total_samples_mp3(music_ptr[playback_order[file_index]], music_size[playback_order[file_index]]) *
                                              music_player_num_channels;
                // app_log_print("MP3 File '%d' Total Samples: %ld\n", playback_order[file_index], curr_mp3_file_total_samples);
                break;
            }
        }
#else
        if (process_music_player_wav_file((uint8_t*)music_ptr[playback_order[file_index]], music_size[playback_order[file_index]]))
        {
            break;
        }
#endif /* ENABLE_MP3_PLAYBACK */

        app_log_print("Error parsing file at index %d...\n", (int)playback_order[file_index]);

        file_index ++;
        if (file_index >= music_file_index_max)
        {
            file_index = 0;
        }

        parser_counter ++;
    }
    while(parser_counter < music_file_index_max);

    if (parser_counter == music_file_index_max)
    {
        app_log_print(">> Music Player file index processing failed for all file indices in %d mode...\n", music_player_audio_source);
        return false;
    }

    *music_file_index = playback_order[file_index];
    *music_relative_file_index = file_index;

    /* Initialize ASRC for music playback and AEC reference */
    initialize_asrc();

    return true;
}

/* ****************************************************************************
 * Function Name: process_music_player_absolute_file_index
 ******************************************************************************
 * Summary:
 *  Process music file and file indexes. File index indicate next sample played over I2S.
 *
 * Parameters:
 *   absolute_file_index: +1 for next song, -1 for prev song, 0 for start current song 
 *   from start
 *
 * Return:
 *  None
 *
 * ***************************************************************************/
static bool process_music_player_absolute_file_index(int32_t absolute_file_index)
{
    const uint8_t **music_ptr;
    unsigned int *music_size;
    int32_t *music_file_index;
    int32_t music_file_index_max;
    int32_t file_index;

    int32_t parser_counter = 0;

#ifdef ENABLE_MP3_PLAYBACK
    uint32_t mp3_input_size;
    cy_rslt_t result;
#endif /* ENABLE_MP3_PLAYBACK */

    if (MUSIC_PLAYER_SOURCE_FLASH == music_player_audio_source)
    {
        music_ptr = flash_music_ptr;
        music_size = flash_music_sizes;
        music_file_index = &flash_music_file_index;
        music_file_index_max = flash_music_file_index_max;
    }
    else
    {
        return false;
    }

    file_index = (absolute_file_index) % music_file_index_max;

    if (file_index < 0)
    {
        file_index += music_file_index_max;
    }

    do
    {
#ifdef ENABLE_MP3_PLAYBACK
        mp3_input_size = music_size[file_index] > UINT16_MAX ? UINT16_MAX :
                         (uint16_t)music_size[file_index];

        /* De-initialize and re-initialize MP3 Decoder */
        result = cy_audio_sw_codec_deinit((cy_audio_sw_codec_t *)&mp3_decoder_handle);
        if (CY_RSLT_SUCCESS != result)
        {
            app_log_print(" >> Error: MP3 decoder deinit failed with error code: 0x%0X\n", result);
            CY_ASSERT(0);
        }

        result = cy_audio_sw_codec_init(CY_AUDIO_CODEC_OPERATION_DECODE,
                                        &mp3_decoder_config,
                                        (cy_audio_sw_codec_t *)&mp3_decoder_handle);
        if (CY_RSLT_SUCCESS != result)
        {
            app_log_print(" >> Error: MP3 decoder init failed with error code: 0x%0X\n", result);
            CY_ASSERT(0);
        }

        result = cy_audio_sw_codec_decode_stream_info_header(mp3_decoder_handle,
                                                (uint8_t*)music_ptr[file_index],
                                                &mp3_input_size,
                                                &mp3_stream_info);
        if (CY_RSLT_SUCCESS == result)
        {
            music_player_ptr = (uint8_t*) (music_ptr[file_index]);
            music_player_index = 0;
            music_player_index_max = music_size[file_index];

            /* Print mp3_stream_info struct */
            // app_log_print("\nMP3 File '%d' Info:", file_index);
            // app_log_print("  Sampling Rate: %d,", mp3_stream_info.sampling_rate);
            // app_log_print("  Channel: %d,", mp3_stream_info.channel);
            // app_log_print("  Bitwidth: %d\n", mp3_stream_info.bitwidth);

            if (((CY_AUDIO_CODEC_SAMPLING_RATE_48000HZ != mp3_stream_info.sampling_rate) &&
                 (CY_AUDIO_CODEC_SAMPLING_RATE_44100HZ != mp3_stream_info.sampling_rate) &&
                 (CY_AUDIO_CODEC_SAMPLING_RATE_16000HZ != mp3_stream_info.sampling_rate)) ||
                (CY_AUDIO_CODEC_BITWIDTH_16 != mp3_stream_info.bitwidth))
            {
                app_log_print(" >> Error: MP3 decoder stream info is not supported\n");
            }
            else
            {
                result = cy_audio_sw_codec_get_decoder_recommended_outbuf_size(
                                                    mp3_decoder_handle,
                                                    &mp3_stream_info,
                                                    &mp3_recommended_out_size);
                if (CY_RSLT_SUCCESS != result)
                {
                    app_log_print(" >> Error: MP3 decoder output size retrieval failed with error code: 0x%0X\n", result);
                    CY_ASSERT(0);
                }
                // app_log_print("MP3 decoder output size: %d\n", mp3_recommended_out_size);

                music_player_sampling_rate = (CY_AUDIO_CODEC_SAMPLING_RATE_48000HZ == mp3_stream_info.sampling_rate) ? SAMPLING_RATE_48kHz :
                                             (CY_AUDIO_CODEC_SAMPLING_RATE_44100HZ == mp3_stream_info.sampling_rate) ? SAMPLING_RATE_44_1kHz :
                                             (CY_AUDIO_CODEC_SAMPLING_RATE_16000HZ == mp3_stream_info.sampling_rate) ? SAMPLING_RATE_16kHz : 0;
                music_player_num_channels = (CY_AUDIO_CODEC_CHANNEL_MONO == mp3_stream_info.channel) ? 1 : 2;
                curr_mp3_file_total_samples = get_total_samples_mp3(music_ptr[file_index], music_size[file_index]) *
                                              music_player_num_channels;
                // app_log_print("MP3 File '%d' Total Samples: %ld\n", file_index, curr_mp3_file_total_samples);
                break;
            }
        }
#else
        if ((music_player_ptr != NULL) || (music_player_index != 0))
        {
            break;
        }
        if (process_music_player_wav_file((uint8_t*)music_ptr[file_index], music_size[file_index]))
        {
            break;
        }
#endif /* ENABLE_MP3_PLAYBACK */

        app_log_print("Error parsing file at index %d...\n", (int)file_index);

        file_index ++;
        if (file_index >= music_file_index_max)
        {
            file_index = 0;
        }

        parser_counter ++;
    }
    while(parser_counter < music_file_index_max);

    if (parser_counter == music_file_index_max)
    {
        app_log_print(">> Music Player file index processing failed for all file indices in %d mode...\n", music_player_audio_source);
        return false;
    }

    *music_file_index = file_index;

    /* Initialize ASRC for music playback and AEC reference */
    initialize_asrc();

    return true;
}

/******************************************************************************
* Function Name: get_pcm_buffer_ptr
*******************************************************************************
* Summary:
*   Function that returns the pointer to memory for PCM data. This is similar
*   to malloc but from a static memory buffer.
*
* Parameters:
*   data_size_bytes : Size in bytes for the required buffer
*
* Return:
*   uint8_t* - Pointer output from the static buffer
*
*******************************************************************************/
uint8_t* get_pcm_buffer_ptr(uint32_t data_size_bytes)
{
    if (PCM_DATA_BUFFER_SIZE <= (pcm_data_buffer_index + data_size_bytes))
    {
        pcm_data_buffer_index = 0;
    }

    uint8_t *ptr = &(pcm_data_buffer[pcm_data_buffer_index]);
    pcm_data_buffer_index += data_size_bytes;

    return ptr;
}

/******************************************************************************
* Function Name: get_aec_ref_buffer_ptr
*******************************************************************************
* Summary:
*   Function that returns the pointer to memory for AEC Reference Storage until
*   it is consumed by the AFE Task.
*
* Parameters:
*   data_size_bytes : Size in bytes for the required buffer
*
* Return:
*   uint8_t* - Pointer output from the static buffer
*
*******************************************************************************/
uint8_t* get_aec_ref_buffer_ptr(uint32_t data_size_bytes)
{
    if (AEC_REF_BUFFER_SIZE <= (aec_ref_buffer_index + data_size_bytes))
    {
        aec_ref_buffer_index = 0;
    }

    uint8_t *ptr = &(aec_ref_buffer[aec_ref_buffer_index]);
    aec_ref_buffer_index += data_size_bytes;

    return ptr;
}

#ifdef ENABLE_MP3_PLAYBACK
/*******************************************************************************
* Function Name: get_mp3_decoded_data
********************************************************************************
* Summary:
*  Returns the MP3 decoded data from the buffer. If the buffer is empty, it will
*  decode more MP3 data into the buffer.
*
* Parameters:
*  None
*
* Return:
*  uint8_t* : Pointer to the decoded MP3 data
*
*******************************************************************************/
static uint8_t* get_mp3_decoded_data(void)
{
    cy_rslt_t result;
    uint8_t* mp3_decoded_buffer_ptr = NULL;
    uint32_t remaining_input_data = 0;
    uint32_t mp3_decoder_input_size = 0;
    uint32_t mp3_decoder_output_size = 0;
    mp3_frame_info_t mp3_frame_info;
    int32_t frame_start_offset = 0;
    int32_t temp = 0;
    int32_t frame_size = 0;
    // int32_t playback_frame_bytes = (music_player_sampling_rate * sizeof(int16_t) * 10) / 1000;
    int32_t playback_frame_bytes = I2S_FRAME_SIZE_SAMPLES(music_player_sampling_rate) * sizeof(int16_t);

    if ((mp3_decoded_buffer_index > prev_i2s_buffer_index) &&
        ((mp3_decoded_buffer_index - prev_i2s_buffer_index) > (music_player_num_channels * playback_frame_bytes)))
    {
        mp3_decoded_buffer_ptr = (uint8_t*) &(mp3_decoded_buffer[prev_i2s_buffer_index]);
        prev_i2s_buffer_index += (music_player_num_channels * playback_frame_bytes);
        mp3_decoded_data_in_buffer -= (music_player_num_channels * playback_frame_bytes);
    }
    else if (mp3_decoded_buffer_index < prev_i2s_buffer_index)
    {
        if ((mp3_decoded_buffer_size > prev_i2s_buffer_index) &&
            ((mp3_decoded_buffer_size - prev_i2s_buffer_index) > (music_player_num_channels * playback_frame_bytes)))
        {
            mp3_decoded_buffer_ptr = (uint8_t*) &(mp3_decoded_buffer[prev_i2s_buffer_index]);
            prev_i2s_buffer_index += (music_player_num_channels * playback_frame_bytes);
            mp3_decoded_data_in_buffer -= (music_player_num_channels * playback_frame_bytes);
        }
        else if ((mp3_decoded_buffer_size > prev_i2s_buffer_index) &&
                 (mp3_decoded_buffer_size - prev_i2s_buffer_index + mp3_decoded_buffer_index) > (music_player_num_channels * playback_frame_bytes))
        {
            memcpy(&(mp3_decoded_buffer[mp3_decoded_buffer_size]), mp3_decoded_buffer,
                   ((music_player_num_channels * playback_frame_bytes) - (mp3_decoded_buffer_size - prev_i2s_buffer_index)));
            mp3_decoded_buffer_ptr = (uint8_t*) &(mp3_decoded_buffer[prev_i2s_buffer_index]);
            prev_i2s_buffer_index = ((music_player_num_channels * playback_frame_bytes) - (mp3_decoded_buffer_size - prev_i2s_buffer_index));
            mp3_decoded_buffer_size = 0;
            mp3_decoded_data_in_buffer -= (music_player_num_channels * playback_frame_bytes);
        }
    }
    else
    {
        mp3_decoded_buffer_ptr = (uint8_t*) &(mp3_decoded_buffer[prev_i2s_buffer_index]);
        prev_i2s_buffer_index += (music_player_num_channels * playback_frame_bytes);
        mp3_decoded_data_in_buffer -= (music_player_num_channels * playback_frame_bytes);
    }

    if (mp3_decoded_data_in_buffer < (int32_t)(2 * music_player_num_channels * playback_frame_bytes))
    {
        frame_size = 0;
        frame_start_offset = 0;
        for (uint32_t i = 0, index = music_player_index; i < MP3_DECODER_FRAME_COUNT; i++)
        {
            parse_mp3_frame(&(music_player_ptr[index]),
                            music_player_index_max - index,
                            &mp3_frame_info, &temp);
            if (temp < 0)
            {
                app_log_print(" >> Error: MP3 frame offset is invalid\n");
                CY_ASSERT(0);
            }
            frame_size += mp3_frame_info.frame_size;
            index += mp3_frame_info.frame_size;
            index += temp;
            frame_start_offset += temp;
            if (index >= music_player_index_max)
            {
                break;
            }
        }

        /* Load more raw MP3 data */
        remaining_input_data = music_player_index_max - music_player_index;
        mp3_decoder_input_size = frame_start_offset + frame_size;
        if (mp3_decoder_input_size > remaining_input_data)
        {
            mp3_decoder_input_size = remaining_input_data;
        }
        mp3_decoder_output_size = MP3_DECODER_FRAME_COUNT * mp3_recommended_out_size *
                                  music_player_num_channels;

        /* Decode MP3 data */
        result = cy_audio_sw_codec_decode(mp3_decoder_handle,
                                          &(music_player_ptr[music_player_index]),
                                          &mp3_decoder_input_size,
                                          &(mp3_decoded_buffer[mp3_decoded_buffer_index]),
                                          &mp3_decoder_output_size);
        if (result != CY_RSLT_SUCCESS)
        {
            app_log_print("Error: MP3 decoding failed with error code: 0x%0X\n", result);
            CY_ASSERT(0);
        }

        music_player_index += mp3_decoder_input_size;
        mp3_decoded_buffer_index += mp3_decoder_output_size;
        mp3_decoded_data_in_buffer += mp3_decoder_output_size;

        if (music_player_index >= music_player_index_max)
        {
            if (!fading_active)
            {
                music_player_restore_volume = music_player_volume;
            }
            start_fade_out(true);

            /* Switch to the next audio file or loop the current one. */
            process_music_player_relative_file_index(music_player_loop ? 0: 1);
#ifdef ENABLE_GFX_UI
            send_mp_gfx_queue_command(GFX_SWITCH_TO_TRACK_ID, flash_music_relative_file_index);
#endif /* ENABLE_GFX_UI */            
        }

        if ((mp3_decoded_buffer_index + mp3_decoder_output_size) >= MP3_DECODED_BUFFER_SIZE_LIMIT)
        {
            /* Store the valid size and reset the buffer index */
            mp3_decoded_buffer_size = mp3_decoded_buffer_index;
            mp3_decoded_buffer_index = 0;
        }
    }

    return mp3_decoded_buffer_ptr;
}
#endif /* ENABLE_MP3_PLAYBACK */

/*******************************************************************************
* Function Name: create_fade_timers
********************************************************************************
* Summary:
*  Initializes the fade-out and fade-in timers used for volume control during
*  transitions. The timers are set to trigger at a defined period and call the
*  respective callback functions to adjust the volume gradually.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void create_fade_timers(void)
{
    fade_out_timer = xTimerCreate("FadeOutTimer",
                                  pdMS_TO_TICKS(FADE_OUT_TIMER_PERIOD_MS),
                                  pdTRUE, NULL, fade_out_timer_callback);
    fade_in_timer  = xTimerCreate("FadeInTimer",
                                  pdMS_TO_TICKS(FADE_IN_TIMER_PERIOD_MS),
                                  pdTRUE, NULL, fade_in_timer_callback);
}

/*******************************************************************************
* Function Name: fade_out_timer_callback
********************************************************************************
* Summary:
*  Callback function for the fade-out timer. It gradually decreases the volume
*  of the music player in steps until it reaches zero or the fade-out is complete.
*
* Parameters:
*  xTimer: Timer handle
*
* Return:
*  None
*
*******************************************************************************/
static void fade_out_timer_callback(TimerHandle_t xTimer)
{
    if (fade_step_count < FADE_OUT_STEPS && fade_out_start_volume > 0)
    {
        uint8_t new_volume = (uint8_t)((fade_out_start_volume *
                             (FADE_OUT_STEPS - fade_step_count - 1)) /
                             (FADE_OUT_STEPS - 1));
        music_player_volume = new_volume;
        i2s_playback_volume_control(music_player_volume);
        fade_step_count++;
    }
    else
    {
        music_player_volume = 0;
        i2s_playback_volume_control(music_player_volume);
        xTimerStop(fade_out_timer, 0);
        fading_active = false;

        /* Start fade in if required */
        if (fade_out_triggers_fade_in)
        {
            fade_out_triggers_fade_in = false;
            vTaskDelay(pdMS_TO_TICKS(FADE_OUT_TO_FADE_IN_DELAY_MS));
            start_fade_in();
        }
    }
}

/*******************************************************************************
* Function Name: fade_in_timer_callback
********************************************************************************
* Summary:
*  Callback function for the fade-in timer. It gradually increases the volume
*  of the music player in steps until it reaches the target volume or the fade-in
*  is complete.
*
* Parameters:
*   xTimer: Timer handle
*
* Return:
*   None
*
*******************************************************************************/
static void fade_in_timer_callback(TimerHandle_t xTimer)
{
    if (fade_step_count < FADE_IN_STEPS && fade_in_target_volume > 0)
    {
        /* Calculate new volume (linear fade) */
        uint8_t new_volume = (uint8_t)((fade_in_target_volume * (fade_step_count + 1)) / (FADE_IN_STEPS));
        music_player_volume = new_volume;
        i2s_playback_volume_control(music_player_volume);
        fade_step_count++;
    }
    else
    {
        music_player_volume = fade_in_target_volume;
        i2s_playback_volume_control(music_player_volume);
        xTimerStop(fade_in_timer, 0);
        fading_active = false;
    }
}

/*******************************************************************************
* Function Name: start_fade_out
********************************************************************************
* Summary:
*  Initiates a fade-out effect for the music player. It gradually decreases the
*  volume to zero over a series of steps. If specified, it can also trigger a fade-in
*  effect after the fade-out completes.
*
* Parameters:
*  start_fade_in_after: If true, triggers a fade-in effect after the fade-out completes.
*
* Return:
*  None
*
*******************************************************************************/
void start_fade_out(bool start_fade_in_after)
{
    fade_out_start_volume = music_player_volume;
    fade_step_count = 0;
    fade_out_triggers_fade_in = start_fade_in_after;
    if (fade_in_timer != NULL)
    {
        xTimerStop(fade_in_timer, 0);
    }
    if (fade_out_timer != NULL)
    {
        xTimerStop(fade_out_timer, 0);
        xTimerStart(fade_out_timer, 0);
        fading_active = true;
    }
}

/*******************************************************************************
* Function Name: start_fade_in
********************************************************************************
* Summary:
*  Initiates a fade-in effect for the music player. It gradually increases the
*  volume from zero to the target volume over a series of steps. This is typically
*  used after a fade-out effect or when starting playback of a new track.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void start_fade_in(void)
{
    fade_in_target_volume = music_player_restore_volume;
    music_player_volume = fade_in_target_volume;

    i2s_playback_volume_control(0);
    fade_step_count = 0;
    if (fade_in_timer != NULL)
    {
        xTimerStop(fade_in_timer, 0);
        xTimerStart(fade_in_timer, 0);
        fading_active = true;
    }
}

/*******************************************************************************
* Function Name: set_music_player_shuffle
********************************************************************************
* Summary:
*  Sets the shuffle mode for the music player. When enabled, the player will
*  play tracks in a random order instead of sequentially. This function allows
*  toggling the shuffle mode on or off.
*
* Parameters:
*  enable_shuffle: Boolean value to enable (true) or disable (false) shuffle mode.
*
* Return:
*  None
*
*******************************************************************************/
void set_music_player_shuffle(bool enable_shuffle)
{
    int32_t i, j, temp, current_index = playback_order[flash_music_relative_file_index];
    music_player_shuffle = enable_shuffle;

    app_log_print(" [%s]: Shuffle mode %s\n", __FUNCTION__, enable_shuffle ? "enabled" : "disabled");

    for (i = 0; i < flash_music_file_index_max; i++)
    {
        playback_order[i] = i;
    }

    /* Generate random order in the playback_order array from 0 to
     * flash_music_file_index_max.
     */
    if (enable_shuffle)
    {
        for (i = 0; i < flash_music_file_index_max; i++)
        {
            j = rand() % flash_music_file_index_max;
            temp = playback_order[i];
            playback_order[i] = playback_order[j];
            playback_order[j] = temp;
        }
    }

    app_log_print(" [%s]: New playback order: ", __FUNCTION__);
    for (i = 0; i < flash_music_file_index_max; i++)
    {
        app_log_print("%d ", playback_order[i]);
    }
    app_log_print("\n");

    /* Find the new relative index of the current music file */
    flash_music_relative_file_index = get_relative_index(current_index);
}

/*******************************************************************************
* Function Name: get_relative_index
********************************************************************************
* Summary:
*  Returns the relative index of the current music file based on the absolute
*  file index.
*
* Parameters:
*  absolute_file_index: The absolute index of the music file in the playback order.
*
* Return:
*  Relative index of the music file in the playback order.
*
*******************************************************************************/
int32_t get_relative_index(int32_t absolute_file_index)
{
    int32_t relative_index = 0;

    for (relative_index = 0; relative_index < flash_music_file_index_max; relative_index++)
    {
        if (playback_order[relative_index] == absolute_file_index)
        {
            break;
        }
    }
    if (relative_index >= flash_music_file_index_max)
    {
        app_log_print("Error: Absolute file index %d not found in playback order.\n", absolute_file_index);
        relative_index = 0;
    }
    return relative_index;
}

/*******************************************************************************
* Function Name: read_and_send_music_player_data
********************************************************************************
* Summary:
*  Function that reads the music player data from Flash and sends
*  them to I2S Task
*
* Parameters:
*  tx_frame_count: Number of frames to be sent to I2S Task
*
* Return:
*  None
*
*******************************************************************************/
static void read_and_send_music_player_data(uint32_t tx_frame_count)
{
    uint8_t* tx_data_ptr = NULL;
    uint8_t* temp_data_ptr = NULL;
    uint32_t tx_data_len = 0;
    uint32_t aec_ref_data_len = 0;
#ifndef ENABLE_MP3_PLAYBACK    
    uint32_t packet_read_len = 0;
#endif /* ENABLE_MP3_PLAYBACK */    
    uint32_t read_len = 0;
    
    /*bool allocate_new_buffer = false;*/
    uint32_t expected_tx_data_len = 0;
    uint32_t expected_aec_ref_data_len = 0;
    uint32_t input_len = 0;
    bool switch_to_next_file = false;
#ifdef ENABLE_MP3_PLAYBACK
    music_player_q_data_t music_player_q_data;
#endif /* ENABLE_MP3_PLAYBACK */

    for (uint32_t i = 0; i < tx_frame_count; i++)
    {
        if(!music_player_active || (MUSIC_PLAYER_SOURCE_NOT_CONFIGURED == music_player_audio_source))
        {
            return;
        }

        if (MUSIC_PLAYER_OUTPUT_I2S == music_player_audio_output)
        {
            tx_data_len = (music_player_num_channels * sizeof(int16_t) *
                        I2S_FRAME_SIZE_SAMPLES(music_player_sampling_rate));
        }

        expected_tx_data_len = ASRC_EXPECTED_OUTPUT_SIZE(tx_data_len, music_player_sampling_rate,
                                                         output_sampling_freq);
        i2s_playback_q_data.num_channels = (uint8_t) music_player_num_channels;

        if (MUSIC_PLAYER_SOURCE_FLASH == music_player_audio_source)
        {
#ifdef ENABLE_MP3_PLAYBACK
            tx_data_ptr = (uint8_t *) get_mp3_decoded_data();
            read_len = tx_data_len;
            /*allocate_new_buffer = true;*/

            if (skip_frames > 0)
            {
                i --;
                skip_frames--;
                continue;
            }

            if (NULL == tx_data_ptr)
            {
                music_player_q_data.cmd = MUSIC_PLAYER_PRELOAD_DATA;
                if (pdTRUE != xQueueSendToFront(music_player_task_q, &music_player_q_data, portMAX_DELAY))
                {
                    app_log_print(">>> Error: Music Player Task Queue Send failed*******************\r\n");
                }
                break;
            }
#else
            tx_data_ptr = (uint8_t *) &(music_player_ptr[music_player_index]);
            read_len = tx_data_len;
            /*allocate_new_buffer = true;*/

            if ( (music_player_index + read_len) >= music_player_index_max )
            {
                tx_data_ptr = get_pcm_buffer_ptr(read_len);
                packet_read_len = (music_player_index_max - music_player_index);
                memcpy(tx_data_ptr, (uint8_t *) &(music_player_ptr[music_player_index]), packet_read_len);
                memset(tx_data_ptr + packet_read_len, 0, read_len - packet_read_len);

                /* Switch to next audio file as this file is done */
                switch_to_next_file = true;
            }
            else
            {
                music_player_index += read_len;
            }
            packet_read_len = read_len;
#endif /* ENABLE_MP3_PLAYBACK */

            if (expected_tx_data_len != tx_data_len)
            {
                temp_data_ptr = get_pcm_buffer_ptr(expected_tx_data_len);
                /*allocate_new_buffer = false;*/
                memcpy(temp_data_ptr, tx_data_ptr, read_len);
                if (expected_tx_data_len > read_len)
                {
                    memset(temp_data_ptr + read_len, 0, expected_tx_data_len - read_len);
                }
                tx_data_ptr = temp_data_ptr;
            }
        }

        /* ASRC for AEC Reference Data */
        if (MUSIC_PLAYER_OUTPUT_I2S == music_player_audio_output)
        {
            if (AEC_REF_SAMPLE_RATE_HZ != music_player_sampling_rate)
            {
                expected_aec_ref_data_len = ASRC_EXPECTED_OUTPUT_SIZE(tx_data_len / music_player_num_channels,
                                                        music_player_sampling_rate, AEC_REF_SAMPLE_RATE_HZ);
                temp_data_ptr = get_aec_ref_buffer_ptr(expected_aec_ref_data_len);
                aec_ref_data_len = perform_asrc((int16_t *)tx_data_ptr, (tx_data_len / sizeof(int16_t)),
                                                (CHANNELS_STEREO == music_player_num_channels),
                                                (int16_t *)temp_data_ptr, false, &aec_ref_asrc_obj);
                i2s_playback_q_data.aec_ref_ptr = (int16_t*) temp_data_ptr;

                aec_ref_data_len *= sizeof(int16_t);
            }
            else
            {
                if (CHANNELS_STEREO == music_player_num_channels)
                {
                    aec_ref_data_len = tx_data_len / CHANNELS_STEREO;
                    temp_data_ptr = get_pcm_buffer_ptr(aec_ref_data_len);
                    convert_stereo_to_mono((int16_t *) tx_data_ptr,
                                           (tx_data_len / sizeof(int16_t)),
                                           (int16_t *) temp_data_ptr);
                    i2s_playback_q_data.aec_ref_ptr = (int16_t*) temp_data_ptr;
                }
                else
                {
                    aec_ref_data_len = tx_data_len;
                    temp_data_ptr = get_aec_ref_buffer_ptr(tx_data_len);
                    memcpy(temp_data_ptr, tx_data_ptr, tx_data_len);
                    i2s_playback_q_data.aec_ref_ptr = (int16_t*) temp_data_ptr;
                }
            }
        }

        /* ASRC for Music Playback Data */
        if (expected_tx_data_len != tx_data_len)
        {
            input_len = tx_data_len;
            memcpy(asrc_in_temp_buf_16bit, tx_data_ptr, input_len);
            tx_data_len = perform_asrc(asrc_in_temp_buf_16bit, (input_len / sizeof(int16_t)),
                                       (CHANNELS_STEREO == music_player_num_channels),
                                       (int16_t *)tx_data_ptr, true, &playback_asrc_obj);
            tx_data_len *= sizeof(int16_t);
        }

        if (tx_data_len != expected_tx_data_len)
        {
            app_log_print(" >> Error: Expected tx_data_len mismatch: tx_data_len = %d, expected_tx_data_len = %d\n",
                   tx_data_len, expected_tx_data_len);
        }

        tx_data_len = expected_tx_data_len;

        if (MUSIC_PLAYER_OUTPUT_I2S == music_player_audio_output)
        {
            i2s_playback_q_data.data = (int16_t*) tx_data_ptr;
            i2s_playback_q_data.data_len = tx_data_len;

            if (aec_ref_data_len != (I2S_FRAME_SIZE_SAMPLES(AEC_REF_SAMPLE_RATE_HZ) * sizeof(int16_t)))
            {
                app_log_print(">> Error: AEC reference data length mismatch '%d' != '%lu'\n",
                    aec_ref_data_len, (I2S_FRAME_SIZE_SAMPLES(AEC_REF_SAMPLE_RATE_HZ) * sizeof(int16_t)));
            }

            if ( (tx_data_len % (I2S_QUEUE_TX_SAMPLES_PER_CH * i2s_playback_q_data.num_channels)) != 0)
            {
                app_log_print(">> Error: tx_data_len is not a multiple of I2S_QUEUE_TX_SAMPLES_PER_CH (%d) for %d channels\n",
                       I2S_QUEUE_TX_SAMPLES_PER_CH, i2s_playback_q_data.num_channels);
                continue;
            }

            for (uint32_t j = 0; j < tx_data_len;)
            {
                i2s_playback_q_data.data_len = (I2S_QUEUE_TX_SAMPLES_PER_CH *
                                                i2s_playback_q_data.num_channels *
                                                sizeof(int16_t));

                if(NULL == i2s_playback_q_data.data)
                {
                    break;
                }

                if (pdTRUE != xQueueSend(i2s_playback_task_q, &i2s_playback_q_data, portMAX_DELAY))
                {
                    app_log_print(">>> Error: I2S playback Queue Send failed !\r\n");
                }

                i2s_playback_q_data.aec_ref_ptr = NULL;
                i2s_playback_q_data.data += (I2S_QUEUE_TX_SAMPLES_PER_CH *
                                             i2s_playback_q_data.num_channels);
                j += i2s_playback_q_data.data_len;
            }
        }

        /* Switch to the next audio file or loop the current one. */
        if (switch_to_next_file)
        {
            if (!fading_active)
            {
                music_player_restore_volume = music_player_volume;
            }
            start_fade_out(true);
            process_music_player_relative_file_index(music_player_loop ? 0 : 1);
#ifdef ENABLE_GFX_UI 
            send_mp_gfx_queue_command(GFX_SWITCH_TO_TRACK_ID, flash_music_relative_file_index);
#endif /* ENABLE_GFX_UI  */            
        }
    }
}

/*******************************************************************************
* Function Name: notify_music_player_outputs
********************************************************************************
* Summary:
*  Function that notifies the I2S Task about change in music
*  playback if it is not already active.
*
* Parameters:
*  enable_fade_in: If true, initiates a fade-in effect when playback starts.
*
* Return:
*  None
*
*******************************************************************************/
static void notify_music_player_outputs(bool enable_fade_in)
{
    if (!is_music_player_active())
    {
        music_player_active = !music_player_active;
        if(music_player_pause)
        {
            music_player_pause = false;
            music_player_wwd_pause = false;
            if (enable_fade_in)
            {
                if (music_player_volume != 0)
                {
                    start_fade_out(true);
                }
                else
                {
                    start_fade_in();
                }
            }
        }
    }
}

/*******************************************************************************
* Function Name: process_music_player_track_change
********************************************************************************
* Summary:
*  Function that sets the configuration variables for the track change and
*  sends the prefill command to the Music Player Task for playback.
*
* Parameters:
*  relative_file_index: +1 for next song, -1 for previous song,
*                       0 for start current song
*
* Return:
*  None
*
*******************************************************************************/
static void process_music_player_track_change(int32_t relative_file_index)
{
    music_player_q_data_t music_player_q_data;

    xQueueReset(music_player_task_q);

    // notify_music_player_outputs(false);
    start_fade_out(true);

#ifdef ENABLE_MP3_PLAYBACK
    mp3_decoded_buffer_index = 0;
    prev_i2s_buffer_index = 0;
    mp3_decoded_data_in_buffer = 0;
#endif /* ENABLE_MP3_PLAYBACK */

    if (MUSIC_PLAYER_SOURCE_NOT_CONFIGURED == music_player_audio_source)
    {
        flash_music_relative_file_index += relative_file_index;
        if (flash_music_relative_file_index < 0)
        {
            flash_music_relative_file_index += flash_music_file_index_max;
        }
        flash_music_relative_file_index %= flash_music_file_index_max;
        flash_music_file_index = playback_order[flash_music_relative_file_index];
        app_log_print(" Playing file '%d'...\n", flash_music_file_index);
        enable_music_player();
    }
    else
    {
        if (!is_music_player_active() && music_player_pause)
        {
            start_i2s();
            music_player_active = true;
            music_player_pause = false;
            music_player_wwd_pause = false;
        }

        process_music_player_relative_file_index(relative_file_index);
        app_log_print(" Playing file '%d'...\n", flash_music_file_index);
    }

    music_player_q_data.cmd = MUSIC_PLAYER_PRELOAD_DATA;
    music_player_q_data.data_len = 0;
    music_player_q_data.data = NULL;
    if (pdTRUE != xQueueSend(music_player_task_q, &music_player_q_data, portMAX_DELAY))
    {
        app_log_print(">>> Error: Next track - Music Player Task Queue Send failed !\r\n");
        return;
    }
}

/******************************************************************************
* Function Name: initialize_asrc
*******************************************************************************
* Summary:
*   Function that initializes Audio Sample Rate Conversion (ASRC) for the music
*   playback and AEC reference.
*
* Parameters:
*   None
*
* Return:
*   None
*
*******************************************************************************/
static void initialize_asrc(void)
{
    /* Initialize ASRC for music playback */
    if (music_player_sampling_rate != output_sampling_freq)
    {
        /*app_log_print("Initializing ASRC for music playback data from %d Hz to %d Hz...\n", music_player_sampling_rate, output_sampling_freq);*/
        init_IFX_asrc(&playback_asrc_obj, music_player_sampling_rate, output_sampling_freq);
        IFX_SetClockDrift(&playback_asrc_obj, 0);
    }

    /* Initialize ASRC for AEC Reference from Music sampling rate to 16 kHz */
    if (music_player_sampling_rate != AEC_REF_SAMPLING_RATE)
    {
        /*app_log_print("Initializing ASRC for AEC Reference from %d Hz to %d Hz...\n", music_player_sampling_rate, AEC_REF_SAMPLING_RATE);*/
        init_IFX_asrc(&aec_ref_asrc_obj, music_player_sampling_rate, AEC_REF_SAMPLING_RATE);
        IFX_SetClockDrift(&aec_ref_asrc_obj, 0);
    }
}

/******************************************************************************
* Function Name: perform_asrc
*******************************************************************************
* Summary:
*   Function that performs Audio Sample Rate Conversion (ASRC) for the a mono
*   channel audio input.
*
* Parameters:
*   input_ptr            : Pointer to the input audio data in int16_t format
*   input_samples        : Total number of audio samples in the input buffer
*                          (including stereo channel if applicable)
*   stereo_audio         : Flag to indicate if the audio input is stereo data
*   output_ptr           : Pointer to the output audio data post ASRC
*   enable_stereo_output : Flag to indicate if the output audio data should be
*                          stereo data. Applicable only if stereo_audio is true
*   asrc_obj             : Pointer to the ASRC object
*
* Return:
*   uint32_t             : Number of output samples post ASRC
*
*******************************************************************************/
static uint32_t perform_asrc(int16_t* input_ptr, uint32_t input_samples, bool stereo_audio,
                             int16_t* output_ptr, bool enable_stereo_output,
                             IFX_ASRC_STRUCT_t* asrc_obj)
{
    int32_t* asrc_in_ptr = asrc_in_buf_32bit;
    int32_t* asrc_out_ptr = asrc_out_buf_32bit;
    int16_t* frame_in_buffer = input_ptr;
    int16_t* frame_out_buffer = output_ptr;
    uint32_t asrc_out_len = 0;
    uint32_t input_len = input_samples;
    uint32_t total_output_samples = 0;

    /* Half the ASRC input samples in case of stereo */
    if (stereo_audio)
    {
        input_len >>= 1;
    }

    for (uint32_t i = 0; i < input_len; i++)
    {
        if (stereo_audio)
        {
            asrc_in_buf_32bit[i] = (int32_t) (*(int16_t *)frame_in_buffer);
            frame_in_buffer ++;

            asrc_in_buf_32bit[i] += (int32_t) (*(int16_t *)frame_in_buffer);
            frame_in_buffer ++;

            /* Average both left and right channel data as ASRC input
             * for stereo case */
            asrc_in_buf_32bit[i] >>= 1;
        }
        else
        {
            /* Use the mono channel data directly if not stereo */
            asrc_in_buf_32bit[i] = (int32_t)
                                            (*(int16_t *)frame_in_buffer);
            frame_in_buffer ++;
        }
    }

    /* IFX_asrc() called multiple times to process data in chunks. */
    while(input_len > 0)
    {
        asrc_out_ptr = asrc_out_buf_32bit;
        asrc_out_len = ASRC_OUTPUT_BUFFER_SIZE;

        if(input_len > ASRC_INPUT_SAMPLES)
        {
            IFX_asrc(asrc_in_ptr, ASRC_INPUT_SAMPLES,
                     asrc_out_ptr, (uint16_t *)&asrc_out_len, asrc_obj);
            input_len -= ASRC_INPUT_SAMPLES;
            asrc_in_ptr += ASRC_INPUT_SAMPLES;
        }
        else
        {
            IFX_asrc(asrc_in_ptr, input_len,
                     asrc_out_ptr, (uint16_t *)&asrc_out_len, asrc_obj);
            input_len = 0;
        }

        total_output_samples += asrc_out_len;

        /* Convert int32_t to int16_t */
        for(uint32_t j = 0; j < asrc_out_len; j++)
        {
            *(frame_out_buffer++) = (int16_t)(*(asrc_out_ptr + j));
        }
    }

    /* If the input was stereo, the ASRC output should also be stereo data.
     * Therefore, duplicate samples to convert mono to stereo in situ
     */
    if (stereo_audio && enable_stereo_output)
    {
        frame_in_buffer = (int16_t *) &output_ptr[total_output_samples];
        frame_out_buffer = (int16_t *) &output_ptr[total_output_samples * 2];

        do
        {
            frame_in_buffer --;
            frame_out_buffer --;
            *frame_out_buffer = *frame_in_buffer;
            frame_out_buffer --;
            *frame_out_buffer = *frame_in_buffer;
            total_output_samples ++;
        }
        while(frame_in_buffer > (int16_t *) output_ptr);
    }

    return (total_output_samples);
}

#ifndef ENABLE_MP3_PLAYBACK
/******************************************************************************
* Function Name: get_wav_seek_offset
*******************************************************************************
* Summary:
*   Function that calculates the seek offset for a WAV file based on the
*   specified seek percentage. It reads the WAV file header to determine the
*   total size and calculates the offset accordingly. The function assumes that
*   the WAV file is well-formed and contains valid data.
*
* Parameters:
*   wav_data     : Pointer to the WAV file data in memory.
*   wav_size     : Total size of the WAV file data in bytes.
*   seek_percent : Percentage of the total WAV file size to seek to (0-100).
*
* Return:
*   int32_t      : The calculated seek offset in bytes. Returns -1 if the
*                  WAV file is invalid or if the seek percentage is out of range.
*
*******************************************************************************/
static int32_t get_wav_seek_offset(const uint8_t *wav_data, uint32_t wav_size, uint8_t seek_percent)
{
    int32_t seek_offset = -1;
    int32_t total_samples = 0;
    int32_t seek_samples = 0;

    if (process_music_player_wav_file((uint8_t *)wav_data, wav_size))
    {
        total_samples = ((int32_t)music_player_index_max - (int32_t)music_player_index) / sizeof(int16_t);
        if (total_samples > 0)
        {
            seek_samples = ((int32_t)total_samples * (int32_t)seek_percent) / 100;
            seek_offset = (int32_t)music_player_index + (seek_samples * sizeof(int16_t));
        }
    }

    return seek_offset;
}
#endif /* ENABLE_MP3_PLAYBACK */

/* [] END OF FILE */
