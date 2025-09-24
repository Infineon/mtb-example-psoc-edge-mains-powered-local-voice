/******************************************************************************
* File Name : app_pdm_pcm.h
*
* Description : Header file for i2s containing  function ptototypes.
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


#ifndef __APP_I2S_H__
#define __APP_I2S_H__


#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

/*******************************************************************************
* Header Files
*******************************************************************************/
#include "cy_pdl.h"
#include "mtb_hal.h"
#include "cybsp.h"

#include "mtb_tlv320dac3100.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cyabs_rtos.h"
#include "cybsp_types.h"

#include "circular_buffer.h"

/*******************************************************************************
* Macros
*******************************************************************************/

/* I2S Clock Settings for 16KHz playback */
#define I2S_16KHZ_MCLK_HZ                   (2048000UL)
#define I2S_CLK_DIV_16KHZ                   (24u)

/* I2S Clock Settings for 48KHz playback */
#define I2S_48KHZ_MCLK_HZ                   (6144000UL)
#define I2S_CLK_DIV_48KHZ                   (8u)

#define SYSCLK_DIV_CHANGE_WAIT_TIME_MS      (50u)

/* I2S word length parameter */
#define I2S_WORD_LENGTH                     (16u)

/* I2C master address */
#define I2C_ADDRESS                         (0x18)
/* I2C frequency in Hz */
#define I2C_FREQUENCY_HZ                    (100000UL)

#define I2S_HW_FIFO_SIZE                    (128u)

#define SAMPLING_RATE_16kHz                             (16000u)
#define SAMPLING_RATE_44_1kHz                           (44100u)
#define SAMPLING_RATE_48kHz                             (48000u)

/* I2S playback volume */
#define NUM_VOLUME_STEPS                                (6u)
#define DEFAULT_VOLUME_INDEX                            (2u)

#define MIN_VOLUME                                      (0u)
#define MAX_VOLUME                                      (110u)

#define MAX_FRAME_SIZE                                  (480u)
#define MAX_PLAYBACK_DATA_FRAME_SIZE                    (2 * MAX_FRAME_SIZE)

#define I2S_CHANNEL_LENGTH                              (16u)
#define I2S_WORD_LENGTH                                 (16u)

#define I2S_PLAYBACK_TASK_QUEUE_LENGTH                  (100u)
#define I2S_PRIORITY                                    (2u)

/* AEC reference */
#define AEC_REF_FRAME_SIZE                              (160u)
#define AEC_REF_SAMPLING_RATE                           (16000u)
#define AEC_REF_DATA_QUEUE_LENGTH                       (10u)
#define AEC_REF_DATA_QUEUE_CHECK_THRESHOLD              (6u)

/* Aec ref circular buffer size*/
#define AEC_REF_CBUF_SIZE                               (AEC_REF_DATA_QUEUE_LENGTH * AEC_REF_FRAME_SIZE * sizeof(int16_t))

/* ASRC related macro definitions */
#define ASRC_OUTPUT_BUFFER_SIZE                         (1000u)
#define ASRC_INPUT_SAMPLES                              (240u)
#define ASRC_NUM_ITERATIONS_PER_FRAME                   (2u)

/* I2S hardware FIFO size */
#define HW_FIFO_SIZE                                    (64u)

/* I2S Queue Samples Count */
#define I2S_QUEUE_TX_SAMPLES_PER_CH                     (HW_FIFO_SIZE / 2)

/* I2s interrupt priority */
#define I2S_INTR_PRIORITY                               (2u)

/* Delay in ticks for I2S operations */
#define I2S_OPERATION_DELAY_TICKS                       (1u)

/* Number of samples (for 2 channels) in the audio frame in the queue */
#define BDM_FRAME_SIZE                                      (2 * 160u)

/* Number of I2S TX transactions per audio frame */
#define NUM_I2S_BUFFERS_IN_AUDIO_FRAME                  (BDM_FRAME_SIZE / HW_FIFO_SIZE)



/*******************************************************************************
* Data structure and enumeration
********************************************************************************/
/* Data-type for I2S playback task's queue data */
typedef struct
{
    uint32_t data_len;
    int16_t* data;
    int16_t* aec_ref_ptr;
    uint8_t num_channels;
} i2s_playback_q_data_t;

/*******************************************************************************
* Global Variables
*******************************************************************************/
extern volatile int16_t *audio_data_ptr;
extern int32_t recorded_data_size;
extern volatile bool i2s_flag;
extern cy_stc_scb_i2c_context_t MW_I2C_context;

extern uint8_t aec_ref_16khz_data[AEC_REF_CBUF_SIZE];
extern circular_buffer_t aec_ref_cbuf;
extern QueueHandle_t aec_ref_data_q;
extern bool i2s_skip_frame;
extern const uint8_t volume_level[NUM_VOLUME_STEPS];
extern uint8_t volume_level_index;

extern uint32_t i2s_playback_sampling_rate_hz;

/*******************************************************************************
* Functions Prototypes
*******************************************************************************/
void i2s_init(uint32_t sampling_rate_hz);
void i2s_deinit(void);
void start_i2s(void);
void stop_i2s(void);

void i2s_playback_volume_control(uint8_t);

void convert_mono_to_stereo(int16_t *mono_data, uint32_t mono_data_num_samples,
                            int16_t *stereo_data);
void convert_stereo_to_mono(int16_t *stereo_data, uint32_t stereo_data_num_samples,
                            int16_t *mono_data);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __APP_I2S_H__ */
/* [] END OF FILE */
