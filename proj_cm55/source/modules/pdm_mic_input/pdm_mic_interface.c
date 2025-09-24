/******************************************************************************
* File Name : pdm_mic_interface.c
*
* Description :
* Code for PDM mic interface. Used by both CM33 and CM55 cores.
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

/*******************************************************************************
* Header Files
*******************************************************************************/
#include "cy_pdl.h"
#include "cybsp.h"
#include "cy_log.h"

#include "pdm_mic_interface.h"
#include "audio_input_configuration.h"
#include "app_logger.h"
#include <math.h>

/*******************************************************************************
* Macros
*******************************************************************************/
#ifdef ENABLE_STEREO_INPUT_FEED
#define MIC_MODE                         (2u)
#else
#define MIC_MODE                         (1u)
#endif /* ENABLE_STEREO_INPUT_FEED */


#define PDM_PCM_HW_FIFO_SIZE             (64u)
#define PDM_PCM_FRAME_INTR_CNT           (5u)
#define MIC_RX_SAMPLES_COUNT             (160*MIC_MODE)

#define PDM_SET_GAIN_ERROR               (-1)


/* The number of interrupts to get frame of 10 msec samples.
    5 interrupts of 2msec makes 10msec frame */
/* 10msec data is 320 samples in STEREO mode*/
/* 10msec data is 160 samples in STEREO mode*/
#ifdef ENABLE_STEREO_INPUT_FEED
#define HALF_FIFO_SIZE              (PDM_PCM_HW_FIFO_SIZE)
#else
#define HALF_FIFO_SIZE              (PDM_PCM_HW_FIFO_SIZE/2)
#endif /* ENABLE_STEREO_INPUT_FEED */

/*******************************************************************************
* Global Variables
*******************************************************************************/
int16_t mic_audio_app_buffer_ping[MIC_RX_SAMPLES_COUNT] = {0};
int16_t mic_audio_app_buffer_pong[MIC_RX_SAMPLES_COUNT] = {0};

volatile bool pdm_pcm_flag = false;
volatile bool pdm_pcm_frame_flag = false;
volatile uint8_t pdm_pcm_intr_cnt = 0;
int16_t audio_frame[HALF_FIFO_SIZE] = {0};

int16_t* ping_pong_local_pointer = NULL;
int16_t* ping_pong_buffer_pointer = NULL;

volatile int16_t *audio_data_ptr = NULL;

/* PDM/PCM interrupt configuration parameters */
const cy_stc_sysint_t PDM_IRQ_cfg = {
    .intrSrc = (IRQn_Type)CYBSP_PDM_CHANNEL_3_IRQ,
    .intrPriority = PDM_PCM_INTR_PRIORITY
};

/*******************************************************************************
* Functions Prototypes
*******************************************************************************/
extern void audio_mic_data_feed_cm55(int16_t *audio_data);

/*******************************************************************************
 * Function Name: pdm_interrupt_handler
 ********************************************************************************
 * Summary:
 *  PDM PCM converter ISR callback.
 *  Populates ping/pong audio data buffer every 10msec.
 *
 * Parameters:
 *  void
 *
 * Return:
 *  void
 *
 *******************************************************************************/
void pdm_interrupt_handler(void)
{
    static bool ping_pong = false;
    volatile uint32_t int_stat;

    /* Choose the ping pong buffer */
    if(pdm_pcm_intr_cnt == 0)
    {
        if (ping_pong)
        {
            ping_pong_local_pointer = mic_audio_app_buffer_ping;
        }
        else
        {
            ping_pong_local_pointer = mic_audio_app_buffer_pong;
        }
        ping_pong_buffer_pointer = ping_pong_local_pointer;
    }

    int_stat = Cy_PDM_PCM_Channel_GetInterruptStatusMasked(PDM0, RIGHT_CH_INDEX);
    if(CY_PDM_PCM_INTR_RX_TRIGGER & int_stat)
    {
        for(uint8_t i=0; i < RX_FIFO_TRIG_LEVEL; i++)
        {
#ifdef ENABLE_STEREO_INPUT_FEED
            int32_t data = (int32_t)Cy_PDM_PCM_Channel_ReadFifo(PDM0, LEFT_CH_INDEX);
            *(ping_pong_buffer_pointer) = (int16_t)(data);
            ping_pong_buffer_pointer++;

            data = (int32_t)Cy_PDM_PCM_Channel_ReadFifo(PDM0, RIGHT_CH_INDEX);
            *(ping_pong_buffer_pointer) = (int16_t)(data);
            ping_pong_buffer_pointer++;
#else
            int32_t data = (int32_t)Cy_PDM_PCM_Channel_ReadFifo(PDM0, LEFT_CH_INDEX);
            data = (int32_t)Cy_PDM_PCM_Channel_ReadFifo(PDM0, RIGHT_CH_INDEX);
            *(ping_pong_buffer_pointer) = (int16_t)(data);
            ping_pong_buffer_pointer++;
#endif
        }

        if (pdm_pcm_intr_cnt < PDM_PCM_FRAME_INTR_CNT)
        {
            pdm_pcm_intr_cnt++;
        }

        if (PDM_PCM_FRAME_INTR_CNT == pdm_pcm_intr_cnt)
        {
            pdm_pcm_intr_cnt = 0;

            audio_mic_data_feed_cm55(ping_pong_local_pointer);

            ping_pong = !ping_pong;
            if (ping_pong)
            {
                ping_pong_local_pointer = mic_audio_app_buffer_ping;
            }
            else
            {
                ping_pong_local_pointer = mic_audio_app_buffer_pong;
            }
            ping_pong_buffer_pointer = ping_pong_local_pointer;
        }

        Cy_PDM_PCM_Channel_ClearInterrupt(PDM0, RIGHT_CH_INDEX, CY_PDM_PCM_INTR_RX_TRIGGER);
    }

    if((CY_PDM_PCM_INTR_RX_FIR_OVERFLOW | CY_PDM_PCM_INTR_RX_OVERFLOW |
        CY_PDM_PCM_INTR_RX_IF_OVERFLOW | CY_PDM_PCM_INTR_RX_UNDERFLOW) & int_stat)
    {
        /*printf("Error: channel %u RX FIFO Error INTR_STATUS = %lu\n",
               (unsigned int) RIGHT_CH_INDEX, (unsigned long int) int_stat);*/
        Cy_PDM_PCM_Channel_ClearInterrupt(PDM0, RIGHT_CH_INDEX, CY_PDM_PCM_INTR_MASK);
    }
}

/*******************************************************************************
* Function Name: pdm_mic_interface_init
********************************************************************************
* Summary:
* Initalize the PDM block with the required settings for PSOC Edge.
*
* Parameters:
*  None
*
* Return:
*  CY_RSLT_SUCCESS
*
*******************************************************************************/
cy_rslt_t pdm_mic_interface_init(void)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;
    int16_t gain_scale = 0;

    /* Initialize PDM/PCM block */
    cy_en_pdm_pcm_status_t volatile status = Cy_PDM_PCM_Init(PDM0, &CYBSP_PDM_config);
    
    if (CY_PDM_PCM_SUCCESS !=status)
    {
        app_log_print("PDM/PCM PDL Initialization has failed! \r\n");
        CY_ASSERT(0);
    }

    /* Enable PDM channel, we will activate channel for record later */
    Cy_PDM_PCM_Channel_Enable(PDM0, LEFT_CH_INDEX);
    Cy_PDM_PCM_Channel_Enable(PDM0, RIGHT_CH_INDEX);

    /* Initialize PDM/PCM channel 0 -Left, 1 -Right */
    Cy_PDM_PCM_Channel_Init(PDM0, &LEFT_CH_CONFIG, (uint8_t)LEFT_CH_INDEX);
    Cy_PDM_PCM_Channel_Init(PDM0, &RIGHT_CH_CONFIG, (uint8_t)RIGHT_CH_INDEX);

    /* Set the gain for both left and right channels. */
 
    gain_scale = convert_db_to_pdm_scale((float)PDM_MIC_GAIN_VALUE);
    app_log_print("Setting default PDM gain to %f dB and %d scale \r\n",(float)PDM_MIC_GAIN_VALUE,gain_scale);
    set_pdm_pcm_gain(gain_scale);

    /* As registred for right channel, clear and set maks for it. */
    Cy_PDM_PCM_Channel_ClearInterrupt(PDM0, RIGHT_CH_INDEX, CY_PDM_PCM_INTR_MASK);
    Cy_PDM_PCM_Channel_SetInterruptMask(PDM0, RIGHT_CH_INDEX, CY_PDM_PCM_INTR_MASK);

    /* Register the PDM/PCM hardware block IRQ handler */
    if(CY_SYSINT_SUCCESS != Cy_SysInt_Init(&PDM_IRQ_cfg, &pdm_interrupt_handler))
    {
        app_log_print("PDM/PCM Initialization has failed! \r\n");
        CY_ASSERT(0);
    }

    NVIC_ClearPendingIRQ(PDM_IRQ_cfg.intrSrc);
    NVIC_EnableIRQ(PDM_IRQ_cfg.intrSrc);

    app_pdm_pcm_activate();

    return result;
}

/*******************************************************************************
* Function Name: pdm_mic_interface_deinit
********************************************************************************
* Summary:
*  De-initializes PDM-PCM block.
*
* Parameters:
*  None
*
* Return:
*  CY_RSLT_SUCCESS
*
*******************************************************************************/
cy_rslt_t pdm_mic_interface_deinit(void)
{
    return CY_RSLT_SUCCESS;
}


/*******************************************************************************
 * Function Name: app_pdm_pcm_activate
 ********************************************************************************
* Summary: This function activates the left and righ channel.
*
* Parameters:
*  None
*
* Return:
*  none
*
*******************************************************************************/
void app_pdm_pcm_activate(void)
{
    /* Activate recording from channel after init Activate Channel */
    Cy_PDM_PCM_Activate_Channel(PDM0, LEFT_CH_INDEX);
    Cy_PDM_PCM_Activate_Channel(PDM0, RIGHT_CH_INDEX);
}

/*******************************************************************************
* Function Name: app_pdm_pcm_deactivate
********************************************************************************
* Summary: This function activates the left and righ channel.
*
* Parameters:
*  none
*
* Return :
*  none
*
*******************************************************************************/
void app_pdm_pcm_deactivate(void)
{
    Cy_PDM_PCM_DeActivate_Channel(PDM0, LEFT_CH_INDEX);
    Cy_PDM_PCM_DeActivate_Channel(PDM0, RIGHT_CH_INDEX);
}

/*******************************************************************************
 * Function Name: convert_db_to_pdm_scale
 ********************************************************************************
 * Summary:
 * Converts dB to PDM scale (fixed scale from 0 to 31)
 * Refer
 *
 * Parameters:
 *  gain  : gain in dB
 * Return:
 *  Scale value
 *
 *******************************************************************************/

int16_t convert_db_to_pdm_scale(float db)
{
    if (db>=PDM_PCM_MIN_GAIN && db<=PDM_PCM_SEL_GAIN_NEGATIVE_103DB)
    {
        return CY_PDM_PCM_SEL_GAIN_NEGATIVE_103DB; 
    }
    else if (db>PDM_PCM_SEL_GAIN_NEGATIVE_103DB && db<=PDM_PCM_SEL_GAIN_NEGATIVE_97DB)
    {
        return CY_PDM_PCM_SEL_GAIN_NEGATIVE_97DB;
    }
     else if (db>PDM_PCM_SEL_GAIN_NEGATIVE_97DB && db<=PDM_PCM_SEL_GAIN_NEGATIVE_91DB)
    {
        return CY_PDM_PCM_SEL_GAIN_NEGATIVE_91DB;
    }
     else if (db>PDM_PCM_SEL_GAIN_NEGATIVE_91DB && db<=PDM_PCM_SEL_GAIN_NEGATIVE_85DB)
    {
        return CY_PDM_PCM_SEL_GAIN_NEGATIVE_85DB;
    }
     else if (db>PDM_PCM_SEL_GAIN_NEGATIVE_85DB && db<=PDM_PCM_SEL_GAIN_NEGATIVE_79DB)
    {
        return CY_PDM_PCM_SEL_GAIN_NEGATIVE_79DB;
    }
     else if (db>PDM_PCM_SEL_GAIN_NEGATIVE_79DB && db<=PDM_PCM_SEL_GAIN_NEGATIVE_73DB)
    {
        return CY_PDM_PCM_SEL_GAIN_NEGATIVE_73DB;
    }
     else if (db>PDM_PCM_SEL_GAIN_NEGATIVE_73DB && db<=PDM_PCM_SEL_GAIN_NEGATIVE_67DB)
    {
        return CY_PDM_PCM_SEL_GAIN_NEGATIVE_67DB;
    }
     else if (db>PDM_PCM_SEL_GAIN_NEGATIVE_67DB && db<=PDM_PCM_SEL_GAIN_NEGATIVE_61DB)
    {
        return CY_PDM_PCM_SEL_GAIN_NEGATIVE_61DB;
    }
     else if (db>PDM_PCM_SEL_GAIN_NEGATIVE_61DB && db<=PDM_PCM_SEL_GAIN_NEGATIVE_55DB)
    {
        return CY_PDM_PCM_SEL_GAIN_NEGATIVE_55DB;
    }
     else if (db>PDM_PCM_SEL_GAIN_NEGATIVE_55DB && db<=PDM_PCM_SEL_GAIN_NEGATIVE_49DB)
    {
        return CY_PDM_PCM_SEL_GAIN_NEGATIVE_49DB;
    }
     else if (db>PDM_PCM_SEL_GAIN_NEGATIVE_49DB && db<=PDM_PCM_SEL_GAIN_NEGATIVE_43DB)
    {
        return CY_PDM_PCM_SEL_GAIN_NEGATIVE_43DB;
    }
     else if (db>PDM_PCM_SEL_GAIN_NEGATIVE_43DB && db<=PDM_PCM_SEL_GAIN_NEGATIVE_37DB)
    {
        return CY_PDM_PCM_SEL_GAIN_NEGATIVE_37DB;
    }
     else if (db>PDM_PCM_SEL_GAIN_NEGATIVE_37DB && db<=PDM_PCM_SEL_GAIN_NEGATIVE_31DB)
    {
        return CY_PDM_PCM_SEL_GAIN_NEGATIVE_31DB;
    }
     else if (db>PDM_PCM_SEL_GAIN_NEGATIVE_31DB && db<=PDM_PCM_SEL_GAIN_NEGATIVE_25DB)
    {
        return CY_PDM_PCM_SEL_GAIN_NEGATIVE_25DB;
    }
     else if (db>PDM_PCM_SEL_GAIN_NEGATIVE_25DB && db<=PDM_PCM_SEL_GAIN_NEGATIVE_19DB)
    {
        return CY_PDM_PCM_SEL_GAIN_NEGATIVE_19DB;
    }
     else if (db>PDM_PCM_SEL_GAIN_NEGATIVE_19DB && db<=PDM_PCM_SEL_GAIN_NEGATIVE_13DB)
    {
        return CY_PDM_PCM_SEL_GAIN_NEGATIVE_13DB;
    }
     else if (db>PDM_PCM_SEL_GAIN_NEGATIVE_13DB && db<=PDM_PCM_SEL_GAIN_NEGATIVE_7DB)
    {
        return CY_PDM_PCM_SEL_GAIN_NEGATIVE_7DB;
    }
    else if (db>PDM_PCM_SEL_GAIN_NEGATIVE_7DB && db<=PDM_PCM_SEL_GAIN_NEGATIVE_1DB)
    {
        return CY_PDM_PCM_SEL_GAIN_NEGATIVE_1DB;
    }
     else if (db>PDM_PCM_SEL_GAIN_NEGATIVE_1DB && db<=PDM_PCM_SEL_GAIN_5DB)
    {
        return CY_PDM_PCM_SEL_GAIN_5DB;
    }
     else if (db>PDM_PCM_SEL_GAIN_5DB && db<=PDM_PCM_SEL_GAIN_11DB)
    {
        return CY_PDM_PCM_SEL_GAIN_11DB;
    }
    else if (db>PDM_PCM_SEL_GAIN_11DB && db<=PDM_PCM_SEL_GAIN_17DB)
    {
        return CY_PDM_PCM_SEL_GAIN_17DB;
    }     
    else if (db>PDM_PCM_SEL_GAIN_17DB && db<=PDM_PCM_SEL_GAIN_23DB)
    {
        return CY_PDM_PCM_SEL_GAIN_23DB;
    } 
    else if (db>PDM_PCM_SEL_GAIN_23DB && db<=PDM_PCM_SEL_GAIN_29DB)
    {
        return CY_PDM_PCM_SEL_GAIN_29DB;
    } 
    else if (db>PDM_PCM_SEL_GAIN_29DB && db<=PDM_PCM_SEL_GAIN_35DB)
    {
        return CY_PDM_PCM_SEL_GAIN_35DB;
    } 
    else if (db>PDM_PCM_SEL_GAIN_35DB && db<=PDM_PCM_SEL_GAIN_41DB)
    {
        return CY_PDM_PCM_SEL_GAIN_41DB;
    } 
    else if (db>PDM_PCM_SEL_GAIN_41DB && db<=PDM_PCM_SEL_GAIN_47DB)
    {
        return CY_PDM_PCM_SEL_GAIN_47DB;
    } 
    else if (db>PDM_PCM_SEL_GAIN_47DB && db<=PDM_PCM_SEL_GAIN_53DB)
    {
        return CY_PDM_PCM_SEL_GAIN_53DB;
    } 
    else if (db>PDM_PCM_SEL_GAIN_53DB && db<=PDM_PCM_SEL_GAIN_59DB)
    {
        return CY_PDM_PCM_SEL_GAIN_59DB;
    } 
    else if (db>PDM_PCM_SEL_GAIN_59DB && db<=PDM_PCM_SEL_GAIN_65DB)
    {
        return CY_PDM_PCM_SEL_GAIN_65DB;
    } 
    else if (db>PDM_PCM_SEL_GAIN_65DB && db<=PDM_PCM_SEL_GAIN_71DB)
    {
        return CY_PDM_PCM_SEL_GAIN_71DB;
    } 
    else if (db>PDM_PCM_SEL_GAIN_71DB && db<=PDM_PCM_SEL_GAIN_77DB)
    {
        return CY_PDM_PCM_SEL_GAIN_77DB;
    } 
    else if (db>PDM_PCM_SEL_GAIN_77DB && db<=PDM_PCM_SEL_GAIN_83DB)
    {
        return CY_PDM_PCM_SEL_GAIN_83DB;
    } 
    else if (db>PDM_PCM_SEL_GAIN_83DB && db<=PDM_PCM_MAX_GAIN)
    {
        return CY_PDM_PCM_SEL_GAIN_83DB;
    } 
    return CY_PDM_PCM_SEL_GAIN_23DB; /* Return default gain value ~20dB if not within range*/
    
}
/*******************************************************************************
 * Function Name: set_pdm_pcm_gain
 ********************************************************************************
 * 
 * Set PDM scale value for gain.
 *
 *******************************************************************************/
void set_pdm_pcm_gain(int16_t gain)
{

    Cy_PDM_PCM_SetGain(PDM0, RIGHT_CH_INDEX, gain);
    Cy_PDM_PCM_SetGain(PDM0, LEFT_CH_INDEX, gain);

}

/* [] END OF FILE */
