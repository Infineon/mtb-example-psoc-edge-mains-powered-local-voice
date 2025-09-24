/******************************************************************************
* File Name : audio_conv_utils.c
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

/*******************************************************************************
* Header Files
*******************************************************************************/
#include "cy_pdl.h"
#include "cycfg.h"
#include "cy_pdl.h"
#include "cy_log.h"


/*******************************************************************************
* Macros
*******************************************************************************/
#define MAX_AUD_SAMPLES_PER_CHANNEL_FOR_10MS_DATA (160)

/*******************************************************************************
* Function Name: convert_stereo_non_interleaved_to_stereo_interleaved
********************************************************************************
* Summary:
* Converts non interleaved audio to interleaved stereo (16kHz, 10ms frame)
*
* Parameters:
*  stereo_non_interleaved - (In) non interleaved data
*  stereo_interleaved - (Out) interleaved data
* Return:
*  None
*
*******************************************************************************/

void convert_stereo_non_interleaved_to_stereo_interleaved(
        uint16_t *stereo_non_interleaved,
        uint16_t *stereo_interleaved)
{
    int i = 0;
    for (i = 0; i < MAX_AUD_SAMPLES_PER_CHANNEL_FOR_10MS_DATA; i++)
    {
        *stereo_interleaved = *stereo_non_interleaved;
        stereo_interleaved++;
        *stereo_interleaved = *(stereo_non_interleaved+MAX_AUD_SAMPLES_PER_CHANNEL_FOR_10MS_DATA);
        stereo_interleaved++;
        stereo_non_interleaved++;
    }
    return;
}

/*******************************************************************************
* Function Name: convert_interleaved_to_stereo_non_interleaved
********************************************************************************
* Summary:
* Converts interleaved stereo to non interleaved audio (16kHz, 10ms frame)
*
* Parameters:
*  stereo_interleaved - (In) interleaved data
*  stereo_non_interleaved - (Out) non interleaved data
* Return:
*  None
*
*******************************************************************************/

void convert_interleaved_to_stereo_non_interleaved(
        uint16_t *stereo_interleaved,
        uint16_t *stereo_non_interleaved)
{
    int i = 0;
    for (i = 0; i < MAX_AUD_SAMPLES_PER_CHANNEL_FOR_10MS_DATA; i++)
    {
        *stereo_non_interleaved = *stereo_interleaved;
        stereo_interleaved++;

        *(stereo_non_interleaved + MAX_AUD_SAMPLES_PER_CHANNEL_FOR_10MS_DATA) =
                *stereo_interleaved;
        stereo_interleaved++;

        stereo_non_interleaved++;
    }
    return;
}

/*******************************************************************************
* Function Name: convert_stereo_interleaved_to_mono
********************************************************************************
* Summary:
* Converts interleaved stereo to mono (16kHz, 10ms frame) 
*
* Parameters:
*  stereo - (In) interleaved data
*  mono - (Out) mono data
* Return:
*  None
*
*******************************************************************************/

void convert_stereo_interleaved_to_mono(uint16_t *stereo, uint16_t *mono)
{
    int i =0;

    for (i = 0; i < MAX_AUD_SAMPLES_PER_CHANNEL_FOR_10MS_DATA; i++)
    {
        *mono = *stereo;
        stereo += 2;
        mono += 1;
    }
}

/*******************************************************************************
* Function Name: convert_mono_to_stereo_interleaved
********************************************************************************
* Summary:
* Converts mono to stereo (16kHz, 10ms frame) 
*
* Parameters:
*  stereo - interleaved data
*  mono -  mono data
* Return:
*  None
*
*******************************************************************************/
void convert_mono_to_stereo_interleaved(uint16_t *stereo,uint16_t *mono)
{
    int i =0;

    for (i = 0; i < MAX_AUD_SAMPLES_PER_CHANNEL_FOR_10MS_DATA; i++)
    {
        *stereo = *mono;
        stereo++;
        *stereo = *mono;
        stereo++;
        mono++;
    }
}


/*******************************************************************************
* Function Name: swap_stereo_channel
********************************************************************************
* Summary:
* Swaps L and R channels
*
* Parameters:
*  in - Input Stereo audio
*  out - Output Stereo audio
* Return:
*  None
*
*******************************************************************************/

void swap_stereo_channel(uint16_t *in, uint16_t *out)
{
    
    int index = 0;
    
    for (index = 0; index < MAX_AUD_SAMPLES_PER_CHANNEL_FOR_10MS_DATA; index++)
    {
        out[2 * index + 1]=in[2 * index];
        out[2 * index]=in[2 * index + 1];
    }
    
    
}



/* [] END OF FILE */
