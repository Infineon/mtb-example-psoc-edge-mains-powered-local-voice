/******************************************************************************
* File Name : audio_conv_utils.c
*
* Description :
* Audio conversion utilities.
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
*  samples_per_10ms - (In) number of samples per 10ms
* Return:
*  None
*
*******************************************************************************/

void convert_stereo_interleaved_to_mono(uint16_t *stereo, uint16_t *mono, int16_t samples_per_10ms)
{
    int i =0;

    for (i = 0; i < samples_per_10ms; i++)
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
*  samples_per_10ms - number of samples per 10ms
* Return:
*  None
*
*******************************************************************************/
void convert_mono_to_stereo_interleaved(uint16_t *stereo,uint16_t *mono, int16_t samples_per_10ms)
{
    int i =0;

    for (i = 0; i < samples_per_10ms; i++)
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
