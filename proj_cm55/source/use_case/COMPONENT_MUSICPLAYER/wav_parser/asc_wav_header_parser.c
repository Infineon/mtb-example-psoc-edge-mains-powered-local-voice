/******************************************************************************
* File Name : asc_wav_header_parser.c
*
* Description: wav header parser source.
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

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "cy_result.h"

/******************************************************************************
 * Function Name: asc_byte_stream_read()
 *******************************************************************************
 * Summary:
 *          Reads byte stream.
 *
 * Parameters:
 *          Byte stream configuration.
 *
 *
 * Return:
 *          Size of read bytes.
 *
 ******************************************************************************/

int asc_byte_stream_read(unsigned char *bitstream, unsigned char *bytes, unsigned int size, unsigned int *current_read_pos, unsigned int bitstream_max )
{
    int      i = 0;

    unsigned short int i_buff_size = 0;
    for (i = 0; i < size && (*current_read_pos < bitstream_max); i++)
    {
        bytes[i] = bitstream[*current_read_pos];
        *current_read_pos = *current_read_pos + 1;
        i_buff_size++;
    }

    return i_buff_size;
}

/******************************************************************************
 * Function Name: cy_wav_header_decode()
 *******************************************************************************
 * Summary: Decodes wav header.
 *
 *
 * Parameters:
 *      Parameters of the byte stream.
 *
 *
 * Return:
 *      Success/Failure of the decode.
 *
 ******************************************************************************/

bool cy_wav_header_decode( uint32_t *n_channels,
                           uint32_t *i_channel_mask,
                           uint32_t *sample_rate,
                           uint32_t *pcm_sz,
                           int32_t *length,
                           unsigned char *pBit_stream,
                           unsigned int *asc_stream_read_counter,
                           unsigned int bit_stream_max )
{
    int8_t wav_hdr[40 + 36];
    int8_t data_start[4];
    int16_t num_ch;
    uint32_t f_samp;
    int16_t output_format;
    int32_t check, count = 0;
    bool wav_format_pcm = 0, wav_format_extensible = 0;
    uint16_t cbSize = 0;

    *i_channel_mask = 0;
    *asc_stream_read_counter = 0;

    if(asc_byte_stream_read(pBit_stream, (unsigned char*)wav_hdr, 40, asc_stream_read_counter, bit_stream_max) != 40)
    {
        return 1;
    }

    if (wav_hdr[0] != 'R' && wav_hdr[1] != 'I' && wav_hdr[2] != 'F' && wav_hdr[3] != 'F')
    {
        return 1;
    }

    if (wav_hdr[20] == 01 && wav_hdr[21] == 00)
    {
        wav_format_pcm = 1;
    }
    else if (wav_hdr[20] == ((int8_t)0xFE) && wav_hdr[21] == ((int8_t)0xFF))
    {
        wav_format_extensible = 1;
    }
    else
    {
        return 1;
    }

    num_ch = (int16_t)((uint8_t)wav_hdr[23] * 256 + (uint8_t)wav_hdr[22]);
    f_samp = ((uint8_t)wav_hdr[27] * 256 * 256 * 256);
    f_samp += ((uint8_t)wav_hdr[26] * 256 * 256);
    f_samp += ((uint8_t)wav_hdr[25] * 256);
    f_samp += ((uint8_t)wav_hdr[24]);

    output_format = ((uint8_t)wav_hdr[35] * 256);
    output_format += ((uint8_t)wav_hdr[34]);

    *n_channels = num_ch;
    *sample_rate = f_samp;
    *pcm_sz = output_format;

    if (wav_format_pcm)
    {
        data_start[0] = wav_hdr[36];
        data_start[1] = wav_hdr[37];
        data_start[2] = wav_hdr[38];
        data_start[3] = wav_hdr[39];
    }
    else if (wav_format_extensible)
    {
        cbSize |= ((uint8_t)wav_hdr[37] << 8);
        cbSize |= ((uint8_t)wav_hdr[36]);

        if(asc_byte_stream_read(pBit_stream,(unsigned char *)&(wav_hdr[40]),(uint16_t)(cbSize - 2 + 4),asc_stream_read_counter, bit_stream_max)!= (uint16_t)(cbSize - 2 + 4))
        {
            return 1;
        }


        *i_channel_mask = 0;
        *i_channel_mask |= (uint8_t)wav_hdr[43] << 24;
        *i_channel_mask |= (uint8_t)wav_hdr[42] << 16;
        *i_channel_mask |= (uint8_t)wav_hdr[41] << 8;
        *i_channel_mask |= (uint8_t)wav_hdr[40];

        data_start[0] = wav_hdr[40 + cbSize - 2 + 0];
        data_start[1] = wav_hdr[40 + cbSize - 2 + 1];
        data_start[2] = wav_hdr[40 + cbSize - 2 + 2];
        data_start[3] = wav_hdr[40 + cbSize - 2 + 3];
    }

    check = 1;
    while (check)
    {
        if (data_start[0] == 'd' && data_start[1] == 'a' && data_start[2] == 't' &&
            data_start[3] == 'a')
        {
            asc_byte_stream_read(pBit_stream,(unsigned char *)length,4,asc_stream_read_counter, bit_stream_max);
            check = 0;
        }
        else
        {
            data_start[0] = data_start[1];
            data_start[1] = data_start[2];
            data_start[2] = data_start[3];

            asc_byte_stream_read(pBit_stream,(unsigned char*)&(data_start[3]),1,asc_stream_read_counter, bit_stream_max);
        }

        count++;

        if (count > 160)
        {
            *length = 0xffffffff;
            return 1;
        }
    }

    return 0;
}

/* [] END OF FILE */
