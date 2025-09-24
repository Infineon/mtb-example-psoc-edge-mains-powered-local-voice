/******************************************************************************
* File Name:   mp3_seek.c
*
* Description: This file contains the MP3 Seek implementation.
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
#include "cy_retarget_io.h"
#include "FreeRTOS.h"
#include "task.h"
#include "mp3_seek.h"

#ifdef ENABLE_MP3_PLAYBACK

/******************************************************************************
* Macros
******************************************************************************/
#define MP3_SYNC_WORD 0xFFE
#define MPEG1 3
#define MPEG2 2
#define MPEG25 0
#define LAYER3 1

/*******************************************************************************
* Global Variables
********************************************************************************/
/* Bitrate table for MPEG1 Layer III [kbps] */
static const uint16_t bitrate_table[2][16] = {
    /* MPEG1 */
    {0,32,40,48,56,64,80,96,112,128,160,192,224,256,320,0},
    /* MPEG2/2.5 */
    {0,8,16,24,32,40,48,56,64,80,96,112,128,144,160,0}
};

/* Sample rate table [Hz] */
static const uint16_t samplerate_table[4][3] = {
    {11025, 12000, 8000},   /* MPEG2.5 */
    {0, 0, 0},              /* reserved */
    {22050, 24000, 16000},  /* MPEG2 */
    {44100, 48000, 32000}   /* MPEG1 */
};

/*******************************************************************************
* Function Prototypes
********************************************************************************/
static uint32_t skip_id3v2(const uint8_t *data, uint32_t size);
static int32_t parse_mp3_header(const uint8_t *data, uint32_t size, mp3_frame_info_t *info);
static int32_t parse_xing(const uint8_t *data, uint32_t size, uint32_t *frames, uint32_t *file_bytes, uint8_t *toc);
static int32_t parse_vbri(const uint8_t *data, uint32_t size, uint32_t *frames, uint32_t *file_bytes);

/*******************************************************************************
* Function Definitions
********************************************************************************/
static uint32_t skip_id3v2(const uint8_t *data, uint32_t size)
{
    if (size < 10) return 0;
    if (memcmp(data, "ID3", 3) == 0)
    {
        uint32_t tag_size = ((data[6] & 0x7F) << 21) | ((data[7] & 0x7F) << 14) |
                            ((data[8] & 0x7F) << 7) | (data[9] & 0x7F);
        return 10 + tag_size;
    }
    return 0;
}

static int32_t parse_mp3_header(const uint8_t *data, uint32_t size, mp3_frame_info_t *info)
{
    uint32_t header;
    uint32_t sync;
    uint8_t version_id;
    uint8_t layer;
    uint8_t bitrate_idx;
    uint8_t samplerate_idx;
    uint8_t padding;
    uint8_t mpeg_ver;
    uint8_t bitrate_row;
    uint32_t bitrate;
    uint32_t sampling_rate;
    uint32_t frame_samples;
    uint32_t frame_size;

    if (size < 4)
    {
        return -1;
    }

    header = (data[0]<<24) | (data[1]<<16) | (data[2]<<8) | data[3];
    sync = (header >> 20) & MP3_SYNC_WORD;
    if (sync != MP3_SYNC_WORD)
    {
        return -1;
    }

    version_id = (header >> 19) & 0x3;
    layer = (header >> 17) & 0x3;
    bitrate_idx = (header >> 12) & 0xF;
    samplerate_idx = (header >> 10) & 0x3;
    padding = (header >> 9) & 0x1;

    if (version_id == 1)
    {
        return -1;
    }
    if (layer != LAYER3)
    {
        return -1;
    }
    if (bitrate_idx == 0xF)
    {
        return -1;
    }
    if (samplerate_idx == 0x3)
    {
        return -1;
    }

    mpeg_ver = (version_id == 3) ? MPEG1 : ((version_id == 2) ? MPEG2 : MPEG25);
    bitrate_row = (mpeg_ver == MPEG1) ? 0 : 1;
    bitrate = bitrate_table[bitrate_row][bitrate_idx] * 1000;
    sampling_rate = samplerate_table[version_id][samplerate_idx];
    frame_samples = (mpeg_ver == MPEG1) ? 1152 : 576;
    frame_size = (144 * bitrate) / sampling_rate + padding;

    info->offset = 0;
    info->frame_size = frame_size;
    info->frame_samples = frame_samples;
    info->bitrate = bitrate;
    info->sampling_rate = sampling_rate;
    info->mpeg_ver = mpeg_ver;
    info->layer = layer;

    return 0;
}

static int32_t parse_xing(const uint8_t *data, uint32_t size, uint32_t *frames, uint32_t *file_bytes, uint8_t *toc)
{
    int32_t xing_offset = 0;
    uint32_t header;
    uint32_t sync;
    uint8_t channel_mode;
    uint32_t flags;
    int32_t pos;

    if (size < 120)
    {
        return 0;
    }

    header = (data[0]<<24) | (data[1]<<16) | (data[2]<<8) | data[3];
    sync = (header >> 20) & MP3_SYNC_WORD;
    if (sync != MP3_SYNC_WORD)
    {
        app_log_print("parse_xing: sync mismatch (got 0x%X, expected 0x%X)\n", sync, MP3_SYNC_WORD);
        return -1;
    }

    channel_mode = (data[3] >> 6) & 0x3;
    xing_offset = (channel_mode == 3) ? 21 : 36;
    if (size < xing_offset + 8)
    {
        return 0;
    }
    if (memcmp(data + xing_offset, "Xing", 4) != 0 && memcmp(data + xing_offset, "Info", 4) != 0)
    {
        return 0;
    }
    flags = (data[xing_offset+4]<<24) | (data[xing_offset+5]<<16) | (data[xing_offset+6]<<8) | data[xing_offset+7];
    pos = xing_offset + 8;
    if (flags & 0x1)
    {
        /* Frames */
        *frames = (data[pos]<<24) | (data[pos+1]<<16) | (data[pos+2]<<8) | data[pos+3];
        pos += 4;
    }
    if (flags & 0x2)
    {
        /* Bytes */
        *file_bytes = (data[pos]<<24) | (data[pos+1]<<16) | (data[pos+2]<<8) | data[pos+3];
        pos += 4;
    }
    if (flags & 0x4)
    {
        /* TOC */
        if (toc)
        {
            memcpy(toc, data+pos, 100);
        }
        pos += 100;
    }
    return 1;
}

static int32_t parse_vbri(const uint8_t *data, uint32_t size, uint32_t *frames, uint32_t *file_bytes)
{
    if (size < 60)
    {
        return 0;
    }
    if (memcmp(data+36, "VBRI", 4) != 0)
    {
        return 0;
    }
    *file_bytes = (data[46]<<24) | (data[47]<<16) | (data[48]<<8) | data[49];
    *frames = (data[50]<<24) | (data[51]<<16) | (data[52]<<8) | data[53];
    return 1;
}

/* Main seek function */
int32_t get_mp3_seek_offset(const uint8_t *mp3_data, uint32_t mp3_size, uint8_t seek_percent)
{
    uint32_t offset;
    mp3_frame_info_t info;
    uint32_t total_frames = 0;
    uint32_t file_bytes = 0;
    uint8_t toc[100] = {0};
    int32_t has_xing;
    int32_t has_vbri = 0;
    uint32_t samples_per_frame;
    uint32_t seek_samples;
    uint32_t seek_offset;
    uint32_t samples;
    uint32_t total_samples = 0;
    float percent;
    uint32_t toc_index;
    uint32_t toc_byte;
    uint32_t data_start;
    uint32_t data_size;
    uint32_t frame_num;
    uint32_t i;

    offset = skip_id3v2(mp3_data, mp3_size);

    /* Find first frame */
    while (offset + 4 < mp3_size)
    {
        if (parse_mp3_header(mp3_data + offset, mp3_size - offset, &info) == 0)
        {
            break;
        }
        offset++;
    }

    if (offset + 4 >= mp3_size)
    {
        app_log_print(" >> Error: No MP3 frame found\n");
        return -1;
    }

    /* Check for Xing/Info/VBRI */
    has_xing = parse_xing(mp3_data + offset, mp3_size - offset, &total_frames, &file_bytes, toc);
    if (!has_xing)
    {
        has_vbri = parse_vbri(mp3_data + offset, mp3_size - offset, &total_frames, &file_bytes);
    }

    samples_per_frame = info.frame_samples;

    if (has_xing && total_frames > 0)
    {
        /* Use TOC if present */
        percent = (float)seek_percent / 100.0f;
        toc_index = (uint32_t)(percent * 99.0f);
        toc_byte = toc[toc_index];
        data_start = offset;
        data_size = mp3_size - data_start;
        seek_offset = data_start + ((toc_byte * data_size) / 256);

        /* Find next frame sync */
        while (seek_offset + 4 < mp3_size)
        {
            if (parse_mp3_header(mp3_data + seek_offset, mp3_size - seek_offset, &info) == 0)
            {
                if (info.frame_size > 0)
                {
                    return seek_offset;
                }
            }
            seek_offset++;
        }
        return offset;
    }
    else if (has_vbri && total_frames > 0)
    {
        /* VBRI: estimate by frame count */
        total_samples = total_frames * samples_per_frame;
        seek_samples = (seek_percent * total_samples) / 100;
        frame_num = seek_samples / samples_per_frame;
        seek_offset = offset;
        for (i = 0; i < frame_num && seek_offset + 4 < mp3_size; i++)
        {
            if (parse_mp3_header(mp3_data + seek_offset, mp3_size - seek_offset, &info) != 0)
            {
                break;
            }
            seek_offset += info.frame_size;
        }
        return seek_offset;
    }
    else
    {
        /* VBR fallback: parse frames */
        seek_offset = offset;

        while (seek_offset + 4 < mp3_size)
        {
            if (parse_mp3_header(mp3_data + seek_offset, mp3_size - seek_offset, &info) != 0)
            {
                seek_offset++;
                continue;
            }
            total_samples += info.frame_samples;
            seek_offset += info.frame_size;

        }

        seek_offset = offset;
        samples = 0;
        seek_samples = (seek_percent * total_samples) / 100;
        while (seek_offset + 4 < mp3_size)
        {
            if (parse_mp3_header(mp3_data + seek_offset, mp3_size - seek_offset, &info) != 0)
            {
                seek_offset++;
                continue;
            }
            if (samples + info.frame_samples > seek_samples)
            {
                return seek_offset;
            }
            samples += info.frame_samples;
            seek_offset += info.frame_size;
        }
        return offset;
    }
}

uint32_t get_mp3_file_duration_ms(const uint8_t *mp3_data, uint32_t mp3_size)
{
    uint64_t total_samples = 0;
    uint32_t offset;
    mp3_frame_info_t info;
    uint32_t total_frames = 0;
    uint32_t file_bytes = 0;
    uint8_t toc[100] = {0};
    int32_t has_xing;
    int32_t has_vbri = 0;
    uint32_t samples_per_frame;
    uint32_t sampling_rate;
    uint32_t duration = 0;

    uint32_t seek_offset;
    uint32_t frame_counter = 0;

    offset = skip_id3v2(mp3_data, mp3_size);

    /* Find first frame */
    while (offset + 4 < mp3_size)
    {
        if (parse_mp3_header(mp3_data + offset, mp3_size - offset, &info) == 0)
        {
            break;
        }
        offset++;
    }

    if (offset + 4 >= mp3_size)
    {
        app_log_print(" >> Error: No MP3 frame found\n");
        return 0;
    }

    has_xing = parse_xing(mp3_data + offset, mp3_size - offset, &total_frames, &file_bytes, toc);
    if (!has_xing)
    {
        has_vbri = parse_vbri(mp3_data + offset, mp3_size - offset, &total_frames, &file_bytes);
    }

    app_log_print("[INFO]: has_xing=%d, has_vbri=%d, total_frames=%u, sampling_rate=%u, bitrate=%u, frame_size=%u\n",
        has_xing, has_vbri, total_frames, info.sampling_rate, info.bitrate, info.frame_size);

    samples_per_frame = info.frame_samples;
    sampling_rate = info.sampling_rate;

    if ((has_xing || has_vbri) && total_frames > 0 && sampling_rate > 0)
    {
        /* Use frame count from Xing/VBRI */
        total_samples = (uint64_t)(total_frames - 1) * samples_per_frame;
        duration = (uint32_t)((total_samples * 1000) / sampling_rate);
        app_log_print("Total samples from Xing/VBRI: %lu\n", (unsigned long) total_samples);
        app_log_print("Duration from Xing/VBRI: %u ms\n", duration);
        return duration;
    }
    else
    {
        /* VBR fallback: parse all frames */
        seek_offset = offset;
        frame_counter = 0;
        while (seek_offset + 4 < mp3_size)
        {
            if (parse_mp3_header(mp3_data + seek_offset, mp3_size - seek_offset, &info) != 0)
            {
                seek_offset++;
                continue;
            }
            total_samples += info.frame_samples;
            seek_offset += info.frame_size;
            frame_counter++;
        }
        if (sampling_rate > 0)
        {
            duration = (uint32_t)((total_samples * 1000) / sampling_rate);
            app_log_print("Total samples from VBR fallback: %lu\n", (unsigned long) total_samples);
            app_log_print("Duration from VBR fallback: %u ms (frames=%u)\n", duration, frame_counter);
            return duration;
        }
        else
        {
            app_log_print("sampling_rate is 0, can't compute duration\n");
            return 0;
        }
    }
}

uint32_t get_total_samples_mp3(const uint8_t *mp3_data, uint32_t mp3_size)
{
    uint64_t total_samples = 0;
    uint32_t offset;
    mp3_frame_info_t info;
    uint32_t total_frames = 0;
    uint32_t file_bytes = 0;
    uint8_t toc[100] = {0};
    int32_t has_xing = 0;
    int32_t has_vbri = 0;
    uint32_t samples_per_frame = 0;
    uint32_t sampling_rate = 0;

    uint32_t seek_offset = 0;

    offset = skip_id3v2(mp3_data, mp3_size);

    /* Find first frame */
    while (offset + 4 < mp3_size)
    {
        if (parse_mp3_header(mp3_data + offset, mp3_size - offset, &info) == 0)
        {
            break;
        }
        offset++;
    }

    if (offset + 4 >= mp3_size)
    {
        app_log_print(" >> Error: No MP3 frame found\n");
        return 0;
    }

    /* Check for Xing/Info/VBRI */
    has_xing = parse_xing(mp3_data + offset, mp3_size - offset, &total_frames, &file_bytes, toc);
    if (!has_xing)
    {
        has_vbri = parse_vbri(mp3_data + offset, mp3_size - offset, &total_frames, &file_bytes);
    }

    samples_per_frame = info.frame_samples;
    sampling_rate = info.sampling_rate;

    if ((has_xing || has_vbri) && total_frames > 0 && sampling_rate > 0)
    {
        /* Use frame count from Xing/VBRI */
        total_samples = (uint64_t)(total_frames - 1) * samples_per_frame;
        return total_samples;
    }
    else
    {
        /* VBR fallback: parse all frames */
        seek_offset = offset;

        while (seek_offset + 4 < mp3_size)
        {
            if (parse_mp3_header(mp3_data + seek_offset, mp3_size - seek_offset, &info) != 0)
            {
                seek_offset++;
                continue;
            }
            total_samples += info.frame_samples;
            seek_offset += info.frame_size;

        }
        if (sampling_rate > 0)
        {
            return total_samples;
        }
        else
        {
            app_log_print("sampling_rate is 0, can't compute duration\n");
            return 0;
        }
    }
}

void parse_mp3_frame(const uint8_t *mp3_data, uint32_t mp3_size,
                     mp3_frame_info_t *info, int32_t *frame_start_offset)
{
    int32_t offset;

    offset = skip_id3v2(mp3_data, mp3_size);

    /* Find first frame */
    while (offset + 4 < mp3_size)
    {
        if (parse_mp3_header(mp3_data + offset, mp3_size - offset, info) == 0)
        {
            *frame_start_offset = offset;
            break;
        }
        offset++;
    }

    if (offset + 4 >= mp3_size)
    {
        *frame_start_offset = -1;
        app_log_print(" >> Error: No MP3 frame found\n");
    }
}

#endif /* ENABLE_MP3_PLAYBACK */

/* [] END OF FILE */
