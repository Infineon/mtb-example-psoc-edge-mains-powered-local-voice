/******************************************************************************
* File Name:   mp3_seek.h
*
* Description: Header file for music player implemenation of mains powered
*
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

#ifndef MP3_SEEK_H_
#define MP3_SEEK_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/*******************************************************************************
* Header Files
*******************************************************************************/
#include "cybsp.h"
#include "app_logger.h"

#ifdef ENABLE_MP3_PLAYBACK
/*******************************************************************************
* Data structure and enumeration
********************************************************************************/
/* Data-type for MP3 Frame Info */
typedef struct {
    uint32_t offset;
    uint32_t frame_size;
    uint32_t frame_samples;
    uint32_t bitrate;
    uint32_t sampling_rate;
    uint8_t  mpeg_ver;
    uint8_t  layer;
} mp3_frame_info_t;

/*******************************************************************************
* Function Prototypes
********************************************************************************/
uint32_t get_mp3_file_duration_ms(const uint8_t *mp3_data, uint32_t mp3_size);
uint32_t get_total_samples_mp3(const uint8_t *mp3_data, uint32_t mp3_size);
int32_t get_mp3_seek_offset(const uint8_t *mp3_data, uint32_t mp3_size, uint8_t seek_percent);
void parse_mp3_frame(const uint8_t *mp3_data, uint32_t mp3_size,
                     mp3_frame_info_t *info, int32_t *frame_start_offset);

#ifdef __cplusplus
} /* extern C */
#endif /* __cplusplus */

#endif /* ENABLE_MP3_PLAYBACK */
#endif /* MP3_SEEK_H_ */

/* [] END OF FILE */
