/******************************************************************************
* File Name:   mp3_seek.h
*
* Description: Header file for music player implemenation of mains powered
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
