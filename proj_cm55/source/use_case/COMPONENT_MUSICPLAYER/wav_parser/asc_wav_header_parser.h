/******************************************************************************
* File Name : asc_wav_header_parser.h
*
* Description: Header for parsing wav files.
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

#ifndef __ASC_WAV_HEADER_PARSER_H__
#define __ASC_WAV_HEADER_PARSER_H__

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

/*******************************************************************************
* Function prototypes
*******************************************************************************/
int asc_byte_stream_read(unsigned char *bitstream, unsigned char *bytes, unsigned int size,unsigned int *current_read_pos, unsigned int bitstream_max );
bool cy_wav_header_decode( uint32_t *n_channels,
                            uint32_t *i_channel_mask, uint32_t *sample_rate,
                            uint32_t *pcm_sz, int32_t *length,
                            unsigned char *pBit_stream,unsigned int *asc_stream_read_counter,unsigned int bit_stream_max);
#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __ASC_WAV_HEADER_PARSER_H__ */
/* [] END OF FILE */
