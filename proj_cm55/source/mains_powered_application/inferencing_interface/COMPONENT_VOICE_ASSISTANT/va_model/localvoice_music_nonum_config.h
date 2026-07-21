/***************************************************************************
* File Name        : localvoice_music_nonum_config.h
*
* Description      : This the header file of WWD and NLU configuration object
*
* Related Document : See README.md
*******************************************************************************
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
#ifndef PROJ_CM55_INC_LOCALVOICE_MUSIC_NONUM_CONFIG_H_
#define PROJ_CM55_INC_LOCALVOICE_MUSIC_NONUM_CONFIG_H_

#include "mtb_wwd_nlu_common.h"

#define LOCALVOICE_MUSIC_NONUM_NO_MAX_WAKE_WORD 8
#define LOCALVOICE_MUSIC_NONUM_NO_OF_WAKE_WORD 1

#define LOCALVOICE_MUSIC_NONUM_WAKE_WORD_1 0x01

#define LOCALVOICE_MUSIC_NONUM_ALL_WAKE_WORD 0x01
void localvoice_music_nonum_wake_word_callback(mtb_wwd_nlu_events_t event);
extern char *localvoice_music_nonum_ww_str[LOCALVOICE_MUSIC_NONUM_NO_OF_WAKE_WORD];

// configuration object
extern mtb_wwd_nlu_config_t *localvoice_music_nonum_ww_nlu_configs[LOCALVOICE_MUSIC_NONUM_NO_OF_WAKE_WORD];

#endif /* PROJ_CM55_INC_LOCALVOICE_MUSIC_NONUM_CONFIG_H_ */
