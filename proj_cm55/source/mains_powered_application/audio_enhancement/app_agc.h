/******************************************************************************
* File Name : app_agc.h
*
* Description :
* Header file for Automatic Gain Control
********************************************************************************
 * (c) 2025, Infineon Technologies AG, or an affiliate of Infineon
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

#ifndef __APP_AGC_H__
#define __APP_AGC_H__

#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */

#include "ifx_pre_post_process.h"
#include "ifx_sp_utils_priv.h"
#include "stdlib.h"
//#include "ifx_agc_config_prms.h"

/*******************************************************************************
* Macros
*******************************************************************************/

#define AGC_SUCCESS                 (0)
#define AGC_FAILURE                 (-1)

#define AGC_PARAM_SAMPLING_RATE     (16000)
#define AGC_PARAM_FRAME_SIZE        (AGC_PARAM_SAMPLING_RATE/100)
#define AGC_PARAM_SNR_TH            (15)
#define AGC_PARAM_TH_REL            (21)
#define AGC_PARAM_ATT_TIME_MS       (1)
#define AGC_PARAM_RELEASE_TIME_MS   (30)
#define AGC_PARAM_RATIO             (5)
#define AGC_PARAM_MGAIN_DB          (15)
#define AGC_PARAM_SMOOTH_MODE       (0)

/*******************************************************************************
 * Function Prototypes
 *******************************************************************************/
int agc_init(void);
int agc_process(int16_t *input_data, int16_t *output_data);
int agc_deinit(void);


#ifdef __cplusplus
} /*extern "C" */
#endif  /* __cplusplus */
#endif /* __APP_AGC_H__ */