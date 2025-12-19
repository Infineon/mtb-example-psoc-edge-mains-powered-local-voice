/******************************************************************************
* File Name : app_lpf_asrc.c
*
* Description : Source file for Low Pass Filter before ASRC processing
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
#include "app_lpf.h"
#include "app_logger.h"


/*******************************************************************************
* Global Variables
*******************************************************************************/

void *lpf_obj_44_1k = NULL;
void *lpf_obj_48k = NULL;

int16_t *lpf_buffer_44_1k = NULL;
int16_t *lpf_buffer_48k = NULL;

ifx_stc_pre_post_process_info_t lpf_info_44_1k;
ifx_stc_pre_post_process_info_t lpf_info_48k;

/*******************************************************************************
 * Function Name: lpf_init_44_1k
 *******************************************************************************
* Summary:
*  Initialize Low Pass filter for 44.1kHz sampling rate.
*
* Parameters:
*  None
*
* Return:
*  LPF_SUCCESS - On Success
*  LPF_FAILURE - On Failure
*
*******************************************************************************/
int8_t lpf_init_44_1k()
{

    uint32_t ErrIdx = 0;
    int32_t ret = 0;
    int32_t lpf_config_prms[] = {
        0,      // configuration version
        SAMPLING_RATE_44_1KHZ,  // sampling rate
        SAMPLING_RATE_44_1KHZ/100,    // input frame size
        IFX_PRE_PROCESS_IP_COMPONENT_ASRC_LPF, // IP component ID: preprocess LPF
        0       // number of parameters
    };
    
    ret = ifx_pre_post_process_parse(lpf_config_prms, &lpf_info_44_1k);
    
    if (ret != LPF_SUCCESS) {
        app_log_print("Error, LPF parsing failed-44.1kHz, Error code=%x\n", ret);
        return LPF_FAILURE;
    }

    lpf_info_44_1k.memory.persistent_mem_pt = malloc(lpf_info_44_1k.memory.persistent_mem);
    lpf_info_44_1k.memory.scratch_mem_pt = malloc(lpf_info_44_1k.memory.scratch_mem);
    
    lpf_buffer_44_1k = malloc(lpf_info_44_1k.output_size * sizeof(int16_t));
    
    /*Disable memory log
    app_log_print("LPF for 44.1kHz output buffer %d bytes\r\n",lpf_info_44_1k.output_size * sizeof(int16_t));
    app_log_print("LPF for 44.1kHz persistent memory %d bytes\r\n",lpf_info_44_1k.memory.persistent_mem);
    app_log_print("LPF for 44.1kHz scratch memory %d bytes\r\n",lpf_info_44_1k.memory.scratch_mem);
    */
    
    if (!lpf_info_44_1k.memory.persistent_mem_pt || !lpf_info_44_1k.memory.scratch_mem_pt || !lpf_buffer_44_1k) {
        app_log_print("Error! LPF 44.1kHz memory allocation failed!\n");
        return LPF_FAILURE;
    }

    ErrIdx = ifx_pre_post_process_init(lpf_config_prms, &lpf_obj_44_1k, &lpf_info_44_1k);
    if (ErrIdx) {
        app_log_print("Error! LPF 44.1kHz initialization failed! Error code=%x\n", ErrIdx);
        return LPF_FAILURE;
    }

    return LPF_SUCCESS;
    
}


/*******************************************************************************
 * Function Name: lpf_init_48k
 *******************************************************************************
* Summary:
*  Initialize Low Pass filter for 48kHz sampling rate.
*
* Parameters:
*  None
*
* Return:
*  LPF_SUCCESS - On Success
*  LPF_FAILURE - On Failure
*
*******************************************************************************/
int8_t lpf_init_48k()
{

    uint32_t ErrIdx = 0;
    int32_t ret = 0;
    int32_t lpf_config_prms[] = {
        0,      // configuration version
        SAMPLING_RATE_48KHZ,  // sampling rate
        SAMPLING_RATE_48KHZ/100,    // input frame size
        IFX_PRE_PROCESS_IP_COMPONENT_ASRC_LPF, // IP component ID: preprocess LPF
        0       // number of parameters
    };
    
    ret = ifx_pre_post_process_parse(lpf_config_prms, &lpf_info_48k);
    
    if (ret != LPF_SUCCESS) {
        app_log_print("Error, LPF parsing failed-48kHz Error code=%x\n", ret);
        return LPF_FAILURE;
    }

    lpf_info_48k.memory.persistent_mem_pt = malloc(lpf_info_48k.memory.persistent_mem);
    lpf_info_48k.memory.scratch_mem_pt = malloc(lpf_info_48k.memory.scratch_mem);
    
    lpf_buffer_48k = malloc(lpf_info_48k.output_size * sizeof(int16_t));
    
    /* Disable memory log
    app_log_print("LPF 48kHz output buffer %d bytes\r\n",lpf_info_48k.output_size * sizeof(int16_t));
    app_log_print("LPF 48kHz persistent memory %d bytes\r\n",lpf_info_48k.memory.persistent_mem);
    app_log_print("LPF 48kHz scratch memory %d bytes\r\n",lpf_info_48k.memory.scratch_mem);
    */
    
    if (!lpf_info_48k.memory.persistent_mem_pt || !lpf_info_48k.memory.scratch_mem_pt || !lpf_buffer_48k) {
        app_log_print("Error! LPF 48kHz memory allocation failed!\n");
        return LPF_FAILURE;
    }

    ErrIdx = ifx_pre_post_process_init(lpf_config_prms, &lpf_obj_48k, &lpf_info_48k);
    if (ErrIdx) {
        app_log_print("Error! LPF 48kHz initialization failed! Error code=%x\n", ErrIdx);
        return LPF_FAILURE;
    }

    return LPF_SUCCESS;
    
}

/*******************************************************************************
 * Function Name: lpf_deinit
 *******************************************************************************
* Summary:
* De-initialize LPF buffers
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/

void lpf_deinit()

{
    /* 44.1kHz */
    if (lpf_info_44_1k.memory.scratch_mem_pt != NULL)
    {
        free(lpf_info_44_1k.memory.scratch_mem_pt);
        lpf_info_44_1k.memory.scratch_mem_pt=NULL;
    }
    if (lpf_info_44_1k.memory.persistent_mem_pt != NULL)
    {
        free(lpf_info_44_1k.memory.persistent_mem_pt);    
        lpf_info_44_1k.memory.persistent_mem_pt=NULL;    
    }

    if (lpf_buffer_44_1k != NULL)
    {
        free(lpf_buffer_44_1k);
        lpf_buffer_44_1k=NULL;
    }
    
    /* 48kHz */
    if (lpf_info_48k.memory.scratch_mem_pt != NULL)
    {
        free(lpf_info_48k.memory.scratch_mem_pt);
        lpf_info_48k.memory.scratch_mem_pt=NULL;
    }
    if (lpf_info_48k.memory.persistent_mem_pt != NULL)
    {
        free(lpf_info_48k.memory.persistent_mem_pt);
        lpf_info_48k.memory.scratch_mem_pt=NULL;        
    }
    
    if (lpf_buffer_48k != NULL)
    {
        free(lpf_buffer_48k);
        lpf_buffer_48k=NULL;
    }
    
}

/*******************************************************************************
 * Function Name: lpf_process
 *******************************************************************************
* Summary:
*  Perform Low Pass Filtering on the data
*
* Parameters:
*  in_buffer - input data
*  out_buffer - low pass filtered data
*
* Return:
*  None
*
*******************************************************************************/
int8_t lpf_process(void *lpf_obj, int16_t *in_buffer, int16_t *out_buffer)
{
    uint32_t ErrIdx = 0;
        
    ErrIdx = ifx_time_pre_process(in_buffer, NULL, lpf_obj, IFX_PRE_PROCESS_IP_COMPONENT_ASRC_LPF, out_buffer, NULL);
    if (ErrIdx != IFX_SP_ENH_SUCCESS) {
        app_log_print("LPF processing failed! Error code=%x\n", ErrIdx);
        return LPF_FAILURE;
    }
    return LPF_SUCCESS;
}


/* [] END OF FILE */
