/******************************************************************************
* File Name : app_hpf.c
*
* Description : Source file for High Pass Filter before audio playback (48kHz)
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
#include "app_hpf.h"
#include "app_logger.h"

/*******************************************************************************
* Macros
*******************************************************************************/
#define HPF_OUTPUT_BUFF_SIZE       (960)
#define HPF_PERSISTENT_MEM_SIZE    (140)
#define HPF_SCRATCH_SIZE           (4)
/*******************************************************************************
* Global Variables
*******************************************************************************/

void *hpf_obj = NULL;
ifx_stc_pre_post_process_info_t hpf_info;

/* Static buffers for HPF (48kHz)*/
int16_t hpf_out_buffer[HPF_OUTPUT_BUFF_SIZE/2] = {0};
int8_t hpf_persistent_mem[HPF_PERSISTENT_MEM_SIZE] = {0};
int8_t hpf_scratch_mem[HPF_SCRATCH_SIZE] = {0};

int16_t *hpf_output_buffer = hpf_out_buffer;

/*******************************************************************************
 * Function Name: hpf_init
 *******************************************************************************
* Summary:
*  Initialize High Pass filter.
*
* Parameters:
*  None
*
* Return:
*  HPF_SUCCESS - On Success
*  HPF_FAILURE - On Failure
*
*******************************************************************************/
int8_t hpf_init(int32_t sampling_rate)
{

    uint32_t ErrIdx = 0;
    int32_t ret = 0;
    int32_t hpf_config_prms[] = {
    0,      // configuration version
    sampling_rate,  // sampling rate
    sampling_rate/100,    // input frame size
    IFX_PRE_PROCESS_IP_COMPONENT_HPF, // IP component ID: preprocess HPF
    0       // number of parameters
    };
        
    ret = ifx_pre_post_process_parse(hpf_config_prms, &hpf_info);
    
    if (ret != HPF_SUCCESS) {
        app_log_print("Error! HPF parsing failed! Error code=%x\n", ret);
        return HPF_FAILURE;
    }

    hpf_info.memory.persistent_mem_pt = hpf_persistent_mem;
    hpf_info.memory.scratch_mem_pt = hpf_scratch_mem;
    
    if (!hpf_info.memory.persistent_mem_pt || !hpf_info.memory.scratch_mem_pt || !hpf_output_buffer) {
        app_log_print("Error! HPF memory allocation failed!\n");
        return HPF_FAILURE;
    }

    ErrIdx = ifx_pre_post_process_init(hpf_config_prms, &hpf_obj, &hpf_info);
    if (ErrIdx) {
        app_log_print("Error! HPF initialization failed! Error code=%x\n", ErrIdx);
        return HPF_FAILURE;
    }

    return HPF_SUCCESS;
    
}

/*******************************************************************************
 * Function Name: hpf_process
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
int8_t hpf_process(int16_t *in_buffer, int16_t *out_buffer)
{
    uint32_t ErrIdx = 0;
    
    ErrIdx = ifx_time_pre_process(in_buffer, NULL, hpf_obj, IFX_PRE_PROCESS_IP_COMPONENT_HPF, out_buffer, NULL);
    if (ErrIdx != IFX_SP_ENH_SUCCESS) {
        app_log_print("HPF processing failed! Error code=%x\n", ErrIdx);
        return HPF_FAILURE;
    }
    return HPF_SUCCESS;
}


/* [] END OF FILE */
