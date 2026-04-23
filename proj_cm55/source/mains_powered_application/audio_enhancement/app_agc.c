/******************************************************************************
* File Name : app_agc.c
*
* Description :
* Source file for Automatic Gain Control
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

#include "app_agc.h"
#include "app_logger.h"

/*******************************************************************************
* Macros
*******************************************************************************/


/*******************************************************************************
* Global Variables
*******************************************************************************/
int32_t post_proc_agc_prms[] = {
    0,
    AGC_PARAM_SAMPLING_RATE,
    AGC_PARAM_FRAME_SIZE,
    IFX_POST_PROCESS_IP_COMPONENT_AGC, /* AGC Component ID*/
    AGC_PARMS_SIZE, /* Number of parameters */
    AGC_PARAM_SNR_TH,
    AGC_PARAM_TH_REL,
    AGC_PARAM_ATT_TIME_MS,
    AGC_PARAM_RELEASE_TIME_MS,
    AGC_PARAM_RATIO,
    AGC_PARAM_MGAIN_DB,
    AGC_PARAM_SMOOTH_MODE
};

void *agc_obj = NULL;
int16_t *agc_output_frame = NULL;
ifx_stc_pre_post_process_info_t post_proc_agc_info;

/*******************************************************************************
* Function Name: agc_init
********************************************************************************
* Summary:
*  This function initializes AGC
*
*******************************************************************************/
int agc_init(void)
{
    uint32_t err_id = 0;
    int32_t ret = 0;

    float agc_target_level = 0.0f;

    ret = ifx_pre_post_process_parse(post_proc_agc_prms, &post_proc_agc_info);
    if (ret != 0) {
        app_log_print("AGC parse failed! Error code=%x\n", ret);
        return AGC_FAILURE;
    }

    post_proc_agc_info.memory.persistent_mem_pt = malloc(post_proc_agc_info.memory.persistent_mem);
    post_proc_agc_info.memory.scratch_mem_pt = malloc(post_proc_agc_info.memory.scratch_mem);
    agc_output_frame = malloc(post_proc_agc_info.output_size * sizeof(int16_t));

    app_log_print("AGC persistent memory %d \r\n",post_proc_agc_info.memory.persistent_mem);
    app_log_print("AGC scratch memory %d \r\n",post_proc_agc_info.memory.scratch_mem);
    app_log_print("AGC output buffer size %d \r\n",post_proc_agc_info.output_size * sizeof(int16_t));

    if (!post_proc_agc_info.memory.persistent_mem_pt || !post_proc_agc_info.memory.scratch_mem_pt || !agc_output_frame) {
        app_log_print("AGC memory allocation failed!\n");
        return AGC_FAILURE;
    }

    err_id = ifx_pre_post_process_init(post_proc_agc_prms, &agc_obj, &post_proc_agc_info);
    if (err_id) {
        app_log_print("AGC initialization failed! Error code=%x\n", err_id);
        return AGC_FAILURE;
    }

    err_id = ifx_agc_set_target_level_and_mode(agc_obj, agc_target_level, SPEECH_MODE);
    if (err_id != IFX_SP_ENH_SUCCESS) {
        app_log_print("AGC set_target_level_and_mode failed! Error code=%x\n", err_id);
        return AGC_FAILURE;
    }

    return AGC_SUCCESS;
}

/*******************************************************************************
* Function Name: agc_process
********************************************************************************
* Summary:
*  This function performs AGC on input data
*
*******************************************************************************/
int agc_process(int16_t *input_data, int16_t *output_data)
{
    uint32_t err_id = 0;

    //output_data=agc_output_frame;

    if (output_data==NULL)
    {
        app_log_print("AGC output is NULL \r\n");
        return AGC_FAILURE;
    }
    err_id = ifx_time_post_process(input_data, agc_obj, IFX_POST_PROCESS_IP_COMPONENT_AGC, output_data);
    if (err_id != IFX_SP_ENH_SUCCESS) {
        app_log_print("AGC processing failed! Error code=%x\n", err_id);
        return AGC_FAILURE;
    }
    return AGC_SUCCESS;

}

/*******************************************************************************
* Function Name: agc_deinit
********************************************************************************
* Summary:
*  This function deinitializes AGC
*
*******************************************************************************/
int agc_deinit(void)
{

    if (post_proc_agc_info.memory.scratch_mem_pt != NULL)
    {
        free(post_proc_agc_info.memory.scratch_mem_pt);
        post_proc_agc_info.memory.scratch_mem_pt=NULL;
    }
    if (post_proc_agc_info.memory.persistent_mem_pt !=NULL)
    {
        free(post_proc_agc_info.memory.persistent_mem_pt);
        post_proc_agc_info.memory.persistent_mem_pt=NULL;
    }
    if (agc_output_frame!=NULL)
    {
        free(agc_output_frame);
        agc_output_frame=NULL;
    }
    return AGC_SUCCESS;
}