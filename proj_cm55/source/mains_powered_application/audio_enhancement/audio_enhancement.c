/******************************************************************************
* File Name : audio_enhancement.c
*
* Description :
* Code for DEEPCRAFT audio enhancement (AE)
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

/*******************************************************************************
* Header Files
*******************************************************************************/
#include "audio_enhancement.h"

/*******************************************************************************
* Macros
*******************************************************************************/

#define AE_APP_TEMP_MEMORY                              (2)
#define AE_MAX_NUM_CHANNELS                             (2)
#define AE_ALGO_SCRATCH_MEMORY                          (40000)
#define AE_ALGO_PERSISTENT_MEMORY                       (160000)


/*******************************************************************************
* Global Variables
*******************************************************************************/
uint8_t ae_scratch_memory[AE_ALGO_SCRATCH_MEMORY] __attribute__((section(".dtcm_data"), aligned(4)));
uint8_t ae_persistent_memory[AE_ALGO_PERSISTENT_MEMORY] __attribute__((section(".dtcm_data"), aligned(4)));
uint8_t ae_temp_mem[AE_APP_TEMP_MEMORY] = {0};
uint8_t ae_output_buffer[AE_FRAME_BUFFER_MEMORY] __attribute__((section(".dtcm_data"), aligned(4)));

cy_afe_t ae_handle = NULL;
void    *ae_dsns_mem = NULL;
void    *ae_dses_mem = NULL;
ae_buffer_info_t ae_output_buffer_info = {0};

/****************************************************************************
* Global variables
*****************************************************************************/
extern volatile int8_t ae_toggle_flag;

/*******************************************************************************
* Function Name: ae_output_callback
********************************************************************************
* Summary:
* Output callback to process the output buffer from AFE middleware.
*
* Parameters:
*  handle - AFE handle.
*  output_buffer_info - Output buffer from MW.
*  user_arg - User argument (unused).
* 
* Return:
*  None
*
*******************************************************************************/
static cy_rslt_t ae_output_callback(cy_afe_t handle, cy_afe_buffer_info_t *output_buffer_info, void *user_arg)
{
    ae_output_buffer_info.output_buf = (int16_t *) output_buffer_info->output_buf;
    ae_output_buffer_info.input_buf = (int16_t *) output_buffer_info->input_buf;
    ae_output_buffer_info.input_aec_ref_buf = (int16_t *) output_buffer_info->input_aec_ref_buf;
#ifdef CY_AFE_ENABLE_TUNING_FEATURE
    ae_output_buffer_info.dbg_output1 = (int16_t *) output_buffer_info->dbg_output1;
    ae_output_buffer_info.dbg_output2 = (int16_t *) output_buffer_info->dbg_output2;
    ae_output_buffer_info.dbg_output3 = (int16_t *) output_buffer_info->dbg_output3;
    ae_output_buffer_info.dbg_output4 = (int16_t *) output_buffer_info->dbg_output4;
#endif
    audio_enhancement_process_output(&ae_output_buffer_info);
    return CY_RSLT_SUCCESS;
}


/*******************************************************************************
* Function Name: ae_get_output_buffer_callback
********************************************************************************
* Summary:
* Output callback to get the output buffer for AFE middleware.
*
* Parameters:
*  context - context (unused)
*  output_buffer - Get a free buffer.
*  user_arg - User argument (unused).
* 
* Return:
*  CY_RSLT_SUCCESS.
*
*******************************************************************************/
static cy_rslt_t ae_get_output_buffer_callback(cy_afe_t context, uint32_t **output_buffer, void *user_args)
{
    *output_buffer = (uint32_t*) ae_output_buffer;

    return CY_RSLT_SUCCESS;
}
/*******************************************************************************
* Function Name: ae_alloc_memory
********************************************************************************
* Summary:
* Callback from middleware to allocate memory for AFE.
*
* Parameters:
*  None
*
* Return:
*  Result of memory allocation.
*
*******************************************************************************/
cy_rslt_t ae_alloc_memory(cy_afe_mem_id_t mem_id, uint32_t size, void **buffer)
{
    cy_rslt_t  ret_val = CY_RSLT_SUCCESS;
    int ae_mem_id = (int)mem_id;
    if(NULL == buffer)
    {
        return ret_val;
    }
    *buffer = NULL;
    app_ae_log("app afe alloc size %ld \r\n", (long)size);
    /* If size equal ZERO, no allocation required */
    if (size == 0)
    {
        app_ae_log("No allocation as size is zero \r\n");
        *buffer = &ae_temp_mem[0];
        return ret_val;
    }

    /* Allocate memory based on the memory ID */
    switch(ae_mem_id)
    {
        case CY_AFE_MEM_ID_ALGORITHM_NS_MEMORY:
        {
            app_ae_log("DSNS Memory requires %ld bytes \r\n", (long)size);
            ae_dsns_mem = calloc(size+15,1);
            if (ae_dsns_mem == NULL)
            {
                app_ae_log("DSNS memory allocation failed \r\n");
                ret_val = -1;
            }
            else
            {
                *buffer = (void *) (((uintptr_t)ae_dsns_mem+15) & ~ (uintptr_t)0x0F);
            }
            break;
        }
        
        case CY_AFE_MEM_ID_ALGORITHM_ES_MEMORY:
        {
            app_ae_log("DSES Memory requires %ld bytes \r\n", (long)size);
            ae_dses_mem = calloc(size+15,1);
            if (ae_dses_mem == NULL)
            {
                app_ae_log("DSES memory allocation failed \r\n");
                ret_val = -1;
            }
            else
            {
                *buffer = (void *) (((uintptr_t)ae_dses_mem+15) & ~ (uintptr_t)0x0F);
            }
            break;
        }
        case CY_AFE_MEM_ID_ALGORITHM_PERSISTENT_MEMORY:
        {
            app_ae_log("Persistent Memory requires %ld bytes \r\n", (long)size);

            if(size > AE_ALGO_PERSISTENT_MEMORY)
            {
                app_ae_log("Defaulting to heap allocation for persistent memory \r\n");
                *buffer = (void *)calloc(size,1);
                if (*buffer == NULL)
                {
                    app_ae_log("Persistent memory allocation failed \r\n");
                    ret_val = -1;
                }
            }
            else
            {
                *buffer = &ae_persistent_memory[0];
                memset(*buffer, 0 , size);
            }
            break;
        }
        case CY_AFE_MEM_ID_ALGORITHM_SCRATCH_MEMORY:
        {
            app_ae_log("Scratch Memory requires %ld bytes \r\n", (long)size);
            if(size > AE_ALGO_SCRATCH_MEMORY)
            {
                app_ae_log("Defaulting to heap allocation for scratch memory \r\n");
                *buffer = (void *)calloc(size,1);
                if (*buffer == NULL)
                {
                    ret_val = -1;
                }
            }
            else
            {
                *buffer = &ae_scratch_memory[0];
                memset(*buffer, 0 , size);
            }
            break;
        }
        default:
        {
            *buffer = (void *)calloc(size,1);
            if (*buffer == NULL)
            {
                app_ae_log("AFE memory allocation failed \r\n");
                ret_val = -1;
            }
            break;
        }
    }

    return ret_val;
}

/*******************************************************************************
* Function Name: ae_free_memory
********************************************************************************
* Summary:
* Callback from middle-ware to free memory for AFE.
*
* Parameters:
*  None
*
* Return:
*  Result of freeing memory.
*
*******************************************************************************/
cy_rslt_t ae_free_memory(cy_afe_mem_id_t mem_id, void *buffer)
{
    int ae_mem_id = (int)mem_id;
    if (buffer != NULL)
    {
          free(buffer);
          buffer = NULL;
    }
    switch(ae_mem_id)
    {
        case CY_AFE_MEM_ID_ALGORITHM_ES_MEMORY:
        {
            if (ae_dses_mem != NULL) 
            {
                free(ae_dses_mem);
                ae_dses_mem=NULL;
            }
            break;
        }
        case CY_AFE_MEM_ID_ALGORITHM_NS_MEMORY:
        {
            if (ae_dsns_mem !=NULL)
            {
                free(ae_dsns_mem);
                ae_dsns_mem=NULL;                
            }
            break;
        }
        default:
            break;
    }

    return CY_RSLT_SUCCESS;
}

#ifdef CY_AFE_ENABLE_TUNING_FEATURE
/*******************************************************************************
* Function Name: ae_tuner_notify_callback
********************************************************************************
* Summary:
*  Tuner notification callback from AFE middleware.
*
* Parameters:
*  handle - AFE handle
*  config_setting - Configuration settings.
*  user_arg - User argument.

* Return:
*  CY_RSLT_SUCCESS
*
*******************************************************************************/
static cy_rslt_t ae_tuner_notify_callback(cy_afe_t handle, 
                                          cy_afe_config_setting_t *config_setting, 
                                          void *user_arg)
{
    (void) user_arg;

    return audio_enhancement_tuner_notify(handle,config_setting);

}

/*******************************************************************************
* Function Name: ae_tuner_read_callback
********************************************************************************
* Summary:
*  Callback for AFE read operation.
*
* Parameters:
*  handle - AFE handle
*  request_buffer - Buffer with read data.
*  user_arg - user argument.
*
* Return:
*  CY_RSLT_SUCCESS
*
*******************************************************************************/
static cy_rslt_t ae_tuner_read_callback(cy_afe_t handle, 
                                        cy_afe_tuner_buffer_t *request_buffer, 
                                        void *user_arg)
{
    (void) user_arg;

    audio_enhancement_tuner_read(request_buffer);

    return CY_RSLT_SUCCESS;
}

/*******************************************************************************
* Function Name: ae_tuner_write_callback
********************************************************************************
* Summary:
*  Callback for AFE write operation.
*
* Parameters:
*  handle - AFE handle
*  response_buffer - buffer with write data.
*  user_arg - user argument.
*
* Return:
*  CY_RSLT_SUCCESS
*
*******************************************************************************/
static cy_rslt_t ae_tuner_write_callback(cy_afe_t handle, 
                                         cy_afe_tuner_buffer_t *response_buffer, 
                                         void *user_arg)
{
    audio_enhancement_tuner_write(response_buffer);

    return CY_RSLT_SUCCESS;
}

#endif

/*******************************************************************************
 * Function Name: audio_enhancement_init
 *******************************************************************************
 * Summary:
 * Initializes the audio enhacement module. Internally, it instantiates the
 * audio-front-end middleware and create an internal task to process the 
 * audio data.
 *
 * Parameters:
 *  void
 *
 * Return:
 *  Returns AE_RSLT_SUCCESS if successful, otherwise returns an error code.
 *
 *******************************************************************************/
ae_rslt_t audio_enhancement_init(uint8_t num_channels)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;
    cy_afe_config_t afe_config = {0};

    if (num_channels > AE_MAX_NUM_CHANNELS)
    {
        return AE_RSLT_INVALID_ARGUMENT;
    }

    afe_config.filter_settings = AFE_FILTER_SETTINGS;
    afe_config.mw_settings = NULL;
    afe_config.afe_get_buffer_callback = ae_get_output_buffer_callback;
    afe_config.afe_output_callback = ae_output_callback;
    afe_config.user_arg_callbacks = NULL;

#if AFE_MW_SETTINGS_SIZE
    afe_config.mw_settings = AFE_MW_SETTINGS;
    afe_config.mw_settings_length = AFE_MW_SETTINGS_SIZE;
#else
    afe_config.mw_settings = NULL;
    afe_config.mw_settings_length = 0;
#endif /* AFE_MW_SETTINGS_SIZE */


#ifdef CY_AFE_ENABLE_TUNING_FEATURE
    cy_afe_tuner_callbacks_t tuner_cb;

    /* Tuner callbacks */
    tuner_cb.notify_settings_callback = ae_tuner_notify_callback;
    tuner_cb.read_request_callback = ae_tuner_read_callback;
    tuner_cb.write_response_callback = ae_tuner_write_callback;
    afe_config.tuner_cb = tuner_cb;

    afe_config.poll_interval_ms = 100; // Invoke next read after 5sec if no data received on previous read call
#endif /* CY_AFE_ENABLE_TUNING_FEATURE */

    afe_config.alloc_memory = ae_alloc_memory;
    afe_config.free_memory = ae_free_memory;
    
    /* Create AFE instance (AFE Handle) */
    result = cy_afe_create(&afe_config, &ae_handle);
    if(CY_RSLT_SUCCESS != result)
    {
        return AE_RSLT_FAIL;
    }

    return AE_RSLT_SUCCESS;
}

/*******************************************************************************
 * Function Name: audio_enhancement_feed_input
 *******************************************************************************
 * Summary:
 * Feeds the input audio data to the audio enhancement module.
 *
 * Parameters:
 *  input_buffer: pointer to the input audio data buffer.
 *  aec_buffer: pointer to the AEC reference buffer. If not used, set to NULL.
 *
 * Return:
 *  Returns AE_RSLT_SUCCESS if successful, otherwise returns an error code.
 *
 *******************************************************************************/
ae_rslt_t audio_enhancement_feed_input(int16_t *input_buffer, int16_t *aec_buffer)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    result = cy_afe_feed(ae_handle, (int16_t*)input_buffer, (int16_t*)aec_buffer);

    if (CY_RSLT_AFE_FUNCTIONALITY_RESTRICTED == result)
    {
        return AE_RSLT_LICENSE_ERROR;

    }
    else if (CY_RSLT_SUCCESS != result)
    {
        return AE_RSLT_FAIL;
    }

    return AE_RSLT_SUCCESS;
}

/*******************************************************************************
 * Function Name: audio_enhancement_process_output
 *******************************************************************************
 * Summary:
 * Weak implementation to process the audio enhancement output.
 *
 * Parameters:
 *  output_buffer: pointer to the output audio data buffer.
 *
 * Return:
 *  void
 *
 *******************************************************************************/
__attribute__((weak)) void audio_enhancement_process_output(ae_buffer_info_t *output_buffer)
{
    return;
}

/* [] END OF FILE */