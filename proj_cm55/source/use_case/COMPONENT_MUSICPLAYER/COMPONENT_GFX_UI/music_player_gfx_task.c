/******************************************************************************
* File Name:   music_player_gfx_task.c
*
* Description: This file contains the Music Player Graphics Task implementation
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
#include "music_player_gfx_task.h"
#include "timers.h"
#include "semphr.h"

#include "lv_port_disp.h"

#include "lvgl.h"
#include "vg_lite.h"
#include "vg_lite_platform.h"
#include "lv_port_disp.h"
#include "lv_port_indev.h"
#include "lv_demos.h"
#include "lv_demo_music_main.h"

#include "music_player_task.h"
#include "inferencing_interface.h"
#include "control_task.h"

#if defined(MTB_DISPLAY_W4P3INCH_RPI)
#include "mtb_disp_dsi_waveshare_4p3.h"
#endif

/******************************************************************************
* Macros
******************************************************************************/
#define GPU_INT_PRIORITY                    (3U)
#define DC_INT_PRIORITY                     (3U)

#define GFX_TASK_NAME                       ("CM55 Gfx Task")
/* stack size in words */
#define GFX_TASK_STACK_SIZE                 (configMINIMAL_STACK_SIZE * 24)

#define GFX_TASK_PRIORITY                   (2U)

#define APP_BUFFER_COUNT                    (2U)
/* 64 KB */
#define DEFAULT_GPU_CMD_BUFFER_SIZE         ((64U) * (1024U))

#define DISP_H                              (480U)

#define GPU_TESSELLATION_BUFFER_SIZE        ((DISP_H) * 128U)

#define VGLITE_HEAP_SIZE                    (((DEFAULT_GPU_CMD_BUFFER_SIZE) * \
                                              (APP_BUFFER_COUNT)) + \
                                             ((GPU_TESSELLATION_BUFFER_SIZE) * \
                                              (APP_BUFFER_COUNT)))

#define GPU_MEM_BASE                        (0x0U)
#define I2C_CONTROLLER_IRQ_PRIORITY         (2UL)
#define VG_PARAMS_POS                       (0UL)
#define LVGL_UPDATE_INTERVAL_MS             (40)

/*******************************************************************************
* Global Variables
********************************************************************************/
CY_SECTION(".cy_gpu_buf") uint8_t contiguous_mem[VGLITE_HEAP_SIZE] = { 0xFF };
volatile void *vglite_heap_base = &contiguous_mem;
TaskHandle_t music_player_gfx_task_handle = NULL;
SemaphoreHandle_t i2c_semaphore;

/* DC IRQ Config */
cy_stc_sysint_t dc_irq_cfg =
{
    .intrSrc      = GFXSS_DC_IRQ,
    .intrPriority = DC_INT_PRIORITY
};

/* GPU IRQ Config */
cy_stc_sysint_t gpu_irq_cfg =
{
    .intrSrc      = GFXSS_GPU_IRQ,
    .intrPriority = GPU_INT_PRIORITY
};

cy_stc_scb_i2c_context_t disp_touch_i2c_controller_context;

cy_stc_sysint_t disp_touch_i2c_controller_irq_cfg =
{
    .intrSrc      = CYBSP_I2C_CONTROLLER_IRQ,
    .intrPriority = I2C_CONTROLLER_IRQ_PRIORITY,
};

/* FreeRTOS queue handle for music player graphics commands */
QueueHandle_t mp_gfx_queue_handle = NULL;

/*******************************************************************************
* Function Prototypes
********************************************************************************/
static void music_player_gfx_task(void *arg);


/*******************************************************************************
* Function Definitions
********************************************************************************/

/*******************************************************************************
* Function Name: create_music_player_gfx_task
********************************************************************************
* Summary:
*  Function that creates the Music Player Graphics task.
*
* Parameters:
*  None
*
* Return:
*  CY_RSLT_SUCCESS upon successful creation of the task, else a non-zero value
*   that indicates the error.
*
*******************************************************************************/
cy_rslt_t create_music_player_gfx_task(void)
{
    BaseType_t status;

    status = xTaskCreate(music_player_gfx_task, GFX_TASK_NAME, GFX_TASK_STACK_SIZE,
                        NULL, GFX_TASK_PRIORITY, &music_player_gfx_task_handle);

    return (status == pdPASS) ? CY_RSLT_SUCCESS : (cy_rslt_t) status;
}


/*******************************************************************************
* Function Name: setup_run_time_stats_timer
********************************************************************************
* Summary:
*  This function configuresTCPWM 0 GRP 0 Counter 0 as the timer source for  
*  FreeRTOS runtime statistics.
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void setup_run_time_stats_timer(void)
{
    /* Initialze TCPWM block with required timer configuration */
    if (CY_TCPWM_SUCCESS != Cy_TCPWM_Counter_Init(CYBSP_TCPWM_0_GRP_0_COUNTER_0_HW, 
        CYBSP_TCPWM_0_GRP_0_COUNTER_0_NUM, 
        &CYBSP_TCPWM_0_GRP_0_COUNTER_0_config))
    {
        CY_ASSERT(0);
    }

    /* Enable the initialized counter */
    Cy_TCPWM_Counter_Enable(CYBSP_TCPWM_0_GRP_0_COUNTER_0_HW, 
                            CYBSP_TCPWM_0_GRP_0_COUNTER_0_NUM);

    /* Start the counter */
    Cy_TCPWM_TriggerStart_Single(CYBSP_TCPWM_0_GRP_0_COUNTER_0_HW, 
                                 CYBSP_TCPWM_0_GRP_0_COUNTER_0_NUM);
}


/*******************************************************************************
* Function Name: get_run_time_counter_value
********************************************************************************
* Summary:
*  Function to fetch run time counter value. This will be used by FreeRTOS for 
*  run time statistics calculation.
*
* Parameters:
*  void
*
* Return:
*  uint32_t: TCPWM 0 GRP 0 Counter 0 value
*
*******************************************************************************/
uint32_t get_run_time_counter_value(void)
{
   return (Cy_TCPWM_Counter_GetCounter(CYBSP_TCPWM_0_GRP_0_COUNTER_0_HW, 
                                       CYBSP_TCPWM_0_GRP_0_COUNTER_0_NUM));
}


/*******************************************************************************
* Function Name: send_mp_gfx_queue_command
********************************************************************************
* Summary:
*  Function to send a command to the music player graphics task via queue.
*
* Parameters:
*  command: The command to send
*  data: The data associated with the command
*
* Return:
*  BaseType_t: pdTRUE if successful, pdFALSE otherwise
*
*******************************************************************************/
BaseType_t send_mp_gfx_queue_command(uint32_t command, uint32_t data)
{
    mp_gfx_queue_data_t queue_data;
    BaseType_t result;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    queue_data.command = command;
    queue_data.data = data;
    
    /* Check if we're in an ISR context */
    if (xPortIsInsideInterrupt())
    {
        result = xQueueSendFromISR(mp_gfx_queue_handle, &queue_data, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
    else
    {
        result = xQueueSend(mp_gfx_queue_handle, &queue_data, 0);
    }
    
    return result;
}


/*******************************************************************************
* Function Name: calculate_idle_percentage
********************************************************************************
* Summary:
*  Function to calculate CPU idle percentage. This function is used by LVGL to  
*  showcase CPU usage.
*
* Parameters:
*  void
*
* Return:
*  uint32_t: CPU idle percentage
*
*******************************************************************************/
uint32_t calculate_idle_percentage(void)
{
    static uint32_t previousIdleTime = 0;
    static TickType_t previousTick = 0;
    uint32_t time_diff = 0;
    uint32_t idle_percent = 0;

    uint32_t currentIdleTime = ulTaskGetIdleRunTimeCounter();
    TickType_t currentTick = portGET_RUN_TIME_COUNTER_VALUE();

    time_diff = currentTick - previousTick;

    if((currentIdleTime >= previousIdleTime) && (currentTick > previousTick))
    {
        idle_percent = ((currentIdleTime - previousIdleTime) * 100)/time_diff;
    }

    previousIdleTime = ulTaskGetIdleRunTimeCounter();
    previousTick = portGET_RUN_TIME_COUNTER_VALUE();

    return idle_percent;
}

/*******************************************************************************
* Function Name: dc_irq_handler
********************************************************************************
* Summary:
*  Display Controller interrupt handler which gets invoked when the DC finishes
*  utilizing the current frame buffer.
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
static void dc_irq_handler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    Cy_GFXSS_Clear_DC_Interrupt(GFXSS, &gfx_context);

    /* Notify the cm55_gfx_task */
    xTaskNotifyFromISR(music_player_gfx_task_handle, 1, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);

    /* Perform a context switch if a higher-priority task was woken */
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/*******************************************************************************
* Function Name: gpu_irq_handler
********************************************************************************
* Summary:
*  GPU interrupt handler which gets invoked when the GPU finishes composing
*  a frame.
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
static void gpu_irq_handler(void)
{
    Cy_GFXSS_Clear_GPU_Interrupt(GFXSS, &gfx_context);
    vg_lite_IRQHandler();
}

/*******************************************************************************
* Function Name: disp_touch_i2c_controller_interrupt
********************************************************************************
* Summary:
*  I2C controller ISR which invokes Cy_SCB_I2C_Interrupt to perform I2C transfer
*  as controller.
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
static void disp_touch_i2c_controller_interrupt(void)
{
    Cy_SCB_I2C_Interrupt(CYBSP_I2C_CONTROLLER_HW, &disp_touch_i2c_controller_context);
}


static void music_player_gfx_task(void *arg)
{
    CY_UNUSED_PARAMETER(arg);

    cy_en_sysint_status_t sysint_status = CY_SYSINT_SUCCESS;
    cy_en_gfx_status_t gfx_status = CY_GFX_SUCCESS;
    vg_lite_error_t vglite_status = VG_LITE_SUCCESS;
    mp_gfx_queue_data_t mp_gfx_queue_data;
    cy_rslt_t status = CY_RSLT_SUCCESS;
    cy_en_scb_i2c_status_t i2c_result = CY_SCB_I2C_SUCCESS;

    /* Create the queue for music player graphics commands */
    mp_gfx_queue_handle = xQueueCreate(MP_GFX_QUEUE_LENGTH, sizeof(mp_gfx_queue_data_t));
    if (mp_gfx_queue_handle == NULL)
    {
        app_log_print("Failed to create music player graphics queue !!\r\n");
        CY_ASSERT(0);
    }

    /* Set frame buffer address to the GFXSS configuration structure */
    GFXSS_config.dc_cfg->gfx_layer_config->buffer_address    = frame_buffer1;
    GFXSS_config.dc_cfg->gfx_layer_config->uv_buffer_address = frame_buffer1;
    //GFXSS_config.mipi_dsi_cfg = &mtb_disp_waveshare_4p3_dsi_config;

    /* Create a FreeRTOS semaphore to handle I2C resource sharing */
    i2c_semaphore = xSemaphoreCreateBinary();
    if (NULL == i2c_semaphore)
    {
        app_log_print("Failed to create I2C semaphore !!\r\n");
        CY_ASSERT(0);
    }
    /* Give the semaphore for the first time */
    xSemaphoreGive(i2c_semaphore);


    /* Take the semaphore while using I2C resource */
    xSemaphoreTake(i2c_semaphore, portMAX_DELAY);

    /* Initialize the I2C in controller mode. */
    i2c_result = Cy_SCB_I2C_Init(CYBSP_I2C_CONTROLLER_HW, &CYBSP_I2C_CONTROLLER_config, &disp_touch_i2c_controller_context);

    if (CY_SCB_I2C_SUCCESS != i2c_result)
    {
        app_log_print("I2C controller initialization failed !!\n");
        CY_ASSERT(0);
    }

    /* Initialize the I2C interrupt */
    sysint_status = Cy_SysInt_Init(&disp_touch_i2c_controller_irq_cfg, &disp_touch_i2c_controller_interrupt);

    if (CY_SYSINT_SUCCESS != sysint_status)
    {
        app_log_print("I2C controller interrupt initialization failed\r\n");
        CY_ASSERT(0);
    }

    /* Enable the I2C interrupts. */
    NVIC_EnableIRQ(disp_touch_i2c_controller_irq_cfg.intrSrc);

    /* Enable the I2C */
    Cy_SCB_I2C_Enable(CYBSP_I2C_CONTROLLER_HW);

    /* Release the semaphore after I2C initialization */
    xSemaphoreGive(i2c_semaphore);

    /* Initialize Graphics subsystem as per the configuration */
    gfx_status = Cy_GFXSS_Init(GFXSS, &GFXSS_config, &gfx_context);

    if (CY_GFX_SUCCESS == gfx_status)
    {
        /* Prevent CPU to go to DeepSleep */
        mtb_hal_syspm_lock_deepsleep();

        /* Initialize GFXSS DC interrupt */
        sysint_status = Cy_SysInt_Init(&dc_irq_cfg, dc_irq_handler);
 
        if (CY_SYSINT_SUCCESS != sysint_status)
        {
            app_log_print("Error in registering DC interrupt: %d\r\n", sysint_status);
            CY_ASSERT(0);
        }

        /* Enable GFX DC interrupt in NVIC. */
        NVIC_EnableIRQ(GFXSS_DC_IRQ);

        /* Initialize GFX GPU interrupt */
        sysint_status = Cy_SysInt_Init(&gpu_irq_cfg, gpu_irq_handler);

        if (CY_SYSINT_SUCCESS != sysint_status)
        {
            app_log_print("Error in registering GPU interrupt: %d\r\n", sysint_status);
            CY_ASSERT(0);
        }
 
        /* Enable GPU interrupt */
        Cy_GFXSS_Enable_GPU_Interrupt(GFXSS);

        /* Enable GFX GPU interrupt in NVIC. */
        NVIC_EnableIRQ(GFXSS_GPU_IRQ);

        xSemaphoreTake(i2c_semaphore, portMAX_DELAY);
        /* Initialize the RPI display */
        
        status =  mtb_disp_waveshare_4p3_init(CYBSP_I2C_CONTROLLER_HW, &disp_touch_i2c_controller_context);

        if (CY_RSLT_SUCCESS != status)
        {
            app_log_print("Display init failed with status = %x\r\n",  status);
            CY_ASSERT(0);
        }
        xSemaphoreGive(i2c_semaphore);

        /* Allocate memory for VGLite from the vglite_heap_base */
        vg_module_parameters_t vg_params;
        vg_params.register_mem_base = (uint32_t)GFXSS_GFXSS_GPU_GCNANO;
        vg_params.gpu_mem_base[VG_PARAMS_POS] = GPU_MEM_BASE;
        vg_params.contiguous_mem_base[VG_PARAMS_POS] = vglite_heap_base;
        vg_params.contiguous_mem_size[VG_PARAMS_POS] = VGLITE_HEAP_SIZE;

        /* Initialize VGlite memory. */
        vg_lite_init_mem(&vg_params);

        /* Initialize the memory and data structures needed for VGLite draw/blit
         * functions
         */
        vglite_status = vg_lite_init(MY_DISP_HOR_RES, MY_DISP_VER_RES);

        if (VG_LITE_SUCCESS == vglite_status)
        {
            lv_init();
            lv_port_disp_init();
            lv_port_indev_init();
            lv_demo_music();
        }
        else
        {
            app_log_print("vg_lite_init failed, status: %d\r\n", vglite_status);
            vg_lite_close();
            CY_ASSERT(0);
        }

    }
    else
    {
        app_log_print("Graphics subsystem init failed, status: %d\r\n", gfx_status);
        CY_ASSERT(0);
    }

    for (;;)
    {
        /* Check if there's a message in the queue */
        if (xQueueReceive(mp_gfx_queue_handle, &mp_gfx_queue_data, 0) == pdTRUE)
        {
            switch(mp_gfx_queue_data.command)
            {
                case OK_INFINEON_CMD_ID:
                {
                    lv_obj_clear_flag(g_spinner, LV_OBJ_FLAG_HIDDEN);
                    lv_obj_add_flag(g_img, LV_OBJ_FLAG_HIDDEN);
                    lv_demo_music_pause();
                    break;
                }
                case PLAY_MUSIC_CMD_ID:
                {
                    lv_obj_clear_flag(g_img, LV_OBJ_FLAG_HIDDEN);
                    lv_obj_add_flag(g_spinner, LV_OBJ_FLAG_HIDDEN);
                    lv_demo_music_resume();
                    break;
                }
                case STOP_MUSIC_CMD_ID:
                {
                    lv_obj_clear_flag(g_img, LV_OBJ_FLAG_HIDDEN);
                    lv_obj_add_flag(g_spinner, LV_OBJ_FLAG_HIDDEN);
                    lv_demo_music_pause();
                    reset_seek_bar();
                    break;
                }

                case NEXT_TRACK_CMD_ID:
                {
                    lv_obj_clear_flag(g_img, LV_OBJ_FLAG_HIDDEN);
                    lv_obj_add_flag(g_spinner, LV_OBJ_FLAG_HIDDEN);
                    lv_demo_music_album_next(true);
                    break;
                }

                case PREVIOUS_TRACK_CMD_ID:
                {
                    lv_obj_clear_flag(g_img, LV_OBJ_FLAG_HIDDEN);
                    lv_obj_add_flag(g_spinner, LV_OBJ_FLAG_HIDDEN);
                    lv_demo_music_album_next(false);
                    break;
                }

                case INCREASE_VOL_CMD_ID:
                {
                    lv_obj_clear_flag(g_img, LV_OBJ_FLAG_HIDDEN);
                    lv_obj_add_flag(g_spinner, LV_OBJ_FLAG_HIDDEN);
#if ENABLE_VOLUME_VOICE_COMMANDS
                    update_volume_level(vol_level_index + 1);
#endif /* ENABLE_VOLUME_VOICE_COMMANDS */
                    if(is_music_player_active() || is_music_player_paused_for_wwd())
                    {
                        lv_demo_music_resume();
                    }
                    break;
                }

                case DECREASE_VOL_CMD_ID:
                {
                    lv_obj_clear_flag(g_img, LV_OBJ_FLAG_HIDDEN);
                    lv_obj_add_flag(g_spinner, LV_OBJ_FLAG_HIDDEN);
#if ENABLE_VOLUME_VOICE_COMMANDS
                    update_volume_level(vol_level_index - 1);
#endif /* ENABLE_VOLUME_VOICE_COMMANDS */
                    if(is_music_player_active() || is_music_player_paused_for_wwd())
                    {
                        lv_demo_music_resume();
                    }
                    break;
                }

                case INFERENCING_TIMEOUT:
                {
                    lv_obj_clear_flag(g_img, LV_OBJ_FLAG_HIDDEN);
                    lv_obj_add_flag(g_spinner, LV_OBJ_FLAG_HIDDEN);
                    if(is_music_player_active() || is_music_player_paused_for_wwd())
                    {
                        lv_demo_music_resume();
                    }
                    break;
                }
                case GOTO_HOMESCREEN_CMD_ID:
                {     
                    lv_obj_clean(lv_scr_act());
                    vg_lite_close();
                    __NVIC_SystemReset();
                    break;
                }
                case GFX_SWITCH_TO_TRACK_ID:
                {
                    if(playing)
                    {
                        lv_demo_music_play(mp_gfx_queue_data.data);
                    }
                    break;
                }

                case PAUSE_MUSIC_CMD_ID:
                {
                    lv_obj_clear_flag(g_img, LV_OBJ_FLAG_HIDDEN);
                    lv_obj_add_flag(g_spinner, LV_OBJ_FLAG_HIDDEN);
                    lv_demo_music_pause();
                    break;
                }

                case VOL_LEVEL_0_CMD_ID:
                case VOL_LEVEL_1_CMD_ID:
                case VOL_LEVEL_2_CMD_ID:
                case VOL_LEVEL_3_CMD_ID:
                case VOL_LEVEL_4_CMD_ID:
                case VOL_LEVEL_5_CMD_ID:
                {
                    lv_obj_clear_flag(g_img, LV_OBJ_FLAG_HIDDEN);
                    lv_obj_add_flag(g_spinner, LV_OBJ_FLAG_HIDDEN);
#if ENABLE_VOLUME_VOICE_COMMANDS
                    update_volume_level((mp_gfx_queue_data.command - VOL_LEVEL_0_CMD_ID));
#endif /* ENABLE_VOLUME_VOICE_COMMANDS */
                    if(is_music_player_active() || is_music_player_paused_for_wwd())
                    {
                        lv_demo_music_resume();
                    }
                    break;
                }

                default:
                {
                    break;
                }
            }
        }


        lv_timer_handler();
        vTaskDelay(pdMS_TO_TICKS(LVGL_UPDATE_INTERVAL_MS));
    }
}


/* [] END OF FILE */
