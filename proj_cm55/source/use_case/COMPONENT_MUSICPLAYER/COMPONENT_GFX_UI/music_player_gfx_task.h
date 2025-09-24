/******************************************************************************
* File Name:   music_player_gfx_task.h
*
* Description: Header file for Music Player Graphics implementation
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

#ifndef MUSIC_PLAYER_GFX_TASK_H_
#define MUSIC_PLAYER_GFX_TASK_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/*******************************************************************************
* Header Files
*******************************************************************************/
#include "cy_pdl.h"
#include "cybsp.h"
#include "lvgl.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "app_logger.h"


/*******************************************************************************
 * Macros
 ********************************************************************************/
#define MP_GFX_QUEUE_LENGTH                (10u)

/*******************************************************************************
* Data structure
********************************************************************************/
/* Data-type for Music Player GFX task's queue data */
typedef struct
{
    uint32_t command;
    uint32_t data;
} mp_gfx_queue_data_t;

/*******************************************************************************
* Variables
*******************************************************************************/
extern cy_stc_scb_i2c_context_t disp_touch_i2c_controller_context;
extern SemaphoreHandle_t i2c_semaphore;
extern QueueHandle_t mp_gfx_queue_handle;
extern bool playing;
extern lv_obj_t * play_obj;
extern lv_obj_t * g_spinner;
extern lv_obj_t * g_img;

/*******************************************************************************
* Function Prototypes
********************************************************************************/
cy_rslt_t create_music_player_gfx_task(void);
BaseType_t send_mp_gfx_queue_command(uint32_t command, uint32_t data);

#ifdef __cplusplus
} /* extern C */
#endif /* __cplusplus */

#endif /* MUSIC_PLAYER_GFX_TASK_H_ */

/* [] END OF FILE */
