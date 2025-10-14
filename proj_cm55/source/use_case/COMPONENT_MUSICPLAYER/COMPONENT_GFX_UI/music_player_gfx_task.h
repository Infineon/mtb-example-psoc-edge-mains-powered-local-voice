/******************************************************************************
* File Name:   music_player_gfx_task.h
*
* Description: Header file for Music Player Graphics implementation
*
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
