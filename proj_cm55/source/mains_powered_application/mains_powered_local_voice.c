/******************************************************************************
* File Name : mains_powered_local_voice.c
*
* Description :
* Mains Powered Local Voice application.
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

#include "app_logger.h"
#include "cycfg_pins.h"
#include "cybsp_types.h"
#include "inferencing_task.h"
#include "audio_enhancement_interface.h"
#include "mains_powered_local_voice.h"
#include "pdm_mic_interface.h"
#include "usb_audio_interface.h"
#include "audio_usb_send_utils.h"
#include "control_task.h"

#ifdef PROFILER_ENABLE
#if AFE_APP_PROFILE || INFERENCING_PROFILE
#include "cy_afe_profiler.h"
#include "cy_profiler.h"
#endif /* AFE_APP_PROFILE || INFERENCING_PROFILE */
#endif /* PROFILER_ENABLE */
#ifdef AUDIO_OUT
#include "music_player_task.h"
#endif /* AUDIO_OUT */
#ifdef ENABLE_GFX_UI
#include "music_player_gfx_task.h"
#endif /*ENABLE_GFX_UI*/
/*******************************************************************************
 * Macros
 ******************************************************************************/

#define MIC_STARTUP_DELAY_MS                                (4200u)

/*******************************************************************************
* Function Name: mains_powered_local_voice
********************************************************************************
* Summary:
* Initialize local voice application for mains powered use-case.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/

void mains_powered_local_voice()
{
    app_core2_boot_log();
#if AFE_APP_PROFILE
    cy_profiler_init();
#endif
#if INFERENCING_PROFILE
    cy_profiler_init();
#endif

    mains_powered_ae_init(AFE_INPUT_NUMBER_CHANNELS);
    
    create_mains_powered_control_task();

    mains_powered_inference_engine_init();

#ifdef AUDIO_OUT    
    create_music_player_task();
#endif /* AUDIO_OUT */    

#ifdef ENABLE_GFX_UI
    create_music_player_gfx_task();
#endif
  
    /* Enable USB interface*/
    app_log_print("Initializing USB interface \r\n");
    usb_audio_interface_init();
    usb_send_out_dbg_init_channels();
    
#if AFE_INPUT_SOURCE==AFE_INPUT_SOURCE_MIC
    app_push_to_talk_cm55_init(app_user_action_cb);
#endif /* AFE_INPUT_SOURCE_MIC */

    app_log_print("\x1b[2J\x1b[;H");

    app_log_print("****************** \
    Mains Powered Local Voice Code Example \
    ****************** \r\n\n");
#ifdef ENABLE_GFX_UI
    app_log_print("Music Player Application with graphics \r\n");
#else
        app_log_print("Music Player Application without graphics \r\n");
#endif /*ENABLE_GFX_UI*/    
    app_log_print("Wake word: Okay Infineon\r\n"
           "Commands:\r\n"
           "    Play music\r\n"
           "    Next track\r\n"
           "    Previous track\r\n"
           "    End music\r\n"
           "    Pause music \r\n"
           "    Raise volume\r\n"
           "    Lower volume\r\n"
           "    Set volume to level <0-5> \r\n"
           
           "Additional intents for the commands: [Optional word] {one of required words} \t --> Matching commands\r\n"
           "[can you] {play/start} [the] {music/track}  \t--> Play music \r\n"
           "[can you] {end} [the] {music/track}  \t--> Stop music \r\n"
           "[can you] {pause} [the] {music/track}  \t--> Pause music \r\n"
           "[can you] [goto/switch to] {next} {track} \t--> Next track\r\n"
           "[can you] [goto/switch to] {previous} {track} \t--> Previous track\r\n"
           "[can you] {raise} {volume} \t--> Raise volume\r\n"
           "[can you] {lower} {volume} \t--> Lower volume\r\n" 
           "[can you] {set volume to level 0-5} \t--> Set volume to level <0-5>\r\n"
           "{I can't/cannot hear you} \t  --> Raise volume\r\n"
           "{Turn up the volume please} \t  --> Raise volume\r\n"
           "{Turn down the volume please} \t  --> Lower volume\r\n"
           "{Music is too loud} \t  --> Lower volume\r\n"
           );

#if !defined(VA_INFERENCING)
    app_log_print("Important: Inferencing Code unavailable - Enable it for CE to work with all functionality. \r\n");
#else
    app_log_print("Waiting for Wake Word \r\n");
    
#endif /* VA_INFERENCING */

#if AFE_INPUT_SOURCE==AFE_INPUT_SOURCE_MIC
    /* PDM mic initialization. Initialize if PDM mic is choosen as the input-mode.
     * PDM mic data arrives via ISR.
    */
    cy_rslt_t result;
    result = pdm_mic_interface_init();
    if(CY_RSLT_SUCCESS != result)
    {
        app_log_print("PDM initialization failed - Reset the board \r\n");
        CY_ASSERT(0);
    }
#endif /* AFE_INPUT_SOURCE */

}

/*******************************************************************************
* Function Name: app_push_to_talk_cm55_init
********************************************************************************
* Summary:
* Initialize user button and pass the callback.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/

void app_push_to_talk_cm55_init(cb_user_action user_action_cb)
{
    user_interaction_init(USER_INTERACTION_BUTTON, user_action_cb);
}

/*******************************************************************************
* Function Name: app_user_action_cb
********************************************************************************
* Summary:
* Callback for user button - Push to Talk - Goes to ASR detection state.
*
* Parameters:
*  action - User button action.
*
* Return:
*  None
*
*******************************************************************************/

void app_user_action_cb(void)
{
    app_log_print("Push-To-Talk : User button pressed - Waiting for ASR command \r\n");
    ptt_flag = true;
}
/* [] END OF FILE */
