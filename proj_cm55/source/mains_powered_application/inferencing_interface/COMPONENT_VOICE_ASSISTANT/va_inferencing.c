/******************************************************************************
* File Name : va_inferencing.c
*
* Description :
* Code for DEEPCRAFT Voice Assistant (VA) inference processing
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

#include "va_inferencing.h"
#include "app_logger.h"
#include MTB_WWD_NLU_CONFIG_HEADER(PROJECT_PREFIX)

#define COMMAND_STRING_SIZE                     (250U)


extern volatile bool ptt_flag;
bool ptt_control = false;
/*******************************************************************************
 * Function Name: print_voice_assistant_status
 *******************************************************************************
 * Summary:
 * Prints an error message if wake-word detection result is not successful. 
 * Or a detection message if wake-word is detected.
 *  
 * Parameters:
 *  result: result of the voice-assisstant operation
 *  event: state of the voice-assistant operation
 *  va_data: data detected from the voice-assistant operation
 *
 * Return:
 *  void
 *
 *******************************************************************************/
void print_voice_assistant_status(cy_rslt_t result, va_event_t event, va_data_t *va_data)
{
    char command_text[COMMAND_STRING_SIZE] = {0};

    if ( result != VA_RSLT_SUCCESS )
    {
        app_log_print("Error! voice_assistant_process!! Error code=%d\r\n", result);
    }
    else
    {
        if ( event == VA_EVENT_WW_DETECTED )
        {
            va_intent_to_id(INF_WAKE_WORD);
        }
        else if ( event == VA_EVENT_WW_NOT_DETECTED )
        {
            //app_log_print("Wake-word rejected!\r\n");
        }
        else if ( event == VA_EVENT_CMD_TIMEOUT || event  == VA_EVENT_CMD_SILENCE_TIMEOUT)
        {
             va_intent_to_id(INF_TIMEOUT);
        }
        else if ( event == VA_EVENT_CMD_DETECTED )
        {
            if (CY_RSLT_SUCCESS == voice_assistant_get_command(command_text))
            {
                //va_command_to_id(command_text);
            }

            if (va_data == NULL)
            {
                return;
            }


            app_log_print("Intent name: %s \r\n", MTB_NLU_INTENT_NAME_LIST(PROJECT_PREFIX)[va_data->intent_index] );

#ifdef VA_VARIABLE_ENABLE
            if (va_data->num_var != 0)
            {
                app_log_print("Variable values: ");
                for (int i = 0; i < va_data->num_var; i++)
                {
                    if (va_data->variable[i].unit_idx < 0)
                    {
                        app_log_print("%s ", MTB_NLU_VARIABLE_PHRASE_LIST(PROJECT_PREFIX)[va_data->variable[i].value]);
                    }
                    else
                    {
                        app_log_print("int %d ", va_data->variable[i].value);
                        convert_number_to_intent(va_data->variable[i].value);
                    }
                }
                app_log_print("\n\rVariable units : ");
                for (int i = 0; i < va_data->num_var; i++)
                {
                    if (va_data->variable[i].unit_idx < 0)
                    {
                        app_log_print("---");
                    }
                    else
                    {
                        app_log_print("%s", MTB_NLU_UNIT_PHRASE_LIST(PROJECT_PREFIX)[va_data->variable[i].unit_idx]);
                    }
                }
                app_log_print("\n\r");
            }
            else
            {
                va_intent_to_id((char *)MTB_NLU_INTENT_NAME_LIST(PROJECT_PREFIX)[va_data->intent_index]);

            }
#endif /* VA_VARIABLE_ENABLE */
        }
    }
}

/*******************************************************************************
 * Function Name: run_voice_assistant_process
 *******************************************************************************
 * Summary:
 * Run the voice assistant process and print any information in the terminal.
 *  
 * Parameters:
 *  audio_frame: pointer to the audio data frame
 *
 * Return:
 *  void
 *
 *******************************************************************************/
void run_voice_assistant_process(int16_t *audio_frame)
{
    va_rslt_t va_result = 0;
    va_data_t va_data;
    va_event_t va_event = VA_NO_EVENT;
    /* Process the audio data */
    
    va_ptt(ptt_flag);
    
    va_result = voice_assistant_process(audio_frame, &va_event, &va_data);

    /* Print the status of the voice assistant */
    print_voice_assistant_status(va_result, va_event, &va_data);

}

/*******************************************************************************
* Function Name: va_ptt
********************************************************************************
* Summary:
*    Push to Talk for Voice Assistant
*
*******************************************************************************/

void va_ptt(volatile bool ptt_flag)
{
    
    if (ptt_control==true && ptt_flag == true)
    {
        return;    
    }
    
    if (ptt_flag == true && ptt_control == false)
    {
        voice_assistant_change_state(VA_RUN_CMD);
        va_intent_to_id(INF_WAKE_WORD);
        ptt_control=true;
    }
    if (ptt_flag == false && ptt_control == true)
    {
        ptt_control=false;
    }
}


/*******************************************************************************
* Function Name: va_command_to_id
********************************************************************************
* Summary:
*    Converts command to map id
*
*******************************************************************************/
void va_command_to_id (char *command)
{
    int map_id=0;
    
    app_log_print("Detected %s \r\n",command);

    if (!strcmp(command,INF_WAKE_WORD))
    {
        map_id = WAKEWORD_CMD_ID;
    }
    else if (!strcmp(command,INF_PLAY_MUSIC))
    {
        map_id = PLAY_MUSIC_CMD_ID;
    }
    else if (!strcmp(command,INF_STOP_MUSIC))
    {
        map_id = STOP_MUSIC_CMD_ID;
    }
    else if (!strcmp(command,INF_INCREASE_VOLUME))
    {
        map_id = INCREASE_VOL_CMD_ID;
    }
    else if (!strcmp(command,INF_DECREASE_VOLUME))
    {
        map_id = DECREASE_VOL_CMD_ID;
    }
    else if (!strcmp(command,INF_NEXT_TRACK))
    {
        map_id = NEXT_TRACK_CMD_ID;
    }
    else if (!strcmp(command,INF_PREVIOUS_TRACK))
    {
        map_id = PREVIOUS_TRACK_CMD_ID;
    }
    
    else if (!strcmp(command, INF_TIMEOUT))
    {
        map_id = INFERENCING_TIMEOUT;
    }

    
    mains_powered_post_processing(map_id);

}

/*******************************************************************************
* Function Name: convert_number_to_intent
********************************************************************************
* Summary:
*    Converts number to intent for post processing
*
*******************************************************************************/

void convert_number_to_intent(int num)
{
    if (num == 0)
    {
        va_intent_to_id(INTENT_SET_VOLUME_0);
    }
    else if (num == 1)
    {
        va_intent_to_id(INTENT_SET_VOLUME_1);
    }
    else if (num == 2)
    {
        va_intent_to_id(INTENT_SET_VOLUME_2);
    }
    else if (num == 3)
    {
        va_intent_to_id(INTENT_SET_VOLUME_3);
    }
    else if (num == 4)
    {
        va_intent_to_id(INTENT_SET_VOLUME_4);
    }
    else if (num == 5)
    {
        va_intent_to_id(INTENT_SET_VOLUME_5);
    }
    else 
    {
        va_intent_to_id(INTENT_SET_VOLUME_n);
    }
    
}

/*******************************************************************************
* Function Name: va_intent_to_id
********************************************************************************
* Summary:
*    Converts intent to map id
*
*******************************************************************************/
void va_intent_to_id (char *intent)
{
    int map_id=0;
    
    app_log_print("Detected intent %s \r\n",intent);

    if (!strcmp(intent,INF_WAKE_WORD))
    {
        map_id = WAKEWORD_CMD_ID;
    }
    else if (!strcmp(intent,INTENT_PLAY_MUSIC))
    {
        map_id = PLAY_MUSIC_CMD_ID;
    }
    else if (!strcmp(intent,INTENT_STOP_MUSIC))
    {
        map_id = STOP_MUSIC_CMD_ID;
    }
    else if (!strcmp(intent,INTENT_INCREASE_VOLUME))
    {
        map_id = INCREASE_VOL_CMD_ID;
    }
    else if (!strcmp(intent,INTENT_DECREASE_VOLUME))
    {
        map_id = DECREASE_VOL_CMD_ID;
    }
    else if (!strcmp(intent,INTENT_NEXT_TRACK))
    {
        map_id = NEXT_TRACK_CMD_ID;
    }
    else if (!strcmp(intent,INTENT_PREVIOUS_TRACK))
    {
        map_id = PREVIOUS_TRACK_CMD_ID;
    }
    
    else if (!strcmp(intent,INTENT_GOTO_HOMESCREEN))
    {
        map_id = GOTO_HOMESCREEN_CMD_ID;
    }
    else if (!strcmp(intent,INTENT_PAUSE_MUSIC))
    {
        map_id = PAUSE_MUSIC_CMD_ID;
    }
    
    else if (!strcmp(intent,INTENT_SET_VOLUME_0))
    {
        map_id = VOL_LEVEL_0_CMD_ID;
    }
    
    else if (!strcmp(intent,INTENT_SET_VOLUME_1))
    {
        map_id = VOL_LEVEL_1_CMD_ID;
    }
    
    else if (!strcmp(intent,INTENT_SET_VOLUME_2))
    {
        map_id = VOL_LEVEL_2_CMD_ID;
    }
    
    else if (!strcmp(intent,INTENT_SET_VOLUME_3))
    {
        map_id = VOL_LEVEL_3_CMD_ID;
    }
    
    else if (!strcmp(intent,INTENT_SET_VOLUME_4))
    {
        map_id = VOL_LEVEL_4_CMD_ID;
    }
    
    else if (!strcmp(intent,INTENT_SET_VOLUME_5))
    {
        map_id = VOL_LEVEL_5_CMD_ID;
    }
    
    else if (!strcmp(intent,INTENT_SET_VOLUME_n))
    {
        map_id = INFERENCING_TIMEOUT;
    }
    
 /* Human centric commands*/   
    else if (!strcmp(intent,INTENT_VOLUME_LEVEL_MAX))
    {
        // map_id = VOL_LEVEL_5_CMD_ID;
        map_id = INCREASE_VOL_CMD_ID;
    }
    
    else if (!strcmp(intent,INTENT_VOLUME_LEVEL_MIN))
    {
        // map_id = VOL_LEVEL_1_CMD_ID;
        map_id = DECREASE_VOL_CMD_ID;
    }
    
    else if (!strcmp(intent, INF_TIMEOUT))
    {
        map_id = INFERENCING_TIMEOUT;
    }

    mains_powered_post_processing(map_id);



}