/******************************************************************************
* File Name : audio_usb_send_utils.h
*
* Description :
* Header file for Buffer management code for sending audio data over USB.

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

#ifndef AUDIO_USB_SEND_UTILS_H
#define AUDIO_USB_SEND_UTILS_H


#ifdef __cplusplus
extern "C"
{
#endif /* __cplusplus */
#include "rtos.h"
#include "cyabs_rtos.h"
#include "cyabs_rtos_internal.h"
/*******************************************************************************
* Macros
*******************************************************************************/
#define USB_CHANNEL_1                       (1)
#define USB_CHANNEL_2                       (2)
#define USB_CHANNEL_3                       (3)
#define USB_CHANNEL_4                       (4)

#define USB_QUEUE_FAILURE                   (-1)

#define USB_MIC_BUFFER_COUNT                (2)

/*******************************************************************************
* Functions Prototypes
*******************************************************************************/
void usb_send_out_dbg_init_channels();
cy_rslt_t usb_send_out_dbg_put(unsigned int channel_no, short *mono_data_10ms);
void usb_send_out_dbg_callback(uint8_t** data, uint16_t* length);

cy_rslt_t usb_queue_push(QueueHandle_t queue, void* item_ptr, bool isr);
cy_rslt_t usb_queue_pop(QueueHandle_t queue, void* item_ptr, bool isr);

cy_rslt_t aec_push(short* item_ptr);
cy_rslt_t aec_pop(short* item_ptr,int bulk_delay);
void usb_aec_flush();

cy_rslt_t bdm_push(short* item_ptr);
cy_rslt_t bdm_pop(short* item_ptr);
void usb_mic_flush();

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* AUDIO_USB_SEND_UTILS_H */

/* [] END OF FILE */
