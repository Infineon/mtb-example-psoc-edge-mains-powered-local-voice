/******************************************************************************
* File Name : audio_usb_send_utils.h
*
* Description :
* Header file for Buffer management code for sending audio data over USB.

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
