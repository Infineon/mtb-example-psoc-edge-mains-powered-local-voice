/******************************************************************************
* File Name : audio_usb_send_utils.c
*
* Description :
* Buffer management code for sending audio data from PSOC Edge to PC via USB.

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


/*******************************************************************************
* Header Files
*******************************************************************************/

#include "audio_receive_task.h"
#include "audio_send_task.h"
#include "cy_pdl.h"
#include "cybsp.h"
#include "cycfg.h"
#include "rtos.h"
#include "cyabs_rtos.h"
#include "app_logger.h"
#include "audio_usb_send_utils.h"


/*******************************************************************************
* Macros
*******************************************************************************/

#define MAX_USB_CH1_DATA_Q              (320)
#define MAX_USB_CH2_DATA_Q              (320)
#define MAX_USB_CH3_DATA_Q              (320)
#define MAX_USB_CH4_DATA_Q              (320)


#define USB_QUEUE_ELEMENTS_CH1          (10)
#define USB_QUEUE_ELEMENTS_CH2          (10)
#define USB_QUEUE_ELEMENTS_CH3          (10)
#define USB_QUEUE_ELEMENTS_CH4          (10)


#define USB_MONO_AUDIO_SIZE_BYTES       (320)
#define USB_MIC_IN_Q_LEN                (10)
#define USB_MIC_IN_Q_SIZE               (640)

#define USB_QUAD_1MS_DATA               (128)
/*******************************************************************************
* Functions Prototypes
*******************************************************************************/

extern int is_audio_usb_send_out_data_from_device_started(void);
extern bool is_in_isr();
/*******************************************************************************
* Global Variables
*******************************************************************************/

uint16_t audio_usb_out_buffer[USB_MONO_AUDIO_SIZE_BYTES] = {0};

QueueHandle_t usb_ch1_queue;
QueueHandle_t usb_ch2_queue;
QueueHandle_t usb_ch3_queue;
QueueHandle_t usb_ch4_queue;
QueueHandle_t usb_aec_ref_queue;
QueueHandle_t usb_mic_queue;

short channel_1[USB_MONO_AUDIO_SIZE_BYTES/2]= {0};
short channel_2[USB_MONO_AUDIO_SIZE_BYTES/2]= {0};
short channel_3[USB_MONO_AUDIO_SIZE_BYTES/2]= {0};
short channel_4[USB_MONO_AUDIO_SIZE_BYTES/2]= {0};

short *ch1 = NULL;
short *ch2 = NULL;
short *ch3 = NULL;
short *ch4 = NULL;

void* bufferch1[USB_MONO_AUDIO_SIZE_BYTES] = {0};

void* bufferch2[USB_MONO_AUDIO_SIZE_BYTES] = {0};
void* bufferch3[USB_MONO_AUDIO_SIZE_BYTES] = {0};
void* bufferch4[USB_MONO_AUDIO_SIZE_BYTES] = {0};

/*******************************************************************************
* Function Name: usb_queue_push
********************************************************************************
* Summary:
*   Wrapper API for pushing data to USB queues.
*
*******************************************************************************/

cy_rslt_t usb_queue_push(QueueHandle_t queue, void* item_ptr, bool isr)
{
    cy_rslt_t ret_val = CY_RSLT_SUCCESS;

    if (isr == 0)
    {
        if (pdTRUE != xQueueSendToBack(queue, item_ptr, 0))
        {
            return USB_QUEUE_FAILURE;
        }
    }
    else
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        BaseType_t ret;
        ret = xQueueSendToBackFromISR(queue, item_ptr, &xHigherPriorityTaskWoken);
        if (ret == pdTRUE)
        {
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
        else
        {
            return USB_QUEUE_FAILURE;
        }

    }
    return ret_val;

}

/*******************************************************************************
* Function Name: usb_queue_pop
********************************************************************************
* Summary:
*   Wrapper API for popping data from USB queues.
*
*******************************************************************************/

cy_rslt_t usb_queue_pop(QueueHandle_t queue, void* item_ptr, bool isr)
{
    cy_rslt_t ret_val = CY_RSLT_SUCCESS;
    if (isr == 0)
    {
        if (pdTRUE != xQueueReceive(queue, item_ptr, 0))
        {
            return USB_QUEUE_FAILURE;
        }
    }
    else
    {
        BaseType_t ret;
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        ret = xQueueReceiveFromISR(queue, item_ptr, &xHigherPriorityTaskWoken);
        if (ret == pdTRUE)
        {
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
        else
        {
            return USB_QUEUE_FAILURE;
        }
    }
    return ret_val;
}



/*******************************************************************************
* Function Name: usb_send_out_dbg_get
********************************************************************************
* Summary:
*   Get the data for 4 channel audio from RTOS queues.
*
*******************************************************************************/

static void usb_send_out_dbg_get(unsigned int channel_no,void *pbuffer)
{
    cy_rslt_t ret_val = CY_RSLT_SUCCESS;

    if (1 == channel_no)
    {
        ret_val = usb_queue_pop(usb_ch1_queue, pbuffer, is_in_isr());
    }

    if (2 == channel_no)
    {
        ret_val = usb_queue_pop(usb_ch2_queue, pbuffer, is_in_isr());
    }
    if (3 == channel_no)
    {
        ret_val = usb_queue_pop(usb_ch3_queue, pbuffer, is_in_isr());
    }
    if (4 == channel_no)
    {
        ret_val = usb_queue_pop(usb_ch4_queue, pbuffer, is_in_isr());
    }

    if ((CY_RSLT_SUCCESS != ret_val))
    {

        if (1 == channel_no)
        {
            memset(pbuffer, 0, USB_MONO_AUDIO_SIZE_BYTES);
        }

        if (2 == channel_no)
        {
            memset(pbuffer, 0, USB_MONO_AUDIO_SIZE_BYTES);
        }
        if (3 == channel_no)
        {
            memset(pbuffer, 0, USB_MONO_AUDIO_SIZE_BYTES);
        }
        if (4 == channel_no)
        {
            memset(pbuffer, 0, USB_MONO_AUDIO_SIZE_BYTES);
        }

    }
}

/*******************************************************************************
* Function Name: usb_send_out_for_2_channel_worth_1ms
********************************************************************************
* Summary:
*   Create 2 channel data worth 1 ms from buffers.
*
*******************************************************************************/

static void usb_send_out_for_2_channel_worth_1ms(short *data_to_send)
{
    static int usb_send_counter = 0;
    if ((usb_send_counter % 10) == 0)
    {
        usb_send_counter=0;

        usb_send_out_dbg_get(1, bufferch1);
        ch1 = (short *)bufferch1;
        usb_send_out_dbg_get(2, bufferch2);
        ch2 = (short *)bufferch2;

        usb_send_out_dbg_get(3, bufferch3);
        ch3 = (short *)bufferch3;

        usb_send_out_dbg_get(4, bufferch4);
        ch4 = (short *)bufferch4;
    }

    usb_send_counter++;

    for (int i = 0; i < 16; i++)
    {
        *data_to_send++ = *ch1++;
        *data_to_send++ = *ch2++;
        *data_to_send++ = *ch3++;
        *data_to_send++ = *ch4++;
    }
}

/*******************************************************************************
* Function Name: usb_send_out_dbg_callback
********************************************************************************
* Summary:
*   USB audio callback to send audio data back to PC.
*
*******************************************************************************/

void usb_send_out_dbg_callback(uint8_t **data, uint16_t *length)
{

    usb_send_out_for_2_channel_worth_1ms((short *)audio_usb_out_buffer);
    *data = (uint8_t*)audio_usb_out_buffer;
    *length = USB_QUAD_1MS_DATA;

}

/*******************************************************************************
* Function Name: usb_send_out_dbg_put
********************************************************************************
* Summary:
*   Store audio data in RTOS queues.
*
*******************************************************************************/

cy_rslt_t usb_send_out_dbg_put(unsigned int channel_no, short *mono_data_10ms)
{
    cy_rslt_t ret_val  = CY_RSLT_SUCCESS;
    size_t num_waiting = 0;
    int index = 0;

    if(false == is_audio_usb_send_out_data_from_device_started())
    {
        num_waiting = uxQueueMessagesWaiting(usb_ch1_queue);
        for (index=0; index < num_waiting; index++)
        {
            usb_send_out_dbg_get(1, bufferch1);
        }

        num_waiting = uxQueueMessagesWaiting(usb_ch2_queue);
        for (index=0; index < num_waiting; index++)
        {
            usb_send_out_dbg_get(2, bufferch2);
        }
        num_waiting = uxQueueMessagesWaiting(usb_ch3_queue);
        for (index=0; index < num_waiting; index++)
        {
            usb_send_out_dbg_get(3, bufferch3);
        }
        num_waiting = uxQueueMessagesWaiting(usb_ch4_queue);
        for (index=0; index < num_waiting; index++)
        {
            usb_send_out_dbg_get(4, bufferch4);
        }

        return ret_val;
    }

    if(1 == channel_no)
    {
        ret_val = usb_queue_push(usb_ch1_queue, mono_data_10ms,is_in_isr());
    }

    if(2 == channel_no)
    {
        ret_val = usb_queue_push(usb_ch2_queue, mono_data_10ms,is_in_isr());
    }
    if(3 == channel_no)
    {
        ret_val = usb_queue_push(usb_ch3_queue, mono_data_10ms,is_in_isr());
    }
    if(4 == channel_no)
    {
        ret_val = usb_queue_push(usb_ch4_queue, mono_data_10ms,is_in_isr());
    }

    return ret_val;
}

/*******************************************************************************
* Function Name: aec_push
********************************************************************************
* Summary:
*   Push data to aec queue.
*
*******************************************************************************/
cy_rslt_t aec_push(short* item_ptr)
{
    cy_rslt_t ret_val  = CY_RSLT_SUCCESS;
    ret_val = usb_queue_push(usb_aec_ref_queue, item_ptr, is_in_isr());

    return ret_val;

}

/*******************************************************************************
* Function Name: aec_pop
********************************************************************************
* Summary:
*   Pop data from aec queue.
*
*******************************************************************************/

cy_rslt_t aec_pop(short* item_ptr, int bulk_delay)
{
    cy_rslt_t ret_val  = CY_RSLT_SUCCESS;
    int queue_frames = 0;
    queue_frames=uxQueueMessagesWaiting(usb_aec_ref_queue);
    if (queue_frames >= bulk_delay)
    {
        ret_val = usb_queue_pop(usb_aec_ref_queue, item_ptr, is_in_isr());
    }
    else
    {
        ret_val=USB_QUEUE_FAILURE;
    }
    return ret_val;
}

/*******************************************************************************
* Function Name: usb_aec_flush
********************************************************************************
* Summary:
*   Reset USB aec queue.
*
*******************************************************************************/
void usb_aec_flush()
{
    xQueueReset(usb_aec_ref_queue);
}

/*******************************************************************************
* Function Name: bdm_push
********************************************************************************
* Summary:
*   Push data to USB mic queue for bulk delay measurement
*
*******************************************************************************/
cy_rslt_t bdm_push(short* item_ptr)
{
   cy_rslt_t ret_val  = CY_RSLT_SUCCESS;
   ret_val = usb_queue_push(usb_mic_queue, item_ptr, is_in_isr());
   return ret_val;

}

/*******************************************************************************
* Function Name: bdm_pop
********************************************************************************
* Summary:
*   Push data to USB mic queue for bulk delay measurement
*
*******************************************************************************/
cy_rslt_t bdm_pop(short* item_ptr)
{
   cy_rslt_t ret_val  = CY_RSLT_SUCCESS;
   ret_val = usb_queue_pop(usb_mic_queue, item_ptr, is_in_isr());

   return ret_val;
}

/*******************************************************************************
* Function Name: usb_mic_flush
********************************************************************************
* Summary:
*   Pop data from USB mic queue.
*
*******************************************************************************/
void usb_mic_flush()
{
    xQueueReset(usb_mic_queue);
}

/*******************************************************************************
* Function Name: usb_send_out_dbg_init_channels
********************************************************************************
* Summary:
*   Initialize RTOS queues for USB data.
*
*******************************************************************************/

void usb_send_out_dbg_init_channels()
{

    usb_ch1_queue = xQueueCreate(USB_QUEUE_ELEMENTS_CH1, MAX_USB_CH1_DATA_Q);
    if (usb_ch1_queue == NULL)
    {
         app_log_print("Init queue for channel 1 failed \r\n");
    }


    usb_ch2_queue = xQueueCreate(USB_QUEUE_ELEMENTS_CH2, MAX_USB_CH2_DATA_Q);
    if (usb_ch2_queue == NULL)
    {
         app_log_print("Init queue for channel 2 failed \r\n");
    }

    usb_ch3_queue = xQueueCreate(USB_QUEUE_ELEMENTS_CH3, MAX_USB_CH3_DATA_Q);
    if (usb_ch3_queue == NULL)
    {
         app_log_print("Init queue for channel 3 failed \r\n");
    }

    usb_ch4_queue = xQueueCreate(USB_QUEUE_ELEMENTS_CH4, MAX_USB_CH4_DATA_Q);
    if (usb_ch4_queue == NULL)
    {
         app_log_print("Init queue for channel 4 failed \r\n");
    }


    usb_mic_queue = xQueueCreate(USB_MIC_IN_Q_LEN, USB_MIC_IN_Q_SIZE);
    if (usb_mic_queue == NULL)
    {
         app_log_print("Init queue for mic failed \r\n");
    }


    usb_aec_ref_queue = xQueueCreate(USB_MIC_IN_Q_LEN, USB_MIC_IN_Q_SIZE);
    if (usb_aec_ref_queue == NULL)
    {
         app_log_print("Init queue for aec failed \r\n");
    }

}

/* [] END OF FILE */
