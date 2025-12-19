/******************************************************************************
* File Name : circular_buffer.c
*
* Description :
* Source file of circular buffer which is used for buffering aec reference
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

/*******************************************************************************
* Header Files
*******************************************************************************/
#include "circular_buffer.h"
#include "app_i2s.h"

/*******************************************************************************
* Function Name: circular_buffer_init
********************************************************************************
* Summary:
* Initialize circular buffer
*
*
*******************************************************************************/
void circular_buffer_init(circular_buffer_t* cbuf, uint8_t* buffer, uint32_t size) 
{
    cbuf->buffer = buffer;
    cbuf->size = size;
    cbuf->head = 0;
    cbuf->tail = 0;
    cbuf->full = false;
}

/*******************************************************************************
* Function Name: circular_buffer_is_full
********************************************************************************
* Summary:
* Check if circular buffer is full
*
*
*******************************************************************************/
bool circular_buffer_is_full(const circular_buffer_t* cbuf) 
{
    return cbuf->full;
}

/*******************************************************************************
* Function Name: circular_buffer_is_empty
********************************************************************************
* Summary:
* Check if circular buffer is empty
*
*
*******************************************************************************/
bool circular_buffer_is_empty(const circular_buffer_t* cbuf) 
{
    return (!cbuf->full && (cbuf->head == cbuf->tail));
}

/*******************************************************************************
* Function Name: circular_buffer_enqueue
********************************************************************************
* Summary:
* Push data into circular buffer
*
*
*******************************************************************************/
uint8_t* circular_buffer_enqueue(circular_buffer_t* cbuf, uint8_t* data_ptr) 
{
    uint8_t* frame_ptr = &(cbuf->buffer[cbuf->head]);
    for (uint16_t i = 0; i < AEC_REF_FRAME_SIZE*sizeof(int16_t); i++) 
    {
        cbuf->buffer[cbuf->head] = *(data_ptr);
        data_ptr++;
        cbuf->head = (cbuf->head + 1) % cbuf->size;
        /* This allows overwriting into circular buffer */
        cbuf->full = 0; 
    }
    data_ptr = frame_ptr;
    return data_ptr;
}

/*******************************************************************************
* Function Name: circular_buffer_dequeue
********************************************************************************
* Summary:
* Pop data into circular buffer
*
*
*******************************************************************************/
uint8_t circular_buffer_dequeue(circular_buffer_t* cbuf) 
{
    uint8_t data = cbuf->buffer[cbuf->tail];
    cbuf->full = false;
    cbuf->tail = (cbuf->tail + 1) % cbuf->size;
    return data;
}
/* [] END OF FILE */