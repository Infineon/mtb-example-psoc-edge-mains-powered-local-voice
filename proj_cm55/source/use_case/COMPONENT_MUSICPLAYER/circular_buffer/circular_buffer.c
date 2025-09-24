/******************************************************************************
* File Name : circular_buffer.c
*
* Description :
* Source file of circular buffer which is used for buffering aec reference
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