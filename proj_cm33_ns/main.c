/******************************************************************************
* File Name : main.c
*
* Description :
*  main function for CM33 Non Secure application for Audio&Voice pipeline.
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

#include "cybsp.h"
#include "cy_retarget_io.h"
//#include "app_logger.h"

/*******************************************************************************
* Macros
*******************************************************************************/
/* The timeout value in microsecond used to wait for core to be booted */
#define CM55_BOOT_WAIT_TIME_USEC          (10U)

/* App boot address for CM55 project */
#define CM55_APP_BOOT_ADDR                  (CYMEM_CM33_0_m55_nvm_START + \
                                            CYBSP_MCUBOOT_HEADER_SIZE)

/*******************************************************************************
* Function Prototypes
*******************************************************************************/
void app_retarget_io_init(void);

/*******************************************************************************
* Global Variables
*******************************************************************************/
/* Debug UART context */
cy_stc_scb_uart_context_t  DEBUG_UART_context;
/* Debug UART HAL object  */
mtb_hal_uart_t             DEBUG_UART_hal_obj;

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function of the CM33 non-secure application.
*
*
* Parameters:
*  none
*
* Return:
*  int
*
*******************************************************************************/

int main(void)
{
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init();

    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    /* Enable CM55. CY_CORTEX_M55_APPL_ADDR must be updated if CM55 memory layout is changed. */
    Cy_SysEnableCM55(MXCM55, CM55_APP_BOOT_ADDR, CM55_BOOT_WAIT_TIME_USEC);
                         
    /* For mains powered configuration, put CM33 in deep sleep. */
    for(;;)
    {
        // Cy_SysPm_CpuEnterDeepSleep(CY_SYSPM_WAIT_FOR_INTERRUPT);
    }

    return CY_RSLT_SUCCESS; 

}


/*******************************************************************************
* Function Name: app_retarget_io_init
********************************************************************************
* Summary:
* This function initializes Debug UART
*
* Parameters:
*  none
*
* Return :
*  none
*
*******************************************************************************/
void app_retarget_io_init()
{
    cy_rslt_t result;

    result = (cy_rslt_t)Cy_SCB_UART_Init(CYBSP_DEBUG_UART_HW, &CYBSP_DEBUG_UART_config, &DEBUG_UART_context);
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    Cy_SCB_UART_Enable(CYBSP_DEBUG_UART_HW);
    result = mtb_hal_uart_setup(&DEBUG_UART_hal_obj, &CYBSP_DEBUG_UART_hal_config, &DEBUG_UART_context, NULL);
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    result = cy_retarget_io_init(&DEBUG_UART_hal_obj);
    
}

/* [] END OF FILE */
