/******************************************************************************
* File Name : main.c
*
* Description :
* main function for CM55 application.
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
#include "mtb_hal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"

#include "FreeRTOS.h"
#include "task.h"
#include "cyabs_rtos.h"
#include "cyabs_rtos_impl.h"
#include "timers.h"

#ifdef COMPONENT_MUSICPLAYER
#include "music_player_task.h"
#ifdef ENABLE_GFX_UI
#include "music_player_gfx_task.h"
#include "lv_demo_music_main.h"
#endif /* ENABLE_GFX_UI */
#endif /* COMPONENT_MUSICPLAYER */

#include "mains_powered_local_voice.h"


/*******************************************************************************
 * Macros
 ******************************************************************************/
/* Enabling or disabling a MCWDT requires a wait time of upto 2 CLK_LF cycles  
 * to come into effect. This wait time value will depend on the actual CLK_LF  
 * frequency set by the BSP.
 */
#define LPTIMER_1_WAIT_TIME_USEC            (62U)

/* Define the LPTimer interrupt priority number. '1' implies highest priority. 
 */
#define APP_LPTIMER_INTERRUPT_PRIORITY      (1U)

/*******************************************************************************
* Function Prototypes
*******************************************************************************/
void app_retarget_io_init(void);
static void lptimer_interrupt_handler(void);
static void setup_tickless_idle_timer(void);
static void handle_error(void);

uint32_t get_clk_measurements(uint32_t clk)
{
    /* Start the clock measurement using the IHO as reference*/
    (void)Cy_SysClk_StartClkMeasurementCounters((cy_en_meas_clks_t)clk,     /* Counter 1 clock */
                                                0x3FFUL,                    /* Counter 1 init value = 1024 */
                                                CY_SYSCLK_MEAS_CLK_IHO);    /* Counter 2 clock        = IHO  */
    /* Wait for counter 1 to reach 0 */
    while (!Cy_SysClk_ClkMeasurementCountersDone()){};

    /* Measure clock 1 with the IMO clock cycles (counter 2) */
    return Cy_SysClk_ClkMeasurementCountersGetFreq(false, CY_SYSCLK_IHO_FREQ);
}
/*******************************************************************************
* Global Variables
*******************************************************************************/
/* Debug UART context */
cy_stc_scb_uart_context_t  DEBUG_UART_context;
/* Debug UART HAL object  */
mtb_hal_uart_t             DEBUG_UART_hal_obj;

/* LPTimer HAL object */
static mtb_hal_lptimer_t lptimer_obj;

/*****************************************************************************
* Function Name: main
******************************************************************************
* Summary:
* This is the main function for CM55 application. 
* 
* Parameters:
*  void
*
* Return:
*  int
*
*****************************************************************************/

int main(void)
{
    cy_rslt_t result       = CY_RSLT_SUCCESS;

    /* Initialize the device and board peripherals */
    result = cybsp_init();

    /* Board init failed. Stop program execution */
    if (CY_RSLT_SUCCESS != result)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    /* Setup the LPTimer instance for CM55*/
    setup_tickless_idle_timer();

        /* Initialize retarget-io */
    app_retarget_io_init();

    /* Init high performance/CM55 core local voice application */
    mains_powered_local_voice();

    /* Start the RTOS Scheduler */
    vTaskStartScheduler();

    return 0;
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


/*******************************************************************************
* Function Name: lptimer_interrupt_handler
********************************************************************************
* Summary:
* Interrupt handler function for LPTimer instance. 
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
static void lptimer_interrupt_handler(void)
{
    mtb_hal_lptimer_process_interrupt(&lptimer_obj);
}

/*******************************************************************************
* Function Name: setup_tickless_idle_timer
********************************************************************************
* Summary:
* 1. This function first configures and initializes an interrupt for LPTimer.
* 2. Then it initializes the LPTimer HAL object to be used in the RTOS 
*    tickless idle mode implementation to allow the device enter deep sleep 
*    when idle task runs. LPTIMER_1 instance is configured for CM55 CPU.
* 3. It then passes the LPTimer object to abstraction RTOS library that 
*    implements tickless idle mode
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
static void setup_tickless_idle_timer(void)
{
    /* Interrupt configuration structure for LPTimer */
    cy_stc_sysint_t lptimer_intr_cfg =
    {
        .intrSrc = CYBSP_CM55_LPTIMER_1_IRQ,
        .intrPriority = APP_LPTIMER_INTERRUPT_PRIORITY
    };

    /* Initialize the LPTimer interrupt and specify the interrupt handler. */
    cy_en_sysint_status_t interrupt_init_status = 
                                    Cy_SysInt_Init(&lptimer_intr_cfg, 
                                                    lptimer_interrupt_handler);
    
    /* LPTimer interrupt initialization failed. Stop program execution. */
    if(CY_SYSINT_SUCCESS != interrupt_init_status)
    {
        handle_error();
    }

    /* Enable NVIC interrupt. */
    NVIC_EnableIRQ(lptimer_intr_cfg.intrSrc);

    /* Initialize the MCWDT block */
    cy_en_mcwdt_status_t mcwdt_init_status = 
                                    Cy_MCWDT_Init(CYBSP_CM55_LPTIMER_1_HW, 
                                                &CYBSP_CM55_LPTIMER_1_config);

    /* MCWDT initialization failed. Stop program execution. */
    if(CY_MCWDT_SUCCESS != mcwdt_init_status)
    {
        handle_error();
    }
  
    /* Enable MCWDT instance */
    Cy_MCWDT_Enable(CYBSP_CM55_LPTIMER_1_HW,
                    CY_MCWDT_CTR_Msk, 
                    LPTIMER_1_WAIT_TIME_USEC);

    /* Setup LPTimer using the HAL object and desired configuration as defined
     * in the device configurator. */
    cy_rslt_t result = mtb_hal_lptimer_setup(&lptimer_obj, 
                                            &CYBSP_CM55_LPTIMER_1_hal_config);
    
    /* LPTimer setup failed. Stop program execution. */
    if(CY_RSLT_SUCCESS != result)
    {
        handle_error();
    }

    /* Pass the LPTimer object to abstraction RTOS library that implements 
     * tickless idle mode 
     */
    cyabs_rtos_set_lptimer(&lptimer_obj);
}

/*******************************************************************************
* Function Name: handle_error
********************************************************************************
* Summary:
* User defined error handling function
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
static void handle_error(void)
{
    /* Disable all interrupts. */
    __disable_irq();

    CY_ASSERT(0);
}
/* [] END OF FILE */
