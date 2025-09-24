/******************************************************************************
* File Name : cy_button_press.c
*
* Description :
* Code for controlling user button.
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

#include "button_press.h"
#include "cybsp.h"
#include "FreeRTOS.h"
#include "timers.h"

/*******************************************************************************
* Global Variables
*******************************************************************************/
volatile bool gpio_intr_flag = false;

/*******************************************************************************
* Macros
*******************************************************************************/
#define USER_BUTTON_DELAY                (500)
#define USER_BUTTON_POLL_DELAY           (100)

#define USER_BTN_1_INIT_VAL               (1u)
#define USER_BTN_1_IRQ_MASK               (1u)

/* Non-secure button interrupt priority */
#define USER_BTN_1_ISR_PRIORITY           (5u)
#define PORT_INTR_MASK                    (0x00000001UL << 8U)
#define INTERRUPT_MASKED                  (1U)
#define BTN_DEBOUNCE_INTERVAL_MS          (300u)

/*******************************************************************************
* Global Variables
*******************************************************************************/
static TimerHandle_t btn_debounce_timer;

/*******************************************************************************
* Functions Prototypes
*******************************************************************************/
static void button_interrupt_handler(void);
void (*button_callback)(void);
static void btn_debounce_timer_callback(TimerHandle_t xTimer);
/*******************************************************************************
* Function Name: user_button_init
********************************************************************************
* Summary:
* Initialize user button.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/

void user_button_init(cb_user_action arg)
{
    button_callback = (cb_user_action)arg;
    /* Interrupt config structure */
    cy_stc_sysint_t intrCfg =
    {
        .intrSrc = CYBSP_USER_BTN1_IRQ,
        .intrPriority = USER_BTN_1_ISR_PRIORITY
    };

    button_callback = (cb_user_action)arg;

    /* Clear GPIO and NVIC interrupt before initializing to avoid false
        * triggering.
        */
    Cy_GPIO_ClearInterrupt(CYBSP_USER_BTN1_PORT,CYBSP_USER_BTN1_PIN);
    Cy_GPIO_ClearInterrupt(CYBSP_USER_BTN2_PORT,CYBSP_USER_BTN2_PIN);
    NVIC_ClearPendingIRQ(CYBSP_USER_BTN1_IRQ);
    NVIC_ClearPendingIRQ(CYBSP_USER_BTN2_IRQ);
    NVIC_DisableIRQ(CYBSP_USER_BTN2_IRQ);

    /* Initialize the interrupt and register interrupt callback */
    Cy_SysInt_Init(&intrCfg, &button_interrupt_handler);

    /* Create the FreeRTOS timers for debouncing and long press detection */
    btn_debounce_timer = xTimerCreate("Debounce Timer",
                                        pdMS_TO_TICKS(BTN_DEBOUNCE_INTERVAL_MS),
                                        pdFALSE,
                                        (void *) 0,
                                        btn_debounce_timer_callback);
    CY_ASSERT(btn_debounce_timer != NULL);

    /* Enable the interrupt in the NVIC */
    NVIC_EnableIRQ(intrCfg.intrSrc);
}

/*******************************************************************************
* Function Name: button_interrupt_handler
********************************************************************************
* Summary:
*   GPIO interrupt handler for User Button.
*
* Parameters:
*  void
*
*******************************************************************************/
static void button_interrupt_handler(void)
{
        /* Get interrupt cause */
        uint32_t interrupt_cause = Cy_GPIO_GetInterruptCause0();

        /* Check if the interrupt was from the user button's port */
        if(PORT_INTR_MASK == (interrupt_cause & PORT_INTR_MASK))
        {
            if(INTERRUPT_MASKED == Cy_GPIO_GetInterruptStatusMasked(CYBSP_USER_BTN_PORT,
                    CYBSP_USER_BTN_PIN))
            {
                /* Clear the interrupt */
                Cy_GPIO_ClearInterrupt(CYBSP_USER_BTN_PORT, CYBSP_USER_BTN_PIN);
                NVIC_ClearPendingIRQ(CYBSP_USER_BTN1_IRQ);
            }
    
            if(INTERRUPT_MASKED == Cy_GPIO_GetInterruptStatusMasked(CYBSP_USER_BTN2_PORT,
                    CYBSP_USER_BTN2_PIN))
            {
                /* Clear the interrupt */
                Cy_GPIO_ClearInterrupt(CYBSP_USER_BTN2_PORT, CYBSP_USER_BTN2_PIN);
                NVIC_ClearPendingIRQ(CYBSP_USER_BTN2_IRQ);
            }
        }
    
        /* Disable the GPIO pin interrupts for both buttons. */
        NVIC_DisableIRQ(CYBSP_USER_BTN1_IRQ);
        NVIC_DisableIRQ(CYBSP_USER_BTN2_IRQ);
    
        /* Start the debouncing timer in both cases when the button is pressed as
         * well as released. */
        xTimerStopFromISR(btn_debounce_timer, NULL);
        xTimerResetFromISR(btn_debounce_timer, NULL);
        xTimerStartFromISR(btn_debounce_timer, NULL);
}

/******************************************************************************
* Function Name: btn_debounce_timer_callback
*******************************************************************************
* Summary:
*   Timer callback for User Button debouncing.
*
* Parameters:
*   xTimer : Timer handle
*
* Return:
*   None
*
*******************************************************************************/
static void btn_debounce_timer_callback(TimerHandle_t xTimer)
{
    (void) xTimer;

    if (gpio_intr_flag == false)
    {
        gpio_intr_flag = true;
        button_callback();
    } else
    {
        gpio_intr_flag = false;
    }

    /* Clear any pending user button 1 interrupt before enabling it */
    Cy_GPIO_ClearInterrupt(CYBSP_USER_BTN1_PORT, CYBSP_USER_BTN1_PIN);
    NVIC_ClearPendingIRQ(CYBSP_USER_BTN1_IRQ);
    NVIC_EnableIRQ(CYBSP_USER_BTN1_IRQ);
}

/* [] END OF FILE */
