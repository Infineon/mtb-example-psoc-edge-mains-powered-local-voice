/******************************************************************************
* File Name : cy_button_press.c
*
* Description :
* Code for controlling user button.
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
