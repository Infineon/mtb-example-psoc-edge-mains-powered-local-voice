/******************************************************************************
* File Name : profiler.c
*
* Description :
* Code for MIPS profiler
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
#include "cy_pdl.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "cyabs_rtos.h"
#include "cy_device.h"
#include "profiler.h"
#include "cy_profiler.h"
#include "app_logger.h"

/*******************************************************************************
* Macros
*******************************************************************************/

#define PROFILE_USE_DWT_COUNT 1
/*******************************************************************************
* Function Prototypes
*******************************************************************************/
static void profiler_init(void);

/*******************************************************************************
* Global Variables
*******************************************************************************/

/* LED timer tick counter */
uint32_t timer_tick_cnt = 0;

/* Timer clock HZ */
uint32_t timer_hz;

/* CM4 system clock vs. Peripheral clock ratio */
int CM4_clk_ratio;

static uint32_t stop_time_val;

#define RESET_CYCLE_CNT (DWT->CYCCNT=0)
#define GET_CYCLE_CNT (DWT->CYCCNT)

/*******************************************************************************
* Function Name: DWTCyCNTInit
********************************************************************************
* Summary:
* Cycle counter initialization.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/

void DWTCyCNTInit(void)
{
    /* Disable TRC */
    CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk; // ~0x01000000;
    /* Enable TRC */
    CoreDebug->DEMCR |=  CoreDebug_DEMCR_TRCENA_Msk; // 0x01000000;

    /* Disable clock cycle counter */
    DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk; //~0x00000001;
    /* Enable  clock cycle counter */
    DWT->CTRL |=  DWT_CTRL_CYCCNTENA_Msk; //0x00000001;
}

/*******************************************************************************
* Function Name: DWTCyCNTInit
********************************************************************************
* Summary:
* Cycle counter initialization.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/

uint32_t Cy_Reset_Cycles(void)
{
    /* Call DWTCyCNTInit before first call */
    return RESET_CYCLE_CNT;
}

/*******************************************************************************
* Function Name: DWTCyCNTInit
********************************************************************************
* Summary:
* Cycle counter initialization.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/

uint32_t Cy_Get_Cycles(void)
{
    /* Call DWTCyCNTInit before first call */
    return GET_CYCLE_CNT;
}

/*******************************************************************************
* Function Name: profiler_init
********************************************************************************
* Summary:
* This function creates and configures a Timer object. The timer ticks
* continuously and produces a periodic interrupt on every terminal count
* event. The period is defined by the 'period' and 'compare_value' of the
* timer configuration structure 'led_blink_timer_cfg'. Without any changes,
* this application is designed to produce an interrupt every 1 second.
*
* Parameters:
*  none
*
* Return:
*  None
*
*******************************************************************************/


 static void profiler_init(void)
 {
    DWTCyCNTInit();

 }

/*******************************************************************************
* Function Name: Cy_SysLib_ProcessingFault
********************************************************************************
* Summary:
* Prints System fault.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/

void Cy_SysLib_ProcessingFault(void)
{

    app_log_print("\r\nCORE FAULT!!\r\n");
    app_log_print("SCB->CFSR  = 0x%08"PRIx32"\r\n", (uint32_t) cy_faultFrame.cfsr.cfsrReg);
    app_log_print("SCB->HFSR  = 0x%08"PRIx32"\r\n", (uint32_t) cy_faultFrame.hfsr.hfsrReg);
    app_log_print("SCB->SHCSR = 0x%08"PRIx32"\r\n", (uint32_t) cy_faultFrame.shcsr.shcsrReg);

    /* If MemManage fault valid bit is set to 1, print MemManage fault address */
    if ((cy_faultFrame.cfsr.cfsrReg & SCB_CFSR_MMARVALID_Msk)
            == SCB_CFSR_MMARVALID_Msk)
    {
        app_log_print("MemManage Fault! Fault address = 0x%08"PRIx32"\r\n", SCB->MMFAR);
    }

    /* If Bus Fault valid bit is set to 1, print BusFault Address */
    if ((cy_faultFrame.cfsr.cfsrReg & SCB_CFSR_BFARVALID_Msk)
            == SCB_CFSR_BFARVALID_Msk)
    {
        app_log_print("Bus Fault! \r\nFault address = 0x%08"PRIx32"\r\n", SCB->BFAR);
    }

    /* Print Fault Frame */
    app_log_print("r0  = 0x%08"PRIx32"\r\n", cy_faultFrame.r0);
    app_log_print("r1  = 0x%08"PRIx32"\r\n", cy_faultFrame.r1);
    app_log_print("r2  = 0x%08"PRIx32"\r\n", cy_faultFrame.r2);
    app_log_print("r3  = 0x%08"PRIx32"\r\n", cy_faultFrame.r3);
    app_log_print("r12 = 0x%08"PRIx32"\r\n", cy_faultFrame.r12);
    app_log_print("lr  = 0x%08"PRIx32"\r\n", cy_faultFrame.lr);
    app_log_print("pc  = 0x%08"PRIx32"\r\n", cy_faultFrame.pc);
    app_log_print("psr = 0x%08"PRIx32"\r\n", cy_faultFrame.psr);

    while (1);
}

/*******************************************************************************
* Function Name: start_time
********************************************************************************
* Summary:
* Starts timer for profiling.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/

static void start_time(void)
{

    Cy_Reset_Cycles();
}

/*******************************************************************************
* Function Name: stop_time
********************************************************************************
* Summary:
* Saves time during stop profiling.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/

static void stop_time(void)
{

    stop_time_val = Cy_Get_Cycles();
  
}

/*******************************************************************************
* Function Name: get_time
********************************************************************************
* Summary:
* Returns profiled time.
*
* Parameters:
*  None
*
* Return:
*  Profiled time.
*
*******************************************************************************/

static uint32_t get_time(void)
{

    return stop_time_val;
}


#if !defined(COMPONENT_CM33) && !defined(__ARMCC_VERSION)

/*******************************************************************************
* Function Name: display_mallinfo
********************************************************************************
* Summary:
* Displays memory info.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/

void display_mallinfo(void)
{
   struct mallinfo mi;

   mi = mallinfo();

   app_log_print("Total non-mmapped bytes (arena):       %u\n", mi.arena);
   app_log_print("# of free chunks (ordblks):            %u\n", mi.ordblks);
   app_log_print("# of free fastbin blocks (smblks):     %u\n", mi.smblks);
   app_log_print("# of mapped regions (hblks):           %u\n", mi.hblks);
   app_log_print("Bytes in mapped regions (hblkhd):      %u\n", mi.hblkhd);
   app_log_print("Max. total allocated space (usmblks):  %u\n", mi.usmblks);
   app_log_print("Free bytes held in fastbins (fsmblks): %u\n", mi.fsmblks);
   app_log_print("Total allocated space (uordblks):      %u\n", mi.uordblks);
   app_log_print("Total free space (fordblks):           %u\n", mi.fordblks);
   app_log_print("Topmost releasable block (keepcost):   %u\n", mi.keepcost);
}
#endif /* #if !defined(COMPONENT_CM33) && !defined(__ARMCC_VERSION) */

/*******************************************************************************
* Function Name: cy_profiler_init
********************************************************************************
* Summary:
* Initialize profiler.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/

void cy_profiler_init(void)
{
    profiler_init();
}

/*******************************************************************************
* Function Name: cy_profiler_start
********************************************************************************
* Summary:
* Start profiler.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/

void cy_profiler_start(void)
{
    start_time();
}

/*******************************************************************************
* Function Name: cy_profiler_stop
********************************************************************************
* Summary:
* Stop profiler.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/

void cy_profiler_stop(void)
{
    stop_time();
}

/*******************************************************************************
* Function Name: cy_profiler_get_cycles
********************************************************************************
* Summary:
* Get profiling cycles.
*
* Parameters:
*  None
*
* Return:
*  Profiling cycles.
*
*******************************************************************************/

uint32_t cy_profiler_get_cycles(void)
{
    return(get_time());
}




/* [] END OF FILE */
