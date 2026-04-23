/******************************************************************************
* File Name : profiler.c
*
* Description :
* Code for MIPS profiler
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
* Function Name: Cy_Reset_Cycles
********************************************************************************
* Summary:
* Reset cycle count.
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
* Function Name: Cy_Get_Cycles
********************************************************************************
* Summary:
* Get cycles.
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
* Initializes profiler.
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
