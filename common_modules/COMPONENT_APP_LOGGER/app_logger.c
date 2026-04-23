/******************************************************************************
* File Name : app_logger.c
*
* Description :
* Application logger for PSOC Edge devices.
*
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


#include "app_logger.h"

#include "cycfg.h"
#include "cy_pdl.h"


#if defined (__GNUC__) && !defined(__ARMCC_VERSION)
#include <malloc.h>
#endif /* #if defined (__GNUC__) && !defined(__ARMCC_VERSION) */

/*******************************************************************************
* Macros
*******************************************************************************/
#define TO_KB(size_bytes)  ((float)(size_bytes)/1024)
/* ARM compiler also defines __GNUC__ */


/*******************************************************************************
* Global Variables
*******************************************************************************/
extern uint32_t SystemCoreClock;

/*******************************************************************************
* Function Name: app_print_mem_information
********************************************************************************
* Summary:
* Prints memory information.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/

void app_print_mem_information(void)
{
#if defined (__GNUC__) && !defined(__ARMCC_VERSION) &&!defined(__llvm__)
    extern uint8_t __HeapBase;  /* Symbol exported by the linker. */
    extern uint8_t __HeapLimit; /* Symbol exported by the linker. */
    extern uint8_t __StackTop;
    extern uint8_t __StackLimit;

    struct mallinfo mall_info = mallinfo();

    uint8_t* heap_base = (uint8_t *)&__HeapBase;
    uint8_t* heap_limit = (uint8_t *)&__HeapLimit;
    uint32_t heap_size = (uint32_t)(heap_limit - heap_base);
    app_log_print("********** Heap Usage **********");
    app_log_print("Heap Range    -[Addr:%p->%p] [Sz:%ldbytes/%.2f KB]",heap_base,heap_limit, heap_size,TO_KB(heap_size));
    app_log_print("Heap utilized -[%u bytes/%.2f KB,   Percent: %.2f%%]",
            mall_info.arena, TO_KB(mall_info.arena), ((float) mall_info.arena * 100u)/heap_size);
    app_log_print("Heap InUse now-[%u bytes/%.2f KB,   Percent: %.2f%%]",
            mall_info.uordblks, TO_KB(mall_info.uordblks), ((float) mall_info.uordblks * 100u)/heap_size);


    uint8_t* stack_base = (uint8_t *)&__StackTop;
    uint8_t* stack_limit = (uint8_t *)&__StackLimit;
    uint32_t stack_size = (uint32_t)(stack_limit - stack_base);
    app_log_print("********** Stack **********");
    app_log_print("Stack    -[Addr:%p->%p] [Sz:%lu bytes/%.2f KB]",stack_base,stack_limit, stack_size,TO_KB(stack_size));

#else
    app_log_print("MEM-Info Print not enabled");
#endif
}

/*******************************************************************************
* Function Name: app_print_sys_information
********************************************************************************
* Summary:
* Print system information.
*
* Parameters:
*  core_number - Core number.
* Return:
*  None
*
*******************************************************************************/

int app_print_sys_information(unsigned int core_number)
{
    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    app_log_print("\x1b[2J\x1b[;H");

    app_log_print("CORE %d APP STARTED, SysClk: %lu, Pwrmode:0x%lx",(int)core_number,(long)SystemCoreClock,
            CY_CFG_PWR_SYS_IDLE_MODE);
    app_print_mem_information();
    return 0;
}

/*******************************************************************************
* Function Name: app_core1_boot_log
********************************************************************************
* Summary:
* Core 1 log initialization.
*
* Parameters:
*  None
* Return:
*  None
*
*******************************************************************************/

int app_core1_boot_log(void)
{
    app_print_sys_information(1);
    return 0;
}

/*******************************************************************************
* Function Name: app_core2_boot_log
********************************************************************************
* Summary:
* Core 2 log initialization.
*
* Parameters:
*  None
* Return:
*  None
*
*******************************************************************************/

int app_core2_boot_log(void)
{
    app_print_sys_information(2);
    return 0;
}


/* [] END OF FILE */
