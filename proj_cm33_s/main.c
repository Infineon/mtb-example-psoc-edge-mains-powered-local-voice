/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for CM33 Secure Project.
*
* Related Document: See README.md
*
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

#include "cy_pdl.h"
#include "cybsp.h"
#include "external_memory.h"

/*****************************************************************************
* Macros
******************************************************************************/
#define CM33_NS_APP_BOOT_ADDR      (CYMEM_CM33_0_m33_nvm_START + CYBSP_MCUBOOT_HEADER_SIZE) 
/*****************************************************************************
* Function Name: main
******************************************************************************
* This is the main function for Cortex M33 CPU secure application
* NOTE: CM33 secure project assumes that certain memory and peripheral regions
* will be accessed from non-secure environment by the CM33 NS /CM55 code,
* For such regions MPC and PPC configurations are applied by cybsp_init to make 
* it non-secure. Any access to these regions from the secure side is recommended 
* to be done before the MPC/PPC configuration is applied.Once a memory or 
* peripheral region is marked as non secure it cannot be accessed from the secure 
* side using secure aliased address but may be accessed using non secure aliased 
* address

* NOTE: In this code example we skip the MPC and PPC initializations in
* cybsp_init using following Makefile defines:
* DEFINES+=CYBSP_SKIP_MPC_INIT
* DEFINES+=CYBSP_SKIP_PPC_INIT
* This allows initialization of SMIF clock and peripheral from secure project
*****************************************************************************/
int main(void)
{
    uint32_t ns_stack;
    cy_cmse_funcptr NonSecure_ResetHandler;
    cy_rslt_t result;
    cy_en_smif_status_t status;

    /* After wakeup from hibernate mode the IOs are in frozen state */
    /* Unfreeze the IOs if the reset reason was hiberate wakeup */
    if(CY_SYSLIB_RESET_HIB_WAKEUP ==
    (Cy_SysLib_GetResetReason() & CY_SYSLIB_RESET_HIB_WAKEUP))
    {
        Cy_SysPm_IoUnfreeze();
    }

    /* Set up internal routing, pins, and clock-to-peripheral connections */
    result = cybsp_init();

    /* Board initialization failed. Stop program execution */
    if (CY_RSLT_SUCCESS != result)
    {
        /* Disable all interrupts. */
        __disable_irq();

        CY_ASSERT(0);
        
        /* Infinite loop */
        while(true);
    }

    /* Enable global interrupts */
    __enable_irq();

    /* 
    * Initialize the clock for the APP_MMIO_TCM (512K) peripheral group.
    * This sets up the necessary clock and peripheral routing to ensure 
    * the APP_MMIO_TCM can be correctly accessed and utilized.
    */
    Cy_SysClk_PeriGroupSlaveInit(
        CY_MMIO_CM55_TCM_512K_PERI_NR, 
        CY_MMIO_CM55_TCM_512K_GROUP_NR, 
        CY_MMIO_CM55_TCM_512K_SLAVE_NR, 
        CY_MMIO_CM55_TCM_512K_CLK_HF_NR
    );

    /* 
    * Initialize the clock for the SMIF0 peripheral group.
    * This sets up the necessary clock and peripheral routing to ensure 
    * the SMIF0 can be correctly accessed and utilized.
    */
    Cy_SysClk_PeriGroupSlaveInit(
        CY_MMIO_SMIF0_PERI_NR,
        CY_MMIO_SMIF0_GROUP_NR,
        CY_MMIO_SMIF0_SLAVE_NR,
        CY_MMIO_SMIF0_CLK_HF_NR
    );

    /* Initialize SMIF in QSPI mode */
    status = external_memory_init(OSPI);
    if(CY_SMIF_SUCCESS != status)
    {
        /* Disable all interrupts. */
        __disable_irq();

        CY_ASSERT(0);
        
        /* Infinite loop */
        while(true);
    }

    /* Initialize MPC and PPC before executing non-secure application */

    /* Memory protection initialization */
    result = Cy_MPC_Init();
    if (CY_RSLT_SUCCESS != result)
    {
        CY_ASSERT(0);

        /* Infinite loop */
        while(true);
    }

    /* Peripheral protection initialization (PPC0) */
    result = Cy_PPC0_Init();
    if (CY_RSLT_SUCCESS != result)
    {
        CY_ASSERT(0);

        /* Infinite loop */
        while(true);
    }

    /* Peripheral protection initialization (PPC1) */
    result = Cy_PPC1_Init();
    if (CY_RSLT_SUCCESS != result)
    {
        CY_ASSERT(0);

        /* Infinite loop */
        while(true);
    }

    ns_stack = (uint32_t)(*((uint32_t*)CM33_NS_APP_BOOT_ADDR));
    __TZ_set_MSP_NS(ns_stack);
    
    NonSecure_ResetHandler = (cy_cmse_funcptr)(*((uint32_t*)(CM33_NS_APP_BOOT_ADDR + 4)));

    /* Start non-secure application */
    NonSecure_ResetHandler();

    for (;;)
    {
    }
}

/* [] END OF FILE */