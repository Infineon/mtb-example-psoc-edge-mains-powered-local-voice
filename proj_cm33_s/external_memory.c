/*******************************************************************************
 * File Name        : external_memory.c
 *
 * Description      : Initializes the Serial Memory Interface (SMIF)
 *
 * This function initializes the SMIF core, resets the connected external memory, 
 * configures the memory device (Quad SPI or Octal), and enables the SMIF 
 * in memory mode. It supports both Quad SPI (QSPI) and Octal modes, including 
 * Hyperbus with DQS for DDR memory.
 * 
 * * Parameters:
*   mode    : The desired external memory mode (QSPI or OSPI).
*
* Return:
*   cy_en_smif_status_t : status of SMIF core
*
 *******************************************************************************
 * Copyright 2023-2025, Cypress Semiconductor Corporation (an Infineon company) or
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
 ******************************************************************************/

#include "external_memory.h"
#include "cy_smif_memslot.h"
#include "cybsp.h"

static cy_stc_smif_context_t SMIFContext;

cy_en_smif_status_t external_memory_init(en_ext_mem_t mode)
{
    /* Initialize SMIF status variable with an error value */
    cy_en_smif_status_t smif_status = CY_SMIF_GENERAL_ERROR;

    do
    {
        /* De-initialize the SMIF core */
        Cy_SMIF_DeInit(CYBSP_SMIF_CORE_0_XSPI_FLASH_hal_config.base);

        /* Reset the external memory connected to the SMIF core */
        Cy_SMIF_Reset_Memory(CYBSP_SMIF_CORE_0_XSPI_FLASH_hal_config.base,smifBlockConfig.memConfig[0]->slaveSelect);

        /* Initialize the SMIF core with the provided configuration */
        smif_status = Cy_SMIF_Init(CYBSP_SMIF_CORE_0_XSPI_FLASH_hal_config.base, CYBSP_SMIF_CORE_0_XSPI_FLASH_hal_config.config, SMIF_INIT_TIMEOUT, &SMIFContext);

        if (CY_SMIF_SUCCESS != smif_status)
        {
            /* Break the loop if initialization fails */
            break;
        }

        /* Set the data select signal for the memory device */
        Cy_SMIF_SetDataSelect(CYBSP_SMIF_CORE_0_XSPI_FLASH_hal_config.base, smifBlockConfig.memConfig[0]->slaveSelect, smifBlockConfig.memConfig[0]->dataSelect);

        /* Enable the SMIF core */
        Cy_SMIF_Enable(CYBSP_SMIF_CORE_0_XSPI_FLASH_hal_config.base, &SMIFContext);

        /* Initialize the external memory device */
        smif_status = Cy_SMIF_MemInit(CYBSP_SMIF_CORE_0_XSPI_FLASH_hal_config.base, &smifBlockConfig, &SMIFContext);

        if (CY_SMIF_SUCCESS != smif_status)
        {
            /* Break the loop if memory initialization fails */
            break;
        }

        if (mode == QSPI)
        {
            bool qe_status = false;

            /* Check if QUAD Mode is already enabled */
            smif_status = Cy_SMIF_MemIsQuadEnabled(CYBSP_SMIF_CORE_0_XSPI_FLASH_hal_config.base, smifBlockConfig.memConfig[0], &qe_status, &SMIFContext);

            if (smif_status != CY_SMIF_SUCCESS)
            {
                /* Break the loop if checking QE status fails */
                break;
            }

            /* If QUAD Mode is not enabled, enable it */
            if (!qe_status)
            {
                smif_status = Cy_SMIF_MemQuadEnable(CYBSP_SMIF_CORE_0_XSPI_FLASH_hal_config.base, smifBlockConfig.memConfig[0], &SMIFContext);

                if (smif_status != CY_SMIF_SUCCESS)
                {
                    /* Break the loop if enabling QE fails */
                    break;
                }
            }
        }
        else
        {
            /* Get the desired data rate for the memory device */
            cy_en_smif_data_rate_t data_rate = smifBlockConfig.memConfig[0]->deviceCfg->readCmd->dataRate;

            /* Enable OCTAL mode for the memory device */
            smif_status = Cy_SMIF_MemOctalEnable(CYBSP_SMIF_CORE_0_XSPI_FLASH_hal_config.base, smifBlockConfig.memConfig[0], data_rate, &SMIFContext);

            if (CY_SMIF_SUCCESS != smif_status)
            {
                /* Break the loop if enabling Octal mode fails */
                break;
            }

            /* If DDR mode is enabled, configure for Hyperbus with DQS */
            if (CY_SMIF_DDR == data_rate)
            {
                Cy_SMIF_Disable(CYBSP_SMIF_CORE_0_XSPI_FLASH_hal_config.base);

                smif_status = Cy_SMIF_SetRxCaptureMode(CYBSP_SMIF_CORE_0_XSPI_FLASH_hal_config.base, CY_SMIF_SEL_XSPI_HYPERBUS_WITH_DQS, smifBlockConfig.memConfig[0]->slaveSelect);

                if (CY_SMIF_SUCCESS != smif_status)
                {
                    /* Break the loop if setting RX capture mode fails */
                    break;
                }
            }
        }
        Cy_SMIF_Enable(CYBSP_SMIF_CORE_0_XSPI_FLASH_hal_config.base, &SMIFContext);
        /* Set the SMIF core to memory mode */
        Cy_SMIF_SetMode(CYBSP_SMIF_CORE_0_XSPI_FLASH_hal_config.base, CY_SMIF_MEMORY);

    } while (false); // The loop will only iterate once

    /* Assert if any of the initialization steps failed */
    if (CY_SMIF_SUCCESS != smif_status)
    {
        CY_ASSERT(0);
    }

    return smif_status;
}