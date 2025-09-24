/******************************************************************************
* File Name : app_logger.h
*
* Description :
* Application logger header for PSOC Edge devices.
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
#ifndef __APP_LOGGER_H__
#define __APP_LOGGER_H__

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */
#include "cy_retarget_io.h"

#define APP_LOG_PRINT_EN                    (1)

#if APP_LOG_PRINT_EN
#define app_log_print(format,...)           printf(format "",##__VA_ARGS__);
#else 
#define app_log_print(format,...)
#endif /* APP_LOG_PRINT_EN */

/****************************************************************************
* Functions Prototypes
*****************************************************************************/

int app_core1_boot_log(void);
int app_core2_boot_log(void);
void app_print_mem_information(void);

#if defined(__cplusplus)
}
#endif /* __cplusplus */

#endif /* __APP_LOGGER_H__ */

/* [] END OF FILE */
