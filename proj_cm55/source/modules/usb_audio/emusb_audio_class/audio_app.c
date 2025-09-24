/******************************************************************************
* File Name : audio_app.c
*
* Description :
* USB audio class configuration code.
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
#include "audio.h"
#include "audio_app.h"
#include "audio_send_task.h"
#include "audio_receive_task.h"
#include "cycfg_emusbdev.h"
#include "USB_Audio.h"
#include "USB_HID.h"
#include "rtos.h"
#include "cy_pdl.h"
#include "cybsp.h"
#include "cycfg.h"


/*******************************************************************************
* Macros
*******************************************************************************/
#define DELAY_TICKS                        (50U)
#define EP_IN_INTERVAL                     (8U)
#define EP_OUT_INTERVAL                    (8U)

/*******************************************************************************
* Global Variables
*******************************************************************************/
uint8_t  audio_app_control_report[2];
uint32_t audio_app_current_sample_rate;
int8_t   audio_app_volume;
int8_t   audio_app_prev_volume;
bool     audio_app_mute;
uint8_t current_speaker_format_index;
uint8_t current_microphone_format_index;

static USBD_AUDIO_INIT_DATA audio_initData;
static USB_HID_INIT_DATA hid_initData;
USBD_AUDIO_HANDLE usb_audioContext;
USB_HID_HANDLE usb_hidControlContext;

/* Speaker configurations */
static USBD_AUDIO_IF_CONF * speakerConfig = (USBD_AUDIO_IF_CONF *) &audio_interfaces[0];

/* Microphone configurations */
static USBD_AUDIO_IF_CONF  * microphoneConfig = (USBD_AUDIO_IF_CONF *) &audio_interfaces[1]; 

/*******************************************************************************
* Functions Prototypes
*******************************************************************************/
void audio_clock_init(void);

/*******************************************************************************
* Function Name: audio_control_callback
********************************************************************************
* Summary:
* Audio control callback for USB audio class.
*
*******************************************************************************/

static int audio_control_callback(void * pUserContext, U8 Event, U8 Unit, U8 ControlSelector, U8 * pBuffer, U32 NumBytes, U8 InterfaceNo, U8 AltSetting) 
{
    int retVal;

    (void)pUserContext;
    (void)InterfaceNo;
    retVal = 0;

    switch (Event) {

        case USB_AUDIO_PLAYBACK_START:

            /* Host enabled transmission */
            audio_out_enable();

        break;

        case USB_AUDIO_PLAYBACK_STOP:

            /* Host disabled transmission. Some hosts do not always send this! */
            audio_out_disable();

        break;

        case USB_AUDIO_RECORD_START:

            /* Host enabled reception */
            audio_in_enable();
        break;

        case USB_AUDIO_RECORD_STOP:

            /* Host disabled reception. Some hosts do not always send this! */
            audio_in_disable();
        break;

        case USB_AUDIO_SET_CUR:

            switch (ControlSelector) 
            {

                case USB_AUDIO_MUTE_CONTROL:
                break;

                case USB_AUDIO_VOLUME_CONTROL:

                break;

                case USB_AUDIO_SAMPLING_FREQ_CONTROL:

                if (NumBytes == 3)
                {
                    if (Unit == speakerConfig->pUnits->FeatureUnitID)
                    {
                        if ((AltSetting > 0) && (AltSetting < speakerConfig->NumFormats))
                        {
                            current_speaker_format_index = AltSetting - 1;
                        }
                    }
                    if (Unit == microphoneConfig->pUnits->FeatureUnitID) {
                        if ((AltSetting > 0) && (AltSetting < microphoneConfig->NumFormats)) 
                        {
                            current_microphone_format_index = AltSetting - 1;
                        }
                    }
                }

                break;

                default:
                    retVal = 1;
                break;
            }

        break;

        case USB_AUDIO_GET_CUR:
            switch (ControlSelector)
            {
                case USB_AUDIO_MUTE_CONTROL:
                      pBuffer[0] = 0;
                break;

                case USB_AUDIO_VOLUME_CONTROL:
                    pBuffer[0] = 0;
                    pBuffer[1] = 0;
                break;

                case USB_AUDIO_SAMPLING_FREQ_CONTROL:
                    if (Unit == speakerConfig->pUnits->FeatureUnitID)
                    {
                        pBuffer[0] =  speakerConfig->paFormats[current_speaker_format_index].SamFreq & 0xff;
                        pBuffer[1] = (speakerConfig->paFormats[current_speaker_format_index].SamFreq >> 8) & 0xff;
                        pBuffer[2] = (speakerConfig->paFormats[current_speaker_format_index].SamFreq >> 16) & 0xff;
                    }

                    if (Unit == microphoneConfig->pUnits->FeatureUnitID)
                    {
                        pBuffer[0] =  microphoneConfig->paFormats[current_microphone_format_index].SamFreq & 0xff;
                        pBuffer[1] = (microphoneConfig->paFormats[current_microphone_format_index].SamFreq >> 8) & 0xff;
                        pBuffer[2] = (microphoneConfig->paFormats[current_microphone_format_index].SamFreq >> 16) & 0xff;
                    }
                break;

                default:
                    pBuffer[0] = 0;
                    pBuffer[1] = 0;
                break;
            }
        break;

        case USB_AUDIO_SET_MIN:
        break;

        case USB_AUDIO_GET_MIN:
            switch (ControlSelector)
            {
                case USB_AUDIO_VOLUME_CONTROL:
                    pBuffer[0] = 0;
                    pBuffer[1] = 0xf1;
                break;

                default:
                    pBuffer[0] = 0;
                    pBuffer[1] = 0;
                break;
            }
        break;

        case USB_AUDIO_SET_MAX:
        break;

        case USB_AUDIO_GET_MAX:
            switch (ControlSelector)
            {
                case USB_AUDIO_VOLUME_CONTROL:
                    pBuffer[0] = 0;
                    pBuffer[1] = 0;
                break;

                default:
                    pBuffer[0] = 0;
                    pBuffer[1] = 0;
                break;
            }
        break;

        case USB_AUDIO_SET_RES:
        break;

        case USB_AUDIO_GET_RES:
            switch (ControlSelector)
            {
                case USB_AUDIO_VOLUME_CONTROL:
                    pBuffer[0] = 0;
                    pBuffer[1] = 1;
                break;

                default:
                    pBuffer[0] = 0;
                    pBuffer[1] = 0;
                break;
            }

        break;

        default:
            retVal = 1;
        break;
    }

    return retVal;
}

/*******************************************************************************
* Function Name: addAudio
********************************************************************************

* Summary:
*  Add a USB Audio interface to the USB stack.
*******************************************************************************/

static USBD_AUDIO_HANDLE addAudio(void)
 {
    USBD_AUDIO_HANDLE hInst;
    
    USB_ADD_EP_INFO       EPOut;
    USB_ADD_EP_INFO       EPIn;
    memset(&EPIn, 0x0, sizeof(EPIn));
    memset(&audio_initData, 0x0, sizeof(audio_initData));

    memset(&EPOut, 0x0, sizeof(EPOut));

    EPOut.MaxPacketSize              = MAX_AUDIO_OUT_PACKET_SIZE_BYTES;
    EPOut.Interval                   = EP_OUT_INTERVAL;
    EPOut.Flags                      = USB_ADD_EP_FLAG_USE_ISO_SYNC_TYPES;
    EPOut.InDir                      = USB_DIR_OUT;
    EPOut.TransferType               = USB_TRANSFER_TYPE_ISO;
    EPOut.ISO_Type                   = USB_ISO_SYNC_TYPE_ASYNCHRONOUS;

    EPIn.MaxPacketSize               = MAX_AUDIO_IN_PACKET_SIZE_BYTES;       /* Max packet size for IN endpoint (in bytes) */
    EPIn.Interval                    = EP_IN_INTERVAL;                       /* Interval of 1 ms (8 * 125us) */
    EPIn.Flags                       = USB_ADD_EP_FLAG_USE_ISO_SYNC_TYPES;   /* Optional parameters */
    EPIn.InDir                       = USB_DIR_IN;                           /* IN direction (Device to Host) */
    EPIn.TransferType                = USB_TRANSFER_TYPE_ISO;                /* Endpoint type - Isochronous. */
    EPIn.ISO_Type                    = USB_ISO_SYNC_TYPE_ASYNCHRONOUS;       /* Async for isochronous endpoints */

    memset(&audio_initData, 0, sizeof(audio_initData));

    audio_initData.EPIn                   = USBD_AddEPEx(&EPIn, NULL, 0);
    audio_initData.EPOut                  = USBD_AddEPEx(&EPOut, NULL, 0);
    audio_initData.OutPacketSize          = EPOut.MaxPacketSize;
    audio_initData.pfOnOut                = &audio_out_endpoint_callback;
    audio_initData.pfOnIn                 = &audio_in_endpoint_callback;
    audio_initData.pfOnControl            = audio_control_callback;
    audio_initData.pControlUserContext    = NULL;
    audio_initData.NumInterfaces          = SEGGER_COUNTOF(audio_interfaces);
    audio_initData.paInterfaces           = audio_interfaces;
    audio_initData.pOutUserContext        = NULL;
    audio_initData.pInUserContext         = NULL;

    hInst = USBD_AUDIO_Add(&audio_initData);
    return hInst;

}

/*******************************************************************************
* Function Name: addHIDControl
********************************************************************************

* Summary:
*  Add HID mouse class to USB stack
*******************************************************************************/
static USB_HID_HANDLE addHIDControl(void) 
{
  
  USB_HID_HANDLE hInst;
  
  USB_ADD_EP_INFO   EPIntIn;

  memset(&hid_initData, 0, sizeof(hid_initData));
  EPIntIn.Flags           = 0;                             // Flags not used.
  EPIntIn.InDir           = USB_DIR_IN;                    // IN direction (Device to Host)
  EPIntIn.Interval        = 8;                             // Interval of 1 ms (125 us * 8)
  EPIntIn.MaxPacketSize   = 2;                             // Report size.
  EPIntIn.TransferType    = USB_TRANSFER_TYPE_INT;         // Endpoint type - Interrupt.
  hid_initData.EPIn       = USBD_AddEPEx(&EPIntIn, NULL, 0);

  hid_initData.pReport = hidReport;
  hid_initData.NumBytesReport = sizeof(hidReport);
  hInst = USBD_HID_Add(&hid_initData);

  return hInst;
}

/*******************************************************************************
* Function Name: audio_app_init
********************************************************************************

* Summary:
*  Initialize USB audio class.
*******************************************************************************/
void audio_app_init()
{

    USBD_Init();
    /* Add Audio endpoints to the USB stack */
    usb_audioContext = addAudio();

    /* Set device information*/
    USBD_SetDeviceInfo(&usb_deviceInfo);

   /* Add HID Audio control endpoint to the USB stack */
     usb_hidControlContext = addHIDControl();

    /* Set USB read/write timeouts */
    USBD_AUDIO_Set_Timeouts(usb_audioContext, READ_TIMEOUT, WRITE_TIMEOUT);

    /* Schedules task for IN endpoint. */
    audio_in_init();

    /* Schedules task for OUT endpoint */
    audio_out_init();
    /* Start the USB device stack */
    USBD_Start();

}


/*******************************************************************************
* Function Name: audio_app_process
********************************************************************************
* Summary:
*   Main audio task. Initialize the USB communication and the audio application.
*   In the main loop, process requests to update the sample rate and change
*   the volume.
*
*******************************************************************************/
void audio_app_process(void *arg)
{

    audio_app_init();

    while(1)
    {
        /* Send out report data */
        USBD_HID_Write(usb_hidControlContext, audio_app_control_report, 2, 0);

        audio_app_control_report[0] = 0x01;
        audio_app_control_report[1] = 0;

        /* Send out report data */
        USBD_HID_Write(usb_hidControlContext, audio_app_control_report, 2, 0);

        vTaskDelay(pdMS_TO_TICKS(DELAY_TICKS));
    }

}
/* [] END OF FILE */
