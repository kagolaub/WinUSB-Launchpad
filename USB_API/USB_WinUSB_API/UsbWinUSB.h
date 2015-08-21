/* --COPYRIGHT--,BSD
 * Copyright (c) 2014, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
/*
 * ======== UsbWinUSB.h ========
 */
#include <stdint.h>

#ifndef _UsbWinUSB_H_
#define _UsbWinUSB_H_

#ifdef __cplusplus
extern "C"
{
#endif

/*----------------------------------------------------------------------------
 * The following function names and macro names are deprecated.  These were
 * updated to new names to follow OneMCU naming convention.
 +---------------------------------------------------------------------------*/
#ifndef DEPRECATED
#define  kUSBWINUSB_sendStarted          USBWINUSB_SEND_STARTED
#define  kUSBWINUSB_sendComplete         USBWINUSB_SEND_COMPLETE
#define  kUSBWINUSB_intfBusyError        USBWINUSB_INTERFACE_BUSY_ERROR
#define  kUSBWINUSB_receiveStarted       USBWINUSB_RECEIVE_STARTED
#define  kUSBWINUSB_receiveCompleted     USBWINUSB_RECEIVE_COMPLETED
#define  kUSBWINUSB_receiveInProgress    USBWINUSB_RECEIVE_IN_PROGRESS
#define  kUSBWINUSB_generalError         USBWINUSB_GENERAL_ERROR
#define  kUSBWINUSB_busNotAvailable      USBWINUSB_BUS_NOT_AVAILABLE
#define  kUSBWINUSB_waitingForSend       USBWINUSB_WAITING_FOR_SEND
#define  kUSBWINUSB_waitingForReceive    USBWINUSB_WAITING_FOR_RECEIVE
#define  kUSBWINUSB_dataWaiting          USBWINUSB_DATA_WAITING
#define  kUSB_allWinUSBEvents            USBWINUSB_ALL_WINUSB_EVENTS
#define  kUSBWINUSB_noDataWaiting        USBWINUSB_NO_DATA_WAITING

#define   USBWINUSB_intfStatus           USBWINUSB_getInterfaceStatus
#define   USBWINUSB_bytesInUSBBuffer     USBWINUSB_getBytesInUSBBuffer

#endif



#define USBWINUSB_SEND_STARTED         0x01
#define USBWINUSB_SEND_COMPLETE        0x02
#define USBWINUSB_INTERFACE_BUSY_ERROR       0x03
#define USBWINUSB_RECEIVE_STARTED      0x04
#define USBWINUSB_RECEIVE_COMPLETED    0x05
#define USBWINUSB_RECEIVE_IN_PROGRESS   0x06
#define USBWINUSB_GENERAL_ERROR        0x07
#define USBWINUSB_BUS_NOT_AVAILABLE     0x08
#define WINUSB_BOOT_PROTOCOL       0x00
//returned by USBWINUSB_rejectData() if no data pending
#define USBWINUSB_NO_DATA_WAITING        1
#define USBWINUSB_WAITING_FOR_SEND      0x01
#define USBWINUSB_WAITING_FOR_RECEIVE   0x02
#define USBWINUSB_DATA_WAITING         0x04
#define USBWINUSB_BUS_NOT_AVAILABLE     0x08
#define USBWINUSB_ALL_WINUSB_EVENTS           0xFF


/*----------------------------------------------------------------------------
 * These functions can be used in application
 +----------------------------------------------------------------------------*/


/*
 * Sends data over interface intfNum, of size size and starting at address data.
 * Returns:  USBWINUSB_SEND_STARTED
 *          USBWINUSB_SEND_COMPLETE
 *          USBWINUSB_INTERFACE_BUSY_ERROR
 */
uint8_t USBWINUSB_sendData (const uint8_t* data, uint16_t size, uint8_t intfNum);

/*
 * Receives data over interface intfNum, of size size, into memory starting at address data.
 */
uint8_t USBWINUSB_receiveData (uint8_t* data, uint16_t size, uint8_t intfNum);
/*
 * Aborts an active receive operation on interface intfNum.
 * size: the number of bytes that were received and transferred
 * to the data location established for this receive operation.
 */
uint8_t USBWINUSB_abortReceive (uint16_t* size, uint8_t intfNum);

/*
 * This function rejects payload data that has been received from the host.
 */
uint8_t USBWINUSB_rejectData (uint8_t intfNum);

/*
 * Aborts an active send operation on interface intfNum.  Returns the number of bytes that were sent prior to the abort, in size.
 */
uint8_t USBWINUSB_abortSend (uint16_t* size, uint8_t intfNum);

/*
 * This function indicates the status of the interface intfNum.
 * If a send operation is active for this interface,
 * the function also returns the number of bytes that have been transmitted to the host.
 * If a receiver operation is active for this interface, the function also returns
 * the number of bytes that have been received from the host and are waiting at the assigned address.
 *
 * returns USBWINUSB_WAITING_FOR_SEND (indicates that a call to USBWINUSB_SendData()
 * has been made, for which data transfer has not been completed)
 *
 * returns USBWINUSB_WAITING_FOR_RECEIVE (indicates that a receive operation
 * has been initiated, but not all data has yet been received)
 *
 * returns USBWINUSB_DATA_WAITING (indicates that data has been received
 * from the host, waiting in the USB receive buffers)
 */
uint8_t USBWINUSB_getInterfaceStatus (uint8_t intfNum, uint16_t* bytesSent, uint16_t* bytesReceived);

/*
 * Returns how many bytes are in the buffer are received and ready to be read.
 */
uint8_t USBWINUSB_getBytesInUSBBuffer (uint8_t intfNum);

/*----------------------------------------------------------------------------
 * Event-Handling routines
 +----------------------------------------------------------------------------*/

/*
 * This event indicates that data has been received for port port, but no data receive operation is underway.
 * returns TRUE to keep CPU awake
 */
uint8_t USBWINUSB_handleDataReceived (uint8_t intfNum);

/*
 * This event indicates that a send operation on port port has just been completed.
 * returns TRUE to keep CPU awake
 */
uint8_t USBWINUSB_handleSendCompleted (uint8_t intfNum);

/*
 * This event indicates that a receive operation on port port has just been completed.
 * returns TRUE to keep CPU awake
 */
uint8_t USBWINUSB_handleReceiveCompleted (uint8_t intfNum);

#ifdef __cplusplus
}
#endif
#endif  //_UsbWinUSB_H_
//Released_Version_5_00_01
