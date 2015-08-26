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
/** @file UsbWinUSB.c
 *  @brief Contains APIs related to WINUSB (Human Interface Device) device class.
 */
//
//! \cond
//

/*
 * ======== UsbWinUSB.c ========
 */
#include "msp430F5529.h"
#include "../USB_Common/device.h"
#include "../USB_Common/defMSP430USB.h"
#include "../USB_Common/usb.h"                  //USB-specific Data Structures
#include "UsbWinUSB.h"
#include <descriptors.h>
#include <string.h>

#ifdef _WINUSB_

//function pointers
extern void *(*USB_TX_memcpy)(void * dest, const void * source, size_t count);
extern void *(*USB_RX_memcpy)(void * dest, const void * source, size_t count);

//Local Macros
#define INTFNUM_OFFSET(X)   (X - WINUSB0_INTFNUM)  //Get the WINUSB offset

extern uint8_t const report_len_input[WINUSB_NUM_INTERFACES];

static struct _WinUSBWrite {
    uint16_t nWinUSBBytesToSend;                       //holds counter of bytes to be sent
    uint16_t nWinUSBBytesToSendLeft;                   //holds counter how many bytes is still to be sent
    const uint8_t* pWinUSBBufferToSend;               //holds the buffer with data to be sent
    uint8_t bCurrentBufferXY;                      //indicates which buffer is to use next for for write into IN OUT endpoint
    uint8_t bZeroPacketSent;                       //= FALSE;
    uint8_t last_ByteSend;
} WinUSBWriteCtrl[WINUSB_NUM_INTERFACES];

static struct _WinUSBRead {
    uint8_t *pUserBuffer;                          //holds the current position of user's receiving buffer. If NULL- no receiving
                                                //operation started
    uint8_t *pCurrentEpPos;                        //current positon to read of received data from curent EP
    uint16_t nBytesToReceive;                       //holds how many bytes was requested by receiveData() to receive
    uint16_t nBytesToReceiveLeft;                   //holds how many bytes is still requested by receiveData() to receive
    uint8_t * pCT1;                                //holds current EPBCTxx register
    uint8_t * pCT2;                                //holds next EPBCTxx register
    uint8_t * pEP2;                                //holds addr of the next EP buffer
    uint8_t nBytesInEp;                            //how many received bytes still available in current EP
    uint8_t bCurrentBufferXY;                      //indicates which buffer is used by host to transmit data via OUT endpoint
} WinUSBReadCtrl[WINUSB_NUM_INTERFACES];

extern uint16_t wUsbEventMask;

uint8_t WinUSBProtocol[WINUSB_NUM_INTERFACES] = {0};
uint8_t WinUSBIdleRate[WINUSB_NUM_INTERFACES] = {0};

/*----------------------------------------------------------------------------+
 | Global Variables                                                            |
 +----------------------------------------------------------------------------*/

extern __no_init tEDB __data16 tInputEndPointDescriptorBlock[];
extern __no_init tEDB __data16 tOutputEndPointDescriptorBlock[];


void WinUSBCopyUsbToBuff (uint8_t* pEP, uint8_t* pCT, uint8_t);

/*----------------------------------------------------------------------------+
 | Functions' implementatin                                                    |
 +----------------------------------------------------------------------------*/

//resets internal WINUSB data structure
void WinUSBResetData ()
{

    //indicates which buffer is used by host to transmit data via OUT endpoint3 - X buffer is first
    //WinUSBReadCtrl[intfIndex].bCurrentBufferXY = X_BUFFER;

    memset(&WinUSBReadCtrl, 0, sizeof(WinUSBReadCtrl));
    memset(&WinUSBWriteCtrl, 0, sizeof(WinUSBWriteCtrl));

}

//
//! \endcond
//

//*****************************************************************************
//
//! Initiates Sending of a User Buffer Over WINUSB Interface.
//!
//! \param data is an array of data to be sent.
//! \param size is the number of bytes to be sent, starting from address
//! 	\b data.
//! \param intfNum is which data interface the \b data should be transmitted
//! 	over.
//!
//! Initiates sending of a user buffer over WINUSB interface \b intfNum, of size
//! \b size and starting at address \b data. If \b size is larger than the
//! packet size, the function handles all packetization and buffer management.
//! \b size has no inherent upper limit (beyond being a 16-bit value).
//!
//! In most cases where a send operation is successfully started, the function
//! will return \b USBWINUSB_SEND_STARTED. A send operation is said to be underway. At
//! some point, either before or after the function returns, the send operation
//! will complete, barring any events that would preclude it. (Even if the
//! operation completes before the function returns, the return code will still
//! be \b USBWINUSB_SEND_STARTED.)
//!
//! If the bus is not connected when the function is called, the function
//! returns \b USBWINUSB_BUS_NOT_AVAILABLE, and no operation is begun. If \b size is 0,
//! the function returns \b USBWINUSB_GENERAL_ERROR. If a previous send operation is
//! already underway for this data interface, the function returns with
//! \b USBWINUSB_INTERFACE_BUSY_ERROR.
//!
//! USB includes low-level mechanisms that ensure valid transmission of data.
//!
//! See Sec. 7.2 of \e "Programmer's Guide: MSP430 USB API Stack for CDC/PHDC/WINUSB/MSC" for a detailed discussion of
//! send operations.
//!
//! \return Any of the following:
//! 		- \b USBWINUSB_SEND_STARTED: a send operation was successfully
//! 			started.
//! 		- \b USBWINUSB_INTERFACE_BUSY_ERROR: a previous send operation is
//! 			underway.
//! 		- \b USBWINUSB_BUS_NOT_AVAILABLE: the bus is either suspended or
//! 			disconnected.
//! 		- \b USBWINUSB_GENERAL_ERROR: \b size was zero, or other
//! 			errorkUSBWINUSB_receiveCompleted.
//
//*****************************************************************************

uint8_t USBWINUSB_sendData (const uint8_t* data, uint16_t size, uint8_t intfNum)
{
	uint16_t state;
    uint8_t edbIndex;

    edbIndex = stUsbHandle[intfNum].edb_Index;

    if (size == 0){
        return (USBWINUSB_GENERAL_ERROR);
    }

    state = usbDisableInEndpointInterrupt(edbIndex);

    //atomic operation - disable interrupts

    //do not access USB memory if suspended (PLL off). It may produce BUS_ERROR
    if ((bFunctionSuspended) ||
        (bEnumerationStatus != ENUMERATION_COMPLETE)){
        //data can not be read because of USB suspended
    	usbRestoreInEndpointInterrupt(state);
        return (USBWINUSB_BUS_NOT_AVAILABLE);
    }

    if (WinUSBWriteCtrl[INTFNUM_OFFSET()].nWinUSBBytesToSendLeft != 0){
        //the USB still sends previous data, we have to wait
    	usbRestoreInEndpointInterrupt(state);
        return (USBWINUSB_INTERFACE_BUSY_ERROR);
    }

    //This function generate the USB interrupt. The data will be sent out from interrupt

    WinUSBWriteCtrl[INTFNUM_OFFSET(intfNum)].nWinUSBBytesToSend = size;
    WinUSBWriteCtrl[INTFNUM_OFFSET(intfNum)].nWinUSBBytesToSendLeft = size;
    WinUSBWriteCtrl[INTFNUM_OFFSET(intfNum)].pWinUSBBufferToSend = data;

    //trigger Endpoint Interrupt - to start send operation
    USBIEPIFG |= 1 << (edbIndex + 1);                                       //IEPIFGx;

    usbRestoreInEndpointInterrupt(state);

    return (USBWINUSB_SEND_STARTED);
}

//
//! \cond
//

//this function is used only by USB interrupt
int16_t WinUSBToHostFromBuffer (uint8_t intfNum)
{
	//P1OUT |= BIT0;
    uint8_t byte_count, nTmp2;
    uint8_t * pEP1;
    uint8_t * pEP2;
    uint8_t * pCT1;
    uint8_t * pCT2;
    uint8_t bWakeUp = FALSE;                                                   //per default we do not wake up after interrupt

    uint8_t edbIndex;

    edbIndex = stUsbHandle[intfNum].edb_Index;

    if (WinUSBWriteCtrl[INTFNUM_OFFSET(intfNum)].nWinUSBBytesToSendLeft == 0){    //do we have somtething to send?
        WinUSBWriteCtrl[INTFNUM_OFFSET(intfNum)].nWinUSBBytesToSend = 0;

        //call event callback function
        if (wUsbEventMask & USB_SEND_COMPLETED_EVENT){
            bWakeUp = USBWINUSB_handleSendCompleted(intfNum);
        }
        return (bWakeUp);
    }

    if (!(tInputEndPointDescriptorBlock[edbIndex].bEPCNF & EPCNF_TOGGLE)){
        //this is the active EP buffer
        pEP1 = (uint8_t*)stUsbHandle[intfNum].iep_X_Buffer;
        pCT1 = &tInputEndPointDescriptorBlock[edbIndex].bEPBCTX;

        //second EP buffer
        pEP2 = (uint8_t*)stUsbHandle[intfNum].iep_Y_Buffer;
        pCT2 = &tInputEndPointDescriptorBlock[edbIndex].bEPBCTY;
    } else {
        //this is the active EP buffer
        pEP1 = (uint8_t*)stUsbHandle[intfNum].iep_Y_Buffer;
        pCT1 = &tInputEndPointDescriptorBlock[edbIndex].bEPBCTY;

        //second EP buffer
        pEP2 = (uint8_t*)stUsbHandle[intfNum].iep_X_Buffer;
        pCT2 = &tInputEndPointDescriptorBlock[edbIndex].bEPBCTX;
    }

    //how many byte we can send over one endpoint buffer
    //2 bytes a reserved: [0] - WINUSB Report Descriptor, [1] - count of valid bytes
    byte_count =
        (WinUSBWriteCtrl[INTFNUM_OFFSET(intfNum)].nWinUSBBytesToSendLeft >
         EP_MAX_PACKET_SIZE) ? EP_MAX_PACKET_SIZE : WinUSBWriteCtrl[INTFNUM_OFFSET(intfNum)].nWinUSBBytesToSendLeft;
    nTmp2 = *pCT1;

    if (nTmp2 & EPBCNT_NAK){
        USB_TX_memcpy(pEP1, WinUSBWriteCtrl[INTFNUM_OFFSET(
                                                 intfNum)].pWinUSBBufferToSend,
            byte_count);                                                        //copy data into IEP3 X or Y buffer

        //64 bytes will be send: we use only one WINUSB report descriptor
        *pCT1 = 0x40;                                                           //Set counter for usb In-Transaction

        WinUSBWriteCtrl[INTFNUM_OFFSET(intfNum)].nWinUSBBytesToSendLeft -= byte_count;
        WinUSBWriteCtrl[INTFNUM_OFFSET(intfNum)].pWinUSBBufferToSend += byte_count;   //move buffer pointer

        //try to send data over second buffer
        nTmp2 = *pCT2;
        if ((WinUSBWriteCtrl[INTFNUM_OFFSET(intfNum)].nWinUSBBytesToSendLeft > 0) &&  //do we have more data to send?
            (nTmp2 & EPBCNT_NAK)){                                              //if the second buffer is free?
            //how many byte we can send over one endpoint buffer
            byte_count =
                (WinUSBWriteCtrl[INTFNUM_OFFSET(intfNum)].nWinUSBBytesToSendLeft >
                 EP_MAX_PACKET_SIZE ) ? EP_MAX_PACKET_SIZE: WinUSBWriteCtrl[INTFNUM_OFFSET(intfNum)].nWinUSBBytesToSendLeft;

            USB_TX_memcpy(pEP2, WinUSBWriteCtrl[INTFNUM_OFFSET(
                                                     intfNum)].pWinUSBBufferToSend,
                byte_count);                                                    //copy data into IEP3 X or Y buffer
            pEP2[0] = 4;                                                     //set WINUSB report descriptor: 0x3F
            pEP2[1] = 0;                                               //set byte count of valid data

            //64 bytes will be send: we use only one WINUSB report descriptor
            *pCT2 = 0x40;                                                       //Set counter for usb In-Transaction

            WinUSBWriteCtrl[INTFNUM_OFFSET(intfNum)].nWinUSBBytesToSendLeft -=
                byte_count;
            WinUSBWriteCtrl[INTFNUM_OFFSET(intfNum)].pWinUSBBufferToSend +=
                byte_count;                                                     //move buffer pointer
        }
    }
    return (bWakeUp);
}



//
//! \endcond
//

//*****************************************************************************
//
//! Aborts an Active Send Operation on Data Interface.
//!
//! \param size is the number of bytes that were sent prior to the aboert
//! 	action.
//! \param intfNum is the data interface for which the send should be
//! 	aborted.
//!
//! Aborts an active send operation on data interface \b intfNum. Returns
//! the number of bytes that were sent prior to the abort, in \b size.
//!
//! An application may choose to call this function if sending failed, due to
//! factors such as:
//! - a surprise removal of the bus
//! - a USB suspend event
//! - any send operation that extends longer than desired//!
//!
//! \return \b USB_SUCCEED
//
//*****************************************************************************

uint8_t USBWINUSB_abortSend (uint16_t* size, uint8_t intfNum)
{
    uint8_t edbIndex;
    uint16_t state;

    edbIndex = stUsbHandle[intfNum].edb_Index;

    state = usbDisableInEndpointInterrupt(edbIndex);

    *size =
        (WinUSBWriteCtrl[INTFNUM_OFFSET(intfNum)].nWinUSBBytesToSend -
         WinUSBWriteCtrl[INTFNUM_OFFSET(intfNum)].nWinUSBBytesToSendLeft);
    WinUSBWriteCtrl[INTFNUM_OFFSET(intfNum)].nWinUSBBytesToSend = 0;
    WinUSBWriteCtrl[INTFNUM_OFFSET(intfNum)].nWinUSBBytesToSendLeft = 0;

    usbRestoreInEndpointInterrupt(state);
    return (USB_SUCCEED);
}

//
//! \cond
//

//This function copies data from OUT endpoint into user's buffer
//Arguments:
//pEP - pointer to EP to copy from
//pCT - pointer to pCT control reg
//
void WinUSBCopyUsbToBuff (uint8_t* pEP, uint8_t* pCT,uint8_t intfNum)
{
    uint8_t nCount;

    //how many byte we can get from one endpoint buffer
    nCount =
        (WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesToReceiveLeft >
         WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp) ? WinUSBReadCtrl[
            INTFNUM_OFFSET(intfNum)].nBytesInEp : WinUSBReadCtrl[INTFNUM_OFFSET(
                                                                  intfNum)].
        nBytesToReceiveLeft;

    USB_RX_memcpy(WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].pUserBuffer, pEP, nCount);   //copy data from OEPx X or Y buffer
    WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesToReceiveLeft -= nCount;
    WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].pUserBuffer += nCount;                     //move buffer pointer
    //to read rest of data next time from this place

    if (nCount == WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp){                 //all bytes are copied from receive buffer?
        //switch current buffer
        WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].bCurrentBufferXY =
            (WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].bCurrentBufferXY + 1) & 0x01;

        WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp = 0;

        //clear NAK, EP ready to receive data
        *pCT = 0;
    } else {
        WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp -= nCount;
        WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].pCurrentEpPos = pEP + nCount;
    }
}

//*****************************************************************************
//
//! Receives \b size Bytes Over WINUSB Interface.
//!
//! \param data is an array to contain the data received.
//! \param size is the number of bytes to be received.
//! \param intfNum is which data interface to receive from.
//!
//! Receives \b size bytes over WINUSB interface \b intfNum into memory starting at
//! address \b data. \b size has no inherent upper limit (beyond being a 16-bit
//! value).
//!
//! The function may return with \b USBWINUSB_RECEIVE_STARTED, indicating that a
//! receive operation is underway. The operation completes when \b size bytes
//! are received. The application should ensure that the data memory buffer be
//! available during the whole of the receive operation.
//!
//! The function may also return with \b USBWINUSB_RECEIVE_COMPLETED. This means that
//! the receive operation was complete by the time the function returned.
//!
//! If the bus is not connected when the function is called, the function
//! returns \b USBWINUSB_BUS_NOT_AVAILABLE, and no operation is begun. If \b size is 0,
//! the function returns \b USBWINUSB_GENERAL_ERROR. If a previous receive operation
//! is already underway for this data interface, the function returns
//! \b USBWINUSB_INTERFACE_BUSY_ERROR.
//!
//! USB includes low-level mechanisms that ensure valid transmission of data.
//!
//! See Sec. 7.2 of \e "Programmer's Guide: MSP430 USB API Stack for CDC/PHDC/WINUSB/MSC" for a detailed discussion of
//! receive operations.
//!
//! \return Any of the following:
//! 		- \b USBWINUSB_RECEIVE_STARTED: A receive operation has been
//! 			successfully started.
//! 		- \b USBWINUSB_RECEIVE_COMPLETED: The receive operation is already
//! 			completed.
//! 		- \b USBWINUSB_INTERFACE_BUSY_ERROR: a previous receive operation is
//! 			underway.
//! 		- \b kUSBWINUSB_ busNotAvailable: the bus is either suspended or
//! 			disconnected.
//! 		- \b USBWINUSB_GENERAL_ERROR: size was zero, or other error.
//
//*****************************************************************************

uint8_t USBWINUSB_receiveDataBulk (uint8_t* data, uint16_t size, uint8_t intfNum)
{
    uint8_t nTmp1;
    uint16_t state;
    uint8_t edbIndex;

    edbIndex = stUsbHandle[intfNum].edb_Index;

    if ((size == 0) ||                                                      //read size is 0
        (data == NULL)){
        return (USBWINUSB_GENERAL_ERROR);
    }

    state = usbDisableOutEndpointInterrupt(edbIndex);

    //atomic operation - disable interrupts

    //do not access USB memory if suspended (PLL off). It may produce BUS_ERROR
    if ((bFunctionSuspended) ||
        (bEnumerationStatus != ENUMERATION_COMPLETE)){
    	usbRestoreOutEndpointInterrupt(state);
        return (USBWINUSB_BUS_NOT_AVAILABLE);
    }

    if (WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].pUserBuffer != NULL){          //receive process already started
    	usbRestoreOutEndpointInterrupt(state);
        return (USBWINUSB_RECEIVE_IN_PROGRESS);
    }


    WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesToReceive = size;            //bytes to receive
    WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesToReceiveLeft = size;        //left bytes to receive
    WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].pUserBuffer = data;                //set user receive buffer

    //read rest of data from buffer, if any
    if (WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp > 0){

        //copy data from pEP-endpoint into User's buffer
        WinUSBCopyUsbToBuff(WinUSBReadCtrl[INTFNUM_OFFSET(
                                         intfNum)].pCurrentEpPos,
            WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1,intfNum);

        if (WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesToReceiveLeft == 0){ //the Receive opereation is completed
            WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].pUserBuffer = NULL;        //no more receiving pending
            USBWINUSB_handleReceiveCompleted(intfNum);                         //call event handler in interrupt context
            usbRestoreOutEndpointInterrupt(state);
            return (USBWINUSB_RECEIVE_COMPLETED);                              //receive completed
        }

        //check other EP buffer for data - exchange pCT1 with pCT2
        if (WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1 ==
            &tOutputEndPointDescriptorBlock[edbIndex].bEPBCTX){
            WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1 =
                &tOutputEndPointDescriptorBlock[edbIndex].bEPBCTY;
            WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].pCurrentEpPos =
                (uint8_t*)stUsbHandle[intfNum].oep_Y_Buffer;
        } else {
            WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1 =
                &tOutputEndPointDescriptorBlock[edbIndex].bEPBCTX;
            WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].pCurrentEpPos =
                (uint8_t*)stUsbHandle[intfNum].oep_X_Buffer;
        }
        nTmp1 = *WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1;
        //try read data from second buffer
        if (nTmp1 & EPBCNT_NAK){                                            //if the second buffer has received data?
            nTmp1 = nTmp1 & 0x7f;                                           //clear NAK bit
            WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp =
                *(WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].pCurrentEpPos);  //holds how many valid bytes in the EP buffer
            if (WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp > nTmp1){
                WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp = nTmp1;
            }
            WinUSBCopyUsbToBuff(WinUSBReadCtrl[INTFNUM_OFFSET(
                                             intfNum)].pCurrentEpPos,
                WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1,intfNum);
        }

        if (WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesToReceiveLeft == 0){ //the Receive opereation is completed
            WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].pUserBuffer = NULL;        //no more receiving pending
            if (wUsbEventMask & USB_RECEIVED_COMPLETED_EVENT){
                USBWINUSB_handleReceiveCompleted(intfNum);                     //call event handler in interrupt context
            }
            usbRestoreOutEndpointInterrupt(state);
            return (USBWINUSB_RECEIVE_COMPLETED);                              //receive completed
        }
    } //read rest of data from buffer, if any

    //read 'fresh' data, if available
    nTmp1 = 0;
    if (WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].bCurrentBufferXY == X_BUFFER){ //this is current buffer
        if (tOutputEndPointDescriptorBlock[edbIndex].bEPBCTX & EPBCNT_NAK){ //this buffer has a valid data packet
            //this is the active EP buffer
            //pEP1
            WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].pCurrentEpPos =
                (uint8_t*)stUsbHandle[intfNum].oep_X_Buffer;
            WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1 =
                &tOutputEndPointDescriptorBlock[edbIndex].bEPBCTX;

            //second EP buffer
            WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].pEP2 =
                (uint8_t*)stUsbHandle[intfNum].oep_Y_Buffer;
            WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].pCT2 =
                &tOutputEndPointDescriptorBlock[edbIndex].bEPBCTY;
            nTmp1 = 1;                                                      //indicate that data is available
        }
    } else {                                                                //Y_BUFFER
        if (tOutputEndPointDescriptorBlock[edbIndex].bEPBCTY & EPBCNT_NAK){
            //this is the active EP buffer
            WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].pCurrentEpPos =
                (uint8_t*)stUsbHandle[intfNum].oep_Y_Buffer;
            WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1 =
                &tOutputEndPointDescriptorBlock[edbIndex].bEPBCTY;

            //second EP buffer
            WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].pEP2 =
                (uint8_t*)stUsbHandle[intfNum].oep_X_Buffer;
            WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].pCT2 =
                &tOutputEndPointDescriptorBlock[edbIndex].bEPBCTX;
            nTmp1 = 1;                                                      //indicate that data is available
        }
    }

    if (nTmp1){
        //how many byte we can get from one endpoint buffer
        nTmp1 = *WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1;

        if (nTmp1 & EPBCNT_NAK){
            nTmp1 = nTmp1 & 0x7f;                                           //clear NAK bit
            WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp =
                *(WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].pCurrentEpPos);  //holds how many valid bytes in the EP buffer
            if (WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp > nTmp1){
                WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp = nTmp1;
            }
            WinUSBCopyUsbToBuff(WinUSBReadCtrl[INTFNUM_OFFSET(
                                             intfNum)].pCurrentEpPos,
                WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1,intfNum);

            nTmp1 = *WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].pCT2;
            //try read data from second buffer
            if ((WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesToReceiveLeft >
                 0) &&                                                      //do we have more data to receive?
                (nTmp1 & EPBCNT_NAK)){                                      //if the second buffer has received data?
                nTmp1 = nTmp1 & 0x7f;                                       //clear NAK bit
                WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp =
                    *(WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].pEP2);       //holds how many valid bytes in the EP buffer
                if (WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp > nTmp1){
                    WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp = nTmp1;
                }
                WinUSBCopyUsbToBuff(WinUSBReadCtrl[INTFNUM_OFFSET(
                                                 intfNum)].pEP2,
                    WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].pCT2,intfNum);
                WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1 =
                    WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].pCT2;
            }
        }
    }

    if (WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesToReceiveLeft == 0){     //the Receive opereation is completed
        WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].pUserBuffer = NULL;            //no more receiving pending
        if (wUsbEventMask & USB_RECEIVED_COMPLETED_EVENT){
            USBWINUSB_handleReceiveCompleted(intfNum);                         //call event handler in interrupt context
        }
        usbRestoreOutEndpointInterrupt(state);
        return (USBWINUSB_RECEIVE_COMPLETED);
    }

    //interrupts enable
    usbRestoreOutEndpointInterrupt(state);
    return (USBWINUSB_RECEIVE_STARTED);
}

uint8_t USBWINUSB_receiveData (uint8_t* data, uint16_t size, uint8_t intfNum)
{
    uint8_t nTmp1;
    uint8_t edbIndex;
    uint16_t state;

    edbIndex = stUsbHandle[intfNum].edb_Index;

    if ((size == 0) ||                                                      //read size is 0
        (data == NULL)){
        return (USBWINUSB_GENERAL_ERROR);
    }

    state = usbDisableOutEndpointInterrupt(edbIndex);
    //atomic operation - disable interrupts

    //do not access USB memory if suspended (PLL off). It may produce BUS_ERROR
    if ((bFunctionSuspended) ||
        (bEnumerationStatus != ENUMERATION_COMPLETE)){
        //data can not be read because of USB suspended
    	usbRestoreOutEndpointInterrupt(state);
        return (USBWINUSB_BUS_NOT_AVAILABLE);
    }

    if (WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].pUserBuffer != NULL){          //receive process already started
    	usbRestoreOutEndpointInterrupt(state);
        return (USBWINUSB_INTERFACE_BUSY_ERROR);
    }

    WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesToReceive = size;            //bytes to receive
    WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesToReceiveLeft = size;        //left bytes to receive
    WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].pUserBuffer = data;                //set user receive buffer

    //read rest of data from buffer, if any
    if (WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp > 0){
        //copy data from pEP-endpoint into User's buffer
    	WinUSBCopyUsbToBuff(WinUSBReadCtrl[INTFNUM_OFFSET(
                                      intfNum)].pCurrentEpPos,
            WinUSBReadCtrl[INTFNUM_OFFSET(
                            intfNum)
            ].pCT1, intfNum);

        if (WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesToReceiveLeft == 0){ //the Receive opereation is completed
            WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].pUserBuffer = NULL;        //no more receiving pending
            if (wUsbEventMask & USB_RECEIVED_COMPLETED_EVENT){
                USBWINUSB_handleReceiveCompleted(intfNum);                     //call event handler in interrupt context
            }
            usbRestoreOutEndpointInterrupt(state);
            return (USBWINUSB_RECEIVE_COMPLETED);                              //receive completed
        }

        //check other EP buffer for data - exchange pCT1 with pCT2
        if (WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1 ==
            &tOutputEndPointDescriptorBlock[edbIndex].bEPBCTX){
            WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1 =
                &tOutputEndPointDescriptorBlock[edbIndex].bEPBCTY;
            WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].pCurrentEpPos =
                (uint8_t*)stUsbHandle[intfNum].oep_Y_Buffer;
        } else {
            WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1 =
                &tOutputEndPointDescriptorBlock[edbIndex].bEPBCTX;
            WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].pCurrentEpPos =
                (uint8_t*)stUsbHandle[intfNum].oep_X_Buffer;
        }

        nTmp1 = *WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1;
        //try read data from second buffer
        if (nTmp1 & EPBCNT_NAK){                                            //if the second buffer has received data?
            nTmp1 = nTmp1 & 0x7f;                                           //clear NAK bit
            WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp = nTmp1;        //holds how many valid bytes in the EP buffer
            WinUSBCopyUsbToBuff(WinUSBReadCtrl[INTFNUM_OFFSET(
                                          intfNum)].pCurrentEpPos,
                WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1, intfNum);
        }

        if (WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesToReceiveLeft == 0){ //the Receive opereation is completed
            WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].pUserBuffer = NULL;        //no more receiving pending
            if (wUsbEventMask & USB_RECEIVED_COMPLETED_EVENT){
                USBWINUSB_handleReceiveCompleted(intfNum);                     //call event handler in interrupt context
            }
            usbRestoreOutEndpointInterrupt(state);
            return (USBWINUSB_RECEIVE_COMPLETED);                              //receive completed
        }
    } //read rest of data from buffer, if any

    //read 'fresh' data, if available
    nTmp1 = 0;
    if (WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].bCurrentBufferXY == X_BUFFER){ //this is current buffer
        if (tOutputEndPointDescriptorBlock[edbIndex].bEPBCTX & EPBCNT_NAK){ //this buffer has a valid data packet
            //this is the active EP buffer
            //pEP1
            WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].pCurrentEpPos =
                (uint8_t*)stUsbHandle[intfNum].oep_X_Buffer;
            WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1 =
                &tOutputEndPointDescriptorBlock[edbIndex].bEPBCTX;

            //second EP buffer
            WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].pEP2 =
                (uint8_t*)stUsbHandle[intfNum].oep_Y_Buffer;
            WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].pCT2 =
                &tOutputEndPointDescriptorBlock[edbIndex].bEPBCTY;
            nTmp1 = 1;                                                      //indicate that data is available
        }
    } else {                                                                //Y_BUFFER
        if (tOutputEndPointDescriptorBlock[edbIndex].bEPBCTY & EPBCNT_NAK){
            //this is the active EP buffer
            WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].pCurrentEpPos =
                (uint8_t*)stUsbHandle[intfNum].oep_Y_Buffer;
            WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1 =
                &tOutputEndPointDescriptorBlock[edbIndex].bEPBCTY;

            //second EP buffer
            WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].pEP2 =
                (uint8_t*)stUsbHandle[intfNum].oep_X_Buffer;
            WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].pCT2 =
                &tOutputEndPointDescriptorBlock[edbIndex].bEPBCTX;
            nTmp1 = 1;                                                      //indicate that data is available
        }
    }

    if (nTmp1){
        //how many byte we can get from one endpoint buffer
        nTmp1 = *WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1;
        while (nTmp1 == 0)
        {
            nTmp1 = *WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1;
        }

        if (nTmp1 & EPBCNT_NAK){
            nTmp1 = nTmp1 & 0x7f;                                           //clear NAK bit
            WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp = nTmp1;        //holds how many valid bytes in the EP buffer

            WinUSBCopyUsbToBuff(WinUSBReadCtrl[INTFNUM_OFFSET(
                                          intfNum)].pCurrentEpPos,
                WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1, intfNum);

            nTmp1 = *WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].pCT2;
            //try read data from second buffer
            if ((WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesToReceiveLeft >
                 0) &&                                                      //do we have more data to send?
                (nTmp1 & EPBCNT_NAK)){                                      //if the second buffer has received data?
                nTmp1 = nTmp1 & 0x7f;                                       //clear NAK bit
                WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp = nTmp1;    //holds how many valid bytes in the EP buffer
                WinUSBCopyUsbToBuff(WinUSBReadCtrl[INTFNUM_OFFSET(
                                              intfNum)].pEP2,
                    WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].pCT2, intfNum);
                WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1 =
                    WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].pCT2;
            }
        }
    }

    if (WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesToReceiveLeft == 0){     //the Receive opereation is completed
        WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].pUserBuffer = NULL;            //no more receiving pending
        if (wUsbEventMask & USB_RECEIVED_COMPLETED_EVENT){
            USBWINUSB_handleReceiveCompleted(intfNum);                         //call event handler in interrupt context
        }
        usbRestoreOutEndpointInterrupt(state);
        return (USBWINUSB_RECEIVE_COMPLETED);
    }

    //interrupts enable
    usbRestoreOutEndpointInterrupt(state);
    return (USBWINUSB_RECEIVE_STARTED);
}
//
//! \cond
//

//this function is used only by USB interrupt.
//It fills user receiving buffer with received data
int16_t WinUSBToBufferFromHost (uint8_t intfNum)
{
    uint8_t * pEP1;
    uint8_t nTmp1;
    uint8_t bWakeUp = FALSE;                                                   //per default we do not wake up after interrupt

    uint8_t edbIndex;

    edbIndex = stUsbHandle[intfNum].edb_Index;

    if (WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesToReceiveLeft == 0){     //do we have something to receive?
        WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].pUserBuffer = NULL;            //no more receiving pending
        return (bWakeUp);
    }
    //No data to receive...
    if (!((tOutputEndPointDescriptorBlock[edbIndex].bEPBCTX |
           tOutputEndPointDescriptorBlock[edbIndex].bEPBCTY)
          & 0x80)){
        return (bWakeUp);
    }

    if (WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].bCurrentBufferXY == X_BUFFER){ //X is current buffer
        //this is the active EP buffer
        pEP1 = (uint8_t*)stUsbHandle[intfNum].oep_X_Buffer;
        WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1 =
            &tOutputEndPointDescriptorBlock[edbIndex].bEPBCTX;

        //second EP buffer
        WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].pEP2 =
            (uint8_t*)stUsbHandle[intfNum].oep_Y_Buffer;
        WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].pCT2 =
            &tOutputEndPointDescriptorBlock[edbIndex].bEPBCTY;
    } else {
        //this is the active EP buffer
        pEP1 = (uint8_t*)stUsbHandle[intfNum].oep_Y_Buffer;
        WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1 =
            &tOutputEndPointDescriptorBlock[edbIndex].bEPBCTY;

        //second EP buffer
        WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].pEP2 =
            (uint8_t*)stUsbHandle[intfNum].oep_X_Buffer;
        WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].pCT2 =
            &tOutputEndPointDescriptorBlock[edbIndex].bEPBCTX;
    }

    //how many byte we can get from one endpoint buffer
    nTmp1 = *WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1;

    if (nTmp1 & EPBCNT_NAK){
        nTmp1 = nTmp1 & 0x7f;                                                   //clear NAK bit
        WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp = *(pEP1);          //holds how many valid bytes in the EP buffer
        if (WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp > nTmp1){
            WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp = nTmp1;
        }                                                           //here starts user data
        WinUSBCopyUsbToBuff(pEP1, WinUSBReadCtrl[INTFNUM_OFFSET(
                                               intfNum)].pCT1,intfNum);

        nTmp1 = *WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].pCT2;
        //try read data from second buffer
        if ((WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesToReceiveLeft > 0) &&   //do we have more data to send?
            (nTmp1 & EPBCNT_NAK)){                                              //if the second buffer has received data?
            nTmp1 = nTmp1 & 0x7f;                                               //clear NAK bit
            WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp = *(pEP1);      //holds how many valid bytes in the EP buffer
            if (WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp > nTmp1){
                WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp = nTmp1;
            }
            WinUSBCopyUsbToBuff(WinUSBReadCtrl[INTFNUM_OFFSET(
                                             intfNum)].pEP2,
                WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].pCT2,intfNum);
            WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1 =
                WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].pCT2;
        }
    }

    if (WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesToReceiveLeft == 0){         //the Receive opereation is completed
        WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].pUserBuffer = NULL;                //no more receiving pending
        if (wUsbEventMask & USB_RECEIVED_COMPLETED_EVENT){
            bWakeUp = USBWINUSB_handleReceiveCompleted(intfNum);
        }

        if (WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp){                   //Is not read data still available in the EP?
            if (wUsbEventMask & USB_DATA_RECEIVED_EVENT){
                bWakeUp = USBWINUSB_handleDataReceived(intfNum);
            }
        }
    }
    return (bWakeUp);
}

//helper for USB interrupt handler
int16_t WinUSBIsReceiveInProgress (uint8_t intfNum)
{
    return (WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].pUserBuffer != NULL);
}

//
//! \endcond
//

//*****************************************************************************
//
//! Aborts an Active Recieve Operation on WINUSB Interface.
//!
//! \param size is the number of bytes that were received and are waiting at the
//! 	assigned address.
//! \param intfNum is the data interface for which the receive should be
//! 	aborted.
//!
//! Aborts an active receive operation on WINUSB interface \b intfNum. Returns the
//! number of bytes that were received and transferred to the data location
//! established for this receive operation. The data moved to the buffer up to
//! that time remains valid.
//!
//! An application may choose to call this function if it decides it no longer
//! wants to receive data from the USB host. It should be noted that if a
//! continuous stream of data is being received from the host, aborting the
//! operation is akin to pressing a "pause" button; the host will be NAK'ed
//! until another receive operation is opened.
//!
//! See Sec. 7.2 of \e "Programmer's Guide: MSP430 USB API Stack for CDC/PHDC/WINUSB/MSC" for a detailed discussion of
//! receive operations.
//!
//! \return \b USB_SUCCEED
//
//*****************************************************************************

uint8_t USBWINUSB_abortReceive (uint16_t* size, uint8_t intfNum)
{
    uint16_t state;
    uint8_t edbIndex;

    edbIndex = stUsbHandle[intfNum].edb_Index;
    state = usbDisableOutEndpointInterrupt(edbIndex);

    *size = 0;                                                                  //set received bytes count to 0

    //is receive operation underway?
    if (WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].pUserBuffer){
        //how many bytes are already received?
        *size = WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesToReceive -
                WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesToReceiveLeft;

        WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp = 0;
        WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].pUserBuffer = NULL;
        WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesToReceiveLeft = 0;
    }

    //restore interrupt status
    usbRestoreOutEndpointInterrupt(state);
    return (USB_SUCCEED);
}

//*****************************************************************************
//
//! Rejects Data Received from the Host.
//!
//! This function rejects data that has been received from the host, for
//! interface \b intfNum, that does not have an active receive operation underway.
//! It resides in the USB endpoint buffer and blocks further data until a
//! receive operation is opened, or until rejected. When this function is
//! called, the buffer for this interface is purged, and the data lost. This
//! frees the USB path to resume communication.
//!
//! See Sec. 7.2 of \e "Programmer's Guide: MSP430 USB API Stack for CDC/PHDC/WINUSB/MSC" for a detailed discussion of
//! receive operations.
//!
//! \return \b USB_SUCCEED
//
//*****************************************************************************

uint8_t USBWINUSB_rejectData (uint8_t intfNum)
{
	uint16_t state;
    uint8_t edbIndex;

    edbIndex = stUsbHandle[intfNum].edb_Index;

    state = usbDisableOutEndpointInterrupt(edbIndex);

    //interrupts disable

    //do not access USB memory if suspended (PLL off). It may produce BUS_ERROR
    if (bFunctionSuspended){
    	usbRestoreOutEndpointInterrupt(state);
        return (USBWINUSB_BUS_NOT_AVAILABLE);
    }

    //Is receive operation underway?
    //- do not flush buffers if any operation still active.
    if (!WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].pUserBuffer){
        uint8_t tmp1 = tOutputEndPointDescriptorBlock[edbIndex].bEPBCTX &
                    EPBCNT_NAK;
        uint8_t tmp2 = tOutputEndPointDescriptorBlock[edbIndex].bEPBCTY &
                    EPBCNT_NAK;

        if (tmp1 ^ tmp2){                                                       //switch current buffer if any and only ONE of the
                                                                                //buffers is full
            //switch current buffer
            WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].bCurrentBufferXY =
                (WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].bCurrentBufferXY +
                 1) & 0x01;
        }

        tOutputEndPointDescriptorBlock[edbIndex].bEPBCTX = 0;                   //flush buffer X
        tOutputEndPointDescriptorBlock[edbIndex].bEPBCTY = 0;                   //flush buffer Y
        WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp = 0;                    //indicates that no more data available in the EP
    }

    usbRestoreOutEndpointInterrupt(state);
    return (USB_SUCCEED);
}

//*****************************************************************************
//
//! Indicates the Status of the WINUSB Interface.
//!
//! \param intfNum is the interface number for which the status is being
//! 	retrieved.
//! \param bytesSent If a send operation is underway, the number of bytes
//! 	that have been transferred to the host is returned in this location. If
//! 	no send operation is underway, this returns zero.
//! \param bytesReceived If a receive operation is underway, the number of
//! 	bytes that have been transferred to the assigned memory location is
//! 	returned in this location. If no receive operation is underway, this
//! 	returns zero.
//!
//! Indicates the status of the WINUSB interface \b intfNum. If a send operation is
//! active for this interface, the function also returns the number of bytes
//! that have been transmitted to the host. If a receive operation is active for
//! this interface, the function also returns the number of bytes that have been
//! received from the host and are waiting at the assigned address.
//!
//! Because multiple flags can be returned, the possible values can be masked
//! together - for example, \b USBWINUSB_WAITING_FOR_SEND + \b USBWINUSB_DATA_WAITING.
//!
//! \return Any combination of the following:
//! 		- \b USBWINUSB_WAITING_FOR_SEND: Indicates that a send operation is open
//! 			on this interface.
//! 		- \b USBWINUSB_WAITING_FOR_RECEIVE: Indicates that a receive operation
//! 			is open on this interface.
//! 		- \b USBWINUSB_DATA_WAITING: Indicates that data has been received from
//! 			the host for this interface, waiting in the USB receive buffers,
//! 			lacking an open receive operation to accept it.
//! 		- \b USBWINUSB_BUS_NOT_AVAILABLE: Indicates that the bus is either
//! 			suspended or disconnected. Any operations that had previously
//! 			been underway are now aborted.
//
//*****************************************************************************

uint8_t USBWINUSB_getInterfaceStatus (uint8_t intfNum, uint16_t* bytesSent, uint16_t* bytesReceived)
{
    uint8_t ret = 0;
    uint16_t stateIn, stateOut;
    uint8_t edbIndex;

    *bytesSent = 0;
    *bytesReceived = 0;

    edbIndex = stUsbHandle[intfNum].edb_Index;

    stateIn = usbDisableInEndpointInterrupt(edbIndex);
    stateOut = usbDisableOutEndpointInterrupt(edbIndex);

    //Is send operation underway?
    if (WinUSBWriteCtrl[INTFNUM_OFFSET(intfNum)].nWinUSBBytesToSendLeft != 0){
        ret |= USBWINUSB_WAITING_FOR_SEND;
        *bytesSent = WinUSBWriteCtrl[INTFNUM_OFFSET(intfNum)].nWinUSBBytesToSend -
                     WinUSBWriteCtrl[INTFNUM_OFFSET(intfNum)].nWinUSBBytesToSendLeft;
    }

    //Is receive operation underway?
    if (WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].pUserBuffer != NULL){
        ret |= USBWINUSB_WAITING_FOR_RECEIVE;
        *bytesReceived = WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesToReceive -
                         WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].
                         nBytesToReceiveLeft;
    } else {                                                                    //not receive operation started
        //do not access USB memory if suspended (PLL off).
        //It may produce BUS_ERROR
        if (!bFunctionSuspended){
            if ((tOutputEndPointDescriptorBlock[edbIndex].bEPBCTX &
                 EPBCNT_NAK)  |                                                 //any of buffers has a valid data packet
                (tOutputEndPointDescriptorBlock[edbIndex].bEPBCTY &
                 EPBCNT_NAK)){
                ret |= USBWINUSB_DATA_WAITING;
            }
        }
    }

    if ((bFunctionSuspended) ||
        (bEnumerationStatus != ENUMERATION_COMPLETE)){
        //if suspended or not enumerated  - report no other tasks pending
        ret = USBWINUSB_BUS_NOT_AVAILABLE;
    }

    //restore interrupt status
    usbRestoreInEndpointInterrupt(stateIn);
    usbRestoreOutEndpointInterrupt(stateOut);

    return (ret);
}

//*****************************************************************************
//
//! Returns the Number of Bytes Waiting in the USB Endpoint Buffer.
//!
//! \param intfNum is the data interface whose buffer is to be checked.
//!
//! Returns the number of bytes waiting in the USB endpoint buffer for
//! \b intfNum. A non-zero value generally means that no receive operation is
//! open by which these bytes can be copied to a user buffer. If the value is
//! non-zero, the application should either open a receive operation so that the
//! data can be moved out of the endpoint buffer, or the data should be rejected
//! (USBWINUSB_rejectData()).
//!
//! \return The number of bytes waiting in this buffer.
//
//*****************************************************************************

uint8_t USBWINUSB_getBytesInUSBBuffer (uint8_t intfNum)
{
    uint8_t bTmp1 = 0;
    uint8_t bTmp2;

    uint8_t edbIndex;
    uint16_t state;


    edbIndex = stUsbHandle[intfNum].edb_Index;

    //interrupts disable
    state = usbDisableOutEndpointInterrupt(edbIndex);

    if ((bFunctionSuspended) ||
        (bEnumerationStatus != ENUMERATION_COMPLETE)){
        //if suspended or not enumerated - report 0 bytes available
    	usbRestoreOutEndpointInterrupt(state);
        return (0);
    }

    if (WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp > 0){                   //If a RX operation is underway, part of data may
    	                                            //was read of the OEP buffer
        bTmp1 = WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp;
        if (*WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].pCT2 & EPBCNT_NAK){           //the next buffer has a valid data packet
            bTmp2 = *(WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].pEP2);           //holds how many valid bytes in the EP buffer
            if (bTmp2 >
                (*WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].pCT2 & 0x7F)){       //check if all data received correctly
                bTmp1 +=
                    (*WinUSBReadCtrl[INTFNUM_OFFSET(intfNum)].pCT2 & 0x7F);
            } else {
                bTmp1 += bTmp2;
            }
        }
    } else {
        if (tOutputEndPointDescriptorBlock[edbIndex].bEPBCTX & EPBCNT_NAK){     //this buffer has a valid data packet
            bTmp2 = tOutputEndPointDescriptorBlock[edbIndex].bEPBCTX & 0x7F;
            bTmp1 = *((uint8_t*)stUsbHandle[intfNum].oep_X_Buffer);
            if (bTmp2 < bTmp1){                                             //check if the count (second byte) is valid
                bTmp1 = bTmp2;
            }
        }
        if (tOutputEndPointDescriptorBlock[edbIndex].bEPBCTY & EPBCNT_NAK){     //this buffer has a valid data packet
            bTmp2 = tOutputEndPointDescriptorBlock[edbIndex].bEPBCTY & 0x7F;
            if (bTmp2 > *((uint8_t*)stUsbHandle[intfNum].oep_Y_Buffer)){   //check if the count (second byte) is valid
                bTmp1 += *((uint8_t*)stUsbHandle[intfNum].oep_Y_Buffer);
            } else {
                bTmp1 += bTmp2;
            }
        }
    }

    //interrupts enable
    usbRestoreOutEndpointInterrupt(state);
    return (bTmp1);
}

//
//! \cond
//

#endif //ifdef _WINUSB_

//
//! \endcond
//

/*----------------------------------------------------------------------------+
 | End of source file                                                          |
 +----------------------------------------------------------------------------*/
/*------------------------ Nothing Below This Line --------------------------*/
//Released_Version_5_00_01
