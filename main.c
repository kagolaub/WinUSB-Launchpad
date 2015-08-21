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
 * ======== main.c ========
 * "Simple Send"
 *
 * This example shows a very simplified way of sending.  This might be used in
 * an application where the main functionality doesn't change much if a USB
 * host is present or not.  It simply calls USBCDC_sendDataInBackground(); if no
 * host is present, it simply carries on.
 *
 * Simply build, run, and attach to a USB host.  Run the Java HID Demo App
 * and open a connection to a device with VID=0x2047 and PID 0x0301.  The time
 * should be displayed, every second.
 *
 * Note that this code is identical to example C9, except for one line of
 * code:  the call to USBHID_sendDataInBackground() instead of
 * USBCDC_sendDataInBackground().  This reflects the symmetry between writing
 * MSP430 code for CDC vs. HID-Datapipe.
 *
 *+----------------------------------------------------------------------------+
 * Please refer to the MSP430 USB API Stack Programmer's Guide,located
 * in the root directory of this installation for more details.
 * ---------------------------------------------------------------------------*/

#include <string.h>

#include "driverlib.h"

#include "USB_config/descriptors.h"
#include "USB_API/USB_Common/device.h"
#include "USB_API/USB_Common/usb.h"                     // USB-specific functions
#include "USB_API/USB_WINUSB_API/UsbWinUSB.h"
#include "USB_app/usbConstructs.h"
#include "msp430F5529.h"
/*
 * NOTE: Modify hal.h to select a specific evaluation board and customize for
 * your own board.
 */
#include "hal.h"


// Function declarations
void initRTC(void);

// Global flags set by events
volatile uint8_t bWINUSBDataReceived_event = FALSE;  // Flag set by event handler to
													 // indicate data has been
													 // received into USB buffer

#define BUFFER_SIZE 64
uint8_t dataBuffer[BUFFER_SIZE];                  // User buffer to hold the data
/*
 * ======== main ========
 */
void main(void)
{
    WDT_A_hold(WDT_A_BASE); // Stop watchdog timer

    // MSP430 USB requires the maximum Vcore setting; do not modify
    PMM_setVCore(PMM_CORE_LEVEL_2);
    USBHAL_initPorts();           // Config GPIOS for low-power (output low)
    USBHAL_initClocks(8000000);   // Config clocks. MCLK=SMCLK=FLL=8MHz; ACLK=REFO=32kHz
    USB_setup(TRUE,TRUE);  // Init USB & events; if a host is present, connect
    initRTC();             // Start the real-time clock

    __enable_interrupt();  // Enable interrupts globally
    while (1)
    {
    	uint8_t ReceiveError = 0, SendError = 0;
    	uint16_t count;

    	// Check the USB state and directly main loop accordingly
    	switch (USB_getConnectionState())
    	{
    	// This case is executed while your device is enumerated on the
    	// USB host
    	case ST_ENUM_ACTIVE:

    		// Sleep if there are no bytes to process.
    		__disable_interrupt();
    		if (!USBWINUSB_getBytesInUSBBuffer(WINUSB0_INTFNUM)) {
    			// Enter LPM0 until awakened by an event handler
    			__bis_SR_register(LPM0_bits + GIE);
    		}

    		__enable_interrupt();

    		count = USBWINUSB_receiveDataInBuffer((uint8_t*)dataBuffer,
    				BUFFER_SIZE,
					WINUSB0_INTFNUM);

    		// Count has the number of bytes received into dataBuffer
    		// Echo back to the host.
    		if (USBWINUSB_sendDataInBackground((uint8_t*)dataBuffer,
    				count, WINUSB0_INTFNUM, 1)){
    			// Exit if something went wrong.
    			SendError = 0x01;
    			break;
    		}
    		break;

    		// These cases are executed while your device is disconnected from
    		// the host (meaning, not enumerated); enumerated but suspended
    		// by the host, or connected to a powered hub without a USB host
    		// present.
                case ST_PHYS_DISCONNECTED:
                case ST_ENUM_SUSPENDED:
                case ST_PHYS_CONNECTED_NOENUM_SUSP:
                    __bis_SR_register(LPM3_bits + GIE);
                    _NOP();
                    break;

                // The default is executed for the momentary state
                // ST_ENUM_IN_PROGRESS.  Usually, this state only last a few
                // seconds.  Be sure not to enter LPM3 in this state; USB
                // communication is taking place here, and therefore the mode must
                // be LPM0 or active-CPU.
                case ST_ENUM_IN_PROGRESS:
                default:;
            }

            if (ReceiveError || SendError){
            	//P1OUT |= BIT0; //if error red LED will be lit
            }
        }  //while(1)
}  //main()


// Starts a real-time clock on TimerA_0.  Earlier we assigned ACLK to be driven
// by the REFO, at 32768Hz.  So below we set the timer to count up to 32768 and
// roll over; and generate an interrupt when it rolls over.
void initRTC(void)
{
    TA0CCR0 = 32768;
    TA0CTL = TASSEL_1+MC_1+TACLR; // ACLK, count to CCR0 then roll, clear TAR
    TA0CCTL0 = CCIE;              // Gen int at rollover (TIMER0_A0 vector)
}


// Timer0 A0 interrupt service routine.  Generated when TimerA_0 (real-time
// clock) rolls over from 32768 to 0, every second.
#if defined(__TI_COMPILER_VERSION__) || (__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER0_A0_VECTOR
__interrupt void TIMER0_A0_ISR (void)
#elif defined(__GNUC__) && (__MSP430__)
void __attribute__ ((interrupt(TIMER0_A0_VECTOR))) TIMER0_A0_ISR (void)
#else
#error Compiler not found!
#endif
{
    __bic_SR_register_on_exit(LPM3_bits);   // Exit LPM
}

/*
 * ======== UNMI_ISR ========
 */
#if defined(__TI_COMPILER_VERSION__) || (__IAR_SYSTEMS_ICC__)
#pragma vector = UNMI_VECTOR
__interrupt void UNMI_ISR (void)
#elif defined(__GNUC__) && (__MSP430__)
void __attribute__ ((interrupt(UNMI_VECTOR))) UNMI_ISR (void)
#else
#error Compiler not found!
#endif
{
        switch (__even_in_range(SYSUNIV, SYSUNIV_BUSIFG )) {
        case SYSUNIV_NONE:
                __no_operation();
                break;
        case SYSUNIV_NMIIFG:
                __no_operation();
                break;
        case SYSUNIV_OFIFG:
                UCS_clearFaultFlag(UCS_XT2OFFG);
                UCS_clearFaultFlag(UCS_DCOFFG);
                SFR_clearInterrupt(SFR_OSCILLATOR_FAULT_INTERRUPT);
                break;
        case SYSUNIV_ACCVIFG:
                __no_operation();
                break;
        case SYSUNIV_BUSIFG:
                // If the CPU accesses USB memory while the USB module is
                // suspended, a "bus error" can occur.  This generates an NMI.  If
                // USB is automatically disconnecting in your software, set a
                // breakpoint here and see if execution hits it.  See the
                // Programmer's Guide for more information.
                SYSBERRIV = 0;  // Clear bus error flag
                USB_disable();  // Disable
        }
}



//Released_Version_5_00_01
