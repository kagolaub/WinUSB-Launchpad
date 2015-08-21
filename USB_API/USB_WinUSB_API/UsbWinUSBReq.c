/*
 * WinUSBReq.cpp
 *
 *  Created on: Aug 5, 2015
 *      Author: HS student
 */
#include "../USB_Common/device.h"
#include "../USB_Common/defMSP430USB.h"
#include "../USB_Common/usb.h"                              //USB-specific Data Structures
#include <USB_API/USB_WinUSB_API/UsbWinUSBReq.h>
#include <descriptors.h>

#ifdef _WINUSB_

void usbClearOEP0ByteCount (void);
void usbSendDataPacketOnEP0 (const uint8_t* pbBuffer);
void usbReceiveDataPacketOnEP0 (uint8_t* pbBuffer);

extern const uint16_t report_desc_size[WINUSB_NUM_INTERFACES];
extern const uint8_t* report_desc[WINUSB_NUM_INTERFACES];         //KLQ
extern uint16_t wUsbHidEventMask;

#ifdef NON_COMPOSITE_MULTIPLE_INTERFACES
extern const struct abromConfigurationDescriptorGroupWinUSB abromConfigurationDescriptorGroupWinUSB;
#endif

uint8_t usbGetMSOSStringDescriptor (void) {
    uint16_t bIndex = 0x00;
    usbClearOEP0ByteCount();    //for status stage
    wBytesRemainingOnIEP0 = abromMSDescriptor[bIndex];
    usbSendDataPacketOnEP0((uint8_t*)&abromMSDescriptor[bIndex]);
    return (FALSE);
}

uint8_t usbGetMSFeatureDescriptor (void) {
	uint16_t bIndex = 0x00;
	usbClearOEP0ByteCount();    //for status stage
	bIndex += abromMSDescriptor[bIndex];
	wBytesRemainingOnIEP0 = abromMSDescriptor[bIndex];
	usbSendDataPacketOnEP0((uint8_t*)&abromMSDescriptor[bIndex]);
	return (FALSE);
}

#endif //_WINUSB_
