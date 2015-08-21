/*
 * WinUSBReq.h
 *
 *  Created on: Aug 5, 2015
 *      Author: HS student
 */

#include <stdint.h>

#ifndef _UsbWinUSBReq_H_
#define _UsbWinUSBReq_H_

#ifdef __cplusplus
extern "C"
{
#endif
/**
 * Return the Microsoft OS String descriptor to tell Windows to look for feature descriptors
 */
uint8_t usbGetMSOSStringDescriptor(void);
/**
 * Return feature descriptors to set up device as a WinUSB device
 */
uint8_t usbGetMSFeatureDescriptor(void);

#ifdef __cplusplus
}
#endif

#endif /* _UsbWinUSBReq_H_ */
