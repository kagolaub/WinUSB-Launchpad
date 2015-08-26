#WinUSBLaunchpad
Firmware for MSP430-F5529 microcontrollers for WinUSB communication, currently implementing interrupt and bulk transfers. Created using a WinUSB API modeled after the default HID/CDC APIs.

Both interrupt transfers and bulk transfers can be used through the configured endpoints set up in the default descriptors. For receiving data, use USBWINUSB_receiveDataInBackground from the WinUSB API, and for sending data use USBWINUSB_sendDataInBackground.
