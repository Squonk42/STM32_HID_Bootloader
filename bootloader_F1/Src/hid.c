/*
 * STM32 HID Bootloader - USB HID bootloader for STM32F10X
 * Copyright (c) 2018 Bruno Freitas - bruno@brunofreitas.com
 *
 * Modified January 2019
 *	by Michel Stempin <michel.stempin@wanadoo.fr>
 *	Cleanup and optimizations
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

/**
 * @file hid.c
 *
 * @brief File containing the HID USB handling.
 */

#include <stm32f10x.h>
#include <string.h>
#include <stdbool.h>
#include "main.h"
#include "usb.h"
#include "hid.h"
#include "gpio.h"
#include "flash.h"

/** This should be <= MAX_EP_NUM defined in usb.h */
#define EP_NUM 			2

/** Maximum packet size */
#define MAX_PACKET_SIZE		8

/** Command size */
#define COMMAND_SIZE		64

/** Buffer table offsset in PMA memory */
#define BTABLE_OFFSET		(0x00)

/** EP0 RX buffer base address  */
#define ENDP0_RXADDR		(0x18)

/** EP0 TX buffer base address  */
#define ENDP0_TXADDR		(0x58)

/** EP1 TX buffer base address */
#define ENDP1_TXADDR		(0x100)

/** Flash page size */
#define PAGE_SIZE		1024

/** Upload started flag */
volatile bool UploadStarted;

/** Upload finished flag */
volatile bool UploadFinished;

/** Flash page buffer */
static uint8_t PageData[PAGE_SIZE];

/** Current page number (starts right after bootloader's end) */
static volatile uint8_t CurrentPage;

/** Byte offset in flash page */
static volatile uint16_t CurrentPageOffset;

/** Sent command (Received command is the same minus last byte) */
static uint8_t Command[] = {'B', 'T', 'L', 'D', 'C', 'M', 'D', 2};

/** USB device descriptor */
static uint8_t USB_DeviceDescriptor[] = {
	0x12,			// bLength
	0x01,			// bDescriptorType (Device)
	0x10, 0x01,		// bcdUSB 1.10
	0x00,			// bDeviceClass (Use class information in the Interface Descriptors)
	0x00,			// bDeviceSubClass
	0x00,			// bDeviceProtocol
	MAX_PACKET_SIZE,	// bMaxPacketSize0 8
	0x09, 0x12,		// idVendor 0x1209
	0xBA, 0xBE,		// idProduct 0xBEBA
	0x00, 0x04,		// bcdDevice 4.00
	0x01,			// iManufacturer (String Index)
	0x02,			// iProduct (String Index)
	0x00,			// iSerialNumber (String Index)
	0x01 			// bNumConfigurations 1
};

/** USB configuration descriptor */
static uint8_t USB_ConfigurationDescriptor[] = {
	0x09,			// bLength
	0x02,			// bDescriptorType (Configuration)
	0x22, 0x00,		// wTotalLength 34
	0x01,			// bNumInterfaces 1
	0x01,			// bConfigurationValue
	0x00,			// iConfiguration (String Index)
	0xC0,			// bmAttributes Self Powered
	0x32,			// bMaxPower 100mA

	0x09,			// bLength
	0x04,			// bDescriptorType (Interface)
	0x00,			// bInterfaceNumber 0
	0x00,			// bAlternateSetting
	0x01,			// bNumEndpoints 1
	0x03,			// bInterfaceClass
	0x00,			// bInterfaceSubClass
	0x00,			// bInterfaceProtocol
	0x00,			// iInterface (String Index)

	0x09,			// bLength
	0x21,			// bDescriptorType (HID)
	0x11, 0x01,		// bcdHID 1.11
	0x00,			// bCountryCode
	0x01,			// bNumDescriptors
	0x22,			// bDescriptorType[0] (HID)
	0x20, 0x00,		// wDescriptorLength[0] 32

	0x07,			// bLength
	0x05,			// bDescriptorType (Endpoint)
	0x81,			// bEndpointAddress (IN/D2H)
	0x03,			// bmAttributes (Interrupt)
	MAX_PACKET_SIZE, 0x00,	// wMaxPacketSize 8
	0x05 			// bInterval 5 (2^(5-1)=16 micro-frames)
};

/** USB HID report descriptor */
static uint8_t USB_ReportDescriptor[32] = {
	0x06, 0x00, 0xFF,	// Usage Page (Vendor Defined 0xFF00)
	0x09, 0x01,		// Usage (0x01)
	0xA1, 0x01,		// Collection (Application)
	0x09, 0x02,		// 	Usage (0x02)
	0x15, 0x00,		// 	Logical Minimum (0)
	0x25, 0xFF,		// 	Logical Maximum (255)
	0x75, 0x08,		// 	Report Size (8)
	0x95, 0x08,		// 	Report Count (8)
	0x81, 0x02,		// 	Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
	0x09, 0x03,		// 	Usage (0x03)
	0x15, 0x00,		// 	Logical Minimum (0)
	0x25, 0xFF,		// 	Logical Maximum (255)
	0x75, 0x08,		// 	Report Size (8)
	0x95, 0x40,		// 	Report Count (64)
	0x91, 0x02,		// 	Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
	0xC0 			// End Collection
};

/** USB Language ID String Descriptor */
static uint8_t USB_LangIDStringDescriptor[] = {
	0x04,			// bLength
	0x03,			// bDescriptorType (String)
	0x09, 0x04		// English (United States)
};

/** USB Vendor String Descriptor */
static uint8_t USB_VendorStringDescriptor[] = {
	0x14,			// bLength
	0x03,			// bDescriptorType (String)
	'S', 0, 'T', 0, 'M', 0, '3', 0, '2', 0, 'D', 0, 'u', 0, 'i', 0, 'n', 0,
	'o', 0
};

/** USB Product String Descriptor */
static uint8_t USB_ProductStringDescriptor[] = {
	0x12,			// bLength
	0x03,			// bDescriptorType (String)
	'H', 0, 'I', 0,	'D', 0, ' ', 0, 'F', 0, '1', 0, 'B', 0, 'L', 0
};

/** USB String Descriptors */
/* Force this array to be in the .data initialiazed data section in
 *  RAM, otherwise the compiler optimizes it to be in read-only data
 *  section in Flash memory, and access is then performed using
 *  absolute addresses, making the bootloader non-rellocatable
 */
__attribute__ ((section(".data")))
static uint8_t *USB_StringDescriptors[] = {
	USB_LangIDStringDescriptor,
	USB_VendorStringDescriptor,
	USB_ProductStringDescriptor,
};

/** USB device status */
uint16_t DeviceStatus = 0;

/**
 * @brief Get the requested USB descriptor.
 *
 * @param[in] setup_packet
 *  Pointer to the received setup packet.
 */
static void HIDUSB_GetDescriptor(USB_SetupPacket *setup_packet)
{
	uint16_t *descriptor = 0;
	uint16_t length = 0;
	unsigned int i = setup_packet->wValue.L;

	switch (setup_packet->wValue.H) {
	case USB_DEVICE_DESC_TYPE:
		descriptor =  (uint16_t *) USB_DeviceDescriptor;
		length = sizeof (USB_DeviceDescriptor);
		break;

	case USB_CFG_DESC_TYPE:
		descriptor = (uint16_t *) USB_ConfigurationDescriptor;
		length =  sizeof (USB_ConfigurationDescriptor);
		break;

	case USB_REPORT_DESC_TYPE:
		descriptor = (uint16_t *) USB_ReportDescriptor;
		length =  sizeof (USB_ReportDescriptor);
		break;

	case USB_STR_DESC_TYPE:
		if (i < sizeof (USB_StringDescriptors) /
			sizeof (USB_StringDescriptors[0])) {
			descriptor = (uint16_t *) USB_StringDescriptors[i];
			length = *(uint8_t *) descriptor;
		}
		break;

	default:
		break;
	}
	if (length > setup_packet->wLength) {
		length = setup_packet->wLength;
	}
	USB_SendData(0, descriptor, length);
}

/**
 * @brief Check if an USB HID packet contains a bootlaoder command.
 *
 * @return the bootloader command index or 0xff in no command found.
 */
static uint8_t HIDUSB_PacketIsCommand(void)
{
	size_t i;

	for (i = 0; i < sizeof (Command) - 1; i++) {
		if (PageData[i] != Command[i]) {
			return 0xff;
		}
 	}
	for (i++; i < COMMAND_SIZE; i++) {
		if (PageData[i]) {
			return 0xff;
		}
 	}
	return PageData[sizeof (Command) - 1];
}

/**
 * @brief USB HID data handler function.
 *
 * This function handles a USB HID data packet.
 *
 * @param[in] data
 *  Pointer to the received data buffer.
 */
static void HIDUSB_HandleData(uint8_t *data)
{
	memcpy(PageData + CurrentPageOffset, data, MAX_PACKET_SIZE);
	CurrentPageOffset += MAX_PACKET_SIZE;
	if (CurrentPageOffset == COMMAND_SIZE) {
		switch (HIDUSB_PacketIsCommand()) {

		case 0x00:

			/* Reset Page Command */
			UploadStarted = true;
			CurrentPage = 0;
			CurrentPageOffset = 0;
		break;

		case 0x01:

			/* Reboot MCU Command */
			UploadFinished = true;
		break;

		default:
			break;
		}
	} else if (CurrentPageOffset >= PAGE_SIZE) {
		uint16_t *page_address = (uint16_t * ) (FLASH_BASE +
			(CurrentPage * PAGE_SIZE));
		if (page_address >= (uint16_t *) (RESET_HANDLER - 1)) {

			/* Do not overwrite the bootloader */
			return;
		} else if (CurrentPage == 0) {
			if (check_user_code((uint32_t) PageData) == false) {
				return;
			}

			/* Patch the user Vector Table */
			uint32_t *patched_vector_table =
				(uint32_t *) PageData;

			/* Make a backup of the user reset handler */
			patched_vector_table[USER_RESET_HANDLER] =
				patched_vector_table[INITIAL_RESET_HANDLER];

			/* Replace it with the bootloader reset handler */
			patched_vector_table[INITIAL_RESET_HANDLER] =
				RESET_HANDLER;

			/* Copy the pin configuration data */
			for (int pin = BOOT_PIN; pin <= DISC_PIN; pin++) {
				patched_vector_table[pin] = VectorTable[pin];
			}
		}
		GPIO_Set(LED_PIN);
		FLASH_WritePage(page_address, (uint16_t *) PageData,
			PAGE_SIZE / sizeof (uint16_t));
		CurrentPage++;
		CurrentPageOffset = 0;
		USB_SendData(ENDP1, (uint16_t *) Command,
			sizeof (Command));
		GPIO_Clear(LED_PIN);
	}
}

/**
 * @brief Perform an USB Reset.
 *
 * This functions initializes the required endpoints and enables the
 * USB peripheral.
 */
void USB_Reset(void)
{

	/* Initialize Flash Page Settings */
	CurrentPage = 0;
	CurrentPageOffset = 0;

	/* Set buffer descriptor table offset in PMA memory */
	WRITE_REG(*BTABLE, BTABLE_OFFSET);

	/* Initialize Endpoint 0 */
	TOGGLE_REG(EP0REG[ENDP0],
		   EP_DTOG_RX | EP_T_FIELD | EP_KIND | EP_DTOG_TX | EPTX_STAT | EPADDR_FIELD,
		   0 | EP_CONTROL | 0,
		   EP_RX_VALID);

	/* Set transmission buffer address for endpoint 0 in buffer descriptor table */
	BTABLE_ADDR_FROM_OFFSET(ENDP0, BTABLE_OFFSET)[USB_ADDRn_TX] = ENDP0_TXADDR;

	/* Set reception buffer address for endpoint 0 in buffer descriptor table */
	BTABLE_ADDR_FROM_OFFSET(ENDP0, BTABLE_OFFSET)[USB_ADDRn_RX] = ENDP0_RXADDR;
	RxTxBuffer[0].MaxPacketSize = MAX_PACKET_SIZE;

	/* Initialize Endpoint 1 */
	TOGGLE_REG(EP0REG[ENDP1],
		   EP_DTOG_RX | EP_T_FIELD | EP_KIND | EP_DTOG_TX | EPADDR_FIELD,
		   1 | EP_INTERRUPT | 0,
		   EP_RX_DIS | EP_TX_NAK);

	/* Set transmission buffer address for endpoint 1 in buffer descriptor table */
	BTABLE_ADDR_FROM_OFFSET(ENDP1, BTABLE_OFFSET)[USB_ADDRn_TX] = ENDP1_TXADDR;

	/* Set transmission byte count for endpoint 1 in buffer descriptor table */
	BTABLE_ADDR_FROM_OFFSET(ENDP1, BTABLE_OFFSET)[USB_COUNTn_TX] = MAX_PACKET_SIZE;
	RxTxBuffer[1].MaxPacketSize = MAX_PACKET_SIZE;

	/* Clear device address and enable USB function */
	WRITE_REG(*DADDR, DADDR_EF | 0);
}

/**
 * @brief Handle an USB endpoint data.
 *
 * @param[in] status
 *  Copy of the USB interrupt status register.
 */
void USB_EPHandler(uint16_t status)
{
	uint8_t endpoint = READ_BIT(status, USB_ISTR_EP_ID);
	uint16_t endpoint_status = EP0REG[endpoint];
	USB_SetupPacket *setup_packet;

	/* OUT and SETUP packets (data reception) */
	if (READ_BIT(endpoint_status, EP_CTR_RX)) {

		/* Copy from packet area to user buffer */
		USB_PMA2Buffer(endpoint);
		if (endpoint == 0) {

			/* If control endpoint */
			if (READ_BIT(endpoint_status, USB_EP0R_SETUP)) {
				setup_packet = (USB_SetupPacket *) RxTxBuffer[endpoint].RXB;
				switch (setup_packet->bRequest) {

				case USB_REQUEST_SET_ADDRESS:
					DeviceAddress = setup_packet->wValue.L;
					USB_SendData(0, 0, 0);
					break;

				case USB_REQUEST_GET_DESCRIPTOR:
					HIDUSB_GetDescriptor(setup_packet);
					break;

				case USB_REQUEST_GET_STATUS:
					USB_SendData(0, (uint16_t *) &DeviceStatus, 2);
					break;

				case USB_REQUEST_GET_CONFIGURATION:
					USB_SendData(0, (uint16_t *) &DeviceConfigured, 1);
					break;

				case USB_REQUEST_SET_CONFIGURATION:
					DeviceConfigured = 1;
					USB_SendData(0, 0, 0);
					break;

				case USB_REQUEST_GET_INTERFACE:
					USB_SendData(0, 0, 0);
					break;

				default:
					USB_SendData(0, 0, 0);
					SET_TX_STATUS(ENDP0, EP_TX_STALL);
					break;
				}
			} else if (RxTxBuffer[endpoint].RXL) {

				/* OUT packet */
				HIDUSB_HandleData((uint8_t *) RxTxBuffer[endpoint].RXB);
			}

		}
		SET_RX_STATUS(endpoint, EP_RX_VALID);
	}
	if (READ_BIT(endpoint_status, EP_CTR_TX)) {

		/* Something transmitted */
		if (DeviceAddress) {

			/* Set device address and enable function */
			WRITE_REG(*DADDR, DADDR_EF | DeviceAddress);
			DeviceAddress = 0;
		}
		USB_Buffer2PMA(endpoint);
		SET_TX_STATUS(endpoint, (endpoint == ENDP1) ? EP_TX_NAK : EP_TX_VALID);
	}
}
