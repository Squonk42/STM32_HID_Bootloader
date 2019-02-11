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
 * @file usb.c
 *
 * @brief File containing the generic USB low-level functions.
 */

#include <stm32f10x.h>
#include <stdlib.h>
#include <stdbool.h>

#include "usb.h"
#include "hid.h"

/** USB interrupt enable mask */
#define CNTR_MASK	(CNTR_RESETM | CNTR_SUSPM | CNTR_WKUPM)

/** USB interrupt flag mask */
#define ISTR_MASK	(ISTR_CTR | ISTR_RESET | ISTR_SUSP | ISTR_WKUP)

/** USB RX/TX buffer */
USB_RxTxBuf_t RxTxBuffer[MAX_EP_NUM];

/** USB device address that will be set up by host */
volatile uint8_t DeviceAddress;

/** USB configuration flag */
volatile uint16_t DeviceConfigured;

/**
 * @brief Transfer data from USB PMA memory to buffer.
 *
 * The PMA (Packet Memory Area) is a dual-port access memory that is
 * access by both the processor as 32-bit wide values and by the USB
 * peripheral device as 16-bit wide values.
 *
 * @param[in] endpoint
 *  The data endpoint to read from.
 */
void USB_PMA2Buffer(uint8_t endpoint)
{
	volatile uint32_t *btable = BTABLE_ADDR(endpoint);
	uint32_t count = RxTxBuffer[endpoint].RXL = btable[USB_COUNTn_RX] & 0x3ff;
	uint32_t *address = (uint32_t *) (PMAAddr + btable[USB_ADDRn_RX] * 2);
	uint16_t *destination = (uint16_t *) RxTxBuffer[endpoint].RXB;

	for (uint32_t i = 0; i < count; i++) {
		*destination++ = *address++;
	}
}

/**
 * @brief Transfer data from buffer to USB PMA memory.
 *
 * The PMA (Packet Memory Area) is a dual-port access memory that is
 * access by both the processor as 32-bit wide values and by the USB
 * peripheral device as 16-bit wide values.
 *
 * @param[in] endpoint
 *  The data endpoint to write to.
 */
void USB_Buffer2PMA(uint8_t endpoint)
{
	volatile uint32_t *btable = BTABLE_ADDR(endpoint);
	uint32_t count = RxTxBuffer[endpoint].TXL <= RxTxBuffer[endpoint].MaxPacketSize ?
		RxTxBuffer[endpoint].TXL : RxTxBuffer[endpoint].MaxPacketSize;
	uint16_t *address = RxTxBuffer[endpoint].TXB;
	uint32_t *destination = (uint32_t *) (PMAAddr + btable[USB_ADDRn_TX] * 2);

	/* Set transmission byte count in buffer descriptor table */
	btable[USB_COUNTn_TX] = count;
	for (uint32_t i = (count + 1) / 2; i; i--) {
		*destination++ = *address++;
	}
	RxTxBuffer[endpoint].TXL -= count;
	RxTxBuffer[endpoint].TXB = address;
}

/**
 * @brief Send a data buffer to an USB endpoint.
 *
 * @param[in] endpoint
 *  The data endpoint to write to.
 *
 * @param[in] data
 *  Pointer to the 16-bit aligned data buffer.
 *
 * @param[in] length
 * Length of the data buffer in bytes.
 */
void USB_SendData(uint8_t endpoint, uint16_t *data, uint16_t length)
{
	if (endpoint > 0 && !DeviceConfigured) {
		return;
	}
	RxTxBuffer[endpoint].TXL = length;
	RxTxBuffer[endpoint].TXB = data;
	USB_Buffer2PMA(endpoint);
	SET_TX_STATUS(endpoint, EP_TX_VALID);
}

/**
 * @brief Shutdown the USB peripheral.
 *
 * The USB peripheral is deconfigured, its interrupts and clock are
 * disabled, and the USB D+ data line is turned into an open drain
 * output connected to GND to signal an USB disconnection to the host.
 */
void USB_Shutdown(void)
{

	/* Disable USB IRQ */
	//NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
	WRITE_REG(*ISTR, 0);
	DeviceConfigured = 0;

	/* Turn USB Macrocell off */
	WRITE_REG(*CNTR, CNTR_FRES | CNTR_PDWN);

	/* PA12: General purpose output 50 MHz open drain */
	SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPAEN);
	MODIFY_REG(GPIOA->CRH,
		GPIO_CRH_CNF12 | GPIO_CRH_MODE12,
		GPIO_CRH_CNF12_0 | GPIO_CRH_MODE12);

	/* Sinks PA12 to GND */
	WRITE_REG(GPIOA->BRR, GPIO_BRR_BR12);

	/* Disable USB Clock on APB1 */
	CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_USBEN);
}

/**
 * @brief Initialize the USB perpiheral.
 *
 * The RX/TX buffers are cleared, the USB D+ data line is turned into
 * an floating input, the USB peripheral is deconfigured, and its
 * interrupts and clock are enabled.
 */
void USB_Init(void)
{

	/* Reset RX and TX lengths inside RxTxBuffer struct for all
	 * endpoints
	 */
	for (int i = 0; i < MAX_EP_NUM; i++) {
		RxTxBuffer[i].RXL = RxTxBuffer[i].TXL = 0;
	}

	/* PA12: General purpose Input Float */
	SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPAEN);
	MODIFY_REG(GPIOA->CRH,
		GPIO_CRH_CNF12 | GPIO_CRH_MODE12,
		GPIO_CRH_CNF12_0);

	/* USB devices start as not configured */
	DeviceConfigured = 0;

	/* Enable USB clock */
	SET_BIT(RCC->APB1ENR, RCC_APB1ENR_USBEN);

	/* CNTR_FRES = 1, CNTR_PWDN = 0 */
	WRITE_REG(*CNTR, CNTR_FRES);

	/* The following sequence is recommended:
	 * 1- FRES = 0
	 * 2- Wait until RESET flag = 1 (polling)
	 * 3- clear ISTR register
	 */

	/* CNTR_FRES = 0 */
	WRITE_REG(*CNTR, 0);

	/* Wait until RESET flag = 1 (polling) */
	while (!READ_BIT(*ISTR, ISTR_RESET)) {
		;
	}

	/* Clear pending interrupts */
	WRITE_REG(*ISTR, 0);

	/* Set interrupt mask */
	WRITE_REG(*CNTR, CNTR_MASK);
}

/**
 * @brief Main USB event polling function.
 *
 * This function checks for the various USB interrupt causes by
 * checking the USB interrupt status register.
 *
 * Note that this function may be called by manual polling, or
 * automatically if it is set into the corresponding USB interrupt
 * handler in the Vector Table.
 */
void USB_Poll(void)
{
	volatile uint16_t istr;

	while ((istr = READ_REG(*ISTR) & ISTR_MASK) != 0) {

		/* Handle EP data */
		if (READ_BIT(istr, ISTR_CTR)) {

			/* Handle data on EP */
			WRITE_REG(*ISTR, CLR_CTR);
			USB_EPHandler(READ_REG(*ISTR));
		}

		/* Handle Reset */
		if (READ_BIT(istr, ISTR_RESET)) {
			WRITE_REG(*ISTR, CLR_RESET);
			USB_Reset();
		}

		/* Handle Suspend */
		if (READ_BIT(istr, ISTR_SUSP)) {
			WRITE_REG(*ISTR, CLR_SUSP);

			/* If device address is assigned, then reset it */
			if (READ_REG(*DADDR) & USB_DADDR_ADD) {
				WRITE_REG(*DADDR, 0);
				CLEAR_BIT(*CNTR, CNTR_SUSPM);
			}
		}

		/* Handle Wakeup */
		if (READ_BIT(istr, ISTR_WKUP)) {
			WRITE_REG(*ISTR, CLR_WKUP);
		}
	}

	/* Default to clear all interrupt flags */
	WRITE_REG(*ISTR, 0);
}
