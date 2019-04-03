/*
 * STM32 HID Bootloader - USB HID bootloader for STM32F10X
 * Copyright (c) 2018 Bruno Freitas - bruno@brunofreitas.com
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
 */

/**
 * @file gpio.c
 *
 * @brief File that contains the LED and GPIO functions.
 */

#include <stdbool.h>
#include <stm32f10x.h>
#include "main.h"
#include "gpio.h"

/**
 * @brief Perform an action on a GPIO pin.
 *
 * @param[in] pin_number
 *  The GPIO pin number to deal with.
 *
 * @param[in] action
 *  The action to perform.
 */
bool GPIO_Action(uint8_t pin_number, uint8_t action)
{
	uint32_t pin = VectorTable[pin_number];
	uint8_t clock_mask = (pin >> 20) & 0xff;
	uint8_t bit = (pin >> 0) & 0xf;
	uint32_t mode = (pin >> 4) & 0xf;
	bool polarity = ((pin >> 8) & 0x1) ? false : true;
	uint8_t init_state = (pin >> 9) & 0x1;
	GPIO_TypeDef *bank = (GPIO_TypeDef *) (pin & 0x40011c00);
	uint32_t *cr = (uint32_t *) bank + (bit >> 3);
	uint8_t shift = (bit & 0x7) << 2;

	if (pin == PIN_UNUSED) {
		return false;
	}
	if (action == GPIO_CONFIG) {

		/* Initialize the required GPIO clock */
		WRITE_REG(RCC->APB2ENR, clock_mask);

		/* Configure the GPIO pin mode */
		MODIFY_REG(*cr, 0xf << shift, mode << shift);
		action = init_state;
	}
	if (action != GPIO_GET) {

	  /* Set the GPIO pin value according to its polarity */
	  if (action ^ polarity) {
			WRITE_REG(bank->BRR, 1 << bit);
		} else {
			WRITE_REG(bank->BSRR, 1 << bit);
		}
	}

	/* Get the GPIO pin value according to its polarity */
	return READ_BIT(bank->IDR, 1 << bit)  ^ polarity;
}

/**
 * @brief Initialize the GPIO pins.
 */
void GPIO_Init(void)
{
	for (int i = BOOT_PIN; i <= DISC_PIN; i++) {
		GPIO_Configure(i);
	}
}
