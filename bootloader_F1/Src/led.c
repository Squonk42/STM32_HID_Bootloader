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
 * @file led.c
 *
 * @brief File that contains the LED and GPIO functions.
 */

#include <stm32f10x.h>
#include "config.h"
#include "led.h"

/**
 * @brief Initialize the LED and GPIO pins.
 */
void pins_init(void)
{
	/* Initialize the required GPIO clocks */
	SET_BIT(RCC->APB2ENR,
		LED1_CLOCK | DISC_CLOCK | RCC_APB2ENR_IOPBEN);

	/* Initialize the LED1 pin */
	LED1_BIT_0;
	LED1_BIT_1;
	LED1_MODE;

	/* Initialize the DISC pin */
	DISC_BIT_0;
	DISC_BIT_1;
	DISC_MODE;
	DISC_LOW;

	/* Initialize the BOOT1 pin */

	/* PB2 is already in FLOATING mode by default. */
#if defined PB2_PULLDOWN

	SET_BIT(GPIOB->CRL, GPIO_CRL_CNF2_1);
	CLEAR_BIT(GPIOB->ODR, GPIO_ODR_ODR2);

#endif
}
