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
 *
 * Split from hid.c by Michel Stempin <michel.stempin@wanadoo.fr>
 */

#ifndef FLASH_H_
#define FLASH_H_

#include <stdbool.h>

/* Actual reset handler in force */
#define RESET_HANDLER		(*(uint32_t *) 0x4)

/* Initial stack pointer index in vector table */
#define INITIAL_MSP		0

/* Initial program counter index in vector table */
#define INITIAL_RESET_HANDLER	1

/* Reset handler index in vector table*/
#define USER_RESET_HANDLER	7

/**
 * @brief Check if the user code is valid.
 *
 * The check consists in making sure the user stack pointer in the
 * Vector Table points to somewhere in RAM.
 *
 * @param[in] user_address
 *   The user code start address.
 *
 * @return true if the user code is valid, false otherwise.
 */
static inline bool check_user_code(uint32_t user_address)
{
	uint32_t sp = *(volatile uint32_t *) user_address;

	/* Check if the stack pointer in the vector table points
	   somewhere in SRAM */
	return ((sp & 0x2FFE0000) == SRAM_BASE) ? true : false;
}

/* Function Prototypes */
void FLASH_WritePage(uint16_t *page, uint16_t *data, uint16_t size);

#endif /* FLASH_H_ */
