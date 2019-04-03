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

/* Actual reset handler in force */
#define RESET_HANDLER		(*(uint32_t *) 0x4)

/* Initial stack pointer index in vector table */
#define INITIAL_MSP			0

/* Initial program counter index in vector table */
#define INITIAL_RESET_HANDLER		1

/* Reset handler index in vector table*/
#define USER_RESET_HANDLER		7


void FLASH_WritePage(uint16_t *page, uint16_t *data, uint16_t size);

#endif /* FLASH_H_ */
