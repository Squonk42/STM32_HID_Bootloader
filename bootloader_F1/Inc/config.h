/*
 * STM32 HID Bootloader - USB HID bootloader for STM32F10X
 * Copyright (c) 2018 Bruno Freitas - bruno@brunofreitas.com
 *
 * Modified April 2019
 *	by Michel Stempin <michel.stempin@wanadoo.fr>
 *	New GPIO config definitions
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

#ifndef __CONFIG_H
#define __CONFIG_H

#if defined TARGET_MAPLE_MINI
	#define PLATFORM_BOOT_PIN	PIN_DEFINE(PB, 2, PULL_UP_DOWN_INPUT, ACTIVE_HIGH, false)
	#define PLATFORM_LED_PIN	PIN_DEFINE(PB, 1, PUSH_PULL_OUTPUT_50_MHZ, ACTIVE_HIGH, false)
	#define PLATFORM_DISC_PIN	PIN_DEFINE(PB, 9, OPEN_DRAIN_OUTPUT_50_MHZ, ACTIVE_HIGH, false)

#elif defined TARGET_GENERIC_F103_PC13
	#define PLATFORM_BOOT_PIN	PIN_DEFINE(PB, 2, FLOATING_INPUT, ACTIVE_HIGH, false)
	#define PLATFORM_LED_PIN	PIN_DEFINE(PC, 13, PUSH_PULL_OUTPUT_50_MHZ, ACTIVE_HIGH, false)
	#define PLATFORM_DISC_PIN	PIN_UNUSED

#elif defined TARGET_GENERIC_F103_PD2
	#define PLATFORM_BOOT_PIN	PIN_DEFINE(PB, 2, FLOATING_INPUT, ACTIVE_HIGH, false)
	#define PLATFORM_LED_PIN	PIN_DEFINE(PD, 2, PUSH_PULL_OUTPUT_50_MHZ, ACTIVE_HIGH, false)
	#define PLATFORM_DISC_PIN	PIN_UNUSED

#elif defined TARGET_GENERIC_F103_PD1
	#define PLATFORM_BOOT_PIN	PIN_DEFINE(PB, 2, FLOATING_INPUT, ACTIVE_HIGH, false)
	#define PLATFORM_LED_PIN	PIN_DEFINE(PD, 1, PUSH_PULL_OUTPUT_50_MHZ, ACTIVE_HIGH, false)
	#define PLATFORM_DISC_PIN	PIN_UNUSED

#elif defined TARGET_GENERIC_F103_PA1
	#define PLATFORM_BOOT_PIN	PIN_DEFINE(PB, 2, FLOATING_INPUT, ACTIVE_HIGH, false)
	#define PLATFORM_LED_PIN	PIN_DEFINE(PA, 1, PUSH_PULL_OUTPUT_50_MHZ, ACTIVE_HIGH, false)
	#define PLATFORM_DISC_PIN	PIN_UNUSED

#elif defined TARGET_GENERIC_F103_PB9
	#define PLATFORM_BOOT_PIN	PIN_DEFINE(PB, 2, FLOATING_INPUT, ACTIVE_HIGH, false)
	#define PLATFORM_LED_PIN	PIN_DEFINE(PB, 9, PUSH_PULL_OUTPUT_50_MHZ, ACTIVE_HIGH, false)
	#define PLATFORM_DISC_PIN	PIN_UNUSED

#elif defined TARGET_GENERIC_F103_PE2
	#define PLATFORM_BOOT_PIN	PIN_DEFINE(PB, 2, FLOATING_INPUT, ACTIVE_HIGH, false)
	#define PLATFORM_LED_PIN	PIN_DEFINE(PE, 2, PUSH_PULL_OUTPUT_50_MHZ, ACTIVE_HIGH, false)
	#define PLATFORM_DISC_PIN	PIN_UNUSED

#elif defined TARGET_GENERIC_F103_PA9
	#define PLATFORM_BOOT_PIN	PIN_DEFINE(PB, 2, FLOATING_INPUT, ACTIVE_HIGH, false)
	#define PLATFORM_LED_PIN	PIN_DEFINE(PA, 9, PUSH_PULL_OUTPUT_50_MHZ, ACTIVE_HIGH, false)
	#define PLATFORM_DISC_PIN	PIN_UNUSED

#elif defined TARGET_GENERIC_F103_PE5
	#define PLATFORM_BOOT_PIN	PIN_DEFINE(PB, 2, FLOATING_INPUT, ACTIVE_HIGH, false)
	#define PLATFORM_LED_PIN	PIN_DEFINE(PE, 5, PUSH_PULL_OUTPUT_50_MHZ, ACTIVE_HIGH, false)
	#define PLATFORM_DISC_PIN	PIN_UNUSED

#elif defined TARGET_GENERIC_F103_PB7
	#define PLATFORM_BOOT_PIN	PIN_DEFINE(PB, 2, FLOATING_INPUT, ACTIVE_HIGH, false)
	#define PLATFORM_LED_PIN	PIN_DEFINE(PB, 7, PUSH_PULL_OUTPUT_50_MHZ, ACTIVE_HIGH, false)
	#define PLATFORM_DISC_PIN	PIN_UNUSED

#elif defined TARGET_GENERIC_F103_PB0
	#define PLATFORM_BOOT_PIN	PIN_DEFINE(PB, 2, FLOATING_INPUT, ACTIVE_HIGH, false)
	#define PLATFORM_LED_PIN	PIN_DEFINE(PB, 0, PUSH_PULL_OUTPUT_50_MHZ, ACTIVE_HIGH, false)
	#define PLATFORM_DISC_PIN	PIN_UNUSED

#elif defined TARGET_GENERIC_F103_PB12
	#define PLATFORM_BOOT_PIN	PIN_DEFINE(PB, 2, FLOATING_INPUT, ACTIVE_HIGH, false)
	#define PLATFORM_LED_PIN	PIN_DEFINE(PB, 12, PUSH_PULL_OUTPUT_50_MHZ, ACTIVE_HIGH, false)
	#define PLATFORM_DISC_PIN	PIN_UNUSED

#else
	#error "No config for this target"
#endif

#endif
