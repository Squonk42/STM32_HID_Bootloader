/*
 * STM32 HID Bootloader - USB HID bootloader for STM32F10X
 * Copyright (c) 2018 Bruno Freitas - bruno@brunofreitas.com
 *
 * Modified February 2019
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
 */

#ifndef GPIO_H_
#define GPIO_H_

/** Index in the Vector Table to store GPIO definition for the BOOT pin */
#define BOOT_PIN		0x08

/** Index in the Vector Table to store GPIO definition for the LED pin */
#define LED_PIN			0x09

/** Index in the Vector Table to store GPIO definition for the DISC pin */
#define DISC_PIN		0x0A

/** GPIO Port A index */
#define PA			0

/** GPIO Port B index */
#define PB			1

/** GPIO Port C index */
#define PC			2

/** GPIO Port D index */
#define PD			3

/** GPIO Port E index */
#define PE			4

/** GPIO Port F index */
#define PF			5

/** GPIO Port G index */
#define PG			6

/** Active-low GPIO flag */
#define ACTIVE_LOW		0

/** Active-high GPIO flag */
#define ACTIVE_HIGH		1

/** GPIO Input mode */
#define GPIO_MODE_INPUT		0x0

/** GPIO Output mode @ 10 MHz */
#define GPIO_MODE_OUTPUT_10_MHZ	0x1

/** GPIO Output mode @ 2 MHz */
#define GPIO_MODE_OUTPUT_2_MHZ	0x2

/** GPIO Output mode @ 50 MHz */
#define GPIO_MODE_OUTPUT_50_MHZ	0x3

/** GPIO Analog Input configuration */
#define GPIO_CNF_ANALOG		0x0

/** GPIO Floating Input configuration */
#define GPIO_CNF_FLOATING	0x1

/** GPIO Output with internal Pull-Down resistor configuration */
#define GPIO_CNF_PULL_DOWN	0x2

/** GPIO Output with internal Pull-Up resistor configuration */
#define GPIO_CNF_PULL_UP	0x2

/** GPIO Push-Pull Output configuration */
#define GPIO_CNF_PUSH_PULL	0x0

/** GPIO Open-Drain Output configuration */
#define GPIO_CNF_OPEN_DRAIN	0x1

/** GPIO Alternate Function Push-Pull Output configuration */
#define GPIO_CNF_ALT_PUSH_PULL	0x2

/** GPIO Alternate Function Open-Drain Output configuration */
#define GPIO_CNF_ALT_OPEN_DRAIN	0x3

/** Macro to pack GPIO Mode + Configuration */
#define GPIO_MODE_CNF(m, c)	(((c) << 2) | ((m) & 0x3))

/** GPIO Mode and Configuration for Analog Input */
#define ANALOG_INPUT \
	GPIO_MODE_CNF(GPIO_MODE_INPUT, GPIO_CNF_ANALOG)

/** GPIO Mode and Configuration for Floating Input */
#define FLOATING_INPUT \
	GPIO_MODE_CNF(GPIO_MODE_INPUT, GPIO_CNF_FLOATING)

/** GPIO Mode and Configuration for Input with internal Pull-Up/Down resistor */
#define PULL_UP_DOWN_INPUT \
	GPIO_MODE_CNF(GPIO_MODE_INPUT, GPIO_CNF_PULL_UPDOWN)

/** GPIO Mode and Configuration for Push-Pull Output @ 10 MHz */
#define PUSH_PULL_OUTPUT_10_MHZ \
	GPIO_MODE_CNF(GPIO_MODE_OUTPUT_10_MHZ, GPIO_CNF_PUSH_PULL)

/** GPIO Mode and Configuration for Open-Drain Output @ 10 MHz */
#define OPEN_DRAIN_OUTPUT_10_MHZ \
	GPIO_MODE_CNF(GPIO_MODE_OUTPUT_10_MHZ, GPIO_CNF_OPEN_DRAIN)

/** GPIO Mode and Configuration for Alternate Function Push-Pull Output @ 10 MHz
 */
#define ALT_PUSH_PULL_OUTPUT_10_MHZ \
	GPIO_MODE_CNF(GPIO_MODE_OUTPUT_10_MHZ, GPIO_CNF_ALTFN_PUSH_PULL)

/** GPIO Mode and Configuration for Alternate Function Open-Drain Output @ 10 MHz
 */
#define ALT_OPEN_DRAIN_OUTPUT_10_MHZ \
	GPIO_MODE_CNF(GPIO_MODE_OUTPUT_10_MHZ, GPIO_CNF_ALTFN_OPEN_DRAIN)

/** GPIO Mode and Configuration for Push-Pull Output @ 2 MHz */
#define PUSH_PULL_OUTPUT_2_MHZ \
	GPIO_MODE_CNF(GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_PUSH_PULL)

/** GPIO Mode and Configuration for Open-Drain Output @ 2 MHz */
#define OPEN_DRAIN_OUTPUT_2_MHZ \
	GPIO_MODE_CNF(GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OPEN_DRAIN)

/** GPIO Mode and Configuration for Alternate Function Push-Pull Output @ 2 MHz
 */
#define ALT_PUSH_PULL_OUTPUT_2_MHZ \
	GPIO_MODE_CNF(GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_ALTFN_PUSH_PULL)

/** GPIO Mode and Configuration for Alternate Function Open-Drain Output @ 2 MHz
 */
#define ALT_OPEN_DRAIN_OUTPUT_2_MHZ \
	GPIO_MODE_CNF(GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_ALTFN_OPEN_DRAIN)

/** GPIO Mode and Configuration for Push-Pull Output @ 50 MHz */
#define PUSH_PULL_OUTPUT_50_MHZ \
	GPIO_MODE_CNF(GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_PUSH_PULL)

/** GPIO Mode and Configuration for Open-Drain Output @ 50 MHz */
#define OPEN_DRAIN_OUTPUT_50_MHZ \
	GPIO_MODE_CNF(GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OPEN_DRAIN)

/** GPIO Mode and Configuration for Alternate Function Push-Pull Output @ 50 MHz
 */
#define ALT_PUSH_PULL_OUTPUT_50_MHZ \
	GPIO_MODE_CNF(GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_ALT_PUSH_PULL)

/** GPIO Mode and Configuration for Alternate Function Open-Drain Output @ 50 MHz
 */
#define ALT_OPEN_DRAIN_OUTPUT_50_MHZ \
	GPIO_MODE_CNF(GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_ALT_OPEN_DRAIN)

/** Pin definition macro
 *
 * A pin contains all the required information to define a GPIO pin in
 * a 32 bit unsigned value:
 *  - Clock Mask = Port index * 4 for Clock enable/disable (c)
 *  - Port index + 2 (P)
 *  - Bit index (b)
 *  - Packed Mode and Configuration (m)
 *  - Polarity (p)
 *  - Initial state (i)
 *  - GPIO bank Base address
 *
 * Base |   Clock  |   Base  | Port| I | P | Mode | Bit
 * -----|----------|---------|-----|---|---|------|------
 * 0100 | cccccccc | 0001000 | PPP | i | p | mmmm | bbbb
 */
#define PIN_DEFINE(port, bit, mode, polarity, init_state) \
	((((1 << (((port) & 0x7) + 2))) << 20) |		  \
	 (((mode) & 0xf) << 4) |				  \
	 (((init_state) ? 1 : 0) << 9) |	\
	 (((polarity) ? 1 : 0) << 8) |		\
	 (((bit) & 0xf) << 0) |			\
	 (GPIOA_BASE + ((port) & 0x7) * 0x400))

/** Unused pin definition */
#define PIN_UNUSED		0

/** Action to clear a GPIO pin */
#define GPIO_CLEAR		0

/** Action to set a GPIO pin */
#define GPIO_SET		1

/** Action to configure a GPIO pin */
#define GPIO_CONFIG		2

/** Action to get a GPIO pin state */
#define GPIO_GET		3

/** Macro to clear a GPIO pin */
#define GPIO_Clear(pn)		GPIO_Action(pn, GPIO_CLEAR)

/** Macr to set a GPIO pin */
#define GPIO_Set(pn)		GPIO_Action(pn, GPIO_SET)

/** Macro to configure a GPIO pin */
#define GPIO_Configure(pn)	GPIO_Action(pn, GPIO_CONFIG)

/** Macro to get a GPIO pin state */
#define GPIO_Get(pn)		GPIO_Action(pn, GPIO_GET)

/* Function Prototypes */
void GPIO_Init(void);
bool GPIO_Action(uint8_t pin_number, uint8_t action);

#endif /* GPIO_H_ */
