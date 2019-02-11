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
* Modified 20 April 2018
*	by Vassilis Serasidis <info@serasidis.gr>
*	This HID bootloader works with STM32F103 + STM32duino + Arduino IDE <http://www.stm32duino.com/>
*
* Modified January 2019
*	by Michel Stempin <michel.stempin@wanadoo.fr>
*	Cleanup and optimizations
*
*/

#include <stm32f10x.h>
#include <stdbool.h>
#include "usb.h"
#include "config.h"
#include "hid.h"
#include "led.h"
#include "flash.h"

/* User Program is at the start of the Flash memory. */
#define USER_PROGRAM			FLASH_BASE

/* Magic word value stored in BACKUP memory to force entering the bootloader */
#define MAGIC_WORD			(('B' << 8) | 'L')

/* End of stack symbol defined by the linker script */
extern char _estack;

/* Start of code text symbol defined by the linker script */
extern uint8_t _stext;

/* Start of initialization data symbol defined by the linker script */
extern uint8_t _sidata;

/* Start of data symbol defined by the linker script */
extern uint8_t _sdata;

/* End of data symbol defined by the linker script */
extern uint8_t _edata;

/* Simple function pointer type to call user program */
typedef void (*funct_ptr)(void);

/* The bootloader entry point function prototype */
void Reset_Handler(void);

/* Minimal initial Flash-based vector table */
uint32_t VectorTable[] __attribute__ ((section(".isr_vector"))) = {

	/* Initial stack pointer (MSP) */
	[INITIAL_MSP] = (uint32_t) &_estack,

	/* Initial program counter (PC): Reset handler */
	[INITIAL_RESET_HANDLER] = (uint32_t) Reset_Handler
};

static void delay(uint32_t timeout)
{
	for (uint32_t i = 0; i < timeout; i++) {
		__NOP();
	}
}

static bool check_flash_complete(void)
{
	static uint32_t counter;

	if (UploadStarted == false) {

		/* Blink the LED nervously */
		if (counter++ & 0x00004000) {
			LED1_ON;
		} else {
			LED1_OFF;
		}
	}
	return UploadFinished;
}

static bool check_user_code(uint32_t user_address)
{
	uint32_t sp = *(volatile uint32_t *) user_address;

	/* Check if the stack pointer in the vector table points
	   somewhere in SRAM */
	return ((sp & 0x2FFE0000) == SRAM_BASE) ? true : false;
}

static bool get_and_clear_magic_word(void)
{

	/* Enable the power and backup interface clocks by setting the
	 * PWREN and BKPEN bits in the RCC_APB1ENR register
	 */
	SET_BIT(RCC->APB1ENR, RCC_APB1ENR_BKPEN | RCC_APB1ENR_PWREN);

	/* Read the magic word */
	uint16_t value = READ_REG(BKP->DR10);

	/* Enable write access to the backup registers and the RTC. */
	SET_BIT(PWR->CR, PWR_CR_DBP);

	/* Clear the magic word */
	WRITE_REG(BKP->DR10, 0x0000);

	/* Disable write access to the backup registers and the RTC. */
	CLEAR_BIT(PWR->CR, PWR_CR_DBP);

	/* Disable the power and backup interface clocks by clearing
	 * the PWREN and BKPEN bits in the RCC_APB1ENR register
	 */
	CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_BKPEN | RCC_APB1ENR_PWREN);
	return value == MAGIC_WORD;
}

static void set_sysclock_to_72_mhz(void)
{

	/* Enable HSE */
	SET_BIT(RCC->CR, RCC_CR_HSEON);

	/* Wait until HSE is ready */
	while (READ_BIT(RCC->CR, RCC_CR_HSERDY) == 0) {
		;
	}

	/* Enable Prefetch Buffer & set Flash access to 2 wait states */
	SET_BIT(FLASH->ACR, FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY_2);

	/* SYSCLK = PCLK2 = HCLK */
	/* PCLK1 = HCLK / 2 */
	/* PLLCLK = HSE * 9 = 72 MHz */
	SET_BIT(RCC->CFGR,
		RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE2_DIV1 | RCC_CFGR_PPRE1_DIV2 |
		RCC_CFGR_PLLSRC_HSE | RCC_CFGR_PLLMULL9);

	/* Enable PLL */
	SET_BIT(RCC->CR, RCC_CR_PLLON);

	/* Wait until PLL is ready */
	while (READ_BIT(RCC->CR, RCC_CR_PLLRDY) == 0) {
		;
	}

	/* Select PLL as system clock source */
	SET_BIT(RCC->CFGR, RCC_CFGR_SW_PLL);

	/* Wait until PLL is used as system clock source */
	while (READ_BIT(RCC->CFGR, RCC_CFGR_SWS_1) == 0) {
		;
	}
}

static void copy_initialized_data(void)
{
	/* The source address of the initialization data in Flash memory is:
	 *  - the offset between the data in Flash and the start of code
	 *  - plus the actual address of the bootloader reset handler
	 *    from  the vector table
	 *  - minus 1 to remove the Thumb flag in the vector LSB bit
	 */
	uint8_t *src = (uint8_t *) (&_sidata - &_stext + RESET_HANDLER) - 1;
	uint8_t *dst = &_sdata;

	/* Copy the initialized data section from Flash to RAM.
	 *
	 * This way, the code has no more position dependence against
	 * the data and becomes rellocatable, since all references
	 * within the code itself are relative.
	 */
	while (dst < &_edata) {
		*dst++ = *src++;
	}
}

__attribute__ ((naked, section(".reset_handler"))) void Reset_Handler(void)
{

	/* Setup the system clock (System clock source, PLL Multiplier
	 * factors, AHB/APBx prescalers and Flash settings)
	 */
	set_sysclock_to_72_mhz();

	/* Initialize GPIOs */
	pins_init();

	/* Copy the initialized data section from Flash to RAM */
	/* Helps to wait a few us so the pull-up settles... */
	copy_initialized_data();

	/* If:
	 *  - a magic word was stored in the battery-backed RAM
	 *    registers from the Arduino IDE or
	 *  - PB2 (BOOT 1 pin) is HIGH or
	 *  - no User Code is uploaded to the MCU
	 * then enter HID bootloader...
	 */
	while (get_and_clear_magic_word() ||
		READ_BIT(GPIOB->IDR, GPIO_IDR_IDR2) ||
		(check_user_code(USER_PROGRAM) == false)) {

		/* Disconnect USB to force a re-enumaration in all
		 * cases
		 */
		USB_Shutdown();
		delay(4000000L);
		USB_Init();
		UploadStarted = UploadFinished = false;
		while (check_flash_complete() == false) {
			USB_Poll();
		};

		/* Reset the USB */
		USB_Shutdown();

		/* Do not exit until we get a valid user program */
	}

	/* Turn GPIO clocks off */
	CLEAR_BIT(RCC->APB2ENR,	LED1_CLOCK | DISC_CLOCK | RCC_APB2ENR_IOPBEN);

	/* Setup the stack pointer to the user-defined one */
	__set_MSP((*(volatile uint32_t *) USER_PROGRAM));

	/* Jump to the user firmware entry point */
	funct_ptr UserProgram =	(funct_ptr) *(((volatile uint32_t *)
		USER_PROGRAM) + USER_RESET_HANDLER);
	UserProgram();

	/* Never reached */
}
