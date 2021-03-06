/*
 * STM32 HID Bootloader - USB HID bootloader for STM32F10X
 * Copyright (c) 2018 Bruno Freitas - bruno@brunofreitas.com
 *
 * Modified 20 April 2018
 *	by Vassilis Serasidis <info@serasidis.gr>
 *	This HID bootloader works with STM32F103 + STM32duino + Arduino IDE <http://www.stm32duino.com/>
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
 * @file main.c
 *
 * @brief Main file for the USB HID bootloader for the STM32F10X.
 */

#include <stm32f10x.h>
#include <stdbool.h>
#include "usb.h"
#include "hid.h"
#include "gpio.h"
#include "flash.h"
#include "config.h"

/** User Program is at the start of the Flash memory. */
#define USER_PROGRAM			FLASH_BASE

/** Magic word value stored in BACKUP memory to force entering the bootloader */
#define MAGIC_WORD			(('B' << 8) | 'L')

/** All GPIO clocks */
#define GPIO_CLOCKS			RCC_APB2ENR_IOPAEN | \
					RCC_APB2ENR_IOPBEN | \
					RCC_APB2ENR_IOPCEN | \
					RCC_APB2ENR_IOPDEN | \
					RCC_APB2ENR_IOPEEN

/** End of stack symbol defined by the linker script */
extern char _estack;

/** Start of code text symbol defined by the linker script */
extern uint8_t _stext;

/** Start of initialization data symbol defined by the linker script */
extern uint8_t _sidata;

/** Start of data symbol defined by the linker script */
extern uint8_t _sdata;

/** End of data symbol defined by the linker script */
extern uint8_t _edata;

/** Simple function pointer type to call user program */
typedef void (*funct_ptr)(void);

/** The bootloader entry point function prototype */
void Reset_Handler(void);

/** Minimal initial Flash-based vector table */
uint32_t VectorTable[] __attribute__ ((section(".isr_vector"))) = {

	/** Initial stack pointer (MSP) */
	[INITIAL_MSP] = (uint32_t) &_estack,

	/** Initial program counter (PC): Reset handler */
	[INITIAL_RESET_HANDLER] = (uint32_t) Reset_Handler,

	/** BOOT pin definition, default is for BluePill board */
	[BOOT_PIN] = PLATFORM_BOOT_PIN,

	/** LED pin definition, default is for BluePill board */
	[LED_PIN] = PLATFORM_LED_PIN,

	/** DISC pin definition, default is for BluePill board */
	[DISC_PIN] = PLATFORM_DISC_PIN
};

/**
 * @brief Basic delay function.
 */
static void delay(uint32_t timeout)
{
	for (uint32_t i = 0; i < timeout; i++) {
		__NOP();
	}
}

/**
 * @brief Check if the Flash is complete.
 *
 * It also blinks the LED before the upload is started. Blinking is
 * designed to be nervous when running @ 72MHz, so it is not confused
 * with the one from the Blink Arduino sketch.
 *
 * @return true if Flash is complete, false otherwise.
 */
static bool check_flash_complete(void)
{
	static uint32_t counter;

	if (UploadStarted == false) {

		/* Blink the LED nervously */
		if (counter++ & 0x00004000) {
			GPIO_Set(LED_PIN);
		} else {
			GPIO_Clear(LED_PIN);
		}
	}
	return UploadFinished;
}

/**
 * @brief Check and clear the magic word in BACKUP memory.
 *
 * The word in data in register 10 of BACKUP memory is checked against
 * a magic value ('B', 'L') and cleared.
 *
 * @return true if the magic word is found, false otherwise.
 */
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

/**
 * @brief Setup the system clocks to run @ 72MHz.
 *
 * USB requires a system clock of at least 48 MHz based on the
 * High-Speed External crystal (HSE).
 *
 * Based on a common 8 MHz crystal frequency, the system clock is set
 * to the maximum frequency of 72 MHz by using the PLL with a 9x
 * multiplication factor, while the peripheral clocks are set to their
 * maximum operating frequency of 36 MHz for the first peripheral
 * bank, and 72 MHz for the second one.
 *
 * Flash memory access is also optimized by enabling the prefetch
 * buffer and setting its latency according to the system clock.
 */
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

/**
 * @brief Copy the initialized data section from Flash to RAM.
 *
 * This way, the code has no more position dependence against the data
 * and becomes rellocatable, since all references within the code
 * itself are relative.
 *
 * The source address of the initialization data in Flash memory is:
 *  - the offset between the data in Flash and the start of code
 *  - plus the actual address of the bootloader reset handler
 *    from  the vector table
 *  - minus 1 to remove the Thumb flag in the vector LSB bit
 */
static void copy_initialized_data(void)
{
	uint8_t *src = (uint8_t *) (&_sidata - &_stext + RESET_HANDLER) - 1;
	uint8_t *dst = &_sdata;

	while (dst < &_edata) {
		*dst++ = *src++;
	}
}

/**
 * @brief Reset handler.
 *
 * This function is called right after power up by setting its address
 * into the Vector Table reset handler entry.
 *
 * This function is never called from other places, and never returns,
 * so it does not need the usual function prologue/epilogue.
 *
 * It is placed in a special ".reset_handler" section that will be the
 * first code section in Flash memory: this is important as we perform
 * some computation to locate the initialized data section relative to
 * it when copying this section from Flash to RAM.
 */
__attribute__ ((naked, section(".reset_handler"))) void Reset_Handler(void)
{

	/* Setup the system clock (System clock source, PLL Multiplier
	 * factors, AHB/APBx prescalers and Flash settings)
	 */
	set_sysclock_to_72_mhz();

	/* Initialize GPIOs */
	GPIO_Init();

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
		GPIO_Get(BOOT_PIN) ||
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
	CLEAR_BIT(RCC->APB2ENR,	GPIO_CLOCKS);

	/* Setup the stack pointer to the user-defined one */
	__set_MSP((*(volatile uint32_t *) USER_PROGRAM));

	/* Jump to the user firmware entry point */
	funct_ptr UserProgram =	(funct_ptr) *(((volatile uint32_t *)
		USER_PROGRAM) + USER_RESET_HANDLER);
	UserProgram();

	/* Never reached */
}
