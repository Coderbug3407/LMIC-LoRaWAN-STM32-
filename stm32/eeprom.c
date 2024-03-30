/*
 * Copyright (c) 2014-2016 IBM Corporation.
 * All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of the <organization> nor the
 *    names of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "hw.h"
#include "../../lmic/lmic.h"
#include "stm32f3xx.h"


static FLASH_EraseInitTypeDef EraseInitStruct;
int current_page =  1;
uint32_t PAGEError = 0;

uint32_t EraseCounter = 0x00, Address = 0x00;//Erase count, write address
void eeprom_init()
{
	EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.PageAddress = FLASH_USER_START_ADDR;
	EraseInitStruct.NbPages     = (FLASH_USER_END_ADDR - FLASH_USER_START_ADDR) / FLASH_PAGE_SIZE;
	HAL_FLASH_Unlock();
	HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError);
	HAL_FLASH_Lock();
}

// write 32-bit word to EEPROM memory
void eeprom_write (uint32_t* addr, uint32_t val) {
	// check previous value
	/*HAL_StatusTypeDef flashstatus = HAL_ERROR;

	if( *addr != val ) {
		// unlock data eeprom memory and registers
		//        FLASH->PEKEYR = 0x89ABCDEF; // FLASH_PEKEY1
		//        FLASH->PEKEYR = 0x02030405; // FLASH_PEKEY2

		HAL_FLASH_Unlock();
		// only auto-erase if neccessary (when content is non-zero)
		//		FLASH->PECR &= ~FLASH_PECR_FTDW; // clear FTDW

		// write value
		while(flashstatus == HAL_ERROR)
		{
			flashstatus = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)&(*addr), val);
			if(flashstatus == HAL_ERROR)
			{

				addr += 1;
				FLASH_PageErase(FLASH_USER_START_ADDR);
			}
			//				FLASH_PageErase(FLASH_USER_START_ADDR);
		}
		//        *addr = val;

		// check for end of programming
		//        while(FLASH->SR & FLASH_SR_BSY); // loop while busy

		// lock data eeprom memory and registers
		//        FLASH->PECR |= FLASH_PECR_PELOCK;
//		HAL_FLASH_Lock();

		// verify value
		while( (*(volatile uint32_t*)addr != val) || ((uint32_t)&(*addr) > FLASH_USER_END_ADDR) ); // halt on mismatch
	}*/

	*addr = val;
}

void eeprom_copy (void* dst, const void* src, uint16_t len) {
	/*	while(((uint32_t)dst & 3) || ((uint32_t)src & 3) || (len & 3)); // halt if not multiples of 4
	uint32_t* d = (uint32_t*)dst;
	uint32_t* s = (uint32_t*)src;
	uint16_t  l = len/4;

	while(l--) {
		eeprom_write(d++, *s++);
	}*/
	memcpy(dst,src,len);
}

void writeToEE(void *data,size_t size)
{
	HAL_StatusTypeDef flashstatus = HAL_ERROR;

	uint32_t* d = (uint32_t*)data;
	uint16_t l = size / 4;
	uint32_t addr = FLASH_USER_START_ADDR;

	eeprom_init();

	HAL_FLASH_Unlock();

	while(l--) {
		flashstatus = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr , *(d++));
		addr+=4;
		if(flashstatus == HAL_ERROR)
		{

		}
	}
	HAL_FLASH_Lock();
}

void readFromEE(void* data, uint8_t size)
{
	memcpy(data,FLASH_USER_START_ADDR,size);
}
