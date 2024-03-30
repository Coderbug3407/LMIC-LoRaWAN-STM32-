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

#include "../../lmic/lmic.h"
#include "hw.h"
#include "stm32f3xx_hal.h"
#include "main.h"
// -----------------------------------------------------------------------------
// I/O

#define myTIMER htim4   //  <--------- change to your setup
#define WakeupTIMER htim7 // <--------- change to your setup

#ifdef CFG_sx1276mb1_board

#define A 0
#define B 1
#define C 2

#if 0
#define NSS_PORT           C // NSS: P, sx1276
#define NSS_PIN            8  // sx1276: PA4

#define TX_PORT            C // TX:  PC15
#define TX_PIN             15

#define RST_PORT           C // RST: PB10
#define RST_PIN            9

#define DIO0_PORT          C // DIO0: PA10, sx1276   (line 1 irq handler)
#define DIO0_PIN           7

#define DIO1_PORT          C // DIO1: PB3, sx1276  (line 10-15 irq handler)
#define DIO1_PIN           0

#define DIO2_PORT          C // DIO2: PB5, sx1276  (line 10-15 irq handler)
#define DIO2_PIN           1

#else	// for stm3 board and lora breakout board
#define NSS_PORT           C // NSS: P, sx1276
#define NSS_PIN            8  // sx1276: PA4

#define TX_PORT            C // TX:  PC15
#define TX_PIN             15

#define RST_PORT           C // RST: PB10
#define RST_PIN            9

#define DIO0_PORT          C // DIO0: PA10, sx1276   (line 1 irq handler)
#define DIO0_PIN           7

#define DIO1_PORT          C // DIO1: PB3, sx1276  (line 10-15 irq handler)
#define DIO1_PIN           0

#define DIO2_PORT          C // DIO2: PB5, sx1276  (line 10-15 irq handler)
#define DIO2_PIN           1
#endif

static const uint8_t outputpins[] = { NSS_PORT, NSS_PIN, TX_PORT, TX_PIN  };
static const uint8_t inputpins[]  = { DIO0_PORT, DIO0_PIN, DIO1_PORT, DIO1_PIN, DIO2_PORT, DIO2_PIN };

#elif CFG_wimod_board

// output lines
#define NSS_PORT           1 // NSS: PB0, sx1272
#define NSS_PIN            0

#define TX_PORT            0 // TX:  PA4
#define TX_PIN             4
#define RX_PORT            2 // RX:  PC13
#define RX_PIN            13
#define RST_PORT           0 // RST: PA2
#define RST_PIN            2

// input lines
#define DIO0_PORT          1 // DIO0: PB1   (line 1 irq handler)
#define DIO0_PIN           1
#define DIO1_PORT          1 // DIO1: PB10  (line 10-15 irq handler)
#define DIO1_PIN          10
#define DIO2_PORT          1 // DIO2: PB11  (line 10-15 irq handler)
#define DIO2_PIN          11

static const uint8_t outputpins[] = { NSS_PORT, NSS_PIN, TX_PORT, TX_PIN, RX_PORT, RX_PIN };
static const uint8_t inputpins[]  = { DIO0_PORT, DIO0_PIN, DIO1_PORT, DIO1_PIN, DIO2_PORT, DIO2_PIN };

#else
#error Missing CFG_sx1276mb1_board/CFG_wimod_board!
#endif

extern SPI_HandleTypeDef hspi2;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim7;
// HAL state
static struct {
	int irqlevel;
	uint32_t ticks;
} HAL;

// -----------------------------------------------------------------------------
// I/O

static void hal_io_init () {
	// clock enable for GPIO ports A,B,C
	//set GPIO in init GPIO
}

// val ==1  => tx 1, rx 0 ; val == 0 => tx 0, rx 1
void hal_pin_rxtx (uint8_t val) {
	/*    ASSERT(val == 1 || val == 0);
#ifndef CFG_sx1276mb1_board
    hw_set_pin(GPIOx(RX_PORT), RX_PIN, ~val);
#endif
    hw_set_pin(GPIOx(TX_PORT), TX_PIN, val);*/
	//endble disable antena switch for rx tx
}


// set radio NSS pin to given value
void hal_pin_nss (uint8_t val) {
	//hw_set_pin(GPIOx(NSS_PORT), NSS_PIN, val);
	HAL_GPIO_WritePin(lora_NSS_PIN_GPIO_Port,lora_NSS_PIN_Pin,val);
}

// set radio RST pin to given value (or keep floating!)
void hal_pin_rst (uint8_t val) {
/*	if(val == 0 || val == 1) { // drive pin
		hw_cfg_pin(GPIOx(RST_PORT), RST_PIN, GPIOCFG_MODE_OUT | GPIOCFG_OSPEED_40MHz | GPIOCFG_OTYPE_PUPD | GPIOCFG_PUPD_PUP);
		hw_set_pin(GPIOx(RST_PORT), RST_PIN, val);
	} else { // keep pin floating
		hw_cfg_pin(GPIOx(RST_PORT), RST_PIN, GPIOCFG_MODE_INP | GPIOCFG_OSPEED_40MHz | GPIOCFG_OTYPE_OPEN);
	}*/
	HAL_GPIO_WritePin(lora_Reset_PIN_GPIO_Port,lora_Reset_PIN_Pin,val);
}

extern void radio_irq_handler(uint8_t dio);

// generic EXTI IRQ handler for all channels
/*
void EXTI_IRQHandler () {
	// DIO 0
	if((EXTI->PR & (1<<DIO0_PIN)) != 0) { // pending
		EXTI->PR = (1<<DIO0_PIN); // clear irq
		// invoke radio handler (on IRQ!)
		radio_irq_handler(0);
	}
	// DIO 1
	if((EXTI->PR & (1<<DIO1_PIN)) != 0) { // pending
		EXTI->PR = (1<<DIO1_PIN); // clear irq
		// invoke radio handler (on IRQ!)
		radio_irq_handler(1);
	}
	// DIO 2
	if((EXTI->PR & (1<<DIO2_PIN)) != 0) { // pending
		EXTI->PR = (1<<DIO2_PIN); // clear irq
		// invoke radio handler (on IRQ!)
		radio_irq_handler(2);
	}

#ifdef CFG_EXTI_IRQ_HANDLER
	// invoke user-defined interrupt handler
	{
		extern void CFG_EXTI_IRQ_HANDLER(void);
		CFG_EXTI_IRQ_HANDLER();
	}
#endif // CFG_EXTI_IRQ_HANDLER
}*/

#if CFG_lmic_clib

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	// DIO 0
	if(GPIO_Pin == lora_DIO0_PIN_Pin)
	{
		// invoke radio handler (on IRQ!)
		radio_irq_handler(0);
	}
	// DIO 1
	if(GPIO_Pin == lora_DIO1_PIN_Pin)
	{ // pending
		// invoke radio handler (on IRQ!)
		radio_irq_handler(1);
	}
	// DIO 2
	if(GPIO_Pin == lora_DIO2_PIN_Pin)
	{ // pending
		// invoke radio handler (on IRQ!)
		radio_irq_handler(2);
	}
}
/*
void EXTI0_IRQHandler () {
	EXTI_IRQHandler();
}

void EXTI1_IRQHandler () {
	EXTI_IRQHandler();
}

void EXTI2_IRQHandler () {
	EXTI_IRQHandler();
}

void EXTI3_IRQHandler () {
	EXTI_IRQHandler();
}

void EXTI4_IRQHandler () {
	EXTI_IRQHandler();
}

void EXTI9_5_IRQHandler () {
	EXTI_IRQHandler();
}

void EXTI15_10_IRQHandler () {
	EXTI_IRQHandler();
}*/
#endif // CFG_lmic_clib

// -----------------------------------------------------------------------------
// SPI

// for sx1272 and 1276

#define SCK_PORT   1 // SCK:  PB13
#define SCK_PIN    13
#define MISO_PORT  1 // MISO: PB14
#define MISO_PIN   14
#define MOSI_PORT  1 // MOSI: PB15
#define MOSI_PIN   15

#define GPIO_AF_SPI1        0x05

static void hal_spi_init () {
	// enable clock for SPI interface 1
	//	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
	//
	//	// use alternate function SPI1 (SCK, MISO, MOSI)
	//	hw_cfg_pin(GPIOx(SCK_PORT),  SCK_PIN,  GPIOCFG_MODE_ALT | GPIOCFG_OSPEED_40MHz | GPIO_AF_SPI1 | GPIOCFG_OTYPE_PUPD | GPIOCFG_PUPD_PDN);
	//	hw_cfg_pin(GPIOx(MISO_PORT), MISO_PIN, GPIOCFG_MODE_ALT | GPIOCFG_OSPEED_40MHz | GPIO_AF_SPI1 | GPIOCFG_OTYPE_PUPD | GPIOCFG_PUPD_PDN);
	//	hw_cfg_pin(GPIOx(MOSI_PORT), MOSI_PIN, GPIOCFG_MODE_ALT | GPIOCFG_OSPEED_40MHz | GPIO_AF_SPI1 | GPIOCFG_OTYPE_PUPD | GPIOCFG_PUPD_PDN);
	//
	//	// configure and activate the SPI (master, internal slave select, software slave mgmt)
	//	// (use default mode: 8-bit, 2-wire, no crc, MSBF, PCLK/2, CPOL0, CPHA0)
	//	SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_SSI | SPI_CR1_SSM | SPI_CR1_SPE;
	//already Init SPI by cube mx
}

uint8_t SPIWrite8bit(uint8_t out)
{
	HAL_StatusTypeDef ret;
	ret = HAL_SPI_Transmit(&hspi2,&out,sizeof(out),10);
	return out;
}

uint8_t SPIRead8bit(uint8_t in)
{
	HAL_StatusTypeDef ret;
	ret = HAL_SPI_Receive(&hspi2,&in,sizeof(in),10);
	return in;
}

// perform SPI transaction with radio
uint8_t hal_spi (uint8_t out) {

	char outbuffer[] ="";
	char inbuffer[] ="";
	outbuffer[0] = out;
	HAL_SPI_TransmitReceive(&hspi2, (uint8_t *) outbuffer, (uint8_t *) inbuffer, sizeof(outbuffer), HAL_MAX_DELAY);

	return inbuffer[0];
}

#ifdef CFG_lmic_clib

// -----------------------------------------------------------------------------
// TIME

static void hal_time_init () {
	  //HAL_TIM_Base_Start_IT(&htim4);    // <-----------  change to your setup
}

uint32_t hal_ticks () {
    hal_disableIRQs();
    uint32_t t = HAL.ticks;
    uint16_t cnt = __HAL_TIM_GET_COUNTER(&myTIMER);
    if(__HAL_TIM_GET_FLAG(&myTIMER, TIM_FLAG_CC1) != RESET){
    	if(__HAL_TIM_GET_IT_SOURCE(&myTIMER, TIM_IT_CC1) !=RESET){
    		cnt = __HAL_TIM_GET_COUNTER(&myTIMER);
    		t++;
        }
     }
    hal_enableIRQs();
    return (t<<16)|cnt;
}

// return modified delta ticks from now to specified ticktime (0 for past, FFFF for far future)
static uint16_t deltaticks (uint32_t time) {
	uint32_t t = hal_ticks();
	int32_t d = time - t;
	if( d<=0 ) return 0;    // in the past
	if( (d>>16)!=0 ) return 0xFFFF; // far ahead
	return (uint16_t)d;
}

void hal_waitUntil (uint32_t time) {
	while( deltaticks(time) != 0 ); // busy wait until timestamp is reached
}

// check and rewind for target time
uint8_t hal_checkTimer (uint32_t time) {
    uint16_t dt;
    myTIMER.Instance->SR &= ~TIM_SR_CC1IF; // clear any pending interrupts
    if((dt = deltaticks(time)) < 5) { // event is now (a few ticks ahead)
    	myTIMER.Instance->DIER &= ~TIM_DIER_CC1IE; // disable IE
        return 1;
    } else { // rewind timer (fully or to exact time))
    	myTIMER.Instance->CCR1 = myTIMER.Instance->CNT + dt;   // set comparator
    	myTIMER.Instance->DIER |= TIM_DIER_CC1IE;  // enable IE
    	myTIMER.Instance->CCER |= TIM_CCER_CC1E;   // enable capture/compare uint 2
        return 0;
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if(htim->Instance == myTIMER.Instance){
		HAL.ticks++;
    }
	if (htim->Instance == WakeupTIMER.Instance)
	  {
		  HAL_ResumeTick();
	  }
}


// -----------------------------------------------------------------------------
// IRQ

void hal_disableIRQs () {
	__disable_irq();
	HAL.irqlevel++;
}

void hal_enableIRQs () {
	if(--HAL.irqlevel == 0)
	{
		__enable_irq();
	}
}

void hal_sleep () {
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);
	HAL_SuspendTick();
	//__HAL_RCC_PWR_CLK_ENABLE();
	HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
	//SysTick_Config();
 }

// -----------------------------------------------------------------------------

void hal_init () {
	memset(&HAL, 0x00, sizeof(HAL));
	hal_disableIRQs();

	// configure radio I/O and interrupt handler
	hal_io_init();
	// configure radio SPI
	hal_spi_init();
	// configure timer and interrupt handler
	hal_time_init();

    // initialize debug library
    debug_init();

	hal_enableIRQs();
}

void hal_failed (const char * f/* file */, uint16_t l/* line */)
{
	// HALT...
	hal_disableIRQs();
	hal_sleep();
	while(1);
}

#endif // CFG_lmic_clib
