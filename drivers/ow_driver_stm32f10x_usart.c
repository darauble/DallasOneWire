/*
 * The source of the OneWire protocol driver for USARTs on STM32F10x family microcontrollers.
 * Currently supports USART1, USART2 and USART3 on STM32F103
 *
 * ow_driver_stm32f10x_usart.c
 *
 *  Created on: Mar 22, 2018
 *      Author: Darau, blė
 *
 *	Credits:
 *	- Игорь (aka dadigor), steel_ne, Jonas Trečiokas (aka Jonis):
 *	  http://we.easyelectronics.ru/STM32/stm32-1-wire-dma-prodolzhenie.html
 *	  http://we.easyelectronics.ru/STM32/esche-raz-o-stm32-i-ds18b20-podpravleno.html
 *
 *  This file is a part of personal use libraries developed specifically for STM32 Cortex-M3
 *  microcontrollers, most notably STM32F103 series.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 */

#include <ow_driver.h>
#include <ow_driver_stm32f10x_usart.h>

#include "stm32f10x.h"

// USART1 uses faster clock (APB2) than USART2/3, hence the difference in constants
// Adjusted to 72 MHz system core clock and APB2. APB1 - 36 MHz. Adjust for different
// prescalers.
#define U1_BAUD_9600	((uint16_t)0x1D4C)
#define U1_BAUD_115200 ((uint16_t)0x0271)

#define Ux_BAUD_9600	((uint16_t)0x0EA6)
#define Ux_BAUD_115200 ((uint16_t)0x0138)

#define RESET_BYTE (uint8_t)0xF0

#define SEND_0	0x00
#define SEND_1	0xff
#define OW_R_1	0xff

const uint8_t ow_bits[] = { SEND_0, SEND_1 };

#ifdef OW_MODE_USART_DMA
// TODO: DMA Mode is still relatively under construction. Writing bytes works,
// but reading still relies on simple DR register use.
#define DMA_BUF_SIZE 8
static uint8_t dma_buf[DMA_BUF_SIZE];
#endif

struct one_wire_driver {
	USART_TypeDef* USARTx;
	uint16_t brr_9600;
	uint16_t brr_115200;
#ifdef OW_MODE_USART_DMA
	DMA_TypeDef *dma;
	DMA_Channel_TypeDef *dma_rx;
	DMA_Channel_TypeDef *dma_tx;
	uint32_t dma_rx_flag;
	uint32_t dma_tx_flag;
	uint32_t dma_ifcr_clear;
#endif
};

struct one_wire_driver one_wire_heap[OW_HEAP_SIZE];
static int heap_ptr = 0;

int init_driver(ow_driver_ptr *d, int u)
{
	if (heap_ptr > OW_HEAP_SIZE) {
		return OW_NOMEMORY;
	}
	*d = &one_wire_heap[heap_ptr++];

	switch (u) {
		case E_USART1:
			(*d)->USARTx = USART1;
		break;
		case E_USART2:
			(*d)->USARTx = USART2;
		break;
		case E_USART3:
			(*d)->USARTx = USART3;
		break;
#if defined (STM32F10X_HD) || defined  (STM32F10X_CL) || defined  (STM32F10X_XL)
		case E_UART4:
			(*d)->USARTx = UART4;
		break;
		case E_UART5:
			(*d)->USARTx = UART5;
		break;
#endif
		default:
			heap_ptr--; // Reclaim heap in case of error
			return OW_ERR;
		break;
	}

	// Enable alternative I/O
	RCC->APB2ENR |=  RCC_APB2ENR_AFIOEN;

	if ((*d)->USARTx == USART1) {
		RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
		(*d)->brr_9600 = U1_BAUD_9600;
		(*d)->brr_115200 = U1_BAUD_115200;
	} else {
		(*d)->brr_9600 = Ux_BAUD_9600;
		(*d)->brr_115200 = Ux_BAUD_115200;
		if ((*d)->USARTx == USART2) {
			RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
		} else if ((*d)->USARTx == USART3) {
			RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
#if defined (STM32F10X_HD) || defined  (STM32F10X_CL) || defined  (STM32F10X_XL)
		} else if ((*d)->USARTx == UART4) {
			RCC->APB1ENR |= RCC_APB1ENR_UART4EN;
		} else if ((*d)->USARTx == UART5) {
			RCC->APB1ENR |= RCC_APB1ENR_UART5EN;
#endif
		}
	}

	// Clear config
	(*d)->USARTx->CR1 = 0;
	(*d)->USARTx->CR2 = 0;
	(*d)->USARTx->CR3 = 0;

	(*d)->USARTx->CR1 |= USART_CR1_UE | USART_CR1_RE | USART_CR1_TE; // enable, 8b word, no parity, RX/TX
	(*d)->USARTx->BRR = (*d)->brr_115200;

	// Enable appropriate GPIO
	if ((*d)->USARTx == USART1 || (*d)->USARTx == USART2 ) {
		// Enable GPIOA
		RCC->APB2ENR |=  RCC_APB2ENR_IOPAEN;
		if ((*d)->USARTx == USART1) {
			GPIOA->CRH &= ~(GPIO_CRH_CNF9 | GPIO_CRH_MODE9 | GPIO_CRH_CNF10 | GPIO_CRH_MODE10); // Clear configuration
			GPIOA->CRH |= GPIO_CRH_CNF9 | GPIO_CRH_MODE9 | GPIO_CRH_CNF10_0; // Alternative open drain, 50 MHz; in floating
		} else {
			GPIOA->CRL &= ~(GPIO_CRL_CNF2 | GPIO_CRL_MODE2 | GPIO_CRL_CNF3 | GPIO_CRL_MODE3);
			GPIOA->CRL |= GPIO_CRL_CNF2 | GPIO_CRL_MODE2 | GPIO_CRL_CNF3_0;
		}
	}
#if !defined (STM32F10X_LD) && !defined (STM32F10X_LD_VL)
	else if ((*d)->USARTx == USART3) {
		// Enable GPIOB
		RCC->APB2ENR |=  RCC_APB2ENR_IOPBEN;
		GPIOB->CRH &= ~(GPIO_CRH_CNF10 | GPIO_CRH_MODE10 | GPIO_CRH_CNF11 | GPIO_CRH_MODE11);
		GPIOB->CRH |= GPIO_CRH_CNF10 | GPIO_CRH_MODE10 | GPIO_CRH_CNF11_0;
	}
#endif
#if defined (STM32F10X_HD) || defined  (STM32F10X_CL) || defined  (STM32F10X_XL)
	else if ((*d)->USARTx == UART4) {
		// Enable GPIOC
		RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
		GPIOC->CRH &= ~(GPIO_CRH_CNF10 | GPIO_CRH_MODE10 | GPIO_CRH_CNF11 | GPIO_CRH_MODE11);
		GPIOC->CRH |= GPIO_CRH_CNF10 | GPIO_CRH_MODE10 | GPIO_CRH_CNF11_0;
	} else if ((*d)->USARTx == UART5) {
		// UART5 TX is on GPIOC, and RX on GPIOD. Crazy stuff.
		RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
		RCC->APB2ENR |= RCC_APB2ENR_IOPDEN;
		GPIOC->CRH &= ~(GPIO_CRH_CNF12 | GPIO_CRH_MODE12);
		GPIOD->CRL &= ~(GPIO_CRL_CNF2 | GPIO_CRL_MODE2);
		GPIOC->CRH |= GPIO_CRH_CNF12 | GPIO_CRH_MODE12;
		GPIOD->CRL |= GPIO_CRL_CNF2_0;
	}
#endif

#ifdef OW_MODE_USART_DMA
	if ((*d)->USARTx == USART1) {
		(*d)->dma = DMA1;
		(*d)->dma_rx = DMA1_Channel5;
		(*d)->dma_tx = DMA1_Channel4;
		(*d)->dma_rx_flag = DMA1_FLAG_TC5;
		(*d)->dma_tx_flag = DMA1_FLAG_TC4;
		(*d)->dma_ifcr_clear = ((uint32_t)
				(DMA_ISR_GIF4 | DMA_ISR_TCIF4 | DMA_ISR_HTIF4 | DMA_ISR_TEIF4)
				|(DMA_ISR_GIF5 | DMA_ISR_TCIF5 | DMA_ISR_HTIF5 | DMA_ISR_TEIF5));
	}
	else if ((*d)->USARTx == USART2) {
		(*d)->dma = DMA1;
		(*d)->dma_rx = DMA1_Channel6;
		(*d)->dma_tx = DMA1_Channel7;
		(*d)->dma_rx_flag = DMA1_FLAG_TC6;
		(*d)->dma_tx_flag = DMA1_FLAG_TC7;
		(*d)->dma_ifcr_clear = ((uint32_t)
				(DMA_ISR_GIF6 | DMA_ISR_TCIF6 | DMA_ISR_HTIF6 | DMA_ISR_TEIF6)
				|(DMA_ISR_GIF7 | DMA_ISR_TCIF7 | DMA_ISR_HTIF7 | DMA_ISR_TEIF7));
	} else if ((*d)->USARTx == USART3) {
		(*d)->dma = DMA1;
		(*d)->dma_rx = DMA1_Channel3;
		(*d)->dma_tx = DMA1_Channel2;
		(*d)->dma_rx_flag = DMA1_FLAG_TC3;
		(*d)->dma_tx_flag = DMA1_FLAG_TC2;
		(*d)->dma_ifcr_clear = ((uint32_t)
				(DMA_ISR_GIF2 | DMA_ISR_TCIF2 | DMA_ISR_HTIF2 | DMA_ISR_TEIF2)
				|(DMA_ISR_GIF3 | DMA_ISR_TCIF3 | DMA_ISR_HTIF3 | DMA_ISR_TEIF3));
	}
#if defined (STM32F10X_HD) || defined  (STM32F10X_CL) || defined  (STM32F10X_XL)
	else if ((*d)->USARTx == USART4) {
		(*d)->dma = DMA2;
		(*d)->dma_rx = DMA1_Channel3;
		(*d)->dma_tx = DMA1_Channel5;
		(*d)->dma_rx_flag = DMA1_FLAG_TC3;
		(*d)->dma_tx_flag = DMA1_FLAG_TC5;
		(*d)->dma_ifcr_clear = ((uint32_t)
			(DMA_ISR_GIF3 | DMA_ISR_TCIF3 | DMA_ISR_HTIF3 | DMA_ISR_TEIF3)
			|(DMA_ISR_GIF5 | DMA_ISR_TCIF5 | DMA_ISR_HTIF5 | DMA_ISR_TEIF5));
	}
#endif /* defined (STM32F10X_HD) || defined  (STM32F10X_CL) || defined  (STM32F10X_XL) */

#ifdef STM32F10X_HD_VL
#warning "This MCU with DMA for UART5 is not supported"
#endif /* STM32F10X_HD_VL */

	// Enable DMA clock
	if ((*d)->dma == DMA1) {
		RCC->AHBENR |= RCC_AHBENR_DMA1EN;
	}
#if defined (STM32F10X_HD) || defined  (STM32F10X_CL) || defined  (STM32F10X_XL)
	else {
		RCC->AHBENR |= RCC_AHBENR_DMA2EN;
	}
#endif
	// Initialize DMA

	// DMA for reading
	(*d)->dma_rx->CPAR = (uint32_t) &((*d)->USARTx->DR);
	(*d)->dma_rx->CCR = 0;
	(*d)->dma_rx->CCR |= (DMA_CCR1_MINC); // Increment memory buffer

	// DMA for writing
	(*d)->dma_tx->CPAR = (uint32_t) &((*d)->USARTx->DR);
	(*d)->dma_tx->CCR = 0;
	(*d)->dma_tx->CCR |= (DMA_CCR1_MINC | DMA_CCR1_DIR ); // Increment memory buffer | Memory to peripheral

#endif /* OW_MODE_USART_DMA */

	return OW_OK;
}

int reset_wire(ow_driver_ptr d)
{
	uint8_t ow_presence;
	// Set 9600 baud for 480 µs reset pulse
	d->USARTx->BRR = d->brr_9600;
	d->USARTx->SR &= ~(USART_SR_TC | USART_SR_RXNE);
	d->USARTx->DR = RESET_BYTE;
	while(!(d->USARTx->SR & USART_SR_TC) && !(d->USARTx->SR & USART_SR_RXNE)) {
		OW_YIELD;
	}
	ow_presence = (uint8_t)(d->USARTx->DR & (uint16_t)0x01FF);
	// Restore "normal" operating speed
	d->USARTx->BRR = d->brr_115200;

	if (ow_presence != RESET_BYTE) {
		return OW_OK;
	}

	return OW_NOTHING;
}

int write_bit(ow_driver_ptr d, uint8_t bit)
{
	d->USARTx->SR &= ~(USART_FLAG_TC | USART_FLAG_RXNE);
	d->USARTx->DR = ow_bits[bit];

	while(!(d->USARTx->SR & USART_FLAG_TC) && !(d->USARTx->SR & USART_FLAG_RXNE)) {
		OW_YIELD;
	}

	return OW_OK;
}

int read_bit(ow_driver_ptr d, uint8_t *rbit)
{
	uint8_t bit;

	d->USARTx->SR &= ~(USART_FLAG_TC | USART_FLAG_RXNE);
	d->USARTx->DR = OW_R_1;
	while(!(d->USARTx->SR & USART_FLAG_TC) && !(d->USARTx->SR & USART_FLAG_RXNE)) {
		OW_YIELD;
	}
	bit = (uint8_t)(d->USARTx->DR & (uint16_t)0x01FF);
	if (bit == OW_R_1) {
		*rbit = 1;
	} else {
		*rbit = 0;
	}

	return OW_OK;
}

#ifdef OW_MODE_USART_DMA
int write_byte(ow_driver_ptr d, uint8_t byte)
{
	uint8_t i;
	for (i=0; i<8; i++) {
		dma_buf[i] = ow_bits[byte & 0x01];
		byte >>= 1;
	}

	d->USARTx->SR = ~( USART_FLAG_TC | USART_FLAG_RXNE); // Clear flags

	d->dma_tx->CMAR = (uint32_t) dma_buf;
	d->dma_tx->CNDTR = DMA_BUF_SIZE;

	d->dma->IFCR |= d->dma_ifcr_clear;

	d->USARTx->CR3 |= USART_CR3_DMAT ; // Enable TX DMA
	d->dma_tx->CCR |= DMA_CCR1_EN;

	while (!(d->dma->ISR & d->dma_tx_flag) // Wait until DMA marks it's finished sending buffer
		// ------------------------------------------------------------------------
		&& !(d->USARTx->SR & USART_FLAG_TC) && !(d->USARTx->SR & USART_FLAG_RXNE)) {
		/* DMA does not clear USART flags on last write :-/
		If USART_FLAG_TC and USART_FLAG_RXNE flags are not checked, bytes in USART
		output get corrupted and commands are not sent properly. It happens because
		DMA finishes its job first, then USART is stil sending last byte from buffer
		during several CPU cycles. */
		OW_YIELD;
	}

	d->dma_tx->CCR &= ~DMA_CCR1_EN;
	d->USARTx->CR3 &= (uint16_t) ~USART_CR3_DMAT;

	return OW_OK;
}
/*
// Can't get this working properly even with two buffers
int read_byte(ow_driver_ptr d, uint8_t *rbyte)
{
	uint8_t i, j;
	for (i=0; i<8; i++) {
		dma_buf[i] = SEND_1;
	}

	d->USARTx->SR = ~( USART_FLAG_TC | USART_FLAG_RXNE); // Clear flags

	d->dma_tx->CMAR = (uint32_t) dma_buf;
	d->dma_tx->CNDTR = DMA_BUF_SIZE;

	d->dma_rx->CMAR = (uint32_t) dma_buf;
	d->dma_rx->CNDTR = DMA_BUF_SIZE;

	d->dma->IFCR |= d->dma_ifcr_clear;
	d->USARTx->CR3 |= (USART_CR3_DMAT | USART_CR3_DMAR) ; // Enable TX/RX DMA

	d->dma_rx->CCR |= DMA_CCR1_EN;
	d->dma_tx->CCR |= DMA_CCR1_EN;

	i = 0;
	while (!(d->dma->ISR & d->dma_tx_flag) && !(d->dma->ISR & d->dma_rx_flag)) {
		i++;
		OW_YIELD;
	}
	j = 0;
	while(!(d->USARTx->SR & USART_FLAG_TC) && !(d->USARTx->SR & USART_FLAG_RXNE)) {
		j++;
		OW_YIELD;
	}

	d->dma_tx->CCR &= ~DMA_CCR1_EN;
	d->dma_rx->CCR &= ~DMA_CCR1_EN;
	d->USARTx->CR3 &= (uint16_t) ~(USART_CR3_DMAT | USART_CR3_DMAR);

	snprintf(buf, 99, "OWDRV: read flag checks: %d, %d\r\n", i, j);
	print_usart(USART1, buf);
	for (i=0; i<8; i++) {
		snprintf(buf, 99, "%02X ", dma_buf[i]);
		print_usart(USART1, buf);
	}
	print_usart(USART1, "\r\n");

	uint8_t byte = 0;

	for (i=0; i<8; i++) {
		byte >>= 1;
		if (dma_buf[i] == OW_R_1) {
			byte |= 0x80;
		}
	}
	*rbyte = byte;

	return OW_OK;
}//*/

#else
int write_byte(ow_driver_ptr d, uint8_t byte)
{
	uint8_t i;
	for (i=0; i<8; i++) {
		write_bit(d, byte & 0x01);
		byte >>= 1;
	}
	return OW_OK;
}
#endif /* OW_MODE_USART_DMA */
int read_byte(ow_driver_ptr d, uint8_t *rbyte)
{
	int c;
	uint8_t byte = 0, bit, i;
	for (i=0; i<8; i++) {
		byte >>= 1;
		c = read_bit(d, &bit);
		if (c != OW_OK) {
			return c;
		}
		if (bit) {
			byte |= 0x80;
		}
	}
	*rbyte = byte;

	return OW_OK;
}
