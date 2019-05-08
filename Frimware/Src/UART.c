/*
 * UART.c
 *
 *  Created on: 10 апр. 2019 г.
 *      Author: dima4
 */
#include "UART.h"
#include "buffer.h"
#include "main.h"
/***************************************************************************************/
/***************************************************************************************/
uint32_t uart_brr(void);

/*******************************Вычисление USART_BRR************************************/
uint32_t uart_brr(void) {

	uint16_t div_man;
	uint16_t div_fra;
	float_t div;
	div = (float) FREQ / (BAUD * 16);
	div_man = (uint32_t) div;
	div_fra = round((div - div_man) * 16.0);
	div_man = (uint32_t) div << 4;
	uint32_t USART_BRR = div_man + div_fra;
	return USART_BRR;

}
/***************************************************************************************/
void UART_init(void) {

	buffer_init(&buf, 256);



	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN; 	/*	Enable  IO port A clock	*/
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;	/*	Enable USART1 clock	*/
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;		/*	Enable alternate Function I/O clock */


	USART1->BRR = uart_brr(); /*	USART DIV	*/
	USART1->CR1 &= ~USART_CR1_M; /*	1Start bit, 8 Data bits	*/
	USART1->CR1 &= ~USART_CR1_PCE; /*	Parity control disable	*/
	USART1->CR2 &= ~USART_CR2_STOP_0 & ~USART_CR2_STOP_1; /*	1 Stop bit	*/
	USART1->CR1 |= USART_CR1_UE; /*	UART Enable	*/
	USART1->CR1 |= USART_CR1_TE; /*	Transmitter enable	*/
	USART1->CR1 |= USART_CR1_RE; /*	Receiver enable	*/

	USART1->CR1 |= USART_CR1_RXNEIE; /*	RXNE interrupt enable	*/

	GPIOA->CRH |= 	GPIO_CRH_CNF9_1;	/*	Alternate function output push-pull PA9(TX)	*/
	GPIOA->CRH &= 	~GPIO_CRH_CNF9_0;	/*	Alternate function output push-pull PA9(TX)	*/
	GPIOA->CRH |= GPIO_CRH_MODE9_0 | GPIO_CRH_MODE9_1;	/*	Max output 50MHz	*/

	GPIOA->CRH |= GPIO_CRH_CNF10_0;		/*	Input floating	PA10(RX)	*/
	GPIOA->CRH &= ~GPIO_CRH_CNF10_1;	/*	Input floating	PA10(RX)	*/
	GPIOA->CRH &= ~GPIO_CRH_MODE10;		/*	MODE1,0 = 00	*/

	__enable_irq();
	NVIC_EnableIRQ(USART1_IRQn);

}
/*******************************Send byte***********************************************/
void UART_send(uint8_t str){



	while(!(USART1->SR & USART_SR_TC));
	USART1->DR = str;
}
/*******************************Send string*********************************************/
void UART_send_str(char* str){
	while(*str){
		UART_send(*str++);
	}
}
/*********************************Receiver**********************************************/
