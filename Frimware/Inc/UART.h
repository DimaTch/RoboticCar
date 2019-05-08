/*
 * UART.h
 *
 *  Created on: 10 апр. 2019 г.
 *      Author: dima4
 */

#ifndef CODE_INC_UART_H_
#define CODE_INC_UART_H_
#include "stm32f1xx.h"
#include "math.h"

#define FREQ  8000000U	/*  frequency UART	*/
#define BAUD  9600U		/*	baud rate	*/


 void UART_init(void);		/*	Инициализация UART	*/
 void UART_send(uint8_t);		/*	Отправка символа	*/
 void UART_send_str(char *);	/*	Отправка строки		*/


#endif /* CODE_INC_UART_H_ */
