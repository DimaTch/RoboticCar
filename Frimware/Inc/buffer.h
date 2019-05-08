/*
 * buffer.h
 *
 *  Created on: 12 апр. 2019 г.
 *      Author: dima4
 */

#ifndef CODE_INC_BUFFER_H_
#define CODE_INC_BUFFER_H_

#include "stm32f1xx.h"

typedef struct{
uint16_t indexIn;
uint16_t indexOut;
uint16_t size;
uint8_t* buffer;
}Type_def_buff;

void buffer_init(Type_def_buff* buf, uint16_t size);
void buffer_push(uint8_t data, Type_def_buff* buf); /*	write data	*/
uint8_t buffer_pull(Type_def_buff* buf);		/*	read data	*/




#endif /* CODE_INC_BUFFER_H_ */
