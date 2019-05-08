/*
 * buffer.c
 *
 *  Created on: 12 апр. 2019 г.
 *      Author: dima4
 */
#include "buffer.h"
#include <stdlib.h>



void buffer_init(Type_def_buff* buf, uint16_t size){

	buf->size = size;
	buf->buffer = (uint8_t*)malloc(size);
	buf->indexIn = 0;
	buf->indexOut = 0;
}

void buffer_push(uint8_t data, Type_def_buff* buf){

	buf->buffer[buf->indexIn++] = data;

	if (buf->indexIn > buf->size){
		buf->indexIn = 0;
	}

}





