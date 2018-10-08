#include "Buffer.h"

Buffer::Buffer(){
	this->count = 0;
	this->head = 0;
	this->tail = 0;
}

Buffer::~Buffer(){
	
}

void Buffer::push(uint8_t data){
	this->buf[this->tail] = data;
	this->tail = (this->tail + 1) % BUFFER_LEN;
	this->count++;
}

uint8_t Buffer::pop(){
	uint8_t data = this->buf[this->head];
	this->head = (this->head + 1) % BUFFER_LEN;
	this->count--;
	return data;
}
