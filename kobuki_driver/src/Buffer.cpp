#include "Buffer.h"

Buffer::Buffer(){
	this->count = 0;
	this->head = 0;
	this->tail = 0;
}

uint8_t Buffer::top(){
	return this->buf[head];
}

Buffer::~Buffer(){
	
}

uint8_t Buffer::size(){
	return count;
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
