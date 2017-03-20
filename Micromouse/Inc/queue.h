#ifndef __QUEUE_H__
#define __QUEUE_H__

#include <stdbool.h>
#include "stm32f4xx.h"


#define LEN 10


typedef struct{
	void (*f_ptr)(int, int, float); // pointer to function
	int a; // function arguments
	int b;
	float c;
} _queue_elem;


typedef struct{
	uint8_t head;
	uint8_t tail;
	_queue_elem queue_buf[LEN];
} _queue;


extern void (*f_ptr)(int, int, float);
extern _queue queue;

void q_push(_queue* queue, void (*f_ptr)(int, int, float), int _a, int _b, float _c);
bool q_empty(_queue* queue);
_queue_elem* q_pop(_queue* queue);


#endif