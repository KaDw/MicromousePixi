//#include <stdbool.h>
//#include "stm32f4xx.h"
#include "queue.h"

_queue queue;

void q_push(_queue* queue, void (*_f_ptr)(int, int, float), int _a, int _b, float _c){
	if((queue->head + 1) % LEN == queue->tail)
		return; // queue error flag
	queue->head = (queue->head + 1) % LEN;
	
	queue->queue_buf[queue->head].f_ptr = _f_ptr;
	queue->queue_buf[queue->head].a = _a;
	queue->queue_buf[queue->head].b = _b;
	queue->queue_buf[queue->head].c = _c;
	
}

bool q_empty(_queue* queue){
	if(queue->head == queue->tail)
		return true;
	return false;
}

_queue_elem* q_pop(_queue* queue){
	if((queue->head + 1) % LEN == queue->tail)
		return NULL; // TODO fix this
	queue->tail = (queue->tail + 1) % LEN;
	return &(queue->queue_buf[queue->tail]);
}


//uint8_t q_size(_queue* queue){
//	return queue->head - queue->tail;
//}

