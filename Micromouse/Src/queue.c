#include "queue.h"

queue_t buffor[QUEUE_BUFF_SIZE];
unsigned short int _tail = 0;
unsigned short int _head = 0;

void queue_push(queue_t val)
{
	// czy ogon nie dogonil glowy
	assert( (_head + 1) % QUEUE_BUFF_SIZE != _tail); 
	_head = (_head + 1) % QUEUE_BUFF_SIZE;
	buffor[_head] = val;
}

queue_t queue_pop()
{
	// czy ogon nie dogonil glowy
	assert( (_head + 1) % QUEUE_BUFF_SIZE != _tail);
	_tail = (_tail + 1) % QUEUE_BUFF_SIZE;
	return buffor[_tail];
}

unsigned char queue_size()
{
	if (_head > _tail)
		return _head - _tail;
	else
		return QUEUE_BUFF_SIZE - (_tail - _head);
}

char queue_empty()
{
	return _tail == _head;
}

void queue_delete()
{
	_tail = _head;
}


