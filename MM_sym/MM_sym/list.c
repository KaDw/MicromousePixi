#include "list.h"



void list_init(list_s * l, list_t * buff, list_i maxSize)
{
	l->buffor = buff;
	l->maxSize = maxSize;
	l->head = 0;
	l->tail = 0;
}

void list_push_back(list_s* l, list_t val)
{
	// czy ogon nie dogonil glowy
	assert((l->head + 1) % l->maxSize != l->tail);
	l->buffor[l->head] = val;
	l->head = (l->head + 1) % l->maxSize;
}

void list_push_front(list_s * l, list_t val)
{
	// czy ogon nie dogonil glowy
	assert((l->head + 1) % l->maxSize != l->tail);

	if (l->tail == 0)
		l->tail = l->maxSize - 1;
	else
		l->tail = (l->tail - 1);

	l->buffor[l->tail] = val;
}

void list_pop_back(list_s * l)
{
	assert(!list_empty(l));

	if (l->head != 0)
		--l->head;
	else
		l->head = l->maxSize - 1;
}

void list_pop_front(list_s * l)
{
	assert(!list_empty(l));
	l->tail = (l->tail + 1) % l->maxSize;
}

list_t list_back(list_s * l)
{
	assert(!list_empty(l));
	if (l->head != 0)
		return l->buffor[l->head - 1];
	else
		return l->buffor[l->maxSize - 1];
}

list_t list_front(list_s * l)
{
	assert(!list_empty(l));
	return l->buffor[l->tail];
}

list_iterator list_begin(list_s * l)
{
	return l->buffor+l->tail;
}

list_iterator list_end(list_s * l)
{
	return l->buffor+l->head;
}

list_iterator list_next(list_s * l, list_iterator i)
{
	return (i-l->buffor+1) % l->maxSize + l->buffor;
}

unsigned char list_size(list_s * l)
{
	if (l->head > l->tail)
		return l->head - l->tail;
	else
		return l->maxSize - (l->tail - l->head);
}

char list_empty(list_s * l)
{
	return l->tail == l->head;
}

void list_delete(list_s * l)
{
	l->head = l->tail = 0;
}
