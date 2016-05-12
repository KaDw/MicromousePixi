//#include "stdafx.h"
#include "wqueue.h"

// wq->tail point to the oldest element
// wq->head point to the newest element

void wqueue_init(wqueue_s * wq, wqueue_c * bufCost, wqueue_v * bufValue, wqueue_i maxSize)
{
	assert(maxSize > 0);
	assert(bufCost != 0);
	assert(bufValue != 0);

	wq->head = 0;
	wq->tail = 0;
	wq->bufCost = bufCost;
	wq->bufValue = bufValue;
	wq->maxSize = maxSize;
}

void wqueue_push(wqueue_s * wq, wqueue_v val, wqueue_c cost)
{
	assert(wq->bufCost != 0);
	assert(wq->bufValue != 0);
	assert(wq->maxSize > 0);

	assert(wq->head+1 != wq->tail); // overload
	wq->bufCost[wq->head] = cost;
	wq->bufValue[wq->head] = val;
	wq->head = wqueue_next(wq, wq->head);
}


wqueue_v wqueue_value(wqueue_s * wq, wqueue_i i)
{
	return wq->bufValue[i];
}


wqueue_c wqueue_cost(wqueue_s * wq, wqueue_i i)
{
	return wq->bufCost[i];
}

wqueue_c * wqueue_costPtr(wqueue_s * wq, wqueue_i i)
{
	return wq->bufCost + i;
}


wqueue_i wqueue_next(wqueue_s* wq, wqueue_i i)
{
	return (i + 1) % wq->maxSize;
}

/// return 0 when 'i' is into wqueue data
char wqueue_into(wqueue_s* wq, wqueue_i i)
{
	if (wq->head > wq->tail) // przed cyrkulacja
		return wq->tail <= i && i < wq->head;
	else
		return i < wq->head || wq->tail <= i;
}


char wqueue_empty(wqueue_s* wq)
{
	return wq->head == wq->tail;
}


/// return index of cheapest element or tail when wqueue is empty
wqueue_i wqueue_min(wqueue_s* wq)
{
	wqueue_i i;
	wqueue_i mini = wq->tail;
	wqueue_c min = wq->bufCost[mini]; //warning: when wqueue is empty then it is bullshit

	for (i = wqueue_next(wq, wq->tail); wqueue_into(wq, i); i = wqueue_next(wq, i))
		if (wq->bufCost[i] < min)
		{
			min = wq->bufCost[i];
			mini = i;
		}

	return mini;
}


/// search by value
/// return index of element or -1 when not exist
wqueue_i wqueue_find(wqueue_s* wq, wqueue_v val)
{
	wqueue_i i = wq->tail;
	
	if (wqueue_empty(wq))
		return -1;

	do
	{
		if (wq->bufValue[i] == val)
			return i;
		i = wqueue_next(wq, i);
	} while (wqueue_into(wq, i));

	return -1;
}


void wqueue_remove(wqueue_s* wq, wqueue_i i)
{
	assert(wqueue_into(wq, i));

	// if i is not tail then copy tail to i and move tail by one
	// when i is tail then only					move tail by one
	/*if (i != wq->tail)
	{
		wq->bufCost[i] = wq->bufCost[wq->tail];
		wq->bufValue[i] = wq->bufValue[wq->tail];
	}

	wq->tail = wqueue_next(wq, wq->tail);*/

	// we have checked that wqueue is not empty at begin of this function, so head > 0
	if (i != wq->head - 1)
	{
		wq->bufCost[i] = wq->bufCost[wq->head - 1];
		wq->bufValue[i] = wq->bufValue[wq->head - 1];
	}

	--wq->head;
}


/// restore wq to the default state
void wqueue_delete(wqueue_s* wq)
{
	wq->head = wq->tail;
}