#ifndef __WQUEUE_H__
#define __WQUEUE_H__
#include <assert.h>
//#include <stddef.h>

typedef unsigned char		wqueue_v; // value
typedef unsigned short int	wqueue_c; // cost
typedef short int			wqueue_i; // index in table


typedef struct 
{
	wqueue_i head, tail;
	wqueue_i maxSize;
	wqueue_v* bufValue;
	wqueue_c* bufCost;
} wqueue_s;


/// init wq data
void wqueue_init(wqueue_s * wq, wqueue_c * bufCost, wqueue_v * bufValue, wqueue_i maxSize);


void wqueue_push(wqueue_s * wq, wqueue_v val, wqueue_c cost);

/// return value stored in element i
wqueue_v wqueue_value(wqueue_s * wq, wqueue_i i);

/// return cost of element i
wqueue_c wqueue_cost(wqueue_s * wq, wqueue_i i);

/// return pointer to cost of element i
wqueue_c* wqueue_costPtr(wqueue_s * wq, wqueue_i i);

/// remove one element
void wqueue_remove(wqueue_s * wq, wqueue_i i);


/// restore whole wq to the default state
void wqueue_delete(wqueue_s * wq);


wqueue_i wqueue_next(wqueue_s * wq, wqueue_i i);


/// return 0 when 'i' is into wqueue data
char wqueue_into(wqueue_s * wq, wqueue_i i);


/// return 1 when queue is empty and 0 otherwise
char wqueue_empty(wqueue_s * wq);


/// search by value
/// return index of element or -1 when not exist
wqueue_i wqueue_find(wqueue_s * wq, wqueue_v val);


/// return index of cheapest element or tail when wqueue is empty
wqueue_i wqueue_min(wqueue_s * wq);


#endif

