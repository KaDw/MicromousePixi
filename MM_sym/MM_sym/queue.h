// author: Karol Trzcinski
// date: 26-03-2016

// read:
// -There is only one instance of queue!
// -Queue has limited (constant) size!

#ifndef __QUEUE_H__
#define __QUEUE_H__
#include <assert.h>

// four times lab_size
#define QUEUE_BUFF_SIZE 16*4

typedef unsigned short int	queue_t;
typedef queue_t*			queue_iterator;

void		queue_push(queue_t val);
queue_t		queue_pop();
unsigned char queue_size();
char		queue_empty(); // return 0 (false) or 1 (true)
void		queue_delete();

#endif