// author: Karol Trzcinski
// date: 26-03-2016

// read:
// -There is only one instance of list!
// -List has limited (constant) size!


#ifndef __LIST_H__
#define __LIST_H__
#include <assert.h>


typedef signed char		list_t;
typedef list_t*			list_iterator;
typedef unsigned char	list_i; // index in table


typedef struct
{
	list_i head, tail;
	list_i maxSize;
	list_t* buffor;
} list_s;



/// @brief init list_s struct
void		list_init(list_s* l, list_t* buff, list_i maxSize);

/// @brief add new element at the end
void		list_push_back(list_s*, list_t val);

/// @brief add new element at the front
void		list_push_front(list_s*, list_t val);

/// @brief delete element at the end
void		list_pop_back(list_s*);

/// @brief delete element at the front
void		list_pop_front(list_s*);

/// @brief return last element from list
list_t		list_back(list_s*);

/// @brief return first element from list
list_t		list_front(list_s*);

/// @brief get iterotor to the first element
list_iterator list_begin(list_s*);

/// @brief get iterator AFTER the last element
list_iterator list_end(list_s*);

/// @brief return next iterator
list_iterator list_next(list_s*, list_iterator);

/// @brief return number of elements in list
unsigned char list_size(list_s*);

/// @brief return 1 when list_size()==0 and 0 otherwise
char		list_empty(list_s*); // return 0 (false) or 1 (true)

/// @brief reset list to the default state
void		list_delete(list_s*);

#endif