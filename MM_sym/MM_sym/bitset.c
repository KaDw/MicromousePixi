#include "stdafx.h"
#include "bitset.h"

void bitset_init(bitset_s * bs, bitset_t * data, int lenght)
{
	bs->data = data;
	bs->size = lenght;
}

void bitset_delete(bitset_s * bs)
{
	bitset_t* i = &bs->data;
	bitset_t* end = bs->data + bs->size;

	while (i != end)
		*i = 0;

}

void bitset_set(bitset_s * bs, int bit)
{
	int b = bit % BITSET_MODULO;
	int i = bit / BITSET_MODULO;

	assert(i < bs->size);

	bs->data[i] |= 1 << b;
}

void bitset_clear(bitset_s * bs, int bit)
{
	int b = bit % BITSET_MODULO;
	int i = bit / BITSET_MODULO;

	assert(i < bs->size);

	bs->data[i] &= ~(1 << b);
}

int bitset_get(bitset_s * bs, int bit)
{
	int b = bit % BITSET_MODULO;
	int i = bit / BITSET_MODULO;

	assert(i < bs->size);

	return (bs->data[i] & (1 << b));
}
