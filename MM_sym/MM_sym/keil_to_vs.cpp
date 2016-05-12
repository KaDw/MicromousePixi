#include "stdafx.h"
#include "keil_to_vs.h"


void TurnA(Motors_t* m, int angle, int radius, int vel, int(*driver)(Motors_t*))
{
	printf("TurnA a:%d  r:%d driver:%i\n", angle, radius, driver > 0);
}

void GoA(Motors_t* m, int left, int right, int vel, int(*driver)(Motors_t*))
{
	printf("GoaA l:%d  r:%d driver:%i\n", left, right, driver > 0);
}