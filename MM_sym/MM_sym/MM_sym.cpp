// MM_sym.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "GUI.h"


int main()
{
	try
	{
		GUI* g = GUI::getGUI();
		g->start();
	}
	catch (std::exception e)
	{
		printf("Wystapil blad: %s", e.what());
		std::cin.get(); // \n
		std::cin.get();
	}

	return 0;
}

