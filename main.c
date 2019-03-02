/*
 * Task1.c
 *
 * Created: 10/21/2018 6:16:18 PM
 * Author : WRL
 */ 

#define F_CPU 8000000UL		// 8 MHz
#define BAUD_RATE 9600

#include "oscilloscope.h"

int main(void)
{	
	OSCI_Init();

	while(1)
	{
		OSCI_MainFunction();
	}
	return 0;
}
