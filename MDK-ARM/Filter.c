#include "Filter.h"
//#include "main.h"
float GetAverage(uint16_t buffer[],int n)
{
	static float buff;
	for(int i = 1;i <= 50;i++)
	{
		
		buff += buffer[n];
		
	}
	buff = buff / 50.0f ;
	return buff ;
}