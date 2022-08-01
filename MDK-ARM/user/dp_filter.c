

#include "dp_filter.h"

float filter(uint32_t buffer[],int n)
{
	float buff;
	for(int i = 1;i <= 50;i++)
	{
		
		buff += buffer[n];
		
	}
	buff = buff / 50 ;
	return buff ;
}



