#include "worked_delay.h"



void delay_us( uint32_t Val) 
{
	Val=Val*10;
  for( ; Val != 0; Val--) 
  		{
		__NOP();
  		}
}

void delay_ms( uint32_t Val) 
{
	Val=Val*10000;
	for( ; Val != 0; Val--) 
  		{
		__NOP();
  		}
}