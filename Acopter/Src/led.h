#ifndef __LED_H
#define __LED_H 

#include "include.h"
#include "mymath.h"
 
class LED
{
public:
	short on_time;
	short off_time;
	void led_change();


	void led_state0();
	void led_state2();
	void led_state1();
	 LED()
         {
         
         }
private:

};
 
extern LED led;



#endif
