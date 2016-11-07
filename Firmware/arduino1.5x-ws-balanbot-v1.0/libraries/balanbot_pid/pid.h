// Only modify this file to include
// - function definitions (prototypes)
// - include files
// - extern variable definitions
// In the appropriate section

#ifndef _pid_H_
#define _pid_H_

#include <Arduino.h>
//add your includes for the project PID_1 here
#include "balanbot.h"
//end of add your includes here
#ifdef __cplusplus
extern "C" {
#endif
void loop();
void setup();
#ifdef __cplusplus
} // extern "C"
#endif

//add your function definitions for the project PID_1 here

class pid
{
public:

	void restoreEEPROMValues();
	void updatePID(double x,double y,double z, double dt,cfg_t &n);
};


//Do not add code below this line
#endif /* _pid_H_ */
