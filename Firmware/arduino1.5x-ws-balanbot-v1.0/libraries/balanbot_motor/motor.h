/*
 * motor.h
 *
 *  Created on: 2014年7月19日
 *      Author: caixiaoliang
 */

#ifndef MOTOR_H_
#define MOTOR_H_
#include <stdint.h> // Needed for uint8_t, uint16_t etc.
#include <Arduino.h>
#include "Usb.h"  //this there is very important ,这里有包含了avrpins.h,重新写了一个引脚快速定义拉高拉低的函数
#include "balanbot.h"

class Motor
{
public:
	    //Motor();//构造函数
		void moveMotor(Command motor, Command direction, double speedRaw);
		void stopMotor(Command motor);
		void setPWM(Command motor, uint16_t dutyCycle);
		void stopAndReset();

};



#endif /* MOTOR_H_ */
