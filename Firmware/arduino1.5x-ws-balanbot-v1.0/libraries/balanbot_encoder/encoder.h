/*
 * encoder.h
 *
 *  Created on: 2014年7月22日
 *      Author: lryain
 */

#ifndef ENCODER_H_
#define ENCODER_H_
#include <Arduino.h>
#include "Usb.h"
//add your includes for the project motor_with_encoder here
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

void setInterrupt();//此函数若放在类中声明，在定义函数时会出现参数不匹配的错误
void leftEncoder();
void rightEncoder();

class encoder{
public:
	void setupTiming();
	void setupEncoders();
	void updateEncoders();

	int32_t readLeftEncoder();
	int32_t readRightEncoder();
	int32_t getWheelsPosition();
	//void setInterrupt();
//	void moveMotor(Command motor, Command direction, double speedRaw);
//	void stopMotor(Command motor);
	//void testMotorSpeed(double *leftSpeed, double *rightSpeed, double leftScaler, double rightScaler) ;
};


#endif /* ENCODER_H_ */
