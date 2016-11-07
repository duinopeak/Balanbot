/*
 * mortor.cpp
 *
 *  Created on: 2014年7月19日
 *      Author: caixiaoliang
 */

#include "motor.h"
//#include <Arduino.h> //包含了中断寄存OCR1A.OCR1B
#include "encoder.h"//在电机复位函数中药要到encoder.h中的getPosition();函数

void Motor::setPWM(Command motor, uint16_t dutyCycle )//dutyCycle is a value between 0-ICR1
{
    if(motor==left)
    {
    	OCR1A=dutyCycle;
    }
    else
    {
    	OCR1B=dutyCycle;
    }
}
void Motor::moveMotor(Command motor,Command direction,double speedRaw)
{//speed is a value in percentage 0-100%
    if (speedRaw>100)
    {
    	speedRaw=100;
    }
    setPWM(motor,speedRaw*((double)PWMVALUE)/100.0);//Scale from 0-100 to 0-PWMVALUE //1-250
    if(motor==left)//这个函数控制左边轮子的正反转
    {
    	if(direction==forward)
    	{
    		M1A::Clear();//控制INA,INB,控制电机正反转
    		M1B::Set();  //现在向前转
    	}
    	else
    	{
    		M1A::Set(); //轮子向后转
    		M1B::Clear();
    	}
    }

    else //控制右边轮子正反转
    {
        if(direction==forward)
        {
        	M2A::Set();
        	M2B::Clear();
        }
        else
        {

        	M2A::Clear();//控制右边的驱动芯片的INA,INB,控制 电机正反转
        	M2B::Set();
        }
    }

}

void Motor::stopMotor(Command motor)
{
	setPWM(motor,PWMVALUE);//set high,
	if(motor==left) //左轮刹车
	{
		M1A::Set();
		M1B::Set();
	}
	else          //右轮刹车
	{
		M2A::Set();
		M2B::Set();
	}
}

void Motor::stopAndReset() //停止左右轮转动
{
	encoder encoderM;
	stopMotor(left);
	stopMotor(right);
	integratedError = 0;
	targetPosition = encoderM.getWheelsPosition();
	lastRestAngle = cfg.targetAngle;
}








