/*
 * balanbot-motor.ino
 *
 *  Created on: 2010年4月12日
 *      Author: caixiaoliang
 */
#include <Arduino.h>
#include "motor.h"
#include "Usb.h"
#include <encoder.h>

const uint16_t PWM_FREQUENCY = 20000; // The motor driver can handle a PWM frequency up to 20kHz
const uint16_t PWMVALUE = F_CPU / PWM_FREQUENCY / 2; // The frequency is given by F_CPU/(2*N*ICR) - where N is the prescaler, prescaling is used so the frequency is given by F_CPU/(2*ICR) - ICR = F_CPU/PWM_FREQUENCY/2

/* Counters used to count the pulses from the encoders */
volatile int32_t leftCounter = 0;
volatile int32_t rightCounter = 0;
Motor motor1;//进行一个类说明

void checkSerialData()  //实现从串口输入数据控制轮子，前，后，转
{
  if (Serial.available())
  {
    int input = Serial.read();//读取串口的数据

    if(input=='1')
    {
        motor1.moveMotor(right,forward,50);
        motor1.moveMotor(left,forward,50);
    }
    else if(input=='2')
    {
    	motor1.moveMotor(right,backward,50);
    	motor1.moveMotor(left,backward,50);
    }
    else if(input=='3') //右转
	{
		motor1.moveMotor(right,backward,50);
		motor1.moveMotor(left,forward,50);
	}
    else if(input=='4') //左转
	{
		motor1.moveMotor(right,forward,50);
		motor1.moveMotor(left,backward,50);
	}
    else
    {
    	motor1.stopMotor(left);
    	motor1.stopMotor(right);
    	Serial.print("not the correct character");
    }
  }
  //else
  //Serial.print("have no singal");
}
void setup()
{
    Serial.begin(9600);

    /* Set the motordriver diagnostic pins to inputs */
    M1EN::SetDirRead();
    M2EN::SetDirRead();
   // M1EN::SetDirWrite();
    //M2EN::SetDirWrite();


    /* Setup motor pins to output */
    PWM1A::SetDirWrite();
    M1A::SetDirWrite();
    M1B::SetDirWrite();
    PWM1B::SetDirWrite();
    M2A::SetDirWrite();
    M2B::SetDirWrite();

    DDRD|=0x30;
    /* Set PWM frequency to 20kHz - see the datasheet http://www.atmel.com/Images/Atmel-8272-8-bit-AVR-microcontroller-ATmega164A_PA-324A_PA-644A_PA-1284_P_datasheet.pdf page 129-139 */
  // Set up PWM, Phase and Frequency Correct on pin 18 (OC1A) & pin 17 (OC1B) with ICR1 as TOP using Timer1
    TCCR1B = (0<<ICNC1)|(0<<ICES1)|(1 << WGM13)|(0<<WGM12)| (0<<CS12)|(0<<CS11)|(1<<CS10); // Set PWM Phase and Frequency Correct with ICR1 as TOP and no prescaling
    ICR1 = PWMVALUE; // ICR1 is the TOP value - this is set so the frequency is equal to 20kHz

  /* Enable PWM on pin 18 (OC1A) & pin 17 (OC1B) */
  // Clear OC1A/OC1B on compare match when up-counting
  // Set OC1A/OC1B on compare match when down-counting
    TCCR1A = (1 << COM1A1) |(0<<COM1A0)| (1 << COM1B1)|(0<<COM1B0)|(0<<WGM11)|(0<<WGM10);
    M1EN::Set();
    M2EN::Set();
    Serial.println("please send command:");
    Serial.println("1 is forward,2 is backward,3 is turn right,4 is turn left");
}

void loop()
{
	//static uint16_t i=0;
	if (!M1EN::IsSet() || !M2EN::IsSet()) { // Motor driver will pull these low on error
	   //buzzer::Set();
		motor1.stopMotor(left);
		motor1.stopMotor(right);
	   while (1);
	  }

	checkSerialData();
	//motor1.moveMotor(right,forward,i++);
	//motor1.moveMotor(left,forward,i++);
	//Serial.print("PWMVALUE");
	//Serial.println(PWMVALUE);
	//Serial.println(OCR1A);
	//Serial.print('\n');


}




