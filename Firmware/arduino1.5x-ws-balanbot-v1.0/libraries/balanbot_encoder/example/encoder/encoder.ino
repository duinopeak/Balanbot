/*
 * encoder.ino
 *
 *  Created on: 2014年8月8日
 *      Author: lryain
 */
#include <Usb.h>
#include <Kalman.h>
#include <encoder.h>
#include <balanbot.h>
encoder encoders;
uint8_t batteryCounter;
double batteryVoltage;
volatile int32_t rightCounter = 0;
volatile int32_t leftCounter = 0;
bool stopped;
int32_t lastWheelPosition; // Used to calculate the wheel velocity
int32_t wheelVelocity; // Wheel velocity based on encoder readings
int32_t targetPosition; // The encoder position the robot should be at
uint32_t encoderTimer;
uint32_t timer;

// Encoder values
#if defined(PIN_CHANGE_INTERRUPT_VECTOR_LEFT) && defined(PIN_CHANGE_INTERRUPT_VECTOR_RIGHT)
const uint16_t zoneA = 8000 * 2;
const uint16_t zoneB = 4000 * 2;
const uint16_t zoneC = 1000 * 2;
const double positionScaleA = 600 * 2; // One resolution is 1856 pulses per encoder
const double positionScaleB = 800 * 2;
const double positionScaleC = 1000 * 2;
const double positionScaleD = 500 * 2;
const double velocityScaleMove = 70 * 2;
const double velocityScaleStop = 60 * 2;
const double velocityScaleTurning = 70 * 2;
#else
const uint16_t zoneA = 8000;
const uint16_t zoneB = 4000;
const uint16_t zoneC = 1000;
const double positionScaleA = 600; // One resolution is 928 pulses per encoder
const double positionScaleB = 800;
const double positionScaleC = 1000;
const double positionScaleD = 500;
const double velocityScaleMove = 70;
const double velocityScaleStop = 60;
const double velocityScaleTurning = 70;
#endif



void setup(){
	Serial.begin(115200);
	encoders.setupTiming();     //定时器初始化
	encoders.setupEncoders();	//编码器初始化
	setInterrupt();				//设置中断
}
void loop(){
	encoders.updateEncoders();
        Serial.println("update encoders");
        Serial.print("rightCounter");
        Serial.println(rightCounter);
        Serial.print("leftCounter");
        Serial.println(leftCounter);
        
}
