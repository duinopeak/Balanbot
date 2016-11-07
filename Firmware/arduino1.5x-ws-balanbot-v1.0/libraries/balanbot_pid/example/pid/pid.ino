//#include<stdint.h>
#include"pid.h"
#include "balanbot.h"
#include <Usb.h>
#include <Kalman.h>
#include <encoder.h>
//cfg_t pid;
cfg_t cfg;//struct for all the configuration values//结构体所有的配置值



uint32_t timer;//用于时间计时
uint32_t pidTimer; // Timer used for the PID loop
double pitchY;
double lastError; // Store last angle error
double integratedError; // Store integrated error
double error;
double pTerm, iTerm, dTerm;
double PIDValue, PIDLeft, PIDRight;
double lastRestAngle; // Used to limit the new restAngle if it's much larger than the previous one

uint8_t batteryCounter;
double batteryVoltage;
volatile int32_t rightCounter = 0;
volatile int32_t leftCounter = 0;
bool stopped;
int32_t lastWheelPosition; // Used to calculate the wheel velocity
int32_t wheelVelocity; // Wheel velocity based on encoder readings
int32_t targetPosition; // The encoder position the robot should be at
uint32_t encoderTimer;

bool steerStop = true; // Stop by default
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

pid pid1;


void setup(){
	Serial.begin(9600);
}

void loop(){
	timer = millis();
	pid1.updatePID(178,1,0, (double)(timer - pidTimer) / 1000.0,cfg);	//该形参都是人为设定的测试条件
        Serial.print("error:");
        Serial.println(error);
    	Serial.print("integratedError:");
        Serial.println(integratedError);
    	Serial.print("iTerm:");
    	Serial.println(iTerm);
    	Serial.print("dTerm:");
    	Serial.println(dTerm);
        Serial.print("pTerm:");
        Serial.println(pTerm);
    	Serial.print("PIDValue:");
        Serial.println(PIDValue);
    
       pidTimer = timer;
}
