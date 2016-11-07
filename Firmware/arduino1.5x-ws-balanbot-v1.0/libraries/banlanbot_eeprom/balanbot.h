/*
 * balanbot.h
 *
 *  Created on: 2010年4月12日
 *      Author: caixiaoliang
 */
/* Copyright (C) 2013-2014 Kristian Lauszus, TKJ Electronics. All rights reserved.

 This software may be distributed and modified under the terms of the GNU
 General Public License version 2 (GPL2) as published by the Free Software
 Foundation and appearing in the file GPL2.TXT included in the packaging of
 this file. Please note that GPL2 Section 2[b] requires that all works based
 on this software must also be made publicly available under the terms of
 the GPL2 ("Copyleft").

 Contact information
 -------------------

 Kristian Lauszus, TKJ Electronics
 Web      :  http://www.tkjelectronics.com
 e-mail   :  kristianl@tkjelectronics.com
*/
#ifndef BALANBOT_H_
#define BALANBOT_H_

#include <Arduino.h>

#if ARDUINO < 156 // Make sure the newest version of the Arduino IDE is used
#error "Please update the Arduino IDE to version 1.5.6 at the following website: http://arduino.cc/en/Main/Software"
#endif

#include "Arduino.h"
//add your includes for the project MPU6050andMotor here

#if ARDUINO < 156 // Make sure the newest version of the Arduino IDE is used
#error "Please update the Arduino IDE to version 1.5.6 at the following website: http://arduino.cc/en/Main/Software"
#endif

#include <stdint.h> // Needed for uint8_t, uint16_t etc.
#include "Usb.h"  //this there is very important ,这里有包含了avrpins.h,重新写了一个引脚快速定义拉高拉低的函数
#include "SPP.h"//用于蓝牙控制
#include "BTD.h"
#include "usbhub.h"
//end of add your includes here
#ifdef __cplusplus
extern "C" {
#endif
void loop();
void setup();
#ifdef __cplusplus
} // extern "C"
#endif

//add your function definitions for the project MPU6050andMotor here
/* Use this to enable and disable the different options */
extern USB Usb; // This will take care of all USB communication
extern USBHub Hub; // Some dongles have a hub inside
extern BTD Btd; // This is the main Bluetooth library, it will take care of all the USB and HCI communication with the Bluetooth dongle
extern SPP SerialBT; // The SPP (Serial Port Protocol) emulates a virtual Serial port, which is supported by most computers and mobile phones


#define ENABLE_TOOLS
#define ENABLE_SPP


//eeprom address definitions
extern uint16_t initFlagsAddr;//set the first byte to the eeprom version
extern uint16_t configAddr;   //save the configuration starting from this location
/* Firmware Version Information */
extern const char *version;
extern const uint16_t eepromVersion ; // EEPROM version - used to restore the EEPROM values if the configuration struct have changed

extern bool sendIMUValues, sendSettings, sendInfo, sendStatusReport, sendPIDValues, sendPairConfirmation, sendKalmanValues; // Used to send out different values via Bluetooth

extern const uint16_t PWM_FREQUENCY; // The motor driver can handle a PWM frequency up to 20kHz
extern const uint16_t PWMVALUE; // The frequency is given by F_CPU/(2*N*ICR) - where N is the prescaler, prescaling is used so the frequency is given by F_CPU/(2*ICR) - ICR = F_CPU/PWM_FREQUENCY/2

/* Used to make commands more readable */
enum Command {
  updatePS3,
  updatePS4,
  updateWii,
  updateXbox,
  updateSpektrum,
  stop,
  forward,
  backward,
  left,
  right,
  imu,
  joystick,
};
extern Command lastCommand; // This is used set a new targetPosition

// These pins macros are defined in avrpins.h in the USB Host library. This allows to read and write directly to the port registers instead of using Arduino's slow digitalRead()/digitalWrite() functions
// The source is available here: https://github.com/felis/USB_Host_Shield_2.0/blob/master/avrpins.h
// I do this to save processing power - see this page for more information: http://www.billporter.info/ready-set-oscillate-the-fastest-way-to-change-arduino-pins/
// Also see the Arduino port manipulation guide: http://www.arduino.cc/en/Reference/PortManipulation

/* Left motor *///小车左轮子，INA,INB,和PWM三个引脚设置
#define M1A P23
#define M1B P24
#define PWM1A P18

/* Right motor *///小车右轮子，INA,INB,和PWM三个引脚设置
#define M2A P25
#define M2B P26
#define PWM1B P17

/* Pins connected to the motor drivers diagnostic pins */
#define M1EN P21 //左电机驱动芯片使能端
#define M2EN P22 //右电机驱动芯片使能端

/* Encoders */
#define leftEncoder1 P15
#define leftEncoder2 P30

#define rightEncoder1 P16
#define rightEncoder2 P31

#define leftEncoder1Pin 15 // Used for attachInterrupt
#define rightEncoder1Pin 16
#define leftEncoder2Pin 30 // Used for pin change interrupt
#define rightEncoder2Pin 31

#define PIN_CHANGE_INTERRUPT_VECTOR_LEFT PCINT0_vect // You should change these to match your pins, if you are in doubt, just comment them out to disable them
#define PIN_CHANGE_INTERRUPT_VECTOR_RIGHT PCINT0_vect

#define buzzer P5 // Buzzer used for feedback, it can be disconnected using the jumper

#define spektrumBindPin P0 // Pin used to bind with the Spektrum satellite receiver - you can use any pin while binding, but you should connect it to RX0 afterwards

#define MAKE_PIN(pin) MAKE_PIN2(pin) // Puts a P in front of the pin number, e.g. 1 becomes P1
#define MAKE_PIN2(pin) P ## pin

#define LED MAKE_PIN(LED_BUILTIN) // LED_BUILTIN is defined in pins_arduino.h in the hardware add-on

//#define VBAT A5 // Not broken out - used for battery voltage measurement

/* Counters used to count the pulses from the encoders */
extern volatile int32_t leftCounter;
extern volatile int32_t rightCounter;

extern double batteryVoltage; // Measured battery level
extern uint8_t batteryCounter; // Counter used to check if it should check the battery level

// This struct will store all the configuration values
typedef struct {
  double P, I, D; // PID variables
  double targetAngle; // Resting angle of the robot
  uint8_t backToSpot; // Set whenever the robot should stay in the same spot
  uint8_t controlAngleLimit; // Set the maximum tilting angle of the robot
  uint8_t turningLimit; // Set the maximum turning value
  double Qangle, Qbias, Rmeasure; // Kalman filter values
  double accYzero, accZzero; // Accelerometer zero values
  double gyroXzero;
  double leftMotorScaler, rightMotorScaler;
  bool bindSpektrum;
  uint16_t initFlag;//
  uint16_t eepromVersion;
} cfg_t;

extern cfg_t cfg;


extern double lastRestAngle; // Used to limit the new restAngle if it's much larger than the previous one

/* IMU Data */


//用于MPU6050计算变量如下
/*IMU Data*/
extern int16_t accX,accY,accZ;//从MPU6050得到的原始数据，这里限制为INT类型，为了达到相应精度，后期需要修改
extern int16_t gyroX,gyroY,gyroZ;

extern int16_t accXzero;   //用于调整加速度计得到的数据偏移
//int16_t accYzero=0;
//int16_t accZzero=0;

extern double gyroXzero;//用于调整陀螺仪得到的数据偏移
extern int16_t gyroYzero;
extern int16_t gyroZzero;

//int16_t gyroXoffset=-139;
//int16_t gyroYoffset=-74;
//int16_t gyroZoffset=-111;//this value is not accurate at all
extern double gyroXrate;
//double gyroYrate;
//double gyroZrate;


extern double pitchY;
extern double accXangle,accYangle,accZangle,accAngle;//Angle calculate using the accelerometer,用加速度计计算角度
extern double gyroXangle,gyroYangle,gyroZangle;//Angle calculate using the gyro ,用陀螺仪计算的角度
extern double compAngleX,compAngleY,compAngleZ;//calculate the angle using a complementary filter,用互补滤波器计算角度
extern double kalAngleX,kalAngleY,kalAngleZ;//Calculate the angle using a kalman filter

extern uint8_t i2cBuffer[8]; // Buffer for I2C data

// Results
extern double gyroRate, gyroAngle;

//下面的数据主要用于PID函数中
extern double pitch;
extern double lastError; // Store last angle error
extern double integratedError; // Store integrated error
extern double error;
extern double pTerm, iTerm, dTerm;
extern double PIDValue, PIDLeft, PIDRight;

/* Used for timing */
extern uint32_t kalmanTimer; // Timer used for the Kalman filter
extern uint32_t pidTimer; // Timer used for the PID loop
extern uint32_t imuTimer; // This is used to set a delay between sending IMU values
extern uint32_t encoderTimer; // Timer used used to determine when to update the encoder values
extern uint32_t reportTimer; // This is used to set a delay between sending report values
extern uint32_t ledTimer; // Used to update the LEDs to indicate battery level on the PS3, PS4, Wii and Xbox controllers
extern uint32_t blinkTimer; // Used to blink the built in LED, starts blinking faster upon an incoming Bluetooth request


extern uint32_t timer;//用于时间计时
/* Used to rumble controllers upon connection */
extern bool ps3RumbleEnable, wiiRumbleEnabled, ps4RumbleEnabled; // These are used to turn rumble off again on the Wiimote and PS4 controller and to turn on rumble on the PS3 controller
extern bool ps3RumbleDisable, xboxRumbleDisable; // Used to turn rumble off again on the PS3 and Xbox controller

extern bool steerStop; // Stop by default
extern bool stopped; // This is used to set a new target position after braking

extern bool layingDown; // Use to indicate if the robot is laying down

extern double targetOffset; // Offset for going forward and backward
extern double turningOffset; // Offset for turning left and right

extern char dataInput[30]; // Incoming data buffer
extern bool bluetoothData; // True if data received is from the Bluetooth connection
extern double sppData1, sppData2; // Data send via SPP connection

extern bool commandSent ; // This is used so multiple controller can be used at once

extern uint32_t receiveControlTimer;
extern const uint16_t receiveControlTimeout; // After how long time should it should prioritize the other controllers instead of the serial control

extern int32_t lastWheelPosition; // Used to calculate the wheel velocity
extern int32_t wheelVelocity; // Wheel velocity based on encoder readings
extern int32_t targetPosition; // The encoder position the robot should be at

// Variables used for Spektrum receiver
//extern uint16_t rcValue[]; // Channel values
extern bool spekConnected; // True if spektrum receiver is connected
extern uint32_t spekConnectedTimer; // Timer used to check if the connection is dropped

#define RC_CHAN_THROTTLE 0
#define RC_CHAN_ROLL     1
#define RC_CHAN_PITCH    2
#define RC_CHAN_YAW      3
#define RC_CHAN_AUX1     4
#define RC_CHAN_AUX2     5
#define RC_CHAN_AUX3     6
#if (SPEKTRUM == 2048) // 8 channels
#define RC_CHAN_AUX4     7
#endif

// Encoder values

extern const uint16_t zoneA;
extern const uint16_t zoneB;
extern const uint16_t zoneC;
extern const double positionScaleA; // One resolution is 1856 pulses per encoder
extern const double positionScaleB;
extern const double positionScaleC;
extern const double positionScaleD;
extern const double velocityScaleMove;
extern const double velocityScaleStop;
extern const double velocityScaleTurning;

/*
// Function prototypes
void readSPPData();
void readUsb();
void updateLEDs();
void onInitPS3();
void onInitPS4();
void onInitWii();
void onInitXbox();
void steer(Command command);
double scale(double input, double inputMin, double inputMax, double outputMin, double outputMax);

bool checkInitializationFlags();
void readEEPROMValues();
void updateConfig();
void restoreEEPROMValues();

uint8_t i2cWrite(uint8_t registerAddress, uint8_t data, bool sendStop);
uint8_t i2cWrite(uint8_t registerAddress, uint8_t *data, uint8_t length, bool sendStop);
uint8_t i2cRead(uint8_t registerAddress, uint8_t *data, uint8_t nbytes);

void updatePID(double restAngle, double offset, double turning, double dt);
void moveMotor(Command motor, Command direction, double speedRaw);
void stopMotor(Command motor);
void setPWM(Command motor, uint16_t dutyCycle);
void stopAndReset();
void leftEncoder();
void rightEncoder();
int32_t readLeftEncoder();
int32_t readRightEncoder();
int32_t getWheelsPosition();

void bindSpektrum();
void readSpektrum(uint8_t input);

void checkSerialData();
void printMenu();
void calibrateMotor();
void testMotorSpeed(double *leftSpeed, double *rightSpeed, double leftScaler, double rightScaler);
void calibrateAcc();
void printValues();
void setValues(char *input);
bool calibrateGyro();
bool checkMinMax(int16_t *array, uint8_t length, int16_t maxDifference);
*/

#endif /* BALANBOT_H_ */
