/*
 * eepromInit.h
 *
 *  Created on: 2010年4月12日
 *      Author: caixiaoliang
 */

#ifndef EEPROMINIT_H_
#define EEPROMINIT_H_

#include "Arduino.h"
#include "balanbot.h"
/*
const uint8_t eepromVersion = 2; // EEPROM version - used to restore the EEPROM values if the configuration struct have changed
//eeprom address definitions
const uint8_t initFlagsAddr=0;//set the first byte to the eeprom version
const uint8_t configAddr=1;   //save the configuration starting from this location

typedef struct
{
	double P,I,D; //PID vriables PID状态值
	double targetAngle;//resting angle of the robot
	uint8_t backToSpot;//set whenever the robot should stay in the same spot
	uint8_t controlAngleLimit;//set the maximum tilting angle of the robot
	uint8_t turningLimit;//set the maximum turning value
	double Qangle,Qbias,Rmeasure;//Kalman filter values
	double accYzero,accZzero;//accelerometer zero values
	double leftMotorScaler,rightMotorScaler;
	bool bindSpektrum;
}cfg_t;

//extern cfg_t cfg;
*/
class eeprombot
{
   public:
	//eeprombot(){};
	bool checkInitializationFlags(uint16_t addr,cfg_t &m);
	void readEEPROMValues(uint16_t addr,cfg_t &m);
    void updateConfig(uint16_t addr,cfg_t &m);
    void restoreEEPROMValues(uint16_t addr,cfg_t &m);
    uint16_t EEPROM_writeAnything(uint16_t addr,cfg_t &m);
    uint16_t EEPROM_updateAnything(uint16_t addr,cfg_t &m);
    uint16_t EEPROM_readAnything(uint16_t addr,cfg_t &m);
    void EEPROM_readAnything2(uint16_t addr ,uint16_t value);//这个函数用于在指定的地址写数值
    void EEPROM_updateAnything2(uint16_t addr,uint16_t value);


};

#endif /* EEPROMINIT_H_ */
