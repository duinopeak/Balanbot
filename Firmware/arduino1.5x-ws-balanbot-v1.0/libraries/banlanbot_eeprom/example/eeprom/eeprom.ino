/*
 * eeprom.ino
 *
 *  Created on: 2014年7月22日
 *      Author: caixiaoliang
 */
#include "eepromInit.h"
#include "balanbot.h"
#include <Usb.h>
#include <Kalman.h>



cfg_t cfg1;//新建一个结构体变量
cfg_t cfg;
//const uint16_t initFlagsAddr=0;

const uint16_t eepromVersion = 2;

//const uint8_t configAddr;   //save the configuration starting from this location

uint16_t initFlagsAddr;
uint16_t configAddr;

void setup()
{
	eeprombot eeprombot1;//新建一个eeprombot对象
	Serial.begin(9600);
//	buzzer::SetDirWrite();//set up the buzzer pin 拉高蜂鸣器

	//读取PID数值，targetangle（目标角度）和其他保存在eeprom中的值
	if(!eeprombot1.checkInitializationFlags(configAddr,cfg))//在
	{
//		eeprombot1.updateConfig(configAddr,cfg);//Store the new values in the EEPROM
		eeprombot1.readEEPROMValues(configAddr,cfg);//在EEPROM地址1写入结构体重的值
		Serial.print("EEPROM initialization have done\n");

		Serial.println("here is some value stored in EEPROM,you can modify it in restoreEEPROMValues(uint16_t addr,cfg_t &m)");

		Serial.print("cfg.P=");//测试有没有真的存入EEPROM里
		Serial.print(cfg.P);   //测试通过
		Serial.print('\t');

		Serial.print("cfg.I=");
		Serial.print(cfg.I);
		Serial.print('\t');

		Serial.print("cfg.D=");
		Serial.print(cfg.D);
		Serial.print('\t');

		Serial.print("cfg.targetAngle=");
		Serial.print(cfg.targetAngle);
		Serial.print('\t');

		Serial.print("cfg.backToSpot=");
		Serial.print(cfg.backToSpot);
		Serial.print('\t');

		Serial.print("cfg.controlAngleLimit=");
		Serial.print(cfg.controlAngleLimit);
		Serial.println('\t');
		}
	else //Indicate that the EEPROM values have been reset by turing on the buzzer
	{
//		buzzer::Set();
		delay(1000);
//		buzzer::Clear();
		delay(100);//wait a little after the pin is cleared

	}
}

// The loop function is called in an endless loop
void loop()
{
//Add your repeated code here
/*
	if(eeprombot1.checkInitializationFlags(configAddr,cfg))//说明已经重载eeprom的数据了
	{
	    Serial.print("have Initialize eeprom!");
	    Serial.print("cfg.P");
	    Serial.println(cfg1.P);
	    Serial.print("cfg1.I");
	    Serial.println(cfg1.I);
	    Serial.print("cfg.D");
	    Serial.println(cfg1.D);
	}
	else
	{
		Serial.print("still Failed");
	    Serial.print("\n");
	}
*/
}

