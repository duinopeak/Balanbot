/*
 * eeprombot.cpp
 *
 *  Created on: 2010年4月12日
 *      Author: caixiaoliang
 */

#include "eepromInit.h"
#include "kalman.h"
#include <avr/eeprom.h>
//#include "balanbot.h"

void eeprombot::updateConfig(uint16_t addr,cfg_t &m)
{
	Kalman kalman;//新建一个kalmand对象
	EEPROM_updateAnything(addr,m);
	kalman.setQangle(m.Qangle);
	kalman.setQbias(m.Qbias);
	kalman.setRmeasure(m.Rmeasure);
}

void eeprombot::restoreEEPROMValues(uint16_t addr,cfg_t &m)
	{
		m.P=9.0;
		m.I=2.0;
		m.D=3.0;

		m.targetAngle = 180.0;
	    m.backToSpot = 1;
	    m.controlAngleLimit = 7;
	    m.turningLimit = 25;

	    m.Qangle = 0.001;
	    m.Qbias = 0.003;
	    m.Rmeasure = 0.03;

	    m.accYzero = m.accZzero = 0;
	    m.gyroXzero=0;
	    m.leftMotorScaler = m.rightMotorScaler = 1;

	    m.bindSpektrum = false;

	    updateConfig(addr,m);
	}


bool eeprombot::checkInitializationFlags(uint16_t addr,cfg_t &m)
{
    // uint16_t initFlag;
	EEPROM_readAnything2(initFlagsAddr,m.initFlag);
    if(m.initFlag!=m.eepromVersion) //check if the  eeprom version matches the current one 测试EEPROM版本能否匹配现在的版本
	{
		restoreEEPROMValues(addr,m);
		EEPROM_updateAnything2(initFlagsAddr,m.eepromVersion);//After the default values have been restored ,set the flag//在默认版本加载后，置一个标志位
		Serial.print("eeprom Initlization successed!");
		return true;
	}


	/*下面的用于测试专用，上面的用于实际开发
	if(initFlag==eepromVersion) //check if the  eeprom version matches the current one 测试EEPROM版本能否匹配现在的版本
		{
			restoreEEPROMValues();
			EEPROM_updateAnything(initFlagsAddr,eepromVersion);//After the default values have been restored ,set the flag//在默认版本加载后，置一个标志位
			return true;
		}
		*/
	return false;

}

void eeprombot::readEEPROMValues(uint16_t addr,cfg_t &m)
{
	Kalman kalman1;//新建一个kalmand对象,类函数调用类函数需要 这样做
	EEPROM_readAnything(addr,m);
    kalman1.setQangle(m.Qangle);
    kalman1.setQbias(m.Qbias);
    kalman1.setRmeasure(m.Rmeasure);
}


 uint16_t eeprombot::EEPROM_writeAnything(uint16_t addr,cfg_t &m)
{
	eeprom_busy_wait();//wait until the EEPROM is  ready //等待eeprom 初始化
	const uint8_t *p=(const uint8_t*)(const void*)&m;
	uint16_t i;
	for(i=0;i<sizeof(m);i++)
		eeprom_write_byte((uint8_t*)addr++,*p++);
	return i;
}

 uint16_t  eeprombot::EEPROM_updateAnything(uint16_t addr,cfg_t &m)
{
    eeprom_busy_wait();//wait until the eeprom is ready
    const uint8_t *p=(const uint8_t*)(const void*)&m;
    //const uint8_t *p=(const uint8_t*)&m;
    uint16_t i;
    for(i=0;i<sizeof(m);i++)
    {
    	if(eeprom_read_byte((uint8_t*)addr)!=*p)//limit number of write/erase cycles//如果不同则擦除重新写入
    	    eeprom_write_byte((uint8_t*)addr,*p);
    	    addr++;
    	    p++;

    }
    return i;
}

uint16_t eeprombot::EEPROM_readAnything(uint16_t addr,cfg_t &m)
{
	 eeprom_busy_wait();//wait until eeprom is ready
	uint8_t *p=(uint8_t*)(void*)&m;
	uint16_t i;
	for(i=0;i<sizeof(m);i++)
	{
	 *p++=eeprom_read_byte((uint8_t*)addr++);
	}
	return i;
}

void eeprombot::EEPROM_readAnything2(uint16_t addr,uint16_t value)
{
	eeprom_busy_wait();//wait until eeprom is ready
	uint16_t *p=&value;
	uint16_t i;
	for(i=0;i<sizeof(value);i++)
		{
		 *p++=eeprom_read_byte((uint8_t*)addr++);
		}
}

void eeprombot::EEPROM_updateAnything2(uint16_t addr,uint16_t value)
{
	 eeprom_busy_wait();//wait until the eeprom is ready
	 const uint16_t *p=&value;
	    uint16_t i;
	    for(i=0;i<sizeof(value);i++)
	    {
	    	if(eeprom_read_byte((uint8_t*)addr)!=*p)//limit number of write/erase cycles//如果不同则擦除重新写入
	    	    eeprom_write_byte((uint8_t*)addr,*p);
	    	    addr++;
	    	    p++;

	    }
}






