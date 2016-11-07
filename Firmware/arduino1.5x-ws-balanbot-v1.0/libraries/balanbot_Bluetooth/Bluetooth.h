/*
 * Bluetooth.h
 *
 *  Created on: 2014年7月30日
 *      Author: caixiaoliang
 */

#ifndef BLUETOOTH_H_
#define BLUETOOTH_H_

#include "Arduino.h"
#include "balanbot.h"

class BluetoothN
{
public:
	void readSPPData();
	void readUsb();
	void updateLEDs();
	void onInitPS3();
	void onInitPS4();
	void onInitWii();
	void onInitXbox();
	void steer(Command command);
	double scale(double input, double inputMin, double inputMax, double outputMin, double outputMax);
};





#endif /* BLUETOOTH_H_ */
