// Only modify this file to include
// - function definitions (prototypes)
// - include files
// - extern variable definitions
// In the appropriate section


#ifndef _tools_H_
#define _tools_H_
#include "Arduino.h"
//add your includes for the project tools here
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

//add your function definitions for the project tools here

class Tool //用于校准测试和菜单命�?
{

	public:
	//Tool();
	void checkSerialData();//用于实时检测用户输入数�?
	void printMenu();//用于显示各种命令菜单
	void calibrateAcc(cfg_t &m);//加速度计校�?
	bool calibrateGyro(cfg_t &m);//用于陀螺仪角度校准
	bool checkMinMax(int16_t *array,uint8_t length,int16_t maxDifference);//used to check to the robot is laying still while calibrating
    void calibrateMotor();//校准电机函数，调用下面的testMotorSpeed函数
	void testMotorSpeed(double *leftSpeed,double *rightSpeed,double leftScaler,double rightScaler);
    void setValues(char *input);
    void printValues();//蓝牙模块
};



//Do not add code below this line
#endif /* _tools_H_ */
