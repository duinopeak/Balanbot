/*
 * imu.ino
 *
 *  Created on: 2014骞�鏈�8鏃�
 *      Author: lryain
 */

// Do not remove the include below
#include "imu.h"
#include "balanbot.h"
#include <Wire.h>
#include "Kalman.h"
#include "MPU6050.H"
#include "I2Cdev.h"
#include "Usb.h"
#include <Arduino.h>

Kalman kalmanX;//建立卡尔曼对象
Kalman kalmanY;
Kalman kalmanZ;
MPU6050 accelgyro;

/*IMU Data*/
int16_t accX,accY,accZ;//从MPU6050得到的原始数据，这里限制为INT类型，为了达到相应精度，后期需要修改
int16_t gyroX,gyroY,gyroZ;

int16_t accXzero=0;   //用于调整加速度计得到的数据偏移
int16_t accYzero=0;
int16_t accZzero=0;

double gyroXzero=0; //用于调整陀螺仪得到的数据偏移
int16_t gyroYzero=0;
int16_t gyroZzero=0;
uint32_t kalmanTimer; // Timer used for the Kalman filter用于卡尔曼滤波的处理时间
uint32_t timer;   //用于读取micro()，系统的运行时间

//int16_t gyroXoffset=-139;
//int16_t gyroYoffset=-74;
//int16_t gyroZoffset=-111;//this value is not accurate at all
double gyroXrate;
double gyroYrate;
double gyroZrate;


double pitchY;
double accXangle,accYangle,accZangle,accAngle;//Angle calculate using the accelerometer,用加速度计计算角度
double gyroXangle,gyroYangle,gyroZangle;//Angle calculate using the gyro ,用陀螺仪计算的角度
double compAngleX,compAngleY,compAngleZ;//calculate the angle using a complementary filter,用互补滤波器计算角度
double kalAngleX,kalAngleY,kalAngleZ;//Calculate the angle using a kalman filter



void setup()
{
	Serial.begin(9600);
	Wire.begin();
	Serial.println("Initializing I2C device.....");
	accelgyro.initialize();
	Serial.println("Testing device connections...");
	Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful":"MPU6050 connection failure");

	accelgyro.getMotion6(&accX,&accY,&accZ,&gyroX,&gyroY,&gyroZ);//获取三个轴的加速度和角速度
	accYangle = (atan2((double)accY - accYzero, (double)accZ -accZzero) + PI) * RAD_TO_DEG;// We then convert it to 0 to 2π and then from radians to degrees

	kalmanY.setAngle(accYangle); // Set starting angle//用加速度计测出对于Y轴的角度，进行卡尔曼滤波
	pitchY = accYangle;
	gyroYangle = accYangle;
	kalmanTimer = micros();//Timer used for the Kalman filter
	//accX = accX/16384.00;
	//accY = accY/16384.00;
	//accZ = accZ/16384.00;

	//accYangle=(atan2(accX,accZ)+PI)*RAD_TO_DEG;/另外一种转换方法，转换成0-2π的弧度值
	//accXangle=(atan2(accY,accZ)+PI)*RAD_TO_DEG;
	//accZangle=(atan2(accX,accY)+PI)*RAD_TO_DEG;

	//kalmanX.setAngle(accXangle);//set starting angle
	//kalmanY.setAngle(accYangle);
	//kalmanZ.setAngle(accZangle);

	//gyroXangle=accXangle;
	//gyroYangle=accYangle;
	//gyroZangle=accZangle;

	//compAngleX=accXangle;
	//compAngleY=accYangle;
	//compAngleZ=accZangle;


}

void loop(){
	/*update all the values*/
	accelgyro.getMotion6(&accX,&accY,&accZ,&gyroX,&gyroY,&gyroZ);//获取三个轴的加速度和角速度
	//atan 2 outputs the value od -π to π (radians),将值转换为弧度值
	//We then convert it to 0 to 2π and then from radians to degrees
	//note:if we only consider the angle ,we don't need transfrom the raw acceleration to its physical meaning
	//the formulation transforming to its physical meaning (uint :g)accX=accX/16384.00//转换成我们所认知的加速度

	accYangle = (atan2((double)accY - accYzero, (double)accZ - accZzero) + PI) * RAD_TO_DEG;

	timer = micros();
	  // This fixes the 0-360 transition problem when the accelerometer angle jumps between 0 and 360 degrees
	  if ((accYangle < 90 && pitchY > 270) || (accYangle > 270 && pitchY < 90)) {
	    kalmanY.setAngle(accYangle);
	    pitchY = accYangle;
	    gyroXangle = accYangle;
	  } else {
	    gyroXrate = ((double)gyroX - gyroXzero) / 131.0; // Convert to deg/s//求得对于X轴的旋转角速度转换成
	    double dt = (double)(timer - kalmanTimer) / 1000000.0;
	    gyroXangle += gyroXrate * dt; // Gyro angle is only used for debugging
	    if (gyroXangle < 0 || gyroXangle > 360)
	      gyroXangle = pitchY; // Reset the gyro angle when it has drifted too much
	    pitchY = kalmanY.getAngle(accYangle, gyroXrate, dt); // Calculate the angle using a Kalman filter
	  }
	  kalmanTimer = timer;
	  //Serial.print(accAngle);Serial.print('\t');Serial.print(gyroAngle);Serial.print('\t');Serial.println(pitch);

	//Serial.print(accY);Serial.print(",");
	//Serial.print(accZ);Serial.print(",");
	//Serial.print(accY);Serial.print(",");//输出各个轴的加速度

	//accX = accX/16384.00;//这个发现目前没有开启输出，没有波形
	//accY = accY/16384.00;//找到了不能正常输出波形的原因，accX,accY,accZ定义为整形数据，只能输出0,1,2等，把小数滤掉了
	//accZ = accZ/16384.00;



	//accXangle=(atan2(accY,accZ)+PI)*RAD_TO_DEG;//将上面加速度计得到的加速度，转换成成角度
    //accYangle=(atan2(accX,accZ)+PI)*RAD_TO_DEG;
    //accZangle=(atan2(accX,accY)+PI)*RAD_TO_DEG;

    //gyroXrate=(double)gyroX/131.0;//circular velocity in X direction //在X轴上的旋转速度
    //gyroYrate=(double)gyroY/131.0;//y轴的旋转速度
    //gyroZrate=(double)gyroZ/131.0;//Z轴的旋转速度

    //gyroXangle +=gyroXrate*((double)(micros()-timer)/1000000);//calculate gyro angle without any filter//不用滤波陀螺仪计算出旋转 的角度
    //gyroYangle +=gyroYrate*((double)(micros()-timer)/1000000);
    //gyroZangle +=gyroZrate*((double)(micros()-timer)/1000000);

    //compAngleX=(0.93*(compAngleX+(gyroXrate*(double)(micros()-timer)/1000000)))+(0.07*accXangle);//calculate the angle using a complimentary filter //用互补滤波计算角度
    //compAngleY=(0.93*(compAngleY+(gyroYrate*(double)(micros()-timer)/1000000)))+(0.07*accYangle);//calculate the angle using a complimentary filter //用互补滤波计算角度
    //compAngleZ=(0.93*(compAngleZ+(gyroZrate*(double)(micros()-timer)/1000000)))+(0.07*accYangle);//calculate the angle using a complimentary filter //用互补滤波计算角度

    //kalAngleX=kalmanX.getAngle(accXangle,gyroXrate,(double)(micros()-timer)/100000);//calculate the angle uising kalman filter //用卡尔曼滤波计算角度
    //kalAngleY=kalmanY.getAngle(accYangle,gyroYrate,(double)(micros()-timer)/100000);//calculate the angle uising kalman filter //用卡尔曼滤波计算角度
    //kalAngleZ=kalmanZ.getAngle(accZangle,gyroZrate,(double)(micros()-timer)/100000);//calculate the angle uising kalman filter //用卡尔曼滤波计算角度


    //temp=((double)tempRaw+12412.0)/340.0;//计算温度

    /*Print Date*/
    //Serial.print(accX);Serial.print(",");//输出各个轴的加速度
    //Serial.print(accY);Serial.print(",");
    //Serial.print(accZ);Serial.print(",");

   // Serial.print(gyroX);Serial.print(",");
   //Serial.print(gyroY);Serial.print(",");
   // Serial.print(gyroZ);Serial.print(",");

    //Serial.print(gyroXrate);Serial.print(",");//各个轴的旋转角速度
   // Serial.print(gyroYrate);Serial.print(",");
   // Serial.print(gyroZrate);Serial.print(",");

   // Serial.print(accXangle);Serial.print(",");//由加速度计得到的角度
    //Serial.print(gyroXangle);Serial.print(",");//由陀螺仪得到的角度
    //Serial.print(compAngleX);Serial.print(",");//互补滤波后得到的角度
    //Serial.print(kalAngleX);Serial.print(","); //卡曼滤波得到的角度

    Serial.print(accYangle);Serial.print(",");//由加速度计得到的角度
	Serial.print(gyroXangle);Serial.print(",");//由陀螺仪得到的角度
	//Serial.print(compAngleY);Serial.print(",");//互补滤波后得到的角度
	//Serial.print(kalAngleY);Serial.print(","); //卡曼滤波得到的角度

    //Serial.print(accZangle);Serial.print(",");//由加速度计得到的角度
	//Serial.print(gyroZangle);Serial.print(",");//由陀螺仪得到的角度
	//Serial.print(compAngleZ);Serial.print(",");//互补滤波后得到的角度
	//Serial.print(kalAngleZ);Serial.print(","); //卡曼滤波得到的角度

    //Serial.print(temp);Serial.print(",");
	//Serial.print("pitchY");
	Serial.print(pitchY);Serial.print(",");
    Serial.print("\n");

}
