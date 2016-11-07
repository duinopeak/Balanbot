/*
 * balanbot_test1.ino
 *
 *  Created on: 2014年7月24日
 *      Author: caixiaoliang
 */

#include "Kalman.h"
#include "Wire.h"
//#include "Arduino.h"


//#include "../balanbot.h"
#include "MPU6050.h"
#include "pid.h"
#include "motor.h"
#include "I2Cdev.h"
#include "encoder.h"
#include "eepromInit.h"
#include "tools.h"
#include "Bluetooth.h"

//USB Usb;
//USBHub Hub(&Usb); // Some dongles have a hub inside
//BTD Btd(&Usb); // This is the main Bluetooth library, it will take care of all the USB and HCI communication with the Bluetooth dongle
//SPP SerialBT(&Btd, "Balanbot", "0000"); // The SPP (Serial Port Protocol) emulates a virtual Serial port, which is supported by most computers and mobile phones

//#include <usbhub.h> // Some dongles can have a hub inside
//USB Usb; // This will take care of all USB communication
//USBHub Hub(&Usb); // Some dongles have a hub inside
//BTD Btd(&Usb); // This is the main Bluetooth library, it will take care of all the USB and HCI communication with the Bluetooth dongle
//SPP SerialBT(&Btd, "Balanduino", "0000"); // The SPP (Serial Port Protocol) emulates a virtual Serial port, which is supported by most computers and mobile phones


#if defined(ENABLE_SPP) || defined(ENABLE_PS3) || defined(ENABLE_PS4) || defined(ENABLE_WII) || defined(ENABLE_XBOX) || defined(ENABLE_ADK)
#define ENABLE_USB
USB Usb; // This will take care of all USB communication
#else
#define _usb_h_ // Workaround include trap in the USB Host library
#include <avrpins.h> // Include this from the USB Host library
#endif

#if defined(ENABLE_SPP) || defined(ENABLE_PS3) || defined(ENABLE_PS4) || defined(ENABLE_WII)
#define ENABLE_BTD
#include <usbhub.h> // Some dongles can have a hub inside
USBHub Hub(&Usb); // Some dongles have a hub inside
BTD Btd(&Usb); // This is the main Bluetooth library, it will take care of all the USB and HCI communication with the Bluetooth dongle
#endif

#ifdef ENABLE_SPP
SPP SerialBT(&Btd, "BalanbotC", "1234"); // The SPP (Serial Port Protocol) emulates a virtual Serial port, which is supported by most computers and mobile phones
#endif


double lastRestAngle; // Used to limit the new restAngle if it's much larger than the previous one
uint16_t initFlagsAddr=0;//set the first byte to the eeprom version
uint16_t configAddr=1;   //save the configuration starting from this location
bool sendIMUValues, sendSettings, sendInfo, sendStatusReport, sendPIDValues, sendPairConfirmation, sendKalmanValues; // Used to send out different values via Bluetooth

const uint16_t PWM_FREQUENCY = 20000; // The motor driver can handle a PWM frequency up to 20kHz
const uint16_t PWMVALUE = F_CPU / PWM_FREQUENCY / 2; // The frequency is given by F_CPU/(2*N*ICR) - where N is the prescaler, prescaling is used so the frequency is given by F_CPU/(2*ICR) - ICR = F_CPU/PWM_FREQUENCY/2
/* Firmware Version Information */
const uint16_t eepromVersion = 2; // EEPROM version - used to restore the EEPROM values if the configuration struct have changed

 /* Counters used to count the pulses from the encoders */
volatile int32_t leftCounter = 0;
volatile int32_t rightCounter = 0;
/* Firmware Version Information */
const char *version = "1.1.0";


double batteryVoltage; // Measured battery level
uint8_t batteryCounter; // Counter used to check if it should check the battery level

/* IMU Data */
//用于MPU6050计算变量如下
/*IMU Data*/
int16_t accX,accY,accZ;//从MPU6050得到的原始数据，这里限制为INT类型，为了达到相应精度，后期需要修改
int16_t gyroX,gyroY,gyroZ;

//int16_t accXzero;   //用于调整加速度计得到的数据偏移
//int16_t accYzero=0;
//int16_t accZzero=0;

double gyroXzero;//用于调整陀螺仪得到的数据偏移,这里没用到，因为我把原来的gyroXzero放到cfg_t 结构体中了
int16_t gyroYzero=0;
int16_t gyroZzero=0;
int16_t accXzero=0;   //用于调整加速度计得到的数据偏移

//int16_t gyroXoffset=-139;
//int16_t gyroYoffset=-74;
//int16_t gyroZoffset=-111;//this value is not accurate at all
double gyroXrate;
//double gyroYrate;
//double gyroZrate;


double pitchY;//因为原来的pitch，不能较好的说明这个事什么倾角，用pitchY表示Y轴的倾角
double accXangle,accYangle,accZangle,accAngle;//Angle calculate using the accelerometer,用加速度计计算角度
double gyroXangle,gyroYangle,gyroZangle;//Angle calculate using the gyro ,用陀螺仪计算的角度
double compAngleX,compAngleY,compAngleZ;//calculate the angle using a complementary filter,用互补滤波器计算角度
double kalAngleX,kalAngleY,kalAngleZ;//Calculate the angle using a kalman filter

uint8_t i2cBuffer[8]; // Buffer for I2C data

// Results
double gyroRate, gyroAngle;

//下面的数据主要用于PID函数中
double lastError; // Store last angle error
double integratedError; // Store integrated error
double error;
double pTerm, iTerm, dTerm;
double PIDValue, PIDLeft, PIDRight;

/* Used for timing */
uint32_t kalmanTimer; // Timer used for the Kalman filter
uint32_t pidTimer; // Timer used for the PID loop
uint32_t imuTimer; // This is used to set a delay between sending IMU values
uint32_t encoderTimer; // Timer used used to determine when to update the encoder values
uint32_t reportTimer; // This is used to set a delay between sending report values
uint32_t ledTimer; // Used to update the LEDs to indicate battery level on the PS3, PS4, Wii and Xbox controllers
uint32_t blinkTimer; // Used to blink the built in LED, starts blinking faster upon an incoming Bluetooth request


uint32_t timer;//用于时间计时
/* Used to rumble controllers upon connection */
bool ps3RumbleEnable, wiiRumbleEnabled, ps4RumbleEnabled; // These are used to turn rumble off again on the Wiimote and PS4 controller and to turn on rumble on the PS3 controller
bool ps3RumbleDisable, xboxRumbleDisable; // Used to turn rumble off again on the PS3 and Xbox controller

bool steerStop = true; // Stop by default
bool stopped; // This is used to set a new target position after braking

bool layingDown = true; // Use to indicate if the robot is laying down

double targetOffset = 0; // Offset for going forward and backward
double turningOffset = 0; // Offset for turning left and right

char dataInput[30]; // Incoming data buffer
bool bluetoothData; // True if data received is from the Bluetooth connection
double sppData1, sppData2; // Data send via SPP connection


bool commandSent = false; // This is used so multiple controller can be used at once

uint32_t receiveControlTimer;
const uint16_t receiveControlTimeout = 500;

int32_t lastWheelPosition; // Used to calculate the wheel velocity
int32_t wheelVelocity; // Wheel velocity based on encoder readings
int32_t targetPosition; // The encoder position the robot should be at

// Variables used for Spektrum receiver
//uint16_t rcValue[]; // Channel values
bool spekConnected; // True if spektrum receiver is connected
uint32_t spekConnectedTimer; // Timer used to check if the connection is dropped
/////////////////////////////////



//uint16_t configAddr=1;   //save the configuration starting from this location
#if defined(PIN_CHANGE_INTERRUPT_VECTOR_LEFT) && defined(PIN_CHANGE_INTERRUPT_VECTOR_RIGHT)
const uint16_t zoneA = 8000 * 2;
const uint16_t zoneB = 4000 * 2;
const uint16_t zoneC = 1000 * 2;
const double positionScaleA = 600 * 2; // One resolution is 1856 pulses per encoder
const double positionScaleB = 800 * 2;
const double positionScaleC = 1000 * 2;
const double positionScaleD = 500 * 2;
const double velocityScaleMove = 70* 2;
const double velocityScaleStop =60* 2;
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





Kalman kalmanY;//建立卡尔曼对象
MPU6050 accelgyro;//建立MPU6050对象
Tool tool1;//定义一个Tool类对象，用于下面的校准测试
Motor motor1;//进行一个电机驱动类说明
encoder encoders;//进行编码器的类说明
eeprombot eeprombot1;//建立第一个eeprombot实例
BluetoothN bluetooth1;
pid pid1;
cfg_t cfg;
Command lastCommand;
/*IMU Data*/

//x:预定角度 相当于setpoint  y:预定偏差  z：转向偏差  m:(timer-pidtimer)/1000000
// updatePID(cfg.targetAngle, targetOffset, turningOffset, (double)(timer - pidTimer) / 1000000.0);  //bluetooth  6050  i2c  motor  velocity  encoder
void updatePID(double x,double y,double z, double dt)
{

	     encoder encoderP;
	     if(steerStop)//这个是小车没有用遥控，处于自动平衡静止的状态,具有回到原位的功能
	     {
	         int32_t wheelPosition=encoderP.getWheelsPosition();
	         int32_t positionError=wheelPosition-targetPosition;
	         //Serial.print("wheelPosition");
	         //Serial.println(wheelPosition);
	         if(cfg.backToSpot)
	         {

	        	      if (abs(positionError) > zoneA) // Inside zone A
	        	       x -= (double)positionError / positionScaleA;
	        	      else if (abs(positionError) > zoneB) // Inside zone B
	        	       x -= (double)positionError / positionScaleB;
	        	      else if (abs(positionError) > zoneC) // Inside zone C
	        	       x -= (double)positionError / positionScaleC;
	        	      else // Inside zone D
	        	       x -= (double)positionError / positionScaleD;

	         }

			 else
			 {
			   if (abs(positionError) < zoneC)
				 x -= (double)positionError / positionScaleD;
			   else
				 targetPosition = wheelPosition;

			 }

	        x -= (double)wheelVelocity / velocityScaleStop;

	        x = constrain(x, cfg.targetAngle - 10, cfg.targetAngle + 10); // Limit rest Angle

	       }

	     //小车向前向后行驶,用手机蓝牙控制方向
	     else
	    {
	    	 if ((y > 0 && wheelVelocity < 0) || (y < 0 && wheelVelocity > 0) || y == 0) // Scale down offset at high speed - wheel velocity is negative when driving forward and positive when driving backward
	    	       y += (double)wheelVelocity/velocityScaleMove; // We will always compensate if the offset is 0, but steerStop is not set
	    	     x -= y;

	     }

	     x=constrain(x,lastRestAngle-1,lastRestAngle+1);

	     lastRestAngle=x;
     	 error = (x - pitchY);
     	 integratedError += error * dt;	//误差*dt 对时间积分    Ki Term

     	 pTerm =1.0*cfg.P*error;//误差*Kp		  Kp Term
     	 iTerm = (cfg.I*40.0)*constrain(integratedError,-1.0,1.0); // Limit the integrated error
     	 dTerm = (cfg.D/40.0)*(error-lastError) / dt;
     	 lastError =error;
     	 PIDValue = pTerm + iTerm + dTerm;

     	if (z< 0) { // Left
     	    z += abs((double)wheelVelocity/velocityScaleTurning); // Scale down at high speed
     	    if (z > 0)
     	      z = 0;
     	  }
     	  else if (z > 0) { // Right
     	    z -= abs((double)wheelVelocity / velocityScaleTurning); // Scale down at high speed
     	    if (z < 0)
     	      z= 0;
     	  }

     	  PIDLeft = PIDValue + z;
     	  PIDRight = PIDValue - z;

     	  PIDLeft *= cfg.leftMotorScaler; // Compensate for difference in some of the motors
     	  PIDRight *= cfg.rightMotorScaler;

}
void setup()
{
	Serial.begin(115200);
	buzzer::SetDirWrite();//set up the buzzer pin 拉高蜂鸣器

	//读取PID数值，targetangle（目标角度）和其他保存在eeprom中的值
	if(!eeprombot1.checkInitializationFlags(configAddr,cfg))//在
	{
		eeprombot1.readEEPROMValues(configAddr,cfg);//在EEPROM地址1写入结构体重的值

		//Serial.print("EEPROM initialization have done\n");
		//Serial.print("cfg.P,cfg.I,cfg.D\n");//测试有没有真的存入EEPROM里
		//Serial.println(cfg.P);              //测试通过
		//Serial.print(",");
		//Serial.println(cfg.I);
		//Serial.print(",");
		//Serial.println(cfg.D);
		//Serial.print("\n");

	}
	else //Indicate that the EEPROM values have been reset by turing on the buzzer
	{
		buzzer::Set();
		delay(1000);
		buzzer::Clear();
		delay(100);//wait a little after the pin is cleared

	}


	//Initializa uart初始化Uart通信

	 leftEncoder1::SetDirRead();//上拉编码器引脚
	 leftEncoder2::SetDirRead();
	 rightEncoder1::SetDirRead();
	 rightEncoder2::SetDirRead();

	 leftEncoder1::Set(); // Enable pull-ups//初始化
	 leftEncoder2::Set();
	 rightEncoder1::Set();
	 rightEncoder2::Set();

	 attachInterrupt(digitalPinToInterrupt(leftEncoder1Pin), leftEncoder, CHANGE);
	 attachInterrupt(digitalPinToInterrupt(rightEncoder1Pin), rightEncoder, CHANGE);

	#if defined(PIN_CHANGE_INTERRUPT_VECTOR_LEFT) && defined(PIN_CHANGE_INTERRUPT_VECTOR_RIGHT)
	  /* Enable encoder pins interrupt sources */
	  *digitalPinToPCMSK(leftEncoder2Pin) |= (1 << digitalPinToPCMSKbit(leftEncoder2Pin));
	  *digitalPinToPCMSK(rightEncoder2Pin) |= (1 << digitalPinToPCMSKbit(rightEncoder2Pin));

	  /* Enable pin change interrupts */
	  *digitalPinToPCICR(leftEncoder2Pin) |= (1 << digitalPinToPCICRbit(leftEncoder2Pin));
	  *digitalPinToPCICR(rightEncoder2Pin) |= (1 << digitalPinToPCICRbit(rightEncoder2Pin));
	#endif

/*
	 //attachInterrupt(digitalPinToInterrupt(leftEncoder1Pin), leftEncoder, CHANGE);//编码器引脚中断
	 //attachInterrupt(digitalPinToInterrupt(rightEncoder1Pin), rightEncoder, CHANGE);//编码器引脚中断

	 //下面是设置另外的编码器端口为中断，也就是说共用了4个中断
#if defined(PIN_CHANGE_INTERRUPT_VECTOR_LEFT) && defined(PIN_CHANGE_INTERRUPT_VECTOR_RIGHT)
  /* Enable encoder pins interrupt sources */
  //*digitalPinToPCMSK(leftEncoder2Pin) |= (1 << digitalPinToPCMSKbit(leftEncoder2Pin));
  //*digitalPinToPCMSK(rightEncoder2Pin) |= (1 << digitalPinToPCMSKbit(rightEncoder2Pin));

  /* Enable pin change interrupts */
  //*digitalPinToPCICR(leftEncoder2Pin) |= (1 << digitalPinToPCICRbit(leftEncoder2Pin));
  //*digitalPinToPCICR(rightEncoder2Pin) |= (1 << digitalPinToPCICRbit(rightEncoder2Pin));
//#endif

  /* Set the motordriver diagnostic pins to inputs 电机配置使能引脚*/
      M1EN::SetDirRead();
      M2EN::SetDirRead();

      /* Setup motor pins to output配置INX和PWM引脚 */
      PWM1A::SetDirWrite();
      M1A::SetDirWrite();
      M1B::SetDirWrite();
      PWM1B::SetDirWrite();
      M2A::SetDirWrite();
      M2B::SetDirWrite();

      /* Set PWM frequency to 20kHz - see the datasheet http://www.atmel.com/Images/Atmel-8272-8-bit-AVR-microcontroller-ATmega164A_PA-324A_PA-644A_PA-1284_P_datasheet.pdf page 129-139 */
       // Set up PWM, Phase and Frequency Correct on pin 18 (OC1A) & pin 17 (OC1B) with ICR1 as TOP using Timer1
      TCCR1B = (0<<ICNC1)|(0<<ICES1)|(1 << WGM13)|(0<<WGM12)| (0<<CS12)|(0<<CS11)|(1<<CS10); // Set PWM Phase and Frequency Correct with ICR1 as TOP and no prescaling
      ICR1 = PWMVALUE; // ICR1 is the TOP value - this is set so the frequency is equal to 20kHz

        /* Enable PWM on pin 18 (OC1A) & pin 17 (OC1B) */
        // Clear OC1A/OC1B on compare match when up-counting
        // Set OC1A/OC1B on compare match when down-counting
       TCCR1A = (1 << COM1A1) |(0<<COM1A0)| (1 << COM1B1)|(0<<COM1B0)|(0<<WGM11)|(0<<WGM10);
       //M1EN::Set();
       //M2EN::Set();
#ifdef ENABLE_USB
  if (Usb.Init() == -1) { // Check if USB Host is working
    Serial.print(F("OSC did not start"));
    buzzer::Set();
    while (1); // Halt
  }
#endif
       Wire.begin();//启动I2C通信
       Serial.println("Initializing I2C device.....");
       accelgyro.initialize();
       Serial.println("Testing device connections...");
       Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful":"MPU6050 connection failure");

       while(tool1.calibrateGyro(cfg));//校准角速度值校准好后返回一个0才可以执行下面的动作
       //tool1.calibrateAcc(cfg);//加速度计校准//实际发现这个值会影响后面的卡尔曼输出值pitchY
       accelgyro.getMotion6(&accX,&accY,&accZ,&gyroX,&gyroY,&gyroZ);//获取三个轴的加速度和角速度
       accYangle = (atan2((double)accY - cfg.accYzero, (double)accZ -cfg.accZzero) + PI) * RAD_TO_DEG;// We then convert it to 0 to 2π and then from radians to degrees

       kalmanY.setAngle(accYangle); // Set starting angle//用加速度计测出对于Y轴的角度，进行卡尔曼滤波
       pitchY = accYangle;
       gyroYangle = accYangle;
       LED::SetDirWrite(); // Set LED pin to output
       motor1.stopAndReset();//这个比较重要，因为 这里会给进行一些变量赋值，如下面所示
							   //integratedError = 0;
							  //targetPosition = encoderM.getWheelsPosition();
							  //lastRestAngle = cfg.targetAngle;

#ifdef ENABLE_TOOLS
       tool1.printMenu();//用于显示用户命令
#endif

       buzzer::Set();
       delay(100);
       buzzer::Clear();

       /* Setup timing */
       kalmanTimer = micros();
       pidTimer = kalmanTimer;
       imuTimer = millis();
       encoderTimer = imuTimer;
       reportTimer = imuTimer;
       ledTimer = imuTimer;
       blinkTimer = imuTimer;
       bluetooth1.readUsb();//主要针对蓝牙不稳定的问题，设置多重检测
}

void loop()
{
	//bluetooth1.readUsb();
	if (!M1EN::IsSet() || !M2EN::IsSet())
	{ // Motor driver will pull these low on error
		   buzzer::Set();
		motor1.stopMotor(left);
		motor1.stopMotor(right);
	   while (1);
    }

	/* Calculate pitch */
	accelgyro.getMotion6(&accX,&accY,&accZ,&gyroX,&gyroY,&gyroZ);//获取三个轴的加速度和角速度
	//atan 2 outputs the value od -π to π (radians),将值转换为弧度值
	//We then convert it to 0 to 2π and then from radians to degrees
	//note:if we only consider the angle ,we don't need transfrom the raw acceleration to its physical meaning
	//the formulation transforming to its physical meaning (uint :g)accX=accX/16384.00//转换成我们所认知的加速度

	accYangle = (atan2((double)accY - cfg.accYzero, (double)accZ - cfg.accZzero) + PI) * RAD_TO_DEG;
	//Serial.print("cfg.accYzero");//调试检测时用
   // Serial.println(cfg.accYzero);
    //Serial.print("\n");
	 timer=micros();

	// This fixes the 0-360 transition problem when the accelerometer angle jumps between 0 and 360 degrees
	  if ((accYangle < 90 && pitchY > 270) || (accYangle > 270 && pitchY < 90))
	  {
		kalmanY.setAngle(accYangle);
		pitchY = accYangle;
		gyroXangle = accYangle;
	  }
	  else
	  {
		gyroXrate = ((double)gyroX - cfg.gyroXzero) / 131.0; // Convert to deg/s//求得对于X轴的旋转角速度转换成
		double dt = (double)(timer - kalmanTimer) / 1000000.0;
		gyroXangle += gyroXrate * dt; // Gyro angle is only used for debugging
		if (gyroXangle < 0 || gyroXangle > 360)
		  gyroXangle = pitchY; // Reset the gyro angle when it has drifted too much
		pitchY = kalmanY.getAngle(accYangle, gyroXrate,dt); // Calculate the angle using a Kalman filter
		//Serial.print("pitchY");//调试检测时用
		//Serial.println(pitchY);
		//Serial.print(",");
		//Serial.print("pitchY");//调试检测时用
		//Serial.println(gyroXangle);
		//Serial.print(",");
		//Serial.println(accYangle);
		//Serial.print("\n");

	  }
	  kalmanTimer = timer;

	  /* Drive motors */
	  timer = micros();
	  // If the robot is laying down, it has to be put in a vertical position before it starts balancing
	  // If it's already balancing it has to be ±45 degrees before it stops trying to balance
	  if ((layingDown && (pitchY< cfg.targetAngle - 5 || pitchY> cfg.targetAngle + 5)) || (!layingDown && (pitchY< cfg.targetAngle - 45 || pitchY> cfg.targetAngle + 45)))
	  {//这里为了防止从倒的状态变成站立状态，出现跑飞现象，角度值由原来的—10~10变成-5~5；
	    layingDown = true; // The robot is in a unsolvable position, so turn off both motors and wait until it's vertical again
	    motor1.stopAndReset();
	   //motor1.moveMotor(left,forward,50);//测试程序用，这一步是正确的
	   //motor1.moveMotor(right,forward,50);
	   //Serial.print("targetAngle");//调试检测时用
	   // Serial.println(cfg.targetAngle);
	    //Serial.print("\n");
	  }
	  else
	  {
	    layingDown = false; // It's no longer laying down
	    //motor1.moveMotor(left,forward,50);//测试程序用，测试倾斜角程序
	   // motor1.moveMotor(right,forward,50);

	  // pid1.updatePID(cfg.targetAngle, targetOffset, turningOffset, (double)(timer - pidTimer) / 1000000.0,cfg);
            updatePID(cfg.targetAngle, targetOffset, turningOffset, (double)(timer - pidTimer) / 1000000.0);
	   // Set PWM Values
	      if (PIDLeft >= 0)
	    	  motor1.moveMotor(left, forward, PIDLeft);
	      else
	    	  motor1.moveMotor(left, backward, -PIDLeft);
	      if (PIDRight >= 0)
	    	  motor1.moveMotor(right, forward, PIDRight);
	      else
	    	  motor1.moveMotor(right, backward, -PIDRight);

	  }
	  pidTimer = timer;//这个写在外面是有原因的，用于积分时间控制

	  //Serial.print(error);
	 // Serial.print(",");
	  //Serial.print("iTerm");//调试检测时用
	 //Serial.print(PIDValue);
	 // Serial.print(",");
	//Serial.print("dTerm");//调试检测时用
	 //Serial.println(pitchY);
	 // Serial.print("\n");
	  //Serial.print("targetOffset");
	 // Serial.println(targetOffset);
	 //Serial.println("turningOffset");
	  //Serial.println(turningOffset);
	  timer = millis();//这个是ms计时器
	  encoders.updateEncoders();
	  //Serial.print("targetPosition");
	  //Serial.println(targetPosition);
#if defined(ENABLE_TOOLS) || defined(ENABLE_SPEKTRUM)
  tool1.checkSerialData();
#endif
#if defined(ENABLE_USB) || defined(ENABLE_SPEKTRUM)
  bluetooth1.readUsb();
#endif
#if defined(ENABLE_TOOLS) || defined(ENABLE_SPP)

  tool1.printValues();
  //Serial.print("great\n");
#endif

//#ifdef ENABLE_BTD
  if(Btd.isReady())
  {
	  timer=millis();
	  if((Btd.watingForConnection &&timer-blinkTimer>1000)||(!Btd.watingForConnection&&timer-blinkTimer>100))
	  {
	  blinkTimer=timer;
	  LED::Toggle();//used to blink in led ,starts blinking faster upon an incoming Bluetooth request
	  }
  }
  else
	  LED::Clear();//This will turn it off
//#endif
}







