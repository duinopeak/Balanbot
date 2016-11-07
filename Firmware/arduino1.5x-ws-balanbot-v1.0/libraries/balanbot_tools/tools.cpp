// Do not remove the include below
#include "tools.h"  //注意：这个tools.h里面包含了balanbot.h文件，其中有结构体cfg_t
#include "MPU6050.h"//调用Mpu6050内部读取数值函数
#include "Arduino.h"
#include "encoder.h"//调用readLeftEncoder();等函数
#include "motor.h"//调用moveMotor()的测试
#include "eepromInit.h"//用于电机调试后，用EEPROM储存新数据
#include "Kalman.h"//在printvalue()函数里面调用kalman
#include "Bluetooth.h"//用于调用steer()函数
#include "SPP.h"//用于蓝牙调试
#include "Usb.h"//用于Usb连接


//extern cfg_t cfg;
/*
void Tool::checkSerialData()//用于实时检测用户输入数据
{
    if(Serial.available())
    {
        int input=Serial.read();
#ifdef ENABLE_TOOLS
        if(input=='m')

        	printMenu();
#endif
        else
        {

#ifdef ENABLE_TOOLS
        	dataInput[0]=static_cast<char>(input);//intentional cast
        	delay(2);
        	uint8_t i=1;
#endif
        	while(1)
        	{
        	    input=Serial.read();//读取输入的字符
        	    if(input==-1)
        	    	continue;
#ifdef ENABLE_TOOLS
        	    dataInput[i]=static_cast<char>(input);//Intentional cast
        	    if(dataInput[i]==';')//keep reading untill it reads a semicolon
        	    	break;
        	    if(++i>=sizeof(dataInput)/sizeof(dataInput[0]))//string is too long
        	    	return;
#endif
        	}
#ifdef ENABLE_TOOLS
        	bluetoothData=false;
        	setValues(dataInput);
            }
#endif
    }

}
*/
void Tool::checkSerialData()//用于实时检测用户输入数据
{
  if (Serial.available())
  {
    int input = Serial.read();
#ifdef ENABLE_TOOLS
    if (input == 'm')
      printMenu();
    else
    {
#endif
#ifdef ENABLE_SPEKTRUM
      readSpektrum(static_cast<uint8_t> (input)); // Intentional cast
#endif
#ifdef ENABLE_TOOLS
      dataInput[0] = static_cast<char> (input); // Intentional cast
      delay(2); // Wait for rest of data
      uint8_t i = 1;
#endif
      while (1)
      {
        input = Serial.read();
        if (input == -1) // Error while reading the string
          return;
#ifdef ENABLE_SPEKTRUM
        readSpektrum(static_cast<uint8_t> (input)); // Intentional cast
#endif
#ifdef ENABLE_TOOLS
        dataInput[i] = static_cast<char> (input); // Intentional cast
        if (dataInput[i] == ';') // Keep reading until it reads a semicolon
          break;
        if (++i >= sizeof(dataInput) / sizeof(dataInput[0])) // String is too long
          return;
#endif
      }
#ifdef ENABLE_TOOLS
      bluetoothData = false;
      setValues(dataInput);
    }
#endif
  }
}

void Tool::printMenu()
{
	Serial.println(F("\r\n========================================== Menu ==========================================\r\n"));

	  Serial.println(F("m\t\t\t\tSend to show this menu\r\n"));

	  Serial.println(F("A;\t\t\t\tSend to abort. Send 'C' again to continue\r\n"));

	  Serial.println(F("AC;\t\t\t\tSend to calibrate the accelerometer"));
	  Serial.println(F("MC;\t\t\t\tSend to calibrate the motors\r\n"));

	  Serial.println(F("GP;\t\t\t\tGet PID values"));
	  Serial.println(F("GK;\t\t\t\tGet Kalman filter values"));
	  Serial.println(F("GS;\t\t\t\tGet settings values"));
	  Serial.println(F("GI;\t\t\t\tGet info values\r\n"));

	  Serial.println(F("SP,Kp;\t\t\t\tUsed to set the Kp value"));
	  Serial.println(F("SI,Ki;\t\t\t\tUsed to set the Ki value"));
	  Serial.println(F("SD,Kd;\t\t\t\tUsed to set the Kd value"));
	  Serial.println(F("ST,targetAngle;\t\t\tUsed to set the target angle"));
	  Serial.println(F("SK,Qangle,Qbias,Rmeasure;\tUsed to set the Kalman filter values"));
	  Serial.println(F("SA,angle;\t\t\tUsed to set the maximum controlling angle"));
	  Serial.println(F("SU,value;\t\t\tUsed to set the maximum turning value"));
	  Serial.println(F("SB,value;\t\t\tUsed to set the back to spot value (true = 1, false = 0)\r\n"));

	  Serial.println(F("IB;\t\t\t\tStart sending IMU values"));
	  Serial.println(F("IS;\t\t\t\tStop sending IMU values"));
	  Serial.println(F("RB;\t\t\t\tStart sending report values"));
	  Serial.println(F("RS;\t\t\t\tStop sending report values\r\n"));

	  Serial.println(F("CS;\t\t\t\tSend stop command"));
	  Serial.println(F("CJ,x,y;\t\t\t\tSteer robot using x,y-coordinates"));
	  Serial.println(F("CM,pitch,roll;\t\t\tSteer robot using pitch and roll"));
	#ifdef ENABLE_WII
	  Serial.println(F("CPW;\t\t\t\tStart paring sequence with Wiimote"));
	#endif
	#ifdef ENABLE_PS4
	  Serial.println(F("CPP;\t\t\t\tStart paring sequence with PS4 controller"));
	#endif
	  Serial.println(F("CR;\t\t\t\tRestore default EEPROM values\r\n"));

	#ifdef ENABLE_SPEKTRUM
	  Serial.println(F("BS;\t\t\t\tBind with Spektrum satellite receiver"));
	#endif
	  Serial.println(F("\r\n==========================================================================================\r\n"));
}

void Tool::calibrateAcc(cfg_t &m) //加速度计校准
{
	MPU6050 MPU60501;//因为接下来要在这个成员函数中调用另外一个类，所以在这里 实例化一个类对象
	eeprombot eeprombotTA;//因为接下来要在这个成员函数中调用eeprombot类，所以实例化一个类对象
	Serial.print("Please put the robot perfectly hotizontal on its side and then send any character to start the calibration routine\n");
	while (Serial.read() == -1);//平时校准加速度的时候打开上面这两个语句

	int16_t accYbuffer[25],accZbuffer[25];//定义两个加速度数组，用于后面的数据存储
	for(uint8_t i=0;i<25;i++)
	{
		accYbuffer[i]=MPU60501.getAccelerationY();//调用实例化的类对象成员函数
		accZbuffer[i]=MPU60501.getAccelerationZ();//利用MPU6050函数读取加速度值
	    delay(10);
	}
	if (!checkMinMax(accYbuffer,25,1000)||!checkMinMax(accZbuffer,25,1000))
	{
		Serial.print("Acclerometer calibration error\n");
		buzzer::Set();
		while(1);//
	}
	for(uint8_t i=0;i<25;i++)
	{
		m.accYzero +=accYbuffer[i];
		m.accZzero +=accZbuffer[i];
	}
	m.accYzero /=25;
	m.accZzero /=25;
	m.accYzero /=16384.0;
    m.accZzero /=16384.0;
    /*
	if(m.accYzero<0)  //check which side is laying down
	{
	   m.accYzero +=16384.0;//16384.0 is equal to 1g while the full scale range is -2g-+2g
	}
	else
		m.accYzero -=16384.0;

	if(m.accZzero<0)  //check which side is laying down
	{
	   m.accZzero +=16384.0;//16384.0 is equal to 1g while the full scale range is -2g-+2g
	}
	else
		m.accZzero -=16384.0;
   */
	//Serial.print("New zero value (g's):");//print the new value in g's
    Serial.print("cfg.accYzero  ");
	Serial.print(m.accYzero);
	Serial.print("  ");
	Serial.print("cfg.accZzero  ");
	Serial.println(m.accZzero);

	//updateConfig();//store the new value in the EEPROM如果加上这部函数，需要在这个类中实例I2C类的一个对象
	eeprombotTA.updateConfig(configAddr,cfg);//把校准后的值放入EEPROM中保存
   Serial.print("calibration of the accelerometer is done\n");
}

bool Tool::checkMinMax(int16_t *array,uint8_t length,int16_t maxDifference)
{
    int16_t min=array[0],max=array[0];

    for(uint8_t i=1;i<length;i++)
    {
    	if(array[i]<min)
    		min=array[i];
    	else if(array[i]>max)
    		max=array[i];
    }
    return max-min<maxDifference;
}

bool Tool::calibrateGyro(cfg_t &m)
{
	MPU6050 MPU60502;//因为接下来要在这个成员函数中调用另外一个类，所以在这里 实例化一个类对象
	int16_t gyroXbuffer[25];

	for(uint8_t i=0;i<25;i++)
	{
		 gyroXbuffer[i]=MPU60502.getRotationX();
		 delay(10);
	}
	if(!checkMinMax(gyroXbuffer,25,2000))
	{
		Serial.print("Gyro calibration error\n");
		//buzzer::Set();
		return 1;
	}
	else
	{
		for(uint8_t i=0;i<25;i++)
		m.gyroXzero +=gyroXbuffer[i];
		m.gyroXzero /=25.0; //将校准得到的x轴的旋转角度平均化，后期将用到这个偏移值
		//Serial.print("cfg.gyroXzero\t");
		//Serial.println(cfg.gyroXzero);
		Serial.print("calibration of the gyrometer is done\n");//2014、7、28,下午测试主程序到了这里
		return 0;

	}
}


void Tool::calibrateMotor()//校准电机函数，调用下面的testMotorSpeed函数
{
	 eeprombot eeprombotT;//新建一个eeprombot,对象，用于调用updateConfig();函数
     Serial.print("Put the robot so the wheels can move freely ande then send any character to start the motor calibration routine");
     while(Serial.read()==-1);//没有字符输入的时候在这里等待，输入任意字符，马上进入下一步

     Serial.println("Estimating minimum starting value,when the first two values do not change,then send any character to continue\r\n");
     delay(2000);
     double leftSpeed=10,rightSpeed=10;
     testMotorSpeed(&leftSpeed,&rightSpeed,1,1);

     Serial.print("\r\nThe speed values are (L/R):");
     Serial.println(leftSpeed);
     Serial.print(",");
     Serial.println(rightSpeed);

     if(leftSpeed>rightSpeed)//this means that the left motor needed a higher PWM signal before it rotated at the same speed
     {
    	 cfg.leftMotorScaler=1;
    	 cfg.rightMotorScaler=rightSpeed/leftSpeed;//therefore we will scale the right motor a bit dowmn ,so they match
     }
     else  //and the same goes for the right motor
     {
    	cfg.leftMotorScaler=leftSpeed/rightSpeed;
    	cfg.rightMotorScaler=1;
     }

     Serial.print("The motor scalers are now (L/R)");
     Serial.println(cfg.leftMotorScaler);
     Serial.print(",");
     Serial.println(cfg.rightMotorScaler);

     Serial.println("Now the motor will spin up again .now the speed values should be almost equal,Send any character to exit\r\n");
     delay(2000);
     leftSpeed=rightSpeed=10;//reset speed values
     testMotorSpeed(&leftSpeed,&rightSpeed,cfg.leftMotorScaler,cfg.rightMotorScaler);

     double maxSpeed=max(leftSpeed,rightSpeed);
     double minSpeed=min(leftSpeed,rightSpeed);

     Serial.print("The difference is now:");
     Serial.print("%");
     Serial.println((maxSpeed-minSpeed)/maxSpeed*100);


     eeprombotT.updateConfig(configAddr,cfg);//Store the new values in the EEPROM
     Serial.println("Calibration of the motors is done");

}

void Tool::testMotorSpeed(double *leftSpeed,double *rightSpeed,double leftScaler,double rightScaler)
{
	encoder encoderT;//新建一个encoder对象，用于下面调用两个计数函数
	Motor MotorT;//新建一个Motor对象，用于下面的moveMortor函数
	int32_t lastLeftPosition=encoderT.readLeftEncoder();//记住上次的编码数
	int32_t lastRightPosition=encoderT.readRightEncoder();

	Serial.print("Velocity (L),Velocity(R),Speed value (L),Speed value(R)");
	while(Serial.read()==-1)
	{
	   MotorT.moveMotor(left,forward,(*leftSpeed)*leftScaler);
	   MotorT.moveMotor(right,forward,(*rightSpeed)*rightScaler);

	   int32_t leftPosition=encoderT.readLeftEncoder();//记住现在的编码数
	   int32_t leftVelocity=leftPosition-lastLeftPosition;
	   lastLeftPosition=leftPosition;

	   int32_t rightPosition=encoderT.readRightEncoder();
	   int32_t rightVelocity=rightPosition-lastRightPosition;
	   lastRightPosition=rightPosition;

	   Serial.println(leftVelocity);
	   Serial.print(",");
	   Serial.println(rightVelocity);
	   Serial.print(",");
	   Serial.println(*leftSpeed);
	   Serial.print(",");
	   Serial.println(*rightSpeed);

	   if(abs(leftVelocity)<200)
	   {
		   (*leftSpeed)+=0.1;
	   }
	   else if(abs(leftVelocity)>203)
	   {
		   (*leftSpeed)-=0.1;
	   }

	   if (abs(rightVelocity) < 200)
	   {
		   (*rightSpeed) += 0.1;
	   }
	   else if (abs(rightVelocity) > 203)
	   {
		   (*rightSpeed) -= 0.1;
	   }

	   delay(100);
	}

	for(double i=*leftSpeed;i>0;i--)  //stop motors gently
	{
		MotorT.moveMotor(left,forward,i);
	    MotorT.moveMotor(right,forward,i);
	    delay(50);
	}

	MotorT.stopMotor(left);
	MotorT.stopMotor(right);

}
void Tool::setValues(char *input)
{
	Motor MotorT2;//新建一个Motor类对象，用于下面的stopAndReset();
	eeprombot eeprombotT2;//新建一个eeptombot对象，用于下面的updateconfig()函数
	BluetoothN bluetoothT;//新建一个BluetoothN类对象，用于下面的steer()函数
	if(input[0]=='A'&&input[1]==';')
	{
		MotorT2.stopAndReset();

#ifdef ENABLE_SPP
   while (Serial.read() != 'C' && SerialBT.read() != 'C') // Wait until continue is sent
      Usb.Task();
#else
   while (Serial.read() != 'C');
#endif
	}
	else if(input[0]=='A'&&input[1]=='C')//Accelerometer calibration
	{
		calibrateAcc(cfg);
	}
	else if(input[0]=='M'&&input[1]=='C')//Motor calibration
		calibrateMotor();
	else if(input[0]=='G')
	{
		if(input[1]=='P')//Get PID values
		{
			sendPIDValues=true;
		}
		else if(input[1]=='S')//Get settings
		{
			sendSettings=true;
		}
		else if(input[1]=='I')//ger info
		{
			sendInfo=true;
		}
		else if(input[1]=='K')//get Kalman filter values
		{
			sendKalmanValues=true;
		}
	}

	else if(input[0]=='S')//set different values
	{ //set PID and target angle
		if(input[1]=='P')
		{
			strtok(input,",");//Ignore 'p'
			cfg.P=atof(strtok(NULL,";"));
		}
		else if(input[1]=='I')
		{
			strtok(input, ","); // Ignore 'I'
			cfg.I = atof(strtok(NULL, ";"));
		}
		else if (input[1] == 'D')
		{
			strtok(input, ","); // Ignore 'D'
			cfg.D = atof(strtok(NULL, ";"));
		}
		else if (input[1] == 'T')// Target Angle
		{
		      strtok(input, ","); // Ignore 'T'
		      cfg.targetAngle = atof(strtok(NULL, ";"));
		}
		else if (input[1] == 'K') // Kalman values
		{
		      strtok(input, ","); // Ignore 'K'
		      cfg.Qangle = atof(strtok(NULL, ","));
		      cfg.Qbias = atof(strtok(NULL, ","));
		      cfg.Rmeasure = atof(strtok(NULL, ";"));
		 }
		else if (input[1] == 'A')  // Controlling max angle
		{
		      strtok(input, ","); // Ignore 'A'
		      cfg.controlAngleLimit = atoi(strtok(NULL, ";"));
		}
		else if (input[1] == 'U') // Turning max value
		{
		      strtok(input, ","); // Ignore 'U'
		      cfg.turningLimit = atoi(strtok(NULL, ";"));
		 }
		else if (input[1] == 'B')// Set Back To Spot
		{
		      if (input[3] == '1')
		        cfg.backToSpot = 1;
		      else
		        cfg.backToSpot = 0;
		 }
		eeprombotT2.updateConfig(configAddr,cfg);
	}

	else if(input[0]=='I')//IMU transmitting states
	{
		if(input[1]=='B')//begin sending IMU values
			sendIMUValues=true;//start sending output to application
		else if(input[1]=='S')//stop sending IMU values
		{
			sendIMUValues=false;//stop sending output to application
		}
	}

	else if(input[0]=='R')//Report states
	{
		if(input[1]=='B')//begin sending report values
			sendStatusReport=true;//start sending output to application
		else if(input[1]=='S')
			sendStatusReport=false;//stop sending output to application
	}

	else if(input[0]=='C')//commands
	{
		if (input[1] == 'S') // Stop

			bluetoothT.steer(stop);	//steer(stop);//这个函数在蓝牙模块中

		else if (input[1] == 'J')
		{ // Joystick
		  receiveControlTimer = millis();
		  strtok(input, ","); // Ignore 'J'
		  sppData1 = atof(strtok(NULL, ",")); // x-axis
		  sppData2 = atof(strtok(NULL, ";")); // y-axis
		  bluetoothT.steer(joystick);
		}
		else if (input[1] == 'M')
		 { // IMU
             receiveControlTimer = millis();
             strtok(input, ","); // Ignore 'M'
             sppData1 = atof(strtok(NULL, ",")); // Pitch
             sppData2 = atof(strtok(NULL, ";")); // Roll
             bluetoothT.steer(imu);
          }
#if defined(ENABLE_WII) || defined(ENABLE_PS4)
    else if (input[1] == 'P')
    { // Pair
#ifdef ENABLE_WII
      if (input[2] == 'W')
      { // Pair with a new Wiimote or Wii U Pro Controller
        Wii.pair();
        sendPairConfirmation = true;
      }
#endif
#ifdef ENABLE_PS4
      if (input[2] == 'P')
      { // Pair with PS4 controller
        PS4.pair();
        sendPairConfirmation = true;
      }
#endif
    }
#endif // defined(ENABLE_WII) || defined(ENABLE_PS4)
    else if (input[1] == 'R')
    {
    eeprombotT2.restoreEEPROMValues(configAddr, cfg); // Restore the default EEPROM values
      sendPIDValues = true;
      sendKalmanValues = true;
      sendSettings = true;
    }
	}
}

//#if defined(ENABLE_TOOLS)||defined(ENABLE_SPP)
void Tool::printValues()
{
#ifdef ENABLE_SPP
	Kalman kalmanT;//新建一个Kalman对象，用于下面的调试
	Print *out;//this allows the robot to use either the hardware UART or the BluetoothSPP connection dynamically
	           //动态地实现硬件与蓝牙连接
	if(SerialBT.connected&&bluetoothData)
	{
		//Serial.println("bluetoothdata trans!");//测试蓝牙这里通过
		out=dynamic_cast<Print *>(&SerialBT);//print using the Bluetooth  SPP interface
	}
	else
		out=dynamic_cast<Print *>(&Serial);//print using the standard UART port
#else
	HardwareSerial *out=&Serial;//print using the standard UART port
#endif

	if(sendPairConfirmation)
	{
		sendPairConfirmation=false;
		 out->println(F("PC"));
	}
	else if(sendPIDValues)
	{
		sendPIDValues=false;

		out->print(F("P,"));
		out->print(cfg.P);
		out->print(F(","));
		out->print(cfg.I);
		out->print(F(","));
		out->print(cfg.D);
		out->print(F(","));
		out->println(cfg.targetAngle);
	}
	else if(sendSettings)
	{
		sendSettings=false;
		out->print(F("S,"));
		out->print(cfg.backToSpot);
		out->print(F(","));
		out->print(cfg.controlAngleLimit);
		out->print(F(","));
		out->println(cfg.turningLimit);
	}
	else if(sendInfo)
	{
	    sendInfo=false;
	    out->print(F("I,"));
		out->print(version);
		out->print(F(","));
		out->print(eepromVersion);
	}
	else if (sendKalmanValues)
	{
	    sendKalmanValues = false;

	    out->print(F("K,"));
	    out->print(kalmanT.getQangle(), 4);
	    out->print(F(","));
	    out->print(kalmanT.getQbias(), 4);
	    out->print(F(","));
	    out->println(kalmanT.getRmeasure(), 4);
	}
	else if (sendIMUValues && millis() - imuTimer > 50)
	{ // Only send data every 50ms
	    imuTimer = millis();

	    out->print(F("V,"));
	    out->print(accYangle);//由小车发送回手机的数据，用于波形显示，加速度显示
	    out->print(F(","));
	    out->print(gyroXangle);//旋转加速度显示
	    out->print(F(","));
	    out->println(pitchY); //卡尔曼波形
	}
	else if (sendStatusReport && millis() - reportTimer > 500)
	{ // Send data every 500ms
	    reportTimer = millis();

	    out->print(F("R,"));
	    out->print(batteryVoltage);
	    out->print(F(","));
	    out->println((double)reportTimer / 60000.0);
	  }

}



