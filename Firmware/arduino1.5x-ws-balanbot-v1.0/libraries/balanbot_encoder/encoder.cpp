/*
 * encoder.cpp
 *
 *  Created on: 2014年7月19日
 *      Author: caixiaoliang
 */
#include "encoder.h"


/* Interrupt routine and encoder read functions */
// It uses gray code to detect if any pulses are missed. See: https://www.circuitsathome.com/mcu/reading-rotary-encoder-on-arduino and http://en.wikipedia.org/wiki/Rotary_encoder#Incremental_rotary_encoder.
/*
#if defined(PIN_CHANGE_INTERRUPT_VECTOR_LEFT) && defined(PIN_CHANGE_INTERRUPT_VECTOR_RIGHT)
const int8_t enc_states[16] = { 0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0 }; // Encoder lookup table if it interrupts on every edge
*/
#if defined(PIN_CHANGE_INTERRUPT_VECTOR_LEFT) && defined(PIN_CHANGE_INTERRUPT_VECTOR_RIGHT)
const int8_t enc_states[16] = { 0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0 }; // Encoder lookup table if it interrupts on every edge

ISR(PIN_CHANGE_INTERRUPT_VECTOR_LEFT) {
  leftEncoder();
#if PIN_CHANGE_INTERRUPT_VECTOR_LEFT != PIN_CHANGE_INTERRUPT_VECTOR_RIGHT
}
ISR(PIN_CHANGE_INTERRUPT_VECTOR_RIGHT) {
#endif
  rightEncoder();
}
#else
const int8_t enc_states[16] = { 0, 0, 0, -1, 0, 0, 1, 0, 0, 1, 0, 0, -1, 0, 0, 0 }; // Encoder lookup table if it only interrupts on every second edge
#endif

//设置时间
void encoder::setupTiming(){
	encoderTimer=millis();
}

//设置编码器
void encoder::setupEncoders(){
	//setup encoder
	  leftEncoder1::SetDirRead();
	  leftEncoder2::SetDirRead();
	  rightEncoder1::SetDirRead();
	  rightEncoder2::SetDirRead();
	  leftEncoder1::Set(); // Enable pull-ups
	  leftEncoder2::Set();
	  rightEncoder1::Set();
	  rightEncoder2::Set();
}

//设置中断
void setInterrupt(){

	//(encoders.*p1)(); //调用对象encoders中p1所指的成员函数(即t1.get_time( ))
	  attachInterrupt(digitalPinToInterrupt(leftEncoder1Pin),leftEncoder, CHANGE);
	  attachInterrupt(digitalPinToInterrupt(rightEncoder1Pin),rightEncoder, CHANGE);

	#if defined(PIN_CHANGE_INTERRUPT_VECTOR_LEFT) && defined(PIN_CHANGE_INTERRUPT_VECTOR_RIGHT)
	  /* Enable encoder pins interrupt sources */
	  *digitalPinToPCMSK(leftEncoder2Pin) |= (1 << digitalPinToPCMSKbit(leftEncoder2Pin));
	  *digitalPinToPCMSK(rightEncoder2Pin) |= (1 << digitalPinToPCMSKbit(rightEncoder2Pin));

	  /* Enable pin change interrupts */
	  *digitalPinToPCICR(leftEncoder2Pin) |= (1 << digitalPinToPCICRbit(leftEncoder2Pin));
	  *digitalPinToPCICR(rightEncoder2Pin) |= (1 << digitalPinToPCICRbit(rightEncoder2Pin));
	#endif
}

//更新编码器
void encoder::updateEncoders()
{
	//uint32_t timer = millis();
	  if (timer - encoderTimer >= 100)
	  { // Update encoder values every 100ms
	    encoderTimer = timer;
	    int32_t wheelPosition = getWheelsPosition();//获取当前编码值
	    wheelVelocity = wheelPosition - lastWheelPosition;//求取车子的速度值
	    lastWheelPosition = wheelPosition;//把当前编码器的值赋值给上一次，便于下一次比较
	    //Serial.print(wheelPosition);Serial.print('\t');Serial.print(targetPosition);Serial.print('\t');Serial.println(wheelVelocity);

	    if (abs(wheelVelocity) <= 40 && !stopped)//这个是当速度值非常小时，认为已经小车已经达到平衡了，于是把目标值付给targetposition
	    { // Set new targetPosition if braking
	      targetPosition = wheelPosition;
	      stopped = true;
	    }
	    /*
	    if (abs(wheelVelocity) <= 5)//这个是当速度值非常小时，认为已经小车已经达到平衡了，于是把目标值付给targetposition
		{ // Set new targetPosition if braking
		  targetPosition = wheelPosition;
		  stopped = true;
		}
	    else
	    {
	    	 stopped = false;
	    }
	    Serial.print("stopped");
	    Serial.println(stopped);
	    */
	    batteryCounter++;
	    if (batteryCounter >= 10)
	    { // Measure battery every 1s
	      batteryCounter = 0;
	      batteryVoltage = (double)analogRead(VBAT) / 63.050847458; // VBAT is connected to analog input 5 which is not broken out. This is then connected to a 47k-12k voltage divider - 1023.0/(3.3/(12.0/(12.0+47.0))) = 63.050847458
	      if (batteryVoltage < 10.2 && batteryVoltage > 5) // Equal to 3.4V per cell - don't turn on if it's below 5V, this means that no battery is connected
	        buzzer::Set();
	      else
	        buzzer::Clear();
	    }
	     //Serial.println("Update encoders");
	  }
}





void leftEncoder() {
  //Serial.println("------------leftEncoder----------");
  static uint8_t old_AB = 0;
  old_AB <<= 2; // Remember previous state
  old_AB |= (leftEncoder1::IsSet() >> (leftEncoder1::Number - 1)) | (leftEncoder2::IsSet() >> leftEncoder2::Number);
  leftCounter += enc_states[ old_AB & 0x0F ];
}

void rightEncoder()
{
  //Serial.println("------------rightEncoder----------");
  static uint8_t old_AB = 0;
  old_AB <<= 2; // Remember previous state
  old_AB |= (rightEncoder1::IsSet() >> (rightEncoder1::Number - 1)) | (rightEncoder2::IsSet() >> rightEncoder2::Number);
  rightCounter += enc_states[ old_AB & 0x0F ];
}

int32_t encoder::readLeftEncoder()
{ // The encoders decrease when motors are traveling forward and increase when traveling backward
  return leftCounter;
}

int32_t encoder::readRightEncoder() {
  return rightCounter;
}

int32_t encoder::getWheelsPosition() {
  return leftCounter + rightCounter;
}

/*
void encoder::testMotorSpeed(double *leftSpeed, double *rightSpeed, double leftScaler, double rightScaler) {
  int32_t lastLeftPosition = readLeftEncoder(), lastRightPosition = readRightEncoder();

  Serial.println(F("Velocity (L), Velocity (R), Speed value (L), Speed value (R)"));
  while (Serial.read() == -1) {
//    encoders.moveMotor(left, forward, (*leftSpeed)*leftScaler);
//    encoders.moveMotor(right, forward, (*rightSpeed)*rightScaler);

    int32_t leftPosition = readLeftEncoder();
    int32_t leftVelocity = leftPosition - lastLeftPosition;
    lastLeftPosition = leftPosition;

    int32_t rightPosition = readRightEncoder();
    int32_t rightVelocity = rightPosition - lastRightPosition;
    lastRightPosition = rightPosition;

    Serial.print(leftVelocity);
    Serial.print(F(","));
    Serial.print(rightVelocity);
    Serial.print(F(","));
    Serial.print(*leftSpeed);
    Serial.print(F(","));
    Serial.println(*rightSpeed);

    if (abs(leftVelocity) < 200)
      (*leftSpeed) += 0.1;
    else if (abs(leftVelocity) > 203)
      (*leftSpeed) -= 0.1;

    if (abs(rightVelocity) < 200)
      (*rightSpeed) += 0.1;
    else if (abs(rightVelocity) > 203)
      (*rightSpeed) -= 0.1;

    delay(100);
  }
}
*/

