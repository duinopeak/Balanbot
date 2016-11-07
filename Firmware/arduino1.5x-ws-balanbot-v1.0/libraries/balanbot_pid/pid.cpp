// Do not remove the include below
#include "pid.h"
#include "encoder.h"

//x:预定角度 相当于setpoint  y:预定偏差  z：转向偏差  m:(timer-pidtimer)/1000000
// updatePID(cfg.targetAngle, targetOffset, turningOffset, (double)(timer - pidTimer) / 1000000.0);  //bluetooth  6050  i2c  motor  velocity  encoder
void pid::updatePID(double x,double y,double z, double dt,cfg_t &n)
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

     	 pTerm =1.0*n.P*error;//误差*Kp		  Kp Term
     	 iTerm = (n.I*40.0)*constrain(integratedError,-1.0,1.0); // Limit the integrated error
     	 dTerm = (n.D/40.0)*(error-lastError) / dt;
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

