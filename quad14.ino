#include <PinChangeInt.h>
#include <Servo.h>
#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>
#include <Wire.h>
#include "declarations.h"

FreeSixIMU sixDOF = FreeSixIMU(); // Set the FreeSixIMU object

void setup() 
{
	Serial.begin(115200);
	initPins();
	LEDshow(0,0,0,0);
	delay(100);
	initMotors();
	LEDshow(1,1,1,1);
	delay(100);
	initSixDOF();
	LEDshow(0,0,0,0);
	delay(100);
	setTunings(&PIDpitch[Kp], &PIDpitch[Ki], &PIDpitch[Kd]);
	setTunings(&PIDroll[Kp], &PIDroll[Ki], &PIDroll[Kd]);
	LEDshow(1,1,1,1);
	delay(100);
	offset();
	initInterrupts();
	LEDshow(3,3,0,0);
}

void loop() 
{
	static uint16_t unThrottleIn, unPITCHIn, unYawIn, unRollIn, bUpdateFlags;
	getAngles();
	
	if(bUpdateFlagsShared)
		readInterrupts(&unThrottleIn, &unPITCHIn, &unYawIn, &unRollIn, &bUpdateFlags); 
	
	changeSP(unRollIn, PIDroll, 930, 1730,1320); //Manövrering av roll axel
	changeSP(unPITCHIn,PIDpitch,974,1750,1340); //Manövrering av pitch axel
	//mapController(unPITCHIn,unRollIn);
	
	calcPID(PIDpitch,&errSum[PITCH],&lastTime[PITCH]); //pitch
	calcPID(PIDroll,&errSum[ROLL],&lastTime[ROLL]); //roll
	//calcPID(PIDyaw,&errSum[YAW],&lastTime[YAW]); //yaw
	
	//printOut3(unThrottleIn,unPITCHIn,unRollIn,PIDpitch[PID_OUTPUT],PIDpitch[SP],PIDroll[SP]);
   
	calcVelocity(unThrottleIn);
	ledAngle(PIDpitch[INPUT],PIDroll[INPUT]);
}

void readInterrupts(uint16_t * unThrottleIn, uint16_t * unPITCHIn, uint16_t * unYawIn, uint16_t * unRollIn, uint16_t * bUpdateFlags) // we have local copies of the PID_INPUTs, so now we can turn interrupts back on
{
	noInterrupts();
	*bUpdateFlags = bUpdateFlagsShared;
    
	if(*bUpdateFlags & THROTTLE_FLAG) *unThrottleIn = unThrottleInShared;
	if(*bUpdateFlags & PITCH_FLAG) *unPITCHIn = unPITCHInShared;
	if(*bUpdateFlags & Yaw_FLAG) *unYawIn = unYawInShared;
	if(*bUpdateFlags & ROLL_FLAG) *unRollIn = unRollInShared;
    bUpdateFlagsShared = 0;
	interrupts();
}

void getAngles()
{
	float ypr[3]; //Yaw Pitch Roll
	sixDOF.getYawPitchRoll(ypr);

	PIDyaw[PID_INPUT]   = (ypr[0]-yawOffset);
	PIDpitch[PID_INPUT] = (ypr[1]-pitchOffset); 
	PIDroll[PID_INPUT]  = (ypr[2]-rollOffset);
	
	//printOut(ypr[0], ypr[1],ypr[2]);
	//printOut(PIDyaw[PID_INPUT] , PIDpitch[PID_INPUT] , PIDroll[PID_INPUT]);
}

void changeSP(uint16_t signalIn, float * PIDaxis, int min, int max, int neutral)
{
	if(signalIn >= max-10)
		PIDaxis[SP] -= 0.3;
	else if(signalIn <= min+10)
		PIDaxis[SP]+=0.3;
	else if(signalIn >= neutral-10 && signalIn <= neutral+10 && PIDaxis[SP] != 0)
		PIDaxis[SP] += (PIDaxis[SP] > 0 ? -0.3 : 0.3);
}

void mapController(int _pitch, int _roll)
{
   PIDpitch[SP] = map(_pitch,964,1792,45,-45); //Vinkling Intervall: -45 till 45 grader
   PIDroll[SP] = map(_roll,932,1724,45,-45); //Vinkling Intervall: -45 till 45 grader
      if(PIDpitch[SP] >= -10 && PIDpitch[SP] <= 10)
	   PIDpitch[SP] = 0;
}

void calcPID(float * PIDaxis, double * errSum, unsigned long * lastTime)
{
	unsigned long now = millis();
	double timeChange = (double)(now - *lastTime);
   
	if(timeChange >= SampleTime) // dt = sampletime 20ms
	{
		double error = PIDaxis[SP] - PIDaxis[PID_INPUT]; //the error
		*errSum += (PIDaxis[Ki] * error); //the integral
		double dPID_INPUT = (PIDaxis[PID_INPUT] - PIDaxis[LAST_INPUT]); //the dy/dx
     
		PIDaxis[PID_OUTPUT] = (PIDaxis[Kp]) * error + (*errSum) + (PIDaxis[Kd]) * dPID_INPUT;
		PIDaxis[LAST_INPUT] = PIDaxis[PID_INPUT]; 
		*lastTime = now;
		//printOut2(timeChange,error,errSum,dPID_INPUT,PIDaxis[PID_OUTPUT], PIDaxis[SP]);
	}
}

void calcVelocity(int throttle)
{
	int frontThr, backThr, rightThr, leftThr;
	if(throttle > 1000)
	{
		frontThr = throttle + PIDpitch[PID_OUTPUT] - PIDyaw[PID_OUTPUT]; //- controllerOutput(_pitch,_yaw);
		backThr = throttle - PIDpitch[PID_OUTPUT]  - PIDyaw[PID_OUTPUT]; //+ controllerOutput(_pitch,_yaw);
		rightThr = throttle + PIDroll[PID_OUTPUT] + PIDyaw[PID_OUTPUT];
		leftThr = throttle - PIDroll[PID_OUTPUT] + PIDyaw[PID_OUTPUT];
	}

	sendToESC(frontThr, backThr, rightThr ,leftThr); //output to motors
	//printOut(throttle,PIDyaw[INPUT],PIDpitch[INPUT],PIDroll[INPUT],frontThr,backThr,rightThr,leftThr,PIDyaw[PID_OUTPUT],PIDpitch[PID_OUTPUT],PIDroll[PID_OUTPUT]);
}

int controllerOutput(int pitch, int yaw)
{
	return pitch - yaw;
}

void sendToESC(int front, int back, int right, int left) //int right, int left
{
	mFront.writeMicroseconds( (front >= MAX_THROTTLE ? MAX_THROTTLE : front)  ); //to make sure that u wont exceed the limited throttle
	mBack.writeMicroseconds( (back >= MAX_THROTTLE ? MAX_THROTTLE : back) );
	mRight.writeMicroseconds( (right >= MAX_THROTTLE ? MAX_THROTTLE : right)  );
	mLeft.writeMicroseconds( (left >= MAX_THROTTLE ? MAX_THROTTLE : left)  );
	//printOut(0,0,0,0,front,back,right,left,0,0,0);
} 

void ledAngle(float pitchAngle, float rollAngle)
{
	if(rollAngle > 10)
		LEDshow(3,3,1,0);
	else if(rollAngle < -10)
		LEDshow(3,3,0,1);
	
	if(pitchAngle > 10)
		LEDshow(0,1,3,3);
	else if(pitchAngle < -10)
		LEDshow(1,0,3,3);
	else if(pitchAngle >= -10 && pitchAngle <= 10 && rollAngle >= -10 && rollAngle <= 10)
		LEDshow(1,1,1,1);
}

void LEDshow(int fram, int bak, int right, int left)
{
	if(fram == 1) digitalWrite(ledFram, HIGH); //LED fram
	else if(fram == 0) digitalWrite(ledFram, LOW);
	
	if(bak == 1) digitalWrite(ledBak, HIGH);
	else if(bak == 0) digitalWrite(ledBak, LOW);
	
	if(right == 1) digitalWrite(ledRight, HIGH);
	else if(right == 0) digitalWrite(ledRight, LOW);
	
	if(left == 1) digitalWrite(ledLeft, HIGH);
	else if(left == 0) digitalWrite(ledLeft, LOW);
}


//-------------------- Terminal printout debug functions ------------------------------------
void printOut3(int throttle,int pitch,int roll,float pitchOutput, float pitchSP, float rollSP)
{
	Serial.print("	 Throttle:");
	Serial.print(throttle);
	Serial.print("	 PWM Pitch:");
	Serial.print(pitch);
	Serial.print("	 PWM Roll:");
	Serial.print(roll);
	Serial.print("	 PIDpitch Output:");
	Serial.print(pitchOutput);
	Serial.print("	 PIDpitch SP:");
	Serial.print(pitchSP);
	Serial.print("	 PIDroll SP:");
	Serial.println(rollSP);
}

void printOut(int throttle,float yaw, float pitch, float roll, int m1, int m2, int m3, int m4,float yawOutput, float pitchOutput, float rollOutput)
{
	Serial.print("Throttle:");
	Serial.print(throttle);
	Serial.print("	 Motor1:");
	Serial.print(m1);
	Serial.print("	 Motor2:");
	Serial.print(m2);
	Serial.print("   Motor3:");
	Serial.print(m3);
	Serial.print("   Motor4:");
	Serial.print(m4);
	Serial.print("	 Yaw:");
	Serial.print(yaw);
	Serial.print("	 Pitch:");
	Serial.print(pitch);
	Serial.print("   Roll:");
	Serial.print(roll);
	Serial.print("	 YawOutput:");
	Serial.print(yawOutput);
	Serial.print("	 PitchOutput:");
	Serial.print(pitchOutput);
	Serial.print("   RollOutput:");
	Serial.println(rollOutput);
}

void printOut2(double timeChange, double error, double *_errSum, double dPID_INPUT, float PIDOutput, float PIDsp)
{
	Serial.print("	 timeChange:");
	Serial.print(timeChange);
	Serial.print("	 error:");
	Serial.print(error);
	Serial.print("	 errSum:");
	Serial.print(*_errSum);
	Serial.print("   dPID_INPUT:");
	Serial.print(dPID_INPUT);
	Serial.print("	 PIDoutput:");
	Serial.print(PIDOutput);
	Serial.print("	 PIDsp:");
	Serial.println(PIDsp);
}


//--------------------- Initialize Functions ----------------------------------
void initPins()
{
	pinMode(ledFram, OUTPUT);
	pinMode(ledBak, OUTPUT);
	pinMode(ledRight, OUTPUT);
	pinMode(ledLeft, OUTPUT);
	pinMode(escPin1, OUTPUT);
	pinMode(escPin2, OUTPUT);
	pinMode(escPin3, OUTPUT);
	pinMode(escPin4, OUTPUT);
}

void initMotors()
{
	mFront.attach(escPin1);
	mBack.attach(escPin2);
	mRight.attach(escPin3);
	mLeft.attach(escPin4);
}

void setTunings(float* _Kp, float* _Ki, float* _Kd)
{
   float SampleTimeInSec = (float)SampleTime/1000;
   *_Ki *= SampleTimeInSec;
   *_Kd /= SampleTimeInSec;
}

void initInterrupts()
{
	PCintPort::attachInterrupt(THROTTLE_IN_PIN, calcThrottle, CHANGE);
	PCintPort::attachInterrupt(PITCH_IN_PIN, calcPITCH, CHANGE);
	//PCintPort::attachInterrupt(YAW_IN_PIN, calcYaw, CHANGE);
	PCintPort::attachInterrupt(ROLL_IN_PIN, calcRoll, CHANGE);
}
void initSixDOF()
{
	delay(5);
	sixDOF.init(); //begin the IMU
	delay(5);
}

void offset()
{
	float yprSum[3]; //Yaw Pitch Roll
	float ypr[3]; //Yaw Pitch Roll
	for(int i = 0; i < 4000; i++){ //Getting a accurate reading of the angles after 2000 samples at start
		sixDOF.getYawPitchRoll(ypr);
		if(i >= 2000)
		{
			yawOffset += ypr[0];
			pitchOffset += ypr[1];
			rollOffset += ypr[2];
		}
		//printOut(yawOffset, pitchOffset, rollOffset);
	}
	yawOffset /= 2000; //The average value of the offset with 2000 samples
	pitchOffset /= 2000;
	rollOffset /= 2000;
}

//------------------------ ISR ROUTINES ---------------------------------
void calcThrottle() //Simple interrupt service routine
{
  if(digitalRead(THROTTLE_IN_PIN) == HIGH)
    ulThrottleStart = micros();
  else
  {
    unThrottleInShared = (uint16_t)(micros() - ulThrottleStart);
    bUpdateFlagsShared |= THROTTLE_FLAG; //Set the throttle flag to indicate that a new throttle signal has been received
  }
}
void calcPITCH()
{
  if(digitalRead(PITCH_IN_PIN) == HIGH)
    ulPITCHStart = micros();
  else
  {
    unPITCHInShared = (uint16_t)(micros() - ulPITCHStart);
    bUpdateFlagsShared |= PITCH_FLAG;
  }
}
void calcYaw()
{
  if(digitalRead(YAW_IN_PIN) == HIGH)
    ulYawStart = micros();
  else
  {
    unYawInShared = (uint16_t)(micros() - ulYawStart);
    bUpdateFlagsShared |= Yaw_FLAG;
  }
}
void calcRoll()
{
  if(digitalRead(ROLL_IN_PIN) == HIGH)
    ulRollStart = micros();
  else
  {
    unRollInShared = (uint16_t)(micros() - ulRollStart);
    bUpdateFlagsShared |= ROLL_FLAG;
  }
}
