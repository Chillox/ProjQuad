//Declarations.h är gruppens egna bibliotek , resterande är ej egna
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
	delay(1000);
	initMotors();
	LEDshow(1,1,1,1);
	delay(1000);
	initSixDOF();
	LEDshow(0,0,0,0);
	delay(1000);
	setTunings(&PIDpitch[Kp], &PIDpitch[Ki], &PIDpitch[Kd]);
	setTunings(&PIDroll[Kp], &PIDroll[Ki], &PIDroll[Kd]);
	LEDshow(1,1,1,1);
	delay(1000);
	offset();
	initInterrupts();
	LEDshow(3,3,0,0);
}

void loop() 
{
	getAngles();
	//mapController(unPITCHIn , unYawIn);
	
	calcPID(PIDpitch,&errSum[PITCH],&lastTime[PITCH]); //pitch
	calcPID(PIDroll,&errSum[ROLL],&lastTime[ROLL]); //roll
	calcPID(PIDyaw,&errSum[YAW],&lastTime[YAW]); //yaw
	
	static uint16_t unThrottleIn, unPITCHIn, unYawIn, unRollIn, bUpdateFlags;
	if(bUpdateFlagsShared)
		readInterrupts(&unThrottleIn, &unPITCHIn, &unYawIn, &unRollIn, &bUpdateFlags); 
   
	calcVelocity(unThrottleIn);
	ledAngle(PIDpitch[INPUT],PIDroll[INPUT]); //NY*******************
}

//Se referens , [http://rcarduino.blogspot.co.uk/2012/04/how-to-read-multiple-rc-channels-draft.html]
void readInterrupts(uint16_t * unThrottleIn, uint16_t * unPITCHIn, uint16_t * unYawIn, uint16_t * unRollIn, uint16_t * bUpdateFlags) // we have local copies of the PID_INPUTs, so now we can turn interrupts back on
{
	noInterrupts();
	*bUpdateFlags = bUpdateFlagsShared;
    
	if(*bUpdateFlags & THROTTLE_FLAG) *unThrottleIn = unThrottleInShared;
	if(*bUpdateFlags & PITCH_FLAG) *unPITCHIn = unPITCHInShared;
	if(*bUpdateFlags & Yaw_FLAG) *unYawIn = unYawInShared;
	if(*bUpdateFlags & ROLL_FLAG) *unYawIn = unYawInShared;
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
}

void mapController(uint16_t unPitchIn, uint16_t unYawIn)
{
   _pitch = map(unPitchIn,935,1780,-40,40);
   _yaw = map(unYawIn,935,1780,-20,20);
}

//Se referens , [http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/]
void calcPID(float * PIDaxis, double * errSum, unsigned long * lastTime)
{
	unsigned long now = millis();
	double timeChange = (double)(now - *lastTime);
   
	if(timeChange >= SampleTime)
	{
		double error = PIDaxis[SP] - PIDaxis[PID_INPUT]; //the error
		*errSum += (PIDaxis[Ki] * error); //the integral
		double dPID_INPUT = (PIDaxis[PID_INPUT] - PIDaxis[LAST_INPUT]); //the dy/dx
     
		PIDaxis[PID_OUTPUT] = (PIDaxis[Kp]) * error + (*errSum) + (PIDaxis[Kd]) * dPID_INPUT;
		PIDaxis[LAST_INPUT] = PIDaxis[PID_INPUT]; 
		*lastTime = now;
		//printOut2(timeChange,error,errSum,dPID_INPUT,PIDaxis[PID_OUTPUT]);
	}
}

void calcVelocity(int throttle)
{
	int frontThr, backThr, rightThr, leftThr;
	if(throttle > 1000)
	{
	//Se referens , [http://en.wikipedia.org/wiki/Helicopter_flight_controls]
					   //[http://theboredengineers.com/2012/05/the-quadcopter-basics/]
					   //[http://robotics.stackexchange.com/questions/1838/how-yaw-pitch-and-roll-effect-the-flight-of-quadcopter]
					   
		frontThr = throttle + PIDpitch[PID_OUTPUT] - PIDyaw[PID_OUTPUT]; //- controllerOutput(_pitch,_yaw);
		backThr = throttle - PIDpitch[PID_OUTPUT]  - PIDyaw[PID_OUTPUT]; //+ controllerOutput(_pitch,_yaw);
		rightThr = throttle + PIDroll[PID_OUTPUT] + PIDyaw[PID_OUTPUT];
		leftThr = throttle - PIDroll[PID_OUTPUT] + PIDyaw[PID_OUTPUT];
	}
	else if(throttle <= 1000)
	{
		frontThr = throttle;
		backThr = throttle;
		rightThr = throttle;
		leftThr = throttle;
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
	//digitalWrite(ledFram,HIGH);
	mFront.writeMicroseconds( front  ); //to make sure that u wont exceed the limited throttle
	mBack.writeMicroseconds( back );
	mRight.writeMicroseconds( right  );
	mLeft.writeMicroseconds( left  );
	//printOut(front,back,right,left);
} 

void ledAngle(float pitchAngle, float rollAngle)
{
	if(rollAngle > 10)
	{
		//digitalWrite(ledRight,HIGH);
		//digitalWrite(ledLeft,LOW);
		LEDshow(3,3,1,0);
	}
	else if(rollAngle < -10)
	{
		//digitalWrite(ledRight,LOW);
		//digitalWrite(ledLeft,HIGH);
		LEDshow(3,3,0,1);
	}
	
	if(pitchAngle > 10)
	{
		//digitalWrite(ledFram,LOW);
		//digitalWrite(ledBak,HIGH);
		LEDshow(0,1,3,3);
	}
	else if(pitchAngle < -10)
	{
		//digitalWrite(ledFram,HIGH);
		//digitalWrite(ledBak,LOW);
		LEDshow(1,0,3,3);
	}
	else if(pitchAngle >= -10 && pitchAngle <= 10 && rollAngle >= -10 && rollAngle <= 10)
	{
		//digitalWrite(ledFram,HIGH);
		//digitalWrite(ledBak,HIGH);
		//digitalWrite(ledRight,HIGH);
		//digitalWrite(ledLeft,HIGH);
		//LEDshow(1,1,1,1);
	}
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

void printOut2(double timeChange, double error, double *_errSum, double dPID_INPUT, float PIDOutput)
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
	Serial.println(PIDOutput);
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

//Se referens , [http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/]
void setTunings(float* _Kp, float* _Ki, float* _Kd)
{
   float SampleTimeInSec = (float)SampleTime/1000;
   *_Ki *= SampleTimeInSec;
   *_Kd /= SampleTimeInSec;
}

void initInterrupts()
{
	PCintPort::attachInterrupt(THROTTLE_IN_PIN, calcThrottle, CHANGE);
	//PCintPort::attachInterrupt(PITCH_IN_PIN, calcPITCH, CHANGE);
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
	for(int i = 0; i < 2000; i++){
		sixDOF.getYawPitchRoll(ypr);
		
		yawOffset = ypr[0];
		pitchOffset = ypr[1];
		rollOffset = ypr[2];
		//printOut(yawOffset, pitchOffset, rollOffset);
		//yprSum[j]+=ypr[j];
	}
	/*yawOffset = ypr[0];
	pitchOffset = ypr[1];
	rollOffset = ypr[2];
	printOut(yawOffset, pitchOffset, rollOffset);*/
}

//------------------------ ISR ROUTINES ---------------------------------
//Se referens , http://rcarduino.blogspot.co.uk/2012/04/how-to-read-multiple-rc-channels-draft.html
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
/*void calcYaw()
{
  if(digitalRead(Yaw_IN_PIN) == HIGH)
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
    ulPITCHStart = micros();
  else
  {
    unRollInShared = (uint16_t)(micros() - ulRollStart);
    bUpdateFlagsShared |= ROLL_FLAG;
  }
}*/
