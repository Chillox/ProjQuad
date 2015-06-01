
#include <FreeSixIMU.h> //Bibilioteket är modifierad, se längst ned
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>
#include <math.h>
#include <Wire.h>
#include "MegunoLink.h" //Need the software megunoLink PRO to work and the library

#define gxGain 1
#define gyGain 1
#define gzGain 1

//float angles[3]; // yaw pitch roll
//float avgAcc[3];

// Set the FreeSixIMU object
FreeSixIMU sixDOF = FreeSixIMU();
double sensorTimer;

float ypr[3];
float accOffset[3];
float gyroOffset[3];
float yprOffset[3];

double anglesGyro[3];
float anglesAcc[3];
double pitchAngle, rollAngle, yawAngle;

TimePlot yprPlot;

void setup() {
	Serial.begin(9600);
	Wire.begin();

	delay(5);
	sixDOF.init(); //begin the IMU
	delay(5);
  
	getOffset(accOffset, gyroOffset);
	getYPRoffset();
  
	sensorTimer = micros();
  //avgMeasure();
}

void loop() {
	sixDOF.getYawPitchRoll(ypr);
	calcAngle(anglesAcc, anglesGyro);
	float pitch = ypr[1] - yprOffset[1];
	megunoOutput(pitchAngle, pitch, yprOffset[1]);
}


void calcAngle(float * angleXYZ, double * gyroAngleXYZ) //(double *angleXYZ, float * gyroAngleXYZ)
{
	float accXYZ[3];
	float gyroValues[3];
	double gyroRates[3];
	
	sixDOF.getAccG(accXYZ); //++++++++
	sixDOF.getGyroRate(gyroValues);
	 
	//---------- Accelerometer----------------------
	accXYZ[0]-= accOffset[0];
	accXYZ[1]-= accOffset[1];
	
	angleXYZ[0] = (((atan2(accXYZ[0],accXYZ[2])) + PI )*(180/PI)) - 180; //pitch
	angleXYZ[1] = (((atan2(accXYZ[1], accXYZ[2])) +PI) * (180/PI) ) -180; //roll
	//angleXYZ[2] = ((atan2(accXYZ[1],accXYZ[0]))*(180/PI));
	
	//----------GyroScope---------------------------
	gyroRates[0] = ((double)gyroValues[0] - gyroOffset[0])* gxGain;
	gyroRates[1] = ((double)gyroValues[1] - gyroOffset[1])* gyGain;
	gyroRates[2] = ((double)gyroValues[2] - gyroOffset[2])* gzGain;
	
	gyroAngleXYZ[1] += -1*(gyroRates[1]*(double)(micros()-sensorTimer)/1000000); //pitch
	
	//Complimentary filter
	rollAngle  = (0.92*(rollAngle+(gyroRates[0]*(double)(micros()-sensorTimer)/1000000))) + (0.08*angleXYZ[1]);  
    pitchAngle = (0.92*(pitchAngle+(-1)*(gyroRates[1]*(double)(micros()-sensorTimer)/1000000))) + (0.08*angleXYZ[0]);  

	sensorTimer = micros();
}

//Measure offset with 1000 samples
void getOffset(float * accOffset, float * gyroOffset) {
  float accXsum = 0, accYsum = 0, accZsum = 0, gyroXsum = 0, gyroYsum=0, gyroZsum=0;
  
  float accXYZ[3];
  float gyroXYZ[3];
  
  for (int i = 0; i < 1000; i++)
  {
    sixDOF.getAccG(accXYZ); //Get the acceleration in G's
    accXsum += accXYZ[0];
    accYsum += accXYZ[1];
    accZsum += accXYZ[2];
  }
  accOffset[0] = accXsum / 1000;
  accOffset[1] = accYsum / 1000;
  accOffset[2] = accZsum / 1000;
  
  for (int i = 0; i < 1000; i++)
  {
    sixDOF.getGyroRate(gyroXYZ); //Gyro in deg/s
    gyroXsum += gyroXYZ[0];
    gyroYsum += gyroXYZ[1];
	gyroZsum += gyroXYZ[2];
  }
  gyroOffset[0] = gyroXsum/1000;
  gyroOffset[1] = gyroYsum/1000;
  gyroOffset[2] = gyroZsum/1000;
  
}

void getYPRoffset()
{
	float yprSum[3];
	float _ypr[3];
	for (int i = 0; i < 1000; i++)
  	{
    	sixDOF.getYawPitchRoll(_ypr); /
    	yprSum[0] += _ypr[0];
    	yprSum[1] += _ypr[1]; 
		yprSum[2] += _ypr[2];	
  	}
  	
  	for(int i = 0; i < 4000; i++){ //Getting a accurate reading of the angles after 2000 samples at start
		sixDOF.getYawPitchRoll(_ypr);
		if(i >= 2000)
		{
			yprSum[0] += _ypr[0];
    		yprSum[1] += _ypr[1]; 
			yprSum[2] += _ypr[2];
		}
	}
  	
  yprOffset[0] = yprSum[0]/2000;
  yprOffset[1] = yprSum[1]/2000;
  yprOffset[2] = yprSum[2]/2000;
}

void megunoOutput(double compPitch, float sixDOFPitch, float offset)
{
	yprPlot.SendData("compPitch", compPitch);
	yprPlot.SendData("6dofPitch", sixDOFPitch);
	yprPlot.SendData("offset", offset);
}


/* //Added functions in the FreeSixIMU.cpp file
void FreeSixIMU::getAccG(float * values) {
	float accval[3];
	acc.get_Gxyz(accval);
	values[0] = ((float) accval[0]);
	values[1] = ((float) accval[1]);
	values[2] = ((float) accval[2]);
}

void FreeSixIMU::getGyroRate(float * values)
{
	gyro.readGyro(&values[0], &values[1], &values[2]);
}
*/

/* //Added the 2 functions in the FreeSixIMU.h header file. Public functions
	void getAccG(float * values);
	void getGyroRate(float * values);
*/
