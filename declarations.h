//Variable declarations and defines

#define escPin1 3
#define escPin2 5
#define escPin3 6
#define escPin4 9

#define PITCH 0
#define ROLL 1
#define YAW 2

#define yawSP 0
#define pitchSP 0
#define rollSP 0
 
#define yawKp 0
#define yawKi 0
#define yawKd 0

#define pitchKp 0.19 //mellan 0.18 - 0.19 
#define pitchKi 0
#define pitchKd 0

#define rollKp 0.19 //mellan 0.18 - 0.19 
#define rollKi 0
#define rollKd 0
            
#define PID_INPUT 0
#define LAST_INPUT 1
#define PID_OUTPUT 2
#define SP 3
#define Kp 4
#define Ki 5
#define Kd 6
#define lastErr 7

#define MAX_THROTTLE 1740
#define MIN_THROTTLE 932

#define THROTTLE_IN_PIN A0
#define PITCH_IN_PIN A1
#define Yaw_IN_PIN A2
#define ROLL_IN_PIN A3

#define ledFram 8
#define ledBak 10
#define ledRight 11
#define ledLeft 12


//Bitflags set in bUpdateFlagsShared to indicate which channels have new signals
#define THROTTLE_FLAG 3 //Throttle is on Ch3
#define PITCH_FLAG 1 //PITCH is on Ch1
#define Yaw_FLAG 2 //Yaw is on Ch2
#define ROLL_FLAG 4 //Roll is on Ch4

float PIDroll[8] = {0,0,0,rollSP, rollKp, rollKi, rollKd,0};
float PIDpitch[8] = {0,0,0,pitchSP, pitchKp, pitchKi, pitchKd,0};
float PIDyaw[8] = {0,0,0,yawSP, yawKp, yawKi, yawKd,0};

double errSum[3];
unsigned long lastTime[3];
int _pitch,_yaw,_roll;
int SampleTime = 20;

Servo mFront;
Servo mBack;
Servo mRight;
Servo mLeft;

float pitchOffset;
float rollOffset;
float yawOffset;

volatile uint16_t bUpdateFlagsShared; //variable to store flag
volatile uint16_t unThrottleInShared;
volatile uint16_t unPITCHInShared;
volatile uint16_t unYawInShared;
volatile uint16_t unRollInShared;

uint32_t ulThrottleStart;
uint32_t ulPITCHStart;
uint32_t ulYawStart;
uint32_t ulRollStart;
