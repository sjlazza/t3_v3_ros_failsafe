//Arduino Mega based 6 channel R/C Reader + Signal Writer to 6 ESCs
//Modified to
//Arduino Due based 6 channel R/C Reader + Signal Writer to 6 ESCs
//Modified to
//Arduino Due based 6 channel R/C Reader + Signal Writer to 4 ESCs for T3_Multirotor
//Modified to
//Arduino Due based 8 channel R/C Reader + Signal Writer to 4 ESCs for T3_V3
//Modified for
//ROS competible system

//reference_publish:  http://wiki.ros.org/rosserial_arduino/Tutorials/Hello%20World
//reference_subscribe:http://wiki.ros.org/rosserial_arduino/Tutorials/Blink
//reference_array:    http://wiki.ros.org/rosserial_arduino/Tutorials/Servo%20Controller

//To interact w. ROS on computer, run rosrun rosserial_python serial_node.py /dev/"@devicename@"

// Arduino Due can change its frequency by modifying the varient.h file
// C:\Users\sjlaz\AppData\Local\Arduino15\packages\arduino\hardware\sam\1.6.10\variants\arduino_due_x
// /home/[username]/.arduino15/packages/arduino/hardware/sam/1.6.12/variants/arduino_due_x
// Reference:         https://folk.uio.no/jeanra/Microelectronics/GettingTheBestOutOfPWMDue.html

// Update Logs
// 2016.05.04 First build
// 2016.05.11 Changed SW functions ex)calcSW
// 2017.01.18 Code modified for Arduino Due
// 2018.08.09 Code modified to operate under the ROS system w. rosserial
// 2018.08.15 Remapped the analogwrite->PWM relationship
// 2019.08.27 Modified for T3_V3

// Seung Jae Lee
// Seoul National University
// Department of Mechanical & Aerospace Engineering

// Loop Frequency : around around 145 kHz
// Generated PWM Frequency : 455.846 Hz

// For ROS==============================================================================
#define USE_USBCON //use the following line if you have an arduino w. native port

#include <ros.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int16.h>
//--------------------------------------------------------------------------------------

#if defined(ARDUINO) && ARDUINO >=100
#include "Arduino.h"
#else
#include <WProgram.h>
#endif

// For ROS==============================================================================
ros::NodeHandle nh;

std_msgs::Int16MultiArray RC;
ros::Publisher RC_readings("RC_readings", &RC);
//--------------------------------------------------------------------------------------

//Assign channel in pins================================================================
#define RC_ROLL_PIN           51
#define RC_PITCH_PIN          49
#define RC_THRUST_PIN         47
#define RC_YAW_PIN            45
#define AUX_1_PIN             43
#define AUX_2_PIN             41
#define AUX_3_PIN             39
#define MASTERSWITCH_PIN      37

#define BATT_V_PIN            A7
//-------------------------------------------------------------------------------------

// Assign channel out pins=============================================================
#define PROP_ONE_OUT_PIN 7
#define PROP_TWO_OUT_PIN 6
#define PROP_THR_OUT_PIN 5
#define PROP_FOU_OUT_PIN 4
//-------------------------------------------------------------------------------------

//Flags===============================================================
#define ROLL_FLAG     1
#define PITCH_FLAG    2
#define THRUST_FLAG   4
#define YAW_FLAG      8
#define AUX_1_FLAG    16
#define AUX_2_FLAG    32
#define AUX_3_FLAG    64
#define MSSW_FLAG     128

volatile uint16_t bUpdateFlagsShared;
//---------------------------------------------------------------------


//Input value=====================================================================
// create local variables to hold a local copies of the channel inputs
// these are declared static so that thier values will be retained
// between calls to loop.

static uint16_t unROLLIn;
static uint16_t unPITCHIn;
static uint16_t unTHRUSTIn;
static uint16_t unYAWIn;
static uint16_t unA1In;
static uint16_t unA2In;
static uint16_t unA3In;
static uint16_t unMSIn;

static uint16_t voltage;
//---------------------------------------------------------------------------------

//Local copy of update flags======================================================
static uint16_t bUpdateFlags;
//---------------------------------------------------------------------------------

//Set volatile variables===============================================================
// shared variables are updated by the ISR and read by loop.
// In loop we immediatley take local copies so that the ISR can keep ownership of the
// shared ones. To access these in loop
// we first turn interrupts off with noInterrupts
// we take a copy to use in loop and the turn interrupts back on
// as quickly as possible, this ensures that we are always able to receive new signals

volatile uint16_t unROLLInShared;
volatile uint16_t unPITCHInShared;
volatile uint16_t unTHRUSTInShared;
volatile uint16_t unYAWInShared;
volatile uint16_t unA1InShared;
volatile uint16_t unA2InShared;
volatile uint16_t unA3InShared;
volatile uint16_t unMSInShared;
//------------------------------------------------------------------------------------

//Pwm Rising Time recording variables=================================================
// These are used to record the rising edge of a pulse in the calcInput functions
// They do not need to be volatile as they are only used in the ISR. If we wanted
// to refer to these in loop and the ISR then they would need to be declared volatile

uint32_t ulROLLStart;
uint32_t ulPITCHStart;
uint32_t ulTHRUSTStart;
uint32_t ulYAWStart;
uint32_t ulA1Start;
uint32_t ulA2Start;
uint32_t ulA3Start;
uint32_t ulMSStart;
//-----------------------------------------------------------------------------------

//Input variables====================================================================
int u1, u2, u3, u4;
//-----------------------------------------------------------------------------------

//For checking the loop frequency //around 145kHz====================================
static int flag = 0;
//-----------------------------------------------------------------------------------

//Write PWM signals to motor ESCs===========================================================
void writePWM_Kill() {
  u1 = 1000;
  u2 = 1000;
  u3 = 1000;
  u4 = 1000;

  u1 = constrain( u1, 1000, 2000 );
  u2 = constrain( u2, 1000, 2000 );
  u3 = constrain( u3, 1000, 2000 );
  u4 = constrain( u4, 1000, 2000 );

  writePWM(u1, u2, u3, u4);
}

void writePWM_Odroid(const std_msgs::Int16MultiArray &cmd_msg) {
  u1 = cmd_msg.data[0];
  u2 = cmd_msg.data[1];
  u3 = cmd_msg.data[2];
  u4 = cmd_msg.data[3];

  u1 = constrain( u1, 1000, 2000 );
  u2 = constrain( u2, 1000, 2000 );
  u3 = constrain( u3, 1000, 2000 );
  u4 = constrain( u4, 1000, 2000 );

  //Kill UAV================================================================================
  if ( unMSIn < 1500 ) {
    writePWM_Kill();
  }
  //----------------------------------------------------------------------------------------

  //Under Odroid's command==================================================================
  else {
    writePWM(u1, u2, u3, u4);
  }
  //----------------------------------------------------------------------------------------
}

void writePWM(int ua1, int ua2, int ua3, int ua4) {

  ua1 = constrain( ua1, 1000, 2000 );
  ua2 = constrain( ua2, 1000, 2000 );
  ua3 = constrain( ua3, 1000, 2000 );
  ua4 = constrain( ua4, 1000, 2000 );

  //Write PWM Values to motor=============================================================
  analogWrite(PROP_ONE_OUT_PIN, ua1 * 1.866891 + 0.868104);
  analogWrite(PROP_TWO_OUT_PIN, ua2 * 1.866891 + 0.868104);
  analogWrite(PROP_THR_OUT_PIN, ua3 * 1.866891 + 0.868104);
  analogWrite(PROP_FOU_OUT_PIN, ua4 * 1.866891 + 0.868104);
  //--------------------------------------------------------------------------------------
}
//------------------------------------------------------------------------------------------


//Interrupt service routine=================================================================
void calcROLL() {
  if (digitalRead(RC_ROLL_PIN))  {
    ulROLLStart = micros();
  }
  else  {
    unROLLInShared = (uint16_t)(micros() - ulROLLStart);
    bUpdateFlagsShared |= ROLL_FLAG;
  }
}

void calcPITCH() {
  if (digitalRead(RC_PITCH_PIN))  {
    ulPITCHStart = micros();
  }
  else  {
    unPITCHInShared = (uint16_t)(micros() - ulPITCHStart);
    bUpdateFlagsShared |= PITCH_FLAG;
  }
}

void calcTHRUST() {
  if (digitalRead(RC_THRUST_PIN))  {
    ulTHRUSTStart = micros();
  }
  else  {
    unTHRUSTInShared = (uint16_t)(micros() - ulTHRUSTStart);
    bUpdateFlagsShared |= THRUST_FLAG;
  }
}

void calcYAW() {
  if (digitalRead(RC_YAW_PIN))  {
    ulYAWStart = micros();
  }
  else  {
    unYAWInShared = (uint16_t)(micros() - ulYAWStart);
    bUpdateFlagsShared |= YAW_FLAG;
  }
}

void calcAUX1() {
  if (digitalRead(AUX_1_PIN)) {
    ulA1Start = micros();
  }
  else  {
    unA1InShared = (uint16_t)(micros() - ulA1Start);
    bUpdateFlagsShared |= AUX_1_FLAG;
  }
}

void calcAUX2() {
  if (digitalRead(AUX_2_PIN)) {
    ulA2Start = micros();
  }
  else  {
    unA2InShared = (uint16_t)(micros() - ulA2Start);
    bUpdateFlagsShared |= AUX_2_FLAG;
  }
}

void calcAUX3() {
  if (digitalRead(AUX_3_PIN)) {
    ulA3Start = micros();
  }
  else  {
    unA3InShared = (uint16_t)(micros() - ulA3Start);
    bUpdateFlagsShared |= AUX_3_FLAG;
  }
}

void calcMSSW() {
  if (digitalRead(MASTERSWITCH_PIN)) {
    ulMSStart = micros();
  }
  else  {
    unMSInShared = (uint16_t)(micros() - ulMSStart);
    bUpdateFlagsShared |= MSSW_FLAG;
  }
}
//-------------------------------------------------------------------------------------------


// For ROS==================================================================================
ros::Subscriber<std_msgs::Int16MultiArray> PWMs("PWMs", writePWM_Odroid);
//------------------------------------------------------------------------------------------

void setup() {
  Serial.begin(57600);

  // For ROS========================================================================
  nh.initNode();
  nh.advertise(RC_readings);
  nh.subscribe(PWMs);

  int RC_data_size=9;

  RC.layout.dim = (std_msgs::MultiArrayDimension *)
                  malloc(sizeof(std_msgs::MultiArrayDimension) * 2);
  RC.layout.dim[0].label = "RC";
  RC.layout.dim[0].size = RC_data_size;
  RC.layout.dim[0].stride = 1 * RC_data_size;
  RC.layout.dim_length = 0;
  RC.layout.data_offset = 0;
  RC.data = (std_msgs::Int16MultiArray::_data_type*) malloc(2 * sizeof(std_msgs::Int16MultiArray::_data_type));
  RC.data_length = RC_data_size;
  //--------------------------------------------------------------------------------

  //Attach Interrupts===============================================================
  // used to read the channels
  attachInterrupt(RC_ROLL_PIN,      calcROLL,   CHANGE);
  attachInterrupt(RC_PITCH_PIN,     calcPITCH,  CHANGE);
  attachInterrupt(RC_THRUST_PIN,    calcTHRUST, CHANGE);
  attachInterrupt(RC_YAW_PIN,       calcYAW,    CHANGE);
  attachInterrupt(AUX_1_PIN,        calcAUX1,   CHANGE);
  attachInterrupt(AUX_2_PIN,        calcAUX2,   CHANGE);
  attachInterrupt(AUX_3_PIN,        calcAUX3,   CHANGE);
  attachInterrupt(MASTERSWITCH_PIN, calcMSSW,   CHANGE);
  //-------------------------------------------------------------------------------

  //Read&Write Resolution Setting==================================================
  analogReadResolution(12);
  analogWriteResolution(12);
  //-------------------------------------------------------------------------------

  //Pinmode Setting================================================================
  pinMode(PROP_ONE_OUT_PIN, OUTPUT);
  pinMode(PROP_TWO_OUT_PIN, OUTPUT);
  pinMode(PROP_THR_OUT_PIN, OUTPUT);
  pinMode(PROP_FOU_OUT_PIN, OUTPUT);

  //pinMode(23,OUTPUT); //used for checking loop time - turns out that it works w. 145 kHz
  pinMode(BATT_V_PIN, INPUT); //used for battery voltage check
  //-------------------------------------------------------------------------------
}

void loop() {
  //Update RCinput====================================================================
  if (bUpdateFlagsShared) {
    noInterrupts();
    bUpdateFlags = bUpdateFlagsShared;

    if (bUpdateFlags & ROLL_FLAG) {
      unROLLIn = unROLLInShared;
    }
    if (bUpdateFlags & PITCH_FLAG) {
      unPITCHIn = unPITCHInShared;
    }
    if (bUpdateFlags & THRUST_FLAG) {
      unTHRUSTIn = unTHRUSTInShared;
    }
    if (bUpdateFlags & YAW_FLAG) {
      unYAWIn = unYAWInShared;
    }
    if (bUpdateFlags & AUX_1_FLAG) {
      unA1In = unA1InShared;
    }
    if (bUpdateFlags & AUX_2_FLAG) {
      unA2In = unA2InShared;
    }
    if (bUpdateFlags & AUX_3_FLAG) {
      unA3In = unA3InShared;
    }
    if (bUpdateFlags & MSSW_FLAG) {
      unMSIn = unMSInShared;
    }

    bUpdateFlagsShared = 0;
    interrupts();
    //}
    //----------------------------------------------------------------------------------------

    //Make constraint from 1000 to 2000 preventing overshoot==================================
    unROLLIn    = constrain( unROLLIn,    1000, 2000 );
    unPITCHIn   = constrain( unPITCHIn,   1000, 2000 );
    unTHRUSTIn  = constrain( unTHRUSTIn,  1000, 2000 );
    unYAWIn     = constrain( unYAWIn,     1000, 2000 );
    unA1In      = constrain( unA1In,      1000, 2000 );
    unA2In      = constrain( unA2In,      1000, 2000 );
    unA3In      = constrain( unA3In,      1000, 2000 );
    unMSIn      = constrain( unMSIn,      1000, 2000 );
    //----------------------------------------------------------------------------------------

    //Voltage Measurement=====================================================================
    voltage = analogRead(BATT_V_PIN)/2.50;//divider value is set heuristically
    //ref: http://shopping.interpark.com/product/productInfo.do?prdNo=6225508308&gclid=CjwKCAjwzJjrBRBvEiwA867bymuQbcY3ltYbQgD4HvxVbC845K4BIXtkXYBwLmLa2C2ijXE5ZtIRMRoC8r8QAvD_BwE
    //----------------------------------------------------------------------------------------

    //For ROS (R/W to odroid ROS)=============================================================
    RC.data[0] = unROLLIn;
    RC.data[1] = unPITCHIn;
    RC.data[2] = unYAWIn;
    RC.data[3] = unTHRUSTIn;
    RC.data[4] = unA1In;
    RC.data[5] = unA2In;
    RC.data[6] = unA3In;
    RC.data[7] = unMSIn;
    RC.data[8] = voltage;

    RC_readings.publish(&RC);
    //----------------------------------------------------------------------------------------
  }
  nh.spinOnce();

  //For checking the loop frequency //around 145kHz
  //  if (flag == 0) {
  //    digitalWrite(23, HIGH);
  //    flag = 1;
  //  }
  //  else {
  //    digitalWrite(23, LOW);
  //    flag = 0;
  //  }

  //For checking the latency btn cmd and response===========================================
  //if(unTHRUSTIn==u1){
  //  digitalWrite(13,HIGH);
  //}
  //else{
  //  digitalWrite(13,LOW);
  //}
  //----------------------------------------------------------------------------------------

  //  delay(2);
}
