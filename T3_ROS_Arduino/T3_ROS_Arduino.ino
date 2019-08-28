//Arduino Mega based 6 channel R/C Reader + Signal Writer to 6 ESCs
//Modified to
//Arduino Due based 6 channel R/C Reader + Signal Writer to 6 ESCs
//Modified to
//Arduino Due based 6 channel R/C Reader + Signal Writer to 4 ESCs for T3_Multirotor
//Modified to
//Arduino MKR Zero based 6 channel R/C Reader + Signal Writer to 4 ESCs for T3_V3
//Modified for
//ROS competible system

//reference_publish:  http://wiki.ros.org/rosserial_arduino/Tutorials/Hello%20World
//reference_subscribe:http://wiki.ros.org/rosserial_arduino/Tutorials/Blink
//reference_array:    http://wiki.ros.org/rosserial_arduino/Tutorials/Servo%20Controller

//To interact w. ROS on computer, run rosrun rosserial_python serial_node.py /dev/"@devicename@"

// Arduino Due can change its frequency by modifying the varient.h file
// C:\Users\sjlaz\AppData\Local\Arduino15\packages\arduino\hardware\sam\1.6.10\variants\arduino_due_x
// Reference:         https://folk.uio.no/jeanra/Microelectronics/GettingTheBestOutOfPWMDue.html

// Update Logs
// 2016.05.04 First build
// 2016.05.11 Changed SW functions ex)calcSW
// 2017.01.18 Code modified for Arduino Due
// 2018.08.09 Code modified to operate under the ROS system w. rosserial
// 2018.08.15 Remapped the analogwrite->PWM relationship
// 2018.08.18 Activated 7th R/C channel
// 2018.10.22 Added Voltage Reading sensor code
// 2019.08.25 Modified for MKR Zero competitivity

// Seung Jae Lee
// Seoul National University
// Department of Mechanical & Aerospace Engineering

// Loop Frequency : around 500 Hz
// Generated PWM Frequency : 455.846 Hz

// For ROS==============================================================================
#define USE_USBCON  //use the following line if you have an arduino w. native port


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
#define MASTERSWITCH_PIN      8
#define AUTOMANUALSWITCH_PIN  9
#define RC_ROLL_PIN           A1
#define RC_PITCH_PIN          A2  //
#define RC_YAW_PIN            0
#define RC_THRUST_PIN         1   //
#define RC_AUX_1_PIN          A3
#define RC_AUX_2_PIN          A4
//-------------------------------------------------------------------------------------

// Assign channel out pins=============================================================
#define PROP_ONE_OUT_PIN 9
#define PROP_TWO_OUT_PIN 8
#define PROP_THR_OUT_PIN 7
#define PROP_FOU_OUT_PIN 6
//-------------------------------------------------------------------------------------

//Flags===============================================================
#define SW_FLAG       1
#define ROLL_FLAG     2
#define PITCH_FLAG    4
#define YAW_FLAG      8
#define THRUST_FLAG   16
#define MSSW_FLAG     32
#define AUX_1_FLAG    64
#define AUX_2_FLAG    128

volatile uint8_t bUpdateFlagsShared;
//---------------------------------------------------------------------

//Set volatile variables===============================================================
// shared variables are updated by the ISR and read by loop.
// In loop we immediatley take local copies so that the ISR can keep ownership of the
// shared ones. To access these in loop
// we first turn interrupts off with noInterrupts
// we take a copy to use in loop and the turn interrupts back on
// as quickly as possible, this ensures that we are always able to receive new signals

volatile uint16_t unSWInShared;
volatile uint16_t unMSSWInShared;
volatile uint16_t unROLLInShared;
volatile uint16_t unPITCHInShared;
volatile uint16_t unYAWInShared;
volatile uint16_t unTHRUSTInShared;
volatile uint16_t unA1InShared;
volatile uint16_t unA2InShared;
//------------------------------------------------------------------------------------

//Pwm Rising Time recording variables=================================================
// These are used to record the rising edge of a pulse in the calcInput functions
// They do not need to be volatile as they are only used in the ISR. If we wanted
// to refer to these in loop and the ISR then they would need to be declared volatile

uint32_t ulSWStart;
uint32_t ulMSSWStart;
uint32_t ulROLLStart;
uint32_t ulPITCHStart;
uint32_t ulYAWStart;
uint32_t ulTHRUSTStart;
uint32_t ulA1Start;
uint32_t ulA2Start;
//-----------------------------------------------------------------------------------

//Input variables====================================================================
int u1, u2, u3, u4;
//-----------------------------------------------------------------------------------

//Input value=====================================================================
// create local variables to hold a local copies of the channel inputs
// these are declared static so that thier values will be retained
// between calls to loop.
static uint16_t unSWIn;
static uint16_t unMSSWIn;
static uint16_t unROLLIn;
static uint16_t unPITCHIn;
static uint16_t unYAWIn;
static uint16_t unTHRUSTIn;
static uint16_t unA1In;
static uint16_t unA2In;
//---------------------------------------------------------------------------------

//Write PWM signals to motor ESCs===========================================================
void writePWM_Kill() {//Kill all motors
  u1 = 1000;
  u2 = 1000;
  u3 = 1000;
  u4 = 1000;

  writePWM(u1, u2, u3, u4);
}

void writePWM_Odroid(const std_msgs::Int16MultiArray &cmd_msg) {//Receive Motor Cmds from ROS
  u1 = cmd_msg.data[0];
  u2 = cmd_msg.data[1];
  u3 = cmd_msg.data[2];
  u4 = cmd_msg.data[3];

  u1 = constrain( u1, 1000, 2000 );
  u2 = constrain( u2, 1000, 2000 );
  u3 = constrain( u3, 1000, 2000 );
  u4 = constrain( u4, 1000, 2000 );

  //Kill UAV================================================================================
  if ( unMSSWIn < 1500 ) {
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
    REG_TCC0_CC0 = 24 * ua1;
    REG_TCC0_CC1 = 24 * ua2;
    REG_TCC0_CC2 = 24 * ua3;
    REG_TCC0_CC3 = 24 * ua4;
  //--------------------------------------------------------------------------------------
}
//------------------------------------------------------------------------------------------


//Interrupt service routine=================================================================

void calcSW() {
  if (digitalRead(AUTOMANUALSWITCH_PIN)) {
    ulSWStart = micros();
  }
  else  {
    unSWInShared = (uint16_t)(micros() - ulSWStart);
    bUpdateFlagsShared |= SW_FLAG;
  }
}

void calcMSSW() {
  if (digitalRead(MASTERSWITCH_PIN))  {
    ulMSSWStart = micros();
  }
  else  {
    unMSSWInShared = (uint16_t)(micros() - ulMSSWStart);
    bUpdateFlagsShared |= MSSW_FLAG;
  }
}

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

void calcYAW() {
  if (digitalRead(RC_YAW_PIN))  {
    ulYAWStart = micros();
  }
  else  {
    unYAWInShared = (uint16_t)(micros() - ulYAWStart);
    bUpdateFlagsShared |= YAW_FLAG;
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
void calcAUX1() {
  if (digitalRead(RC_AUX_1_PIN)) {
    ulA1Start = micros();
  }
  else  {
    unA1InShared = (uint16_t)(micros() - ulA1Start);
    bUpdateFlagsShared |= AUX_1_FLAG;
  }
}
void calcAUX2() {
  if (digitalRead(RC_AUX_2_PIN)) {
    ulA2Start = micros();
  }
  else  {
    unA2InShared = (uint16_t)(micros() - ulA2Start);
    bUpdateFlagsShared |= AUX_2_FLAG;
  }
}
//-------------------------------------------------------------------------------------------

// For ROS==================================================================================
ros::Subscriber<std_msgs::Int16MultiArray> PWMs("PWMs", writePWM_Odroid);
//------------------------------------------------------------------------------------------

void setup() {
  Serial.begin(57600);

  // For ROS========================================================================
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  RC.data = (short int *)malloc(sizeof(short int) * 2);
  //RC.data=(std_msgs::Int16MultiArray::_data_type*) malloc(2*sizeof(std_msgs::Int16MultiArray::_data_type));
  RC.data_length = 7;

//  RC.layout.dim=(std_msgs::MultiArrayDimension *)
//  malloc(sizeof(std_msgs::MultiArrayDimension) *2);
//  RC.layout.dim[0].label = "RC";
//  RC.layout.dim[0].size = 7;
//  RC.layout.dim[0].stride = 1*7;
//  RC.layout.data_offset = 0;
//  RC.layout.dim_length = 1;
//  RC.data_length = 7;
//  RC.data = (short int *)malloc(sizeof(short int)*7);
  

  nh.advertise(RC_readings);
  nh.subscribe(PWMs);
  //--------------------------------------------------------------------------------

  //Attach Interrupts===============================================================
  // used to read the channels
  attachInterrupt(AUTOMANUALSWITCH_PIN, calcSW, CHANGE);
  attachInterrupt(MASTERSWITCH_PIN, calcMSSW, CHANGE);
  attachInterrupt(RC_ROLL_PIN, calcROLL, CHANGE);
  attachInterrupt(RC_PITCH_PIN, calcPITCH, CHANGE);
  attachInterrupt(RC_YAW_PIN, calcYAW, CHANGE);
  attachInterrupt(RC_THRUST_PIN, calcTHRUST, CHANGE);
  attachInterrupt(RC_AUX_1_PIN, calcAUX1, CHANGE);
  attachInterrupt(RC_AUX_2_PIN, calcAUX2, CHANGE);
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

  //pinMode(A8, INPUT);//used for analog voltage check
  //pinMode(A10, INPUT);//used for analog voltage check
  //pinMode(9,OUTPUT); //used for checking loop time
  //pinMode(13,OUTPUT);//used for latency time check
  //-------------------------------------------------------------------------------

  //  // Output 250kHz PWM on timer TCC0 (6-bit resolution)//https://forum.arduino.cc/index.php?topic=346731.0
    REG_GCLK_GENDIV = GCLK_GENDIV_DIV(1) |          // Divide the 48MHz clock source by divisor 1: 48MHz/1=48MHz
                      GCLK_GENDIV_ID(4);            // Select Generic Clock (GCLK) 4
    while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization
  
    REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOW
                       GCLK_GENCTRL_GENEN |         // Enable GCLK4
                       GCLK_GENCTRL_SRC_DFLL48M |   // Set the 48MHz clock source
                       GCLK_GENCTRL_ID(4);          // Select GCLK4
    while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization
  
    // Enable the port multiplexer for the digital pin D4,D5,D6,D7
    PORT->Group[g_APinDescription[4].ulPort].PINCFG[g_APinDescription[4].ulPin].bit.PMUXEN = 1;
    PORT->Group[g_APinDescription[5].ulPort].PINCFG[g_APinDescription[5].ulPin].bit.PMUXEN = 1;
    PORT->Group[g_APinDescription[6].ulPort].PINCFG[g_APinDescription[6].ulPin].bit.PMUXEN = 1;
    PORT->Group[g_APinDescription[7].ulPort].PINCFG[g_APinDescription[7].ulPin].bit.PMUXEN = 1;
  
    // Connect the TCC0 timer to digital output D7 - port pins are paired odd PMUO and even PMUXE
    // F & E specify the timers: TCC0, TCC1 and TCC2
    PORT->Group[g_APinDescription[4].ulPort].PMUX[g_APinDescription[4].ulPin >> 1].reg = PORT_PMUX_PMUXO_F | PORT_PMUX_PMUXE_F;
    PORT->Group[g_APinDescription[6].ulPort].PMUX[g_APinDescription[6].ulPin >> 1].reg = PORT_PMUX_PMUXO_F | PORT_PMUX_PMUXE_F;
  
    // Feed GCLK4 to TCC0 and TCC1
    REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable GCLK4 to TCC0 and TCC1
                       GCLK_CLKCTRL_GEN_GCLK4 |     // Select GCLK4
                       GCLK_CLKCTRL_ID_TCC0_TCC1;   // Feed GCLK4 to TCC0 and TCC1
    while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization
  
    // Dual slope PWM operation: timers countinuously count up to PER register value then down 0
    REG_TCC0_WAVE |= TCC_WAVE_POL(0xF) |         // Reverse the output polarity on all TCC0 outputs
                     TCC_WAVE_WAVEGEN_DSBOTH;    // Setup dual slope PWM on TCC0
    while (TCC0->SYNCBUSY.bit.WAVE);               // Wait for synchronization
  
    // Each timer counts up to a maximum or TOP value set by the PER register,
    // this determines the frequency of the PWM operation:
    REG_TCC0_PER = 50000;         // Set the frequency of the PWM on TCC0 to 250kHz
    while (TCC0->SYNCBUSY.bit.PER);                // Wait for synchronization
  
    // Set the PWM signal to output 50% duty cycle
    //REG_TCC0_CC3 = 24000;         // TCC0 CC3 - on D7
    //while (TCC0->SYNCBUSY.bit.CC3);                // Wait for synchronization
  
    // Divide the 48MHz signal by 1 giving 48MHz (20.83ns) TCC0 timer tick and enable the outputs
    REG_TCC0_CTRLA |= TCC_CTRLA_PRESCALER_DIV1 |    // Divide GCLK4 by 1
                      TCC_CTRLA_ENABLE;             // Enable the TCC0 output
    while (TCC0->SYNCBUSY.bit.ENABLE);              // Wait for synchronization/
}

void loop() {

  //Local copy of update flags======================================================
  static uint8_t bUpdateFlags;
  //---------------------------------------------------------------------------------

  //Update RCinput====================================================================

  if (bUpdateFlagsShared) {
    noInterrupts();
    bUpdateFlags = bUpdateFlagsShared;

    if (bUpdateFlags & SW_FLAG) {
      unSWIn = unSWInShared;
    }
    if (bUpdateFlags & MSSW_FLAG) {
      unMSSWIn = unMSSWInShared;
    }
    if (bUpdateFlags & ROLL_FLAG) {
      unROLLIn = unROLLInShared;
    }
    if (bUpdateFlags & PITCH_FLAG) {
      unPITCHIn = unPITCHInShared;
    }
    if (bUpdateFlags & YAW_FLAG) {
      unYAWIn = unYAWInShared;
    }
    if (bUpdateFlags & THRUST_FLAG) {
      unTHRUSTIn = unTHRUSTInShared;
    }
    if (bUpdateFlags & AUX_1_FLAG) {
      unA1In = unA1InShared;
    }
    if (bUpdateFlags & AUX_2_FLAG) {
      unA2In = unA2InShared;
    }

    bUpdateFlagsShared = 0;
    interrupts();
  }
  //----------------------------------------------------------------------------------------

  //Make constraint from 1000 to 2000 preventing overshoot==================================
  unSWIn = constrain( unSWIn, 1000, 2000 );
  unMSSWIn = constrain(unMSSWIn, 1000, 2000 );
  unROLLIn = constrain( unROLLIn, 1000, 2000 );
  unPITCHIn = constrain( unPITCHIn, 1000, 2000 );
  unYAWIn = constrain( unYAWIn, 1000, 2000 );
  unTHRUSTIn = constrain( unTHRUSTIn, 1000, 2000 );
  unA1In = constrain( unA1In, 1000, 2000);
  unA2In = constrain( unA2In, 1000, 2000);
  //------------------------------------------------------------------------------------------

  //Kill UAV================================================================================
  if ( unMSSWIn < 1500 ) {
    writePWM_Kill();
  }
  //----------------------------------------------------------------------------------------

  //Read Battery Voltage===================================================================
  //float voltage;
  //voltage = analogRead(A8)/(float)0.1223;//mV;the number 0.1223 has been achieved by empherical approach
  //----------------------------------------------------------------------------------------

  // For ROS (Write to odroid ROS)==========================================================
  RC.data[0] = unROLLIn;
  RC.data[1] = unPITCHIn;
  RC.data[2] = unYAWIn;
  RC.data[3] = unTHRUSTIn;
  RC.data[4] = unSWIn;
  RC.data[5] = unA1In;
  RC.data[6] = unA2In;

//  RC.data[0] = 1200;
//  RC.data[1] = 1200;
//  RC.data[2] = 1200;/
//  RC.data[3] = 1200;/
//  RC.data[4] = 1200;/
//  RC.data[5] = 1200;/
//  RC.data[6] = 1200;  /
  
  //RC.data[6] = voltage;//mV;battery voltage

  RC_readings.publish(&RC);
  nh.spinOnce();
  //----------------------------------------------------------------------------------------

  //For checking the latency btn cmd and response===========================================
  //if(unTHRUSTIn==u1){
  //  digitalWrite(13,HIGH);
  //}
  //else{
  //  digitalWrite(13,LOW);
  //}
  //----------------------------------------------------------------------------------------

  delay(2);
  ///  sleep();
}
