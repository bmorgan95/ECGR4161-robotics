/*
 * Energia Robot Library for Texas Instruments' Robot System Learning Kit (RSLK)
 * Line Following Example
 *
 * Summary:
 * This example has the TI Robotic System Learning Kit (TI RSLK) follow a line
 * using a basic line following algorithm. This example works on a dark floor with
 * a white line or a light floor with a dark line. The robot first needs to be calibrated
 * Then place the robot on the hit the left button again to begin the line following.
 *
 * How to run:
 * 1) Push left button on Launchpad to have the robot perform calibration.
 * 2) Robot will drive forwards and backwards by a predefined distance.
 * 3) Place the robot center on the line you want it to follow.
 * 4) Push left button again to have the robot begin to follow the line.
 *
 * Parts Info:
 * o Black eletrical tape or white electrical tape. Masking tape does not work well
 *   with IR sensors.
 *
 * Learn more about the classes, variables and functions used in this library by going to:
 * https://fcooper.github.io/Robot-Library/
 *
 * Learn more about the TI RSLK by going to http://www.ti.com/rslk
 *
 * created by Franklin Cooper Jr.
 *
 * This example code is in the public domain.
 */

#include "SimpleRSLK.h"

uint16_t sensorVal[LS_NUM_SENSORS];
uint16_t sensorCalVal[LS_NUM_SENSORS];
uint16_t sensorMaxVal[LS_NUM_SENSORS];
uint16_t sensorMinVal[LS_NUM_SENSORS];

#define WHEELSPEED_L 10
#define WHEELSPEED_R 10
#define stepWait 50


void setup()
{
  Serial.begin(9600);

  setupRSLK();
  /* Left button on Launchpad */
  setupWaitBtn(LP_LEFT_BTN);
  /* Red led in rgb led */
  setupLed(RED_LED);
  clearMinMax(sensorMinVal,sensorMaxVal);
}

void floorCalibration() {
  /* Place Robot On Floor (no line) */
  delay(2000);
  String btnMsg = "Push left button on Launchpad to begin calibration.\n";
  btnMsg += "Make sure the robot is on the floor away from the line.\n";
  /* Wait until button is pressed to start robot */
  waitBtnPressed(LP_LEFT_BTN,btnMsg,RED_LED);

  delay(1000);

  Serial.println("Running calibration on floor");
  simpleCalibrate();
  Serial.println("Reading floor values complete");

  btnMsg = "Push left button on Launchpad to begin line following.\n";
  btnMsg += "Make sure the robot is on the line.\n";
  /* Wait until button is pressed to start robot */
  waitBtnPressed(LP_LEFT_BTN,btnMsg,RED_LED);
  delay(1000);

  enableMotor(BOTH_MOTORS);
}

void simpleCalibrate() {
  /* Set both motors direction forward */
  setMotorDirection(BOTH_MOTORS,MOTOR_DIR_FORWARD);
  /* Enable both motors */
  enableMotor(BOTH_MOTORS);
  /* Set both motors speed 20 */
  setMotorSpeed(BOTH_MOTORS,20);

  for(int x = 0;x<100;x++){
    readLineSensor(sensorVal);
    setSensorMinMax(sensorVal,sensorMinVal,sensorMaxVal);
  }

  /* Disable both motors */
  disableMotor(BOTH_MOTORS);
}

bool isCalibrationComplete = false;
void loop()
{
  uint16_t normalSpeed = 10;
  uint16_t fastSpeed = 20;

  /* Valid values are either:
   *  DARK_LINE  if your floor is lighter than your line
   *  LIGHT_LINE if your floor is darker than your line
   */
  uint8_t lineColor = DARK_LINE;

  /* Run this setup only once */
  if(isCalibrationComplete == false) {
    floorCalibration();
    isCalibrationComplete = true;
  }

  readLineSensor(sensorVal);
  readCalLineSensor(sensorVal,
            sensorCalVal,
            sensorMinVal,
            sensorMaxVal,
            lineColor);
  int ENCODER_DIFF = 0;
  int OLD_ENC = 0;
  int l_motor_speed = WHEELSPEED_L;
  int r_motor_speed = WHEELSPEED_R;

  while(finishLine() == false) {

//Serial.println("while loop");


    //run the loop as long as no collision is detected
    
    int l_totalCount = getEncoderLeftCnt(); int r_totalCount = getEncoderRightCnt(); 

    int avg_totalCount = (l_totalCount + r_totalCount) / 2;

    if ((avg_totalCount - OLD_ENC) > stepWait){
      uint32_t linePos = getLinePosition(sensorCalVal,lineColor);
      if (linePos < 3450){
        ENCODER_DIFF--;}
      if (linePos > 3550){
        ENCODER_DIFF++;}
      avg_totalCount = OLD_ENC;
      //Serial.println(ENCODER_DIFF);
      }
    
 //Serial.println("error correction");
    // if right motor is too fast, speed up the left motor and slow the right 
    if(((l_totalCount + ENCODER_DIFF) < r_totalCount) and (min(l_totalCount, r_totalCount) > 0)) {
    setMotorSpeed(LEFT_MOTOR, ++l_motor_speed);
    setMotorSpeed(RIGHT_MOTOR, --r_motor_speed);}
    Serial.println(l_motor_speed);

    // if left motor is too fast, speed up the right motor and slow the left
    if((r_totalCount < (l_totalCount + ENCODER_DIFF)) and (min(l_totalCount, r_totalCount) > 0)) {
    setMotorSpeed(RIGHT_MOTOR, ++r_motor_speed);
    setMotorSpeed(LEFT_MOTOR, --l_motor_speed);}
    Serial.println(l_motor_speed);
 
    //////// Stop motors if they reach 1 meter ///////////
    //if (l_totalCount >= num_pulses) disableMotor(LEFT_MOTOR); 
    //if (r_totalCount >= num_pulses ) disableMotor(RIGHT_MOTOR); 
  }
}

bool finishLine(){
  return false;
}
