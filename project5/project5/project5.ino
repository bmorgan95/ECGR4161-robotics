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

#define TURNSPEED 10
#define DRIVESPEED 10

bool toyCarMode = false;

void setup()
{
  Serial.begin(9600);

  setupRSLK();
  /* Left button on Launchpad */
  setupWaitBtn(LP_LEFT_BTN);
  /* Red led in rgb led */
  setupLed(RED_LED);
  clearMinMax(sensorMinVal,sensorMaxVal);
  floorCalibration();
}



void loop()
{
//  uint16_t normalSpeed = 10;
//  uint16_t fastSpeed = 20;

  /* Valid values are either:
   *  DARK_LINE  if your floor is lighter than your line
   *  LIGHT_LINE if your floor is darker than your line
   */
  uint8_t lineColor = DARK_LINE;


while(finishLine(sensorCalVal,lineColor) == false){

lineFollow();

//delay(1000);

//Serial.println("loopin the night away");

}


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

void lineFollow(){

  uint8_t lineColor = DARK_LINE;

    readLineSensor(sensorVal);
  readCalLineSensor(sensorVal,
            sensorCalVal,
            sensorMinVal,
            sensorMaxVal,
            lineColor);



  uint32_t linePos = getLinePosition(sensorCalVal,lineColor);
  Serial.println(linePos);

  //when robot is far left of center, sharper left turn and scoot forward
  if (linePos < 1501){
    Serial.println("Big left turn");
    lineCCWDeg(20);
    lineStraight(1);
  }

  //when robot is slightly left of center, slight left turn and scoot forward
  if ((linePos > 1500) and (linePos < 3000)){
    Serial.println("Small feft turn");
    lineCCWDeg(10);
    lineStraight(1);
  }

  //when robot is centered on the line, scoot forward
  if ((linePos > 2999) and (linePos < 4001)){
    Serial.println("Forward");
    lineStraight(1);
  }

  //when robot is slightly right of center, slight right turn and scoot forward
  if ((linePos > 4000) and (linePos < 5500)){
    Serial.println("Small right turn");
    lineCWDeg(10);
    lineStraight(1);
  }

  //when robot is far right of center, sharper right turn and scoot forward
  if (linePos > 5499){
    Serial.println("Big right turn");
    lineCWDeg(20);
    lineStraight(1);
  }
  
}

void lineCCWDeg(int turnAngle){
  Serial.println("turning Left");


  
  setMotorDirection(LEFT_MOTOR,MOTOR_DIR_BACKWARD);
  setMotorDirection(RIGHT_MOTOR,MOTOR_DIR_FORWARD);
  setMotorSpeed(BOTH_MOTORS,TURNSPEED);

  if (toyCarMode == false) {enableMotor(BOTH_MOTORS);}

  int l_totalCount = getEncoderLeftCnt(); int r_totalCount = getEncoderRightCnt();
  int avg_totalCount = (l_totalCount + r_totalCount) / 2;
  int turnPulses = (2.00714*(turnAngle*1.00625-3));
  int OLD_ENC = avg_totalCount;
  
  while (((getEncoderLeftCnt() + getEncoderRightCnt()) / 2 - OLD_ENC) < turnPulses){
    
  }

disableMotor(BOTH_MOTORS);

}

void lineCWDeg(int turnAngle){
  Serial.println("Turning Right");


  
  setMotorDirection(LEFT_MOTOR,MOTOR_DIR_FORWARD);
  setMotorDirection(RIGHT_MOTOR,MOTOR_DIR_BACKWARD);
  setMotorSpeed(BOTH_MOTORS,TURNSPEED);

  if (toyCarMode == false) {enableMotor(BOTH_MOTORS);}

  int l_totalCount = getEncoderLeftCnt(); int r_totalCount = getEncoderRightCnt();
  int avg_totalCount = (l_totalCount + r_totalCount) / 2;
  int turnPulses = (2.00714*(turnAngle*1.00625-3));
  int OLD_ENC = avg_totalCount;
  
  while (((getEncoderLeftCnt() + getEncoderRightCnt()) / 2 - OLD_ENC) < turnPulses){
    
  }

disableMotor(BOTH_MOTORS);

}

void lineStraight(int dist){
  Serial.println("driving straight");

  setMotorDirection(LEFT_MOTOR,MOTOR_DIR_FORWARD);
  setMotorDirection(RIGHT_MOTOR,MOTOR_DIR_FORWARD);
  setMotorSpeed(BOTH_MOTORS,DRIVESPEED);

  if (toyCarMode == false) {enableMotor(BOTH_MOTORS);}

  int l_totalCount = getEncoderLeftCnt(); int r_totalCount = getEncoderRightCnt();
  int avg_totalCount = (l_totalCount + r_totalCount) / 2;
  int drivePulses = (16.37*dist);
  int OLD_ENC = avg_totalCount;
  
  while (((getEncoderLeftCnt() + getEncoderRightCnt()) / 2 - OLD_ENC) < drivePulses){
    
  }

disableMotor(BOTH_MOTORS);

}

bool finishLine (uint16_t* calVal, uint8_t mode){

  bool finish;

  uint32_t avg = 0; // this is for the weighted total
  uint32_t sum = 0; // this is for the denominator, which is <= 64000

  uint32_t _lastPosition;
  for (uint8_t i = 0; i < LS_NUM_SENSORS; i++)
  {
    uint16_t value = calVal[i];
    Serial.println(value);

    // only average in values that are above a noise threshold
    if (value > 50)
    {
      sum += (value);
    }
  }
Serial.println();
Serial.println(sum);

if (sum > 6000){
  Serial.println("Finish!!!!");
  finish = true;
  }

  else {
  Serial.println("Not there yet...");
  finish = false;
  }
return finish; 
}
