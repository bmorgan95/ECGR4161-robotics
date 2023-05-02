//
//  Bradley Morgan and Jacob Santschi
//  Intro to Robotics
//  5-2-2023
//  practical final
//

#include "SimpleRSLK.h"

#define SECONDS01 60
#define SECONDS02 30
#define SECONDS03 15

  #define PULSES_1CM 16.37022272
  #define ENCODER_DIFF 1  


long double encTarget = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  setupRSLK();

      pinMode(LP_LEFT_BTN, INPUT_PULLUP);
    pinMode(LP_RIGHT_BTN, INPUT_PULLUP);

    //reset time and distance measurements
  resetLeftEncoderCnt();  
  resetRightEncoderCnt();
}

void loop() {
  // put your main code here, to run repeatedly: 

      while(digitalRead(LP_LEFT_BTN) == HIGH){
    } //do nothing until left button

    //countdown
  for(int s=0; s<=2; s++){
  digitalWrite(BLUE_LED, HIGH);
  delay(500);
  digitalWrite(BLUE_LED, LOW);
  delay(500);
  }
  
  robotSecondHand (SECONDS01);
  robotSecondHand (SECONDS02);
  robotSecondHand (SECONDS03);
  
}

/////////////////////////////////////////////////////////
//Function Name: allStop
//Description: Bring robot to a stop by applying braking force to both wheels
//Input: none
//output: void
//ROBOT IS ASSUMED TO BE IN MOTION WHEN THIS FUNCTION IS CALLED
/////////////////////////////////////////////////////////
void allStop(){
  //enableMotor(BOTH_MOTORS);
  //set speed to zero rather than disabling in order to actively brake
  //setMotorSpeed(BOTH_MOTORS, 0);
  disableMotor(BOTH_MOTORS);
}


//////////////////////////////////////////////////////////
//Function Name: rotateDegrees
//Description: rotate on the spot by a given number of degrees,
//in the given direction,
//with error correction to maintain positional accuracy
//input: turnAngle, direction
//output: void
//ROBOT IS ASSUMED TO BE AT REST WHEN THIS FUNCTION IS CALLED
/////////////////////////////////////////////////////////////
void rotateDegrees (int turnAngle, String direction, int speedLeft, int speedRight, float overshoot, int correction, int endStop){

  // Define speed and encoder count variables 

  //set motor directions for clockwise turning
  if(direction == "CW"){
  setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
  setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_BACKWARD);
  }

  //set motor directions for counter-clockwise turning
  if(direction == "CCW"){
  setMotorDirection(LEFT_MOTOR, MOTOR_DIR_BACKWARD);
  setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
  }

  //14.05*pi cm per 1 degree of robot turn, 
  encTarget = encTarget + (2.00714*(turnAngle*1.00625-overshoot)); 
  //compensate for measurement error and stoppage rollover

  setRawMotorSpeed(LEFT_MOTOR, speedLeft);         // Set motor speeds - variable,  
  setRawMotorSpeed(RIGHT_MOTOR, speedRight);        //   may change (adjust) later 
  enableMotor(BOTH_MOTORS);                         // "Turn on" the motor  

    
    // if right motor is too fast, speed up the left motor and slow the right 
    while((getEncoderLeftCnt() < encTarget) or (getEncoderRightCnt()< encTarget)) {

    if(correction ==1){  
    if((getEncoderLeftCnt()+ENCODER_DIFF) < getEncoderRightCnt()) {
    setRawMotorSpeed(LEFT_MOTOR, speedLeft + 2);
    setRawMotorSpeed(RIGHT_MOTOR, speedRight - 2);
    //digitalWrite(RED_LED, HIGH);
    }

    // if left motor is too fast, speed up the right motor and slow the left
    else if((getEncoderRightCnt()+ENCODER_DIFF) < getEncoderLeftCnt()) {
    setRawMotorSpeed(RIGHT_MOTOR, speedRight + 2);
    setRawMotorSpeed(LEFT_MOTOR, speedLeft - 2);
    //digitalWrite(BLUE_LED, HIGH);
    }

    // if encoders are equal to within allowed offset, set motors to default speed
    else{
    setRawMotorSpeed(RIGHT_MOTOR, speedRight);
    setRawMotorSpeed(LEFT_MOTOR, speedLeft);
    }

    }

    if(endStop == 1){
     //Stop motors if they reach requisite pulses 
    if (getEncoderLeftCnt() >= encTarget) disableMotor(LEFT_MOTOR); 
    if (getEncoderRightCnt() >= encTarget) disableMotor(RIGHT_MOTOR);

    }

  }

  if(endStop == 1){
  //just in case, stop both motors after loop exits
  disableMotor(LEFT_MOTOR); 
  disableMotor(RIGHT_MOTOR);
  }

}

////////////////////////////////////////
//function: robotsecondhand
//description: move robot like clock second hand
//input: uint16_t seconds
//output void
//////////////////////////////////////

void robotSecondHand(uint16_t seconds){
  
  for(int i = 0; i< seconds; i++){
  int startTime = millis();
  rotateDegrees(6, "CW", 15, 15, 0, 1, 1);
  while(millis() < startTime +1000){}
  }
}
