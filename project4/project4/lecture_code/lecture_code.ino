//============================================================================ 
// File:  Drive_straight_encoder_adjust 
// 
// 2022-06-13 - James Conrad, from code borrowed from TI 
//    (some of this original code by Franklin S. Cooper Jr.) 
// Summary: 
//  This example will demonstrate the various features of the motor library. 
//  The robot will go forward for a specified amount of time.  
// 
//============================================================================ 
 
#include "SimpleRSLK.h" 
 
#define PULSES_1CM 16.37022272     // Number of pulses for 1 cm straight 
#define WHEELSPEED_L 30 
#define WHEELSPEED_R 30 
#define ENCODER_DIFF 1         // The difference between wheel encoders to cause 
                               // a motor speed change.  Initially 1 
 
//============================================================================ 
// The setup() funtion runs one time at the beginning of the Energia program  
//============================================================================ 
void setup() { 
  setupRSLK();  // Set up all of the pins & functions needed to be used by the TI bot 

    /* Left button on Launchpad */
  setupWaitBtn(LP_LEFT_BTN);
  /* Red led in rgb led */
  setupLed(RED_LED);
} 
 
//============================================================================ 
// The loop() function runs after the setup() function completes in an  
// Energia program and will continue to run in a repeating loop until the  
// LaunchPad is reset or powered off  
//============================================================================ 
void loop() { 

  /* message for the serial output */
  String btnMsg = "press left button to let 'er rip";
  /* Wait until button is pressed to start robot */
  waitBtnPressed(LP_LEFT_BTN,btnMsg,RED_LED);

  delay(1000);  //wait one second
 
     driveStraight(100);    //kick the tires and light the fires
} //end of loop function 
 
  
// File:  Drive_straight_encoder_adjust 
//=============================================================================== 
// The driveStraight function
// Drive the TI RSLK MAX Straight for drivecm centimeters, reading the encoders  
// and adjusting the motor speed if the one side traveled further (faster) than  
// the other side      2022-06-13 James Conrad 
//=============================================================================== 
 
void driveStraight (uint16_t drivecm) { 
 
  // Define speed and encoder count variables 
  uint16_t l_motor_speed = WHEELSPEED_L; 
  uint16_t r_motor_speed = WHEELSPEED_R; 
  uint16_t l_totalCount = 0; 
  uint16_t r_totalCount = 0; 
  uint16_t num_pulses = 0; 
 
  // compute the number of pulses for drivecm centimeters 
  num_pulses = (uint16_t) (((float)drivecm * PULSES_1CM) + 0.5); 
 
  resetLeftEncoderCnt();  
  resetRightEncoderCnt();   // Set encoder pulse count back to 0  
  setMotorDirection(BOTH_MOTORS,MOTOR_DIR_FORWARD); // Cause the robot to drive forward  
  enableMotor(BOTH_MOTORS);                         // "Turn on" the motor  
  setMotorSpeed(LEFT_MOTOR, l_motor_speed);         // Set motor speeds - variable,  
  setMotorSpeed(RIGHT_MOTOR, r_motor_speed);        //   may change (adjust) later 
 
  while( (l_totalCount< num_pulses ) || (r_totalCount< num_pulses )) {

    //run the loop as long as either wheel has travelled
    //fewer than the required number of pulses
    
    l_totalCount = getEncoderLeftCnt(); r_totalCount = getEncoderRightCnt(); 
 
    // if right motor is too fast, speed up the left motor and slow the right 
    if((l_totalCount+ENCODER_DIFF) < r_totalCount) {
    setMotorSpeed(LEFT_MOTOR, ++l_motor_speed);
    setMotorSpeed(RIGHT_MOTOR, --l_motor_speed);}

    // if left motor is too fast, speed up the right motor and slow the left
    if((r_totalCount+ENCODER_DIFF) < l_totalCount) {
    setMotorSpeed(RIGHT_MOTOR, ++r_motor_speed);
    setMotorSpeed(LEFT_MOTOR, --r_motor_speed);}
 
    // Stop motors if they reach 1 meter 
    if (l_totalCount >= num_pulses) disableMotor(LEFT_MOTOR); 
    if (r_totalCount >= num_pulses ) disableMotor(RIGHT_MOTOR); 
  } 
  delay(200);
}
