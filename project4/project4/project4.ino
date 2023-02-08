//
//  Bradley Morgan and Jacob Santschi
//  Intro to Robotics
//  2-4-2023
//  LAB 04
//
#include <SimpleRSLK.h>

#define PULSES_1CM 16.37022272     // Number of pulses for 1 cm straight 
#define WHEELSPEED_L 10 
#define WHEELSPEED_R 10 
#define ENCODER_DIFF 1         // The difference between wheel encoders to cause 
                               // a motor speed change.  Initially 1 
							   

void setup() {
  // put your setup code here, to run once:
    setupRSLK();  // Set up all of the pins & functions needed to be used by the TI bot 

    /* Left button on Launchpad */
  setupWaitBtn(LP_LEFT_BTN);
  /* Red led in rgb led */
  setupLed(RED_LED);

}

void loop() {
  // put your main code here, to run repeatedly: 

  
}

////////////////////////////////////////////////////////
//Function name: driveUntil
//description: drive straight forward with error correction
//until frontBumpSensor returns true,
//then return number of motor pulses that have been counted
//input: none
//output: int pulsesDriven
//ROBOT IS ASSUMED TO BE AT REST WHEN THIS FUNCTION IS CALLED
////////////////////////////////////////////////////////////
int driveUntil(){

}

//////////////////////////////////////////////////////////
//Function Name: rotateCWDegrees
//Description: rotate clockwise on the spot by a given number of degrees,
//with error correction to maintain positional accuracy
//input: turnAngle
//output: void
//ROBOT IS ASSUMED TO BE AT REST WHEN THIS FUNCTION IS CALLED
/////////////////////////////////////////////////////////////
void rotateCWDegrees (turnAngle){
	setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
	setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_BACKWARD);
	
}


//////////////////////////////////////////////////////////
//Function Name: driveStraight
//Descriprion: Robot will drive straight by a given distance, with error correction, then stop
//Input: drivecm
//output: void
//ROBOT IS ASSUMED TO BE AT REST WHEN THIS FUNCTION IS CALLED
//////////////////////////////////////////////////////////
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

//Function to read bump sensors
/////////////////////////////////////////////////////////
//Function Name: frontBumpSensor
//Description: 
//	This function reads when either of the Bump Sensors (2 or 3)
//	on the front of the RSLK are pressed and returns a Boolean value
//Input: no input
//Return: bool
//ROBOT IS ASSUMED TO BE IN MOTION WHEN THIS FUNCTION IS CALLED
/////////////////////////////////////////////////////////
bool frontBumpSensor(){
	if(isBumpSwitchPressed(2) || isBumpSwitchPressed(3) == true) {
		return true;
	} else {
		return false;
	}
}

/////////////////////////////////////////////////////////
//Function Name: fullStop
//Description: Bring robot to a stop by applying braking force to both wheels
//Input: none
//output: void
//ROBOT IS ASSUMED TO BE IN MOTION WHEN THIS FUNCTION IS CALLED
/////////////////////////////////////////////////////////
void allStop(){
	enableMotor(BOTH_MOTORS);
	//set speed to zero rather than disabling in order to actively brake
	setMotorSpeed(BOTH_MOTORS, 0);
	delay(200);
}













// void loop() {
// 	bool hitObstacle = false;

// 	String btnMsg = "Push left button on Launchpad to start demo.\n";
// 	/* Wait until button is pressed to start robot */
// 	waitBtnPressed(LP_LEFT_BTN,btnMsg,RED_LED);

// 	/* Wait two seconds before starting */
// 	delay(2000);

// 	/* Enable both motors, set their direction and provide a default speed */
// 	enableMotor(BOTH_MOTORS);
// 	setMotorDirection(BOTH_MOTORS,MOTOR_DIR_FORWARD);
// 	setMotorSpeed(BOTH_MOTORS,motorSpeed);

// 	/* Keep checking if the robot has hit an object */
// 	while(!hitObstacle) {
// 		/* Loop through all bump switchees to see if it has been pressed */
// 		for(int x = 0;x<TOTAL_BP_SW;x++)
// 		{
// 			/* Check if bump switch was pressed */
// 			if(isBumpSwitchPressed(x) == true) {
// 				hitObstacle = true;
// 				break;
// 			}
// 		}
// 	}

// 	Serial.println("Collision detected");
// 	disableMotor(BOTH_MOTORS);
// }

