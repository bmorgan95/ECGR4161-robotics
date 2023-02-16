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
#define ENCODER_DIFF 1         // The difference between wheel encoders until corrections are made 
#define pi 3.14159265
//int turnAngle = 360;
//#define trackWidth 14.05  //vehicle width between wheel centers in cm
//#define wheelDia 7        //wheel diameter in cm
							   

void setup() {
  // put your setup code here, to run once:
    setupRSLK();  // Set up all of the pins & functions needed to be used by the TI bot 
    Serial.begin(9600);
    

    /* Left button on Launchpad */
  setupWaitBtn(LP_RIGHT_BTN);
  /* Red led in rgb led */
  setupLed(RED_LED);

}

void loop() {
  // put your main code here, to run repeatedly: 

    /* message for the serial output */
  String btnMsg = "press left button to let 'er rip"; 
  /* Wait until button is pressed to start robot */
  waitBtnPressed(LP_RIGHT_BTN,btnMsg,RED_LED);
  delay(2000);

  int pulses = driveUntil();
  delay(3000);
  rotateCWDegrees(180);
  delay(1000);
  driveStraight(pulses+8);
  delay(1000);
  rotateCWDegrees(180);

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
  // Define speed and encoder count variables 
  uint16_t l_motor_speed = WHEELSPEED_L; 
  uint16_t r_motor_speed = WHEELSPEED_R; 
  uint16_t l_totalCount = 0; 
  uint16_t r_totalCount = 0;  

  resetLeftEncoderCnt();  
  resetRightEncoderCnt();   // Set encoder pulse count back to 0  
  setMotorDirection(BOTH_MOTORS,MOTOR_DIR_FORWARD); // Cause the robot to drive forward  
  enableMotor(BOTH_MOTORS);                         // "Turn on" the motor  
  setMotorSpeed(LEFT_MOTOR, l_motor_speed);         // Set motor speeds - variable,  
  setMotorSpeed(RIGHT_MOTOR, r_motor_speed);        //   may change (adjust) later 
 
  while(frontBumpSensor() == false) {

    //run the loop as long as no collision is detected
    
    l_totalCount = getEncoderLeftCnt(); r_totalCount = getEncoderRightCnt(); 
 
    // if right motor is too fast, speed up the left motor and slow the right 
    if((l_totalCount+ENCODER_DIFF) < r_totalCount) {
    setMotorSpeed(LEFT_MOTOR, ++l_motor_speed);
    setMotorSpeed(RIGHT_MOTOR, --l_motor_speed);}

    // if left motor is too fast, speed up the right motor and slow the left
    if((r_totalCount+ENCODER_DIFF) < l_totalCount) {
    setMotorSpeed(RIGHT_MOTOR, ++r_motor_speed);
    setMotorSpeed(LEFT_MOTOR, --r_motor_speed);}
 
    //////// Stop motors if they reach 1 meter ///////////
    //if (l_totalCount >= num_pulses) disableMotor(LEFT_MOTOR); 
    //if (r_totalCount >= num_pulses ) disableMotor(RIGHT_MOTOR); 
  }
  allStop();
   
  delay(200);

  return (l_totalCount + r_totalCount)/2;
}


//////////////////////////////////////////////////////////
//Function Name: rotateCWDegrees
//Description: rotate clockwise on the spot by a given number of degrees,
//with error correction to maintain positional accuracy
//input: turnAngle
//output: void
//ROBOT IS ASSUMED TO BE AT REST WHEN THIS FUNCTION IS CALLED
/////////////////////////////////////////////////////////////
void rotateCWDegrees (int turnAngle){
  Serial.println("turn function starts...");

  // Define speed and encoder count variables 
  int l_motor_speed = WHEELSPEED_L; 
  int r_motor_speed = WHEELSPEED_R; 
  int l_totalCount = 0; 
  int r_totalCount = 0;

  resetLeftEncoderCnt();  
  resetRightEncoderCnt();

	setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
	setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_BACKWARD);

  //14.05*pi cm per 1 degree of robot turn, 
  int turnPulses = (2.00714*(turnAngle*1.00625-3)); 
  //compensate for measurement error and stoppage rollover
  
  enableMotor(BOTH_MOTORS);                         // "Turn on" the motor  
  setMotorSpeed(LEFT_MOTOR, l_motor_speed);         // Set motor speeds - variable,  
  setMotorSpeed(RIGHT_MOTOR, r_motor_speed);        //   may change (adjust) later 


  while((l_totalCount< turnPulses ) or (r_totalCount< turnPulses )) {

    l_totalCount = getEncoderLeftCnt();
    r_totalCount = getEncoderRightCnt();
 
    // if right motor is too fast, speed up the left motor and slow the right 
    if((l_totalCount+ENCODER_DIFF) < r_totalCount) {
    setMotorSpeed(LEFT_MOTOR, ++l_motor_speed);
    setMotorSpeed(RIGHT_MOTOR, --l_motor_speed);}

    // if left motor is too fast, speed up the right motor and slow the left
    if((r_totalCount+ENCODER_DIFF) < l_totalCount) {
    setMotorSpeed(RIGHT_MOTOR, ++r_motor_speed);
    setMotorSpeed(LEFT_MOTOR, --r_motor_speed);}

    // Stop motors if they reach requisite pulses 
    if (l_totalCount >= turnPulses) disableMotor(LEFT_MOTOR); 
    if (r_totalCount >= turnPulses ) disableMotor(RIGHT_MOTOR);

  }

   //allStop();
   
}


//////////////////////////////////////////////////////////
//Function Name: driveStraight
//Descriprion: Robot will drive straight by a given distance, with error correction, then stop
//Input: drivecm
//output: void
//ROBOT IS ASSUMED TO BE AT REST WHEN THIS FUNCTION IS CALLED
//////////////////////////////////////////////////////////
void driveStraight (uint16_t num_pulses) { 
 
  // Define speed and encoder count variables 
  uint16_t l_motor_speed = WHEELSPEED_L; 
  uint16_t r_motor_speed = WHEELSPEED_R; 
  uint16_t l_totalCount = 0; 
  uint16_t r_totalCount = 0; 
  //uint16_t num_pulses = 0; 
 
  // compute the number of pulses for drivecm centimeters 
  //num_pulses = (uint16_t) (((float)drivecm * PULSES_1CM) + 0.5); 
 
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
  //delay(200);
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
	if(isBumpSwitchPressed(2) or isBumpSwitchPressed(3) == true) {
		return true;
	} else {
		return false;
	}
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
	delay(200);
  disableMotor(BOTH_MOTORS);
}
