  #include "SimpleRSLK.h"

  const int trigPin = 32;           //connects to the trigger pin on the distance sensor       
  const int echoPin = 33;           //connects to the echo pin on the distance sensor      
  float distance = 0;               //stores the distance measured by the distance sensor

  #define PULSES_1CM 16.37022272
  #define ENCODER_DIFF 1

void setup() {
  // put your setup code here, to run once:

  #include "SimpleRSLK.h"

  Serial.begin (9600);
  setupRSLK();
  pinMode(trigPin, OUTPUT);   //the trigger pin will output pulses of electricity 
  pinMode(echoPin, INPUT);    //the echo pin will measure the duration of 
                              //pulses coming back from the distance sensor
  pinMode(LP_LEFT_BTN, INPUT_PULLUP);
  pinMode(LP_RIGHT_BTN, INPUT_PULLUP);

  resetLeftEncoderCnt();  
  resetRightEncoderCnt();

}

void loop() {
  // put your main code here, to run repeatedly:

if (digitalRead(LP_LEFT_BTN) == LOW){

  //countdown
  for(int s=0; s<=2; s++){
  digitalWrite(BLUE_LED, HIGH);
  delay(500);
  digitalWrite(BLUE_LED, LOW);
  delay(500);
  }

  //off and running
  escapeArena();


}
}

/////////////////////////////////////////////
//function name: normalizedDist
//description: collects a defined number of distance measurements,
//             and returns the median
//inputs: none
//outputs: float
///////////////////////////////////////////////
float normalizedDist(){
  int numScans = 7;
  float values[numScans];
  for (int i = 0; i < numScans; i++){

  float echoTime;            //variable to store the time it takes for a ping to bounce off an object
  float calculatedDistanceInches;      //variable to store the distance calculated from the echo time
  float calculatedDistanceCentimeters; //variable to store the distance calculated from the echo time


  //send out an ultrasonic pulse that's 10ms long
  digitalWrite(trigPin, LOW); //ensures a clean pulse beforehand
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10); 
  digitalWrite(trigPin, LOW);

  echoTime = pulseIn(echoPin, HIGH);      //use the pulsein command to see how long it takes for the
                                          //pulse to bounce back to the sensor in microseconds
  //calculate the distance in inches of the object 
  //that reflected the pulse (half the bounce time multiplied by the speed of sound)
  calculatedDistanceInches = echoTime / 148.0;  
  //calculate the distance in centimeters of the object that 
  //reflected the pulse (half the bounce time multiplied by the speed of sound)
  calculatedDistanceCentimeters = echoTime / 58.0;  
  //Serial.println(calculatedDistanceInches);
  
  values[i] = calculatedDistanceCentimeters;
  
    delay(50);
  }

    // bubble sort the array
    for (int i = 0; i < numScans - 1; i++) {
        for (int j = 0; j < numScans - i - 1; j++) {
            if (values[j] > values[j+1]) {
                // swap values[j] and values[j+1]
                float temp = values[j];
                values[j] = values[j+1];
                values[j+1] = temp;
            }
        }
    }
 
        //return array median
        return (float)values[numScans/2];
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
void rotateDegrees (int turnAngle, String direction, int speedLeft, int speedRight, float overshoot){

  // Define speed and encoder count variables 
  int l_oldCount = getEncoderLeftCnt(); 
  int r_oldCount = getEncoderRightCnt();

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
  int turnPulses = (2.00714*(turnAngle*1.00625-overshoot)); 
  //compensate for measurement error and stoppage rollover

  setRawMotorSpeed(LEFT_MOTOR, speedLeft);         // Set motor speeds - variable,  
  setRawMotorSpeed(RIGHT_MOTOR, speedRight);        //   may change (adjust) later 
  enableMotor(BOTH_MOTORS);                         // "Turn on" the motor  


    // if right motor is too fast, speed up the left motor and slow the right 
    while((getEncoderLeftCnt() < turnPulses + l_oldCount) or (getEncoderRightCnt()< turnPulses + r_oldCount)) {
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

    // Stop motors if they reach requisite pulses 
    if (getEncoderLeftCnt() >= turnPulses + l_oldCount) disableMotor(LEFT_MOTOR); 
    if (getEncoderRightCnt() >= turnPulses + r_oldCount) disableMotor(RIGHT_MOTOR);

  }

  //just in case, stop both motors after loop exits
  disableMotor(LEFT_MOTOR); 
  disableMotor(RIGHT_MOTOR);

}


//////////////////////////////////////////////////////////
//Function Name: driveStraight
//Descriprion: Robot will drive straight by a given distance, with error correction, then stop
//Input: drivecm
//output: void
//ROBOT IS ASSUMED TO BE AT REST WHEN THIS FUNCTION IS CALLED
//////////////////////////////////////////////////////////
void driveStraight (int speedLeft, int speedRight, int distCM, float overshoot) { 
 
  // Define speed and encoder count variables 

  int l_oldCount = getEncoderLeftCnt(); 
  int r_oldCount = getEncoderRightCnt(); 
  int old_count = 0;
  String steering = "center";
  //uint16_t num_pulses = 0; 
 
  // compute the number of pulses for drivecm centimeters 
  int  num_pulses = (int) (((distCM - overshoot) * PULSES_1CM) + 0.5); 

  setMotorDirection(BOTH_MOTORS,MOTOR_DIR_FORWARD); // Cause the robot to drive forward  
  setRawMotorSpeed(LEFT_MOTOR, speedLeft);         // Set motor speeds - variable,  
  setRawMotorSpeed(RIGHT_MOTOR, speedRight);        //   may change (adjust) later 
  enableMotor(BOTH_MOTORS);                         // "Turn on" the motor  
 
  while((getEncoderLeftCnt() < num_pulses + l_oldCount) || (getEncoderRightCnt() < num_pulses + r_oldCount)) {
    digitalWrite(RED_LED, LOW);
    digitalWrite(BLUE_LED, LOW);

    //run the loop as long as either wheel has travelled
    //fewer than the required number of pulses

        // if right motor is too fast, speed up the left motor and slow the right 
        if((getEncoderLeftCnt()+ENCODER_DIFF) < getEncoderRightCnt()) {
        setRawMotorSpeed(LEFT_MOTOR, speedLeft + 2);
        setRawMotorSpeed(RIGHT_MOTOR, speedRight - 2);
        digitalWrite(RED_LED, HIGH);
        }

        // if left motor is too fast, speed up the right motor and slow the left
        else if((getEncoderRightCnt()+ENCODER_DIFF) < getEncoderLeftCnt()) {
        setRawMotorSpeed(RIGHT_MOTOR, speedRight + 2);
        setRawMotorSpeed(LEFT_MOTOR, speedLeft - 2);
        digitalWrite(BLUE_LED, HIGH);
        }

        // if encoders are equal to within allowed offset, set motors to default speed
        else{
        setRawMotorSpeed(RIGHT_MOTOR, speedRight);
        setRawMotorSpeed(LEFT_MOTOR, speedLeft);
        }

        // Stop motors if they reach requisite pulses 
        if ((getEncoderLeftCnt() - l_oldCount) >= num_pulses) disableMotor(LEFT_MOTOR); 
        if (getEncoderRightCnt() - r_oldCount >= num_pulses ) disableMotor(RIGHT_MOTOR);
  }

  disableMotor(LEFT_MOTOR); 
  disableMotor(RIGHT_MOTOR);

}


////////////////////////////////////////
//function: escapeArena
//inputs: none
//outputs: void
//description: rotates until robot can see past obstacles, then completes one 
//full circular sweep to ensure 2 consecutive edge detections.
// then rotates to face the average of the two angles, and drives straight out.
////////////////////////////////////////

void escapeArena(){;

  int angles[1];

  //das blinken lightsen
  digitalWrite(RED_LED, LOW);
  digitalWrite(BLUE_LED, LOW);
  digitalWrite(GREEN_LED, LOW);

  //if robot begins pointing at obstacle, rotate until robot can see past it.
  while (normalizedDist() <= 75){

      //das blinken lightsen indicator
      digitalWrite(RED_LED, HIGH);
      digitalWrite(BLUE_LED, LOW);
      digitalWrite(GREEN_LED, LOW);
  
    //scooch 3 degrees clockwise
    rotateDegrees(3, "CW", 15, 15, 0);
  }

  //reset encoder counts, establishing "origin" away from obstacles
  resetLeftEncoderCnt();  
  resetRightEncoderCnt();


  //rotate until robot detects leading edge of first obstacle
  while(normalizedDist() > 75){

      //das blinken lightsen indicator
      digitalWrite(RED_LED, LOW);
      digitalWrite(BLUE_LED, LOW);
      digitalWrite(GREEN_LED, HIGH);
    
    rotateDegrees(3, "CW", 15, 15, 0);
  }

  //save angular position of leading edge in pulses
  angles[0] = {getEncoderLeftCnt()};

  //rotate until robot detects trailing edge of second object
  while(normalizedDist() <= 75){

      //das blinken lightsen indicator
      digitalWrite(RED_LED, LOW);
      digitalWrite(BLUE_LED, HIGH);
      digitalWrite(GREEN_LED, LOW);
    
    
    rotateDegrees(3, "CW", 15, 15, 0);
  }

  //save angular position of second object in pulses
  angles[1] = {getEncoderLeftCnt()};

  //rotate through the rest of the complete circle if it isn't already achieved
  while(getEncoderLeftCnt() < 720){

      //das blinken lightsen indicator
      digitalWrite(RED_LED, LOW);
      digitalWrite(BLUE_LED, LOW);
      digitalWrite(GREEN_LED, HIGH);
    
    
    rotateDegrees(3, "CW", 15, 15, 0);
  }

      //das blinken lightsen indicator
      digitalWrite(RED_LED, LOW);
      digitalWrite(BLUE_LED, LOW);
      digitalWrite(GREEN_LED, LOW);
    
  allStop();

  delay(4000);

  //robot needs to rotate through the difference between the total distance travelled,
  //and the average of the two found angles
  int rotateTarget = (getEncoderLeftCnt() - ((angles[1] + angles[0]) / 2));

  //reset origin to current orientation
  resetLeftEncoderCnt();  
  resetRightEncoderCnt();

  //stepwise rotate back to escape route
  while (getEncoderLeftCnt() < rotateTarget){
  rotateDegrees(1, "CCW", 15, 15, 0);
  }

  //das blinken lightsen indicator
  digitalWrite(RED_LED, LOW);
  digitalWrite(BLUE_LED, LOW);
  digitalWrite(GREEN_LED, HIGH);

  delay(2000);

  //das blinken lightsen indicator
  digitalWrite(RED_LED, LOW);
  digitalWrite(BLUE_LED, LOW);
  digitalWrite(GREEN_LED, LOW);

  //bat outta hell
  driveStraight(31, 30, 125, 0);
  

  

}
