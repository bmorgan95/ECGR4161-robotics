//
//  Bradley Morgan and Jacob Santschi
//  Intro to Robotics
//  4-4-2023
//  LAB 09
//

#include <Servo.h>
#include "SimpleRSLK.h"

Servo myservo;  // create servo object to control a servo

uint16_t sensorVal[LS_NUM_SENSORS];
uint16_t sensorCalVal[LS_NUM_SENSORS];
uint16_t sensorMaxVal[LS_NUM_SENSORS];
uint16_t sensorMinVal[LS_NUM_SENSORS];

  const int trigPin = 32;           //connects to the trigger pin on the distance sensor       
  const int echoPin = 33;           //connects to the echo pin on the distance sensor      
  float distance = 0;               //stores the distance measured by the distance sensor

  long double encTarget = 0;

  #define PULSES_1CM 16.37022272
  #define ENCODER_DIFF 1  

void setup() {
  // put your setup code here, to run once:
    setupRSLK();
    myservo.attach(38, 600, 2650);   // attaches the servo on Port 2.4 (P2.4 or pin 38)to the servo object
    myservo.write(90);     // Send it to the center position
    Serial.begin(9600);
    pinMode(trigPin, OUTPUT);   //the trigger pin will output pulses of electricity 
    pinMode(echoPin, INPUT);    //the echo pin will measure the duration of 
                              //pulses coming back from the distance sensor
    pinMode(LP_LEFT_BTN, INPUT_PULLUP);
    pinMode(LP_RIGHT_BTN, INPUT_PULLUP);
    
    delay(2000);

    floorCalibration(); //prepare for line following

    delay(2000);

    resetLeftEncoderCnt();  
    resetRightEncoderCnt();

}

void loop() {
  // put your main code here, to run repeatedly: 

  if (digitalRead(LP_LEFT_BTN) == LOW){

    resetLeftEncoderCnt();  
    resetRightEncoderCnt();


    delay(3000);
    digitalWrite(GREEN_LED, HIGH);
    delay(1000);
    digitalWrite(GREEN_LED, LOW);
    myservo.write(90);

    float dist = 9999;
    int scan;
    int lowest;
    int lowest2;
    int servoPos = 0;

  for(int i=0;i<=180;i=i+5){  //sweep servo through 180 degrees in increments

        servoSweep(servoPos, i, 5);
        servoPos = i;
    
        scan = normalizedDist(9);     //save the boundary locations for the lowest values of the sweep
        if(scan < dist){
          lowest = i;
          dist = scan;
        }
        if (scan == dist){
          lowest2 = i;
        }
      
      }

      if(lowest>90){
        rotateDegrees(((lowest+lowest2)/2-90), "CCW", 20, 20, 0, 1);  //rotate to the average of the boundary values
      }

      if(lowest<=90){
        rotateDegrees((90-(lowest+lowest2)/2), "CW", 20, 20, 0, 1);
      }

      servoSweep(servoPos, 90, 5);    //center the servo
      servoPos = 90;
      delay(1000);


  float distances[4];                 //measure orthogonal distance to each wall
  for(int i=0; i<4; i++){
    distances[i] = {normalizedDist(9)};
    digitalWrite(BLUE_LED, HIGH);
    delay(100);
    digitalWrite(BLUE_LED, LOW);
    delay(100);
    if(i<3){rotateDegrees(90, "CW", 20, 20, 0, 1);}
  }

  float xError = abs(((distances[1] + distances[3])/2) - distances[3]);       //quick maths. calculate angle and distance of error
  float yError = abs(((distances[2] + distances[0])/2) - distances[0]);
  float linError = sqrt(pow(xError, 2) + pow(yError, 2));
  float angError = (90-abs(atan(yError / xError)*(180/3.141592)));
  rotateDegrees(90, "CCW", 20, 20, 0, 1);

  delay(1000);
  String turn;
  if(distances[3] < distances[1]){ turn = "CCW";}       //set direction of rotation
  if(distances[1] < distances[3]){ turn = "CW";}


    
  rotateDegrees(angError, turn, 20, 20, 0, 1);          //navigate to center
  delay(500);
  driveStraight(30, 30, linError, 0, 1);

  delay(500);

  while(1){
  lineFollow();                         //follow line home
  }
  
  
  

}

}

////////////////////////////////
//Function Name: floorCalibration
//description: robot will drive a short distance forward while scanning the floor without the line to
//establish a background reflectance level
//input: none
//output: void
/////////////////////////////////
void floorCalibration() {
  /* Place Robot On Floor (no line) */

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

///////////////////////////////////////////
//Function name: lineFollow
//description: orchestrates movements and line scanning to make the robot follow the line on the floor.
//input: none
//output: void
////////////////////////////////////////
void lineFollow(){

  uint8_t lineColor = DARK_LINE;

    readLineSensor(sensorVal);
  readCalLineSensor(sensorVal,
            sensorCalVal,
            sensorMinVal,
            sensorMaxVal,
            lineColor);

uint32_t linePos = getLinePosition(sensorCalVal,lineColor);
  //Serial.println(linePos);

  if((linePos == 0) or (linePos > 7000)){
    rotateDegrees(15, "CCW", 20, 20, 0, 0);
  }

  //when robot is far left of center, sharper left turn and scoot forward
  if ((linePos < 1501) and (linePos != 0)){
    //Serial.println("Big left turn");
    rotateDegrees(20,"CCW",20,20,0, 0);
    driveStraight(20,20,1,0,0);
  }

  //when robot is slightly left of center, slight left turn and scoot forward
  if ((linePos > 1500) and (linePos < 3000)){
    //Serial.println("Small feft turn");
    rotateDegrees(10,"CCW", 20, 20, 0,0);
    driveStraight(20,20,1,0,0);
  }

  //when robot is centered on the line, scoot forward
  if ((linePos > 2999) and (linePos < 4001)){
    //Serial.println("Forward");
    driveStraight(20,20,1,0,0);
  }

  //when robot is slightly right of center, slight right turn and scoot forward
  if ((linePos > 4000) and (linePos < 5500)){
    //Serial.println("Small right turn");
    rotateDegrees(10,"CW", 20, 20, 0,0);
    driveStraight(20,20,1,0,0);
  }

  //when robot is far right of center, sharper right turn and scoot forward
  if (linePos > 5499){
    //Serial.println("Big right turn");
    rotateDegrees(20,"CW", 20, 20, 0,0);
    driveStraight(20,20,1,0,0);
  }

  if(normalizedDist(3) <= 23){   // if detect obstacle, freeze forever
    while(1){}
    }

}

/////////////////////////////////////////////
//function name: normalizedDist
//description: collects a defined number of distance measurements,
//             and returns the median
//inputs: none
//outputs: float
///////////////////////////////////////////////
float normalizedDist(int numScans){
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
void rotateDegrees (int turnAngle, String direction, int speedLeft, int speedRight, float overshoot, int correction){

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


    // Stop motors if they reach requisite pulses 
    if (getEncoderLeftCnt() >= encTarget) disableMotor(LEFT_MOTOR); 
    if (getEncoderRightCnt() >= encTarget) disableMotor(RIGHT_MOTOR);

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
void driveStraight (int speedLeft, int speedRight, float distCM, float overshoot, int correction) { 
 
  // Define speed and encoder count variables 
 
  // compute the number of pulses for drivecm centimeters 
  encTarget =(int) (encTarget + (((distCM - overshoot) * PULSES_1CM) + 0.5)); 

  setMotorDirection(BOTH_MOTORS,MOTOR_DIR_FORWARD); // Cause the robot to drive forward  
  setRawMotorSpeed(LEFT_MOTOR, speedLeft);         // Set motor speeds - variable,  
  setRawMotorSpeed(RIGHT_MOTOR, speedRight);        //   may change (adjust) later 
  enableMotor(BOTH_MOTORS);                         // "Turn on" the motor  
 
  while((getEncoderLeftCnt() < encTarget) || (getEncoderRightCnt() < encTarget)) {

    if(correction == 1){
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

    }

        // Stop motors if they reach requisite pulses 
        if (getEncoderLeftCnt() >= encTarget) disableMotor(LEFT_MOTOR); 
        if (getEncoderRightCnt() >= encTarget) disableMotor(RIGHT_MOTOR);
  }

  disableMotor(LEFT_MOTOR); 
  disableMotor(RIGHT_MOTOR);

}

///////////////////////////////////////
//function: servoSweep
//description: Moves a servo from a specified start point to a specified end point.
//             User can specifiy degree incrememntation.
//input: int startDeg, int endDeg, int incDeg. 
//output: void
///////////////////////////////////////

void servoSweep(int startDeg, int endDeg, int incDeg) {

  if(startDeg < endDeg){
    for(int pos=startDeg; pos<=endDeg; pos+=incDeg) {
      myservo.write(pos);                           // tell servo to go to position in variable 'pos'
      //Serial.print("Servo Position: ");             // Print servo position to serial monitor
      //Serial.println(pos);
      delay(17);                                    // waits for the servo to reach the position
    }
  }
  
  if(startDeg > endDeg){
    for(int pos=startDeg; pos>=endDeg; pos-=incDeg) {
      myservo.write(pos);                           // tell servo to go to position in variable 'pos'
      //Serial.print("Servo Position: ");             // Print servo position to serial monitor
      //Serial.println(pos);
      delay(17);                                    // waits for the servo to reach the position
    }
  }
}
