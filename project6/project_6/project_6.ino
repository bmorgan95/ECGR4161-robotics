#include "SimpleRSLK.h"

const int trigPin = 32;           //connects to the trigger pin on the distance sensor       
const int echoPin = 33;           //connects to the echo pin on the distance sensor      
float distance = 0;               //stores the distance measured by the distance sensor

float shape1sides[3] = {91.44, 102.23, 102.23};
float shape1turns[3] = {116.5, 127, 116.5};

float shape2sides[4] = {91.44, 93.67, 50.8-2.5, 93.67};
float shape2turns[4] = {102.5, 77.5, 77.5, 102.5};

#define PULSES_1CM 16.37022272
#define ENCODER_DIFF 1

void setup() {
  // put your setup code here, to run once:
  Serial.begin (9600);
  setupRSLK();
  pinMode(trigPin, OUTPUT);   //the trigger pin will output pulses of electricity 
  pinMode(echoPin, INPUT);    //the echo pin will measure the duration of 
                              //pulses coming back from the distance sensor
  pinMode(LP_LEFT_BTN, INPUT_PULLUP);
  pinMode(LP_RIGHT_BTN, INPUT_PULLUP);
}

void loop() {
  // put your main code here, to run repeatedly: 

  int back[] = {0, 0};


if (digitalRead(LP_RIGHT_BTN) == LOW){    //shape 2b

  Serial.println("Right Button Detected");
  int shape = 1;
  
  for(int s=0; s<=2; s++){
  digitalWrite(BLUE_LED, HIGH);
  delay(500);
  digitalWrite(BLUE_LED, LOW);
  delay(500);
  }

  //drive shape 2b, stopping to wait for any obstacles to be removed

  for(int i = 0; i<=2; i++){

    driveStraight(50, 50, shape1sides[i], shape, back);
    allStop();
    delay(300);
    rotateDegrees(shape1turns[i], "CCW", 10);
    delay(300);
    
  }

}

if (digitalRead(LP_LEFT_BTN) == LOW){     //shape 2a


  int completedSides;

  Serial.println("Left Button Detected");
  int shape = 2;
  
  for(int s=0; s<=2; s++){
  digitalWrite(GREEN_LED, HIGH);
  delay(500);
  digitalWrite(GREEN_LED, LOW);
  delay(500);
  }

  //drive shape 2a, backtracking to beginning in the event of an obstacle

  for(int i = 0; i<=3; i++){

    completedSides = i;

    driveStraight(50, 50, shape2sides[i], shape, back);
    allStop();
    if (back[0] == 1){
      break;
    }
    delay(300);
    rotateDegrees(shape2turns[i], "CCW", 10);
    delay(300);
    
  }

  if (back[0] == 1){              //backtrack through the shape back to starting point
    Serial.println("Backtracking...");
    Serial.print("first leg is ");
    Serial.print(back[1]);
    Serial.println(" cm");
    rotateDegrees(180, "CCW", 10);
    driveStraight(50, 50, back[1], 3, back);
    delay(300);
    for (int j = completedSides; j >= 1; j--){
      Serial.print("Beginning turn ");
      Serial.print(j-1);
      Serial.println("...");
      Serial.println();
      Serial.println();
      rotateDegrees(shape2turns[j-1], "CW", 10);
      if (j == 1){
        rotateDegrees(8, "CW", 10);       //cheeky band-aid error compensation
      }
      delay(300);
      Serial.print("Beginning leg ");
      Serial.print(j-1);
      Serial.println("...");
      Serial.println();
      Serial.println();
      driveStraight(50, 50, shape2sides[j-1], 3, back);
      if (j == 1){
        driveStraight(50, 50, 5, 3, back);       //cheeky band-aid error compensation
      }
      allStop();
      delay(300);
     
    }
    rotateDegrees(180, "CCW", 10);
  }
  

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
  int numScans = 13;
  float values[numScans];
  for (int i = 0; i < numScans; i++){
    float scan = getDistance();
    values[i] = {scan};
  }

//  for(int k = 0; k < numScans; k++)
//{
  //Serial.println(k);
  //Serial.println(values[k]);
  //Serial.println(numScans);
//}

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

for(int k = 0; k < numScans; k++)
{
  //Serial.println(values[k]);
  //Serial.println(numScans);
}

        //return array median
        return (float)values[numScans/2];

}
  

//  float median = findMedian(values, numScans);
//  return median;
//}



////////////////////////////////////////////////
//function name: findMedian
//description: takes an array of data, sorts it using a bubble sort,
//             and returns the median.
//inputs: float array
//outputs: float
////////////////////////////////////////////////
//float findMedian(float values[], int numScans){
//
//
//
//    // bubble sort the array
//    for (int i = 0; i < numScans - 1; i++) {
//        for (int j = 0; j < numScans - i - 1; j++) {
//            if (values[j] > values[j+1]) {
//                // swap values[j] and values[j+1]
//                float temp = values[j];
//                values[j] = values[j+1];
//                values[j+1] = temp;
//            }
//        }
//    }
//
//for(int k = 0; k < numScans; k++)
//{
//  //Serial.println(values[k]);
//  //Serial.println(numScans);
//}
//
//        //return array median
//        return (float)values[numScans/2];
//
//}


//////////////////////////////////////////////
//function: getDistance
//description: uses the ultrasonic sensor to take a single distance reading
//inputs: none
//outputs: float
//FUNCTION BORROWED FROM EXAMPLE CODE
//////////////////////////////////////////////
float getDistance() {


  float echoTime;                   //variable to store the time it takes for a ping to bounce off an object
  float calculatedDistanceInches;         //variable to store the distance calculated from the echo time
  float calculatedDistanceCentimeters;         //variable to store the distance calculated from the echo time


  //send out an ultrasonic pulse that's 10ms long
  digitalWrite(trigPin, LOW); //ensures a clean pulse beforehand
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10); 
  digitalWrite(trigPin, LOW);

  echoTime = pulseIn(echoPin, HIGH);      //use the pulsein command to see how long it takes for the
                                          //pulse to bounce back to the sensor in microseconds

  calculatedDistanceInches = echoTime / 148.0;  //calculate the distance in inches of the object that reflected the pulse (half the bounce time multiplied by the speed of sound)
  calculatedDistanceCentimeters = echoTime / 58.0;  //calculate the distance in centimeters of the object that reflected the pulse (half the bounce time multiplied by the speed of sound)

  //Serial.println(calculatedDistanceInches);
  
  return calculatedDistanceCentimeters;              //send back the distance that was calculated
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
//Function Name: driveStraight
//Descriprion: Robot will drive straight by a given distance, with error correction, then stop
//Input: drivecm
//output: void
//ROBOT IS ASSUMED TO BE AT REST WHEN THIS FUNCTION IS CALLED
//////////////////////////////////////////////////////////
void driveStraight (int speedLeft, int speedRight, int distCM, int shape, int back[]) { 
 
  // Define speed and encoder count variables 

  int l_motor_speed = speedLeft; 
  int r_motor_speed = speedRight; 
  int l_totalCount = 0; 
  int r_totalCount = 0; 
  int old_count = 0;
  String steering = "center";
  //uint16_t num_pulses = 0; 
 
  // compute the number of pulses for drivecm centimeters 
  int  num_pulses = (uint16_t) (((float)distCM * PULSES_1CM) + 0.5); 
 
  resetLeftEncoderCnt();  
  resetRightEncoderCnt();   // Set encoder pulse count back to 0  
  setMotorDirection(BOTH_MOTORS,MOTOR_DIR_FORWARD); // Cause the robot to drive forward  
  enableMotor(BOTH_MOTORS);                         // "Turn on" the motor  
  setRawMotorSpeed(LEFT_MOTOR, l_motor_speed);         // Set motor speeds - variable,  
  setRawMotorSpeed(RIGHT_MOTOR, r_motor_speed);        //   may change (adjust) later 
 
  while( (l_totalCount< num_pulses ) || (r_totalCount< num_pulses )) {
    //digitalWrite(RED_LED, LOW);
    //digitalWrite(BLUE_LED, LOW);

    //run the loop as long as either wheel has travelled
    //fewer than the required number of pulses
    
    
    
    


    
    l_totalCount = getEncoderLeftCnt(); r_totalCount = getEncoderRightCnt(); 
    int avg_totalCount = (l_totalCount + r_totalCount) / 2;
    if (avg_totalCount > old_count + 0){
   
        // if right motor is too fast, speed up the left motor and slow the right 
        if((l_totalCount+ENCODER_DIFF) < r_totalCount) {
        setRawMotorSpeed(LEFT_MOTOR, l_motor_speed + 8);
        setRawMotorSpeed(RIGHT_MOTOR, r_motor_speed - 8);
        //digitalWrite(RED_LED, HIGH);
        steering = "right";
        }

        // if left motor is too fast, speed up the right motor and slow the left
        else if((r_totalCount+ENCODER_DIFF) < l_totalCount) {
        setRawMotorSpeed(RIGHT_MOTOR, r_motor_speed + 8);
        setRawMotorSpeed(LEFT_MOTOR, l_motor_speed - 8);
        //digitalWrite(BLUE_LED, HIGH);
        steering = "left";
        }

        else{
        setRawMotorSpeed(RIGHT_MOTOR, r_motor_speed);
        setRawMotorSpeed(LEFT_MOTOR, l_motor_speed);
        steering = "center";
        }
        
    old_count = avg_totalCount;
    }
 
    // Stop motors if they reach 1 meter 
    if (l_totalCount >= num_pulses) disableMotor(LEFT_MOTOR); 
    if (r_totalCount >= num_pulses ) disableMotor(RIGHT_MOTOR); 

    //Serial.print(l_totalCount);
    //Serial.print("            ");
    //Serial.println(r_totalCount);
    //Serial.println(old_count);

    if (shape == 1 and normalizedDist() <= 19.5){
      
      allStop();
      while(normalizedDist() <= 25){
          digitalWrite(RED_LED, HIGH);
          //delay(500);
      }
        digitalWrite(RED_LED, LOW);
        delay(500);
        enableMotor(BOTH_MOTORS);
      
    }

    if (shape == 2 and normalizedDist() <= 19.5){

        allStop();
        digitalWrite(RED_LED, HIGH);
        delay(5000);
        digitalWrite(RED_LED, LOW);
        delay(100);
        back[0] = 1;
        back[1] = avg_totalCount / PULSES_1CM;
        break;
      
    }

    
    
  } 
  //delay(200);
  //digitalWrite(RED_LED, LOW);
  //digitalWrite(BLUE_LED, LOW);

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
void rotateDegrees (int turnAngle, String direction, int speed){
  //Serial.println("turn function starts...");

  // Define speed and encoder count variables 
  int l_motor_speed = speed; 
  int r_motor_speed = speed; 
  int l_totalCount = 0; 
  int r_totalCount = 0;

  resetLeftEncoderCnt();  
  resetRightEncoderCnt();

  if(direction == "CW"){
  setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
  setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_BACKWARD);
  }

  if(direction == "CCW"){
  setMotorDirection(LEFT_MOTOR, MOTOR_DIR_BACKWARD);
  setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
  }

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
