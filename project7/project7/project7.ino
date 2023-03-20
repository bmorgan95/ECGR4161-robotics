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




}

void loop() {
  // put your main code here, to run repeatedly:
  float scans[60];
  float dist;

  while (digitalRead(LP_LEFT_BTN) == HIGH){

    dist = getDistance();
    Serial.println(dist);
        if(dist < 75){
          digitalWrite(RED_LED, HIGH);
          digitalWrite(GREEN_LED, LOW);
          digitalWrite(BLUE_LED, LOW);
          
        }
        else if(dist>=75 && dist<110){
          digitalWrite(RED_LED, LOW);
          digitalWrite(GREEN_LED, HIGH);
          digitalWrite(BLUE_LED, LOW);
          
        }
        else{
          digitalWrite(RED_LED, LOW);
          digitalWrite(GREEN_LED, LOW);
          digitalWrite(BLUE_LED, HIGH);
          
        }

     delay(100);   
  }

  delay(5000);

    while (digitalRead(LP_LEFT_BTN) == HIGH){

    dist = normalizedDist();
    Serial.println(dist);
        if(dist < 75){
          digitalWrite(RED_LED, HIGH);
          digitalWrite(GREEN_LED, LOW);
          digitalWrite(BLUE_LED, LOW);
          
        }
        else if(dist>=75 && dist<110){
          digitalWrite(RED_LED, LOW);
          digitalWrite(GREEN_LED, HIGH);
          digitalWrite(BLUE_LED, LOW);
          
        }
        else{
          digitalWrite(RED_LED, LOW);
          digitalWrite(GREEN_LED, LOW);
          digitalWrite(BLUE_LED, HIGH);
          
        }

     delay(100);   
  }

  for(int s=0; s<=2; s++){
  digitalWrite(BLUE_LED, HIGH);
  delay(500);
  digitalWrite(BLUE_LED, LOW);
  delay(500);
  }

  //float dist[];
    for (int i=0; i<=5; i++){
      for(int j=0; j<=19; j++){
        dist = getDistance();
        if(dist < 75){
          digitalWrite(RED_LED, HIGH);
          digitalWrite(GREEN_LED, LOW);
          digitalWrite(BLUE_LED, LOW);
          
        }
        else if(dist>=75 && dist<110){
          digitalWrite(RED_LED, LOW);
          digitalWrite(GREEN_LED, HIGH);
          digitalWrite(BLUE_LED, LOW);
          
        }
        else{
          digitalWrite(RED_LED, LOW);
          digitalWrite(GREEN_LED, LOW);
          digitalWrite(BLUE_LED, HIGH);
          
        }
        scans[(i*10 + j)] = {dist};
        rotateDegrees(3, "CW", 15, 15, 1.5);
        delay(50);
        }
      
    }

          digitalWrite(RED_LED, LOW);
          digitalWrite(GREEN_LED, LOW);
          digitalWrite(BLUE_LED, LOW);
          

      while(1){
        for (int i=0; i<=5; i++){
        for(int j=0; j<=19; j++){
        Serial.print(scans[(i*10 + j)]);
        Serial.print("    ");
        }
        Serial.println();
        }
        Serial.println();
        Serial.println();
        Serial.println();
        Serial.println();
        Serial.println();
        Serial.println();
        Serial.println();
        
        delay(10000);
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
  int numScans = 15;
  float values[numScans];
  for (int i = 0; i < numScans; i++){
    values[i] = {getDistance()};
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
//Function Name: rotateDegrees
//Description: rotate on the spot by a given number of degrees,
//in the given direction,
//with error correction to maintain positional accuracy
//input: turnAngle, direction
//output: void
//ROBOT IS ASSUMED TO BE AT REST WHEN THIS FUNCTION IS CALLED
/////////////////////////////////////////////////////////////
void rotateDegrees (int turnAngle, String direction, int speedLeft, int speedRight, float overshoot){
  Serial.println("turn function starts...");

  // Define speed and encoder count variables 
  int l_totalCount = 0; 
  int r_totalCount = 0;
  String steering = "center";

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
  int turnPulses = (2.00714*(turnAngle*1.00625-overshoot)); 
  Serial.println(turnPulses);
  //compensate for measurement error and stoppage rollover

  setRawMotorSpeed(LEFT_MOTOR, speedLeft);         // Set motor speeds - variable,  
  setRawMotorSpeed(RIGHT_MOTOR, speedRight);        //   may change (adjust) later 
  enableMotor(BOTH_MOTORS);                         // "Turn on" the motor  


  while((l_totalCount< turnPulses ) or (r_totalCount< turnPulses )) {

    Serial.println("steering loop");

    l_totalCount = getEncoderLeftCnt();
    r_totalCount = getEncoderRightCnt();
    //digitalWrite(RED_LED, LOW);
    //digitalWrite(BLUE_LED, LOW);
 
    // if right motor is too fast, speed up the left motor and slow the right 
    if((l_totalCount+ENCODER_DIFF) < r_totalCount) {
    setRawMotorSpeed(LEFT_MOTOR, speedLeft + 2);
    setRawMotorSpeed(RIGHT_MOTOR, speedRight - 2);
    //digitalWrite(RED_LED, HIGH);
    steering = "right";
    }

    // if left motor is too fast, speed up the right motor and slow the left
    else if((r_totalCount+ENCODER_DIFF) < l_totalCount) {
    setRawMotorSpeed(RIGHT_MOTOR, speedRight + 2);
    setRawMotorSpeed(LEFT_MOTOR, speedLeft - 2);
    //digitalWrite(BLUE_LED, HIGH);
    steering = "left";
    }

    // if encoders are equal to within allowed offset, set motors to default speed
    else{
    setRawMotorSpeed(RIGHT_MOTOR, speedRight);
    setRawMotorSpeed(LEFT_MOTOR, speedLeft);
    steering = "center";
    }


    // Stop motors if they reach requisite pulses 
    if (l_totalCount >= turnPulses) disableMotor(LEFT_MOTOR); 
    if (r_totalCount >= turnPulses ) disableMotor(RIGHT_MOTOR);

  }

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

  int l_totalCount = 0; 
  int r_totalCount = 0; 
  int old_count = 0;
  String steering = "center";
  //uint16_t num_pulses = 0; 
 
  // compute the number of pulses for drivecm centimeters 
  int  num_pulses = (uint16_t) (((float)(distCM - overshoot) * PULSES_1CM) + 0.5); 
 
  resetLeftEncoderCnt();  
  resetRightEncoderCnt();   // Set encoder pulse count back to 0  
  setMotorDirection(BOTH_MOTORS,MOTOR_DIR_FORWARD); // Cause the robot to drive forward  
  setRawMotorSpeed(LEFT_MOTOR, speedLeft);         // Set motor speeds - variable,  
  setRawMotorSpeed(RIGHT_MOTOR, speedRight);        //   may change (adjust) later 
  enableMotor(BOTH_MOTORS);                         // "Turn on" the motor  
 
  while( (l_totalCount< num_pulses ) || (r_totalCount< num_pulses )) {
    digitalWrite(RED_LED, LOW);
    digitalWrite(BLUE_LED, LOW);

    //run the loop as long as either wheel has travelled
    //fewer than the required number of pulses

    l_totalCount = getEncoderLeftCnt(); r_totalCount = getEncoderRightCnt(); 
    int avg_totalCount = (l_totalCount + r_totalCount) / 2;
    
   
        // if right motor is too fast, speed up the left motor and slow the right 
        if((l_totalCount+ENCODER_DIFF) < r_totalCount) {
        setRawMotorSpeed(LEFT_MOTOR, speedLeft + 2);
        setRawMotorSpeed(RIGHT_MOTOR, speedRight - 2);
        digitalWrite(RED_LED, HIGH);
        steering = "right";
        }

        // if left motor is too fast, speed up the right motor and slow the left
        else if((r_totalCount+ENCODER_DIFF) < l_totalCount) {
        setRawMotorSpeed(RIGHT_MOTOR, speedRight + 2);
        setRawMotorSpeed(LEFT_MOTOR, speedLeft - 2);
        digitalWrite(BLUE_LED, HIGH);
        steering = "left";
        }

        // if encoders are equal to within allowed offset, set motors to default speed
        else{
        setRawMotorSpeed(RIGHT_MOTOR, speedRight);
        setRawMotorSpeed(LEFT_MOTOR, speedLeft);
        steering = "center";
        }

        // Stop motors if they reach requisite pulses 
        if (l_totalCount >= num_pulses) disableMotor(LEFT_MOTOR); 
        if (r_totalCount >= num_pulses ) disableMotor(RIGHT_MOTOR);
  }

}
