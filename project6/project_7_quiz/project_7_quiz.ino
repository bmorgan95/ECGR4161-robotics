#include "SimpleRSLK.h"

const int trigPin = 32;           //connects to the trigger pin on the distance sensor       
const int echoPin = 33;           //connects to the echo pin on the distance sensor      
float distance = 0;               //stores the distance measured by the distance sensor

void setup() {
  // put your setup code here, to run once:
  Serial.begin (9600);
  setupRSLK();
  pinMode(trigPin, OUTPUT);   //the trigger pin will output pulses of electricity 
  pinMode(echoPin, INPUT);    //the echo pin will measure the duration of pulses coming back from the distance sensor
}

void loop() {
  // put your main code here, to run repeatedly: 

  float normalized = normalizedDist();

  Serial.print("Normalized Distance = ");
  Serial.print(normalized);
  Serial.println(" inches.");
  delay(2000);

}

float normalizedDist(){
  int numScans = 9;
  float values[numScans];
  for (int i = 0; i < numScans; i++){
    float scan = getDistance();
    while (scan = 0){
      float scan = getDistance();
    }
    values[i] = {scan};
  }

  for(int k = 0; k < numScans; k++)
{
  //Serial.println(values[k]);
  //Serial.println(numScans);
}
  

  float median = findMedian(values, numScans);
  return median;
}

float findMedian(float values[], int numScans){



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
  Serial.println(values[k]);
  //Serial.println(numScans);
}

        //return array median
        return (float)values[numScans/2];

}

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
  
  return calculatedDistanceInches;              //send back the distance that was calculated
}
