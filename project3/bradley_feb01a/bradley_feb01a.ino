#include <Bump_Switch.h>
#include <Encoder.h>
#include <GP2Y0A21_Sensor.h>
#include <QTRSensors.h>
#include <Romi_Motor_Power.h>
#include <RSLK_Pins.h>
#include <SimpleRSLK.h>

void setup() {
  // put your setup code here, to run once:

setupRSLK(); //setup code recommended by the library





}

void loop() {
  // put your main code here, to run repeatedly: 

bumper1();
bumper2();
bumper3();
bumper12();
bumper13();
bumper23();
bumper123();
bumper4();
bumper2();

  
}


void bumper1() {                                     //blink red 1Hz (cycle time 1000ms)
  if (isBumpSwitchPressed(0) == true){

    while (isBumpSwitchPressed(0) == false and      //loop as long as no bumpers are pressed
           isBumpSwitchPressed(1) == false and
           isBumpSwitchPressed(2) == false and
           isBumpSwitchPressed(3) == false and
           isBumpSwitchPressed(4) == false and
           isBumpSwitchPressed(5) == false){

            digitalWrite(RED_LED, HIGH);             //red on
            digitalWrite(BLUE_LED, LOW);
            digitalWrite(GREEN_LED, LOW);

            delay(500);

            digitalWrite(RED_LED, LOW);             //all off
            digitalWrite(BLUE_LED, LOW);
            digitalWrite(GREEN_LED, LOW);

            delay(500);
           }

  }
}


void bumper2(){                                     //blink green 2Hz (cycle time 500ms)
  if (isBumpSwitchPressed(1) == true){

    while (isBumpSwitchPressed(0) == false and      //loop as long as no bumpers are pressed
           isBumpSwitchPressed(1) == false and
           isBumpSwitchPressed(2) == false and
           isBumpSwitchPressed(3) == false and
           isBumpSwitchPressed(4) == false and
           isBumpSwitchPressed(5) == false){

            digitalWrite(RED_LED, LOW);             //green on
            digitalWrite(BLUE_LED, LOW);
            digitalWrite(GREEN_LED, HIGH);

            delay(250);

            digitalWrite(RED_LED, LOW);             //all off
            digitalWrite(BLUE_LED, LOW);
            digitalWrite(GREEN_LED, LOW);

            delay(250);
           }

  }
}


void bumper3(){                                     //blink blue 4Hz (cycle time 250ms)
  if (isBumpSwitchPressed(2) == true){

    while (isBumpSwitchPressed(0) == false and      //loop as long as no bumpers are pressed
           isBumpSwitchPressed(1) == false and
           isBumpSwitchPressed(2) == false and
           isBumpSwitchPressed(3) == false and
           isBumpSwitchPressed(4) == false and
           isBumpSwitchPressed(5) == false){

            digitalWrite(RED_LED, LOW);             //blue on
            digitalWrite(BLUE_LED, HIGH);
            digitalWrite(GREEN_LED, LOW);

            delay(125);

            digitalWrite(RED_LED, LOW);             //all off
            digitalWrite(BLUE_LED, LOW);
            digitalWrite(GREEN_LED, LOW);

            delay(125);
           }

  }

}

void bumper12(){

  if (isBumpSwitchPressed(0) == true and          //red green blink same time, 16Hz (125ms cycle time)
      isBumpSwitchPressed(1) == true){

        while (isBumpSwitchPressed(0) == false and      //loop as long as no bumpers are pressed
           isBumpSwitchPressed(1) == false and
           isBumpSwitchPressed(2) == false and
           isBumpSwitchPressed(3) == false and
           isBumpSwitchPressed(4) == false and
           isBumpSwitchPressed(5) == false){

            digitalWrite(RED_LED, HIGH);             //red and green on
            digitalWrite(BLUE_LED, LOW);
            digitalWrite(GREEN_LED, HIGH);

            delay(62);

            digitalWrite(RED_LED, LOW);             //all off
            digitalWrite(BLUE_LED, LOW);
            digitalWrite(GREEN_LED, LOW);

            delay(63);

           }

      }

}



void bumper13(){

  if (isBumpSwitchPressed(1) == true and          //red blue blink same time, 16Hz (62.5ms cycle time, 62ms for ease, effective 16.12903Hz because integers :P)
      isBumpSwitchPressed(2) == true){


        while (isBumpSwitchPressed(0) == false and      //loop as long as no bumpers are pressed
           isBumpSwitchPressed(1) == false and
           isBumpSwitchPressed(2) == false and
           isBumpSwitchPressed(3) == false and
           isBumpSwitchPressed(4) == false and
           isBumpSwitchPressed(5) == false){

            digitalWrite(RED_LED, HIGH);             //red and blue on
            digitalWrite(BLUE_LED, HIGH);
            digitalWrite(GREEN_LED, LOW);

            delay(31);

            digitalWrite(RED_LED, LOW);             //all off
            digitalWrite(BLUE_LED, LOW);
            digitalWrite(GREEN_LED, LOW);

            delay(31);


      
      }

   }

}

void bumper23(){
  if (isBumpSwitchPressed(1) == true and          //green blue blink same time, 32Hz (31.25ms cycle time, 32ms for ease, effective 31.25Hz because integers :P)
      isBumpSwitchPressed(2) == true){
        while (isBumpSwitchPressed(0) == false and      //loop as long as no bumpers are pressed
           isBumpSwitchPressed(1) == false and
           isBumpSwitchPressed(2) == false and
           isBumpSwitchPressed(3) == false and
           isBumpSwitchPressed(4) == false and
           isBumpSwitchPressed(5) == false){

            digitalWrite(RED_LED, LOW);             //green and blue on
            digitalWrite(BLUE_LED, HIGH);
            digitalWrite(GREEN_LED, HIGH);

            delay(16);

            digitalWrite(RED_LED, LOW);             //all off
            digitalWrite(BLUE_LED, LOW);
            digitalWrite(GREEN_LED, LOW);

            delay(16);

      }
}

}


void bumper123(){
  if (isBumpSwitchPressed(0) == true and          //1 2 3 same time
      isBumpSwitchPressed(1) == true and
      isBumpSwitchPressed(2) == true){
      
    digitalWrite(RED_LED, HIGH);                  //all leds on, leave on
    digitalWrite(BLUE_LED, HIGH);
    digitalWrite(GREEN_LED, HIGH);
  }
}


void bumper4(){  //rgb rainbow vomit when bumper 4 is pressed
  if (isBumpSwitchPressed(3) == true){
    while (isBumpSwitchPressed(0) == false and      //loop as long as no bumpers are pressed
           isBumpSwitchPressed(1) == false and
           isBumpSwitchPressed(2) == false and
           isBumpSwitchPressed(3) == false and
           isBumpSwitchPressed(4) == false and
           isBumpSwitchPressed(5) == false){

            digitalWrite(RED_LED, LOW);             //all off
            digitalWrite(BLUE_LED, LOW);
            digitalWrite(GREEN_LED, LOW);

            delay(630);

            digitalWrite(RED_LED, HIGH);            //all on
            digitalWrite(BLUE_LED, HIGH);
            digitalWrite(GREEN_LED, HIGH);

            delay(630);

            digitalWrite(RED_LED, LOW);            //all off
            digitalWrite(BLUE_LED, LOW);
            digitalWrite(GREEN_LED, LOW);

            delay(630);

            digitalWrite(RED_LED, LOW);           //only green
            digitalWrite(BLUE_LED, LOW);
            digitalWrite(GREEN_LED, HIGH);

            delay(630);

            digitalWrite(RED_LED, LOW);             //only blue
            digitalWrite(BLUE_LED, HIGH);
            digitalWrite(GREEN_LED, LOW);

            delay(630);

            digitalWrite(RED_LED, HIGH);            //only red
            digitalWrite(BLUE_LED, LOW);
            digitalWrite(GREEN_LED, LOW);

            delay(630);

            digitalWrite(RED_LED, LOW);             //all off
            digitalWrite(BLUE_LED, LOW);
            digitalWrite(GREEN_LED, LOW);

            delay(630);

            digitalWrite(RED_LED, HIGH);            //red and green
            digitalWrite(BLUE_LED, LOW);
            digitalWrite(GREEN_LED, HIGH);

            delay(630);

            digitalWrite(RED_LED, HIGH);            //blue and red
            digitalWrite(BLUE_LED, HIGH);
            digitalWrite(GREEN_LED, LOW);

            delay(630);

            digitalWrite(RED_LED, LOW);            //green and blue
            digitalWrite(BLUE_LED, HIGH);
            digitalWrite(GREEN_LED, HIGH);

            delay(630);

            digitalWrite(RED_LED, LOW);           //all off
            digitalWrite(BLUE_LED, LOW);
            digitalWrite(GREEN_LED, LOW);

            delay(630);

           }

  }
 
}

void bumper5(){  //turn off all leds when bumper 5 is pressed
  if (isBumpSwitchPressed(4) == true){
    digitalWrite(RED_LED, LOW);
    digitalWrite(BLUE_LED, LOW);
    digitalWrite(GREEN_LED, LOW);
  }
  
}
