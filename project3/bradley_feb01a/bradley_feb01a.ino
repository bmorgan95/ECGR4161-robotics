//*************************************************************
//  Bradley Morgan and Jacob Santschi
//  Intro to Robotics
//  2-2-2023
//  LAB 03
//*************************************************************

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

//*************************************************************
//  Function for only Bumper 1, blinking Red
//*************************************************************

void bumper1() {                          //blink red 1Hz (cycle time 1000ms)

  
  if (isBumpSwitchPressed(0) == true and
      isBumpSwitchPressed(1) == false and
      isBumpSwitchPressed(2) == false and
      isBumpSwitchPressed(3) == false and
      isBumpSwitchPressed(4) == false and
      isBumpSwitchPressed(5) == false){

    while (isBumpSwitchPressed(0) == true){       //do nothing while switch is held
      
    }

    while (isBumpSwitchPressed(0) == false and      //loop if no bumpers are pressed
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

      delay(50);           //delay after function loop to ensure smooth transitions

  }
}

//*************************************************************
//  Function for only Bumper 2, blinking Green
//*************************************************************

void bumper2(){                           //blink green 2Hz (cycle time 500ms)

  
  if (isBumpSwitchPressed(0) == false and
      isBumpSwitchPressed(1) == true and
      isBumpSwitchPressed(2) == false and
      isBumpSwitchPressed(3) == false and
      isBumpSwitchPressed(4) == false and
      isBumpSwitchPressed(5) == false){

    while (isBumpSwitchPressed(1) == true){         //do nothing while switch is held
      
    }

    while (isBumpSwitchPressed(0) == false and      //loop if no bumpers are pressed
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

      delay(50);           //delay after function loop to ensure smooth transitions

  }
}

//*************************************************************
//  Function for only Bumper 3, blinking Blue
//*************************************************************

void bumper3(){                            //blink blue 4Hz (cycle time 250ms)

  
  if (isBumpSwitchPressed(0) == false and
      isBumpSwitchPressed(1) == false and
      isBumpSwitchPressed(2) == true and
      isBumpSwitchPressed(3) == false and
      isBumpSwitchPressed(4) == false and
      isBumpSwitchPressed(5) == false){

    while (isBumpSwitchPressed(2) == true){       //do nothing while switch is held
      
    }

    while (isBumpSwitchPressed(0) == false and      //loop if no bumpers are pressed
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

      delay(50);           //delay after function loop to ensure smooth transitions

  }

}

//*************************************************************
//  Function for Bumpers 1 and 2, blinking Red and Green
//*************************************************************

void bumper12(){


  if (isBumpSwitchPressed(0) == true and
      isBumpSwitchPressed(1) == true and
      isBumpSwitchPressed(2) == false and
      isBumpSwitchPressed(3) == false and
      isBumpSwitchPressed(4) == false and
      isBumpSwitchPressed(5) == false){

  
                                                       //red green blink, 16Hz (125ms cycle time)
     

        while (isBumpSwitchPressed(0) == true or       //do nothing while switch is held
               isBumpSwitchPressed(1) == true){
                
               }

        while (isBumpSwitchPressed(0) == false and      //loop if no bumpers are pressed
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

      delay(50);           //delay after function loop to ensure smooth transitions

      }

}

//*************************************************************
//  Function for Bumpers 1 and 3, blinking Red and Blue
//*************************************************************

void bumper13(){


  if (isBumpSwitchPressed(0) == true and
      isBumpSwitchPressed(1) == false and
      isBumpSwitchPressed(2) == true and
      isBumpSwitchPressed(3) == false and
      isBumpSwitchPressed(4) == false and
      isBumpSwitchPressed(5) == false){

                                                       //red blue blink, 16Hz
     

        while (isBumpSwitchPressed(1) == true or     //do nothing while switch is held
            isBumpSwitchPressed(2) == true){
              
            }


        while (isBumpSwitchPressed(0) == false and //loop if no bumpers are pressed
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

      delay(50);           //delay after function loop to ensure smooth transitions

   }

}

//*************************************************************
//  Function for Bumpers 2 and 3, blinking Green and Blue
//*************************************************************

void bumper23(){


  if (isBumpSwitchPressed(0) == false and
      isBumpSwitchPressed(1) == true and
      isBumpSwitchPressed(2) == true and
      isBumpSwitchPressed(3) == false and
      isBumpSwitchPressed(4) == false and
      isBumpSwitchPressed(5) == false){

  
                                                    //green blue blink, 32Hz 


        while (isBumpSwitchPressed(1) == true or          //do nothing while switch is held 
            isBumpSwitchPressed(2) == true){
              
            }

        
        while (isBumpSwitchPressed(0) == false and //loop if no bumpers are pressed
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

      delay(50);           //delay after function loop to ensure smooth transitions
      
    }

}

//*************************************************************
//  Function for all 3 Bumpers, keeping all 3 LEDs on
//*************************************************************

void bumper123(){


  if (isBumpSwitchPressed(0) == true and
      isBumpSwitchPressed(1) == true and
      isBumpSwitchPressed(2) == true and
      isBumpSwitchPressed(3) == false and
      isBumpSwitchPressed(4) == false and
      isBumpSwitchPressed(5) == false){

        while (isBumpSwitchPressed(0) == true or          //do nothing while switch is held
            isBumpSwitchPressed(1) == true or
            isBumpSwitchPressed(2) == true){
              
            }
      
    digitalWrite(RED_LED, HIGH);                  //all leds on, leave on
    digitalWrite(BLUE_LED, HIGH);
    digitalWrite(GREEN_LED, HIGH);

      delay(50);           //delay after function loop to ensure smooth transitions
    
  }
}

//*************************************************************
//  Function for only Bumper 4, blinking 3 LEDs in sequence
//*************************************************************

void bumper4(){                                     //rgb rainbow vomit when bumper 4 is pressed


  
  if (isBumpSwitchPressed(0) == false and
      isBumpSwitchPressed(1) == false and
      isBumpSwitchPressed(2) == false and
      isBumpSwitchPressed(3) == true and
      isBumpSwitchPressed(4) == false and
      isBumpSwitchPressed(5) == false){

    while (isBumpSwitchPressed(3) == true){         //do nothing while switch is held
      
    }


    
    while (isBumpSwitchPressed(0) == false and      //loop if no bumpers are pressed
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

      delay(50);           //delay after function loop to ensure smooth transitions
           

  }
 
}

//*************************************************************
//  Function for only Bumper 5, deactivating all LEDs
//*************************************************************

void bumper5(){  //turn off all leds when bumper 5 is pressed

  
  if (isBumpSwitchPressed(0) == false and
      isBumpSwitchPressed(1) == false and
      isBumpSwitchPressed(2) == false and
      isBumpSwitchPressed(3) == false and
      isBumpSwitchPressed(4) == true and
      isBumpSwitchPressed(5) == false){

    while (isBumpSwitchPressed(4) == true){      //do nothing while switch is pressed
      
    }


    digitalWrite(RED_LED, LOW);
    digitalWrite(BLUE_LED, LOW);
    digitalWrite(GREEN_LED, LOW);
  }

      delay(50);           //delay after function loop to ensure smooth transitions
  
  
}
