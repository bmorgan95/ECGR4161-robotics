//
//  Bradley Morgan and Jacob Santschi
//  Intro to Robotics
//  2-4-2023
//  LAB 04
//
#include <SimpleRSLK.h>

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly: 
  
}













/* Edited by Joey Phillips 6-20-22
 * Energia Robot Library for Texas Instruments' Robot System Learning Kit (RSLK)
 * Simplified Drive Straight Encoder Example
 *
 * Summary:
 * This example has the TI Robotic System Learning Kit (TI RSLK) driving straight forward
 * by a predefined distance utilizing wheel encoders.
 *
 * How to run:
 * 1) Push left button on Launchpad to start the demo
 * 2) Robot will drive forward by a predefined distance
 * 3) Once distance has been reached the robot will stop
 * 4) Push left button again to start demo again
 *
 */

#include "SimpleRSLK.h"

/* Diameter of Romi wheels */
float wheelDiameter = 2.7559055;  // in inches
//float wheelDiameter = 6.99999997; // in centimeters

/* Number of encoder (rising) pulses every time the wheel turns completely */
int cntPerRevolution = 360;

/* How far in inches for the robot to travel */
int inchesToTravel = 48;
//int centimetersToTravel = 122;

int wheelSpeed = 15; // Default raw pwm speed for motor.

/* The distance the wheel turns per revolution is equal to the diameter * PI.
 * The distance the wheel turns per encoder pulse is equal to the above divided
 * by the number of pulses per revolution.
 * Remember to use the appropriate number for specified units of measurements 
 *    - i.e. inches or centimeters
 */
 /* Function returns how far the robot has traveled based on the number of pulses */
float distanceTraveled(float wheel_diam, uint16_t cnt_per_rev, uint8_t current_cnt) {
	float temp = (wheel_diam * PI * current_cnt) / cnt_per_rev;
	return temp;
}

/* Function returns the number of pulses needed to drive for a specific distance */
uint32_t countForDistance(float wheel_diam, uint16_t cnt_per_rev, uint32_t distance) {
	float temp = (wheel_diam * PI) / cnt_per_rev;
	temp = distance / temp;
	return int(temp);
}

void setup() {
	Serial.begin(9600);

	setupRSLK();
	/* Left button on Launchpad */
	setupWaitBtn(LP_LEFT_BTN);
	/* Red led in rgb led */
	setupLed(RED_LED);
}

void loop() {
	uint16_t leftPulseCount = 0; // Total amount of encoder pulses received
  uint16_t rightPulseCount = 0; // Total amount of encoder pulses received

	/* Amount of encoder pulses needed to achieve distance */
	uint16_t x = countForDistance(wheelDiameter, cntPerRevolution, inchesToTravel);
	String btnMsg = "Expected count: ";
	btnMsg += x;

	/* Wait until button is pressed to start robot */
	btnMsg += "\nPush left button on Launchpad to start demo.\n";
	/* Wait until button is pressed to start robot */
	waitBtnPressed(LP_LEFT_BTN,btnMsg,RED_LED);

	delay(2000);

	/* Set the encoder pulses count back to zero */
	resetLeftEncoderCnt();
	resetRightEncoderCnt();

	/* Cause the robot to drive forward */
	setMotorDirection(BOTH_MOTORS,MOTOR_DIR_FORWARD);

	/* "Turn on" the motor */
	enableMotor(BOTH_MOTORS);

	/* Set motor speed */
	setMotorSpeed(BOTH_MOTORS,wheelSpeed);
  delay(500);

	/* Drive motor until it has received x pulses */
  while(leftPulseCount < x && rightPulseCount < x) {
    /* Assign variable to hold encoder count */
		leftPulseCount = getEncoderLeftCnt();
		rightPulseCount = getEncoderRightCnt();

    /* IF statements to control the speed of the motors based off of encoder counts.
       This will help keep the robot driving straight. These statements and wheel speeds
       may need to be altered in order to acheive a specfic expectation 

       If the left encoder count is less than the right, increase
       the left motor speed */
    if (leftPulseCount < rightPulseCount) {
      setMotorSpeed(LEFT_MOTOR,wheelSpeed++);
      setMotorSpeed(RIGHT_MOTOR,wheelSpeed);
    }
    
    /* If the right encoder count is less than the left, increase
       the left motor speed */
    if (rightPulseCount < leftPulseCount) {
      setMotorSpeed(LEFT_MOTOR,wheelSpeed);
      setMotorSpeed(RIGHT_MOTOR,wheelSpeed++);
    }
	}
	/* Halt motors */
	disableMotor(BOTH_MOTORS);
}