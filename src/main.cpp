/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// leftRear             motor         1               
// ptoFront             motor         2               
// ptoRear              motor         3               
// leftFront            motor         4               
// leftMid              motor         5               
// rightRear            motor         6               
// intakeMtr            motor         7               
// cataMtr              motor         8               
// flapsCtl             digital_out   A               
// cataButton           bumper        B               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

const double pi = 3.14159265359; // Define the number pi
const double circum4Omni = 4*pi;
const double innerCircum = 2*pi*6;//circumference of robot
const double botCircum = 2*pi*13;//circumference of robot's turning circle

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

void stopDrivetrain() {
  leftFront.stop();
  leftMid.stop();
  leftRear.stop();
  ptoFront.stop();
  ptoRear.stop();
  rightRear.stop();
  // Stop the robot.
}

// This function resets all the chassis motor encoders to zero.
void clearChassisRotation() {
  leftFront.resetPosition();
  leftMid.resetPosition();
  leftRear.resetPosition();
  ptoFront.resetPosition();
  ptoRear.resetPosition();
  rightRear.resetPosition();
  // Stop the robot.
}


/*
moveChassis Function: This function takes a distance, in inches,
and converts it into a certain angle that the robot's motors will
rotate to.
Positive distance = the robot moves forward
Negative distance = the robot moves backward

Parameters:
(travel distance in inches, velocity of drive train's left side, velocity of drive train's right side)
*/
void moveChassis(double distance, int leftVel, int rightVel) {
  leftFront.setVelocity(leftVel, percent);
  leftMid.setVelocity(leftVel/(21/5), percent);
  leftRear.setVelocity(leftVel, percent);
  ptoFront.setVelocity(rightVel, percent);
  ptoRear.setVelocity(rightVel, percent);
  rightRear.setVelocity(rightVel, percent);
  clearChassisRotation(); // Call this function to prevent confusion from a
                          // previous function.
  double degGoal = ((360 / circum4Omni) * distance); // Creates a variable that stores the degree to
                             // determine how many times the motors will rotate.
  double degC = 0; // Creates a variable to store the drivetrain's motor
                   // positions in degrees.
  if (degGoal > 0) {
    // Move the robot forwards while the robot hasn't reached the specified
    // distance.
    while (degC < degGoal) {
      /*
      1. Calculate the drivetrain's motor positions in degrees by getting the
      average value of all the drivetrain's motors.
      2. Store the average value of the drivetrain's motor positions.
      */
      degC = (leftFront.position(degrees) + (leftMid.position(degrees)/21.5) + leftRear.position(degrees) + ptoFront.position(degrees) + ptoRear.position(degrees)+ rightRear.position(degrees)) /6;

      leftFront.spin(forward);
      leftMid.spin(forward);
      leftRear.spin(forward);
      ptoFront.spin(forward);
      ptoRear.spin(forward);
      rightRear.spin(forward);
    }
  } else if (degGoal < 0) {
    // Move the robot backwards while the robot hasn't reached the specified
    // distance.
    while (degC > degGoal) {
      // Same calculations that are in the previous while loop.
      degC = (leftFront.position(degrees) + (leftMid.position(degrees)/21.5) + leftRear.position(degrees) + ptoFront.position(degrees) + ptoRear.position(degrees)+ rightRear.position(degrees)) /6;

      leftFront.spin(reverse);
      leftMid.spin(reverse);
      leftRear.spin(reverse);
      ptoFront.spin(reverse);
      ptoRear.spin(reverse);
      rightRear.spin(reverse);
    }
  }
  leftFront.stop(brake);
  leftMid.stop(brake);
  leftRear.stop(brake);
  ptoFront.stop(brake);
  ptoRear.stop(brake);
  rightRear.stop(brake);
}
/*
This function turns the bot to the right by pivoting it on its right side.
Positive angle = the robot pivots forward
Negative angle = the robot pivots backward
*/
void pivotRight(double degr) {
  clearChassisRotation(); // Call this function to prevent confusion from a
                          // previous function.
  double distG = (botCircum/ 360) * degr; // Creates a variable that stores the distance, in inches, of how
            // much the robot's left side will have to cover
  double distC = 0; // Creates a variable to store the distance covered by the
                    // left side of the drivetrain in inches.
  if (distG > 0) {
    // Pivot the robot forwards while the robot hasn't met the desired angle.
    while (distC < distG) {
      /*
      Calculate the distance covered by the left side drivetrain's motors in
      inches by:
      1. Retrieving the left front motor's angle, in degrees.
      2. Converting them into inches.
      Then:
      Store the average value of the distance covered by the drivetrain's left
      side motors.
      */
      distC = leftFront.position(degrees) * ((circum4Omni) / 360) * 0.845;
      leftFront.spin(forward);
      leftMid.spin(forward);
      leftRear.spin(forward);
      ptoFront.stop(brake);
      ptoRear.stop(brake);
      rightRear.stop(brake);
    }
  } else if (distG < 0) {
    // Pivot the robot backwards while the robot hasn't met the desired angle.
    while (distC > distG) {
      // Same calculations that are in the above while loop.
      distC = leftFront.position(degrees) * ((circum4Omni) / 360) * 0.845;
      leftFront.spin(reverse);
      leftMid.spin(reverse);
      leftRear.spin(reverse);
      ptoFront.stop(brake);
      ptoRear.stop(brake);
      rightRear.stop(brake);
    }
  }
  leftFront.stop(brake);
  leftMid.stop(brake);
  leftRear.stop(brake);
  ptoFront.stop(brake);
  ptoRear.stop(brake);
  rightRear.stop(brake);
}

void rotateRight(double degr) {
  leftFront.setVelocity(50, percent);
  leftMid.setVelocity(11.9, percent);
  leftRear.setVelocity(50, percent);
  ptoFront.setVelocity(50, percent);
  ptoRear.setVelocity(50, percent);
  rightRear.setVelocity(50, percent);

  clearChassisRotation(); // Call this function to prevent confusion from a
                          // previous function.
  double distG = (innerCircum/ 360) * degr; // Creates a variable that stores the distance, in inches, of how
            // much the robot's left side will have to cover
  double distC = 0; // Creates a variable to store the distance covered by the
                    // left side of the drivetrain in inches.
  if (distG > 0) {
    // Pivot the robot forwards while the robot hasn't met the desired angle.
    while (distC < distG) {
      /*
      Calculate the distance covered by the left side drivetrain's motors in
      inches by:
      1. Retrieving the left front motor's angle, in degrees.
      2. Converting them into inches.
      Then:
      Store the average value of the distance covered by the drivetrain's left
      side motors.
      */
      distC = ((leftFront.position(degrees) + rightRear.position(degrees))/2) * ((circum4Omni) / 360) * 0.845;
      leftFront.spin(forward);
      leftMid.spin(forward);
      leftRear.spin(forward);
      ptoFront.spin(reverse);
      ptoRear.spin(reverse);
      rightRear.spin(reverse);
    }
  } else if (distG < 0) {
    // Pivot the robot backwards while the robot hasn't met the desired angle.
    while (distC > distG) {
      // Same calculations that are in the above while loop.
      distC = ((leftFront.position(degrees) + rightRear.position(degrees))/2) * ((circum4Omni) / 360) * 0.845;
      leftFront.spin(reverse);
      leftMid.spin(reverse);
      leftRear.spin(reverse);
      ptoFront.spin(forward);
      ptoRear.spin(forward);
      rightRear.spin(forward);
    }
  }
  leftFront.stop(brake);
  leftMid.stop(brake);
  leftRear.stop(brake);
  ptoFront.stop(brake);
  ptoRear.stop(brake);
  rightRear.stop(brake);
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  //match auton
  moveChassis(36, 100, 100);
  moveChassis(-18, 100, 100);

  //Skills Auton
  /*
  moveChassis(2, 50, 50);
  cataMtr.setVelocity(50, percent);
  cataMtr.spin(forward);
  wait(30, seconds);
  cataMtr.setVelocity(0, percent);
  wait(500, msec);
  moveChassis(-48, 50, 50);
  */

  //win point auton


}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // User control code here, inside the loop
  
  //for pto
	bool pto = true; //true = chassis, false = lift
	
	//for wings
	bool wing = false;//true = expanded, false = retracted

	//for catapult
	bool cataAdj = false;//false = auto launch, true = manually adjust gear

  //for chassis controls
	int power = 0;
	int turn = 0;
	int left = 0;
	int right = 0;

  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

		power = Controller1.Axis3.position()*0.12;
    turn = Controller1.Axis1.position()*0.12;
		left = power + turn;
		right = power - turn;

    leftFront.spin(forward, left, volt);
    leftMid.spin(forward, left/(21/5), volt);
    leftRear.spin(forward, left, volt);

    rightRear.spin(forward, right, volt);

		if(pto){
      ptoFront.spin(forward, right, volt);
      ptoRear.spin(forward, right, volt);
    }
    else{
      if(Controller1.ButtonUp.pressing()){
        ptoFront.spin(forward, 12, volt);
        ptoRear.spin(forward, 12, volt);        
      }
      else if(Controller1.ButtonDown.pressing()){
        ptoFront.spin(reverse, 12, volt);
        ptoRear.spin(reverse, 12, volt); 
      }
      else{
        ptoFront.stop(brake);
        ptoRear.stop(brake);
      }
    }
  
    if (Controller1.ButtonX.pressing()) {
      ptoFront.stop(brake);
      ptoRear.stop(brake);

      if(pto){
				pto = false;
			}
			else{
				pto = true;
			}
      ptoCtl.set(!pto);			
			waitUntil(!Controller1.ButtonX.pressing());
    }

    if (Controller1.ButtonA.pressing()) {
      if(!wing){
				wing = true;
			}
			else{
				wing = false;
			}
      flapsCtl.set(wing);
      waitUntil(!Controller1.ButtonA.pressing());
    }
    
    if (Controller1.ButtonLeft.pressing()) {
      if(!cataAdj){
				cataAdj = true;
			}
			else{
				cataAdj = false;
				cataMtr.resetPosition();
			}
      waitUntil(!Controller1.ButtonLeft.pressing());
    }

    if (Controller1.ButtonL1.pressing()) {
			if(!cataAdj){
				cataMtr.setVelocity(100, percent);
        cataMtr.setTimeout(1, seconds);
        cataMtr.spinToPosition(360, degrees);
				/*
				while(!cataButton.get_value()){
					cataMtr.setVelocity(51, percent);
          cataMtr.spin(forward);
				}
				*/
				cataMtr.stop(brake);
				cataMtr.resetPosition();
			}
			else if(cataAdj){
				while(Controller1.ButtonL1.pressing()){
				  cataMtr.setVelocity(25, percent);
          cataMtr.spin(forward);
				}
				cataMtr.stop(brake);
			}
    }
    else{
			cataMtr.stop();
		}

    if (Controller1.ButtonL2.pressing()) {
			if(cataAdj){
        while(Controller1.ButtonL2.pressing()){
				  cataMtr.setVelocity(25, percent);
          cataMtr.spin(reverse);
				}
				cataMtr.stop(brake);
      }
    }

    if (Controller1.ButtonR1.pressing()) {
      intakeMtr.setVelocity(100, percent);
      intakeMtr.spin(forward);
    }
    else if (Controller1.ButtonR2.pressing()) {
      intakeMtr.setVelocity(100, percent);
      intakeMtr.spin(reverse);
    }
    else{
      intakeMtr.setVelocity(0, percent);
      intakeMtr.stop(coast);
    }

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
