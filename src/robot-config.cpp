#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor leftRear = motor(PORT1, ratio18_1, false);
motor ptoFront = motor(PORT2, ratio18_1, true);
motor ptoRear = motor(PORT3, ratio18_1, false);
motor leftFront = motor(PORT4, ratio18_1, false);
motor leftMid = motor(PORT5, ratio6_1, false);
motor rightRear = motor(PORT6, ratio18_1, true);
motor intakeMtr = motor(PORT7, ratio18_1, true);
motor cataMtr = motor(PORT8, ratio18_1, true);
digital_out flapsCtl = digital_out(Brain.ThreeWirePort.A);
bumper cataButton = bumper(Brain.ThreeWirePort.B);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}