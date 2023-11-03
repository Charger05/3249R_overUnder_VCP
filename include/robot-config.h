using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor leftRear;
extern motor ptoFront;
extern motor ptoRear;
extern motor leftFront;
extern motor leftMid;
extern motor rightRear;
extern motor intakeMtr;
extern motor cataMtr;
extern digital_out flapsCtl;
extern bumper cataButton;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );