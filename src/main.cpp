/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       patrick                                                   */
/*    Created:      10/8/2023, 5:21:11 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;


brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);

motor left_front = motor(PORT9, ratio6_1, false);
motor left_back = motor(PORT20, ratio6_1, false);

motor_group LeftDriveSmart = motor_group(left_front, left_back);

motor right_front = motor(PORT14, ratio6_1, true);
motor right_back = motor(PORT19, ratio6_1, true);

motor_group RightDriveSmart = motor_group(right_front, right_back);
drivetrain Drivetrain = drivetrain(LeftDriveSmart, RightDriveSmart, 319.19, 295, 40, mm, 1);
motor intake = motor(PORT15, ratio6_1, false);
motor catapult1 = motor(PORT10, ratio36_1, false);
motor catapult2 = motor(PORT11, ratio36_1, false);
inertial imu = inertial(PORT3);
digital_out blocker = digital_out(Brain.ThreeWirePort.B);

digital_out wings = digital_out(Brain.ThreeWirePort.A);
digital_out balance = digital_out(Brain.ThreeWirePort.C);


const int scale = 120;
bool piston_set = false;
bool last_press = false;
bool cata_set = false;
bool cataLastPress = false;
bool wings_set = false;
bool wingsLastPress = false;

void set_tank(double left, double right)
{
  left_back.spin(reverse, left*scale, voltageUnits::mV);
  left_front.spin(reverse, left*scale, voltageUnits::mV);
  right_front.spin(reverse, right*scale, voltageUnits::mV);
  right_back.spin(reverse, right*scale, voltageUnits::mV);

}

void left_FW(double left)
{
  left_back.spin(reverse, left*scale, voltageUnits::mV);
  left_front.spin(reverse, left*scale, voltageUnits::mV);
}

void right_FW(double right)
{
  right_back.spin(reverse, right*scale, voltageUnits::mV);
  right_front.spin(reverse, right*scale, voltageUnits::mV);
}

void pre_auton(void) 
{
  right_back.setBrake(hold);
  right_front.setBrake(hold);
  left_back.setBrake(hold);
  left_front.setBrake(hold);
  
  imu.calibrate();
  // while(imu.isCalibrating()){
  //   wait(100, msec);
  // }
  int a = 0;
  int b = 0;
  printf("a: %d, b: %d ", a, b);

  wait(2, sec);
}

// void pd_turn(double target, double speed){
//   // Two parameters: degrees (target) and speed
//   double k = 1.5;
//   double rk = 2;
//   double lk = 2;
//   double left_output = 0;
//   double right_output = 0;
//   imu.resetRotation();
//   double error = 0;
//   error = target - imu.rotation(deg);
//   while(true){
//     error = target - imu.rotation(deg);
//     Brain.Screen.print("Imu Rotation:  %d  ", imu.rotation(deg));
//     double output = k*error;
//     set_tank(-output, output);
//     if(error<0.5){
//       break;
//     }
//   }
// }

// Clipnum for speed
double clipnum(double input, double max, double min) {
  if (input > max)
  {
    return max;
  }
  else if (input < min)
  {
    return min;
  }
  return input;
}

double headingTarget = 0.0;
void setTurnPID(double target)
{
  headingTarget = target;
  double error = 0;
  double power1 = 0;
  double prevError = 0;
  double kP = 0.9;
  double kD = 0.9; // 1
  double kI = 0; // 0.001
  double derivative = 0;
  double integral = 0;
  int counter = 0;


  while(counter < 50)
  {


   //imu.resetRotation();

    error = target - imu.rotation(deg);
    /*integral = integral + error;
    if(abs(error) < 1)
    {
      integral = 0;
    }
    if(error > 40)
    {
      integral = 0;
    }*/

    derivative = error - prevError;
    prevError = error;
    power1 = error * kP + integral * kI + derivative * kD;
    //power1 = error * kP;
    power1 = -power1;
    if(abs(left_front.velocity(pct)) < 3.5)
    {
      counter += 10;
    }
    else
    {
      counter = 0;
    }

     Brain.Screen.print("Imu Rotation Deg: %f  ", imu.rotation(deg));
     Brain.Screen.newLine();

    left_back.spin(forward, power1*scale, voltageUnits::mV);
    left_front.spin(forward, power1*scale, voltageUnits::mV);
    right_front.spin(reverse, power1*scale, voltageUnits::mV);
    right_back.spin(reverse, power1*scale, voltageUnits::mV);
    wait(10, msec);


  } 
  left_front.stop(brake);
  left_back.stop(brake); 
  right_front.stop(brake);
  right_back.stop(brake);
}

void setDrivePID(double target, double speed)
{
  int leftError = 0;
  int rightError = 0;
  int power1 = 0;
  int power2 = 0;
  int prevLeftError = 0;
  int prevRightError = 0;
  int derivative1 = 0;
  int derivative2 = 0;
  int integral1 = 0;
  int integral2 = 0;
  float kP = 0.15;
  float kD = 0;
  float kI = 0;
  int counter = 0;

  double kPHeading = 0.2;
  double headingError = 0;
  double headingOutput = 0;

  left_front.resetPosition();
  right_front.resetPosition();

  const double wheel_Diameter = 4.0;
  const double gearRatio = 84.0/36.0;
  double conversion = (360.0 * gearRatio) / (wheel_Diameter * M_PI);
  target = target * conversion;

  while(counter < 50)
  {

    leftError = target - ( -left_front.position(deg));
    rightError = target -  (-  right_front.position(deg));
    headingError = headingTarget - imu.rotation(deg);


    integral1 = integral1 + leftError;
    integral2 = integral2 + rightError;
    
    // if(abs(leftError) < 5)
    // {
    //   integral1 = 0;
    // }
    // if(abs(rightError) < 5)
    // {
    //   integral2 = 0;
    // }
    // if(leftError > 500)
    // {
    //   integral1 = 0;
    // }
    // if(rightError > 500)
    // {
    //   integral2 = 0;
    // }
    
    derivative1 = leftError - prevLeftError;
    derivative2 = rightError - prevRightError;
    
    prevLeftError = leftError;
    prevRightError = rightError;
    
    power1 = leftError * kP + integral1 * kI + derivative1 * kD;
    power2 = rightError * kP + integral1 * kI + derivative2 * kD;
    headingOutput = headingError * kPHeading;

    power1 = clipnum(power1, speed, -speed);
    power2 = clipnum(power2, speed, -speed);

    // if(left_front.velocity(pct) < 5 && right_front.velocity(pct) < 5)
    if(abs(left_front.velocity(pct)) < 3.5 && abs(right_front.velocity(pct)) < 3.5)
    {
      counter += 10;

     // Brain.Screen.print("HERE"); wait(1, sec);

    }
    else
    {
      counter = 0;
    }

    // if(abs(leftError) < 0.25 && abs(rightError) < 0.25)
    // {
    //   counter += 10;

    //  // Brain.Screen.print("HERE"); wait(1, sec);

    // }
    // else
    // {
    //   counter = 0;
    // }


    left_front.spin(reverse, (power1 + headingOutput) *120, voltageUnits::mV);
    left_back.spin(reverse, (power1 + headingOutput) *120, voltageUnits::mV);
    right_front.spin(reverse, (power2 - headingOutput) *120, voltageUnits::mV);
    right_back.spin(reverse, (power2 - headingOutput)*120, voltageUnits::mV);

    


    //Brain.Screen.print("Inside PID Loop ");
    //Brain.Screen.newLine();
    //printf("Left Error: %d.  Right Error: %d. ", leftError, rightError);
    Brain.Screen.newLine();
    
    //Brain.Screen.print("Left Motor Position: %d.  Right Motor Position: %d. ", left_front.position(deg), right_front.position(deg));
    //Brain.Screen.print("Derivative1:  %d. ", derivative1);

    wait(10,msec);


  }

  

  left_front.stop(brake);
  left_back.stop(brake); 
  right_front.stop(brake);
  right_back.stop(brake);
  
}

// void set_TestAutonDrive(int revolutions)
// {
//   left_front.startRotateTo(directionType::rev, revolutions, rotationUnits::rev);
//   left_back.startRotateFor(directionType::rev, revolutions, rotationUnits::rev);
//   right_front.startRotateFor(directionType::rev, revolutions, rotationUnits::rev);
//   right_back.rotateFor(directionType::rev, revolutions, rotationUnits::rev);
// }

double curve(double x) {
  double t = 1;
  double e = 2.7182818284590452353602874713527;
  double exponent = ((abs(x)-100)*t)/1000.0;
  return pow(e, exponent) * x;
}

void set_Drive()
{
  int left_joy = Controller1.Axis3.value();
  int right_joy = Controller1.Axis2.value();

  if (abs(left_joy) < 5)
  {
    left_joy = 0; 
  }

  if (abs(right_joy) < 5)
  {
    right_joy = 0; 
  }

  double left = curve(left_joy);
  double right = curve(right_joy);

  set_tank(left, right);
}

void intake_ButtonControl()
{
  if(Controller1.ButtonR1.pressing())
  {
    intake.setVelocity(600, rpm);
    intake.spin(reverse, 600, rpm);
  }
  else if(Controller1.ButtonR2.pressing())
  {
    intake.setVelocity(600, rpm);
    intake.spin(forward, 600, rpm);
  }
  else
  {
    intake.stop();
  }

}

const int CATA_SPEED = 55 ;
void blocker_PneumaticControl()
{
  // catapult.spin(forward, 100, pct);
 
  // if(Controller1.ButtonL2.pressing())
  // {
  //  // catapult.setVelocity(100, rpm);
  //  // Brain.Screen.print("Entered 2st If");
  //   catapult.spin(forward, CATA_SPEED*scale, voltageUnits::mV);
  // }
  // else
  // {
  //   catapult.stop(brake);
  // }

  if(Controller1.ButtonL2.pressing() and last_press == false)
  {
    piston_set = !piston_set;
  }
  // keep track of press
  last_press = Controller1.ButtonL2.pressing();

  blocker.set(piston_set);
}


void catapult_ButtonControl()
{
  if(Controller1.ButtonA.pressing() and cataLastPress == false)
  {
    cata_set = !cata_set;
  }
  // keep track of press
  cataLastPress = Controller1.ButtonA.pressing();

  if (cata_set) 
  {
    catapult1.spin(forward, CATA_SPEED*120, voltageUnits::mV);
    catapult2.spin(reverse, CATA_SPEED*120, voltageUnits::mV);

  } else
  {
    catapult1.spin(forward, 0, voltageUnits::mV);
    catapult2.spin(reverse, 0, voltageUnits::mV);
  }

  
}

void wings_PneumaticControl()
{
  if(Controller1.ButtonL1.pressing() and wingsLastPress == false)
  {
    wings_set = !wings_set;
  }
  // keep track of press
  wingsLastPress = Controller1.ButtonL1.pressing();

  wings.set(wings_set);
  /*
  if(Controller1.ButtonL2.pressing() and last_press == false)
  {
    piston_set = !piston_set;
  }
  // keep track of press
  last_press = Controller1.ButtonL2.pressing();

  blocker.set(piston_set);
  */
}

void balance_PneumaticControl()
{
  balance.set(Controller1.ButtonLeft.pressing());  
}

// void intake_AutomatedControl()
// {
//   intake.setVelocity(5, rpm);
//   intake.spin(reverse);
// }
    

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;



void turn(double target, int speed) {
  double kP = 0.5; // kP (scaling number)
  int x = 0; // Timer for exit condition
  while (true) {
    double error = target - imu.rotation(deg); // error = (target - current)
    int output = clipnum(error * kP, speed, -speed);
    set_tank(output, -output); // Set motors to (error * kP) and clip it to speed

    // If the velocity of the left and right motors are 0...
    if (left_back.velocity(pct) == 0 && right_back.velocity(pct) == 0) {
      x+=10; // Increase x by 10
      if (x >= 50){ // If x is 50 (meaning the motors were at 0 velocity for 50ms)...
        break; // Break the while loop
      }
    } 
    // If the velocity of the left and right motors are not 0...
    else {
      x = 0; // Set the timer to 0
    }

    wait(10, msec);
  }
  set_tank(0, 0); // Make sure motors are off before leaving this function
}





bool driveTrainMoving = false;
bool driveTrainMovingForOutake = false;

int rotateIntakeWhileMoving() {

  intake.stop();

  while (driveTrainMoving) {

    intake.spin(reverse, 600, rpm);
    wait(10, msec);

  
  }
  
  driveTrainMoving = false;
  intake.stop();

  // A task's callback must return an int, even though the code will never get
  // here. You must return an int here. Tasks can exit, but this one does not.
  return 0;
}

int outakeWhileMoving()
{
  intake.stop();

  while(driveTrainMovingForOutake)
  {
    intake.spin(forward, 600, rpm);
    wait(10, msec);
  }

  driveTrainMovingForOutake = false;
  intake.stop();

  return 0;
}

void backFieldAutonomousOneTriball(void)
{
  wings.set(false);
  setDrivePID(4, 80);
  setTurnPID(-50);
  setDrivePID(32.5, 80);
  setTurnPID(4);
  intake.spin(forward, 600, rpm);
  wait(0.5, sec);
  intake.stop();
  setDrivePID(-4, 80);
  setTurnPID(170);
  setDrivePID(-10, 100);
  wait(0.1, sec);
  setDrivePID(4, 80);
  wait(0.1, sec);
  setTurnPID(140);
  setDrivePID(27, 80);
  setTurnPID(-82);
  setDrivePID(-34.75, 80);
  blocker.set(true);
}

void backFieldAutonomousOneTribalAndDescore()
{
  wings.set(false);
  setDrivePID(4, 80);
  setTurnPID(-50);
  setDrivePID(32.5, 80);
  setTurnPID(4);
  intake.spin(forward, 600, rpm);
  wait(0.5, sec);
  intake.stop();
  setDrivePID(-4, 80);
  setTurnPID(170);
  setDrivePID(-10, 100);
  wait(0.1, sec);
  setDrivePID(12, 80);
  setTurnPID(125);
  wings.set(true);
  setDrivePID(16, 200);
  wings.set(false);
  //Starting Rotation
  wait(0.3, sec);
  setDrivePID(6, 80);
  setTurnPID(85);
  setDrivePID(34.75, 80);
}

void frontFieldAutonomousOneTriball(void) 
{
  setDrivePID(50, 80);
  setTurnPID(90);
  intake.spin(forward, 600, rpm);
  wait(1.5, sec);
  intake.stop();
  setDrivePID(10, 200);
  setTurnPID(45);
  setDrivePID(-34, 80);
  setTurnPID(0);
  setDrivePID(-12, 80);
  blocker.set(true);


  /*
  Drivetrain.driveFor(reverse, 44, inches, 100, rpm);
  Drivetrain.turnFor(left, 82, deg, 100, rpm);
  Drivetrain.driveFor(reverse, 12, inches, 100, rpm);

  intake.spin(forward, 50, rpm);
  wait(1.25, sec);
  intake.stop();

  Drivetrain.driveFor(reverse, 6, inches, 100, rpm);
  Drivetrain.driveFor(forward, 12, inches, 100, rpm);
  
 
  Drivetrain.turnFor(left, 175, deg, 100, rpm);

  driveTrainMoving = true;
  task rotateIntake = task(rotateIntakeWhileMoving);

  
  Drivetrain.driveFor(dixrectionType::rev, 18, distanceUnits::in, 100, rpm);


  driveTrainMoving = false;
  rotateIntake.stop();



  intake.stop();

  Drivetrain.turnFor(left, 175, deg, 100, rpm);

  Drivetrain.driveFor(directionType::rev, 28, distanceUnits::in, 100, rpm);

  intake.spin(forward, 50, rpm);
  wait(1.25, sec);
  intake.stop();

  Drivetrain.driveFor(directionType::rev, 6, distanceUnits::in, 100, rpm);
*/

}


void frontFieldAutonomousThreeTriball()
{
  setDrivePID(45, 100);
  printf("HiHi\n");
  intake.spin(forward, 100, rpm);
  wait(1.25, sec);
  intake.stop();
  setDrivePID(-15, 100);
  setTurnPID(-45);
  driveTrainMoving = true;
  task rotateIntake = task(rotateIntakeWhileMoving);
  printf("Hi\n");
  setDrivePID(36, 100);
  driveTrainMoving = false;
  intake.stop();
  setTurnPID(90);
  intake.spin(forward, 300, rpm);
  wait(1, sec);
  intake.stop();
  wait(0.1, msec);
  wings.set(true);
  driveTrainMovingForOutake = true;
  task outakeWhileMovingAtEnd = task(outakeWhileMoving);
  setDrivePID(30, 100);
  driveTrainMovingForOutake = false;
  intake.stop();
  wings.set(false);
  setTurnPID(25.5);
  setDrivePID(-53, 200);
  blocker.set(true);
}

void frontFieldAutonomousFourTriball()
{
  setDrivePID(45, 100);
  printf("HiHi\n");
  intake.spin(forward, 100, rpm);
  wait(1.25, sec);
  intake.stop();
  setDrivePID(-15, 100);
  setTurnPID(-45);
  driveTrainMoving = true;
  task rotateIntake = task(rotateIntakeWhileMoving);
  printf("Hi\n");
  setDrivePID(36, 100);
  driveTrainMoving = false;
  intake.stop();
  setTurnPID(90);
  intake.spin(forward, 300, rpm);
  wait(1, sec);
  intake.stop();
  wait(0.1, msec);
  wings.set(true);
  driveTrainMovingForOutake = true;
  task outakeWhileMovingAtEnd = task(outakeWhileMoving);
  setDrivePID(30, 100);
  driveTrainMovingForOutake = false;
  intake.stop();
  wings.set(false);
  setTurnPID(-120);
  intake.spin(reverse, 600, rpm);
  setDrivePID(35, 60);
  intake.stop();
  setTurnPID(60);
  wings.set(true);
  intake.spin(forward, 600, rpm);
  setDrivePID(50, 300);
  intake.stop();
}

void skill_AutonWithWingsMatchLoading()
{
  wings.set(true);
  wait(1, sec);
  wings.set(false);
  setDrivePID(10, 80);
  setTurnPID(-130);
  setDrivePID(-23, 200);
  setTurnPID(-90);
  setDrivePID(-15, 600);
  setDrivePID(10, 200);
  setTurnPID(-15.5);
  wings.set(true);
  catapult1.spin(forward, CATA_SPEED*120, voltageUnits::mV);
  catapult2.spin(reverse, CATA_SPEED*120, voltageUnits::mV);
  wait(30, sec);
  catapult1.stop();
  catapult2.stop();  
  wings.set(false);
  setTurnPID(-140);
  setDrivePID(31.5, 200);
  setTurnPID(-182.7);
  setDrivePID(75, 200);
  setTurnPID(-114.75);
  setDrivePID(-34, 200);
  setTurnPID(-175.5);
  setDrivePID(-20, 200);
  setTurnPID(-210);
  setTurnPID(-210);
  wings.set(true);
  setDrivePID(40, 200);
  wings.set(false);
  setTurnPID(-170);
  setDrivePID(-40, 200);
  setTurnPID(-185);
  wings.set(true);
  setDrivePID(40, 200);
  wings.set(false);
  wait(0.2, sec);
  setDrivePID(-6, 200);
  setTurnPID(-140);
  setDrivePID(-40, 200);
  setTurnPID(-140);
  wings.set(true);
  setDrivePID(40, 200);
  wings.set(false);
  setDrivePID(-10, 200);

// setDrivePID(-23, 200);
  // setTurnPID(-90);
}

void skill_Auton()
{
  // catapult.spin(forward, CATA_SPEED*scale, voltageUnits::mV);
  // wait(10, sec);
  // catapult.spin(forward, 0, voltageUnits::mV);
  // setDrivePID(-24, 100);
  // setTurnPID(-37);
  // setDrivePID(19, 80);


  
  wings.set(true);
  wait(1, sec);
  wings.set(false);
  
  setDrivePID(10, 80);
  setTurnPID(-37.125);
  setDrivePID(5.825, 80);


  catapult1.spin(forward, CATA_SPEED*120, voltageUnits::mV);
  catapult2.spin(reverse, CATA_SPEED*120, voltageUnits::mV);

  wait(2, sec);

  catapult1.stop();
  catapult2.stop();
  setDrivePID(-4,200);
  setTurnPID(-130);
  setDrivePID(-23, 200);
  setTurnPID(-90);
  setDrivePID(-15, 600);
  setDrivePID(7, 200);
  setTurnPID(-140);
  setDrivePID(29.50, 200);
  setTurnPID(-180.7);
  setDrivePID(80, 200);
  setTurnPID(-114.75);
  setDrivePID(-34, 200);
  setTurnPID(-175.5);
  setDrivePID(-20, 200);
  setTurnPID(-210);
  wings.set(true);
  setDrivePID(40, 200);
  wings.set(false);
  setTurnPID(-170);
  setDrivePID(-40, 200);
  setTurnPID(-185);
  wings.set(true);
  setDrivePID(40, 200);
  wings.set(false);
  wait(0.2, sec);
  setDrivePID(-15, 80);
  setTurnPID(-120);
  setDrivePID(-35, 200);
  wings.set(true);
  setDrivePID(40, 200);
  wings.set(false);
  setDrivePID(-10, 200);
  // // setDrivePID(-25, 80);
  // // setTurnPID(-2);
  // setDrivePID(-85, 80);
  // setTurnPID(70);
  // setDrivePID(55, 80);
  // setTurnPID(-25);
  // wings.set(true);
  // setDrivePID(-30, 80);

  // setTurnPID(0);
  // setDrivePID(20, 80);
  // setTurnPID(-45);

  // setTurnPID(0);
  // setDrivePID(0,0);
  // setTurnPID(0);
  // setDrivePID(0,0);
  // setTurnPID(0);
  // setDrivePID(0,0);
  // setTurnPID(0);
  // setDrivePID(0,0);
}

// void skill_Auton2()
// {
//   setDrivePID(-24, 100);
//   setTurnPID(-37);
//   setDrivePID(20, 80);
//   wait(2, sec);
//   setTurnPID(143);
//   setDrivePID(108, 600);
// }


/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/



void usercontrol(void) 
{
  // User control code here, inside the loop
  // imu.calibrate();
  // wait(2, sec);
  while (1) 
  {
    // if(Controller1.ButtonX.pressing())
    // {
    //   autonomous();
    // }
    //setTurnPID(90);


    set_Drive();
    intake_ButtonControl();
    catapult_ButtonControl();
    blocker_PneumaticControl();
    wings_PneumaticControl();
    balance_PneumaticControl();
    //pd_turn(90,100);
    wait(100, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(frontFieldAutonomousFourTriball);
  Competition.drivercontrol(usercontrol);
  //setDrivePID(-1500);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
