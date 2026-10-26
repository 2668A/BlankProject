#include "vex.h"
#include "motor-control.h"
#include "../custom/include/autonomous.h"
#include <cstdio>
#include <cstdlib>
#include <string>

// template<typename T>
// auto to_string(T to_convert) -> std::string {
//   std::ostringstream converter;
//   converter << to_convert;
//   return converter.str();
// }

// Modify autonomous, driver, or pre-auton code below
int auton_selected = 1;


void runAutonomous() {
  switch(auton_selected) {
    case 1:
      awpauto();
      break;
    case 2:
      rightauto();
      break;  
    case 3:
      leftauto();
      break;
    case 4:
      skillsauto();
      break; 
    case 5:
      rightrush();
      break;
    case 6:
      leftrush();
      break;
    case 7:
      testauto();
      break;
    case 8:
      break;
    case 9:
      break;
  }
}

// controller_1 input variables (snake_case)
int ch1, ch2, ch3, ch4;
bool l1, l2, r1, r2;
bool button_a, button_b, button_x, button_y;
bool button_up_arrow, button_down_arrow, button_left_arrow, button_right_arrow;


void autonswitch(){
  double screenx = Brain.Screen.xPosition();
  double screeny = Brain.Screen.yPosition();
  if (screenx>2 && screenx<92 && screeny>90 && screeny<160){
    auton_selected-=1;
  }
  else if (screenx>97 && screenx<187 && screeny>90 && screeny<160){
    auton_selected+=1;
  }
  screenx=0;
  screeny=0;
}


void printautos(){
  controller_1.Screen.setCursor(1,1);
  controller_1.Screen.print("Auton: %d - ",auton_selected);
  switch(auton_selected) {
    case 1:
      controller_1.Screen.print("awp auto        ");
      break;
    case 2:
      controller_1.Screen.print("right auto            ");
      break;  
    case 3:
      controller_1.Screen.print("left auto          ");
      break;
    case 4:
      controller_1.Screen.print("skills auto             ");
      break; 
    case 5:
      controller_1.Screen.print("right rush             ");
      break;
    case 6:
      controller_1.Screen.print("left rush               ");
      break;
    case 7:
      controller_1.Screen.print("test auto                  ");
      break;
    case 8:
      controller_1.Screen.print("                     ");
      break;
    case 9:
      controller_1.Screen.print("                     ");
      break;
    default:
      controller_1.Screen.print("                     ");
      break;
  }
}

void traytoggle(){
  if(tray.value() == true){
    tray.set(false);
  }
  else{
    tray.set(true);
  }
}

void clawtoggle(){
  if(claw.value() == true){
    claw.set(false);
  }
  else{
    claw.set(true);
  }
}

void tiltertoggle(){
  if(tilter.value() == true){
    tilter.set(false);
  }
  else{
    tilter.set(true);
  }
}


bool autonrunning = false;
bool disabledriver = false;


void runDriver() {
  resetChassis();
  std::string xposstr;
  std::string yposstr;
  stopChassis(brake);
  heading_correction = false;

  
  double drivesens = 1.0;
  double turnsens = 0.85;

  double xcompute;
  double ycompute;

  intake.setStopping(brake);
  lift.setStopping(hold);
      
  while (true) {

    if(left_chassis.temperature(celsius)>60){
      controller_1.rumble("-");
    }
    if(right_chassis.temperature(celsius)>60){
      controller_1.rumble("-");
    }

     
    Brain.Screen.printAt(2, 30, "x: %.2f", x_pos);
    Brain.Screen.printAt(2, 50, "y: %.2f", y_pos);
    Brain.Screen.printAt(2, 70, "heading: %.2f", normalizeTarget(getInertialHeading()));
    // //Brain.Screen.printAt(2+90+5+90+15, 100, "Auton: %d       ", auton_selected );
    //printautos();
    
    //Brain.Screen.print(std::to_string(x_pos));
    //Brain.Screen.print(to_string(x_pos));
    // Brain.Screen.released(autonswitch);
    

    ch1 = controller_1.Axis1.value();
    ch2 = controller_1.Axis2.value();
    ch3 = controller_1.Axis3.value();
    ch4 = controller_1.Axis4.value();

    // true/false for controller button presses
    r1 = controller_1.ButtonR1.pressing();
    r2 = controller_1.ButtonR2.pressing();
    l1 = controller_1.ButtonL1.pressing(); 
    l2 = controller_1.ButtonL2.pressing(); 
    button_b = controller_1.ButtonB.pressing();
    button_a = controller_1.ButtonA.pressing();
    button_y = controller_1.ButtonY.pressing();
    button_up_arrow = controller_1.ButtonUp.pressing();
    button_down_arrow = controller_1.ButtonDown.pressing();
    button_left_arrow = controller_1.ButtonLeft.pressing();
    button_right_arrow = controller_1.ButtonRight.pressing();

    xcompute = ch1*turnsens;

    if (disabledriver == false){
      driveChassis((ch3*drivesens+xcompute)*0.125, (ch3*drivesens-xcompute)*0.125);
    }

    
    if(button_right_arrow){
      auton_selected++;
      controller_1.rumble(".");
      printautos();
      wait(100,msec);
    }
    else if(button_left_arrow){
      auton_selected--;
      controller_1.rumble(".");
      printautos();
      wait(100,msec);
    }

    controller_1.ButtonX.pressed(traytoggle);
    controller_1.ButtonB.pressed(tiltertoggle);
    controller_1.ButtonR1.pressed(clawtoggle);


    if (button_down_arrow && button_a){
      //runPreAutonomous();
      runAutonomous();
    }
    if (l1){
      lift.spin(forward,80,rpm);
    }
    else if (l2){
      lift.spin(reverse,70,rpm);
    }
    else{
      lift.stop();
    }

    if(r2){
      intake.spin(forward,80,percent);
    }
    else{
      intake.stop();
    }

    wait(5, msec);
    
    
  }
}

void runPreAutonomous() {
    // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  
  // Calibrate inertial sensor
  inertial_sensor.calibrate();

  // Wait for the Inertial Sensor to calibrate
  while (inertial_sensor.isCalibrating()) {
    wait(10, msec);
  }
  controller_1.rumble(".");

  double current_heading = inertial_sensor.heading();
  
  // odom tracking
  resetChassis();
  if(using_horizontal_tracker && using_vertical_tracker) {
    thread odom = thread(trackXYOdomWheel);
  } else if (using_horizontal_tracker) {
    thread odom = thread(trackXOdomWheel);
  } else if (using_vertical_tracker) {
    thread odom = thread(trackYOdomWheel);
  } else {
    thread odom = thread(trackNoOdomWheel);
  }
  tray.set(true);
  claw.set(true);
  tilter.set(false);
  resetChassis();
  resetOdom();
}