#include "vex.h"
#include "motor-control.h"
#include "../custom/include/autonomous.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


// Modify autonomous, driver, or pre-auton code below

void runAutonomous() {
  int auton_selected = 5;
  switch(auton_selected) {
    case 1:
      rightauto();
      break;
    case 2:
      leftauto();
      break;  
    case 3:
      awpauto();
      break;
    case 4:
      skillsauto();
      break; 
    case 5:
      testauto();
      break;
    case 6:
      break;
    case 7:
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
int chassis_flag = 0;
int lever_toggle;





void togglebottom(){
  
  if (lever.position(degrees)>130){
    lever_toggle = 0;
    hood.set(false);
    controller_1.rumble(".");
    lever.spinToPosition(0,degrees,200,rpm,false);
  }
  else if (lever.position(degrees)<15){
    lever_toggle = 1;
    hood.set(true);
    controller_1.rumble("-");
    lever.spinToPosition(150,degrees,200,rpm,false);
  }
  else{
    hood.set(false);
    lever_toggle = 0;
    lever.spinToPosition(0,degrees,200,rpm,false);
  }
}

void liftswitch(){
  if (lifter.value()){
    lifter.set(false);
  }
  else{
    lifter.set(true);
  }
}

void wingswitch(){
  if (wing.value()){
    wing.set(false);
  }
  else{
    wing.set(true);
  }
}

void loadswitch(){
  if (loader.value()){
    loader.set(false);
  }
  else{
    loader.set(true);
  }
}

void runDriver() {
  std::string xposstr;
  std::string yposstr;
  lever_toggle = 0;
  stopChassis(coast);
  heading_correction = false;
  while (true) {
    // [-100, 100] for controller stick axis values)


    // Brain.Screen.print(xpos);
    // Brain.Screen.print(", ");
    // Brain.Screen.print(ypos);
    // Brain.Screen.print("            ");


    ch1 = controller_1.Axis1.value();
    ch2 = controller_1.Axis2.value();
    ch3 = controller_1.Axis3.value();
    ch4 = controller_1.Axis4.value();

    // true/false for controller button presses
    r1 = controller_1.ButtonR1.pressing();
    button_b = controller_1.ButtonB.pressing();
    button_a = controller_1.ButtonA.pressing();
    button_up_arrow = controller_1.ButtonUp.pressing();
    button_down_arrow = controller_1.ButtonDown.pressing();
    button_left_arrow = controller_1.ButtonLeft.pressing();
    button_right_arrow = controller_1.ButtonRight.pressing();

    // default tank drive or replace it with your preferred driver code here: 
    driveChassis((ch3+ch1) * 0.12, (ch3-ch1) * 0.12);

    



    if (r1 || lever_toggle>0){
      intake.spin(forward, 200, rpm);
    }
    else if (button_a){
      intake.spin(reverse, 200, rpm);
    }
    else{
      intake.stop();
    }

    if(button_down_arrow && button_b){
      runAutonomous();
    }
    
    controller_1.ButtonL1.pressed(togglebottom);
    controller_1.ButtonL2.pressed(liftswitch);
    controller_1.ButtonR2.pressed(wingswitch);
    controller_1.ButtonX.pressed(loadswitch);


    wait(10, msec);
    
    
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
  Brain.Screen.print(current_heading);
  
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
}