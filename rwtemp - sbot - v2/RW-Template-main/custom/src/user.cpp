#include "vex.h"
#include "motor-control.h"
#include "../custom/include/autonomous.h"
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>



// Auton Runner 

int auton_selected = 1;

void runAutonomous() {
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



// Controller Variables

int ch1, ch2, ch3, ch4;
bool l1, l2, r1, r2;
bool button_a, button_b, button_x, button_y;
bool button_up_arrow, button_down_arrow, button_left_arrow, button_right_arrow;




//misc.

int chassis_flag = 0;




// Controller Auton Selector

std::vector<std::string> autolist = {"right_auto","left_auto","awp_auto","skills_auto","test_auto"};

void printautos(){
  controller_1.Screen.setCursor(1,1);
  controller_1.Screen.clearLine(1);
  controller_1.Screen.print("Auton: %d - ",auton_selected);
  switch(auton_selected) {
    case 1:
      controller_1.Screen.print(autolist.at(0));
      break;
    case 2:
      controller_1.Screen.print(autolist.at(1));
      break;  
    case 3:
      controller_1.Screen.print(autolist.at(2));
      break;
    case 4:
      controller_1.Screen.print(autolist.at(3));
      break; 
    case 5:
      controller_1.Screen.print(autolist.at(4));
      break;
    case 6:
      controller_1.Screen.print("");
      break;
    case 7:
      controller_1.Screen.print("");
      break;
    case 8:
      controller_1.Screen.print("");
      break;
    case 9:
      controller_1.Screen.print("");
      break;
    default:
      controller_1.Screen.print("");
      break;
  }
}



// Controller Toggle Functions

void hoodtoggle(){
  if(hood.value() == true){
    hood.set(false);
  }
  else{
    hood.set(true);
  }
}

void wingtoggle(){
  if(wing.value() == true){
    wing.set(false);
  }
  else{
    wing.set(true);
  }
}

void loadertoggle(){
  if(loader.value() == true){
    loader.set(false);
  }
  else{
    loader.set(true);
  }
}



// Driver Code

void runDriver() {

  resetChassis();
  std::string xposstr;
  std::string yposstr;
  stopChassis(coast);
  heading_correction = false;

  double drivesens = 1.1;
  double turnsens = 1.2;
      
  while (true) {

    // Debug Prints

    outtake.setStopping(hold);      
    Brain.Screen.printAt(2, 30, "x: %.2f", x_pos);
    Brain.Screen.printAt(2, 50, "y: %.2f", y_pos);
    Brain.Screen.printAt(2, 70, "heading: %.2f", normalizeTarget(getInertialHeading()));



    // Button Recorders

    ch1 = controller_1.Axis1.value();
    ch2 = controller_1.Axis2.value();
    ch3 = controller_1.Axis3.value();
    ch4 = controller_1.Axis4.value();

    r1 = controller_1.ButtonR1.pressing();
    l1 = controller_1.ButtonL1.pressing(); 
    l2 = controller_1.ButtonL2.pressing(); 
    button_b = controller_1.ButtonB.pressing();
    button_a = controller_1.ButtonA.pressing();
    button_y = controller_1.ButtonY.pressing();
    button_up_arrow = controller_1.ButtonUp.pressing();
    button_down_arrow = controller_1.ButtonDown.pressing();
    button_left_arrow = controller_1.ButtonLeft.pressing();
    button_right_arrow = controller_1.ButtonRight.pressing();



    // Driving Controls

    stopChassis(brake);
    driveChassis((ch3*drivesens+ch1*turnsens)*0.125, (ch3*drivesens-ch1*turnsens)*0.125);



    // Auton Selector
    
    if (button_down_arrow){
      intake.stop();
      outtake.stop();
      if(button_right_arrow){
        auton_selected++;
        controller_1.rumble(".");
        printautos();
      }
      else if(button_left_arrow){
        auton_selected--;
        controller_1.rumble(".");
        printautos();
      }
      printautos();
      wait(100,msec);
    }

    if(button_down_arrow && button_b){
      runAutonomous();
    }



    // Subsystem Controls

    controller_1.ButtonR2.pressed(wingtoggle);
    controller_1.ButtonX.pressed(loadertoggle);
    controller_1.ButtonA.pressed(hoodtoggle);

    if (l2){
      intake.spin(reverse,600,rpm);
    }
    else if (l1 || r1){
      intake.spin(forward,600,rpm);
    }
    else{
      intake.stop();
    }

    if(l1){
      outtake.spin(forward,600,rpm);
    }
    else if (l2){
      outtake.spin(reverse,600,rpm);
    }
    else{
      outtake.stop();
    }

    if (button_up_arrow){
      lifter.set(true);
      wait(150,msec);
    }



    // Loop Delay

    wait(10, msec);
    
    
  }
}



// Utils for setup

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
  resetChassis();
  resetOdom();
}