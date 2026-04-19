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
int chassis_flag = 0;
int lever_toggle;
int leverspeed;
int intake_toggle = 0;





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

int spinspeed = 600;
int colorsort = 1; //1 is red, -1 is blue
bool parry = false;

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
    blocker.set(false);
  }
}

void intaketoggle(){
  if(intake_toggle==0){
    intake_toggle=2;
  }
  else{
    intake_toggle=0;
  }
}

void printsettings(){
  controller_1.Screen.clearScreen();
  controller_1.Screen.setCursor(1,1);
  controller_1.Screen.print("spin: ");
  switch(spinspeed){
    case 60:
      controller_1.Screen.print("MID     ");
      break;
    case 600:
      controller_1.Screen.print("TOP     ");
      break;
  }
  controller_1.Screen.setCursor(2,1);
  controller_1.Screen.print("clr: ");
  switch(colorsort){
    case 1:
      controller_1.Screen.print( "RED    ");
      break;
    case -1:
      controller_1.Screen.print("BLUE    ");
      break;
  }
  controller_1.Screen.setCursor(3,1);
  controller_1.Screen.print("parry: ");
  switch(parry){
    case 0:
      controller_1.Screen.print( "OFF   ");
      break;
    case 1:
      controller_1.Screen.print("ON     ");
      break;
  }
}

bool setchanged = false;

void speedtoggle(){
  if (spinspeed==600){
    spinspeed=60;
    controller_1.rumble(".");
  }
  else if(spinspeed==60){
    spinspeed=600;
    controller_1.rumble("..");
  }
  setchanged = true;
}

bool autonrunning = false;

void colortoggle(){
  if(!autonrunning){
    if(colorsort==1){
      colorsort=-1;
      controller_1.rumble(".");
    }
    else if (colorsort==-1){
      colorsort=1;
      controller_1.rumble("..");
    }
    setchanged = true;
  }
} 

void parrytoggle(){
  if(!autonrunning){
    if(parry){
      parry=false;
    }
    else{
      parry=true;
      controller_1.rumble(".");
    }
    setchanged = true;
  }
}

double colorvalue;
int resultcolor;
bool disabledriver = false;

void autoparry(){
  while (true){
    wingsensor.setLightPower(75,percent);
    colorvalue = wingsensor.value();
    resultcolor = 0;
    if(colorvalue<20.0){
      resultcolor=1;
    }
    else if ((colorvalue>70.0) && (colorvalue<300)){
      resultcolor=-1;
    }
    else{
      resultcolor=0;
    }
    
    if(parry){
      if((resultcolor!=0) && (resultcolor==colorsort)){
        disabledriver = true;

        // wing.set(false);
        // driveChassis(10,10);
        // wait(200,msec);
        // driveChassis(4,4);
        // wait(50,msec);
        // driveChassis(-2,-2);
        // wait(50,msec);
        // driveChassis(-4,-4);
        // wait(50,msec);
        // driveChassis(-6,-6);
        // wing.set(true);
        // wait(500,msec);
        // driveChassis(0,0);

        wing.set(false);
        driveChassis(0,12);
        wait(200,msec);
        driveChassis(0,0);
        wait(50,msec);
        driveChassis(0,-6);
        wait(200,msec);
        driveChassis(0,0);
        


        resultcolor=0;
        printsettings();
        disabledriver=false;
      }
    }

    wait(5,msec);
  }

}


void runDriver() {
  resetChassis();
  std::string xposstr;
  std::string yposstr;
  stopChassis(brake);
  heading_correction = false;
  // Brain.Screen.drawRectangle(2, 90, 90, 70);
  // Brain.Screen.drawRectangle(2+90+5, 90, 90, 70);

  
  double drivesens = 1.0;
  double turnsens = 0.85;

  int outspeed=600;

  wingsensor.integrationTime(20);

  thread autoparrythread = thread(autoparry);
  double xcompute;
  double ycompute;
      
  while (true) {

    wingsensor.integrationTime(20);
    if(left_chassis.temperature(celsius)>60){
      controller_1.rumble("-");
    }
    if(right_chassis.temperature(celsius)>60){
      controller_1.rumble("-");
    }

    outtake.setStopping(coast);
    // [-100, 100] for controller stick axis values)
     
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
    l1 = controller_1.ButtonL1.pressing(); 
    l2 = controller_1.ButtonL2.pressing(); 
    button_b = controller_1.ButtonB.pressing();
    button_a = controller_1.ButtonA.pressing();
    button_y = controller_1.ButtonY.pressing();
    button_up_arrow = controller_1.ButtonUp.pressing();
    button_down_arrow = controller_1.ButtonDown.pressing();
    button_left_arrow = controller_1.ButtonLeft.pressing();
    button_right_arrow = controller_1.ButtonRight.pressing();

    // default tank drive or replace it with your preferred driver code here:
    // if (ch1>=5){
    //   xcompute = turnsens * 118.3 / ( 1.0 + pow( 2.718 , -0.062*ch1 + 4.22 ) );
    // }
    // else if (ch1<=-5){
    //   xcompute = turnsens * -118.3 / ( 1.0 + pow( 2.718 , -0.062*-ch1 + 4.22 ) );
    // }
    // else{
    //   xcompute = 0;
    // }
    xcompute = ch1*turnsens;

    if(controller_1.ButtonDown.pressing()){
      controller_1.Screen.setCursor(1,1);
      controller_1.Screen.print(xcompute);
    }


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

    controller_1.ButtonR2.pressed(wingtoggle);
    controller_1.ButtonL1.pressed(intaketoggle);
    controller_1.ButtonX.pressed(loadertoggle);
    controller_1.ButtonA.pressed(hoodtoggle);
    //controller_1.ButtonB.pressed(colortoggle);
    //controller_1.ButtonDown.pressed(parrytoggle);
    //controller_1.ButtonY.pressed(speedtoggle);



    if (l2){
      intake.spin(reverse,spinspeed,rpm);
    }
    else if (l1 || r1){
      intake.spin(forward,600,rpm);
    }
    else{
      intake.stop();
    }

    if(l1){
      outtake.spin(forward,spinspeed,rpm);
    }
    else if (l2){
      outtake.spin(reverse,spinspeed,rpm);
    }
    else{
      outtake.stop();
    }

    // wingsensor.setLightPower(75,percent);
    // colorvalue = wingsensor.value();
    // resultcolor = 0;
    // if(colorvalue<30.0){
    //   resultcolor=1;
    // }
    // else if (colorvalue>70.0){
    //   resultcolor=-1;
    // }
    // else{
    //   resultcolor=0;
    // }
    
    // if(parry){
    //   if((resultcolor!=0) && (resultcolor!=colorsort)){
    //     wing.set(false);
    //     driveChassis(10,10);
    //     wait(200,msec);
    //     driveChassis(4,4);
    //     wait(50,msec);
    //     driveChassis(-2,-2);
    //     wait(50,msec);
    //     driveChassis(-4,-4);
    //     wait(50,msec);
    //     driveChassis(-6,-6);
    //     wing.set(true);
    //     wait(500,msec);
    //     driveChassis(0,0);
    //     resultcolor=0;
    //     parry=false;
    //     printsettings();
    //   }
    // 


    if(button_down_arrow && button_b){
      parry=false;
      autonrunning=true;
      runAutonomous();
      autonrunning=false;  
    }
    else if(button_b){
      spinspeed=600;
      setchanged=true;
    }
    else if(button_down_arrow && !button_b){
      if(blocker.value()){
        blocker.set(false);
      }
      else{
        loader.set(false);
        blocker.set(true);
      }
      wait(100,msec);
    }

    if(button_y){
      spinspeed=60;
      setchanged=true;
    }

    // if(button_y){
    //   outspeed=120;
    // }
    // else if(button_b){       
    //   outspeed=600;
    // }
    

    if (button_up_arrow){
      lifter.set(true);
      wait(150,msec);
    }

    if(setchanged){
      printsettings();
      setchanged=false;
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
  resetChassis();
  resetOdom();
}