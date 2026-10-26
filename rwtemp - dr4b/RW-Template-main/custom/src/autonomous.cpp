#include "vex.h"
#include "utils.h"
#include "pid.h"
#include <ctime>
#include <cmath>
#include <thread>

#include "../include/autonomous.h"
#include "motor-control.h"

// IMPORTANT: Remember to add respective function declarations to custom/include/autonomous.h
// Call these functions from custom/include/user.cpp
// Format: returnType functionName() { code }

void exampleAuton() {

}

void exampleAuton2() {

}


void testauto(){
  
}


void rightauto(){
  
}

void leftauto(){
  
}

void rightrush(){
  
}

void leftrush(){
  
}

void awpauto(){
    lift.setStopping(hold);
    resetOdom();
    resetChassis();   
    driveTo(-14.5,3000);
    turnToAngle(90, 1000);
    lift.spinFor(forward, 90, degrees);
    wait(50, msec);
    driveTo(-20,3000);
    lift.spin(reverse, 12, volt);
    wait(1000, msec);
    claw.set(false);
    lift.stop();
    //lift.spinFor(reverse, 20, degrees);
    wait(50, msec);
    driveTo(20,3000);
    turnToAngle(180, 1000);
    driveTo(-20,1500);
    driveTo(5,3000);
    driveTo(-5,3000);
    driveTo(15,3000);
    turnToAngle(46,1000);
    driveTo(-26.3,3000, true, 3);
    wait(500,msec);
    claw.set(true);
    lift.spinFor(forward, 180, degrees);
    turnToAngle(160,1000);
    driveTo(-15,3000);
    lift.spin(reverse, 12, volt);
    wait(1000, msec);
    claw.set(false);
    lift.stop();
    driveTo(-5,3000);
    turnToAngle(225,1000);
}



void awpauto2(){
  
}




void skillsauto(){
  
}