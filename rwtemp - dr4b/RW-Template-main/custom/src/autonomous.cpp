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
    //left 30 inches
    //moveToPoint(30, 0, 1, 3000, true, 6);
  
}

void leftauto(){
  
}

void rightrush(){
  
}

void leftrush(){
  
}

void awpauto(){
    //lift.setStopping(hold);
    
    resetOdom();
    resetChassis();
    driveTo(-14.5,3000);
    turnToAngle(90, 1000);
    //lift.spinFor(forward, 90, degrees);
    wait(50, msec);
    driveTo(-17,3000);
    //scoring
    driveTo(1.5,3000);
    //lift.spin(reverse, 12, volt);
    wait(1100, msec);
    claw.set(false);
    //lift.stop();
    wait(500, msec);
    //
    wait(50, msec);
    driveTo(20,3000,true, 6);
    turnToAngle(180, 1000);
    //toggle
    driveTo(-17,1200);
    driveTo(5,3000);
    driveTo(-6,3000);
    //second pin
    driveTo(15,3000);
    turnToAngle(48,1000);
    driveTo(-27,3000, true, 3);
    // moveToPoint(-26.3,15,-1,3000,true,5);
    wait(500,msec);
    claw.set(true);
    //lift.spinFor(forward, 240, degrees);
    turnToAngle(160,1000);
    driveTo(-15,3000);
    driveTo(1.5,3000);
    //lift.spin(reverse, 8, volt);
    wait(1000, msec);
    claw.set(false);
    lift.stop();
    driveTo(5,3000);
    turnToAngle(225,1000);
    //side side then forwards backwards (from starting position)
    moveToPoint(10,-24,-1,3000,true,5);
    //driveTo(-24-15,3000);
    turnToAngle(180,1000);
}



void awpauto2(){
  
}




void skillsauto(){
  
}