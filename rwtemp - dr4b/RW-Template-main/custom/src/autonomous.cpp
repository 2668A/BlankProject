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
  lift.spinFor(forward, 180, degrees, false);
    wait(500, msec);
    driveTo(-10,3000);
    //driveTo(1.7,3000);
    lift.spin(reverse, 10, volt);
    wait(500, msec);
    claw.set(false);
    lift.stop();
    wait(500, msec);
    driveTo(6.5,3000);
    turnToAngle(90,1000);
    driveTo(-7.2,3000, true, 3);
    
    claw.set(true);
    wait(500, msec);
    lift.spinFor(forward, 280, degrees, false);
    turnToAngle(-30,1000);
    // move To point (0,-10) for other goal ;-;
    // moveToPoint(0,-10,1,3000);
    driveTo(-8,3000, true, 5);
    lift.spin(reverse, 8, volt);
    wait(500, msec);
    claw.set(false);
    lift.stop();
    wait(500, msec);
    driveTo(8,3000, true, 5);
    turnToAngle(15,1000);
    driveTo(-5,3000, false);
    swingpower(-9,-3, -135,2000);
    turnToAngle(-135,500);
    // Toggle
    driveTo(5,3000, true);
    driveTo(-15,1000);
    driveTo(7,3000, true);
    wait(500, msec);
    driveTo(-9,1000, true);
    //wait(500, msec);
    driveTo(7,1000, true);
}


// void rightauto(){
//     //left 30 inches
//     //moveToPoint(30, 0, 1, 3000, true, 6);
  
// }

void leftauto(){
  lift.spinFor(forward, 180, degrees, false);
    wait(500, msec);
    driveTo(-10,3000);
    //driveTo(1.7,3000);
    lift.spin(reverse, 10, volt);
    wait(500, msec);
    claw.set(false);
    lift.stop();
    wait(500, msec);
    driveTo(6.5,3000);
    turnToAngle(90,1000);
    driveTo(-7.2,3000, true, 3);
    
    claw.set(true);
    wait(500, msec);
    lift.spinFor(forward, 280, degrees, false);
    turnToAngle(-30,1000);
    // move To point (0,-10) for other goal ;-;
    // moveToPoint(0,-10,1,3000);
    driveTo(-8,3000, true, 5);
    lift.spin(reverse, 8, volt);
    wait(500, msec);
    claw.set(false);
    lift.stop();
    wait(500, msec);
    
    driveTo(4,2000,true,6);
    turnToAngle(-45,2000);

    driveTo(24,3000, true, 5);
    turnToAngle(0,2000);
    driveTo(-34,3000, true, 4);
    driveTo(-4,3000, true, 2);
    claw.set(true);
    // Toggle
    lift.spin(forward,8,volt);
    wait(700,msec);
    lift.stop();
    driveTo(-4,2000,true,6);
    turnToAngle(-135,2000);
    driveTo(-24,3000,true,6);
    lift.spin(reverse,4,volt);
    wait(500,msec);
    lift.stop();
    claw.set(false);
    turnToAngle(-45,2000);
    driveTo(6,3000,true,6);
    turnToAngle(-100,1500);
    driveTo(-40,3000,true,8);
    turnToAngle(-135,2000);
    driveTo(-12,3000,true);
    driveTo(8,3000,true,6);
    driveTo(-12,3000,true);
    //wait(500, msec);
    driveTo(40,1000, true);
}

void rightrush(){
  
}

void leftrush(){
  
}

void awpauto(){

    resetOdom();
    resetChassis();
    driveTo(-14.5,3000);
    turnToAngle(-90, 1000);
    lift.spinFor(forward, 140, degrees);
    wait(50, msec);
    driveTo(-17,3000);
    // Scoring first pin
    driveTo(1.7,3000);
    lift.spin(reverse, 12, volt);
    wait(1100, msec);
    claw.set(false); 
    lift.stop();
    wait(500, msec);
    //
    wait(50, msec);
    driveTo(20,3000,true, 6);
    turnToAngle(180, 1000);
    // Toggle
    driveTo(-17,1200);
    driveTo(7,3000);
    driveTo(-9,3000);
    // Picking up second pin
    driveTo(15,3000);
    // turnToAngle(48,1000);
    // driveTo(-26.7,3000, true, 3);
    // // moveToPoint(-26.3,15,-1,3000,true,5);
    // wait(500,msec);
    // claw.set(true);
    // lift.spinFor(forward, 240, degrees);
    // turnToAngle(160,1000);
    // driveTo(-15,2000);
    // // Scoring Second Pin
    // driveTo(1.5,3000);
    // lift.spin(reverse, 8, volt);
    // wait(1000, msec);
    // claw.set(false);
    // lift.stop();
    // // Picking up third pin
    // driveTo(5,3000);
    // turnToAngle(225,1000);
    // //side side then forwards backwards (from starting position)
    // moveToPoint(11,-20,-1,3000,true,5);
    // swingpower(-12,3,180,3000);
    // //driveTo(-24-15,3000);
    // // turnToAngle(180,1000);
    // // driveTo(-4,3000);
    // claw.set(true);
    // driveTo(4,3000);
    // lift.spin(forward, 8, volt);
    // turnToAngle(0,1000);
    // // // Scoring Third Pin
    // driveTo(-4,3000);
    // lift.spin(reverse, 8, volt);
    // claw.set(false);
    // lift.stop();
}



void rightauto(){
    lift.spinFor(forward, 180, degrees, false);
    wait(500, msec);
    driveTo(-10,3000);
    //driveTo(1.7,3000);
    lift.spin(reverse, 10, volt);
    wait(500, msec);
    claw.set(false);
    lift.stop();
    wait(500, msec);
    driveTo(6.5,3000);
    turnToAngle(85,1000);
    driveTo(-7,3000, true, 2.5);
    
    claw.set(true);
    wait(500, msec);
    lift.spinFor(forward, 280, degrees, false);
    turnToAngle(-25,1000);
    // move To point (0,-10) for other goal ;-;
    // moveToPoint(0,-10,1,3000);
    driveTo(-8.2,3000, true, 5);
    lift.spin(reverse, 8, volt);
    wait(500, msec);
    claw.set(false);
    
    wait(500, msec);
    lift.stop();
    driveTo(8,3000, true, 5);
    turnToAngle(15,1000);
    driveTo(-5,3000, false);
    swingpower(-9,-3, -135,2000);
    turnToAngle(-135,500);
    // Toggle
    driveTo(5,3000, true);
    driveTo(-15,1000);
    driveTo(7,3000, true);
    wait(500, msec);
    driveTo(-9,1000, true);
    //wait(500, msec);
    driveTo(7,1000, true);

    // thats about 15 seconds.


    // turnToAngle(-45,1000);
    // driveTo(-10,3000, true, 5);
    // swingpower(-12,3,180,3000);
    // claw.set(true);
    // driveTo(4,3000);
    // lift.spin(forward, 8, volt);
    // turnToAngle(0,1000);
    // // Scoring Third Pin
    // driveTo(-4,3000);
    // lift.spin(reverse, 8, volt);
    // claw.set(false);
    // lift.stop();
}




void skillsauto(){
    resetOdom();
    resetChassis();
    driveTo(-14,3000);
    turnToAngle(-90, 1000);
    lift.spinFor(forward, 150, degrees);
    wait(50, msec);
    driveTo(-17,3000);
    // Scoring first pin
    driveTo(3,3000);
    lift.spin(reverse, 12, volt);
    wait(1100, msec);
    claw.set(false);
    lift.stop();
    wait(500, msec);
    //wait(50, msec);
    driveTo(26.25,3000,true, 6);
    turnToAngle(180, 1000);
    claw.set(true);
    driveTo(-15, 1000, true,5);
    lift.spinFor(forward, 160, degrees, true);
    //wait(500, msec);
    driveTo(14,1000);
    lift.spinFor(forward, 110, degrees, false);
    wait(50, msec);
    turnToAngle(-90, 1000);
    driveTo(-25.25,1000,true,8);
    lift.spin(reverse, 12, volt);
    wait(500, msec);
    claw.set(false);
    lift.stop();
    wait(500, msec);

    //Iteration 2
    driveTo(24.75,3000,true, 6);
    turnToAngle(180, 1000);
    claw.set(true);
    driveTo(-15, 1000, true, 5);
    lift.spinFor(forward, 160, degrees, true);
    //wait(500, msec);
    driveTo(14,1000);
    lift.spinFor(forward, 210, degrees, false);
    wait(50, msec);
    turnToAngle(-90, 1000);
    driveTo(-25.25,1000,true,8);
    lift.spin(reverse, 12, volt);
    wait(500, msec);
    claw.set(false);
    lift.stop();
    wait(500, msec);
    
    //Iteration 3
    driveTo(24.75,3000,true, 6);
    turnToAngle(180, 1000);
    claw.set(true);
    driveTo(-15, 1000, true,5);
    lift.spinFor(forward, 160, degrees, true);
    //wait(500, msec);
    driveTo(14,1000);
    lift.spinFor(forward, 310, degrees, false);
    wait(50, msec);
    turnToAngle(-90, 1000);
    driveTo(-25.25,1000,true,8);
    lift.spin(reverse, 12, volt);
    wait(500, msec);
    claw.set(false);
    wait(200, msec);
    lift.stop();
    wait(500, msec);

    //Iteration 4
    driveTo(24.75,3000,true, 6);
    turnToAngle(180, 1000);
    claw.set(true);
    driveTo(-15, 1000, true,5);
    lift.spinFor(forward, 160, degrees, true);
    //wait(500, msec);
    driveTo(14,1000);
    lift.spinFor(forward, 410, degrees, false);
    wait(50, msec);
    turnToAngle(-90, 1000);
    driveTo(-25.25,1000,true,8);
    lift.spin(reverse, 12, volt);
    wait(500, msec);
    claw.set(false);
    wait(300, msec);
    lift.stop();
    wait(500, msec);

    driveTo(8,3000, true, 5);
    turnToAngle(15,1000);
    driveTo(-5,3000, false);
    swingpower(-9,-3, -135,2000);
    turnToAngle(-135,500);
    // Toggle
    driveTo(5,3000, true);
    driveTo(-15,1000);
    driveTo(7,3000, true);
    wait(500, msec);
    driveTo(-9,1000, true);
    //wait(500, msec);
    driveTo(20,1000, true);
    
}