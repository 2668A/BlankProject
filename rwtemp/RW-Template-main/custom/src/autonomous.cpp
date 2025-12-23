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
  // Use this for tuning linear and turn pid
  driveTo(60, 3000);
  turnToAngle(90, 2000);
  turnToAngle(135, 2000);
  turnToAngle(150, 2000);
  turnToAngle(160, 2000);
  turnToAngle(165, 2000);
  turnToAngle(0, 2000);
  driveTo(-60, 3000);
}

void exampleAuton2() {
  moveToPoint(24, 24, 1, 2000, false);
  moveToPoint(48, 48, 1, 2000, true);
  moveToPoint(24, 24, -1, 2000, true);
  moveToPoint(0, 0, 1, 2000, true);
  correct_angle = 0;
  driveTo(24, 2000, false, 8);
  turnToAngle(90, 800, false);
  turnToAngle(180, 800, true);
}


void testauto(){
  resetChassis();
  stopChassis(brake);
  // driveTo(24,3000);
  // turnToAngle(90,3000);
  // driveTo(24,3000);
  // turnToAngle(0,3000);
  // driveTo(-24,3000);
  // turnToAngle(90,3000);
  // driveTo(-24,3000);
  moveToPoint(24,24,1,4000);
  turnToAngle(0,1000);
  // moveToPoint(0,0,1,5000,false);
  // turnToAngle(0,1000);
}

void rightauto(){
  resetChassis();
  resetOdom();
  stopChassis(hold);
  wing.set(true);
  lifter.set(true);
  intake.spin(forward,600,rpm);
  moveToPoint(0,10,1,3000,false,6);
  moveToPoint(12,28,1,3000,false,5);
  moveToPoint(30,40,1,3000,true,5);
  wait(500,msec);
  moveToPoint(12,23,-1,3000);
  wait(100,msec);
  turnToAngle(-45,3000);
  driveTo(15,1000);
  intake.spin(reverse,150,rpm);
  wait(500,msec);
  intake.spin(forward,600,rpm);
  moveToPoint(37,0,-1,3000);
  wait(100,msec);
  loader.set(true);
  turnToAngle(180,3000);
  driveTo(-24,1000,true,6);
  hood.set(true);
  lever.spinToPosition(145,degrees,200,rpm,true);
  wait(800,msec);
  lever.spinToPosition(0,degrees,200,rpm,true);
  moveToPoint(34,-11.5,1,3000,true,6);
  hood.set(false);
  wait(750,msec);
  moveToPoint(34,16,-1,3000,true,6);
  hood.set(true);
  lever.spinToPosition(145,degrees,200,rpm,true);
  wait(800,msec);
  lever.spinToPosition(0,degrees,200,rpm,false);




}



void leftauto(){
  resetChassis();
  resetOdom();
  stopChassis(hold);
  wing.set(true);
  lifter.set(true);
  intake.spin(forward,600,rpm);
  moveToPoint(-0,10,1,3000,false,6);
  moveToPoint(-12,28,1,3000,false,5);
  moveToPoint(-30,40,1,3000,true,5);
  wait(500,msec);
  moveToPoint(-12,23,-1,3000);
  wait(100,msec);
}


void awpauto(){};
void skillsauto(){};