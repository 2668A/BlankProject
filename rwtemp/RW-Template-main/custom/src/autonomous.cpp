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
  driveUntil(225,3000,front,true,8);
}

void rightauto(){
  resetChassis();
  resetOdom();
  stopChassis(hold);
  lever.setPosition(125,degrees);
  lever.spinToPosition(0,degrees,200,rpm,false);
  wing.set(true);
  lifter.set(true);
  intake.spin(forward,600,rpm);
  moveToPoint(-1,10,1,3000,false,5);
  moveToPoint(12,28,1,3000,false,5);
  moveToPoint(30,39,1,1000,true,5);
  wait(500,msec);
  moveToPoint(15,22,-1,1500);
  wait(100,msec);
  turnToAngle(-46,1000);
  driveTo(15.5,1000);
  intake.spin(reverse,100,rpm);
  wait(800,msec);
  intake.spin(forward,600,rpm);
  wait(100,msec);
  moveToPoint(36.5,0,-1,3000);     //37
  wait(100,msec);
  loader.set(true);
  turnToAngle(180,3000);
  driveUntil(200,1000,front,true,5,1.0);
  hood.set(false);
  wait(700,msec);
  turnToAngle(180,1000);
  driveTo(-27,1000,true,10);
  loader.set(false);
  hood.set(true);
  lever.spinToPosition(145,degrees,50,rpm,true);
  lever.spinToPosition(0,degrees,200,rpm,false);
  turnToAngle(140,1000);
  
  driveTo(15,1000,true,8);
  turnToAngle(180,1000);
  wing.set(false);
  wait(500,msec);
  driveTo(-28,1000,true,10);





}



void leftauto(){
  resetChassis();
  resetOdom();
  stopChassis(hold);
  lever.setPosition(125,degrees);
  lever.spinToPosition(0,degrees,200,rpm,false);
  wing.set(true);
  
  intake.spin(forward,600,rpm);
  moveToPoint(1.5,10,1,3000,false,6);
  moveToPoint(-12,28,1,3000,false,5);
  moveToPoint(-30,39,1,3000,true,5);
  moveToPoint(-12,24,-1,3000);
  wait(100,msec);
  turnToAngle(-135,3000,true);
  lifter.set(false);
  driveTo(-11,3000,true);
  hood.set(true);
  lever.spinToPosition(110,degrees,30,rpm,true);
  lever.spinToPosition(0,degrees,200,rpm,false);
  loader.set(true);
  hood.set(false);
  lifter.set(true);
  moveToPoint(-36,0,1,3000,false);  //35
  turnToAngle(180,3000,true);
  driveTo(10,3000,true,8);
  driveUntil(190,2000,front,true,5);
  wait(500,msec);
  moveToPoint(-38,16,-1,3000,true);
  hood.set(true);
  loader.set(false);
  lever.spinToPosition(150,degrees,80,rpm,true);
  lever.spinToPosition(0,degrees,200,rpm,false);
  turnToAngle(140,3000);
  driveTo(13,3000,true);
  turnToAngle(180,3000);
  wing.set(false);
  driveTo(-28,3000,true,10);
  wing.set(true);
}


void awpauto(){
  resetChassis();
  resetOdom();
  stopChassis(hold);
  lever.setPosition(125,degrees);
  lever.spinToPosition(0,degrees,200,rpm,false);
  wing.set(true);
  lifter.set(true);
  loader.set(true);
  driveTo(36.3,2000,true,10);
  turnToAngle(90,2000);
  intake.spin(forward,600,rpm);
  driveUntil(200,3000,front,true,5);
  wait(700,msec);
  driveTo(-29,1700,true,8);
  hood.set(true);
  loader.set(false);
  lever.spinToPosition(150,degrees,200,rpm,true);
  intake.spin(reverse,600,rpm);
  lever.spinToPosition(0,degrees,200,rpm,false);
  intake.spin(forward,600,rpm);
  
  
  turnToAngle(-180,2000);
  hood.set(false);
  moveToPoint(-25,8,1,3000,false,8);
  lifter.set(false);
  moveToPoint(-24,-38,1,3000,false,8);
  
  turnToAngle(135,1000);
  driveTo(-15.5,2000,true,8);
  hood.set(true);
  lever.spinToPosition(145,degrees,30,rpm,true);
  lever.spinToPosition(0,degrees,200,rpm,false);
  hood.set(false);
  lifter.set(true);
  moveToPoint(0,-60.5,1,3000);
  loader.set(true);
  turnToAngle(90,1000);
  driveUntil(200,3000,front,true,5);
  wait(700,msec);
  //driveTo(-28,3000,true,8);
  moveToPoint(-16,-64,-1,1500,true,9);
  hood.set(true);
  loader.set(false);
  lever.spinToPosition(135,degrees,200,rpm,true);
  intake.spin(reverse,600,rpm);
  lever.spinToPosition(0,degrees,200,rpm,true);
  intake.spin(forward,600,rpm);





};



void skillsauto(){
  resetChassis();
  resetOdom();
  stopChassis(hold);
  lever.setPosition(125,degrees);
  lever.spinToPosition(0,degrees,200,rpm,false);
  wing.set(true);
  lifter.set(true);
  loader.set(true);
  driveTo(37,2000,true,10);
  turnToAngle(92,2000);
  intake.spin(forward,600,rpm);
  driveUntil(180,3000,front,true,5);
  wait(2500,msec);
  driveTo(-12,3000,true,8);
  turnToAngle(180,3000);
  driveUntil(160,3000,rear,true,6,-1.0);
  turnToAngle(-90,3000);
  driveTo(80,3000);
  turnToAngle(-180,3000);
  driveTo(11,1000,true);
  turnToAngle(-90,3000);
  driveTo(-10,1000,true,8);
  wait(250,msec);
  hood.set(true);
  lever.spinToPosition(145,degrees,60,rpm,true);
  lever.spinToPosition(0,degrees,200,rpm,true);
  turnToAngle(-93,1000);
  driveTo(20,3000,true,8);
  hood.set(false);
  driveUntil(190,3000,front,true,5);
  wait(2500,msec);
  driveTo(-27,3000,true,8);
  hood.set(true);
  lever.spinToPosition(145,degrees,30,rpm,true);
  lever.spinToPosition(0,degrees,200,rpm,true);
  loader.set(false);
  driveTo(12,3000,true,8);
  turnToAngle(180,3000);
  driveTo(16,3000,true,8);
  turnToAngle(90,3000);
  hood.set(true);
  driveTo(100,5000);
  driveTo(20,3000,true,5);
  driveTo(-12,3000,true,8);
  driveTo(16,3000,true,6);
  turnToAngle(180,3000);
  odomlift.set(true);
  driveChassis(10,10);
  wait(1000,msec);
  driveChassis(-5,-5);
  wait(250,msec);
  driveChassis(0,0);
  
};