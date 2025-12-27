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
  lever.setPosition(120,degrees);
  lever.spinToPosition(0,degrees,200,rpm,false);
  wing.set(true);
  lifter.set(true);
  intake.spin(forward,600,rpm);
  moveToPoint(-1,10,1,3000,false,5);
  moveToPoint(12,28,1,3000,false,5);
  moveToPoint(30,40,1,3000,true,5);
  wait(500,msec);
  moveToPoint(12,22,-1,3000);
  wait(100,msec);
  turnToAngle(-45,3000);
  driveTo(14.5,1000);
  intake.spin(reverse,150,rpm);
  wait(500,msec);
  intake.spin(forward,600,rpm);
  moveToPoint(37,0,-1,3000);
  wait(100,msec);
  loader.set(true);
  turnToAngle(180,3000);
  driveUntil(200,3000,front,true,5,1.0);
  hood.set(false);
  wait(750,msec);
  moveToPoint(36,16,-1,3000,true,6);
  hood.set(true);
  lever.spinToPosition(145,degrees,200,rpm,true);
  lever.spinToPosition(0,degrees,200,rpm,false);




}



void leftauto(){
  resetChassis();
  resetOdom();
  stopChassis(hold);
  lever.setPosition(120,degrees);
  lever.spinToPosition(0,degrees,200,rpm,false);
  wing.set(true);
  
  intake.spin(forward,600,rpm);
  moveToPoint(1.5,10,1,3000,false,6);
  moveToPoint(-12,28,1,3000,false,5);
  moveToPoint(-30,40,1,3000,true,5);
  wait(500,msec);
  moveToPoint(-12,23,-1,3000);
  wait(100,msec);
  turnToAngle(-135,3000,true);
  lifter.set(false);
  driveTo(-15,3000,true,8);
  hood.set(true);
  lever.spinToPosition(140,degrees,30,rpm,true);
  lever.spinToPosition(0,degrees,200,rpm,true);
  loader.set(true);
  hood.set(false);
  lifter.set(true);
  moveToPoint(-36,0,1,3000,true);
  turnToAngle(180,3000);
  driveUntil(200,2000,front,true,5);
  wait(900,msec);
  moveToPoint(-37,15,-1,3000,true);
  hood.set(true);
  lever.spinToPosition(150,degrees,30,rpm,true);
  lever.spinToPosition(0,degrees,200,rpm,true);

}


void awpauto(){
  resetChassis();
  resetOdom();
  stopChassis(hold);
  lever.setPosition(120,degrees);
  lever.spinToPosition(0,degrees,200,rpm,false);
  wing.set(true);
  lifter.set(true);
  loader.set(true);
  driveTo(36.3,2000,true,10);
  turnToAngle(90,2000);
  intake.spin(forward,600,rpm);
  driveUntil(200,3000,front,true,5);
  wait(1100,msec);
  driveTo(-29,1700,true,8);
  hood.set(true);
  loader.set(false);
  lever.spinToPosition(150,degrees,200,rpm,true);
  intake.spin(reverse,600,rpm);
  lever.spinToPosition(0,degrees,200,rpm,true);
  intake.spin(forward,600,rpm);
  
  
  turnToAngle(-180,2000);
  hood.set(false);
  moveToPoint(-25,8,1,3000,false,6);
  lifter.set(false);
  moveToPoint(-24,-38,1,3000,false,6);
  
  turnToAngle(135,1000);
  driveTo(-15.5,2000,true,8);
  hood.set(true);
  lever.spinToPosition(145,degrees,30,rpm,true);
  lever.spinToPosition(0,degrees,200,rpm,true);
  hood.set(false);
  lifter.set(true);
  moveToPoint(0,-60.5,1,3000,true,9);
  loader.set(true);
  turnToAngle(90,1000);
  driveUntil(200,3000,front,true,5);
  wait(900,msec);
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
  lever.setPosition(120,degrees);
  lever.spinToPosition(0,degrees,200,rpm,false);
  wing.set(true);
  lifter.set(true);
  loader.set(true);
  driveTo(36.3,2000,true,10);
  turnToAngle(-90,2000);
  intake.spin(forward,600,rpm);
  driveUntil(200,3000,front,true,5);
  wait(2000,msec);
  driveTo(-12,3000,true,8);
  driveUntil(140,3000,rear,true,6,-1.0);
  turnToAngle(180,3000);
  turnToAngle(90,3000);
  driveTo(72,3000);
  turnToAngle(180,3000);
  driveUntil(490,3000,rear,true,7,1.0);
  turnToAngle(90,3000);
  
};