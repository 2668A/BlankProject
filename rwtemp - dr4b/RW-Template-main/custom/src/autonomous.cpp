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
  driveTo(24,3000,true,8);
  swingpower(1,5,-90,3000);
  turnToAngle(-90,30,true);
  driveTo(-12,3000,true,7);
  turnToAngle(0,3000,true);
  //moveToPoint(12,12,3000,true,8);
}

void closeallianceauto(){
  claw.setVelocity(100,percent);
  lift.setVelocity(70,percent);
  lift.setStopping(brake);
  claw.spinFor(reverse,300,msec);
  lift.spinFor(forward,140,degrees);
  driveTo(11,2000,true,7);
  lift.spin(reverse);
  wait(750,msec);
  lift.stop();
  claw.spin(reverse);

  driveTo(-8,2000,true);
  claw.stop();
  turnToAngle(85,2000,true);
  lift.spin(reverse);
  wait(500,msec);
  claw.spin(forward);
  lift.stop();
  driveTo(10,2000,true,2);
  claw.stop();
  lift.spinFor(forward,240,degrees,false);

  turnToAngle(-36,2000,true);
  driveTo(14,2000,true,7);
  lift.spin(reverse);
  wait(600,msec);
  lift.stop();
  claw.spin(reverse);
  wait(250,msec);
  lift.spinFor(forward,260,degrees,false);
  driveTo(-12,1000,true);
  claw.stop();
  turnToAngle(-90,1000,true,7);
  driveTo(5,1000,false,8);
  swing(-40,1,1000,false,8);
  driveTo(24,1000,false,8);
  swing(-135,1,1000,false,8);
  driveTo(5,500,true,5);
  roller.spinFor(forward,240,degrees,false);
  driveTo(12,800,true,3);
  driveTo(-6,800,true);








  // 3 pin
  // driveTo(-12,2000,true);
  // claw.stop();
  // turnToAngle(6,2000,true);
  // driveTo(16,2000,true,8);
  // claw.spin(forward);
  // driveTo(12,2000,true,2);
  // claw.stop();
  // lift.setVelocity(50,percent);
  // lift.spinFor(forward,240,degrees,true);
  // wait(300,msec);
  // turnToAngle(-140,2000,true);
  // driveTo(14,2000,true,7);
  // lift.setVelocity(70,percent);
  // lift.spin(reverse);
  // wait(500,msec);
  // lift.stop();
  // claw.spin(reverse);
  // driveTo(-7,2000,true);
  // claw.stop();
  // turnToAngle(-100,2000,true);
  // swingpower(7,8,-135,3000);
  



}

void closemidauto(){
  claw.setVelocity(100,percent);
  lift.setVelocity(70,percent);
  lift.setStopping(brake);
  claw.setVelocity(100,percent);
  lift.setVelocity(70,percent);
  lift.setStopping(brake);
  claw.spinFor(reverse,300,msec);
  lift.spinFor(forward,140,degrees);
  driveTo(11,2000,true,7);
  lift.spin(reverse);
  wait(750,msec);
  lift.stop();
  claw.spin(reverse);


  driveTo(-8,2000,true);
  claw.stop();
  turnToAngle(-45,2000,true);
  driveTo(24,2000,true);
  turnToAngle(0,2000,true);
  claw.spin(forward);
  driveTo(12,2000,true,2);
  claw.stop();
  lift.spinFor(forward,240,degrees,false);
  turnToAngle(130,2000,true);
  driveTo(20,2000,true,6);
  lift.spin(reverse);
  wait(600,msec);
  lift.stop();
  claw.spin(reverse);
  driveTo(-8,2000,true);
  claw.stop();
  
  turnToAngle(180,2000,true);
  driveTo(8,1000,false,6);
  lift.spinFor(forward,260,degrees,false);
  swing(135,1,1000,false,6);
  driveTo(20,1500,true,6);
  driveTo(5,500,true,5);
  roller.spinFor(forward,240,degrees,false);
  driveTo(12,800,true,3);
  driveTo(-6,800,true);
}

void farallianceauto(){
  
}

void farmidauto(){
  
}

void awpauto(){
 
}

void awpauto2(){
  
}




void skillsauto(){
  
}