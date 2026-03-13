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
  stopChassis(hold);
  resetOdom();



  // driveTo(48,3000);
  // wait(2000,msec);
  // driveTo(-24,3000);
  // wait(2000,msec);
  // driveTo(-12,3000);
  // wait(2000,msec);
  // driveTo(-6,3000);
  // wait(2000,msec);
  // driveTo(-6,3000);


  // turnToAngle(180,3000,true,8);
  // wait(2000,msec);
  // turnToAngle(90,3000);
  // wait(2000,msec);
  // turnToAngle(-45,3000);
  // wait(2000,msec);
  // turnToAngle(45,3000);
  // wait(2000,msec);
  // turnToAngle(0,3000);

  // moveToPoint(12,24,1,3000,true);
  // wait(2000,msec);
  // turnToAngle(45,3000,true);
  // wait(100,msec);
  // moveToPoint(24,0,-1,3000,true);
  // wait(2000,msec);
  // moveToPoint(0,0,1,3000,true);

  swingpower(-3,6,-100,2000);
}

void scorelong(){
  intake.spin(reverse,600,rpm);
  outtake.spin(reverse,600,rpm);
  wait(100,msec);
  intake.spin(forward,600,rpm);
  outtake.spin(forward,600,rpm);
  wait(1500,msec);
  outtake.stop();
  intake.stop();
}

void scoreskills(){
  intake.spin(reverse,600,rpm);
  outtake.spin(reverse,600,rpm);
  wait(100,msec);
  intake.spin(forward,600,rpm);
  outtake.spin(forward,150,rpm);
  wait(3000,msec);
  outtake.stop();
  intake.stop();
}


void scoreshort(){
  intake.spin(forward,600,rpm);
  outtake.spin(forward,70,rpm);
  wait(400,msec);
  outtake.spin(reverse,20,rpm);
  intake.spin(reverse,50,rpm);
  wait(100,msec);
  intake.stop();
  outtake.stop();

}

void scoremid(){
  intake.spin(reverse,600,rpm);
  outtake.spin(reverse,600,rpm);
  wait(100,msec);
  intake.spin(forward,600,rpm);
  outtake.spin(forward,100,rpm);
  wait(1000,msec);
  intake.stop();
  outtake.stop();

}

void rightauto(){
  resetChassis();
  resetOdom();
  stopChassis(hold);
  hood.set(true);
  wing.set(true);
  
  intake.spin(forward,600,rpm);
  moveToPoint(-1.5,2,1,3000,false,7);
  moveToPoint(12,28,1,3000,false,7);
  moveToPoint(16,36,1,3000,false,7);
  turnToAngle(80,500,true);
  driveTo(10,1000);
  intake.stop();
  moveToPoint(12,24,-1,3000);
  wait(100,msec);
  turnToAngle(-45,700,true);
  intake.spin(reverse,600,rpm);
  driveTo(16,3000,true);
  wait(300,msec);
  intake.spin(forward,600,rpm);
  loader.set(true);
  hood.set(true);
  intake.spin(forward,600,rpm);
  moveToPoint(25,8,-1,3000,false);  //35
  turnToAngle(90,700,true);
  driveUntil(470,2000,front,true);
  turnToAngle(-180,3000);
  driveUntil(160,2000,front,true);
  wait(500,msec);
  turnToAngle(180,5000);
  driveTo(-29,1000);
  hood.set(true);
  loader.set(false);
  scoremid();
  turnToAngle(140,500);
  driveTo(13,1000,true); 
  turnToAngle(-180,500);
  wing.set(false);
  driveTo(-28,3000,true,10);
}



void leftauto(){
  resetChassis();
  resetOdom();
  stopChassis(hold);
  hood.set(false);
  wing.set(true);
  
  intake.spin(forward,600,rpm);
  moveToPoint(1.5,8,1,3000,false,7);
  moveToPoint(-12,28,1,3000,false,7);
  moveToPoint(-26,39,1,3000,true,7);
  wait(100,msec);
  moveToPoint(-9,26,-1,3000);
  wait(100,msec);
  turnToAngle(-135,700,true);
  intake.stop();
  driveTo(-17,1000,true);
  scoreshort();
  loader.set(true);
  hood.set(true);
  intake.spin(forward,600,rpm);
  moveToPoint(-25,8,1,3000,false);  //35
  turnToAngle(-92,3000,true);
  driveUntil(480,2000,front,true);
  turnToAngle(180,3000);
  driveUntil(160,2000,front,true);
  wait(300,msec);
  turnToAngle(181.5,5000);
  driveTo(-29,1000);
  hood.set(true);
  loader.set(false);
  scorelong();
  turnToAngle(140,700);
  driveTo(13,1000,true);
  turnToAngle(180,500);
  wing.set(false);
  driveTo(-26,3000,true,10);
}


void awpauto(){
  resetChassis();
  resetOdom();
  stopChassis(hold);
  hood.set(true);
  wing.set(true);
  loader.set(true);
  intake.spin(forward,600,rpm);
  driveUntil(450,3000,front,true);
  turnToAngle(90,700);
  driveUntil(160,3000,front,true);
  wait(150,msec);
  turnToAngle(90,200);
  driveTo(-30,1000);
  intake.spin(forward,600,rpm);
  outtake.spin(forward,100,rpm);
  wait(1000,msec);
  intake.stop();
  outtake.stop();
  intake.spin(forward,600,rpm);
  loader.set(false);
  outtake.stop();
  intake.spin(forward,600,rpm);
  driveChassis(7,-12);
  wait(600,msec);
  turnToAngle(180,100); 
  hood.set(false);
  driveTo(76,3000,true);

  driveTo(-19,3000);
  turnToAngle(120,700);
  driveTo(-8,800,false);
  intake.spin(forward,600,rpm);
  outtake.spin(forward,140,rpm);
  driveTo(-6,700,true);
  outtake.spin(forward,120,rpm);
  wait(300,msec);
  outtake.stop();
  wait(100,msec);
  hood.set(true);
  loader.set(true);

  // stopChassis(hold);
  // driveChassis(-9,-4);
  // wait(600,msec);
  // wait(100,msec);
  // driveChassis(-4,-4);
  // wait(100,msec);
  // driveTo(-1,100);
  // outtake.spin(forward,160,rpm);
  // wait(400,msec);
  // outtake.stop();
  // wait(100,msec);
  // hood.set(true);
  // loader.set(true);
  // turnToAngle(120,100);

  driveTo(35,1500,true);
  outtake.stop();
  intake.spin(forward,600,rpm);
  turnToAngle(180,700);
  driveUntil(470,3000,front,true);
  turnToAngle(88,700);
  intake.spin(forward,600,rpm);
  driveUntil(160,3000,front,true);
  
  wait(250,msec);
  turnToAngle(88,200);
  driveTo(-29,1000);
  outtake.spin(forward,600,rpm);
  wait(900,msec);
  

  
  





};



void skillsauto(){
  resetChassis();
  resetOdom();
  stopChassis(hold);
  hood.set(true);
  driveTo(20,3000);
  loader.set(true);
  intake.spin(forward,600,rpm);
  driveUntil(460,3000,front,true);
  turnToAngle(90,3000);
  stopChassis(brake);
  driveUntil(170,3000,front,true);
  wait(2300,msec);
  stopChassis(hold);
  driveTo(-8,3000,true);
  turnToAngle(180,3000);
  intake.stop();

  driveUntil(100,3000,rear,true,12,-1);
  turnToAngle(-90,3000);
  driveTo(90,5000);
  turnToAngle(180,3000);
  driveUntil(410,3000,rear,true,12,-1);
  turnToAngle(-90,3000);  
  driveTo(-12,1000,true,5);
  scoreskills();

  intake.spin(forward,600,rpm);
  turnToAngle(-92,7000,true,12);
  driveTo(24,3000);
  stopChassis(brake);
  driveUntil(170,3000,front,true);
  wait(2300,msec);    
  stopChassis(hold);
  driveTo(-24,3000,true);
  driveTo(-7,1200,true,5);
  scoreskills();

  loader.set(false);
  turnToAngle(-120,3000);
  driveTo(34,3000,true,7);
  swing(180,1,3000);
  lifter.set(true);
  driveTo(12,2000,true,10);
  intake.spin(forward,600,rpm);
  outtake.spin(forward,600,rpm);
  driveUntil(1300,5000,front,true,8,1); 
  wait(500,msec);


  driveTo(24,5000,true);
  swing(-240,1,3000);
  driveTo(14,1000,true,10);
  swing(180,1,3000);
  driveUntil(450,3000,front,true);
  turnToAngle(-90,3000);
  driveTo(10,3000);
  intake.spin(forward,600,rpm);
  stopChassis(brake);
  driveUntil(170,3000,front,true);
  stopChassis(hold);
  wait(2300,msec);
  driveTo(-8,3000,true);
  turnToAngle(0,3000);
  intake.stop();
  







  // driveTo(8,3000);
  // turnToAngle(-178,3000);
  // driveTo(100,3000);
  // wait(200,msec);
  // driveUntil(420,3000,front,true);
  // turnToAngle(-90,3000);
  // driveTo(10,3000);
  // intake.spin(forward,600,rpm);
  // stopChassis(brake);
  // driveUntil(170,3000,front,true);
  // stopChassis(hold);
  // wait(2300,msec);
  // driveTo(-8,3000,true);
  // turnToAngle(0,3000);
  // intake.stop();

  // driveUntil(100,3000,rear,true,12,-1);
  // turnToAngle(88,3000);
  // driveTo(90,5000);
  // turnToAngle(0,3000);
  // driveUntil(420,3000,rear,true,12,-1);
  // turnToAngle(90,3000);
  // driveTo(-12,1000,true,5);
  // scoreskills();

  // intake.spin(forward,600,rpm);
  // turnToAngle(88,700,true,12);
  // driveTo(24,3000);
  // stopChassis(brake);
  // driveUntil(160,3000,front,true);
  // stopChassis(hold);
  // wait(2300,msec);
  // driveTo(-24,2000,true);
  // driveTo(-7,1000,true,5);
  // scoreskills();

  // loader.set(false);
  // turnToAngle(60,3000);
  // driveTo(32,3000,true,7);
  // swing(0,1,3000);
  // lifter.set(true);
  // driveTo(14,2000,true,10);
  // intake.spin(forward,600,rpm);
  // outtake.spin(forward,600,rpm);
  // driveUntil(1300,5000,front,true,8,1); 



  
};


void skills2(){
  resetChassis();
  resetOdom();
  stopChassis(coast);
  lifter.set(true);
  intake.spin(forward,600,rpm);

  driveChassis(4,4);
  wait(600,msec);
  driveChassis(-3,-3);
  wait(200,msec);
  driveChassis(1,1);
  wait(200,msec);
  driveChassis(-3,-3);
  wait(200,msec);
  driveChassis(1,1);
  wait(200,msec);
  driveChassis(-3,-3);
  wait(200,msec);
  driveChassis(1,1);
  wait(200,msec);

  driveChassis(6,6);
  wait(600,msec);
  driveChassis(-2,-2);
  wait(200,msec);
  driveChassis(2,2);
  wait(200,msec);
  driveChassis(-3,-3);
  wait(200,msec);
  driveChassis(4,4);
  wait(200,msec);
  driveChassis(-2,-2);
  wait(400,msec);
  driveChassis(4,4);
  wait(400,msec);
  driveChassis(-3,-3);
  wait(200,msec);
  driveChassis(4,4);
  wait(200,msec);
  driveChassis(-2,-2);
  wait(400,msec);
  driveChassis(4,4);
  wait(400,msec);
   

  driveChassis(0,0);
  wait(200,msec);
  driveTo(-40,3000,true);
  driveChassis(3,3);
  wait(1000,msec);
  driveChassis(0,0);
  lifter.set(false);
  hood.set(true);
  intake.stop();

  wait(200,msec);
  resetChassis();
  resetOdom();
  stopChassis(hold);

  intake.spin(forward,600,rpm);
  driveTo(-24,2000);
  turnToAngle(-102,800);
  driveTo(20,2000,true,6);
  intake.stop();
  wait(200,msec);
  //moveToPoint(-20,-17,-1,800);
  intake.stop();
  turnToAngle(135,900);
  driveChassis(4,4);
  wait(1300,msec);


  // outtake.spin(reverse,120,rpm);
  // intake.spin(reverse,120,rpm);
  // wait(4500,msec);
  // driveTo(-1,500);
  // wait(500,msec);
  // driveTo(-2,700);
  // intake.spin(reverse,80,rpm);
  // wait(1500,msec);


  driveTo(-3,500);
  outtake.spin(reverse,120,rpm);
  intake.spin(reverse,120,rpm);
  wait(4500,msec);
  wait(500,msec);
  wait(1500,msec);
  driveTo(-6,800);


  swing(180,-1,800);
  outtake.stop();
  intake.stop();
  driveTo(-24,2000);
  turnToAngle(-90,7000);
  loader.set(true);
  intake.spin(forward,600,rpm);
  
  driveUntil(460,3000,front,true);
  turnToAngle(0,3000);
  stopChassis(brake);
  driveUntil(170,3000,front,true);
  wait(2300,msec);
  stopChassis(hold);
  driveTo(-8,3000,true);
  turnToAngle(90,700);
  intake.stop();

  driveUntil(110,3000,rear,true,12,-1);
  turnToAngle(178,700);
  driveTo(90,5000);
  turnToAngle(90,700);
  driveUntil(410,3000,rear,true,12,-1);
  turnToAngle(180,700);  
  driveTo(-12,1000,true,5);
  scoreskills();

  intake.spin(forward,600,rpm);
  turnToAngle(-184,300,true,12);
  driveTo(24,3000);
  stopChassis(brake);
  driveUntil(170,3000,front,true);
  wait(2300,msec);    
  stopChassis(hold);
  driveTo(-24,3000,true);
  driveTo(-7,1200,true,5);
  scoreskills();

  driveTo(8,3000);
  turnToAngle(92,700);
  driveTo(100,3000);
  wait(200,msec);
  driveUntil(420,3000,front,true);
  turnToAngle(180,700);
  driveTo(10,3000);
  intake.spin(forward,600,rpm);
  stopChassis(brake);
  driveUntil(170,3000,front,true);
  stopChassis(hold);
  wait(2300,msec);
  driveTo(-8,3000,true);
  turnToAngle(-90,700);
  intake.stop();

  driveUntil(100,3000,rear,true,12,-1);
  turnToAngle(-2,700);
  driveTo(90,5000);
  turnToAngle(-90,700);
  driveUntil(410,3000,rear,true,12,-1);
  turnToAngle(-3,700);
  driveTo(-12,1000,true,5);
  scoreskills();

  intake.spin(forward,600,rpm);
  turnToAngle(-6,300,true,12);
  driveTo(24,3000);
  stopChassis(brake);
  driveUntil(160,3000,front,true);
  stopChassis(hold);
  wait(2300,msec);
  turnToAngle(0,300);
  driveTo(-24,2000,true);
  driveTo(-7,1000,true,5);
  scoreskills();

  loader.set(false);
  turnToAngle(-30,700);
  driveTo(32,3000,true,7);
  swing(-90,1,3000);
  lifter.set(true);
  driveTo(14,2000,true,10);
  intake.spin(forward,600,rpm);
  outtake.spin(forward,600,rpm);
  driveChassis(6,6);
  wait(800,msec);
  driveChassis(0,0);
}