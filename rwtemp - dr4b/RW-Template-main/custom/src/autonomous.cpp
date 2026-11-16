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
  moveToPoint(12,24,3000,true,8);
  turnToAngle(-90,3000,true);
  driveTo(12,3000,true,8);
  swingpower(4,6,180,3000);
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
 
}

void awpauto2(){
  
}




void skillsauto(){
  
}