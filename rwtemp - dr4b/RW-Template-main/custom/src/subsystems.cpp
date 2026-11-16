//#include "subsystems.h"
#include "pid.h"
#include "vex.h"

vex::color targetColor;
double targetSpeed;
bool toggleRunning = false;


void toggleTask(){
    ToggleSensor.setLight(ledState::on);
    while(targetColor != ToggleSensor.color()){
        roller.spin(forward, targetSpeed, percent);
        wait(5, msec);
        ToggleSensor.setLightPower(50, percent);
    }
    roller.stop();
    ToggleSensor.setLight(ledState::off);
    toggleRunning = false;
    

}

void autoToggle(vex::color Target, double speed) {
    
    targetColor = Target;
    targetSpeed = speed;
    thread toggl = thread(toggleTask);
}

void toggleLoop(bool r2, bool button_y, bool button_a){
    if(r2){
      roller.spin(forward,100,percent);
    }
    else if(button_y){
      roller.spin(reverse, 80, percent);
    }
    else if(!toggleRunning){
      roller.stop();
    }

    if(button_a){
      toggleRunning = true;
      autoToggle(vex::color::red, 50);
    }
}

// void toTarget(double targetPosition) {
//     liftPID.setTarget(targetPosition);
//     thread lifter = thread(liftLoop);
//     //lift.stop();
// }

// void liftLoop() {
//     while(!liftPID.targetArrived()) {
//         lift.spin(forward, liftPID.update(lift.position(degrees)), voltageUnits::volt);
//         wait(10, msec);
//     }
//     lift.stop();
// }

// void toTargetBlocking(double targetPosition) {
//     liftPID.setTarget(targetPosition);
//     while(!liftPID.targetArrived()){
//         lift.spin(forward, liftPID.update(lift.position(degrees)), voltageUnits::volt);
//         wait(10, msec);
//     }
//     lift.stop();
// }