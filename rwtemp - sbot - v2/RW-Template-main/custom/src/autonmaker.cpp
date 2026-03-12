#include "vex.h"
#include "motor-control.h"
#include "../custom/include/autonomous.h"
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>



std::string movement = "P";
int voltage = 12;
int timeout = 3000;
int motionchain = 0;
std::string sensor = "front";
int direction = 1;
double x,y = 0;
double startx, starty = 0;
double angle = 0;
int dist = 0;
std::string filename = "";
int autonindex = 0;

std::vector<std::string> autolist = {"right_auto","left_auto","awp_auto","skills_auto","test_auto"};

template<typename T>
auto to_string(T to_convert) -> std::string {
    std::ostringstream converter;
    converter << to_convert;
    return converter.str();
}



void controllerprint_init(){
    controller_1.Screen.clearScreen();
    controller_1.Screen.setCursor(1,1);
    controller_1.Screen.print(autolist.at(autonindex));
    controller_1.Screen.setCursor(2,1);
    controller_1.Screen.print("Confirm: R");
}



void nextauton(){
    autonindex++;
    controllerprint_init();
}

void prevauton(){
    autonindex--;
    controllerprint_init();
}



//RUN IN TOGGLE ON/OFF FUNCTION

void makerinit(){
    resetChassis();
    resetOdom();
    x_pos=0;
    y_pos=0;
    stopChassis(coast);
    while(true){
        stopChassis(coast);
        controller_1.ButtonUp.pressed(nextauton);
        controller_1.ButtonDown.pressed(prevauton);
        if(controller_1.ButtonRight.pressing()){
            break;
        }
    }
    filename=autolist[autonindex]+".txt";
}



//Runs in driver loop when maker is on

int makerstage = 1;

void movetoggle(){
    if(movement == "P"){
        movement = "T";
    }
    else if (movement == "T"){
        movement = "D";
    }
    else if (movement == "D"){
        movement = "M";
    }
    else if (movement == "M"){
        movement = "P";
    }
    controllerprint();
}

void togglechain(){
    if (motionchain == 0){
        motionchain = 1;
    }
    else{
        motionchain = 0;
    }
    controllerprint();
}

void timeouttoggle(){
    if(timeout == 3000){
        timeout = 1000;
    }
    else if (timeout = 1000){
        timeout = 700;
    }
    else if (timeout = 700){
        timeout = 200;
    }
    else if (timeout = 200){
        timeout = 3000;
    }
    controllerprint();
}

void volttoggle(){
    if(voltage == 12){
        voltage = 10;
    }
    else if (voltage = 10){
        voltage = 8;
    }
    else if (voltage = 8){
        voltage = 6;
    }
    else if (voltage = 6){
        voltage = 12;
    }
    controllerprint();
}

void btoggle(){
    if(movement == "P" || movement == "M"){
        if (direction == 1){
            direction = -1;
        }
        else{
            direction = 1;
        }
    }
    else if (movement == "D"){
        if(sensor=="front"){
            sensor="rear";
        }
        else{
            sensor="front";
        }
    }
    controllerprint();
}

void autonmaker(){
    while(true){
        if (makerstage == 1){
            while(true){
                controller_1.ButtonUp.pressed(movetoggle);
                if (controller_1.ButtonRight.pressing()){
                    makerstage=2;
                    wait(250,msec);
                    break;
                }
            }
        }
        if (makerstage == 2){
            startx = x_pos;
            starty = y_pos;
            while(true){
                controller_1.ButtonLeft.pressed(togglechain);
                controller_1.ButtonDown.pressed(timeouttoggle);
                controller_1.ButtonY.pressed(volttoggle);
                controller_1.ButtonB.pressed(btoggle);
                if (controller_1.ButtonRight.pressing()){
                    makerstage=3;
                    wait(250,msec);
                    break;
                }
            }
        }
        if (makerstage == 3){
            if(movement == "P"){
                x=round(10.0*x_pos)/10.0;
                y=round(10.0*y_pos)/10.0;

                std::string txtformat = movement+" "+to_string(x)+" "+to_string(y)+" "+to_string(direction)+" "+to_string(motionchain)+" "+to_string(voltage)+"\n";
                const uint8_t* writetxt = reinterpret_cast<const uint8_t*>(txtformat.c_str());

            }
            else if (movement == "T"){
                angle=getInertialHeading();
                if(87<angle && angle<93){
                    angle=90;
                }
                if(177<angle && angle<183){
                    angle=180;
                }
                if(267<angle && angle<273){
                    angle=270;
                }
                if(angle>357 || angle<3){
                    angle=0;
                }

                std::string txtformat = movement+" "+to_string(angle)+" "+to_string(motionchain)+" "+to_string(voltage)+"\n";
                const uint8_t* writetxt = reinterpret_cast<const uint8_t*>(txtformat.c_str());

            }
            else if (movement == "D"){
                if (sensor=="front"){
                    dist = front.objectDistance(mm);
                    direction = 1;
                }
                else if (sensor=="rear"){
                    dist = rear.objectDistance(mm);
                    direction = -1;
                }

                std::string txtformat = movement+" "+to_string(dist)+" "+to_string(sensor)+" "+to_string(direction)+" "+to_string(motionchain)+" "+to_string(voltage)+"\n";
                const uint8_t* writetxt = reinterpret_cast<const uint8_t*>(txtformat.c_str());

            }
            else if (movement == "M"){
                dist = round(10.0*sqrt(pow(x-startx,2)+pow(y-starty,2)))/10.0;

                std::string txtformat = movement+" "+to_string(dist)+" "+to_string(motionchain)+" "+to_string(voltage)+"\n";
                uint8_t writetxt = reinterpret_cast<uint8_t>(txtformat.c_str());
                Brain.SDcard.appendfile(filename,&writetxt,sizeof(writetxt));

            }
            
        }
    }
}



void controllerprint(){
    controller_1.Screen.clearScreen();
    controller_1.Screen.setCursor(1,1);
    controller_1.Screen.print(movement);
    controller_1.Screen.setCursor(2,1);
    controller_1.Screen.print("V: ");
    controller_1.Screen.print(voltage);
    if(movement == "D"){
        controller_1.Screen.print(" S: ");
        controller_1.Screen.print(sensor);
    }
    else{
        controller_1.Screen.print(" D: ");
        controller_1.Screen.print(direction);
    }
    controller_1.Screen.setCursor(3,1);
    controller_1.Screen.print("T: ");
    controller_1.Screen.print(timeout);
    controller_1.Screen.print(" M: ");
    controller_1.Screen.print(motionchain);
}

