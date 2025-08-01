#include "main.h"

#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/colors.h"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "lemlib/api.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include "pros/vision.hpp"
#include <future>
//#include "pros/rtos.h"
//#include "pros/vision.hpp"

using namespace pros::c;


//Robot system motors
inline pros::Motor intake_bottom(6,pros::v5::MotorGears::blue);
inline pros::Motor intake_middle(-7,pros::v5::MotorGears::blue);
inline pros::Motor intake_top(8,pros::v5::MotorGears::blue);
pros::Optical intake_optical(4);
//Robot system pneumatics
pros::adi::Pneumatics ramp1('a', false);
pros::adi::Pneumatics little_will('b', false);
pros::adi::Pneumatics rake('c', false);
pros::Controller master(pros::E_CONTROLLER_MASTER);
//Drive motor groups
pros::MotorGroup leftMotors({-11, 12, -13}, pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup rightMotors({17, -18, 19}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)
//Inertial Sensor
pros::Imu imu(5);
//horizontal and vertical tracking wheel encoders
pros::Rotation horizontalEnc(-15);
pros::Rotation verticalEnc(-2);
//tracking wheels
lemlib::TrackingWheel horizontal(&horizontalEnc, 2, -5); //-5
lemlib::TrackingWheel vertical(&verticalEnc, 2.80, -0.8); //2*1.030, -1.5
// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              11.5, // 10 inch track width
                              3.25/1.021, // using new 4" omnis
                              450, // drivetrain rpm is 360
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);
// lateral motion controller
lemlib::ControllerSettings linearController(20, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            140, // derivative gain (kD)
                                            0.5, // anti windup
                                            0.25, // small error range, in inches
                                            50, // small error range timeout, in milliseconds
                                            0.5, // large error range, in inches
                                            100, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(2, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             10, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);
// sensors for odometry
lemlib::OdomSensors sensors(&vertical, // &vertical, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            &horizontal, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);
// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     5, // minimum output where drivetrain will move out of 127
                                     1.00 // expo curve gain
);
// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  5, // minimum output where drivetrain will move out of 127
                                  1.017 // expo curve gain
);
// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);



void screentask(){
    while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading

            //pros::lcd::print(6, "X: %f", chassis.getPose().x-dX); // x
            //pros::lcd::print(7, "Y: %f", chassis.getPose().y-dY); // y

            
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
			
        }
}


int autonvalue = 0;
int autonnum = 8;
std::string autonname[] = {"Right Auto", "Left Auto", "Right Long Auto", "Left Long Auto", "Mid Auto", "AWP Auto", "Test Auto", "Skills"};





void initialize() {

    


    leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);


	intake_top.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	

	pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors

    pros::delay(1000);
    
    pros ::lcd::register_btn0_cb([]() {
        autonvalue = (autonvalue + 1) % autonnum;    
        pros::lcd::print(3, "Auton: %d", autonvalue);
        pros::lcd::print(4, "Auton: %s", autonname[autonvalue]); // print the current auton value
    });
    pros ::lcd::register_btn2_cb([]() {
        autonvalue = (autonvalue - 1);
        if (autonvalue == -1){
            autonvalue = autonnum - 1;  
        }
        pros::lcd::print(3, "Auton: %d", autonvalue);
        pros::lcd::print(4, "Auton: %s", autonname[autonvalue]); // print the current auton value
    });

    // thread to for brain screen and position logging
    
}


void disabled() {}

void competition_initialize() {}

void pidtestpath(){
    chassis.setBrakeMode(MOTOR_BRAKE_BRAKE);
    chassis.setPose(0,0,0);
    // Move to x: 20 and y: 15, and face heading 90. Timeout set to 4000 ms
    chassis.moveToPoint(0, 72, 10000, {.maxSpeed = 60});
    chassis.waitUntilDone();
    pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
        pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
        pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
    pros::delay(5000);
    // Move to x: 0 and y: 0 and face heading 270, going backwards. Timeout set to 4000ms

    // chassis.turnToHeading(90,4000,{.maxSpeed = 60});
    // chassis.waitUntilDone();
    // pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
    //     pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
    //     pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
    // pros::delay(5000);

    // chassis.moveToPoint(48, 72, 10000,{.maxSpeed = 60, .minSpeed = 5} );  
    // chassis.waitUntilDone();
    // pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
    //     pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
    //     pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
    // pros::delay(5000);
   
    

}

void pidtestpathpose(){
    chassis.setBrakeMode(MOTOR_BRAKE_BRAKE);
    chassis.setPose(0,0,0);
    // Move to x: 20 and y: 15, and face heading 90. Timeout set to 4000 ms
    chassis.moveToPose(24, 24, 90, 10000, {.maxSpeed = 80, .minSpeed = 0});
    chassis.waitUntilDone();
    pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
        pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
        pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
    pros::delay(5000);
    // Move to x: 0 and y: 0 and face heading 270, going backwards. Timeout set to 4000ms

    chassis.turnToHeading(-90,4000,{.maxSpeed = 60});
    chassis.waitUntilDone();
    chassis.moveToPose(0, 0, 180, 10000,{.maxSpeed = 80, .minSpeed = 0} );
    
    // cancel the movement after it has traveled 10 inches
    chassis.waitUntilDone();
    pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
        pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
        pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
    pros::delay(5000);
    // Turn to face the point x:45, y:-45. Timeout set to 1000
    // dont turn faster than 60 (out of a maximum of 127)
   
    

}

void rightlongauto(){
    chassis.setBrakeMode(MOTOR_BRAKE_BRAKE);
    chassis.setPose(6.75,-1,0);
    intake_bottom.move(200);
    intake_middle.move(200);
    chassis.moveToPoint(27,30,3000,{.maxSpeed=60});
    chassis.waitUntilDone();
    chassis.turnToPoint(48,0,2000,{.maxSpeed=60});
    chassis.waitUntilDone();
    chassis.moveToPoint(48,0,2000,{.maxSpeed=60});
    chassis.waitUntilDone();
    chassis.turnToHeading(180,2000,{.maxSpeed=60});
    chassis.waitUntilDone();
    intake_bottom.move(0);
    intake_middle.move(0);
    ramp1.set_value(true);
    chassis.moveToPoint(47,13,2000,{.forwards=false,.maxSpeed=60});
    chassis.waitUntilDone();
    intake_bottom.move(200);
    intake_middle.move(200);
    intake_top.move(200);
    pros::delay(1500);
    intake_top.move(0);
    little_will.extend();
    chassis.moveToPoint(46.5,-16,2000,{.maxSpeed=40,.minSpeed=10});
    intake_bottom.move(200);
    intake_middle.move(200);
    pros::delay(1500);
    chassis.moveToPoint(46.5,13,2000,{.forwards=false,.maxSpeed=60});
    chassis.waitUntilDone();
    intake_top.move(200);
    int counter = 0;
    while(counter<150){
        intake_bottom.move(200);
        intake_middle.move(200);
        intake_top.move(200);
        if(intake_optical.get_hue() > 100 && intake_optical.get_hue() < 300){
            counter=0;
        }
        else if (intake_optical.get_hue() > 0 && intake_optical.get_hue() < 20){
            break;
        }
        else{
            counter++;
        }
        pros::delay(20);

    }
    intake_bottom.move(-200);
    intake_middle.move(-200);
    intake_top.move(-200);
    pros::delay(3000);


}

void leftlongauto(){
    chassis.setBrakeMode(MOTOR_BRAKE_BRAKE);
    chassis.setPose(-6.75,-1,0);
    intake_bottom.move(200);
    intake_middle.move(200);
    chassis.moveToPoint(-27,30,3000,{.maxSpeed=60});
    chassis.waitUntilDone();
    chassis.turnToPoint(-48,0,2000,{.maxSpeed=60});
    chassis.waitUntilDone();
    chassis.moveToPoint(-48,0,2000,{.maxSpeed=60});
    chassis.waitUntilDone();
    chassis.turnToHeading(-180,2000,{.maxSpeed=60});
    chassis.waitUntilDone();
    intake_bottom.move(0);
    intake_middle.move(0);
    ramp1.set_value(true);
    chassis.moveToPoint(-47,13,2000,{.forwards=false,.maxSpeed=60});
    chassis.waitUntilDone();
    intake_bottom.move(200);
    intake_middle.move(200);
    intake_top.move(200);
    pros::delay(1500);
    intake_top.move(0);
    little_will.extend();
    chassis.moveToPoint(-46.5,-16,2000,{.maxSpeed=40,.minSpeed=10});
    intake_bottom.move(200);
    intake_middle.move(200);
    pros::delay(1500);
    chassis.moveToPoint(-46.5,13,2000,{.forwards=false,.maxSpeed=60});
    chassis.waitUntilDone();
    intake_top.move(200);
    int counter = 0;
    while(counter<150){
        intake_bottom.move(200);
        intake_middle.move(200);
        intake_top.move(200);
        if(intake_optical.get_hue() > 100 && intake_optical.get_hue() < 300){
            counter=0;
        }
        else if (intake_optical.get_hue() > 0 && intake_optical.get_hue() < 20){
            break;
        }
        else{
            counter++;
        }
        pros::delay(20);

    }
    intake_bottom.move(-200);
    intake_middle.move(-200);
    intake_top.move(-200);
    pros::delay(3000);


}

void rightauto(){
    chassis.setBrakeMode(MOTOR_BRAKE_BRAKE);
    chassis.setPose(6.75,-1,0);
    intake_bottom.move(200);
    intake_middle.move(200);
    chassis.moveToPoint(28,30,3000,{.maxSpeed=40});
    chassis.waitUntilDone();
    chassis.turnToPoint(48,0,2000,{.maxSpeed=60});
    chassis.waitUntilDone();
    chassis.moveToPoint(48,0,2000,{.maxSpeed=60});
    chassis.waitUntilDone();
    chassis.turnToHeading(180,2000,{.maxSpeed=60});
    chassis.waitUntilDone();
    intake_bottom.move(0);
    intake_middle.move(0);
    ramp1.set_value(true);
    chassis.moveToPoint(47,13,2000,{.forwards=false,.maxSpeed=30});
    chassis.waitUntilDone();
    intake_bottom.move(200);
    intake_middle.move(200);
    intake_top.move(200);
    pros::delay(3500);
    intake_top.move(0);
    little_will.extend();
    chassis.moveToPoint(46.75,-4,2000,{.maxSpeed=60, .minSpeed=20, .earlyExitRange=4});
    chassis.moveToPoint(46.75,-16,2000,{.maxSpeed=40,.minSpeed=20});
    pros::delay(1500);
    ramp1.retract();
    chassis.moveToPoint(12.5,30,3000,{.forwards=false,.maxSpeed=60});
    chassis.waitUntilDone();
    intake_bottom.move(200);
    intake_middle.move(200);
    intake_top.move(200);
    little_will.retract();
    pros::delay(1000);
    int counter = 0;
    while(counter<100){
        intake_bottom.move(200);
        intake_middle.move(200);
        intake_top.move(200);
        if(intake_optical.get_hue() > 100 && intake_optical.get_hue() < 300){
            counter=0;
        }
        else if (intake_optical.get_hue() > 0 && intake_optical.get_hue() < 20){
            break;
        }
        else{
            counter++;
        }
        pros::delay(20);

    }
    intake_bottom.move(-200);
    intake_middle.move(-200);
    intake_top.move(-200);
    pros::delay(5000);

}

void leftauto(){
    chassis.setBrakeMode(MOTOR_BRAKE_BRAKE);
    chassis.setPose(-6.75,-1,0);
    intake_bottom.move(200);
    intake_middle.move(200);
    chassis.moveToPoint(-28,30,3000,{.maxSpeed=40});
    chassis.waitUntilDone();
    chassis.turnToPoint(-48,0,2000,{.maxSpeed=60});
    chassis.waitUntilDone();
    chassis.moveToPoint(-48,0,2000,{.maxSpeed=60});
    chassis.waitUntilDone();
    chassis.turnToHeading(-180,2000,{.maxSpeed=60});
    chassis.waitUntilDone();
    intake_bottom.move(0);
    intake_middle.move(0);
    ramp1.set_value(true);
    chassis.moveToPoint(-47,13,2000,{.forwards=false,.maxSpeed=30});
    chassis.waitUntilDone();
    intake_bottom.move(200);
    intake_middle.move(200);
    intake_top.move(200);
    pros::delay(3500);
    intake_top.move(0);
    little_will.extend();
    chassis.moveToPoint(-46.75,-4,2000,{.maxSpeed=60, .minSpeed=20, .earlyExitRange=4});
    chassis.moveToPoint(-46.75,-16,2000,{.maxSpeed=40,.minSpeed=20});
    pros::delay(1500);
    ramp1.retract();
    chassis.moveToPoint(-12.5,30,3000,{.forwards=false,.maxSpeed=60});
    chassis.waitUntilDone();
    intake_bottom.move(200);
    intake_middle.move(200);
    intake_top.move(200);
    little_will.retract();
    pros::delay(1000);
    int counter = 0;
    while(counter<100){
        intake_bottom.move(200);
        intake_middle.move(200);
        intake_top.move(200);
        if(intake_optical.get_hue() > 100 && intake_optical.get_hue() < 300){
            counter=0;
        }
        else if (intake_optical.get_hue() > 0 && intake_optical.get_hue() < 20){
            break;
        }
        else{
            counter++;
        }
        pros::delay(20);

    }
    intake_bottom.move(-200);
    intake_middle.move(-200);
    intake_top.move(-200);
    pros::delay(5000);

}

void mintest() {
    chassis.setBrakeMode(MOTOR_BRAKE_BRAKE);
    chassis.setPose(0,0,0);
    // Move to x: 20 and y: 15, and face heading 90. Timeout set to 4000 ms
    chassis.moveToPoint(0, 24, 10000, {.maxSpeed = 25});
    chassis.waitUntilDone();
    pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
        pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
        pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
    pros::delay(5000);
}

void awpauto(){
    chassis.setBrakeMode(MOTOR_BRAKE_BRAKE);
    chassis.setPose(0,0,0);
    intake_bottom.move(200);
    intake_middle.move(200);
    chassis.moveToPoint(-24,24,2000,{.maxSpeed=60,.minSpeed=40});
    chassis.turnToHeading(45,1000,{.maxSpeed=80});
    chassis.moveToPoint(-14,32, 1500,{.maxSpeed=80});
    chassis.waitUntilDone();
    intake_bottom.move(-200);
    intake_middle.move(-200);
    pros::delay(500);
    intake_bottom.move(200);
    intake_middle.move(200);
    chassis.moveToPoint(-24,24,1500,{.forwards=false, .maxSpeed=80});
    chassis.waitUntilDone();
    chassis.turnToHeading(90,1000,{.maxSpeed=60});
    chassis.waitUntilDone();
    chassis.moveToPoint(4,8,2000,{.maxSpeed=80,.minSpeed=60,.earlyExitRange=4});
    chassis.moveToPoint(28,36,2000,{.maxSpeed=60,.minSpeed=40});
    chassis.moveToPoint(24,20,2000,{.forwards=false,.maxSpeed=80});
    chassis.turnToHeading(135,1000,{.maxSpeed=80});
    chassis.moveToPoint(10,29,2000,{.forwards=false,.maxSpeed=50});
    chassis.waitUntilDone();
    intake_bottom.move(200);
    intake_middle.move(200);
    intake_top.move(200);
    pros::delay(500);
    intake_top.move(0);
    chassis.moveToPoint(46,0,2000,{.maxSpeed=80});
    chassis.turnToHeading(178,1000,{.maxSpeed=80});
    chassis.waitUntilDone();
    little_will.extend();
    pros::delay(250);
    chassis.moveToPoint(45.5,-24,1000,{.maxSpeed=40,.minSpeed=20});
    chassis.waitUntilDone();
    pros::delay(1000);
    chassis.moveToPoint(45.,18,2000,{.forwards=false,.maxSpeed=60,.minSpeed=30});
    ramp1.extend();
    chassis.waitUntilDone();
    intake_bottom.move(200);
    intake_middle.move(200);
    intake_top.move(200);
    int counter = 0;
    while(counter<100){
        intake_bottom.move(200);
        intake_middle.move(200);
        intake_top.move(200);
        if(intake_optical.get_hue() > 100 && intake_optical.get_hue() < 300){
            counter=0;
        }
        else if (intake_optical.get_hue() > 0 && intake_optical.get_hue() < 20){
            break;
        }
        else{
            counter++;
        }
        pros::delay(20);

    }
    intake_bottom.move(-200);
    intake_middle.move(-200);
    intake_top.move(-200);
    pros::delay(5000);





}

void skillsauto(){
    chassis.setBrakeMode(MOTOR_BRAKE_HOLD);
    chassis.setPose(-48,0,90);
    pros::delay(1500);
    intake_bottom.move(200);
    intake_middle.move(200);
    chassis.moveToPoint(-20,28,5000,{.maxSpeed=60});
    chassis.waitUntilDone();
    pros::delay(500);

    chassis.moveToPoint(-26,24,5000,{.forwards=false, .maxSpeed=60, .minSpeed=40});
    chassis.waitUntilDone();
    pros::delay(100);

    chassis.turnToHeading(135,5000,{.maxSpeed=60});
    chassis.waitUntilDone();
    pros::delay(100);    

    chassis.moveToPoint(-17.5,13.5, 2000,{.maxSpeed=60});
    chassis.waitUntilDone();
    intake_bottom.move(-200);
    intake_middle.move(-200);
    pros::delay(1500);
    intake_bottom.move(0);
    intake_middle.move(0);
    intake_top.move(0);  

    chassis.moveToPoint(-24,24,2000,{.forwards=false, .maxSpeed=60});
    chassis.waitUntilDone();
    chassis.turnToHeading(120+90,1000,{.maxSpeed=60});
    chassis.waitUntilDone();

    chassis.moveToPoint(-40,-4,2000,{.maxSpeed=60});
    intake_bottom.move(200);
    intake_middle.move(200);

    chassis.moveToPoint(-12,-28,2000,{.maxSpeed=60});
    chassis.waitUntilDone();
    pros::delay(500);

    // chassis.moveToPoint(-31,-17,2000,{.forwards=false,.maxSpeed=60});
    // chassis.waitUntilDone();
    // pros::delay(100);

    // chassis.turnToHeading(225,1000,{.maxSpeed=60});
    // chassis.waitUntilDone();
    // pros::delay(100);

    // chassis.moveToPoint(-23,-8,2000,{.forwards=false,.maxSpeed=50});
    // chassis.waitUntilDone();
    // intake_bottom.move(200);
    // intake_middle.move(200);
    // intake_top.move(80);
    // pros::delay(2000);
    // intake_bottom.move(0);
    // intake_middle.move(0);
    // intake_top.move(0);

    // chassis.moveToPoint(47,-4,2000,{.maxSpeed=60});
    // chassis.waitUntilDone();
    // pros::delay(100);
    // chassis.turnToHeading(178,1000,{.maxSpeed=60});
    // chassis.waitUntilDone();
    // little_will.extend();
    // pros::delay(500);
    // intake_bottom.move(200);
    // intake_middle.move(200);
    // intake_top.move(0);
    // chassis.moveToPoint(46,-12,1000,{.maxSpeed=30,.minSpeed=10});
    // chassis.waitUntilDone();
    // pros::delay(2000);
    // ramp1.extend();
    // chassis.moveToPoint(47,18,2000,{.forwards=false,.maxSpeed=60});
    // ramp1.extend();
    // chassis.waitUntilDone();
    // intake_top.move(200);
    // pros::delay(2000);
    // intake_bottom.move(0);
    // intake_middle.move(0);
    // intake_top.move(0);
    // little_will.retract();
    // chassis.moveToPoint(45,0,2000,{.maxSpeed=60});
    // ramp1.retract();
    // chassis.waitUntilDone();

    // pros::delay(100);
    // chassis.turnToHeading(-90,2000,{.maxSpeed=60});
    // chassis.waitUntilDone();
    // pros::delay(100);
    // chassis.moveToPoint(-46.25,0,5000,{.maxSpeed=100});
    // chassis.waitUntilDone();
    // pros::delay(100);
    // chassis.turnToHeading(180,2000,{.maxSpeed=60});
    // chassis.waitUntilDone();
    // pros::delay(100);

    // little_will.extend();
    // pros::delay(250);
    // chassis.moveToPoint(-48,-12,1000,{.maxSpeed=60});
    // chassis.waitUntilDone();
    // pros::delay(2000);
    // chassis.moveToPoint(-48,18,2000,{.forwards=false,.maxSpeed=60});
    // ramp1.extend();
    // chassis.waitUntilDone();
    // intake_bottom.move(200);
    // intake_middle.move(200);
    // intake_top.move(200);
    // pros::delay(2000);
    // intake_top.move(0);
    // chassis.moveToPoint(-24,24,2000,{.maxSpeed=60});
    // ramp1.retract();
    // chassis.waitUntilDone();
    // chassis.moveToPoint(-24,80,2000,{.maxSpeed=60});
    // chassis.turnToHeading(135,2000,{.maxSpeed=60});
    






}

void midauto(){
    chassis.setBrakeMode(MOTOR_BRAKE_BRAKE);
    chassis.setPose(0,0,0);
    intake_bottom.move(200);
    intake_middle.move(200);
    chassis.moveToPoint(-24,24,2000,{.maxSpeed=60,.minSpeed=40});
    chassis.turnToHeading(45,1000,{.maxSpeed=60});
    chassis.moveToPoint(-14,32, 1500,{.maxSpeed=60});
    chassis.waitUntilDone();
    intake_bottom.move(-200);
    intake_middle.move(-200);
    pros::delay(1000);
    intake_bottom.move(200);
    intake_middle.move(200);
    chassis.moveToPoint(-24,24,1500,{.forwards=false, .maxSpeed=60, .minSpeed=40});
    chassis.waitUntilDone();
    chassis.turnToHeading(90,1000,{.maxSpeed=60, .minSpeed=40});
    chassis.waitUntilDone();
    chassis.moveToPoint(4,8,2000,{.maxSpeed=60,.minSpeed=60,.earlyExitRange=4});
    chassis.moveToPoint(28,36,2000,{.maxSpeed=60,.minSpeed=40});
    chassis.moveToPoint(24,20,2000,{.forwards=false,.maxSpeed=60,.minSpeed=40});
    chassis.turnToHeading(135,1000,{.maxSpeed=60});
    chassis.moveToPoint(13,29,2000,{.forwards=false,.maxSpeed=50});
    chassis.waitUntilDone();
    intake_bottom.move(200);
    intake_middle.move(200);
    intake_top.move(200);
    pros::delay(1000);
    intake_top.move(0);
}

void autonomous() {
    chassis.setBrakeMode(MOTOR_BRAKE_BRAKE);
    chassis.setPose(0,0,0);
    switch(autonvalue){
        case 0:
            rightauto();
            break;
        case 1:
            leftauto();
            break;
        case 2:
            rightlongauto();
            break;
        case 3:
            leftlongauto();
            break;
        case 4:
            midauto();
            break;
        case 5:
            awpauto();
            break;
        case 6:
            pidtestpath();
            break;
        case 7:
            skillsauto();
            break;
        default:
            rightauto();
            break;
    }
}   




void opcontrol() {

    //pros::Task screenTask(screentask);

	bool intake_toggle = false;
	
	bool ramp_toggle = false;

	bool little_will_toggle = false;

    bool rake_toggle = false;



	while (true) {

         // print robot location to the brain screen
        pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
        pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
        pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading

        //pros::lcd::print(6, "X: %f", chassis.getPose().x-dX); // x
        //pros::lcd::print(7, "Y: %f", chassis.getPose().y-dY); // y

        
        // log position telemetry
        lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
        // delay to save resources
        pros::delay(50);


		int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        chassis.arcade(leftY, rightX);

		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)&&master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){
            autonomous();
        }	

		
		//score
		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
			intake_top.move(127);
		}
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)){
            intake_top.move(60);
        }     
        else{
			intake_top.move(0);
		}
			
        //intake    
        if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
            intake_bottom.move(-260);
			intake_middle.move(-260);
			intake_top.move(-260);
			
        }
        else{
			if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
				intake_bottom.move(127);
				intake_middle.move(127);
			}else{
				intake_bottom.move(0);
				intake_middle.move(0);
			}
        }
        
        // else{
		// 	if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)){
		// 		intake_toggle = !intake_toggle;
		// 	}
		// 	if(intake_toggle){
		// 		intake_bottom.move(top_speed);
		// 		intake_middle.move(top_speed);
		// 	}else{
		// 		intake_bottom.move(0);
		// 		intake_middle.move(0);
		// 	}
        // }


		//ramp
		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)){
            ramp_toggle = !ramp_toggle;
			if(ramp_toggle){
				ramp1.extend();
			}else{
				ramp1.retract();
			}
        }


		//will
		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)){
           	little_will_toggle = !little_will_toggle;
            if(rake_toggle){
                rake_toggle=false;
                rake.retract();
            }
			if(little_will_toggle){
				little_will.extend();
			}else{
				little_will.retract();
			}
			
        }
        if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)){
           	rake_toggle = !rake_toggle;
            if(little_will_toggle){
                little_will_toggle=false;
                little_will.retract();
            }
			if(rake_toggle){
				rake.extend();
			}else{
				rake.retract();
			}
			
        }
	
        
        
		pros::delay(20);                               // Run for 20 ms then update
	}
}