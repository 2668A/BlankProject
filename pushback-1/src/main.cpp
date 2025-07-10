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
 
//DRIVETRAIN
//a
inline pros::Motor intake_bottom(6,pros::v5::MotorGears::blue);
inline pros::Motor intake_middle(-7,pros::v5::MotorGears::blue);
inline pros::Motor intake_top(8,pros::v5::MotorGears::blue);

pros::Rotation horizontal_encoder(15);

pros::adi::Pneumatics ramp1('a', false);
pros::adi::Pneumatics little_will('b', false);
pros::Controller master(pros::E_CONTROLLER_MASTER);
// pros::adi::Pneumatics clamp2('a', false);
//-1, -2, 3
pros::MotorGroup left_motors({-11, 12, -13}, pros::MotorGearset::blue,pros::v5::MotorEncoderUnits::deg); // left motors use 600 RPM cartridges
pros::MotorGroup right_motors({17, -18, 19}, pros::MotorGearset::blue,pros::v5::MotorEncoderUnits::deg); // right motors use 200 RPM cartridges

pros::Imu imu(5);

lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_2, -3.5);




lemlib::Drivetrain drivetrain(&left_motors, // left motor group
                              &right_motors, // right motor group
                              11.5, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              450, // drivetrain rpm is 360
                              2 // horizontal drift is 2 (for now)
);
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            &horizontal_tracking_wheel, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);
//used to be 50

lemlib::ControllerSettings lateral_controller(14, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              40, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              250, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(1.8, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              2, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              5, // large error range, in degrees
                                              250, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);


lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.000 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.000 // expo curve gain
);

lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors, // odometry sensors
						&throttleCurve, &steerCurve
);



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

void initialize() {

    left_motors.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    right_motors.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);


	intake_top.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	

	pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    

    // thread to for brain screen and position logging
    //pros::Task screenTask(screentask);
}


void disabled() {}

void competition_initialize() {}



void pidtestturn(){
	chassis.setBrakeMode(MOTOR_BRAKE_BRAKE);
	imu.set_heading(0);
	double initpos = imu.get_heading();
	chassis.setPose(0,0,0);
	chassis.turnToHeading(90,1000);
	chassis.waitUntilDone();
	master.set_text(1,1,std::to_string(imu.get_heading()-initpos));
}

void pidtestdrive(){
	chassis.setBrakeMode(MOTOR_BRAKE_BRAKE);
	chassis.moveToPoint(0,24,3000,{.maxSpeed=100});
    chassis.waitUntilDone();
    pros::delay(250);
    chassis.moveToPoint(24,0,3000,{.maxSpeed=100});
    chassis.waitUntilDone();
    pros::delay(250);
    chassis.moveToPoint(0,0,3000,{.maxSpeed=100});
    chassis.waitUntilDone();
}

void autonomous() {
	pidtestdrive();
   
}   



void antijam(){

  while(true){
    
    
    if(intake_middle.get_target_velocity()>0 && intake_middle.get_actual_velocity()<50){
		intake_middle.move_velocity(-200);
		intake_bottom.move_velocity(-200);
		pros::delay(750);
		intake_middle.move_velocity(200);
		intake_bottom.move_velocity(200);
	}

    pros::delay(20);
  }
}

void opcontrol() {

	bool intake_toggle = false;
	
	bool ramp_toggle = false;

	bool little_will_toggle = false;

    int top_speed = 450;



	while (true) {
        pros::lcd::print(5, "intake: %d", top_speed);

		int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        chassis.arcade(leftY, rightX);

		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)&&master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){
            autonomous();
        }	

		/*
        if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)){
            top_speed += 20;
        }
		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)){
            top_speed -= 20;
        }
		*/	
		//score
		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
			intake_top.move(127);
		}else{
			intake_top.move(0);
		}
			
        //intake    
        if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
            intake_bottom.move(-260);
			intake_middle.move(-260);
			intake_top.move(-260);
			
        }else{
			if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)){
				intake_toggle = !intake_toggle;
			}
			if(intake_toggle){
				intake_bottom.move(top_speed);
				intake_middle.move(top_speed);
			}else{
				intake_bottom.move(0);
				intake_middle.move(0);
			}
        }
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
			if(little_will_toggle){
				little_will.extend();
			}else{
				little_will.retract();
			}
			
        }

	
        
        
		pros::delay(20);                               // Run for 20 ms then update
	}
}