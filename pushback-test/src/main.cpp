#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({-11, 12, -13},
                            pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup rightMotors({17, -18, 19}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)

// Inertial Sensor on port 10
pros::Imu imu(5);

// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
pros::Rotation horizontalEnc(15);
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_2, 4.25);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              11.5, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              450, // drivetrain rpm is 360
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(15, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            50, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
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
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            &horizontal, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors

    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });
}

/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(example_txt); // '.' replaced with "_" to make c++ happy

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
void autonomous() {
    // Move to x: 20 and y: 15, and face heading 90. Timeout set to 4000 ms
    chassis.moveToPose(-24, 24, -90, 4000);
	pros::delay(2000);
	pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
        pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
        pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
    pros::delay(5000);
    // Move to x: 0 and y: 0 and face heading 270, going backwards. Timeout set to 4000ms
    //chassis.moveToPose(0, 0, 270, 4000, {.forwards = false});
	
    // cancel the movement after it has traveled 10 inches
    //chassis.waitUntil(10);
    //chassis.cancelMotion();
	// pros::delay(2000);
    // pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
    //     pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
    //     pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
    // pros::delay(5000);
	// Turn to face the point x:45, y:-45. Timeout set to 1000
    // dont turn faster than 60 (out of a maximum of 127)
    // chassis.turnToPoint(45, -45, 1000, {.maxSpeed = 60});
	// pros::delay(2000);
	// pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
    //     pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
    //     pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
    // pros::delay(5000);
    // Turn to face a direction of 90º. Timeout set to 1000
    // will always be faster than 100 (out of a maximum of 127)
    // also force it to turn clockwise, the long way around
    // chassis.turnToHeading(90, 1000, {.direction = AngularDirection::CW_CLOCKWISE, .minSpeed = 100});
	// pros::delay(2000);
	// pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
    //     pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
    //     pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
    // pros::delay(5000);
}

/**
 * Runs in driver control
 */
void opcontrol() {
    // controller
    // loop to continuously update motors
    while (true) {
		if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)&&controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){
            autonomous();
        }	
        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        // move the chassis with curvature drive
        chassis.arcade(leftY, rightX);
        // delay to save resources
        pros::delay(10);
    }
}