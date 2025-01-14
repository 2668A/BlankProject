#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({1, -2, -3},
                            pros::MotorGearset::blue); // left motor group - ports 4 (reversed), 5, 6 
pros::MotorGroup rightMotors({-4, 5, 6}, pros::MotorGearset::blue); // right motor group - ports 1, 7 (reversed), 9 (reversed)

// Inertial Sensor on port 10
pros::Imu imu(10);

// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 14, reversed
pros::Rotation horizontalEnc(-14);
// vertical tracking wheel encoder. Rotation sensor, port 20, not reversed
pros::Rotation verticalEnc(20);
// horizontal tracking wheel. 2" diameter, 0.75" offset, back of the robot (negative)
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_2, -0.75);
// vertical tracking wheel. 2" diameter, 1.75" offset, left of the robot (negative)
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_2, -1.75);

inline pros::Rotation ArmSensor(7);
inline pros::Motor Intake1(8);
inline pros::Motor Intake2(12);
inline pros::Motor Arm(13);
inline pros::Rotation Horizontal(14);
inline pros::Distance Intakedist(17);
inline pros::Optical Intakecolor(18);
inline pros::Distance Frontdist(19);
inline pros::Rotation Vertical(20);
inline pros::adi::DigitalOut Clamp('A');
inline pros::adi::DigitalOut Doink('B');
inline pros::adi::DigitalOut Lifter('C');

lemlib::PID armPid(0.0175,0,0);



// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              12.5, // 12.5 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
                              360, // drivetrain rpm is 360
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(10, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            3, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(10,// proportional gain (kP)
                                             0, // integral gain (kI)
                                             100, // derivative gain (kD)
                                             0, // anti windup
                                             0, // small error range, in degrees
                                             0, // small error range timeout, in milliseconds
                                             0, // large error range, in degrees
                                             0, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel
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
    chassis.moveToPose(20, 15, 90, 4000);
    // Move to x: 0 and y: 0 and face heading 270, going backwards. Timeout set to 4000ms
    chassis.moveToPose(0, 0, 270, 4000, {.forwards = false});
    // cancel the movement after it has traveled 10 inches
    chassis.waitUntil(10);
    chassis.cancelMotion();
    // Turn to face the point x:45, y:-45. Timeout set to 1000
    // dont turn faster than 60 (out of a maximum of 127)
    chassis.turnToPoint(45, -45, 1000, {.maxSpeed = 60});
    // Turn to face a direction of 90º. Timeout set to 1000
    // will always be faster than 100 (out of a maximum of 127)
    // also force it to turn clockwise, the long way around
    chassis.turnToHeading(90, 1000, {.direction = AngularDirection::CW_CLOCKWISE, .minSpeed = 100});
    // Follow the path in path.txt. Lookahead at 15, Timeout set to 4000
    // following the path with the back of the robot (forwards = false)
    // see line 116 to see how to define a path
    chassis.follow(example_txt, 15, 4000, false);
    // wait until the chassis has traveled 10 inches. Otherwise the code directly after
    // the movement will run immediately
    // Unless its another movement, in which case it will wait
    chassis.waitUntil(10);
    pros::lcd::print(4, "Traveled 10 inches during pure pursuit!");
    // wait until the movement is done
    chassis.waitUntilDone();
    pros::lcd::print(4, "pure pursuit finished!");
}




void pidtest(){
    chassis.turnToHeading(90,700);
}








/**
 * Runs in driver control
 */
void opcontrol() {
    // controller
    // loop to continuously update motors

    bool clampstate=0;
  Clamp.set_value(false);

  Arm.move_velocity(0);
  Arm.set_brake_mode(MOTOR_BRAKE_HOLD);
  Arm.set_encoder_units(MOTOR_ENCODER_DEGREES);
  ArmSensor.reset();

  bool lifterstate=0;
  Lifter.set_value(false);

  bool doinkstate=0;
  Doink.set_value(false);

  int colorstate=0; //0 is none, -1 is red, 1 is blue
  Intakecolor.set_led_pwm(100);

  double armtarget=35500;

    while (true) {
        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        // move the chassis with curvature drive
        chassis.arcade(leftY, rightX);
        // delay to save resources
        pros::delay(10);

        if(controller.get_digital_new_press(DIGITAL_LEFT)&&controller.get_digital_new_press(DIGITAL_B)){
            pidtest();
        }

        int angle_reading = ArmSensor.get_position();
            if (0<=angle_reading && angle_reading<18000){
                angle_reading=36000-angle_reading;
        }

        //Arm 1button control
        if (controller.get_digital_new_press(DIGITAL_L2)){
            if (armtarget==35500){
                armtarget=33300;
            }
            else if (armtarget==33300){
                armtarget=22000;
            } 
            else if (armtarget==22000){
                armtarget=35500;
            }
            else{
                armtarget=35500;
            }
        }

        Arm.move_velocity(armPid.update(armtarget-angle_reading));

        // Intake Control
        if (controller.get_digital(DIGITAL_R1)){
            Intake1.move_velocity(-200);
            Intake2.move_velocity(120);
        }
        else if (controller.get_digital(DIGITAL_R2)){
            Intake1.move_velocity(200); 
            Intake2.move_velocity(-120);
        }
        else{
            Intake1.move_velocity(0);
            Intake2.move_velocity(0);
        }
        
        if (controller.get_digital_new_press(DIGITAL_L1)){
            if (clampstate==1){
                clampstate=0;
            }
            else{
                clampstate=1;
            }
            Clamp.set_value(clampstate);
        }

        //intake lifter
        if (controller.get_digital_new_press(DIGITAL_X)){
            if (lifterstate==1){
                lifterstate=0;
            }
            else{
                lifterstate=1;
            }
            Lifter.set_value(lifterstate);
        }



        //Doinker
        if (controller.get_digital_new_press(DIGITAL_A)){
            if (doinkstate==1){
                doinkstate=0;
            }
            else{
                doinkstate=1;
            }
            Doink.set_value(doinkstate);
        }
    }
}
