#include "main.h"

/* Documentation */
// https://ez-robotics.github.io/EZ-Template/

// Chassis constructor, edit accordingly
ez::Drive chassis(
    // These are your drive motors, the first motor is used for sensing!
    {1,-2,-3},  // Right Chassis Ports (use negative numbers for reversed motors!)
    {-4,5,6},  // Left Chassis Ports, (use negative numbers for reversed motors!)
    

    10,          // IMU (inertial) port
    3.25,      // Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)
    360         // Wheel RPM
);






/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{
  ez::ez_template_print();

  pros::delay(500);  // Stop the user from doing anything while legacy ports configure

  // Configure your chassis controls

  // Enables modifying the controller curve with buttons on the joysticks
  chassis.opcontrol_curve_buttons_toggle(false);

  // Sets the active brake kP. We recommend ~2.  0 will disable.
  chassis.opcontrol_drive_activebrake_set(2);

  // Defaults for curve. If using tank, only the first parameter is used.
  // (Comment this line out if you have an SD card!) 
  chassis.opcontrol_curve_default_set(0, 0);

  // Set the constants using the function defined in autons.cpp
  default_constants();






  

  // Autonomous Selector

  ez::as::auton_selector.autons_add(
    {
    Auton("PID TTESTING",pid_test),
    Auton("RED Right Side", red_right),
    Auton("RED Left Side", red_left),
    Auton("BLUE Right Side\non edge between tiles 2 and 3\nwall riders between edges", blue_right),
    Auton("BLUE Left Side\non edge between tiles 2 and 3\nwall riders between edges", blue_left),
    Auton("SKILLS", skillsauto)
    }
  );


  // Initialize chassis and auton selector
  chassis.initialize();
  ez::as::initialize();    
  master.rumble(".");

  Arm.tare_position();
  ArmSensor.reset();
  ArmSensor.reset_position();
  //ArmSensor.set_reversed(true);
  ArmSensor.set_data_rate(50);
}







/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() 
{
  // Add your code here
}







/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() 
{
  // Add your code here
}







/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() 
{
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold. This helps autonomous consistency

  // Calls selected auton from autonomous selector
  ez::as::auton_selector.selected_auton_call();
}


void move_arm(int input){
  Arm.move(input);
}



ez::PID armPid{0.0175,0,0,0,"LBMech"};

void neutral_load(){
  while(Intakedist.get()>50){
    Intake1.move_velocity(-200);
    Intake2.move_velocity(100);
    chassis.opcontrol_arcade_standard(ez::SPLIT);
    pros::delay(ez::util::DELAY_TIME);
    int angle_reading = ArmSensor.get_position();
    if (0<=angle_reading && angle_reading<18000){
      angle_reading=36000-angle_reading;
    }
    move_arm(armPid.compute(angle_reading));
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)){
      break;
    }
  }
  pros::delay(500);
  Intake1.move_velocity(0);
  Intake2.move_velocity(0);
}

void neutral_score(){
  double currentdist = Frontdist.get();
  chassis.drive_angle_set(0);
  chassis.pid_drive_set((currentdist-90.0)*0.0394,100);
  chassis.pid_wait();
  armPid.target_set(22000);
}






/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

void opcontrol()
{
  pros::motor_brake_mode_e_t driver_preference_brake = MOTOR_BRAKE_HOLD;
  chassis.drive_brake_set(driver_preference_brake);

  bool clampstate=0;
  Clamp.set_value(false);

  Arm.move_velocity(0);
  Arm.set_brake_mode(MOTOR_BRAKE_HOLD);
  Arm.set_encoder_units(MOTOR_ENCODER_DEGREES);
  ArmSensor.reset();
  int armtarget=0;
  armPid.target_set(35500);

  bool lifterstate=0;
  Lifter.set_value(false);

  bool doinkstate=0;
  Doink.set_value(false);

  int colorstate=0; //0 is none, -1 is red, 1 is blue
  Intakecolor.set_led_pwm(100);


  while (true) 
  {

    if (master.get_digital(DIGITAL_B) && master.get_digital(DIGITAL_LEFT)){
      autonomous();
    }

    //driving
    chassis.opcontrol_arcade_standard(ez::SPLIT);
    
    
    
    
    // Intake Control
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
      Intake1.move_velocity(-200);
      Intake2.move_velocity(120);
    }
    else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
      Intake1.move_velocity(200); 
      Intake2.move_velocity(-120);
    }
    else{
      Intake1.move_velocity(0);
      Intake2.move_velocity(0);
    }
    




    // Clamp Control
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)){
      if (clampstate==1){
        clampstate=0;
      }
      else{
        clampstate=1;
      }
      Clamp.set_value(clampstate);
    }
    
    int angle_reading = ArmSensor.get_position();
    if (0<=angle_reading && angle_reading<18000){
      angle_reading=36000-angle_reading;
    }







    //Arm 1button control
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)){
      if (armPid.target==35500){
        armPid.target_set(33000);
      }
      else if (armPid.target==33000){
        armPid.target_set(22000);
      } 
      else if (armPid.target==22000){
        armPid.target_set(35500);
      }
      else{
        armPid.target_set(35500);
      }
    }

    move_arm(armPid.compute(angle_reading));


    //intake lifter
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)){
      if (lifterstate==1){
        lifterstate=0;
      }
      else{
        lifterstate=1;
      }
      Lifter.set_value(lifterstate);
    }



    //Doinker
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)){
      if (doinkstate==1){
        doinkstate=0;
      }
      else{
        doinkstate=1;
      }
      Doink.set_value(doinkstate);
    }

    //autoload trust
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)){
      neutral_load();
    }

    //autoscore trust
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)){
      neutral_score();
    }


    // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
    pros::delay(ez::util::DELAY_TIME);
  }
}