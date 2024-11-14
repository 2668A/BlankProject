#include "main.h"

/* Documentation */
// https://ez-robotics.github.io/EZ-Template/

// Chassis constructor, edit accordingly
ez::Drive chassis(
    // These are your drive motors, the first motor is used for sensing!
    {-11, 12, -13},  // Left Chassis Ports, (use negative numbers for reversed motors!)
    {14, -15, 1},  // Right Chassis Ports (use negative numbers for reversed motors!)

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
    Auton("Do Nothing\nPLEASE DONT ACTUALLY RUN THIS DURING COMPETITION", donothing),
    Auton("RED Left Side\nSetup on 3rd from left\nBack lined up with inner forward edge\nWall riders lined up with inner left edge", red_left),
    Auton("RED Right Side\nSetup on 2nd from right\nBack lined up with inner forward edge\nWall riders lined up with inner left edge", red_right),
    Auton("BLUE Right Side\nSetup on 3nd from right\nBack lined up with inner forward edge\nWall riders lined up with inner right edge", blue_right),
    Auton("BLUE Left Side\nSetup on 2nd from left\nBack lined up with inner forward edge\nWall riders lined up with inner right edge", blue_left),
    Auton("SKILLS auto\nsetup on 2nd from right\nFront drive hole on 8th hole\nwall rider between edges",skillsauto)
    }
  );

  // Initialize chassis and auton selector
  chassis.initialize();
  ez::as::initialize();
  master.rumble(".");
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
  Arm.tare_position();
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

ez::PID armPid{1,0,0,0,"LBMech"};

void neutral_load(){
  armPid.target_set(-130);
  while(armPid.exit_condition(Arm,true)==ez::RUNNING){
    move_arm(armPid.compute(Arm.get_position()));
    chassis.opcontrol_arcade_standard(ez::SPLIT);
    pros::delay(ez::util::DELAY_TIME);
  }
  while(ColorSorter.get_hue()>20 && ColorSorter.get_hue()<50){
    Intake.move_velocity(-250);
    chassis.opcontrol_arcade_standard(ez::SPLIT);
    pros::delay(ez::util::DELAY_TIME);
  }
  pros::delay(200);
  Intake.move_velocity(0);
  Intake.move_velocity(50);
  pros::delay(100);
  Intake.move_velocity(0);
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
  // This is preference to what you like to drive on
  // MOTOR_BRAKE_HOLD (recommended), MOTOR_BRAKE_BRAKE, MOTOR_BRAKE_COAST
  pros::motor_brake_mode_e_t driver_preference_brake = MOTOR_BRAKE_COAST;
  // chassis.opcontrol_tank();  //  Tank control
     // Standard split arcade USE THIS
  // chassis.opcontrol_arcade_standard(ez::SINGLE);  // Standard single arcade
  // chassis.opcontrol_arcade_flipped(ez::SPLIT);    // Flipped split arcade
  // chassis.opcontrol_arcade_flipped(ez::SINGLE);   // Flipped single arcade
  chassis.drive_brake_set(driver_preference_brake);
  bool clampstate=0;
  Clamp.set_value(false);
  bool lifterstate=0;
  Lifter.set_value(false);
  Arm.move_velocity(0);
  Arm.set_brake_mode(MOTOR_BRAKE_HOLD);
  Arm.set_encoder_units(MOTOR_ENCODER_DEGREES);
  int armtarget=0;
  int colorside = 0;
  ColorSorter.disable_gesture();
  //0 is off, -1 is allow red, 1 is allow blue
  master.set_text(0,0,"ALLOW ALL");
  while (true) 
  {
    chassis.opcontrol_arcade_standard(ez::SPLIT);
    // Intake Control
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
      Intake.move_velocity(-200);
    }
    else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
      Intake.move_velocity(200);   
    }
    else{
      Intake.move_velocity(0);
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
    
    // Arm Automatic Control
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)){
      armPid.target_set(-410);
      //scoring pos
    }
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
      armPid.target_set(-130);
      //loading pos
    }
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)){
      armPid.target_set(0);
      //off pos
    }
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)){
      armPid.target_set(-490);
      //pushdown
    }
    move_arm(armPid.compute(Arm.get_position()));

    //Arm 1button control
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)){
      if (armPid.target==0){
        armPid.target_set(-130);
      }
      else if (armPid.target==-130){
        armPid.target_set(-410);
      }
      else if (armPid.target==-410){
        armPid.target_set(-490);
      }
      else if (armPid.target==-490){
        armPid.target_set(0);
      }
      else{
        armPid.target_set(0);
      }
    }


    // Lifterr Control
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)){
      if (lifterstate==1){
        lifterstate=0;
      }
      else{
        lifterstate=1;
      }
      Lifter.set_value(lifterstate);
    }

    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){
      neutral_load();
    }

    //colorsort
    double colorhuedetect = ColorSorter.get_hue();
    if (colorside==-1){
      if(100.0<colorhuedetect && colorhuedetect<220.0){
        pros::delay(50);
        Intake.move_velocity(100);
        pros::delay(100);
        Intake.move_velocity(-200);
        pros::delay(100);
      }
    }
    if (colorside==1){
      if(0.0<colorhuedetect && colorhuedetect<20.0){
        pros::delay(50);
        Intake.move_velocity(100);
        pros::delay(100);
        Intake.move_velocity(-200);
        pros::delay(100);
      }
    }

    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)){
      if (colorside==0){
        colorside=-1; 
        master.set_text(0,0,"ALLOW RED");
      }
      else if(colorside==-1){
        colorside=1;
        master.set_text(0,0,"ALLOW BLUE");
      }
      else if(colorside==1){
        colorside=0;
        master.set_text(0,0,"ALLOW ALL");
      }
      
    }

    //TESTING ONLY
    if (master.get_digital(DIGITAL_A) && master.get_digital(DIGITAL_LEFT)){
      autonomous();
    }



    // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
    pros::delay(ez::util::DELAY_TIME);
  }
}