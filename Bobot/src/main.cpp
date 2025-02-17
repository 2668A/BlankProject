#include "main.h"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// Chassis constructor
ez::Drive chassis(
    // These are your drive motors, the first motor is used for sensing!
    {1, -2, -3},     // Left Chassis Ports (negative port will reverse it!)
    {-4, 5, 6},  // Right Chassis Ports (negative port will reverse it!)

    10,      // IMU Port
    3.25,  // Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)
    360);   // Wheel RPM = cartridge * (motor gear / wheel gear)

// Uncomment the trackers you're using here!
// - `8` and `9` are smart ports (making these negative will reverse the sensor)
//  - you should get positive values on the encoders going FORWARD and RIGHT
// - `2.75` is the wheel diameter
// - `4.0` is the distance from the center of the wheel to the center of the robot
ez::tracking_wheel vert1_tracker(-14, 1.95, 6.75);  // This tracking wheel is parallel lefgt to the drive wheels
ez::tracking_wheel vert2_tracker(20, 1.95, 6.75);   // This tracking wheel is parallel right to the drive wheels

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  // Print our branding over your terminal :D
  ez::ez_template_print();

  pros::delay(500);  // Stop the user from doing anything while legacy ports configure

  // Look at your horizontal tracking wheel and decide if it's in front of the midline of your robot or behind it
  //  - change `back` to `front` if the tracking wheel is in front of the midline
  //  - ignore this if you aren't using a horizontal tracker
  chassis.odom_tracker_left_set(&vert1_tracker);
  // Look at your vertical tracking wheel and decide if it's to the left or right of the center of the robot
  //  - change `left` to `right` if the tracking wheel is to the right of the centerline
  //  - ignore this if you aren't using a vertical tracker
  chassis.odom_tracker_right_set(&vert2_tracker);

  // Configure your chassis controls
  chassis.opcontrol_curve_buttons_toggle(true);   // Enables modifying the controller curve with buttons on the joysticks
  chassis.opcontrol_drive_activebrake_set(0.0);   // Sets the active brake kP. We recommend ~2.  0 will disable.
  chassis.opcontrol_curve_default_set(0.0, 0.0);  // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)

  // Set the drive to your own constants from autons.cpp!
  default_constants();

  // These are already defaulted to these buttons, but you can change the left/right curve buttons here!
  // chassis.opcontrol_curve_buttons_left_set(pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT);  // If using tank, only the left side is used.
  // chassis.opcontrol_curve_buttons_right_set(pros::E_CONTROLLER_DIGITAL_Y, pros::E_CONTROLLER_DIGITAL_A);

  // Autonomous Selector using LLEMU
  ez::as::auton_selector.autons_add({
      {"SKILLS AUTO\nSetup using 5-by on rear hs axle\nreset imu, arm, pneumatic before",skillsauto},
      {"RED LEFT SIDE GOAL\nSetup on edge 2 from left\nScores 1 tr, 3 nr",red_left_goal},
      {"RED LEFT SIDE STAKE\nSetup on outer edge 3 from left\nScores 2 tr, 2 nr",red_left_stake},
      {"RED RIGHT SIDE RUSH\nSetup on outer edge 1 from right\nScores 2 tr",red_right_rush},
      {"BLUE RIGHT SIDE GOAL\nSetup on edge 2 from right\nScores 1 tr, 3 nr",blue_right_goal},
      {"BLUE RIGHT SIDE STAKE\nSetup on outer edge 3 from right\nScores 2 tr, 2 nr",blue_right_stake},
      {"BLUE LEFT SIDE RUSH\nSetup on outer edge 1 from left\nScores 2 tr",blue_left_rush},
      {"Drive and Turn\n\nDrive forward, turn, come back", drive_and_turn},
      {"Pure Pursuit\nPure Pursuit test with odom", odom_pure_pursuit_example},
      {"Measure Offsets\n\nThis will turn the robot a bunch of times and calculate your offsets for your tracking wheels.", measure_offsets}
  });

  // Initialize chassis and auton selector  
  chassis.initialize();
  ez::as::initialize();
  master.rumble(chassis.drive_imu_calibrated() ? "." : "---");
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
  // . . .
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
void competition_initialize() {
  // . . .
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
void autonomous() {
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the current position, you can start at a specific position with this
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency

  /*
  Odometry and Pure Pursuit are not magic

  It is possible to get perfectly consistent results without tracking wheels,
  but it is also possible to have extremely inconsistent results without tracking wheels.
  When you don't use tracking wheels, you need to:
   - avoid wheel slip
   - avoid wheelies
   - avoid throwing momentum around (super harsh turns, like in the example below)
  You can do cool curved motions, but you have to give your robot the best chance
  to be consistent
  */

  ez::as::auton_selector.selected_auton_call();  // Calls selected auton from autonomous selector
}

/**
 * Simplifies printing tracker values to the brain screen
 */
void screen_print_tracker(ez::tracking_wheel *tracker, std::string name, int line) {
  std::string tracker_value = "", tracker_width = "";
  // Check if the tracker exists
  if (tracker != nullptr) {
    tracker_value = name + " tracker: " + util::to_string_with_precision(tracker->get());             // Make text for the tracker value
    tracker_width = "  width: " + util::to_string_with_precision(tracker->distance_to_center_get());  // Make text for the distance to center
  }
  ez::screen_print(tracker_value + tracker_width, line);  // Print final tracker text
}

/**
 * Ez screen task
 * Adding new pages here will let you view them during user control or autonomous
 * and will help you debug problems you're having
 */
void ez_screen_task() {
  while (true) {
    // Only run this when not connected to a competition switch
    if (!pros::competition::is_connected()) {
      // Blank page for odom debugging
      if (chassis.odom_enabled() && !chassis.pid_tuner_enabled()) {
        // If we're on the first blank page...
        if (ez::as::page_blank_is_on(0)) {
          // Display X, Y, and Theta
          ez::screen_print("x: " + util::to_string_with_precision(chassis.odom_x_get()) +
                               "\ny: " + util::to_string_with_precision(chassis.odom_y_get()) +
                               "\na: " + util::to_string_with_precision(chassis.odom_theta_get()),
                           1);  // Don't override the top Page line

          // Display all trackers that are being used
          screen_print_tracker(chassis.odom_tracker_left, "l", 4);
          screen_print_tracker(chassis.odom_tracker_right, "r", 5);
          screen_print_tracker(chassis.odom_tracker_back, "b", 6);
          screen_print_tracker(chassis.odom_tracker_front, "f", 7);
          
        }
      }
    }

    // Remove all blank pages when connected to a comp switch
    else {
      if (ez::as::page_blank_amount() > 0)
        ez::as::page_blank_remove_all();
    }

    pros::delay(ez::util::DELAY_TIME);
  }
}
pros::Task ezScreenTask(ez_screen_task);

/**
 * Gives you some extras to run in your opcontrol:
 * - run your autonomous routine in opcontrol by pressing DOWN and B
 *   - to prevent this from accidentally happening at a competition, this
 *     is only enabled when you're not connected to competition control.
 * - gives you a GUI to change your PID values live by pressing X
 */
void ez_template_extras() {
  // Only run this when not connected to a competition switch
  if (!pros::competition::is_connected()) {
    // PID Tuner
    // - after you find values that you're happy with, you'll have to set them in auton.cpp

    // Enable / Disable PID Tuner
    //  When enabled:
    //  * use A and Y to increment / decrement the constants
    //  * use the arrow keys to navigate the constants
    if (master.get_digital_new_press(DIGITAL_X))
      chassis.pid_tuner_toggle();

    // Trigger the selected autonomous routine
    if (master.get_digital(DIGITAL_B) && master.get_digital(DIGITAL_DOWN)) {
      pros::motor_brake_mode_e_t preference = chassis.drive_brake_get();
      autonomous();
      chassis.drive_brake_set(preference);
    }

    // Allow PID Tuner to iterate
    chassis.pid_tuner_iterate();
  }

  // Disable PID Tuner when connected to a comp switch
  else {
    if (chassis.pid_tuner_enabled())
      chassis.pid_tuner_disable();
  }
}





void move_arm(int input){
  Arm.move(input);
}



ez::PID armPid{0.02 ,0,0,0,"LBMech"};

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
  pros::delay(150);
  Intake1.move_velocity(0);
  Intake2.move_velocity(-50);
  pros::delay(400);
  Intake2.move_velocity(0);
}

void neutral_score(){
  double currentdist = Frontdist.get();
  chassis.drive_angle_set(0);
  chassis.pid_drive_set((currentdist-90.0)*0.0394,100);
  chassis.pid_wait();
  armPid.target_set(22000);
}

void alliance_load(){
  while(Intakedist.get()>50){
    Intake1.move_velocity(-200);
    Intake2.move_velocity(100);
    chassis.opcontrol_arcade_standard(ez::SPLIT);
    pros::delay(ez::util::DELAY_TIME);
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)){
      break;
    }
  }
  Intake1.move_velocity(0);
  Intake2.move_velocity(0);
}

void armtest(){
  Clamp.set_value(false);
  Arm.move_velocity(0);
  Arm.set_brake_mode(MOTOR_BRAKE_HOLD);
  Arm.set_encoder_units(MOTOR_ENCODER_DEGREES);
  Arm.tare_position();
  ArmSensor.reset();
  int angle_reading = 36000;
  while(angle_reading>32900){
    master.set_text(0,0,to_string(angle_reading));
    angle_reading = ArmSensor.get_position();
    if (-18000<=angle_reading && angle_reading<5000){
      angle_reading=36000+abs(angle_reading);
    }
    Arm.move_velocity(-100);
    pros::delay(50);
  }
  Arm.move_velocity(0);
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
void opcontrol() {
  // This is preference to what you like to drive on
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);

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

  while (true) {
    // Gives you some extras to make EZ-Template ezier
    //ez_template_extras();

    

    if (master.get_digital(DIGITAL_B) && master.get_digital(DIGITAL_LEFT)){
      autonomous();
    }
    else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)){
      armPid.target_set(13000);
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
    if (angle_reading<0|| angle_reading<1000){
      angle_reading=36000+angle_reading;
    }
    

    





    //Arm 1button control
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)){
      if (armPid.target==35500){
        armPid.target_set(33200);
      }
      else if (armPid.target==33200){
        armPid.target_set(20000);
      } 
      else if (armPid.target==20000){
        armPid.target_set(35500);
      }
      else if (armPid.target==30000){
        armPid.target_set(20000);
      }
      else{
        armPid.target_set(35500);
      }
    }

    move_arm(armPid.compute(angle_reading));


    //intake lifter
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)){
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
    //if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)&&master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)){
    //  neutral_load();
    //}

    //autoscore trust
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
      armPid.target_set(33200);
    }
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)){
      armPid.target_set(15000);
    }

    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)){
      armPid.target_set(30000);
    }

    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
