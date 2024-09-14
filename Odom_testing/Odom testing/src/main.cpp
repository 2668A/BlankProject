/* ---------------------------------------------------------------------------- */
/*                                                                              */                 

/*                                                                              */                 
/* ---------------------------------------------------------------------------- */

#include "vex.h"

using namespace vex;

// Robot configuration code.

// Brain should be defined by default
brain Brain;
motor FrontLeft = motor (PORT18, ratio6_1, true);
motor MiddleLeft = motor (PORT17, ratio6_1, true);
motor BackLeft = motor (PORT19, ratio6_1, false);

motor FrontRight = motor (PORT13, ratio6_1, false);
motor MiddleRight = motor (PORT15, ratio6_1, false);
motor BackRight = motor (PORT12, ratio6_1, true);

inertial DrivetrainInertial = inertial (PORT11);
rotation L_tracking = rotation (PORT10);
rotation R_tracking = rotation (PORT1);
rotation S_tracking = rotation (PORT9);
gps GPS14 = gps (PORT14, 4.5, 12, inches, 180);

controller Controller1 = controller(primary);
motor_group LeftDrive = motor_group (FrontLeft, MiddleLeft, BackLeft);
motor_group RightDrive = motor_group (FrontRight, MiddleRight, BackRight);
smartdrive Drivetrain = smartdrive (LeftDrive, RightDrive, DrivetrainInertial, 319.19, 320, 266.7, mm, 0.6);

//gps GPS14 = gps (PORT14, 3,4.75, inches, 180);
//odometry variables
double wheel_radius = 1.0; //tracking wheel radius
double deg_start = 180.0;
double theta_start = deg_start * M_PI / 180;
double x_start = 48.0;
double y_start = 0.0;
//double gps_tracking_x_offset = 0;
//double gps_tracking_y_offset = 8.5;

double L_encode_radius = 5.75;
double R_encode_radius = 7;
double S_encode_radius = 5.5;

double L_pos = 0.0;
double R_pos = 0.0;
double S_pos = 0.0;

double L_prev_pos = 0.0;
double R_prev_pos = 0.0;
double S_prev_pos = 0.0;

double delta_L_pos = 0.0;
double delta_R_pos = 0.0;
double delta_S_pos = 0.0;

double avg_theta = 0.0;
double delta_theta = 0.0;
double prev_theta = theta_start;
double absolute_heading = theta_start;

double delta_x_relative = 0.0;
double delta_y_relative = 0.0;
double delta_x_global = 0.0;
double delta_y_global = 0.0;

double x_pos = x_start;
double y_pos = y_start;

void drive_function ();
void odometry_position ();
void print_data ();
void Brake ();


int main() {
wait (20, msec);
Brake();
L_tracking.setPosition (0, degrees);
R_tracking.setPosition (0, degrees);
S_tracking.setPosition (0, degrees);
thread Driving = thread (drive_function);
thread odometry = thread (odometry_position);
thread print = thread (print_data);

}

void drive_function (){
  
  while (true){
    FrontLeft.setVelocity (Controller1.Axis3.position() + Controller1.Axis1.position(), percent);
    MiddleLeft.setVelocity (Controller1.Axis3.position() + Controller1.Axis1.position(), percent);
    BackLeft.setVelocity (Controller1.Axis3.position() + Controller1.Axis1.position(), percent);
    FrontRight.setVelocity (Controller1.Axis3.position() - Controller1.Axis1.position(), percent);
    MiddleRight.setVelocity (Controller1.Axis3.position() - Controller1.Axis1.position(), percent);
    BackRight.setVelocity (Controller1.Axis3.position() - Controller1.Axis1.position(), percent);
    FrontLeft.spin (forward);
    MiddleLeft.spin (forward);
    BackLeft.spin (forward);
    FrontRight.spin (forward);
    MiddleRight.spin (forward);
    BackRight.spin (forward);
    wait (5, msec);
  }
}
void Brake (){
  FrontLeft.setStopping (brake);
  MiddleLeft.setStopping (brake);
  BackLeft.setStopping (brake);
  FrontRight.setStopping (brake);
  MiddleRight.setStopping (brake);
  BackRight.setStopping (brake);
}

void print_data (){
  while (true){
    Controller1.Screen.setCursor (1,1);
    Controller1.Screen.print ("gx:%.2f", GPS14.xPosition (inches));
    Controller1.Screen.setCursor (2,1);
    Controller1.Screen.print ("drx:%.2f", delta_x_relative);
    Controller1.Screen.setCursor (3,1);
    Controller1.Screen.print ("x:%.2f", x_pos);
    Controller1.Screen.setCursor (1, 10);
    Controller1.Screen.print ("gy:%.2f",GPS14.yPosition (inches));
    Controller1.Screen.setCursor (2,10);
    Controller1.Screen.print("dry:%.2f", delta_y_relative);
    Controller1.Screen.setCursor (3,10);
    Controller1.Screen.print ("y:%.2f", y_pos);
    wait (10, msec);
  }
}

void odometry_position (){
  while (true){
  
  L_pos = -1 * L_tracking.position (degrees);
  R_pos = R_tracking.position (degrees);
  S_pos = - 1 * S_tracking.position (degrees);

  delta_L_pos = (L_pos - L_prev_pos) * M_PI / 180 * wheel_radius ;//degrees to inches
  delta_R_pos = (R_pos - R_prev_pos) * M_PI / 180 * wheel_radius ;
  delta_S_pos = (S_pos - S_prev_pos) * M_PI / 180 * wheel_radius ;

  L_prev_pos = L_pos;
  R_prev_pos = R_pos;
  S_prev_pos = S_pos;

  //formula from part 2 vid
  //not using because inconsistency and heading is really important
  //delta_theta  = (delta_L_pos - delta_R_pos) / (L_encode_radius + R_encode_radius);

  absolute_heading = DrivetrainInertial.rotation (degrees) * M_PI / 180 + theta_start;

  delta_theta = absolute_heading - prev_theta;

  prev_theta = absolute_heading;

  //formula from part 3 vid
  if (delta_theta == 0){
    delta_x_relative = delta_S_pos;
    delta_y_relative = delta_L_pos;
  }
  else{
    delta_x_relative = 2 * sin (delta_theta / 2) * ((delta_S_pos / delta_theta) + S_encode_radius);
    //taking average of l and r
    delta_y_relative = 1 * sin (delta_theta / 2) * ((delta_R_pos / delta_theta) + R_encode_radius) +  1 * sin (delta_theta / 2) * ((delta_L_pos / delta_theta) - L_encode_radius);
  }

  avg_theta = absolute_heading - (delta_theta /2);
//TODO: test by switching delta_y_relative and delta_x_relative
  delta_x_global =  -1 * (cos (avg_theta) * delta_x_relative) - (sin (avg_theta) * delta_y_relative);
  delta_y_global =  1 * (sin (avg_theta) * delta_x_relative) - (cos (avg_theta) * delta_y_relative);

  x_pos += delta_x_global;
  y_pos += delta_y_global;
  wait (10, msec);
  }
}
