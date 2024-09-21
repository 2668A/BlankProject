#pragma once

#include "api.h"

// Your non-drivetrain motors, sensors, etc. should go here. Below are examples:
// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');

inline pros::Motor Intake(19);
inline pros::ADIDigitalOut Clamp('A');
inline pros::Motor Arm1(11);
inline pros::Motor Arm2(12);
inline pros::Rotation MiddleTracker(20);
inline pros::Rotation LeftTracker(17);
inline pros::Rotation RightTracker(18);
inline pros::ADIDigitalOut Lifter('B');
inline pros::Distance FrontDistance(13);