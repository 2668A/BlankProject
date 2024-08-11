#pragma once

#include "api.h"

// Your non-drivetrain motors, sensors, etc. should go here. Below are examples:
// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');

inline pros::adi::Ultrasonic sound_sensor ('A', 'B');
inline pros::Distance distance_back(12);
inline pros::Distance distance_front(20);