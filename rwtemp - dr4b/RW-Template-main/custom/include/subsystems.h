#include "vex.h"
#include "pid.h"

void autoToggle(vex::color Target, double speed);
void toTarget(double targetPosition);
void liftLoop();
void toTargetBlocking(double targetPosition);
void toggleLoop(bool r2, bool button_y, bool button_a);