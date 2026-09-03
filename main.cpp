#include "vex.h"

using namespace vex;

// Brain and Controller
brain Brain;
controller Controller1;

// Motors - adjust motor cartridge/gearing as needed (blue = 200 RPM / 6:1)
motor FL(PORT3, ratio6_1, false);
motor FR(PORT4, ratio6_1, true);   // reversed - right side motors usually need to spin opposite
motor BL(PORT6, ratio6_1, false);
motor BR(PORT5, ratio6_1, true);   // reversed

// Joystick deadband - values within this range of center (0) are treated as
// 0 to filter out stick drift/controller noise instead of creeping the robot.
const int STICK_DEADBAND = 8;

// Simple helper functions (no <algorithm>/<cmath> needed)
int myAbs(int val) {
  if (val < 0) return -val;
  return val;
}

int myMax(int a, int b) {
  if (a > b) return a;
  return b;
}

int applyDeadband(int value) {
  if (myAbs(value) < STICK_DEADBAND) return 0;
  return value;
}

int main() {

  // Brake (instead of coast) so the drive stops immediately when the
  // sticks return to center rather than rolling to a stop.
  FL.setStopping(brake);
  FR.setStopping(brake);
  BL.setStopping(brake);
  BR.setStopping(brake);

  while (true) {

    // Get joystick values (-100 to 100), filtered through the deadband
    int y    = applyDeadband(Controller1.Axis3.position()); // Left stick, up/down (forward/back)
    int x    = applyDeadband(Controller1.Axis4.position()); // Left stick, left/right (strafe)
    int turn = applyDeadband(Controller1.Axis1.position()); // Right stick, left/right (rotate)

    // X-drive mixing formula for 45-degree mounted wheels
    int flPower = y + x + turn;
    int frPower = y - x - turn;
    int blPower = y - x + turn;
    int brPower = y + x - turn;

    // Find the largest magnitude so we can scale down proportionally
    // (prevents clipping/distortion when multiple inputs are maxed out)
    int maxMag = 100;
    maxMag = myMax(maxMag, myAbs(flPower));
    maxMag = myMax(maxMag, myAbs(frPower));
    maxMag = myMax(maxMag, myAbs(blPower));
    maxMag = myMax(maxMag, myAbs(brPower));

    if (maxMag > 100) {
      float scale = 100.0 / maxMag;
      flPower = flPower * scale;
      frPower = frPower * scale;
      blPower = blPower * scale;
      brPower = brPower * scale;
    }

    // Spin the motors
    FL.spin(forward, flPower, percent);
    FR.spin(forward, frPower, percent);
    BL.spin(forward, blPower, percent);
    BR.spin(forward, brPower, percent);

    wait(20, msec); // small delay to prevent hogging CPU
  }
}
