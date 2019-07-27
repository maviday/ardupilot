#include "mode.h"
#include "Rover.h"

void ModeManual::_exit()
{
    // clear lateral when exiting manual mode
    g2.motors.set_lateral(0);
}

void ModeManual::update()
{
    float desired_steering, desired_throttle, desired_lateral, brake = 0;
    get_pilot_desired_steering_and_throttle(desired_steering, desired_throttle);
    get_pilot_desired_lateral(desired_lateral);

    // if vehicle is balance bot, calculate actual throttle required for balancing
    if (rover.is_balancebot()) {
        rover.balancebot_pitch_control(desired_throttle);

    } else if (desired_throttle <= 0) {
        // with no throttle, add in a little brake just to have something
        brake = 10;
    }

    // set sailboat mainsail from throttle position
    g2.motors.set_mainsail(desired_throttle);

    // copy RC scaled inputs to outputs
    rover.set_throttle(desired_throttle);
    rover.set_brake(brake * attitude_control.get_brake_gain());
    g2.motors.set_steering(desired_steering, false);
    g2.motors.set_lateral(desired_lateral);
}
