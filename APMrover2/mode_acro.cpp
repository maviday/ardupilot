#include "mode.h"
#include "Rover.h"

void ModeAcro::update()
{
    uint32_t now_ms = AP_HAL::millis();
    static uint32_t gcs_send_last[10] = {};

    // get speed forward
    float speed, desired_steering;
    if (!attitude_control.get_forward_speed(speed)) {
        float desired_throttle;
        // convert pilot stick input into desired steering and throttle
        get_pilot_desired_steering_and_throttle(desired_steering, desired_throttle);
        // no valid speed, just use the provided throttle
        rover.set_throttle(desired_throttle);
        if (desired_throttle > 0) {
            rover.set_brake(0.0f);
        } else {
            // add a little if no throttle
            rover.set_brake(rover.g2.attitude_control.get_brake_manual_pct() * attitude_control.get_brake_gain());
        }
    } else {
        // convert pilot stick input into desired steering and speed
        float desired_speed;
        get_pilot_desired_steering_and_speed(desired_steering, desired_speed);
        calc_throttle(desired_speed, true);
        if (now_ms - gcs_send_last[0] >= 500) {
            gcs_send_last[0] = now_ms;
            gcs().send_text(MAV_SEVERITY_DEBUG,"\n1A: desired_steering:%.2f, desired_speed:%.2f", desired_steering, desired_speed);
        }
    }

    float steering_out;

    // handle sailboats
    if (!is_zero(desired_steering)) {
        // steering input return control to user
        rover.g2.sailboat.clear_tack();
    }
    if (rover.g2.sailboat.tacking()) {
        // call heading controller during tacking

        steering_out = attitude_control.get_steering_out_heading(rover.g2.sailboat.get_tack_heading_rad(),
                                                                 g2.wp_nav.get_pivot_rate(),
                                                                 g2.motors.limit.steer_left,
                                                                 g2.motors.limit.steer_right,
                                                                 rover.G_Dt);
    } else {
        // convert pilot steering input to desired turn rate in radians/sec
        const float target_turn_rate = (desired_steering / 4500.0f) * radians(g2.acro_turn_rate);
        if (now_ms - gcs_send_last[1] >= 500) {
            gcs_send_last[1] = now_ms;
            gcs().send_text(MAV_SEVERITY_DEBUG,"1B: target_turn_rate:%.2f", target_turn_rate);
        }
        // run steering turn rate controller and throttle controller
        steering_out = attitude_control.get_steering_out_rate(target_turn_rate,
                                                              g2.motors.limit.steer_left,
                                                              g2.motors.limit.steer_right,
                                                              rover.G_Dt);
    }

    if (now_ms - gcs_send_last[2] >= 500) {
        gcs_send_last[2] = now_ms;
        gcs().send_text(MAV_SEVERITY_DEBUG,"1C: steering_out:%.2f", steering_out);
    }
    set_steering(steering_out * 4500.0f);
}

bool ModeAcro::requires_velocity() const
{
    return g2.motors.have_skid_steering()? false: true;
}

// sailboats in acro mode support user manually initiating tacking from transmitter
void ModeAcro::handle_tack_request()
{
    rover.g2.sailboat.handle_tack_request_acro();
}
