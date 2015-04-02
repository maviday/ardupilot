/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
  landing logic
 */

/*
  update navigation for landing. Called when on landing approach or
  final flare
 */
static bool verify_land()
{
    // we don't 'verify' landing in the sense that it never completes,
    // so we don't verify command completion. Instead we use this to
    // adjust final landing parameters

    // If a go around has been commanded, we are done landing.  This will send
    // the mission to the next mission item, which presumably is a mission
    // segment with operations to perform when a landing is called off.
    // If there are no commands after the land waypoint mission item then
    // the plane will proceed to loiter about its home point.
    if (auto_state.commanded_go_around) {
        return true;
    }

    float height = height_above_target();

    // use rangefinder to correct if possible
    height -= rangefinder_correction();

    // calculate the sink rate.
    float sink_rate;
    Vector3f vel;
    if (ahrs.get_velocity_NED(vel)) {
        sink_rate = vel.z;
    } else if (gps.status() >= AP_GPS::GPS_OK_FIX_3D && gps.have_vertical_velocity()) {
        sink_rate = gps.velocity().z;
    } else {
        sink_rate = -barometer.get_climb_rate();        
    }

    // low pass the sink rate to take some of the noise out
    auto_state.land_sink_rate = 0.8f * auto_state.land_sink_rate + 0.2f*sink_rate;
    
    /* Set land_complete (which starts the flare) under 3 conditions:
       1) we are within LAND_FLARE_ALT meters of the landing altitude
       2) we are within LAND_FLARE_SEC of the landing point vertically
          by the calculated sink rate
       3) we have gone past the landing point and don't have
          rangefinder data (to prevent us keeping throttle on 
          after landing if we've had positive baro drift)
    */
    if (height <= g.land_flare_alt ||
        height <= auto_state.land_sink_rate * g.land_flare_sec ||
        (!rangefinder_state.in_range && location_passed_point(current_loc, prev_WP_loc, next_WP_loc)) ||
        (fabsf(auto_state.land_sink_rate) < 0.2f && !is_flying())) {

        if (!auto_state.land_complete) {
            if (!is_flying()) {
                gcs_send_text_fmt(PSTR("Flare crash detected: speed=%.1f"), gps.ground_speed());
            } else {
                gcs_send_text_fmt(PSTR("Flare %.1fm sink=%.2f speed=%.1f"), 
                                  height, auto_state.land_sink_rate, gps.ground_speed());
            }
        }
        auto_state.land_complete = true;

        if (gps.ground_speed() < 3) {
            // reload any airspeed or groundspeed parameters that may have
            // been set for landing. We don't do this till ground
            // speed drops below 3.0 m/s as otherwise we will change
            // target speeds too early.
            g.airspeed_cruise_cm.load();
            g.min_gndspeed_cm.load();
            aparm.throttle_cruise.load();
        }
    }

    /*
      when landing we keep the L1 navigation waypoint 200m ahead. This
      prevents sudden turns if we overshoot the landing point
     */
    struct Location land_WP_loc = next_WP_loc;
	int32_t land_bearing_cd = get_bearing_cd(prev_WP_loc, next_WP_loc);
    location_update(land_WP_loc,
                    land_bearing_cd*0.01f, 
                    get_distance(prev_WP_loc, current_loc) + 200);
    nav_controller->update_waypoint(prev_WP_loc, land_WP_loc);

    // check if we should auto-disarm after a confirmed landing
    disarm_if_autoland_complete();

    /*
      we return false as a landing mission item never completes

      we stay on this waypoint unless the GCS commands us to change
      mission item or reset the mission, or a go-around is commanded
     */
    return false;
}

/*
    If land_DisarmDelay is enabled (non-zero), check for a landing then auto-disarm after time expires
 */
static void disarm_if_autoland_complete()
{
    if (g.land_disarm_delay > 0 && 
        auto_state.land_complete && 
        !is_flying() && 
        arming.arming_required() != AP_Arming::NO &&
        arming.is_armed()) {
        /* we have auto disarm enabled. See if enough time has passed */
        if (hal.scheduler->millis() - auto_state.last_flying_ms >= g.land_disarm_delay*1000UL) {
            if (disarm_motors()) {
                gcs_send_text_P(SEVERITY_LOW,PSTR("Auto-Disarmed"));
            }
        }
    }
}



/*
  a special glide slope calculation for the landing approach

  During the land approach use a linear glide slope to a point
  projected through the landing point. We don't use the landing point
  itself as that leads to discontinuities close to the landing point,
  which can lead to erratic pitch control
 */
static void setup_landing_glide_slope(void)
{
        Location loc = next_WP_loc;

        // project a point 500 meters past the landing point, passing
        // through the landing point
        const float land_projection = 500;        
        int32_t land_bearing_cd = get_bearing_cd(prev_WP_loc, next_WP_loc);
        float total_distance = get_distance(prev_WP_loc, next_WP_loc);

        // height we need to sink for this WP
        float sink_height = (prev_WP_loc.alt - next_WP_loc.alt)*0.01f;

        // current ground speed
        float groundspeed = ahrs.groundspeed();
        if (groundspeed < 0.5f) {
            groundspeed = 0.5f;
        }

        // calculate time to lose the needed altitude
        float sink_time = total_distance / groundspeed;
        if (sink_time < 0.5f) {
            sink_time = 0.5f;
        }

        // find the sink rate needed for the target location
        float sink_rate = sink_height / sink_time;

        // the height we aim for is the one to give us the right flare point
        float aim_height = g.land_flare_sec * sink_rate;

        // don't allow the aim height to be too far above LAND_FLARE_ALT
        if (g.land_flare_alt > 0 && aim_height > g.land_flare_alt*2) {
            aim_height = g.land_flare_alt*2;
        }

        // time before landing that we will flare
        float flare_time = aim_height / SpdHgt_Controller->get_land_sinkrate();

        // distance to flare is based on ground speed, adjusted as we
        // get closer. This takes into account the wind
        float flare_distance = groundspeed * flare_time;
        
        // don't allow the flare before half way along the final leg
        if (flare_distance > total_distance/2) {
            flare_distance = total_distance/2;
        }

        // now calculate our aim point, which is before the landing
        // point and above it
        location_update(loc, land_bearing_cd*0.01f, -flare_distance);
        loc.alt += aim_height*100;

        // calculate slope to landing point
        float land_slope = (sink_height - aim_height) / total_distance;

        // calculate point along that slope 500m ahead
        location_update(loc, land_bearing_cd*0.01f, land_projection);
        loc.alt -= land_slope * land_projection * 100;

        // setup the offset_cm for set_target_altitude_proportion()
        target_altitude.offset_cm = loc.alt - prev_WP_loc.alt;

        // calculate the proportion we are to the target
        float land_proportion = location_path_proportion(current_loc, prev_WP_loc, loc);

        // now setup the glide slope for landing
        set_target_altitude_proportion(loc, 1.0f - land_proportion);

        // stay within the range of the start and end locations in altitude
        constrain_target_altitude_location(loc, prev_WP_loc);
}

/* 
   find the nearest landing sequence starting point (DO_LAND_START) and
   switch to that mission item.  Returns false if no DO_LAND_START
   available.
 */
static bool jump_to_landing_sequence(void) 
{
    uint16_t land_idx = mission.get_landing_sequence_start();
    if (land_idx != 0) {
        if (mission.set_current_cmd(land_idx)) {
            set_mode(AUTO);

            //if the mission has ended it has to be restarted
            if (mission.state() == AP_Mission::MISSION_STOPPED) {
                mission.resume();
            }

            gcs_send_text_P(SEVERITY_LOW, PSTR("Landing sequence begun."));
            return true;
        }            
    }

    gcs_send_text_P(SEVERITY_HIGH, PSTR("Unable to start landing sequence."));
    return false;
}

/*
  the height above field elevation that we pass to TECS
 */
static float tecs_hgt_afe(void)
{
    /*
      pass the height above field elevation as the height above
      the ground when in landing, which means that TECS gets the
      rangefinder information and thus can know when the flare is
      coming.
    */
    float hgt_afe;
    if (flight_stage == AP_SpdHgtControl::FLIGHT_LAND_FINAL ||
        flight_stage == AP_SpdHgtControl::FLIGHT_LAND_APPROACH) {
        hgt_afe = height_above_target();
        hgt_afe -= rangefinder_correction();
    } else {
        // when in normal flight we pass the hgt_afe as relative
        // altitude to home
        hgt_afe = relative_altitude();
    }
    return hgt_afe;
}

/*
 * in order to always land into the wind, this function will update the location of
 *  this command to enforce an always-into-the-wind landing approach.
 *
 * returns true if the cmd location is changed, false otherwise
 */
static bool handle_bidirectional_landing(AP_Mission::Mission_Command& cmd)
{
    AP_Mission::Mission_Command next_cmd;

    // check if current cmd is nav_waypoint, and next is land, and bi-directional is enabled
    if (cmd.id != MAV_CMD_NAV_WAYPOINT ||
        g.land_bidirectional_thresh <= 0 ||
        mission.get_next_nav_cmd(mission.get_current_nav_index()+1, next_cmd) == false ||
        next_cmd.id != MAV_CMD_NAV_LAND)
    {
        // this is not the waypoint segment you're looking for!
        return false;
    }

    int32_t approach_bearing_cd = mission.get_next_ground_course_cd(-1);
    if (approach_bearing_cd == -1) {
        // something wrong with the current wp+land bearing calc
        return false;
    }

    // convert from centi-degree to degrees
    float approach_bearing = approach_bearing_cd/100.0f;

    // get wind values
    Vector3f wind = ahrs.wind_estimate();
    float wind_length = wind.length();
    float wind_rad = atan2f(wind.y, wind.x);
    float wind_deg = degrees(wind_rad);

    // with wind: (wind_deg - approach_bearing) == 0   (wind dir matches)
    // into wind: (wind_deg - approach_bearing) == 180 (wind dir opposite)
    // with wind if (wind_dot_approach > 0)
    // perpendicular to wind if (wind_dot_approach == 0)
    // into wind if (wind_dot_approach < 0)
    float wind_dot_approach = wind_length * cos(radians(wind_deg - approach_bearing));

    // check if we're landing with the wind. Negative would mean into the wind.
    bool do_rotate_approach_Wp = (wind_dot_approach > 0);

    gcs_send_text_fmt(PSTR("BiDirectional Land dot product: %.2f, %d"), wind_dot_approach, do_rotate_approach_Wp);

    if (!do_rotate_approach_Wp) {
        // we're already landing into the wind or land waypoint param of wind threshold not met
        return false;
    }

    // everything checks out, go ahead and rotate the land approach by 180deg
    rotate_location_around_another_location(180, next_cmd.content.location, cmd.content.location);

    return true;
}


/*
 * rotate Location B around Location A (stationary).
 * bearing = 0 or 360 means location B does not change
 */
static void rotate_location_around_another_location(const float rotation_angle, const Location locA, Location& locB)
{
    // traverse locB to locA
    float bearing_cd = get_bearing_cd(locB, locA);
    float distance = get_distance(locA, locB);
    location_update(locB, bearing_cd / 100.0f, distance);

    // rotate locB by (rotation_angle-180) degrees. The -180 is because at this point
    // "0" means no change (keep traversing forward) which is an effective 180deg
    // reflection and 180 means go back where we started from
    float effective_rotation_angle = rotation_angle - 180;
    bearing_cd += (int32_t)(effective_rotation_angle * 100);
    bearing_cd = wrap_180_cd(bearing_cd);

    // traverse locB away from locA
    location_update(locB, bearing_cd / 100.f, distance);
}


