/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 * control_cruise.pde - init and run calls for cruise flight mode | by Julien Dubois
 * flight mode developed for "Walking with a drone - France 2014" project | by Benoit Pereira da silva
 * http://en.walkingworking.com/category/walk-work-outdoors-drone-2014/
 */

 
 #if MAIN_LOOP_RATE == 100
 // definitions for 100hz loop update rate          // 1km/h = 27.78cm/s
 # define VEL_INCREASE_RATE_MAX     (55.56f/100.0f) // number of cm/s the cruise velocity will be increased/decreased if the pitch_stick is maintained full range during 1s 
 #else
 // definitions for 400hz loop update rate
 # define VEL_INCREASE_RATE_MAX     (55.56f/400.0f) // number of cm/s the cruise velocity will be increased/decreased if the pitch_stick is maintained full range during 1s 
 #endif
 
 # define POT_OFFSET_RANGE           27000          // number of cdeg of the course_pot_offset range
 # define POT_OFFSET_FACTOR          (POT_OFFSET_RANGE/900)
 # define VEL_MAX                    166.7f         // maximum cruise velocity in cm/s  - 1km/h = 27.78cm/s
 
 // declare some function to keep compiler happy
static void cruise_init_course_target(); 

// declare variables
static bool offset_course_pot_enabled;   // course_pot is used to tune simple_yaw_angle
static int32_t yaw_target_offset;        // Centideg - Target offset angle set by course pot
static int32_t simple_yaw_angle;         // Centideg - N=0, E=9000, S=18000, O=27000
static int16_t course_pot_trim;          // RC course pot channel value on course_pot_enabling
static float des_vel_cms;                // desired cruise velocity in cm/s
static float vel_to_angle_factor;        // used to convert desired velocity (in cm/s) into a fake pilot_pitch input (in centidegrees)
 
// cruise_init - initialise cruise controller
static bool cruise_init(bool ignore_checks)
{
    if (GPS_ok() || ignore_checks) {

        // set target to current position
        wp_nav.init_loiter_target();

        // initialize vertical speeds
        pos_control.set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);

        // initialise altitude target to stopping point
        pos_control.set_target_to_stopping_point_z();
        
        // initialize the course to steer
        cruise_init_course_target(); 
        
        // initialise values and compute factors
        des_vel_cms = 0;

        // vel_to_angle_factor = 4500*2*(wp_nav._loiter_speed_cms/2 - WPNAV_LOITER_ACCEL_MIN) / (wp_nav._loiter_speed_cms * wp_nav._loiter_speed_cms)
        vel_to_angle_factor = 9000.0f * (wp_nav.get_loiter_speed_cms()/2.0f - 25.0f) / (wp_nav.get_loiter_speed_cms() * wp_nav.get_loiter_speed_cms());
        
        return true;
    }else{
        return false;
    }
}

// cruise_run - runs the cruise controller
// should be called at 100hz or more
static void cruise_run()
{
    float target_yaw_rate = 0;
    float target_climb_rate = 0;
    int16_t rc2_control_in = 0;

    // if not auto armed set throttle to zero and exit immediately
    if(!ap.auto_armed || !inertial_nav.position_ok()) {
        wp_nav.init_loiter_target();
        attitude_control.init_targets();
        attitude_control.set_throttle_out(0, false);
        return;
    }

    // process pilot inputs
    if (!failsafe.radio) {
        
        // check if course_pot is enabled (a switch on the radio inhibits the pot to let the user reposinion it at middle position without moving the copter's course)
        // and compute yaw_target_offset + simple_mode cos/sin_yaw if enabled
        if(g.rc_6.control_in < 100){
            // pot trim disabled
            if(offset_course_pot_enabled){
                // update simple_yaw_angle with current course only once
                simple_yaw_angle += yaw_target_offset;
                // set offset to 0
                yaw_target_offset = 0;
                offset_course_pot_enabled = false;
            }
        }else if(!offset_course_pot_enabled){  
            // get course_pot_trim value as the pot position reference (move pot right = move copter course to the right, and pot left = course to the left)
            course_pot_trim = g.rc_6.control_in; 
            offset_course_pot_enabled = true;
        }else{
            // scaling yaw_target_offset
            yaw_target_offset = POT_OFFSET_FACTOR*(g.rc_6.control_in-course_pot_trim);
             // add course pot offset
            float angle_rad = radians((simple_yaw_angle+yaw_target_offset)/100);
            // update simple_cos_yaw and simple_sin_yaw
            simple_cos_yaw = cosf(angle_rad);
            simple_sin_yaw = sinf(angle_rad);
        }
     
        // Debug condition
        if(ap.CH7_flag!=0) {
        
        // update desired cruise velocity and equivalent "fake stick input for loiter" from pilot stick input
        if(g.rc_2.control_in != 0){
            // increase/decrease desired cruise velocity if pilot is moving pitch stick
            des_vel_cms += VEL_INCREASE_RATE_MAX*(float)(g.rc_2.control_in)/4500.0f;
            // ensure we are in a correct range: v=[0;VEL_MAX]
            des_vel_cms = constrain_float(des_vel_cms, 0.0f, VEL_MAX);
        }
        // convert des_vel to a "fake stick angle" that will give this velocity through loiter code
        // we have to create rc2_control_in variable instead of updating g.rc_2.control_in because, between 2 loops, g.rc_2.control_in may not be updated if there's no "new_radio_frame"
        rc2_control_in = (int16_t)(des_vel_cms*vel_to_angle_factor);
        }else{
        rc2_control_in = g.rc_2.control_in;
        }
        
        // apply SIMPLE mode transform to pilot inputs
        update_simple_mode();

        // process pilot's roll and pitch input
        wp_nav.set_pilot_desired_acceleration(g.rc_1.control_in, rc2_control_in);

        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(g.rc_4.control_in);

        // get pilot desired climb rate
        target_climb_rate = get_pilot_desired_climb_rate(g.rc_3.control_in);

        // check for pilot requested take-off
        if (ap.land_complete && target_climb_rate > 0) {
            // indicate we are taking off
            set_land_complete(false);
            // clear i term when we're taking off
            set_throttle_takeoff();
        }
    } else {
        // clear out pilot desired acceleration in case radio failsafe event occurs and we do not switch to RTL for some reason
        wp_nav.clear_pilot_desired_acceleration();
    }

    // when landed reset targets and output zero throttle
    if (ap.land_complete) {
        wp_nav.init_loiter_target();
        attitude_control.init_targets();
        attitude_control.set_throttle_out(0, false);
    }else{
        // run loiter controller
        wp_nav.update_loiter();

        // call attitude controller
        attitude_control.angle_ef_roll_pitch_rate_ef_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), target_yaw_rate);

        // body-frame rate controller is run directly from 100hz loop

        // run altitude controller
        // vérifier comment l'option 7-8 Sonar intervient pour inhiber le sonar
        if (sonar_alt_health >= SONAR_ALT_HEALTH_MAX) {
            // if sonar is ok, use surface tracking
            target_climb_rate = get_throttle_surface_tracking(target_climb_rate, pos_control.get_alt_target(), G_Dt);
        }

        // update altitude target and call position controller
        pos_control.set_alt_target_from_climb_rate(target_climb_rate, G_Dt);
        pos_control.update_z_controller();
    }
}

// cruise_init_course_target - initialises course target, not the yaw but the velocity copter's direction.
static void cruise_init_course_target()
{
    // set current course direction with simple_cos_yaw and simple_sin_yaw to current ahrs.cos_yaw(), ahrs.sin_yaw()
    init_simple_bearing();  
    
    // set the current angle course direction
    simple_yaw_angle = wrap_360_cd(ahrs.yaw_sensor);
    // simple_yaw_angle = wrap_360_cd(ahrs.yaw_sensor+18000);
    // simple_yaw_angle = constrain_float(atan2f(-simple_sin_yaw,simple_cos_yaw), -3.15f, 3.15f);
    
    // get course_pot_trim value as the pot position reference (move pot right = move copter course right, and pot left = course left)
    course_pot_trim = g.rc_6.control_in;    
    
    //init desired_vel to 0
    
}


        