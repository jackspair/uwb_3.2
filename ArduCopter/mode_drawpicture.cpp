#include "Copter.h"

// init - initialise guided controller
bool ModeDrawPicture::init(bool ignore_checks)
{
    if(ignore_checks){
        auto_yaw.set_mode_to_default(false);

        path_num=0;
        generate_path();

        wp_control_start();
        return true;
    }else{
        return false;
    }
}

void ModeDrawPicture::generate_path()
{
    float radius_cm=1000.0;
    wp_nav->get_wp_stopping_point(path[0]);
    path[1]=path[0]+Vector3f(1.0f,0,0)*radius_cm;
    path[2]=path[0]+Vector3f(1.0f,1.0f,0)*radius_cm;
    path[3]=path[0]+Vector3f(0,1.0f,0)*radius_cm;
    path[4]=path[0];
}

// run - runs the guided controller
// should be called at 100hz or more
void ModeDrawPicture::run()
{
    // run pause control if the vehicle is paused
    if(path_num<6){
        if(wp_nav->reached_wp_destination()){
            path_num++;
            wp_nav->set_wp_destination(path[path_num],false);
        }
    }
    
    wp_control_run();
 }


// initialise guided mode's waypoint navigation controller
void ModeDrawPicture::wp_control_start()
{
    // initialise waypoint and spline controller
    wp_nav->wp_and_spline_init();

    // initialise wpnav to stopping point
    wp_nav->set_wp_destination(path[0],false);

    // initialise yaw
    auto_yaw.set_mode_to_default(false);
}

// run guided mode's waypoint navigation controller
void ModeDrawPicture::wp_control_run()
{
    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!copter.failsafe.radio && use_pilot_yaw()) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());
        if (!is_zero(target_yaw_rate)) {
            auto_yaw.set_mode(AUTO_YAW_HOLD);
        }
    }

    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        // do not spool down tradheli when on the ground with motor interlock enabled
        make_safe_ground_handling(copter.is_tradheli() && motors->get_interlock());
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // run waypoint controller
    copter.failsafe_terrain_set_status(wp_nav->update_wpnav());

    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control->update_z_controller();

    // call attitude controller
    if (auto_yaw.mode() == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control->input_thrust_vector_rate_heading(wp_nav->get_thrust_vector(), target_yaw_rate);
    } else if (auto_yaw.mode() == AUTO_YAW_RATE) {
        // roll & pitch from waypoint controller, yaw rate from mavlink command or mission item
        attitude_control->input_thrust_vector_rate_heading(wp_nav->get_thrust_vector(), auto_yaw.rate_cds());
    } else {
        // roll, pitch from waypoint controller, yaw heading from GCS or auto_heading()
        attitude_control->input_thrust_vector_heading(wp_nav->get_thrust_vector(), auto_yaw.yaw());
    }
}

