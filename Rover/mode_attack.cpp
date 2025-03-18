#include "Rover.h"

const AP_Param::GroupInfo ModeAttack::var_info[] = {
    // @Param: _DIST
    // @DisplayName: Turn distance
    // @Description: Vehicle will turn at specific distance
    // @Units: m
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_TURN_DIST", 1, ModeAttack, side_dist, 20),

    // @Param: _SPEED
    // @DisplayName: Cruise Speed
    // @Description: Vehicle will move at this speed in turn
    // @Units: m/s
    // @Range: 0 10
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("_TURN_SPEED", 2, ModeAttack, dspeed, 15),

    // @Param: _TIMEMS
    // @DisplayName: Mode timeout in ms
    // @Description: 
    // @Values: ms
    // @User: Standard
    AP_GROUPINFO("_TURN_TIME", 3, ModeAttack, turn_timems, 5000),
    
    // @Param: _TURN_ANG
    // @DisplayName: Turn Angle according to heading
    // @Description: Turn Angle
    // @Values: 0 - Steering Angle max
    // @User: Standard
    AP_GROUPINFO("_TURN_ANG", 4, ModeAttack, side_angle, 30.0f),

    // @Param: _USE_MIXCH
    // @DisplayName: Using mode subfunction in 1 rc channel
    // @Description: 
    // @Values: True false
    // @User: Standard
    AP_GROUPINFO("_USE_MIXCH", 5, ModeAttack, use_mix_channel, true),

    // @Param: _TIMEMS
    // @DisplayName: Mode timeout in ms
    // @Description: 
    // @Values: ms
    // @User: Standard
    AP_GROUPINFO("_FWD_TIME", 6, ModeAttack, forward_timems, 10000),

    AP_GROUPEND
};

ModeAttack::ModeAttack() : Mode () {
    AP_Param::setup_object_defaults(this, var_info);
}

bool ModeAttack::_enter() 
{
    //Initialize waypoint controller
    g2.wp_nav.init();
    send_notification = true;
    stage = AttackStage::MANUAL;
    ch_pos = 0;
    _sub_func = ModeFuncOpt::MANUAL_REGAINED;
    //Get rc channel assigned to this mode function
    if (use_mix_channel) {
        mode_func_channel = rc().find_channel_for_option(RC_Channel::AUX_FUNC::ATCK_FUNC_CTRL);
        if (mode_func_channel == nullptr) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Set RC function to %d",(uint16_t) RC_Channel::AUX_FUNC::ATCK_FUNC_CTRL);
        }
        reset_subfunc_switch();
    }
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Mode Attack Initialised");
    return true;
}

void ModeAttack::update()
{
    if (use_mix_channel && read_rc_input(ch_pos)) {
        set_submode_function(ch_pos, _sub_func);
    }

    switch (stage)
    {
    case AttackStage::AUTO:
        auto_control();
        break;
    
    case AttackStage::MANUAL:
        manual_control();
        break;
    default:
        break;
    }
}

void ModeAttack::auto_control()
{
    switch (_sub_func)
    {
        case ModeFuncOpt::MANUAL_REGAINED:
            return_to_manual_control();
            break;
        case ModeFuncOpt::DODGING_LEFT:
        case ModeFuncOpt::DODGING_RIGHT:
            do_sidestep_movement();
            break;
        case ModeFuncOpt::FORWARD:
            do_forward_movement(); 
            break;
        default:
            return_to_manual_control();
            break;
    }
}

void ModeAttack::manual_control()
{
    // get speed forward
    float speed, desired_steering;
    if (!attitude_control.get_forward_speed(speed)) {
        float desired_throttle;
        // convert pilot stick input into desired steering and throttle
        get_pilot_desired_steering_and_throttle(desired_steering, desired_throttle);

        // if vehicle is balance bot, calculate actual throttle required for balancing
        if (rover.is_balancebot()) {    
            rover.balancebot_pitch_control(desired_throttle);
        }

        // no valid speed, just use the provided throttle
        g2.motors.set_throttle(desired_throttle);
    } else {
        float desired_speed;
        // convert pilot stick input into desired steering and speed
        get_pilot_desired_steering_and_speed(desired_steering, desired_speed);
        calc_throttle(desired_speed, true);
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

        // run steering turn rate controller and throttle controller
        steering_out = attitude_control.get_steering_out_rate(target_turn_rate,
                                                              g2.motors.limit.steer_left,
                                                              g2.motors.limit.steer_right,
                                                              rover.G_Dt);
    }

    set_steering(steering_out * 4500.0f);
}

void ModeAttack::do_forward_movement() 
{
    if (have_attitude_target) {
        // run steering and throttle controllers
        calc_steering_to_heading(_desired_yaw_cd);
        calc_throttle(_desired_speed, true);
        _distance_to_origin = rover.current_loc.get_distance(_origin_pos);
        if (_distance_to_origin >= 100.0f) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Reaching target");
                have_attitude_target = false;
        }
    } else {
        return_to_manual_control();
    }

    // stop vehicle if target not updated within 10 seconds
    if (have_attitude_target && (AP_HAL::millis() - _des_att_time_ms) > (uint32_t) forward_timems) {
        gcs().send_text(MAV_SEVERITY_WARNING, "timeout mode %f secs", (uint32_t) forward_timems / 1000.0f);
        have_attitude_target = false;
        return_to_manual_control();
    }
}

void ModeAttack::do_sidestep_movement() 
{
    if (use_posctrl) { //Run navigation controller
        if (!g2.wp_nav.reached_destination()) {
            // update navigation controller
            navigate_to_waypoint();
        if (AP_HAL::millis() - _wp_nav_time_ms > 500) {
            _wp_nav_time_ms = AP_HAL::millis();
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "distance: %f", rover.current_loc.get_distance(g2.wp_nav.get_destination()));
        }
        } else {
            // send notification
            if (send_notification) {
                send_notification = false;
                gcs().send_text(MAV_SEVERITY_INFO, "Reached destination");
            }
            // update distance to destination
            _distance_to_destination = rover.current_loc.get_distance(g2.wp_nav.get_destination());
            stage = AttackStage::MANUAL;
        }
    } else { //Run throttle and steering controller
        if (have_attitude_target) {
            // run steering and throttle controllers
            calc_steering_to_heading(_desired_yaw_cd);
            calc_throttle(dspeed, true);
            
            _distance_to_origin = rover.current_loc.get_distance(_origin_pos);

            // float desired_distance = side_dist / sinf(side_angle);
            if (_distance_to_origin >= side_dist) {
                // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Return heading to %f", _origin_yaw);
                //Return back to original heading without changing speed
                set_desired_heading_and_speed(wrap_180_cd(_origin_yaw),dspeed);
                if(abs(ahrs.yaw_sensor - _origin_yaw) <= 50) {
                    have_attitude_target = false;
                }
            }
        } else {
            return_to_manual_control();
        }

        // stop vehicle if target not updated within 5 seconds
        if (have_attitude_target && (AP_HAL::millis() - _des_att_time_ms) > (uint32_t) turn_timems) {
            gcs().send_text(MAV_SEVERITY_WARNING, "timeout mode %f secs", (uint32_t) turn_timems/1000.0f);
            have_attitude_target = false;
            return_to_manual_control();
        }
    }
}

bool ModeAttack::read_rc_input(uint8_t &pos) 
{
    //Sanity check
    if (mode_func_channel == nullptr) {
        return false;
    }
    
    const uint16_t rc_pwm = mode_func_channel->get_radio_in();
    if (rc_pwm <= RC_MIN_LIMIT_PWM || rc_pwm >= RC_MAX_LIMIT_PWM) {
        // This is an error condition
        return false;  
    }
    if (rc_pwm < 1231) {            //800 < PWM < 1231
        pos = 0;
    } else if (rc_pwm < 1491) {     //1231 < PWM < 1491
        pos = 1;
    } else if (rc_pwm < 1750) {     //1491 < PWM < 1750
        pos = 2;
    }
    else {                          //1750 < PWM < 2000
        pos = 3;
    }
    
    //Check the rc signal debouncing
    if (!debounce_check(pos)) {
        return false;
    }

    return true;
}

bool ModeAttack::debounce_check(int8_t pos) {
    if (subfunc_state.current_pos == pos) {
        //reset debouncing
        subfunc_state.debounce_pos = pos;
    } else {
        // switch change detected
        const uint32_t timems = AP_HAL::millis();

        // position not established yet
        if (subfunc_state.debounce_pos != pos) {
            subfunc_state.debounce_pos = pos;
            subfunc_state.last_edge_time_ms = timems;
        } else if (timems - subfunc_state.last_edge_time_ms >= 200) { //debouncing timeout 0.2s
            // position estabilished; debounce completed
            subfunc_state.current_pos = pos;
            return true;
        }
    }
    return false;
}

//reset switch state can be called at initialise
void ModeAttack::reset_subfunc_switch() 
{
    subfunc_state.debounce_pos = -1;
    subfunc_state.current_pos = -1;
    read_rc_input(ch_pos);
}

void ModeAttack::set_submode_function(uint8_t pos, ModeFuncOpt &func) 
{
    switch (pos)
    {
    case 0:
        func = ModeFuncOpt::MANUAL_REGAINED;
        return_to_manual_control();
        break;
    case 1:
        func = ModeFuncOpt::DODGING_LEFT;
        move_to_side(Direction::LEFT);
        break;
    case 2:
        func = ModeFuncOpt::DODGING_RIGHT;
        move_to_side(Direction::RIGHT);
        break;
    case 3:
        func = ModeFuncOpt::FORWARD;
        move_forward();
        break;
    default:
        func = ModeFuncOpt::MANUAL_REGAINED;
        break;
    }
}

//Calculate next destination base on side distance and angle
bool ModeAttack::calc_next_dest(Direction dir, Location &next_dest) 
{
    Location curr_pos;
    //Get current location
    const bool is_posok = rover.ahrs.get_location(curr_pos); 
    //if no position estimate return false
    if (!is_posok) {
        return false;
    }

    if (is_zero(side_dist)) {
        return false;
    }
    int8_t dir_sign = (dir == Direction::LEFT) ? -1 : 1;
    float desired_distance = side_dist/cosf(side_angle);
    float desired_angle = dir_sign * (side_angle) + (ahrs.yaw_sensor);
    next_dest.offset_bearing(desired_angle * 0.01f, desired_distance);
    return true;
}

bool ModeAttack::move_to_side(Direction dir) 
{
    if (!use_mix_channel) {
        _sub_func = (dir == Direction::LEFT) ? ModeFuncOpt::DODGING_LEFT : ModeFuncOpt::DODGING_RIGHT;
    }
    if (use_posctrl) {  //Calculate next location
        Location next_dest = rover.current_loc;
        float desired_speed = is_positive(dspeed) ? dspeed : rover.g.speed_cruise;
        set_desired_direction(dir);
        bool ret;
        if (calc_next_dest(_dir, next_dest)) {
            ret = g2.wp_nav.set_desired_location(next_dest);
            ret = g2.wp_nav.set_speed_max(desired_speed);
            GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "New location set");
            stage = AttackStage::AUTO;
        } else {
            //return to manual
            stage = AttackStage::MANUAL;
            return false;
        }
        send_notification = true;
        return ret;
    } else { 
        //Calculate desired throttle and steering
        float desired_speed = is_positive(dspeed) ? dspeed : g.speed_cruise * 1.5f;
        int8_t dir_sign = (dir == Direction::LEFT) ? -1 : 1;
        int8_t offs_angle_cd = dir_sign * 200;
        set_desired_heading_delta_and_speed(dir_sign * side_angle * 100.0f + offs_angle_cd, desired_speed);
        //Stored original position and yaw
        _origin_pos = rover.current_loc;
        _origin_yaw = ahrs.yaw_sensor;
        
        return true;
    }
    
}

void ModeAttack::move_forward() 
{
    if (!use_mix_channel) {
        _sub_func = ModeFuncOpt::FORWARD;
    }

    float desired_speed = g2.speed_max;
    //Check if valid of speed max value
    if (desired_speed <= 0 || desired_speed < g.speed_cruise) {
        desired_speed = calc_speed_max(g.speed_cruise, g.throttle_cruise);
    }
    float yaw_delta_cd = 0;
    //Locked yaw heading and push speed to max in this mode
    set_desired_heading_delta_and_speed(yaw_delta_cd, desired_speed);
    _origin_pos = rover.current_loc;
}

void ModeAttack::set_desired_heading_delta_and_speed(float yaw_delta_cd, float target_speed)
{
    if(stage != AttackStage::AUTO) {
        stage = AttackStage::AUTO;
        _desired_yaw_cd = wrap_180_cd(ahrs.yaw_sensor + yaw_delta_cd);
    }
    set_desired_heading_and_speed(_desired_yaw_cd,target_speed);
}

void ModeAttack::set_desired_heading_and_speed(float yaw_angle_cd, float target_speed)
{
    _des_att_time_ms = AP_HAL::millis();
    have_attitude_target = true;
    //record targets
    _desired_yaw_cd = yaw_angle_cd;
    _desired_speed = target_speed;
}

//Set heading back before doing a turn
void ModeAttack::return_to_manual_control() 
{
    if (stage == AttackStage::AUTO) {
        stage = AttackStage::MANUAL;
        _sub_func = ModeFuncOpt::MANUAL_REGAINED;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Return to manual control");
    }
}

void ModeAttack::set_desired_direction(Direction dir) 
{
    _dir = dir;
}