#include "Rover.h"

bool ModeStalking::_enter() {

    // //initialise submode to stop or loiter
    // if (rover.is_boat()) {
        
    //     if(!start_loiter()) {
    //         start_stop();
    //     }

    // }
    // else {
    //     start_stop();
    // }

    // //Initialise waypoint navigation library
    // g2.wp_nav.init();
    // send_notification = true;
    
    return true;
}

void ModeStalking::update() {

}

// return heading (in degrees) and cross track error (in meters) for reporting to ground station (NAV_CONTROLLER_OUTPUT message)
float ModeStalking::wp_bearing() const
{
    switch (_stalking_mode) {
    case SubMode::WP:
        return g2.wp_nav.wp_bearing_cd() * 0.01f;
    case SubMode::HeadingAndSpeed:
    case SubMode::TurnRateAndSpeed:
        return 0.0f;
    case SubMode::Loiter:
        return rover.mode_loiter.wp_bearing();
    case SubMode::SteeringAndThrottle:
    case SubMode::Stop:
        return 0.0f;
    }

    // we should never reach here but just in case, return 0
    return 0.0f;
}

float ModeStalking::nav_bearing() const
{
    switch (_stalking_mode) {
    case SubMode::WP:
        return g2.wp_nav.nav_bearing_cd() * 0.01f;
    case SubMode::HeadingAndSpeed:
    case SubMode::TurnRateAndSpeed:
        return 0.0f;
    case SubMode::Loiter:
        return rover.mode_loiter.nav_bearing();
    case SubMode::SteeringAndThrottle:
    case SubMode::Stop:
        return 0.0f;
    }

    // we should never reach here but just in case, return 0
    return 0.0f;
}

float ModeStalking::crosstrack_error() const
{
    switch (_stalking_mode) {
    case SubMode::WP:
        return g2.wp_nav.crosstrack_error();
    case SubMode::HeadingAndSpeed:
    case SubMode::TurnRateAndSpeed:
        return 0.0f;
    case SubMode::Loiter:
        return rover.mode_loiter.crosstrack_error();
    case SubMode::SteeringAndThrottle:
    case SubMode::Stop:
        return 0.0f;
    }

    // we should never reach here but just in case, return 0
    return 0.0f;
}

float ModeStalking::get_desired_lat_accel() const
{
    switch (_stalking_mode) {
    case SubMode::WP:
        return g2.wp_nav.get_lat_accel();
    case SubMode::HeadingAndSpeed:
    case SubMode::TurnRateAndSpeed:
        return 0.0f;
    case SubMode::Loiter:
        return rover.mode_loiter.get_desired_lat_accel();
    case SubMode::SteeringAndThrottle:
    case SubMode::Stop:
        return 0.0f;
    }

    // we should never reach here but just in case, return 0
    return 0.0f;
}

// return distance (in meters) to destination
float ModeStalking::get_distance_to_destination() const
{
    switch (_stalking_mode) {
    case SubMode::WP:
        return _distance_to_destination;
    case SubMode::HeadingAndSpeed:
    case SubMode::TurnRateAndSpeed:
        return 0.0f;
    case SubMode::Loiter:
        return rover.mode_loiter.get_distance_to_destination();
    case SubMode::SteeringAndThrottle:
    case SubMode::Stop:
        return 0.0f;
    }

    // we should never reach here but just in case, return 0
    return 0.0f;
}

// return true if vehicle has reached or even passed destination
bool ModeStalking::reached_destination() const
{
    switch (_stalking_mode) {
    case SubMode::WP:
        return g2.wp_nav.reached_destination();
    case SubMode::HeadingAndSpeed:
    case SubMode::TurnRateAndSpeed:
    case SubMode::Loiter:
    case SubMode::SteeringAndThrottle:
    case SubMode::Stop:
        return true;
    }

    // we should never reach here but just in case, return true is the safer option
    return true;
}

// set desired speed in m/s
bool ModeStalking::set_desired_speed(float speed)
{
    switch (_stalking_mode) {
    case SubMode::WP:
        return g2.wp_nav.set_speed_max(speed);
    case SubMode::HeadingAndSpeed:
    case SubMode::TurnRateAndSpeed:
        // speed is set from mavlink message
        return false;
    case SubMode::Loiter:
        return rover.mode_loiter.set_desired_speed(speed);
    case SubMode::SteeringAndThrottle:
    case SubMode::Stop:
        // no speed control
        return false;
    }
    return false;
}

// get desired location
bool ModeStalking::get_desired_location(Location& destination) const
{
    switch (_stalking_mode) {
    case SubMode::WP:
        if (g2.wp_nav.is_destination_valid()) {
            destination = g2.wp_nav.get_oa_destination();
            return true;
        }
        return false;
    case SubMode::HeadingAndSpeed:
    case SubMode::TurnRateAndSpeed:
        // not supported in these submodes
        return false;
    case SubMode::Loiter:
        // get destination from loiter
        return rover.mode_loiter.get_desired_location(destination);
    case SubMode::SteeringAndThrottle:
    case SubMode::Stop:
        // no desired location in this submode
        break;
    }

    // should never get here but just in case
    return false;
}

// set desired location
bool ModeStalking::set_desired_location(const Location &destination, Location next_destination)
{
    if (use_scurves_for_navigation()) {
        // use scurves for navigation
        if (!g2.wp_nav.set_desired_location(destination, next_destination)) {
            return false;
        }
    } else {
        // use position controller input shaping for navigation
        // this does not support object avoidance but does allow faster updates of the target
        if (!g2.wp_nav.set_desired_location_expect_fast_update(destination)) {
            return false;
        }
    }

    // handle guided specific initialisation and logging
    _stalking_mode = SubMode::WP;
    send_notification = true;
    rover.Log_Write_GuidedTarget((uint8_t)_stalking_mode, Vector3f(destination.lat, destination.lng, 0), Vector3f(g2.wp_nav.get_speed_max(), 0.0f, 0.0f));
    return true;
}

bool ModeStalking::use_scurves_for_navigation() const
{
    return ((rover.g2.guided_options.get() & uint32_t(Options::SCurvesUsedForNavigation)) != 0);
}
