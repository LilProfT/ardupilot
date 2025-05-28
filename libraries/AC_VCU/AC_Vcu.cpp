
#include "AC_Vcu.h"

#if HAL_VCU_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL& hal;
#define VCU_HEALTHY_LAST_RECEIVED_MS 3000
#define VCU_MAVLINK_BATT_ID 2
// parameters
const AP_Param::GroupInfo AC_Vcu::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Sprayer CAN enable/disable
    // @Description: Allows you to enable (1) or disable (0) the sprayer
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO_FLAGS("_ENABLE", 0, AC_Vcu, _enabled, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: FUEL_TANK
    // @DisplayName: Fuel tank
    // @Description: Fuel tank vaule in liters
    // @Units: Liters
    // @Range:
    // @User: Standard
    AP_GROUPINFO("_THRSH_CWI",   1, AC_Vcu, _temp_threshold_cwi, 120),

    // @Param: MAX_VAL
    // @DisplayName: Max adc value
    // @Description: The adc value according to maximum fuel tank
    // @Units: %
    // @Range: 0 INT16_MAX
    // @User: Standard
    AP_GROUPINFO("_THRSH_CWO",   2, AC_Vcu, _temp_threshold_cwo, 110),

    // @Param: MID1_VAL
    // @DisplayName: Middle adc value
    // @Description: The adc value according to the any appropriate middle volume
    // @Units: 
    // @Range: 0 INT16_MAX
    // @User: Standard
    AP_GROUPINFO("_THRSH_EC",   3, AC_Vcu, _temp_threshold_ec, 50),

    // @Param: MID_VOL
    // @DisplayName: Middle volume litter
    // @Description: The middle volume which is input from user
    // @Units: %
    // @Range: 0 100
    // @User: Standard
    AP_GROUPINFO("_THRSH_FC",   4, AC_Vcu, _temp_threshold_fc, 50),
    
    // @Param: ZERO_VAL
    // @DisplayName: Zero adc value
    // @Description: The adc value according to 0L
    // @Units: %
    // @Range: 0 INT16_MAX
    // @User: Standard
    AP_GROUPINFO("_THRSH_EX",   5, AC_Vcu, _temp_threshold_ex, 90),

    // @Param: _CAM_ZERO
    // @DisplayName: Camera zero angle offset
    // @Description: 
    // @Values: ms
    // @User: Standard
    AP_GROUPINFO("_CAM_ZERO", 6, AC_Vcu, _offset_zero, 0),

    // @Param: _CAM_ZERO
    // @DisplayName: Camera zero angle offset
    // @Description: 
    // @Values: ms
    // @User: Standard
    AP_GROUPINFO("_INSTANCE_ID", 7, AC_Vcu, _instance, 0),

    AP_GROUPEND
};
/*
 * Get the AP_VCU singleton
 */
AC_Vcu *AC_Vcu::_singleton;
AC_Vcu *AC_Vcu::get_singleton()
{
    return _singleton;
}


AC_Vcu::AC_Vcu()
{
    AP_Param::setup_object_defaults(this, var_info);

    if (_singleton != nullptr) {
        AP_HAL::panic("AP_RPM must be singleton");
    }
    _singleton = this;
}

// initialise driver
void AC_Vcu::init()
{
    // only init once
    // Note: a race condition exists here if init is called multiple times quickly before thread_main has a chance to set _initialise
    if (!_initialised) {
        return;
    }

    // // create background thread to process serial input and output
    // if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AC_Vcu::thread_loop, void), "vcu", 2048, AP_HAL::Scheduler::PRIORITY_UART, 1)) {
    //     return;
    // }
}
// Requests backend to update the frontend. Should be called at 10Hz.
void AC_Vcu::update_log()
{
    // exit immediately if we are disabled or shouldn't be running
    if (!_enabled) {
        return;
    }
    log_status();
    // GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "%f", vcu_state.thermo_data[0]);
}

void AC_Vcu::handle_vcu_message(const mavlink_message_t &msg)
{
    //Decode message from vcu
    mavlink_battery_status_t data;
    mavlink_msg_battery_status_decode(&msg,&data);
    if(data.id == VCU_MAVLINK_BATT_ID) {
        //Put to vcu state
        vcu_state.instance = data.id;
        vcu_state.fuel_level_pct = data.battery_remaining;
        vcu_state.thermo_error_mask = data.fault_bitmask;
        vcu_state.steering_angle = data.current_battery;
        vcu_state.throttle_pct = data.temperature;
        for (long unsigned i = 0; i < ARRAY_SIZE(data.voltages); i++) {
            vcu_state.thermo_data[i] = (float) data.voltages[i] * 0.1f;

        }
        vcu_state.contactor_state = data.charge_state;
        vcu_state.last_thermo_update_ms = AP_HAL::millis();
    }
}

void AC_Vcu::handle_custom_gimbal_message(const mavlink_message_t &msg)
{
    //Decode message from vcu
    mavlink_gimbal_device_attitude_status_t packet;
    
    mavlink_msg_gimbal_device_attitude_status_decode(&msg, &packet);
    if(packet.target_component == MAV_COMP_ID_PERIPHERAL) {
        //Only receive message from ID: 158
    }
    vcu_state.raw_angle = wrap_360(packet.angular_velocity_x);
    vcu_state.zoom_pos = packet.angular_velocity_z;
    //get relative pan angle of camera
    vcu_state.pan_angle = wrap_360(packet.angular_velocity_x) - wrap_360(_offset_zero);
    vcu_state.tilt_angle = packet.angular_velocity_y;
    vcu_state.last_cam_update_ms = AP_HAL::millis();
}

void AC_Vcu::get_thermo_array_data(float *temp)
{
    memcpy(temp,vcu_state.thermo_data,sizeof(vcu_state.thermo_data));
}

bool AC_Vcu::is_themro_healthy(void) const
{
    return ((AP_HAL::millis() - vcu_state.last_thermo_update_ms) < VCU_HEALTHY_LAST_RECEIVED_MS);
}

bool AC_Vcu::is_camera_healthy(void) const
{
    return ((AP_HAL::millis() - vcu_state.last_cam_update_ms) < VCU_HEALTHY_LAST_RECEIVED_MS);
}

// get latest battery status info.  returns true on success and populates arguments
bool AC_Vcu::get_batt_info(float &charge_state, float &current_amps, float &temp_C, uint8_t &pct_remaining, uint32_t &error_mask) const
{

    // use battery info from display_system_state if available (tiller connection)
    if (is_themro_healthy()) {
        charge_state = vcu_state.contactor_state;
        current_amps = vcu_state.steering_angle;
        temp_C = vcu_state.throttle_pct;
        pct_remaining = vcu_state.fuel_level_pct;
        error_mask = vcu_state.thermo_error_mask;
        return true;
    }

    return false;
}

void AC_Vcu::send_mavlink_vcu_status(mavlink_channel_t chan)
{
    if(!_enabled) {
        return;
    }

    if (!is_themro_healthy()) {
        return;
    }

    uint16_t cells[AP_BATT_MONITOR_CELLS_MAX];
    for (int i = 0; i <AP_BATT_MONITOR_CELLS_MAX; i++) {
        cells[i] = vcu_state.thermo_data[i] * 10;
    }
    mavlink_msg_battery_status_send(
        chan,
        vcu_state.instance,
        MAV_BATTERY_FUNCTION_UNKNOWN,
        MAV_BATTERY_TYPE_UNKNOWN,
        vcu_state.throttle_pct,
        cells,
        vcu_state.steering_angle,
        0,
        0,
        constrain_int16(vcu_state.fuel_level_pct, -1, 100),
        0,
        vcu_state.contactor_state,
        0,
        MAV_BATTERY_MODE_UNKNOWN,
        vcu_state.thermo_error_mask);
}

void AC_Vcu::send_mavlink_camera_status(mavlink_channel_t chan)
{
    if(!_enabled) {
        return;
    }

    if (!is_camera_healthy()) {
        return;
    }

    AP_AHRS &ahrs = AP::ahrs();
    
    //Camera yaw angle in NED
    float pan_angle = wrap_360((vcu_state.pan_angle * 100 + ahrs.yaw_sensor)/100.0f);
    uint16_t flags = GIMBAL_DEVICE_FLAGS_ROLL_LOCK | GIMBAL_DEVICE_FLAGS_RETRACT;
    Quaternion quatt;
    quatt.from_euler(0,vcu_state.tilt_angle,pan_angle);
    const float quat_array[4] = {quatt.q1, quatt.q2, quatt.q3, quatt.q4};
    mavlink_msg_gimbal_device_attitude_status_send(chan,
                                                    0,   // target system
                                                    0,   // target component
                                                    AP_HAL::millis(),    // autopilot system time
                                                    flags,
                                                    quat_array,    // not used
                                                    pan_angle,    // roll axis angular velocity (NaN for unknown)
                                                    vcu_state.tilt_angle,    // pitch axis angular velocity (NaN for unknown)
                                                    vcu_state.zoom_pos,    // yaw angle in NED (NaN for unknown)
                                                    0,                                           // failure flags (not supported)
                                                    vcu_state.raw_angle,    // delta_yaw (NaN for unknonw)
                                                    std::numeric_limits<double>::quiet_NaN(),    // delta_yaw_velocity (NaN for unknonw)
                                                    _instance + 1);  // gimbal_device_id);
}


#if HAL_LOGGING_ENABLED
void AC_Vcu::log_status(void)
{
// @LoggerMessage: VCU
// @Description: Electronic Fuel Injection system data
// @Field: TimeUS: Time since system startup
// @Field: ICW: Reported inlet coolant water temperature channel
// @Field: OCW: Reported outlet coolant water temperature channel
// @Field: ECa: Reported engine cabin temperature channel
// @Field: FCa: Reported fuel cabin temperature channel
// @Field: ExP: Reported exhaust pipe temperature channel
// @Field: STR: Steering angle
// @Field: RemPct: Fuel level percentage
// @Field: LckSt: Contactor lock status
// @Field: ErrV: Error bitmask value

    AP::logger().WriteStreaming("VCU",
                       "TimeUS,ICW,OCW,ECa,FCa,ExP,STR,RemPct,LckSt,ErrV",
                       "s---------",
                       "F---------",
                       "Qfffffffff",
                       AP_HAL::micros64(),
                       float(vcu_state.thermo_data[0]),
                       float(vcu_state.thermo_data[1]),
                       float(vcu_state.thermo_data[2]),
                       float(vcu_state.thermo_data[3]),
                       float(vcu_state.thermo_data[4]),
                       float(vcu_state.steering_angle),
                       float(vcu_state.fuel_level_pct),
                       float(vcu_state.contactor_state),
                       float(vcu_state.thermo_error_mask));

}
#endif // LOGGING_ENABLED

namespace AP {

    AC_Vcu *vcumonitor()
    {
        return AC_Vcu::get_singleton();
    }
    
};

#endif
