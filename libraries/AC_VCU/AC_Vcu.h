#pragma once

#include "AC_Vcu_config.h"


#include <AP_ESC_Telem/AP_ESC_Telem_Backend.h>
#include <AP_Param/AP_Param.h>
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#if HAL_VCU_ENABLED
class AC_Vcu {
public:
    AC_Vcu();

    CLASS_NO_COPY(AC_Vcu);

    static AC_Vcu* get_singleton();

    // initialise driver
    void init();

    void update_log();

    static const struct AP_Param::GroupInfo var_info[];
    
    void handle_vcu_message(const mavlink_message_t &msg);
    void send_mavlink_status(mavlink_channel_t chan);
    bool is_healthy(void) const;

    struct VcuMonitorState {
        uint8_t instance;
        float thermo_data[AP_BATT_MONITOR_CELLS_MAX];  //temp * 10
        uint32_t thermo_error_mask;
        float steering_angle;
        float throttle_pct;
        float contactor_state;
        int8_t fuel_level_pct;
        uint32_t last_updated_ms;
    };

    void get_thermo_array_data(float *temp);
    bool get_batt_info(float &charge_state, float &current_amps, float &temp_C, uint8_t &pct_remaining, uint32_t &error_mask) const;

private:;                             
    static AC_Vcu *_singleton;
    bool _initialised;

    void log_status(void);

    
    // parameters
    AP_Int8         _enabled;                       ///< top level enable/disable control
    AP_Float        _temp_threshold_cwi;            ///<Threshold value for coolant water inlet channel
    AP_Float        _temp_threshold_cwo;            ///<Threshold value for coolant water outlet channel
    AP_Float        _temp_threshold_ec;             ///< Threshold value for engine cabin channel
    AP_Float        _temp_threshold_fc;             ///< Threshold value for fuel cabin channel

    AP_Float        _temp_threshold_ex;             ///< Threshold value for EXHAUST channel
    uint8_t _sysid;                 // sysid of VCU
    uint8_t _compid;                // component id of VCU
    mavlink_channel_t _chan = MAVLINK_COMM_0;        // mavlink channel used to communicate with VCU

    struct VcuMonitorState vcu_state;
};

namespace AP {
    AC_Vcu *vcumonitor();
};
#endif // HAL_VCU_ENABLED

