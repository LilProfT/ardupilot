#pragma once

#include "AP_BattMonitor_Backend.h"

#if AP_BATTERY_VCU_ENABLED

class AP_BattMonitor_VCU : public AP_BattMonitor_Backend
{
public:

    // Inherit constructor
    using AP_BattMonitor_Backend::AP_BattMonitor_Backend;

    // update state
    void read(void) override;

    /// returns true if battery monitor instance provides current info
    bool has_current() const override { return have_info; };

    // returns true if battery monitor provides temperature
    bool has_temperature() const override { return have_info; };

    // capacity_remaining_pct - returns true if the battery % is available and writes to the percentage argument
    bool capacity_remaining_pct(uint8_t &percentage) const override WARN_IF_UNUSED;

    uint32_t get_mavlink_fault_bitmask() const override;


private:
    bool have_info;             // true if vcu has provided battery info at least once
    bool have_capacity;         // true once vcu has provided battery capacity
    uint8_t remaining_pct;      // battery remaining capacity as a percentage
    uint32_t error_mask;

};

#endif // AP_BATTERY_VCU_ENABLED
