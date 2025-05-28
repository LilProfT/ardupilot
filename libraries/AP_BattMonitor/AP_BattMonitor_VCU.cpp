/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "AP_BattMonitor_config.h"

#if AP_BATTERY_VCU_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <AC_VCU/AC_Vcu.h>
#include "AP_BattMonitor_VCU.h"

// update state
void AP_BattMonitor_VCU::read()
{
    AC_Vcu *vcu = AP::vcumonitor();
    if (vcu == nullptr) {
        return;
    }

    if (!vcu->is_themro_healthy()) {
        _state.healthy = false;
        return;
    }
    _state.healthy = true;

    float data[AP_BATT_MONITOR_CELLS_MAX];
    
    float charge_state;
    float current_amps;
    float temp_C; 
    uint8_t pct_remaining; 
    uint32_t error;
    if (vcu->get_batt_info(charge_state, current_amps, temp_C, pct_remaining,error)) {
        have_info = true;
        _state.current_amps = current_amps;
        _state.state_of_health_pct = charge_state;
        _state.temperature = temp_C;
        vcu->get_thermo_array_data(data);
        error_mask = error;
        //Put to battery state backend
        for (int i = 0; i < AP_BATT_MONITOR_CELLS_MAX; i ++) {
            _state.cell_voltages.cells[i] = data[i]/10.0f;
        }
    }
}

// capacity_remaining_pct - returns true if the battery % is available and writes to the percentage argument
bool AP_BattMonitor_VCU::capacity_remaining_pct(uint8_t &percentage) const
{
    if (have_info) {
        percentage = remaining_pct;
    }
    return have_info;
}
uint32_t AP_BattMonitor_VCU::get_mavlink_fault_bitmask() const
{
    return error_mask;
}
#endif // AP_BATTERY_VCU_ENABLED
