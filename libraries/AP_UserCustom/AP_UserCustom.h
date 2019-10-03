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

#pragma once

#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS.h>

class AP_UserCustom {
public:
    AP_UserCustom() {
        if (_singleton) {
    #if CONFIG_HAL_BOARD == HAL_BOARD_SITL
            AP_HAL::panic("Too many AP_UserCustom");
    #endif
            return;
        }
        _singleton = this;

        AP_Param::setup_object_defaults(this, var_info);
    }

    // remove copy constructor for any class that has a singleton
    AP_UserCustom(const AP_UserCustom &other) = delete;
    AP_UserCustom &operator=(const AP_UserCustom&) = delete;

    static const struct AP_Param::GroupInfo        var_info[];

    static AP_UserCustom *get_singleton() { return _singleton; }

    // initialise this module
    void init();

    // update - should be called at at least 10hz but usually 50Hz
    void update();

    // indicate whether this module is enabled or not
    bool enabled() const { return _enabled; }

    bool arming_check(bool report);

    bool handle_user_message(const mavlink_command_long_t &packet);

    // public parameters
    AP_Int32    test_int1;
    AP_Int32    test_int2;
    AP_Int32    test_int3;
    AP_Float    test_float1;
    AP_Float    test_float2;
    AP_Float    test_float3;

private:
    static AP_UserCustom *_singleton;
    bool is_initialized;

    uint32_t last_update_timestamp_ms;

    // private parameters
    AP_Int8     _enabled;               //  module enable/disable

};

namespace AP {
    AP_UserCustom *usercustom();
};
