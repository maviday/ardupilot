#include "AP_UserCustom.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_UserCustom::var_info[] = {
        // @Param: ENABLE
        // @DisplayName: UserCustom Enable/Disable
        // @Description: UserCustom enable/disable
        // @User: Standard
        // @Values: 0:Disabled, 1:Enabled
        AP_GROUPINFO_FLAGS("ENABLE", 0, AP_UserCustom, _enabled, 0, AP_PARAM_FLAG_ENABLE),

        // @Param: TEST_INT1
        // @DisplayName: User Custom Integer 1
        // @Description: User Custom Integer 1
        // @User: Standard
        AP_GROUPINFO("TEST_INT1", 1, AP_UserCustom, test_int1, 0),

        // @Param: TEST_INT2
        // @DisplayName: User Custom Integer 2
        // @Description: User Custom Integer 3
        // @User: Standard
        AP_GROUPINFO("TEST_INT2", 2, AP_UserCustom, test_int2, 0),

        // @Param: TEST_INT3
        // @DisplayName: User Custom Integer 4
        // @Description: User Custom Integer 4
        // @User: Standard
        AP_GROUPINFO("TEST_INT3", 3, AP_UserCustom, test_int3, 0),

        // @Param: TEST_FLT1
        // @DisplayName: User Custom Float 1
        // @Description: User Custom Float 1
        // @User: Standard
        AP_GROUPINFO("TEST_FLT1", 4, AP_UserCustom, test_float1, 0),

        // @Param: TEST_FLT2
        // @DisplayName: User Custom Float 2
        // @Description: User Custom Float 2
        // @User: Standard
        AP_GROUPINFO("TEST_FLT2", 5, AP_UserCustom, test_float2, 0),

        // @Param: TEST_FLT3
        // @DisplayName: User Custom Float 3
        // @Description: User Custom Float 3
        // @User: Standard
        AP_GROUPINFO("TEST_FLT3", 6, AP_UserCustom, test_float3, 0),

    AP_GROUPEND
};

void AP_UserCustom::init()
{
    // return immediately if not enabled
    if (!enabled()) {
        return;
    }

    // TODO: add custom init work

    is_initialized = true;
}

// periodic callback that runs at 50Hz
void AP_UserCustom::update()
{
    // return immediately if not enabled
    if (!enabled()) {
        return;
    }
    if (!is_initialized) {
        init();
        return;
    }
    const uint32_t now_ms = AP_HAL::millis();

    if (now_ms - last_update_timestamp_ms > 1000) {
        last_update_timestamp_ms = now_ms;
        // period work that happens every 1000 ms (1Hz)

        // TODO: add custom periodic slow work
    }

    // TODO: add custom periodic fast work
}


// return true if checks are OK and we allow arming.
// return false if checks fail and we want to block arming. If "report" is true we can optionally send a message to inform the user and/or external system(s) of the reason
bool AP_UserCustom::arming_check(bool report)
{
    if (!enabled()) {
        return true;
    }

    bool checks_passed = false;

    // TODO: add custom arming check and set checks_passed to false if the custom checks don't pass
    checks_passed = true;

    if (report && !checks_passed) {
        // TODO: add custom arming check failure message
        const char* reason = "Error";
        int32_t error_number = 42;
        gcs().send_text(MAV_SEVERITY_INFO, "UserCustom: %s %d", reason, error_number);
    }
    return checks_passed;
}

// handle inbound MAVLink messages
bool AP_UserCustom::handle_user_message(const mavlink_command_long_t &packet)
{
    if (!enabled()) {
        return false;
    }

    switch (packet.command) {
    case MAV_CMD_WAYPOINT_USER_1:
    case MAV_CMD_WAYPOINT_USER_2:
    case MAV_CMD_WAYPOINT_USER_3:
    case MAV_CMD_WAYPOINT_USER_4:
    case MAV_CMD_WAYPOINT_USER_5:
    case MAV_CMD_SPATIAL_USER_1:
    case MAV_CMD_SPATIAL_USER_2:
    case MAV_CMD_SPATIAL_USER_3:
    case MAV_CMD_SPATIAL_USER_4:
    case MAV_CMD_SPATIAL_USER_5:
    case MAV_CMD_USER_1:
    case MAV_CMD_USER_2:
    case MAV_CMD_USER_3:
    case MAV_CMD_USER_4:
    case MAV_CMD_USER_5:
        // TODO: add custom MAVLink handling work
        return true;

    default:
        return false;
    }
}

// singleton instance
AP_UserCustom *AP_UserCustom::_singleton;

namespace AP {

AP_UserCustom *usercustom()
{
    return AP_UserCustom::get_singleton();
}

};

