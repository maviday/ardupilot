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
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL_ChibiOS/CANIface.h>
#include "AP_Periph.h"
#include "hal.h"

#ifdef HAL_PERIPH_ENABLE_RCOUT_TRANSLATOR

#define ESC_MAX_VALUE 8191.0f
extern const AP_HAL::HAL &hal;

// table of user settable CAN bus parameters
const AP_Param::GroupInfo AP_Periph_FW::RCOUTTranslator_Params::var_info[] = {

    // @Param: RCOUT_CHAN_LOW
    // @DisplayName: RCOUT first servo output channel number.
    // @Description: RCOUT first servo output channel number. Consecutive output channels are enabled from RCOUT_CHAN_LOW through RCOUT_CHAN_HIGH. Use 1 for PWM1
    // @Range: 900 2100
    AP_GROUPINFO("CHAN_LOW", 1, AP_Periph_FW::RCOUTTranslator_Params, chan_start, 0),

    // @Param: RCOUT_CHAN_HIGH
    // @DisplayName: RCOUT last servo output channel number.
    // @Description: RCOUT last servo output channel number. Consecutive output channels are enabled from RCOUT_CHAN_LOW through RCOUT_CHAN_HIGH.
    // @Range: 900 2100
    AP_GROUPINFO("CHAN_HIGH", 2, AP_Periph_FW::RCOUTTranslator_Params, chan_end, 0),

    // @Param: RCOUT_PROTOCOL
    // @DisplayName: Enable use of specific RCOUT protocol
    // @Description: Enabling this option starts selected rcout protocol that will be used
    // @Values {CAN_OUT == 1}: 0:UAVCAN,1:KDECAN
    // @Values {CAN_OUT == 0}: maps to RCOutput.h output_mode where 1 means MODE_PWM_NORMAL
    // @User: Advanced
    AP_GROUPINFO("PROTOCOL", 3, AP_Periph_FW::RCOUTTranslator_Params, protocol, AP_HAL::RCOutput::MODE_PWM_NORMAL),

#if HAL_NUM_CAN_IFACES > 1
    AP_GROUPINFO("CAN_OUT", 4, AP_Periph_FW::RCOUTTranslator_Params, can_out, 0),
#endif

    // @Param: RCOUT_FREQ
    // @RebootRequired: True
    AP_GROUPINFO("FREQ", 5, AP_Periph_FW::RCOUTTranslator_Params, frequency, 50),

    // @Param: RCOUT_ACT_TYPE
    // @DisplayName: RCOut type
    // @Description: RCOut type
    // @Values : 0:ESC,1:SRV
    AP_GROUPINFO("ACT_TYPE", 6, AP_Periph_FW::RCOUTTranslator_Params, act_type, RCOUTTranslator_Params::ActType::ESC),

    // @Param: RCOUT_PWM_MIN
    AP_GROUPINFO("PWM_MIN", 7, AP_Periph_FW::RCOUTTranslator_Params, pwm_min, 1000),
    // @Param: RCOUT_PWM_MAX
    AP_GROUPINFO("PWM_MAX", 8, AP_Periph_FW::RCOUTTranslator_Params, pwm_max, 2000),

#if HAL_NUM_CAN_IFACES > 1
    // @Param: RCOUT_KDE_ENUM
    AP_GROUPINFO("KDE_ENUM", 9, AP_Periph_FW::RCOUTTranslator_Params, kdecan_enum_mode, 0),

    // @Group: KDE_
    // @Path: ../AP_KDECAN/AP_KDECAN.cpp
    AP_SUBGROUPPTR(kdecan, "KDE_", 10, AP_Periph_FW::RCOUTTranslator_Params, AP_KDECAN),
#endif

    AP_GROUPEND
};

#if HAL_NUM_CAN_IFACES > 1
static ChibiOS::CANIface rcout_can_iface(1);
#endif

void AP_Periph_FW::init_rcout_translator()
{
    rcout.chan_start = rcout_params.chan_start;
    rcout.chan_end = rcout_params.chan_end;
    rcout.protocol = rcout_params.protocol;
    rcout.act_type = rcout_params.act_type;

    if (rcout.chan_end < rcout.chan_start || rcout.chan_start <= 0) {
        rcout.num_channels = 0;
        return;
    }
    rcout.num_channels = rcout.chan_end - rcout.chan_start + 1;

#if HAL_NUM_CAN_IFACES > 1
    if (rcout_params.can_out == 1) {
        // initialise CAN driver
        if (!rcout.can_init_done) {
            rcout.can_init_done = true;
            rcout_can_iface.init(1000000, AP_HAL::CANIface::NormalMode);
        }

        switch (rcout.protocol) {
            case RCOUTTranslator_Params::RCOUT_KDECAN: {
                if (rcout_params.kdecan != nullptr) {
                    // already initialized
                    break;
                }
                rcout_params.kdecan = new AP_KDECAN;

                if (rcout_params.kdecan == nullptr) {
                    printf("Failed to allocate KDECAN");
                    return;
                }

                AP_Param::load_object_from_eeprom((AP_KDECAN*)rcout_params.kdecan, AP_KDECAN::var_info);
                rcout_params.kdecan->add_interface(&rcout_can_iface);
                rcout_params.kdecan->init(0, false);
                break;
            }
            case RCOUTTranslator_Params::RCOUT_UAVCAN: {
                break;
            }
        }
    }
#endif
    for (uint8_t i = 0; i < 16; i++) {
        hal.rcout->disable_ch(i);
    }

    // Configure non CAN translation via hal rcoutput driver
    const AP_HAL::RCOutput::output_mode mode = (AP_HAL::RCOutput::output_mode)rcout.protocol;
    if (mode == AP_HAL::RCOutput::MODE_PWM_NONE) {
        return;
    }
    for (uint8_t i = rcout.chan_start; i <= rcout.chan_end; i++) {
        hal.rcout->enable_ch(i-1);
    }
    const uint16_t mask = (1UL << (rcout.num_channels)) - 1UL;
    hal.rcout->set_freq(mask, rcout_params.frequency);
    hal.rcout->set_output_mode(mask, mode);

    if (rcout.act_type == RCOUTTranslator_Params::ActType::ESC) {
        // only set esc scales for ESC MODE
        const uint16_t pwm_min = constrain_int16(rcout_params.pwm_min, 900, 2100);
        const uint16_t pwm_max = constrain_int16(rcout_params.pwm_max, 900, 2100);
        hal.rcout->set_esc_scaling(pwm_min, pwm_max);
    }
}

void AP_Periph_FW::translate_rcout_esc(int16_t *rc, uint8_t num_channels)
{
    if (rc == nullptr || 
        rcout.num_channels == 0 ||
        (num_channels <= rcout.chan_end) ||
        rcout.act_type != RCOUTTranslator_Params::ActType::ESC)
    {
        return;
    }
    // sanity check params
    const uint16_t pwm_min = constrain_int16(rcout_params.pwm_min, 900, 2100);
    const uint16_t pwm_max = constrain_int16(rcout_params.pwm_max, 900, 2100);

    for (uint8_t i = rcout.chan_start; i <= rcout.chan_end; i++) {
        // scale rc from "0 -> 8191" to "pwm_min -> pwm_max"
        const uint16_t output_pwm = linear_interpolate(pwm_min, pwm_max, rc[i], 0, ESC_MAX_VALUE);
        hal.rcout->write(i - rcout.chan_start - 1, output_pwm);
    }
#if HAL_NUM_CAN_IFACES > 1
    if (rcout_params.can_out == 1) {
        switch (rcout.protocol) {
            case RCOUTTranslator_Params::RCOUT_KDECAN: {
                if (rcout_params.kdecan == nullptr) {
                    return;
                }
                rcout_params.kdecan->lock_rcout();
                for (uint8_t i = rcout.chan_start; i <= rcout.chan_end; i++) {
                    const float norm_output = 2.0f*constrain_float(rc[i]/ESC_MAX_VALUE, 0.0f, 1.0f) - 1.0f;
                    rcout_params.kdecan->set_output(i - rcout.chan_start - 1, norm_output);
                }
                rcout_params.kdecan->release_rcout();
                break;
            }
            case RCOUTTranslator_Params::RCOUT_UAVCAN:
                break;
        }
    }
#endif
}

void AP_Periph_FW::translate_rcout_srv(uint8_t chan, float rc)
{
    if (rcout.num_channels == 0 ||
        chan < rcout.chan_start ||
        chan > rcout.chan_end ||
        rcout.act_type != RCOUTTranslator_Params::ActType::SRV)
    {
        return;
    }

    // sanity check params
    const uint16_t pwm_min = constrain_int16(rcout_params.pwm_min, 900, 2100);
    const uint16_t pwm_max = constrain_int16(rcout_params.pwm_max, 900, 2100);

    // scale rc from "-1 -> +1" to "_pwm_min -> _pwm_max"
    const uint16_t output_pwm = linear_interpolate(pwm_min, pwm_max, rc, -1.0f, 1.0f);

    hal.rcout->write(chan-1, output_pwm);
}

void AP_Periph_FW::translate_rcout_handle_safety_state(uint8_t safety_state)
{
    if (safety_state == 255) {
        hal.rcout->force_safety_off();
    } else {
        hal.rcout->force_safety_on();
    }
}


void AP_Periph_FW::translate_rcout_update()
{
    if (rcout.chan_start != rcout_params.chan_start ||
            rcout.chan_end != rcout_params.chan_end ||
            rcout.protocol != rcout_params.protocol ||
            rcout.act_type != rcout_params.act_type) {
        // allow for run-time configuration
        init_rcout_translator();
    }

#if HAL_NUM_CAN_IFACES > 1
    if (rcout_params.can_out == 1 &&
        rcout.protocol == RCOUTTranslator_Params::RCOUT_KDECAN &&
        rcout_params.kdecan != nullptr) {
        // check if enum needs to run/stop
        if (rcout.kdecan_enum_state != (bool)rcout_params.kdecan_enum_mode) {
            rcout.kdecan_enum_state = (bool)rcout_params.kdecan_enum_mode;
            rcout_params.kdecan->run_enumeration(rcout.kdecan_enum_state);
        }
        // check and transmit new telemetry message is available
        for (uint8_t i = 0; i < rcout.num_channels; i++) {
            AP_KDECAN::telemetry_info_t telem_data = rcout_params.kdecan->read_telemetry(i);
            if (!telem_data.new_data) {
                continue;
            }
            uavcan_equipment_esc_Status esc_telem {};
            esc_telem.esc_index = i + rcout.chan_start;
            esc_telem.current = telem_data.current;
            esc_telem.voltage = telem_data.voltage;
            esc_telem.rpm = telem_data.rpm  * 60UL * 2 / rcout_params.kdecan->get_num_poles();
            can_send_esc_telem(esc_telem);
        }
    }
#endif
}


#endif // HAL_PERIPH_ENABLE_RCOUT_TRANSLATOR

