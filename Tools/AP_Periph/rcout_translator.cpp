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

    AP_GROUPINFO("CHAN_LOW", 1, AP_Periph_FW::RCOUTTranslator_Params, _chan_start, 0),
    AP_GROUPINFO("CHAN_HIGH", 2, AP_Periph_FW::RCOUTTranslator_Params, _chan_end, 0),

    // @Param: RCOUT_PROTOCOL
    // @DisplayName: Enable use of specific RCOUT protocol
    // @Description: Enabling this option starts selected rcout protocol that will be used
    // @Values {CAN_OUT == 1}: 0:UAVCAN,1:KDECAN
    // @Values {CAN_OUT == 0}: maps to RCOutput.h output_mode
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("PROTOCOL", 3, AP_Periph_FW::RCOUTTranslator_Params, _protocol, 1),

#if HAL_NUM_CAN_IFACES > 1
    AP_GROUPINFO("CAN_OUT", 4, AP_Periph_FW::RCOUTTranslator_Params, _can_out, 0),
#endif

    AP_GROUPINFO("FREQ", 5, AP_Periph_FW::RCOUTTranslator_Params, _frequency, 50),

    // @Values : 0:ESC,1:SRV
    AP_GROUPINFO("ACT_TYPE", 6, AP_Periph_FW::RCOUTTranslator_Params, _act_type, 0),

    // For use only under ESC mode
    AP_GROUPINFO("PWM_MIN", 7, AP_Periph_FW::RCOUTTranslator_Params, _pwm_min, 1000),
    AP_GROUPINFO("PWM_MAX", 8, AP_Periph_FW::RCOUTTranslator_Params, _pwm_max, 2000),

#if HAL_NUM_CAN_IFACES > 1
    AP_GROUPINFO("KDE_ENUM", 9, AP_Periph_FW::RCOUTTranslator_Params, _enum_mode, 0),

    // @Group: KDE_
    // @Path: ../AP_KDECAN/AP_KDECAN.cpp
    AP_SUBGROUPPTR(_kdecan, "KDE_", 10, AP_Periph_FW::RCOUTTranslator_Params, AP_KDECAN),
#endif

    AP_GROUPEND
};

#if HAL_NUM_CAN_IFACES > 1
static ChibiOS::CANIface rcout_can_iface(1);
#endif

void AP_Periph_FW::init_rcout_translator() {
    if (rcout_translator._chan_end < rcout_translator._chan_start) {
        return;
    }
    _rcout_protocol = rcout_translator._protocol;
    _num_rcout_channels = rcout_translator._chan_end - rcout_translator._chan_start + 1;

#if HAL_NUM_CAN_IFACES > 1
    if (rcout_translator._can_out == 1) {
        // initialise CAN driver
        rcout_can_iface.init(1000000, AP_HAL::CANIface::NormalMode);

        switch (rcout_translator._protocol) {
            case RCOUTTranslator_Params::RCOUT_KDECAN: {
                rcout_translator._kdecan = new AP_KDECAN;

                if (rcout_translator._kdecan == nullptr) {
                    printf("Failed to allocate KDECAN");
                    return;
                }

                AP_Param::load_object_from_eeprom((AP_KDECAN*)rcout_translator._kdecan, AP_KDECAN::var_info);
                rcout_translator._kdecan->add_interface(&rcout_can_iface);
                rcout_translator._kdecan->init(0, false);
                break;
            }
            case RCOUTTranslator_Params::RCOUT_UAVCAN: {
                break;
            }
        }
    }
#endif
    // Configure non CAN translation via hal rcoutput driver
    AP_HAL::RCOutput::output_mode mode = (AP_HAL::RCOutput::output_mode)_rcout_protocol;
    if (mode == AP_HAL::RCOutput::MODE_PWM_NONE) {
        return;
    }
    uint16_t mask = (1UL << (_num_rcout_channels)) - 1UL;
    for (uint8_t i = 0; i < _num_rcout_channels; i++) {
        hal.rcout->enable_ch(i);
    }
    hal.rcout->set_freq(mask, rcout_translator._frequency);
    hal.rcout->set_output_mode(mask, mode);
    if (rcout_translator._act_type == 0) {
        // only set esc scales for ESC MODE
        hal.rcout->set_esc_scaling(rcout_translator._pwm_min, rcout_translator._pwm_max);
    }
}

void AP_Periph_FW::translate_rcout_esc(int16_t *rc, uint8_t num_channels) {
    if (rc == nullptr || 
        (num_channels <= rcout_translator._chan_end) || 
        _num_rcout_channels == 0 || 
        rcout_translator._act_type == 1) {
        return;
    }
    for (uint8_t i = rcout_translator._chan_start; i <= rcout_translator._chan_end; i++) {
        uint16_t output_pwm = rcout_translator._pwm_min + ((rcout_translator._pwm_max - rcout_translator._pwm_min) * constrain_float(rc[i]/ESC_MAX_VALUE, 0.0f, 1.0f));
        hal.rcout->write(i - rcout_translator._chan_start, output_pwm);
    }
#if HAL_NUM_CAN_IFACES > 1
    if (rcout_translator._can_out == 1) {
        switch (_rcout_protocol) {
            case RCOUTTranslator_Params::RCOUT_KDECAN: {
                if (rcout_translator._kdecan == nullptr) {
                    return;
                }
                rcout_translator._kdecan->lock_rcout();
                for (uint8_t i = rcout_translator._chan_start; i <= rcout_translator._chan_end; i++) {
                    float norm_output = 2.0f*constrain_float(rc[i]/ESC_MAX_VALUE, 0.0f, 1.0f) - 1.0f;
                    rcout_translator._kdecan->set_output(i - rcout_translator._chan_start, norm_output);
                }
                rcout_translator._kdecan->release_rcout();
                break;
            }
            case RCOUTTranslator_Params::RCOUT_UAVCAN:
                break;
        }
    }
#endif
}

void AP_Periph_FW::translate_rcout_srv(uint8_t chan, float rc) {
    if (rcout_translator._act_type == 0) {
        return;
    }
    if (chan < rcout_translator._chan_start || chan > rcout_translator._chan_end) {
        return;
    }
    uint16_t output_pwm = 1000 + constrain_float(500.0f*(rc + 1.0f), 0.0f, 1000.0f);
    hal.rcout->write(chan, output_pwm);
}

void AP_Periph_FW::translate_rcout_handle_safety_state(uint8_t safety_state) {
    if (safety_state == 255) {
        hal.rcout->force_safety_off();
    } else {
        hal.rcout->force_safety_on();
    }
}


void AP_Periph_FW::translate_rcout_update() {
#if HAL_NUM_CAN_IFACES > 1
    if (rcout_translator._can_out == 1 && 
        _rcout_protocol == RCOUTTranslator_Params::RCOUT_KDECAN &&
        rcout_translator._kdecan != nullptr) {
        // check if enum needs to run/stop
        if (_enum_state != (bool)rcout_translator._enum_mode) {
            _enum_state = (bool)rcout_translator._enum_mode;
            rcout_translator._kdecan->run_enumeration(_enum_state);
        }
        // check and transmit new telemetry message is available
        for (uint8_t i = 0; i < _num_rcout_channels; i++) {
            AP_KDECAN::telemetry_info_t telem_data = rcout_translator._kdecan->read_telemetry(i);
            if (!telem_data.new_data) {
                continue;
            }
            uavcan_equipment_esc_Status esc_telem {};
            esc_telem.esc_index = i + rcout_translator._chan_start;
            esc_telem.current = telem_data.current;
            esc_telem.voltage = telem_data.voltage;
            esc_telem.rpm = telem_data.rpm  * 60UL * 2 / rcout_translator._kdecan->get_num_poles();
            can_send_esc_telem(esc_telem);
        }
    }
#endif
}


#endif // HAL_PERIPH_ENABLE_RCOUT_TRANSLATOR