/*
   Copyright (C) 2021 Kraus Hamdani Aerospace. All rights reserved.
  
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

   ArduPilot in-house Eelectronic Speed Controller (ESC) for Motors
   by David Maye <David.Maye@krausaerospace.com> and Tom Pittenger <Tom.Pittenger@krausaerospace.com>
*/

#include "AP_ESC.h"
#include <SRV_Channel/SRV_Channel.h>

#if HAL_AP_ESC_ENABLED

extern const AP_HAL::HAL& hal;
AP_ESC *AP_ESC::singleton;

#ifndef HAL_AP_ESC_TYPE_DEFAULT
    #define HAL_AP_ESC_TYPE_DEFAULT 0
#endif
#ifndef HAL_AP_ESC_FREQ_DEFAULT
    #define HAL_AP_ESC_FREQ_DEFAULT 123
#endif

#define UAVCAN_ESC_MAX_VALUE    8191


// table of user settable parameters
const AP_Param::GroupInfo AP_ESC::var_info[] = {

    // @Param: TYPE
    // @DisplayName: Type
    // @Description: Type
    // @Values: 0:Disabled,1:FOC
    // @User: Standard
    AP_GROUPINFO_FLAGS("TYPE", 1, AP_ESC, escType, HAL_AP_ESC_TYPE_DEFAULT, AP_PARAM_FLAG_ENABLE),

    // @Param: DEBUG1
    // @DisplayName: DEBUG value
    // @Description: DEBUG
    // @User: Standard
    AP_GROUPINFO("DEBUG1", 2, AP_ESC, debug1, 0),
    
    // @Param: DEBUG2
    // @DisplayName: DEBUG value
    // @Description: DEBUG
    // @User: Standard
    AP_GROUPINFO("DEBUG2", 3, AP_ESC, debug2, 0),
    
    // @Param: DEBUG3
    // @DisplayName: DEBUG value
    // @Description: DEBUG
    // @User: Standard
    AP_GROUPINFO("DEBUG3", 4, AP_ESC, debug3, 0),
    
    AP_GROUPINFO("FREQ", 5, AP_ESC, esc_freq, HAL_AP_ESC_FREQ_DEFAULT),
    
    AP_GROUPEND
};

/*
 * init function intended to run once but lazy inits are allowed
 */
void AP_ESC::init()
{
    if (initialized || escType == EscType::Disabled) {
        return;
    }

    hal.scheduler->register_io_process(FUNCTOR_BIND_MEMBER(&AP_ESC::tick, void));
    initialized = true;


    uint16_t esc_mask = 0;
    for (uint8_t i=0; i<6; i++) {
        const SRV_Channel::Aux_servo_function_t func = (SRV_Channel::Aux_servo_function_t)((int)SRV_Channel::k_h_bridge_A_high + i); 

        SRV_Channels::set_range(func, UAVCAN_ESC_MAX_VALUE);
        uint8_t chan;
        if (SRV_Channels::find_channel(func, chan)) {
            esc_mask |= 1U << chan;
            SRV_Channels::set_rc_frequency(func, esc_freq);
        }
    }

    hal.rcout->set_output_mode(esc_mask, AP_HAL::RCOutput::MODE_PWM_BRUSHED);
}

/*
 * periodic update to handle low-speed events, typically 10Hz
 * NOTE: this is done in vehicle/ap_periph thread scope
 */
void AP_ESC::update(void)
{
    if (!initialized) {
        // allow for lazy init
        init();
        return;
    }
}

/*
 * periodic update to handle high-speed events, typically 1kHz
 * NOTE: this is done in vehicle/ap_periph thread scope
 */
void AP_ESC::update_fast()
{
    if (!initialized) {
        return;
    }
}

/*
 * handle inbound UAVCAN.RawCommand packets
 * NOTE: this is done in vehicle/ap_periph's UAVCAN handler thread scope
 */
void AP_ESC::handle_can_rx(uint8_t source_id, const int16_t *rc, uint8_t num_channels)
{

}

/*
 * Thread background tick. This runs as fast as it can.
 * NOTE: This is in it's own AP_ESC thread scope
 */
void AP_ESC::tick(void)
{
    static uint32_t this_time = 0;
    static int i = 0; 
    const uint32_t now = AP_HAL::millis();
    if (now - last_tick1Hz_ms >= 1000) {
        last_tick1Hz_ms = now;
        // do somehting every 1000ms (1Hz)
        if (debug1 > 5) {
            hal.console->printf("now=%d\n", (int)now);
        }

    }
     if (now - this_time >= 100) {
        this_time = now;
        
        SRV_Channels::set_output_scaled(SRV_Channel::k_rcin5, abs(debug2));
        SRV_Channels::set_output_scaled(SRV_Channel::k_rcin6, abs(debug2));
        i++;
        if(i == 1023) i = 0;
    }

    // need all 6


    //k_h_bridge_A_high

}


#endif // HAL_AP_ESC