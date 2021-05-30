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

#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/AP_HAL_Boards.h>

#ifndef HAL_AP_ESC_ENABLED
#define HAL_AP_ESC_ENABLED (!defined(HAL_BUILD_AP_PERIPH) && !HAL_MINIMIZE_FEATURES && BOARD_FLASH_SIZE > 1024) || (defined(HAL_BUILD_AP_PERIPH) && defined(HAL_PERIPH_ENABLE_AP_ESC))
#endif

#if HAL_AP_ESC_ENABLED

#include <AP_Param/AP_Param.h>
#include <stdio.h>
class AP_ESC
{
    public:
        //constructor
        AP_ESC() {
            singleton = this;
            AP_Param::setup_object_defaults(this, var_info);
        }

        /* Do not allow copies */
        AP_ESC(const AP_ESC &other) = delete;
        AP_ESC &operator=(const AP_ESC&) = delete;

        enum class EscType : uint8_t {
            Disabled            = 0,
            FOC                 = 1,
        };

        // get singleton instance
        static AP_ESC *get_singleton(void) {
            return singleton;
        }
        
        static const struct AP_Param::GroupInfo var_info[];

        // run-once init
        void init();

        // update slow, usually 10Hz in ap_periph/vehicle thread
        void update();

        // update fast (1Kz) in ap_periph/vehicle thread
        void update_fast();

        //some input as demand

        // 
        uint8_t load_adjustment;
        struct Phases
        {
            // measuring is accomplished by calculating the differenc between the two driven phases
            // and then  
            bool measuring;
            uint8_t DTCY_percent;
            uint8_t Table_Offset;
        };


        // 50,53,57,60,64,67,70,73,76,79,
        // 82,85,87,89,91,93,95,96,98,99,
        // 99,100,100,100,100,99,99,98,96,95,
        // 93,91,89,87,85,82,79,76,73,70,
        // 67,64,60,57,53,50,47,43,40,36,
        // 33,30,27,24,21,18,15,13,11,9,
        // 7,5,4,2,1,1,0,0,0,0,
        // 1,1,2,4,5,7,9,11,13,15,
        // 18,21,24,27,30,33,36,40,43,47
        struct Three_Phase_Control
        {
            // Duty cycle sine table 90 pionts MUST BE MULTIPLE OF 3
           const uint8_t sine_table[90] = {   50,53,57,60,64,67,70,73,76,79,
                                        82,85,87,89,91,93,95,96,98,99,
                                        99,99,99,99,99,99,99,98,96,95,
                                        93,91,89,87,85,82,79,76,73,70,
                                        67,64,60,57,53,50,47,43,40,36,
                                        33,30,27,24,21,18,15,13,11,9,
                                        7,5,4,2,1,1,1,1,1,1,
                                        1,1,2,4,5,7,9,11,13,15,
                                        18,21,24,27,30,33,36,40,43,47 };
            uint16_t phase_currents_meassered[3];
            uint16_t phase_currents_integration;
            uint8_t master_table_position;
            uint16_t motor_angle;
            uint16_t rotor_speed;
            Phases phase[3];
            

        };
       

        // handle incoming RawCommand UAVCAN packets
        void handle_can_rx(uint8_t source_id, const int16_t *rc, uint8_t num_channels);

        void Update_table_positions_on_interupt(void);

        void get_Curents_on_interupt(void);

        uint16_t rotor_position(void);

        long map_to_table(long x, long in_min, long in_max, long out_min, long out_max);

    private:
        static AP_ESC *singleton;

        // tick - main call in it's own thread running as fast as it can ( >1kHz )
        void tick(void);

        uint32_t last_tick1Hz_ms;
        bool initialized;

        AP_Enum<EscType> escType;
        AP_Float debug1;
        AP_Float debug2;
        AP_Float debug3;
        AP_Int32 esc_freq;

        float adc_voltage;

        AP_HAL::AnalogSource *adc_pin;
};

namespace AP {
    AP_ESC *esc();
};


// REGISTER TIMER 1 PHASE A CALLBACK FOR CURRNT CONTROL
// REGISTER TIMER 1 PHASE B CALLBACK FOR CURRNT CONTROL
// REGISTER TIMER 1 PHASE C CALLBACK FOR CURRNT CONTROL

// REGISTER TIMER 1 PHASE A CALLBACK FOR CURRENT SENCE
// REGISTER TIMER 1 PHASE B CALLBACK FOR CURRENT SENCE
// REGISTER TIMER 1 PHASE C CALLBACK FOR CURRENT SENCE

// UAVCAN gui_tool
// https://files.zubax.com/products/org.uavcan.gui_tool/

// params:
// https://ardupilot.org/plane/docs/parameters.html

// ./waf build NucleoH743-periph

// ./waf configure --board NucleoH743

// now us
// ./waf configure --board NucleoH743-periph




 #endif // HAL_AP_ESC_ENABLED

// TIMER 1 PHASE A CALLBACK FOR CURRNT CONTROL
// TIMER 1 PHASE B CALLBACK FOR CURRNT CONTROL
// TIMER 1 PHASE C CALLBACK FOR CURRNT CONTROL

// TIMER 1 PHASE A CALLBACK FOR CURRENT SENCE
// TIMER 1 PHASE B CALLBACK FOR CURRENT SENCE
// TIMER 1 PHASE C CALLBACK FOR CURRENT SENCE