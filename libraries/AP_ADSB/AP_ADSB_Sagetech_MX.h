#pragma once

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

#include "AP_ADSB_Backend.h"

#ifndef HAL_ADSB_SAGETECH_MX_ENABLED
#define HAL_ADSB_SAGETECH_MX_ENABLED HAL_ADSB_ENABLED
#endif

#if HAL_ADSB_SAGETECH_MX_ENABLED
class AP_ADSB_Sagetech_MX : public AP_ADSB_Backend {
public:
    using AP_ADSB_Backend::AP_ADSB_Backend;

    // init - performs any required initialisation for this instance
    bool init() override;

    // update - should be called periodically
    void update() override;

    // static detection function
    static bool detect();

private:

    static const uint32_t PAYLOAD_MX_MAX_SIZE  = 63;

    enum class SystemStateBits {
        Error_Transponder       = (1U<<0),
        Altitidue_Source        = (1U<<1),
        Error_GPS               = (1U<<2),
        Error_ICAO              = (1U<<3),
        Error_Over_Temperature  = (1U<<4),
        Error_Extended_Squitter = (1U<<5),
        Mode_Transponder        = (3U<<6),   // 2 bit status:
    };


    enum class Transponder_Type {
        Mode_C                  = 0x00,
        Mode_S_ADSB_OUT         = 0x01,
        Mode_S_ADSB_OUT_and_IN  = 0x02,
        Unknown                 = 0xFF,
    };

    enum class MsgType_MX {
        //sending
        INVALID                 = 0,
        Installation_Set        = 0x01,
        Flight_ID               = 0x02,
        Operating_Set           = 0x03,
        GPS_Set                 = 0x04,
        Request                 = 0x05,
        Target_Request          = 0x0B,
        Mode                    = 0x0C,
        //recieving
        ACK                     = 0x80,
        Installation_Response   = 0x81,
        Preflight_Response      = 0x82,
        Status_Response         = 0x83,
        Mode_setting            = 0x8C,
        Version_Response        = 0x8E,
        Serial_Number_Response  = 0x8F,
        Target_Summary_Report   = 0x90,
        ADSB_StateVector_Report = 0x91,
        ADSB_ModeStatus_Report  = 0x92,
        TISB_StateVector_Report = 0x93,
        TISB_ModeStatus_Report  = 0x94,
        TISB_CorasePos_Report   = 0x95,
        TISB_ADSB_Mgr_Report    = 0x96,
        ADSB_TargetState_Report = 0x97,
        ADSB_AirRef_Velo_Report = 0x98,

    };
    
       enum class ParseState {
        WaitingFor_Start,
        WaitingFor_MsgType,
        WaitingFor_MsgId,
        WaitingFor_PayloadLen,
        WaitingFor_PayloadContents,
        WaitingFor_Checksum,
        WaitingFor_End,
    };


    struct Packet_MX {
        const uint8_t   start = 0xAA;
        const uint8_t   assemAddr = 0x01;
        MsgType_MX      type;
        uint8_t         id;
        uint8_t         payload_length;
        uint8_t         payload[PAYLOAD_MX_MAX_SIZE];
        uint8_t         checksum;
    };

    struct {
        ParseState      state;
        uint8_t         index;
        Packet_MX       packet;
    } message_in;

    // compute Sum and FletcherSum values
    uint16_t checksum_generate_MX(Packet_MX &msg) const;
    bool checksum_verify_MX(Packet_MX &msg) const;
    void checksum_assign_MX(Packet_MX &msg);


    // handling inbound byte and process it in the state machine
    bool parse_byte_MX(const uint8_t data);

    // handle inbound packet
    void handle_packet_MX(const Packet_MX &msg);

    // send message to serial port
    void send_msg(Packet_MX &msg);


    // handle inbound msgs
    void handle_adsb_in_msg(const Packet_MX &msg);
    void handle_ack(const Packet_MX &msg);


    // send messages to to transceiver
    void send_msg_Installation();
    void send_msg_PreFlight();
    void send_msg_Operating();
    void send_msg_GPS();
    void send_msg_Resquest(const MsgType_MX type);
    void send_msg_Tartget_Request();
    void send_msg_Mode();
    // send packet by type
    void send_packet(const MsgType_MX type);


    // Convert base 8 or 16 to decimal. Used to convert an octal/hexadecimal value
    // stored on a GCS as a string field in different format, but then transmitted
    // over mavlink as a float which is always a decimal.
    uint32_t convert_base_to_decimal(const uint8_t baseIn, uint32_t inputNumber);

    // timers for each out-bound packet
    uint32_t        last_packet_initialize_ms;
    uint32_t        last_packet_PreFlight_ms;
    uint32_t        last_packet_GPS_ms;
    uint32_t        last_packet_Operating_ms;

    // cached variables to compare against params so we can send msg on param change.
    uint16_t        last_operating_squawk;
    int32_t         last_operating_alt;
    uint8_t         last_operating_rf_select;

    // track status changes in acks
    uint8_t         last_ack_transponder_mode;
    Transponder_Type transponder_type = Transponder_Type::Unknown;
};
#endif // HAL_ADSB_SAGETECH_MX_ENABLED

