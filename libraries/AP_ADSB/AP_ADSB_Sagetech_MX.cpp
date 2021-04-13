/*
   Copyright (C) 2020  Kraus Hamdani Aerospace Inc. All rights reserved.

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

#include "AP_ADSB_Sagetech_MX.h"

#if HAL_ADSB_SAGETECH_MX_ENABLED
#include <GCS_MAVLink/GCS.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_RTC/AP_RTC.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <stdio.h>
#include <time.h>
#include <string.h>
#include <math.h>

#define SAGETECH_SCALER_LATLNG              (1.0f/2.145767E-5f)     //   180/(2^23)
#define SAGETECH_SCALER_KNOTS_TO_CMS        ((KNOTS_TO_M_PER_SEC/0.125f) * 100.0f)
#define SAGETECH_SCALER_ALTITUDE            (1.0f/0.015625f)
#define SAGETECH_SCALER_HEADING_CM          ((360.0f/256.0f) * 100.0f)

#define SAGETECH_VALIDFLAG_LATLNG           (1U<<0)
#define SAGETECH_VALIDFLAG_ALTITUDE         (1U<<1)
#define SAGETECH_VALIDFLAG_VELOCITY         (1U<<2)
#define SAGETECH_VALIDFLAG_GND_SPEED        (1U<<3)
#define SAGETECH_VALIDFLAG_HEADING          (1U<<4)
#define SAGETECH_VALIDFLAG_V_RATE_GEO       (1U<<5)
#define SAGETECH_VALIDFLAG_V_RATE_BARO      (1U<<6)
#define SAGETECH_VALIDFLAG_EST_LATLNG       (1U<<7)
#define SAGETECH_VALIDFLAG_EST_VELOCITY     (1U<<8)

// detect if any port is configured as Sagetech
bool AP_ADSB_Sagetech_MX::detect()
{
    return (AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_ADSB, 0) != nullptr);
}

// Init, called once after class is constructed
bool AP_ADSB_Sagetech_MX::init()
{
    _port = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_ADSB, 0);

    return (_port != nullptr);
}

void AP_ADSB_Sagetech_MX::update()
{
    if (_port == nullptr) {
        return;
    }

    const uint32_t now_ms = AP_HAL::millis();

    // -----------------------------
    // read any available data on serial port
    // -----------------------------
    uint32_t nbytes = MIN(_port->available(), 10 * PAYLOAD_MX_MAX_SIZE);
    while (nbytes-- > 0) {
        const int16_t data = (uint8_t)_port->read();
        if (data < 0) {
            break;
        }
        if (parse_byte_MX((uint8_t)data)) {
            handle_packet_MX(message_in.packet);
        }
    } // while nbytes


    // -----------------------------
    // handle timers for generating data
    // -----------------------------
    if (!last_packet_initialize_ms || (now_ms - last_packet_initialize_ms >= 5000)) {
        last_packet_initialize_ms = now_ms;
        send_packet(MsgType_MX::Installation_Set);

    } else if (!last_packet_PreFlight_ms || (now_ms - last_packet_PreFlight_ms >= 8200)) {
        last_packet_PreFlight_ms = now_ms;
        // TODO: allow callsign to not require a reboot
        send_packet(MsgType_MX::Preflight_Set);

    } else if (now_ms - last_packet_Operating_ms >= 1000 && (
            // send once at boot
            last_packet_Operating_ms == 0 || 
            // send as data changes
            last_operating_squawk != _frontend.out_state.cfg.squawk_octal ||
            abs(last_operating_alt - _frontend._my_loc.alt) > 1555 ||      // 1493cm == 49ft. The output resolution is 100ft per bit
            last_operating_rf_select != _frontend.out_state.cfg.rfSelect))
    {
        last_packet_Operating_ms = now_ms;
        last_operating_squawk = _frontend.out_state.cfg.squawk_octal;
        last_operating_alt = _frontend._my_loc.alt;
        last_operating_rf_select = _frontend.out_state.cfg.rfSelect;
        send_packet(MsgType_MX::Operating_Set);

    } else if (now_ms - last_packet_GPS_ms >= (_frontend.out_state.is_flying ? 200 : 1000)) {
        // 1Hz when not flying, 5Hz when flying
        last_packet_GPS_ms = now_ms;
        send_packet(MsgType_MX::GPS_Set);
    }
}

//sending packets
void AP_ADSB_Sagetech_MX::send_packet(const MsgType_MX type)
{
    switch (type) {
    case MsgType_MX::Installation_Set:
        send_msg_Installation();
        break;
    case MsgType_MX::Flight_ID:
        send_msg_PreFlight();
        break;
    case MsgType_MX::Operating_Set:
        send_msg_Operating();
        break;
    case MsgType_MX::GPS_Set:
        send_msg_GPS();
        break;
    case MsgType_MX::Request:
        send_msg_Request(const MsgType_MX type);   //add function ...........................
        break;

    case MsgType_MX::Request:
        send_msg_Target_Request();   //add function ...........................
        break;
    case MsgType_MX::Request:
        send_msg_mode();   //add function ...........................
        break;
    default:
        break;
    }
}




void AP_ADSB_Sagetech_MX::handle_packet_MX(const Packet_MX &msg)
{
    switch (msg.type) {
        case MsgType_MX::ACK:
            handle_ack(msg);
            break;

        case MsgType_MX::Installation_Response:
        case MsgType_MX::Preflight_Response:
        case MsgType_MX::Status_Response:
            // TODO add support for these
            break;

        case MsgType_MX::ADSB_StateVector_Report:
        case MsgType_MX::ADSB_ModeStatus_Report:
        case MsgType_MX::TISB_StateVector_Report:
        case MsgType_MX::TISB_ModeStatus_Report:
        case MsgType_MX::TISB_CorasePos_Report:
        case MsgType_MX::TISB_ADSB_Mgr_Report:
            handle_adsb_in_msg(msg);
            break;

        case MsgType_MX::Installation_Set:
        case MsgType_MX::Preflight_Set:
        case MsgType_MX::Operating_Set:
        case MsgType_MX::GPS_Set:
        case MsgType_MX::Request:
            // these are out-bound only and are not expected to be received
        case MsgType_MX::INVALID:
        break;
    }
}

void AP_ADSB_Sagetech_MX::handle_ack(const Packet_MX &msg)
{
    // ACK received!
    const uint8_t system_state = msg.payload[2];
    transponder_type = (Transponder_Type)msg.payload[6];

    const char* rfmode = "RF mode: ";
    const uint8_t prev_transponder_mode = last_ack_transponder_mode;
    last_ack_transponder_mode = (system_state >> 6) & 0x03;
    if (prev_transponder_mode != last_ack_transponder_mode) {
        switch (last_ack_transponder_mode) {
        case 0: gcs().send_text(MAV_SEVERITY_INFO, "ADSB: %sOFF",   rfmode); break;
        case 1: gcs().send_text(MAV_SEVERITY_INFO, "ADSB: %sSTBY",  rfmode); break;
        case 2: gcs().send_text(MAV_SEVERITY_INFO, "ADSB: %sON",    rfmode); break;
        case 3: gcs().send_text(MAV_SEVERITY_INFO, "ADSB: %sON-ALT",rfmode); break;
        default:  break;
        }
    }
}

void AP_ADSB_Sagetech_MX::handle_adsb_in_msg(const Packet_MX &msg)
{
    AP_ADSB::adsb_vehicle_t vehicle {};

    vehicle.last_update_ms = AP_HAL::millis();

    switch (msg.type) {
    case MsgType_MX::ADSB_StateVector_Report: { // 0x91
        const uint16_t validFlags = le16toh_ptr(&msg.payload[8]);   //research endianess
        vehicle.info.ICAO_address = le24toh_ptr(&msg.payload[10]);

        if (validFlags & SAGETECH_VALIDFLAG_LATLNG) {
            vehicle.info.lat = ((int32_t)le24toh_ptr(&msg.payload[20])) * SAGETECH_SCALER_LATLNG;
            vehicle.info.lon = ((int32_t)le24toh_ptr(&msg.payload[23])) * SAGETECH_SCALER_LATLNG;
            vehicle.info.flags |= ADSB_FLAGS_VALID_COORDS;
        }

        if (validFlags & SAGETECH_VALIDFLAG_ALTITUDE) {
            vehicle.info.altitude = (int32_t)le24toh_ptr(&msg.payload[26]);
            vehicle.info.flags |= ADSB_FLAGS_VALID_ALTITUDE;
        }

        if (validFlags & SAGETECH_VALIDFLAG_VELOCITY) {
            const float velNS = ((int32_t)le16toh_ptr(&msg.payload[29])) * SAGETECH_SCALER_KNOTS_TO_CMS;
            const float velEW = ((int32_t)le16toh_ptr(&msg.payload[31])) * SAGETECH_SCALER_KNOTS_TO_CMS;
            vehicle.info.hor_velocity = Vector2f(velEW, velNS).length();
            vehicle.info.flags |= ADSB_FLAGS_VALID_VELOCITY;
        }

        if (validFlags & SAGETECH_VALIDFLAG_HEADING) {
            vehicle.info.heading = ((float)msg.payload[29]) * SAGETECH_SCALER_HEADING_CM;
            vehicle.info.flags |= ADSB_FLAGS_VALID_HEADING;
        }

        if ((validFlags & SAGETECH_VALIDFLAG_V_RATE_GEO) || (validFlags & SAGETECH_VALIDFLAG_V_RATE_BARO)) {
            vehicle.info.ver_velocity = (int16_t)le16toh_ptr(&msg.payload[38]);
            vehicle.info.flags |= ADSB_FLAGS_VERTICAL_VELOCITY_VALID;
        }

        _frontend.handle_adsb_vehicle(vehicle);
        break;
    }
    case MsgType_MX::ADSB_ModeStatus_Report:   // 0x92
        vehicle.info.ICAO_address = le24toh_ptr(&msg.payload[9]);

        if (msg.payload[16] != 0) {
            // if string is non-null, consider it valid
            memcpy(&vehicle.info, &msg.payload[16], 8);
            vehicle.info.flags |= ADSB_FLAGS_VALID_CALLSIGN;
        }

        _frontend.handle_adsb_vehicle(vehicle);
        break;
    case MsgType_MX::TISB_StateVector_Report:
    case MsgType_MX::TISB_ModeStatus_Report:
    case MsgType_MX::TISB_CorasePos_Report:
    case MsgType_MX::TISB_ADSB_Mgr_Report:
        // TODO
        return;

    default:
        return;
    }

}

// handling inbound byte and process it in the state machine
// return true when a full packet has been received
bool AP_ADSB_Sagetech_MX::parse_byte_MX(const uint8_t data)
{
    switch (message_in.state) {
        default:
        case ParseState::WaitingFor_Start:
            if (data == 0xA5) {
                message_in.state = ParseState::WaitingFor_AssmAddr;
            }
            break;
        case ParseState::WaitingFor_AssmAddr:
            message_in.state = (data == 0x01) ? ParseState::WaitingFor_MsgType : ParseState::WaitingFor_Start;
            break;
        case ParseState::WaitingFor_MsgType:
            message_in.packet.type = static_cast<MsgType_MX>(data);
            message_in.state = ParseState::WaitingFor_MsgId;
            break;
        case ParseState::WaitingFor_MsgId:
            message_in.packet.id = data;
            message_in.state = ParseState::WaitingFor_PayloadLen;
            break;
        case ParseState::WaitingFor_PayloadLen:
            message_in.packet.payload_length = data;
            message_in.index = 0;
            message_in.state = (data == 0) ? ParseState::WaitingFor_Checksum : ParseState::WaitingFor_PayloadContents;
            break;
        case ParseState::WaitingFor_PayloadContents:
            message_in.packet.payload[message_in.index++] = data;
            if (message_in.index >= message_in.packet.payload_length) {
                message_in.state = ParseState::WaitingFor_Checksum;
                message_in.index = 0;
            }
            break;

        case ParseState::WaitingFor_Checksum:
            message_in.packet.checksum = data;
            message_in.state = ParseState::WaitingFor_End;
            if (checksum_verify_MX(message_in.packet)) {
                return true;
            }
            break;

        case ParseState::WaitingFor_End:
            // we don't care if the end value matches
            message_in.state = ParseState::WaitingFor_Start;
            break;
    }
    return false;
}

// compute Sum and FletcherSum values into a single value
// returns uint16_t with MSByte as Sum and LSByte FletcherSum
uint16_t AP_ADSB_Sagetech_MX::checksum_generate_MX(Packet_MX &msg) const
{
    uint8_t sum = 0;
    uint8_t sumFletcher = 0;

    const uint8_t header_message_format[5] {
            0xA5,   // start
            0x01,   // assembly address
            static_cast<uint8_t>(msg.type),
            msg.id,
            msg.payload_length
    };

    for (uint8_t i=0; i<5; i++) {
        sum += header_message_format[i];
        sumFletcher += sum;
    }
    for (uint8_t i=0; i<msg.payload_length; i++) {
        sum += msg.payload[i];
        sumFletcher += sum;
    }

    return UINT16_VALUE(sum, sumFletcher);
}

// computes checksum and returns true if it matches msg checksum
bool AP_ADSB_Sagetech_MX::checksum_verify_MX(Packet_MX &msg) const
{
    const uint16_t checksum = checksum_generate_MX(msg);
    return (HIGHBYTE(checksum) == msg.checksum) && (LOWBYTE(checksum) == msg.checksumFletcher);
}

// computes checksum and assigns checksum values to msg
void AP_ADSB_Sagetech_MX::checksum_assign_MX(Packet_MX &msg)
{
    const uint16_t checksum = checksum_generate_MX(msg);
    msg.checksum = HIGHBYTE(checksum);
    msg.checksumFletcher = LOWBYTE(checksum);
}

// send message to serial port
void AP_ADSB_Sagetech_MX::send_msg(Packet_MX &msg)
{
    // generate and populate checksums.
    checksum_assign_MX(msg);

    const uint8_t message_format_header[5] {
            0xA5,   // start
            0x01,   // assembly address
            static_cast<uint8_t>(msg.type),
            msg.id,
            msg.payload_length
    };
    const uint8_t message_format_tail[3] {
            msg.checksumFletcher,
            msg.checksum,
            0x5A    // end
    };

    if (_port != nullptr) {
        _port->write(message_format_header, sizeof(message_format_header));
        _port->write(msg.payload, msg.payload_length);
        _port->write(message_format_tail, sizeof(message_format_tail));
    }
}

// build the installation message
void AP_ADSB_Sagetech_MX::send_msg_Installation()    // ??? mode no longer handled here***********************
{
    Packet_MX pkt {};

    pkt.type = MsgType_MX::Installation_Set;
    pkt.payload_length = 36; // 28 == 0x1C

    // Mode C = 3, Mode S = 0
    pkt.id = (transponder_type == Transponder_Type::Mode_C) ? 3 : 0;

    // convert a decimal 123456 to 0x123456
    // TODO: do a proper conversion. The param contains "131313" 
    // but what gets transmitted over the air is 0x200F1.
    const uint32_t icao_hex = convert_base_to_decimal(16, _frontend.out_state.cfg.ICAO_id_param);
    //put_le24_ptr(&pkt.payload[0], icao_hex);
    memcpy(&pkt.payload[0], &icao_hex, 3);      // ICAO address 
    memcpy(&pkt.payload[3], &_frontend.out_state.cfg.callsign, 7);  // make sure callsign is no longer than 7 bytes*********************
    //two bytes reserved        

    pkt.payload[12] = 0;                // COM Port 0 baud, fixed at 57600
    pkt.payload[13] = 0;                // COM Port 1 baud, fixed at 57600

    (uint32_t)(*pkt.payload[14]) = 0;     // IP Adress
    (uint32_t)(*pkt.payload[18]) = 0;     // Netmask

    (uint16_t)(*pkt.payload[22]) = 1;      // GPS from COM port 0 (this port) *******************

    pkt.payload[24] = 1;                // GPS Integrity

    pkt.payload[25] = _frontend.out_state.cfg.emitterType / 8;      // Emitter Set
    pkt.payload[26] = _frontend.out_state.cfg.emitterType & 0x0F;   // Emitter Type/catagory

    pkt.payload[27] = _frontend.out_state.cfg.lengthWidth;          // Aircraft Size
    pkt.payload[28] = 0;                // airspeed MAX
    (uint16_t)(*pkt.payload[29]) = 0;      // Altitude Encoder Offset
    //two bytes reserved
    //antenna bottom, reserved, host alt resolutio, heading-true, airspd-true, pres sense heat disabled, weight in wheels
    pkt.payload[33] = 0x5C;             // inatallation configuration ******************* not sure TODO
    //two bytes reserved
    send_msg(pkt);
}

void AP_ADSB_Sagetech_MX::send_msg_PreFlight()
{
    Packet_MX pkt {};

    pkt.type = MsgType_MX::Preflight_Set;
    pkt.id = 0;
    pkt.payload_length = 10;

    memcpy(&pkt.payload[0], &_frontend.out_state.cfg.callsign, 8);

    memset(&pkt.payload[8], 0, 2);

    send_msg(pkt);
}

void AP_ADSB_Sagetech_MX::send_msg_Operating()
{
    Packet_MX pkt {};

    pkt.type = MsgType_MX::Operating_Set;
    pkt.id = 0;
    pkt.payload_length = 12;

    // squawk
    //squawk = convert_base_to_decimal(8, last_operating_squawk);
    //put_le16_ptr(&pkt.payload[0], squawk);
    (uint16_t)(*pkt.payload[0]) = last_operating_squawk;


    // RF mode
    pkt.payload[4] = last_operating_rf_select;

    // altitude
    if (_frontend.out_state.cfg.rf_capable & 0x01) {
        const float alt_meters = last_operating_alt * 0.01f;
        const int32_t alt_feet = (int32_t)(alt_meters * FEET_TO_METERS); 
        const int16_t alt_feet_adj = (alt_feet + 50) / 100; // 1 = 100 feet, 1 = 149 feet, 5 = 500 feet
        put_le16_ptr(&pkt.payload[2], alt_feet_adj);

    } else {
        // use integrated altitude - recommend by sagetech
        (uint_16)(*pkt.payload[2]) = 0x80;
        pkt.payload[3] = 0x00;
    }

    

    send_msg(pkt);
}

void AP_ADSB_Sagetech_MX::send_msg_GPS()
{
    Packet_MX pkt {};

    pkt.type = MsgType_MX::GPS_Set;
    pkt.payload_length = 52;
    pkt.id = 0;

    const int32_t longitude = _frontend._my_loc.lng;
    const int32_t latitude =  _frontend._my_loc.lat;

    // longitude and latitude
    // NOTE: these MUST be done in double or else we get roundoff in the maths
    const double lon_deg = longitude * (double)1.0e-7 * (longitude < 0 ? -1 : 1);
    const double lon_minutes = (lon_deg - int(lon_deg)) * 60;
    snprintf((char*)&pkt.payload[0], 12, "%03u%02u.%05u", (unsigned)lon_deg, (unsigned)lon_minutes, unsigned((lon_minutes - (int)lon_minutes) * 1.0E5));

    const double lat_deg = latitude * (double)1.0e-7 * (latitude < 0 ? -1 : 1);
    const double lat_minutes = (lat_deg - int(lat_deg)) * 60;
    snprintf((char*)&pkt.payload[11], 11, "%02u%02u.%05u", (unsigned)lat_deg, (unsigned)lat_minutes, unsigned((lat_minutes - (int)lat_minutes) * 1.0E5));

    // ground speed
    const Vector2f speed = AP::ahrs().groundspeed_vector();
    float speed_knots = norm(speed.x, speed.y) * M_PER_SEC_TO_KNOTS;
    snprintf((char*)&pkt.payload[21], 7, "%03u.%02u", (unsigned)speed_knots, unsigned((speed_knots - (int)speed_knots) * 1.0E2));

    // heading
    float heading = wrap_360(degrees(speed.angle()));
    snprintf((char*)&pkt.payload[27], 10, "%03u.%04u", unsigned(heading), unsigned((heading - (int)heading) * 1.0E4));

    // hemisphere
    uint8_t hemisphere = 0;
    hemisphere |= (latitude >= 0) ? 0x01 : 0;   // isNorth
    hemisphere |= (longitude >= 0) ? 0x02 : 0;  // isEast
    hemisphere |= (AP::gps().status() < AP_GPS::GPS_OK_FIX_2D) ? 0x80 : 0;  // isInvalid
    pkt.payload[35] = hemisphere;

    // time
    uint64_t time_usec;
    if (AP::rtc().get_utc_usec(time_usec)) {
        // not completely accurate, our time includes leap seconds and time_t should be without
        const time_t time_sec = time_usec / 1000000;
        struct tm* tm = gmtime(&time_sec);

        // format time string
        snprintf((char*)&pkt.payload[36], 11, "%02u%02u%06.3f", tm->tm_hour, tm->tm_min, tm->tm_sec + (time_usec % 1000000) * 1.0e-6);
    } else {
        memset(&pkt.payload[36],' ', 10);
    }

    send_msg(pkt);
}

void AP_ADSB_Sagetech_MX::send_msg_Resquest(const MsgType_MX type)
{
    // set all bytes in packet to 0 via {} so we only need to set the ones we need to
    Packet_MX pkt {};

    pkt.type = MsgType_MX::Request;
    pkt.id = 0;
    pkt.payload_length = 4;

    pkt.payload[0] = static_cast<uint8_t>(type);

    send_msg(pkt);
}

    void send_msg_Tartget_Request();    //**********************************************


    void send_msg_Mode()               //**********************************************
    {
        Packet_MX pkt {};

        pkt.type = MsgType_MX::Mode;
        pkt.id = 0;
        pkt.payload_length = 5;  // stoped here

        // squawk
        // param is saved as native octal so we need convert back to
        // decimal because Sagetech will convert it back to octal
        uint16_t squawk = convert_base_to_decimal(8, last_operating_squawk);
        put_le16_ptr(&pkt.payload[0], squawk);

        // altitude
        if (_frontend.out_state.cfg.rf_capable & 0x01) {
            const float alt_meters = last_operating_alt * 0.01f;
            const int32_t alt_feet = (int32_t)(alt_meters * FEET_TO_METERS);
            const int16_t alt_feet_adj = (alt_feet + 50) / 100; // 1 = 100 feet, 1 = 149 feet, 5 = 500 feet
            put_le16_ptr(&pkt.payload[2], alt_feet_adj);

        } else {
            // use integrated altitude - recommend by sagetech
            pkt.payload[2] = 0x80;
            pkt.payload[3] = 0x00;
        }

        // RF mode
        pkt.payload[4] = last_operating_rf_select;
        send_msg(pkt);
    }

/*
 * Convert base 8 or 16 to decimal. Used to convert an octal/hexadecimal value stored on a GCS as a string field in different format, but then transmitted over mavlink as a float which is always a decimal.
 * baseIn: base of input number
 * inputNumber: value currently in base "baseIn" to be converted to base "baseOut"
 *
 * Example: convert ADSB squawk octal "1200" stored in memory as 0x0280 to 0x04B0
 *          uint16_t squawk_decimal = convertMathBase(8, squawk_octal);
 */
uint32_t AP_ADSB_Sagetech_MX::convert_base_to_decimal(const uint8_t baseIn, uint32_t inputNumber)
{
    // Our only sensible input bases are 16 and 8
    if (baseIn != 8 && baseIn != 16) {
        return inputNumber;
    }

    uint32_t outputNumber = 0;
    for (uint8_t i=0; inputNumber != 0; i++) {
        outputNumber += (inputNumber % 10) * powf(10, i);
        inputNumber /= 10;
    }
    return outputNumber;
}

#endif // HAL_ADSB_SAGETECH_ENABLED

