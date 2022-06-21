#include <AP_HAL/AP_HAL.h>
#include "AP_Beacon_Loco.h"
#include <ctype.h>
#include <stdio.h>

extern const AP_HAL::HAL& hal;

bool AP_Beacon_Loco::healthy()
{
    // healthy if we have parsed a message within the past 300ms
    return ((AP_HAL::millis() - last_update_ms) < AP_BEACON_TIMEOUT_MS);
}

void AP_Beacon_Loco::update(void)
{
    if (uart == nullptr) {
        return;
    }

    // read any available characters
    int16_t nbytes = uart->available();
    while (nbytes-- > 0) {
        char c = uart->read();

        switch (parse_state) {

            default:
            case ParseState_WaitingForHeader:
                if (c == AP_BEACON_LOCO_HEADER)
                {
                    parse_state = ParseState_WaitingForMsgId;
                    linebuf_len = 0;
                }
                break;

            case ParseState_WaitingForMsgId:
                parse_msg_id = c;
                switch (parse_msg_id)
                {
                case AP_BEACON_LOCO_MSGID_BEACON_CONFIG:
                case AP_BEACON_LOCO_MSGID_BEACON_DIST:
                case AP_BEACON_LOCO_MSGID_POSITION:
                    parse_state = ParseState_WaitingForLen;
                    break;
                default:
                    // invalid message id
                    parse_state = ParseState_WaitingForHeader;
                    break;
                }
                break;

            case ParseState_WaitingForLen:
                parse_msg_len = c;
                if (parse_msg_len > AP_BEACON_LOCO_MSG_LEN_MAX)
                {
                    // invalid message length
                    parse_state = ParseState_WaitingForHeader;
                }
                else
                {
                    parse_state = ParseState_WaitingForContents;
                }
                break;

            case ParseState_WaitingForContents:
                // add to buffer
                linebuf[linebuf_len++] = c;
                if ((linebuf_len == parse_msg_len) || (linebuf_len == sizeof(linebuf)))
                {
                    // process buffer
                    parse_buffer();
                    // reset state for next message
                    parse_state = ParseState_WaitingForHeader;
                }
                break;
        }
    }
}

void AP_Beacon_Loco::parse_buffer()
{
    // check crc
    uint8_t checksum = 0;
    checksum ^= parse_msg_id;
    checksum ^= parse_msg_len;
    for (uint8_t i=0; i<linebuf_len; i++) {
        checksum ^= linebuf[i];
    }
    // return if failed checksum check
    if (checksum != 0) {
        return;
    }

    bool parsed = false;

    switch (parse_msg_id)
    {
        case AP_BEACON_LOCO_MSGID_BEACON_CONFIG:
            {
                uint8_t beacon_id = linebuf[0];
                int32_t beacon_x = (uint32_t)linebuf[4] << 24 | (uint32_t)linebuf[3] << 16 | (uint32_t)linebuf[2] << 8 | (uint32_t)linebuf[1];
                int32_t beacon_y = (uint32_t)linebuf[8] << 24 | (uint32_t)linebuf[7] << 16 | (uint32_t)linebuf[6] << 8 | (uint32_t)linebuf[5];
                int32_t beacon_z = (uint32_t)linebuf[12] << 24 | (uint32_t)linebuf[11] << 16 | (uint32_t)linebuf[10] << 8 | (uint32_t)linebuf[9];
                Vector3f beacon_pos(beacon_x / 1000.0f, beacon_y / 1000.0f, -beacon_z / 1000.0f);

                if (beacon_pos.length() <= AP_BEACON_DISTANCE_MAX)
                {
                    set_beacon_position(beacon_id, beacon_pos);
                    parsed = true;
                }
            }
            break;

        case AP_BEACON_LOCO_MSGID_BEACON_DIST:
            {
                uint8_t beacon_id = linebuf[0];
                uint32_t beacon_distance = (uint32_t)linebuf[4] << 24 | (uint32_t)linebuf[3] << 16 | (uint32_t)linebuf[2] << 8 | (uint32_t)linebuf[1];
                float beacon_dist = beacon_distance/1000.0f;

                if (beacon_dist <= AP_BEACON_DISTANCE_MAX) {
                    set_beacon_distance(beacon_id, beacon_dist);
                    parsed = true;
                }
            }
            break;

        case AP_BEACON_LOCO_MSGID_POSITION:
            {
                int32_t vehicle_x = (uint32_t)linebuf[3] << 24 | (uint32_t)linebuf[2] << 16 | (uint32_t)linebuf[1] << 8 | (uint32_t)linebuf[0];
                int32_t vehicle_y = (uint32_t)linebuf[7] << 24 | (uint32_t)linebuf[6] << 16 | (uint32_t)linebuf[5] << 8 | (uint32_t)linebuf[4];
                int32_t vehicle_z = (uint32_t)linebuf[11] << 24 | (uint32_t)linebuf[10] << 16 | (uint32_t)linebuf[9] << 8 | (uint32_t)linebuf[8];
                float position_error = 0.1f;
                Vector3f veh_pos(Vector3f(vehicle_x / 1000.0f, vehicle_y / 1000.0f, -vehicle_z / 1000.0f));

                if (veh_pos.length() <= AP_BEACON_DISTANCE_MAX) {
                    set_vehicle_position(veh_pos, position_error);
                    parsed = true;
                }
            }
            break;

        default:
            // message id unvalid
            break;
    }


    if (parsed) {
        last_update_ms = AP_HAL::millis();
    }

}