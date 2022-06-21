#pragma once

#include "AP_Beacon_Backend.h"

#define AP_BEACON_LOCO_MSG_LEN_MAX         20      
#define AP_BEACON_LOCO_HEADER              0x01    
#define AP_BEACON_LOCO_MSGID_BEACON_CONFIG 0x02    
#define AP_BEACON_LOCO_MSGID_BEACON_DIST   0x03    
#define AP_BEACON_LOCO_MSGID_POSITION      0x04    
#define AP_BEACON_DISTANCE_MAX             200.0f  

class AP_Beacon_Loco : public AP_Beacon_Backend
{

public:
    // constructor
    using AP_Beacon_Backend::AP_Beacon_Backend;

    // return true if sensor we are receiving data
    bool healthy() override;

    // updates vehicle position, beacon distance and configuration
    void update() override;

private:

    enum ParseState{
        ParseState_WaitingForHeader = 0,
        ParseState_WaitingForMsgId = 1,
        ParseState_WaitingForLen = 2,
        ParseState_WaitingForContents = 3
    } parse_state;

    // parse buffer
    void parse_buffer();

    uint8_t parse_msg_id;
    uint8_t parse_msg_len;

    uint8_t linebuf[AP_BEACON_LOCO_MSG_LEN_MAX];
    uint8_t linebuf_len = 0;
    uint32_t last_update_ms = 0;
};