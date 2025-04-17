#pragma once
#include "DW1000.hpp"
#include "../../../shared/inc/twr_dw1000_frame_spec.hpp"
enum DeviceType {
    TAG,
    ANCHOR,
};

enum State {
    POLL,
    RESPONSE,
    FINAL,
    REPORT,
};


extern uint16_t pan;
extern uint16_t tag_id;
extern uint16_t anchor_id;

class DW1000Ranging
{
    protected:
        DeviceType deviceType;
        DW1000 dw1000;
        State currentState = POLL;
    public:
        DW1000Ranging(DeviceType deviceMode, DW1000& dw1000);
        virtual void loop();
        void twr_send(twr_message_t message);
};

class DW1000RangingTag : public  DW1000Ranging
{
    private:
        DW1000Time init_tx_ts, ack_rx_ts, fin_tx_ts;
        DW1000Time esp_init_rx_ts, esp_resp_tx_ts, esp_fin_rx_ts;
    
    public:
        DW1000RangingTag(DeviceType deviceMode, DW1000& dw1000);
        void loop();
};

class DW1000RangingAnchor : public DW1000Ranging
{
    private:
        DW1000Time esp_init_rx_ts, esp_resp_tx_ts, esp_fin_rx_ts;
    
    public:
        DW1000RangingAnchor(DeviceType deviceMode, DW1000& dw1000);
        void loop();
};