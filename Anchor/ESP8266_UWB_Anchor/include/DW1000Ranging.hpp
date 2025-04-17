#pragma once
#include "DW1000.hpp"
#include "../../../shared/inc/twr_dw1000_frame_spec.hpp"
enum DeviceType {
    TAG,
    ANCHOR,
};

enum CommState {
    POLL,
    RESPONSE,
    FINAL,
    REPORT,
};

enum SystemState {
    STATE_IDLE,
    STATE_MEASURING_ACTIVE,
    STATE_MEASURING_WAITING,
};


extern uint16_t pan;

class DW1000Ranging
{
    protected:
        DeviceType deviceType;
        DW1000& dw1000;
        CommState currentCommState = POLL;
        SystemState systemState = STATE_IDLE;
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
        uint16_t anchor_address;
        uint16_t deviceAddress;
    public:
        DW1000RangingTag(DeviceType deviceMode, DW1000& dw1000, uint16_t deviceAddress, uint16_t anchorAddress);
        void loop();
        void pollStateIRQHandler(uint32_t sys_status);
        void finalStateIRQHandler(uint32_t sys_status);
        void reportStateIRQHandler(uint32_t sys_status);
        void getDistanceToAnchor(uint16_t anchor_address);
        void resetTimestamps();
};

class DW1000RangingAnchor : public DW1000Ranging
{
    private:
        DW1000Time esp_init_rx_ts, esp_resp_tx_ts, esp_fin_rx_ts;
        uint16_t tag_address;
        uint16_t deviceAddress;
    
    public:
        DW1000RangingAnchor(DeviceType deviceMode, DW1000& dw1000, uint16_t deviceAddress, uint16_t tagAddress);
        void pollStateIRQHandler(uint32_t sys_status);
        void ackStateIRQHandler(uint32_t sys_status);
        void loop();
        void resetTimestamps();
};
