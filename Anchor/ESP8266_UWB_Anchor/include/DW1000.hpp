#pragma once
#include <cstdint>
#include "Mode.hpp"
#include <Arduino.h>
#include <SPI.h>

#include "logger.hpp"
#include "../extern/uwb-dw1000/hw/drivers/uwb/uwb_dw1000/include/dw1000/dw1000_regs.h"
#include "dw1000_modes.h"
/**
 * @brief SPI command structure for the DWM1000
 */
typedef struct {
    uint32_t reg:6;          //!< Indicates the register to be read or write into
    uint32_t subindex:1;     //!< Indicates offset address of the register
    uint32_t operation:1;    //!< Read or Write operation
    uint32_t extended:1;     //!< If subaddress is higher than 128
    uint32_t subaddress:15;  //!< Indicates subaddress of register
} dw1000_spi_cmd_t;

struct ClockSpeed {
    uint8_t pmsc0_clock;
    SPISettings spiSettings;

    static ClockSpeed automatic;
    static ClockSpeed slow;
    static ClockSpeed fast;
};



enum class FrameLength : uint8_t {
    STANDARD = 0x00, // 00 – Standard Frame mode. Use this setting is for IEEE 802.15.4 compliance.
    EXTENDED = 0x03, // 11 – Long Frames mode. Proprietary PHR encoding. Frame Length 0-1023.
};

enum class InterruptTable : uint32_t{
    INTERRUPT_ON_TX = SYS_STATUS_TXFRS,               // transmit frame sent event
    INTERRUPT_ON_RX = 1 << 12,              // receiver data frame ready event
    INTERRUPT_ON_RX_CRC_GOOD = 1 << 13,     // receiver FCS good event
    INTERRUPT_ON_RX_CRC_BAD = 1 << 14,      // receiver FCS error event
    INTERRUPT_ON_RX_FAIL_LDEERR = 1 << 17,  // leading edge detection processing error event
    INTERRUPT_ON_RX_FAIL_PHE = 1 << 11,     // receiver PHY header error even
    INTERRUPT_ON_RX_FAIL_RFSL = 1 << 15,    // Reed Solomon Frame Sync Loss event
    INTERRUPT_ON_RX_TIMEOUT = 1 << 16,      // Receive Frame Wait Timeout event
    INTERRUPT_ON_LDE_DONE = 1 << 9,         // LDE processing done event
    INTERRUPT_ON_AUTOMATIC_ACK = 1 << 2,     // automatic acknowledge trigger event

    INTERRUPT_ALL = INTERRUPT_ON_TX | INTERRUPT_ON_RX | INTERRUPT_ON_RX_CRC_GOOD | INTERRUPT_ON_RX_CRC_BAD |
                    INTERRUPT_ON_RX_FAIL_LDEERR | INTERRUPT_ON_RX_FAIL_PHE | INTERRUPT_ON_RX_FAIL_RFSL | INTERRUPT_ON_RX_TIMEOUT |
                    INTERRUPT_ON_LDE_DONE | INTERRUPT_ON_AUTOMATIC_ACK
    
};
inline InterruptTable operator|(InterruptTable lhs, InterruptTable rhs) {
    return static_cast<InterruptTable>(
        static_cast<uint32_t>(lhs) | static_cast<uint32_t>(rhs)
    );
}
inline InterruptTable& operator|=(InterruptTable& lhs, InterruptTable rhs) {
    lhs = lhs | rhs;
    return lhs;
};


    /* SPI Transaction Header operation modes  */
const uint8_t    READ        = 0x00;
const uint8_t    READ_SUB    = 0x40;
const uint8_t    WRITE       = 0x80;
const uint8_t    WRITE_SUB   = 0x80;

/* If we just wanna read a normal register... */
static const uint8_t    NO_SUB_ADDRESS    = 0x00;

class DW1000 {
    public:
        void initialize();
        void getPrintableDeviceIdentifier(char msgBuffer[]);
        void handleInterrupt();
        void idle();
        void readBytes(uint8_t reg, uint16_t offset, uint8_t* data, uint32_t length);
        void readBytes(uint8_t reg, uint16_t offset, uint32_t* data);
        void writeBytes(uint8_t reg, uint16_t offset, uint8_t* data, uint32_t length);
        void writeBytes(uint8_t reg, uint16_t offset, uint32_t data);
        void readNetworkIdAndDeviceAddress(uint8_t* data);
        void writeNetworkIdAndDeviceAddress(uint8_t* data);
        void setDataRate(uint8_t rate);
        void transmit(uint8_t data[], uint16_t length);
        void setMode(dw1000_mode_t mode);
        void setDeviceAddress(uint16_t id);
        void setPANAdress(uint16_t address);
        void startReceiving();
        void setReceiverAutoReenable(boolean val);
        void readReceivedData(uint8_t** data, uint16_t* length);
        uint16_t getReceivedDataLength();
        void deleteReceivedDataLength();


        /*  Responsible for loading Leading Edge Detection microcode from ROM to RAM as described in 7.2.46.3 LDELOAD
            Must be run before receiver mode is enabled!
        */
        void loadLDECode(); 
        void setFrameLength(FrameLength frame_length);
        void enableInterrupts(enum InterruptTable table);
        void soft_reset();
        void setClock(ClockSpeed clock);


        void addLogger(Logger* logger);
        void readBytesOTP(uint16_t addr, uint8_t* data, uint32_t len);
        void forceIdle();
        Logger* logger = nullptr;
        SPISettings* spiSettings;
    private:
        uint8_t irq = 2;
        uint8_t chip_select = 5;
        DeviceMode current_mode = IDLE;
        
        void spi_transceive(uint8_t header[], uint8_t header_length, uint8_t data[], uint16_t data_length);
        void clearStatusRegister();
        void _readBytesOTP(uint16_t addr, uint8_t* data);
        
        
};