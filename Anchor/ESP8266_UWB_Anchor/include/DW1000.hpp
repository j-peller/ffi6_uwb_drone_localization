#pragma once
#include <cstdint>
#include "Mode.hpp"
#include <Arduino.h>
#include <SPI.h>
#include "../extern/uwb-dw1000/hw/drivers/uwb/uwb_dw1000/include/dw1000/dw1000_regs.h"
#include <logger.hpp>
#include "dw1000_time.hpp"
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

    static const ClockSpeed automatic;
    static const ClockSpeed slow;
    static const ClockSpeed fast;
};

struct Channel {
    uint8_t rf_rxctrlh; /* see Table 37 for 0x28:0B– RF_RXCTRLH*/
    uint32_t rf_txctrl; //TODO Len is 3 in manual?
    uint8_t tc_pgdelay;
    uint32_t fs_pllcfg; //TODO Len is 5 in FS_PLLCFG_LEN?
};

struct PRF {
    uint32_t txprf;
    uint32_t rxfprf;
    uint16_t drx_tune1a;
};
struct Bitrate {
    uint32_t txbr;
    uint32_t rxm110k; //RXM110K
};

enum SFD {
    STD,
    DecaWave,
};

extern Bitrate bitrate_110k;
extern Bitrate bitrate_850k;
extern Bitrate bitrate_6M;

extern PRF prf64;
extern PRF prf16;
extern PRF prf4;

extern Channel channel1;
extern Channel channel2;
extern Channel channel3;
extern Channel channel4;
extern Channel channel5;
extern Channel channel7;

struct Mode {
    Channel channel;
    PRF prf;
    Bitrate bitrate;
    uint32_t preamble_code;
    uint32_t preamble_length;
    SFD sfd;
};



enum class FrameLength : uint8_t {
    STANDARD = 0x00, // 00 – Standard Frame mode. Use this setting is for IEEE 802.15.4 compliance.
    EXTENDED = 0x03, // 11 – Long Frames mode. Proprietary PHR encoding. Frame Length 0-1023.
};

enum class InterruptTable : uint32_t{
    INTERRUPT_ON_TX = SYS_STATUS_TXFRS,               // transmit frame sent event
    INTERRUPT_ON_RX = SYS_STATUS_RXDFR,             // receiver data frame ready event
    INTERRUPT_ON_RX_CRC_GOOD = SYS_STATUS_RXFCG,     // receiver FCS good event
    INTERRUPT_ON_RX_CRC_BAD = SYS_STATUS_RXFCE,      // receiver FCS error event
    INTERRUPT_ON_RX_FAIL_LDEERR = SYS_STATUS_LDEERR,  // leading edge detection processing error event
    INTERRUPT_ON_RX_FAIL_PHE = SYS_STATUS_RXPHE,     // receiver PHY header error even
    INTERRUPT_ON_RX_FAIL_RFSL = SYS_STATUS_RXRFSL,    // Reed Solomon Frame Sync Loss event
    INTERRUPT_ON_RX_TIMEOUT = SYS_STATUS_RXRFTO,      // Receive Frame Wait Timeout event
    INTERRUPT_ON_LDE_DONE = SYS_STATUS_LDEDONE,         // LDE processing done event
    INTERRUPT_ON_AUTOMATIC_ACK = SYS_STATUS_AAT,     // automatic acknowledge trigger event

    INTERRUPT_ALL = INTERRUPT_ON_TX | INTERRUPT_ON_RX | INTERRUPT_ON_RX_CRC_GOOD | INTERRUPT_ON_RX_CRC_BAD |
                    INTERRUPT_ON_RX_FAIL_LDEERR | INTERRUPT_ON_RX_FAIL_PHE | INTERRUPT_ON_RX_FAIL_RFSL | INTERRUPT_ON_RX_TIMEOUT |
                    INTERRUPT_ON_LDE_DONE | INTERRUPT_ON_AUTOMATIC_ACK,
    INTERRUPT_ALL_ALL = INTERRUPT_ALL | 0b111111111101111111111100001110,//0b111111111101111111111111111110,
    
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
inline InterruptTable& operator&(InterruptTable& lhs, InterruptTable rhs) {
    lhs = lhs & rhs;
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
        void setMode(Mode mode);
        void setDeviceID(uint16_t id);
        void setPANAdress(uint16_t address);
        void startReceiving();
        void setReceiverAutoReenable(boolean val);
        void readReceivedData(uint8_t** data, uint16_t* length);
        uint16_t getReceivedDataLength();
        void deleteReceivedDataLength();
        void addCustomInterruptHandler(InterruptTable interruptTable, std::function<void(uint32_t)> callback);
        void removeCustomInterruptHandler();


        /*  Responsible for loading Leading Edge Detection microcode from ROM to RAM as described in 7.2.46.3 LDELOAD
            Must be run before receiver mode is enabled!
        */
        void loadLDECode(); 
        void setFrameLength(FrameLength frame_length);
        void enableInterrupts(enum InterruptTable table);
        void soft_reset();
        void setClock(ClockSpeed clock);
        uint16_t getDeviceID();


        void addLogger(Logger* logger);
        void forceIdle();

        void get_tx_timestamp(DW1000Time& time);
        void get_rx_timestamp(DW1000Time& time);

        Logger* logger = nullptr;
        
    private:
        uint8_t irq = D2;
        uint8_t chip_select = D8;
        DeviceMode current_mode = IDLE;
        SPISettings spiSettings = SPISettings(20000000L, MSBFIRST, SPI_MODE0);

        std::function<void(uint32_t)> customInterruptCallback;
        InterruptTable customInterruptCallbackTable;
        void spi_transceive(uint8_t header[], uint8_t header_length, uint8_t data[], uint16_t data_length);
        void clearStatusRegister();
        
        
};