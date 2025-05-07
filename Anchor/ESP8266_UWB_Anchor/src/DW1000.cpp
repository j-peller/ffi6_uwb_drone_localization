#include "DW1000.hpp"
#include <Arduino.h>
#include <cassert>
#include "FunctionalInterrupt.h"

const ClockSpeed ClockSpeed::automatic = {
    .pmsc0_clock = PMSC_CTRL0_SYSCLKS_AUTO,
    .spiSettings = SPISettings(20000000L, MSBFIRST, SPI_MODE0),
};
const ClockSpeed ClockSpeed::slow = {
    .pmsc0_clock = PMSC_CTRL0_SYSCLKS_19M,
    .spiSettings = SPISettings(2000000L, MSBFIRST, SPI_MODE0),
};
const ClockSpeed ClockSpeed::fast = {
    .pmsc0_clock = PMSC_CTRL0_SYSCLKS_125M,
    .spiSettings = SPISettings(20000000L, MSBFIRST, SPI_MODE0),
};
Bitrate bitrate_110k {
    .txbr = TX_FCTRL_TXBR_110k,
    .rxm110k = SYS_CFG_RXM110K,
};
Bitrate bitrate_850k {
    .txbr = TX_FCTRL_TXBR_850k,
    .rxm110k = 0,
};
Bitrate bitrate_6M {
    .txbr = TX_FCTRL_TXBR_6M,
    .rxm110k = 0,
};

PRF prf64 {
    .txprf = TX_FCTRL_TXPRF_64M,
    .rxfprf = CHAN_CTRL_RXFPRF_64,
    .drx_tune1a = DRX_TUNE1a_PRF64,
};

PRF prf16 {
    .txprf = TX_FCTRL_TXPRF_16M,
    .rxfprf = CHAN_CTRL_RXFPRF_16,
    .drx_tune1a = DRX_TUNE1a_PRF16,
};

PRF prf4 {
    .txprf = TX_FCTRL_TXPRF_4M,
    .rxfprf = CHAN_CTRL_RXFPRF_4,
    .drx_tune1a = 0,
};

Channel channel1 = {
    .rf_rxctrlh = RF_RXCTRLH_NBW,
    .rf_txctrl = RF_TXCTRL_CH1,
    .tc_pgdelay = TC_PGDELAY_CH1,
    .fs_pllcfg = FS_PLLCFG_CH1,
};

Channel channel2 = {
    .rf_rxctrlh = RF_RXCTRLH_NBW,
    .rf_txctrl = RF_TXCTRL_CH2,
    .tc_pgdelay = TC_PGDELAY_CH2,
    .fs_pllcfg = FS_PLLCFG_CH2,
};

Channel channel3 = {
    .rf_rxctrlh = RF_RXCTRLH_NBW,
    .rf_txctrl = RF_TXCTRL_CH3,
    .tc_pgdelay = TC_PGDELAY_CH3,
    .fs_pllcfg = FS_PLLCFG_CH3,
};

Channel channel4 = {
    .rf_rxctrlh = RF_RXCTRLH_WBW,
    .rf_txctrl = RF_TXCTRL_CH4,
    .tc_pgdelay = TC_PGDELAY_CH4,
    .fs_pllcfg = FS_PLLCFG_CH4,
};

Channel channel5 = {
    .rf_rxctrlh = RF_RXCTRLH_NBW,
    .rf_txctrl = RF_TXCTRL_CH5,
    .tc_pgdelay = TC_PGDELAY_CH5,
    .fs_pllcfg = FS_PLLCFG_CH5,
};

Channel channel7 = {
    .rf_rxctrlh = RF_RXCTRLH_WBW,
    .rf_txctrl = RF_TXCTRL_CH7,
    .tc_pgdelay = TC_PGDELAY_CH7,
    .fs_pllcfg = FS_PLLCFG_CH7,
};

Tune tuneTest = {
    .drx_tune2 = DRX_TUNE2_PRF64_PAC64,
};

static void IRAM_ATTR dw1000_interrupt_handler(void* arg) {
    DW1000* instance = static_cast<DW1000*>(arg);
    instance->handleInterrupt();
}

void DW1000::setReceiverAutoReenable(boolean val)
{
    uint32_t data = 0;
    readBytes(SYS_CFG_ID, NO_SUB_ADDRESS, &data);
    data &= ~(SYS_CFG_RXAUTR);
    if(val) data |= SYS_CFG_RXAUTR;
    writeBytes(SYS_CFG_ID, NO_SUB_ADDRESS, data);
}

void DW1000::setMode(Mode mode)
{

    /* Clock PLL lock detect tune. This bit should be set to 1 to ensure reliable operation of the clock PLL lock detect flags.*/
    uint32_t ec_ctrl = 0;
    readBytes(EXT_SYNC_ID, EC_CTRL_OFFSET, &ec_ctrl);
    ec_ctrl |= EC_CTRL_PLLLCK;
    writeBytes(EXT_SYNC_ID, EC_CTRL_OFFSET, ec_ctrl);


    uint32_t sys_cfg = 0;
    readBytes(SYS_CFG_ID, NO_SUB_ADDRESS, (uint8_t*)&sys_cfg, SYS_CFG_LEN);

    //sys_cfg |= SYS_CFG_FFE;         //< Enable Frame Filtering. This requires SHORT_ADDR to be set beforehand.
    sys_cfg |= SYS_CFG_FFAD;        //< Allow Data Frame
    sys_cfg |= SYS_CFG_PHR_MODE_00; //< Standard Frame mode IEEE 802.15.4 compliant
    sys_cfg |= mode.bitrate.rxm110k;

    //sys_cfg |= SYS_CFG_RXWTOE; // TODO testing purposes;

    writeBytes(SYS_CFG_ID, NO_SUB_ADDRESS, (uint8_t*)&sys_cfg, SYS_CFG_LEN);
    

     /**
     * Required Configuration for Transmitter on Channel 5
     */

    uint32_t rf_txctrl_val = mode.channel.rf_txctrl;     //< See DW1000 User Manual Page 148 Table 38
    writeBytes(RF_CONF_ID, RF_TXCTRL_OFFSET, rf_txctrl_val);

    uint8_t tc_pgdelay_val = mode.channel.tc_pgdelay;    //< See DW1000 User Manual Page 155 Table 40 
    writeBytes(TX_CAL_ID, TC_PGDELAY_OFFSET, &tc_pgdelay_val, sizeof(uint8_t));

    uint32_t fs_pllcfg_val = mode.channel.fs_pllcfg;     //< See DW1000 User Manual Page 157 Table 43
    writeBytes(FS_CTRL_ID, FS_PLLCFG_OFFSET, fs_pllcfg_val);

    uint8_t tx_fctrl[TX_FCTRL_LEN] = {0};
    readBytes(TX_FCTRL_ID, NO_SUB_ADDRESS, tx_fctrl, TX_FCTRL_LEN);

    *(uint64_t *) tx_fctrl &= ~TX_FCTRL_TXBR_MASK; //< Clear current setting
    *(uint64_t *) tx_fctrl |= mode.bitrate.txbr;

    *(uint64_t *) tx_fctrl &= ~TX_FCTRL_TXPRF_MASK;
    *(uint64_t *) tx_fctrl |= mode.prf.txprf;

    *(uint64_t *) tx_fctrl &= ~TX_FCTRL_TXPSR_MASK;
    *(uint64_t *) tx_fctrl &= ~TX_FCTRL_TXPSR_PE_MASK;

    *(uint64_t *) tx_fctrl |= mode.preamble_length;

    writeBytes(TX_FCTRL_ID, NO_SUB_ADDRESS, tx_fctrl, TX_FCTRL_LEN);

     /**
     * Required Configuration for Receiver on Channel 5
     */
    uint8_t rf_rxctrlh_val = mode.channel.rf_rxctrlh; //< See DW1000 User Manual Page 148 Table 37
    writeBytes(RF_CONF_ID, RF_RXCTRLH_OFFSET, &rf_rxctrlh_val, sizeof(uint8_t));


    /* Channel Control Settings for both Receiver and Transmitter */
    uint8_t chan_ctrl[CHAN_CTRL_LEN] = {0};
    readBytes(CHAN_CTRL_ID, NO_SUB_ADDRESS, chan_ctrl, CHAN_CTRL_LEN);

    *(uint32_t *) chan_ctrl &= ~CHAN_CTRL_RXFPRF_MASK;            //< Clear current RX PRF
    *(uint32_t *) chan_ctrl |= mode.prf.rxfprf;               //< Set RX PRF to 64 MHz to match Transmitter

    *(uint32_t *) chan_ctrl &= ~CHAN_CTRL_TX_PCOD_MASK;           //< Clear current Preamble Code for Transceiver
    *(uint32_t *) chan_ctrl |= (mode.preamble_code) << CHAN_CTRL_TX_PCOD_SHIFT;  //< Set Preamble Code 9. Supported according to page 214 table 61

    *(uint32_t *) chan_ctrl &= ~CHAN_CTRL_RX_PCOD_MASK;           //< Clear current Preamble Code for Receiver
    *(uint32_t *) chan_ctrl |= (mode.preamble_code) << CHAN_CTRL_RX_PCOD_SHIFT;  //< Set Preamble Code 9. Supported according to page 214 table 61
    writeBytes(CHAN_CTRL_ID, NO_SUB_ADDRESS, chan_ctrl, CHAN_CTRL_LEN);

    switch(mode.sfd){
        case SFD::STD:
        {

            break;
        }
        case SFD::DecaWave:
        {
            break;
        }
    }
       
    writeBytes(DRX_CONF_ID, DRX_TUNE1a_OFFSET, (uint8_t*)&mode.prf.drx_tune1a, DRX_TUNE1a_LEN);  //< See DW1000 User Manual Page 141 Table 31 
    writeBytes(DRX_CONF_ID, DRX_TUNE2_OFFSET, (uint8_t*)&mode.tune.drx_tune2, DRX_TUNE2_LEN);  //< See DW1000 User Manual Page 141 Table 31 

}
void DW1000::initialize()
{
    delay(5);
    pinMode(this->irq, INPUT);
    SPI.begin();
    //attachInterrupt(digitalPinToInterrupt(irq), DW1000::handleInterrupt, RISING);
    //attachInterrupt(digitalPinToInterrupt(irq), std::bind(&DW1000::handleInterrupt, this), RISING); // TODO was rising
    attachInterruptArg(digitalPinToInterrupt(irq), dw1000_interrupt_handler, this, RISING);
    pinMode(chip_select, OUTPUT);
    digitalWrite(chip_select, HIGH);
    delay(1000);

}

void DW1000::addLogger(Logger* logger)
{
    this->logger = logger;
    this->logger->setDeviceID(getDeviceID());
}

void DW1000::readBytes(uint8_t reg, uint16_t offset, uint8_t* data, uint32_t length)
{
    SPI.begin();
    /* Build SPI Transaction Header according to 2.2.1.2 p4 DW1000 User Manual */
    dw1000_spi_cmd_t cmd = {
        .reg = reg,
        .subindex = offset != 0,
        .operation = READ,
        .extended = offset > 0x7F,
        .subaddress = offset
    };

    uint8_t header[] = {
        [0] = cmd.operation << 7 | cmd.subindex << 6 | cmd.reg,
        [1] = cmd.extended << 7 | (uint8_t)(offset),
        [2] = (uint8_t)(offset >> 7)
    };

    uint8_t header_len = cmd.subindex ? (cmd.extended ? 3 : 2) : 1;
    //spi_transceive(header, header_len, data, length);
    //noInterrupts();
    SPI.beginTransaction(spiSettings);
    digitalWrite(chip_select, LOW);
    for(uint16_t i = 0; i < header_len; i++) {
	    SPI.transfer(header[i]); // send header
	}
    for(uint16_t i = 0; i < length; i++) {
		data[i] = SPI.transfer(0x00); // read values, write junk
	}

    delayMicroseconds(5);
	digitalWrite(chip_select, HIGH);
	SPI.endTransaction();
    SPI.end();

    delayMicroseconds(10);
    //interrupts();
    
    /*
    SPI.beginTransaction(spiSettings);
    digitalWrite(chip_select, LOW);
    
    for(uint16_t i = 0; i < header_len; i++) {
	    SPI.transfer(header[i]); // send header
	}
    for(uint16_t i = 0; i < length; i++) {
		data[i] = SPI.transfer(0x00); // read values
	}

    delayMicroseconds(5);
	digitalWrite(chip_select, HIGH);
	SPI.endTransaction();
    */

}
void DW1000::readBytes(uint8_t reg, uint16_t offset, uint32_t* data)
{
    uint8_t cache[4] = {0};
    readBytes(reg, offset, cache, 4);
    *data = (cache[3]) << 24 |  (cache[2]) << 16 | (cache[1]) << 8 | (cache[0]);
}

void DW1000::spi_transceive(uint8_t header[], uint8_t header_length, uint8_t data[], uint16_t data_length)
{
    if(logger != nullptr)
    {
        logger->output("SPI TX Communcation: %X - %X --- %X - %X", data[0], data[1], data[2], data[3]);
    }
    SPI.beginTransaction(spiSettings);
    digitalWrite(chip_select, LOW);
    for(uint16_t i = 0; i < header_length; i++) {
	    SPI.transfer(header[i]); // send header
	}
    for(uint16_t i = 0; i < data_length; i++) {
		data[i] = SPI.transfer(data[i]); // read/write values
	}

    delayMicroseconds(5);
	digitalWrite(chip_select, HIGH);
	SPI.endTransaction();
    if(logger != nullptr)
    {
        logger->output("SPI RX Communcation: %X - %X --- %X - %X", data[0], data[1], data[2], data[3]);
    }
}
void DW1000::soft_reset() 
 {
    /* Set SYSCLKS to 01 */
    uint8_t reg = 0;
    setClock(ClockSpeed::slow);

    /* Reset HIF, TX, RX and PMSC */
    reg = PMSC_CTRL0_RESET_ALL;
    writeBytes(PMSC_ID, PMSC_CTRL0_SOFTRESET_OFFSET, &reg, sizeof(uint8_t));

    /* DW1000 needs a 10us sleep to let clk PLL lock after reset */
    delayMicroseconds(10);

    /* Finish reset */
    reg = PMSC_CTRL0_RESET_CLEAR; 
    writeBytes(PMSC_ID, PMSC_CTRL0_SOFTRESET_OFFSET, &reg, sizeof(uint8_t));


    setClock(ClockSpeed::fast);
 }
void DW1000::loadLDECode()
{
    /* see 2.5.5.10 LDELOAD */

    setClock(ClockSpeed::slow);
    delayMicroseconds(1000);

    uint8_t otpctrl[OTP_CTRL_LEN] = {0};
    uint8_t pmsc_ctrl0[PMSC_CTRL0_LEN] = {0};

    readBytes(OTP_IF_ID, OTP_CTRL, otpctrl, OTP_CTRL_LEN);
    logger->output("otp %x %x", otpctrl[1], otpctrl[0]);

    readBytes(PMSC_ID, PMSC_CTRL0_OFFSET, pmsc_ctrl0, PMSC_CTRL0_LEN);
    logger->output("pmsc %x %x %x %x", pmsc_ctrl0[3], pmsc_ctrl0[2], pmsc_ctrl0[1], pmsc_ctrl0[0]);

    otpctrl[1] |= 1 << 7; /* LDELOAD Bit */
    logger->output("otp %x %x", otpctrl[1], otpctrl[0]);
    pmsc_ctrl0[0] = 0x01;
    pmsc_ctrl0[1] = 0x03;
    logger->output("pmsc %x %x %x %x", pmsc_ctrl0[3], pmsc_ctrl0[2], pmsc_ctrl0[1], pmsc_ctrl0[0]);
    writeBytes(PMSC_ID, PMSC_CTRL0_OFFSET, pmsc_ctrl0, PMSC_CTRL0_LEN);
    
    writeBytes(OTP_IF_ID, OTP_CTRL, otpctrl, OTP_CTRL_LEN);

    delayMicroseconds(150);

    pmsc_ctrl0[0] = 0x00;
	pmsc_ctrl0[1] = 0x02;
    writeBytes(PMSC_ID, PMSC_CTRL0_OFFSET, pmsc_ctrl0, PMSC_CTRL0_LEN);
    logger->output("pmsc %x %x %x %x", pmsc_ctrl0[3], pmsc_ctrl0[2], pmsc_ctrl0[1], pmsc_ctrl0[0]);

    readBytes(PMSC_ID, PMSC_CTRL0_OFFSET, pmsc_ctrl0, PMSC_CTRL0_LEN);
    logger->output("pmsc %x %x %x %x", pmsc_ctrl0[3], pmsc_ctrl0[2], pmsc_ctrl0[1], pmsc_ctrl0[0]);

    //delayMicroseconds(1000);
    setClock(ClockSpeed::automatic);
    delayMicroseconds(1000);

    readBytes(OTP_IF_ID, OTP_CTRL, otpctrl, OTP_CTRL_LEN);
    logger->output("otp %x %x", otpctrl[1], otpctrl[0]);
}
void DW1000::transmit(uint8_t data[], uint16_t length)
{
    assert(length < (1 << 10) && "length exceeds 10-bit maximum (1023)");
    // TODO enable frame check - add 2 bytes length etc...
    forceIdle();
    writeBytes(TX_BUFFER_ID, NO_SUB_ADDRESS, data, length);
    
    uint32_t frame_control = 0;
    readBytes(TX_FCTRL_ID, NO_SUB_ADDRESS, &frame_control);

    frame_control &= ~((1 << 10) - 1); /* Reset LEN Bits */

    frame_control |= length + 2; /* TFLEN: set first 10 bits to length*/
    logger->output("TX_FCTRL_ID %x", frame_control);
    writeBytes(TX_FCTRL_ID, NO_SUB_ADDRESS, frame_control);

    uint32_t system_ctrl = 0;
    readBytes(SYS_CTRL_ID, NO_SUB_ADDRESS, &system_ctrl);
    system_ctrl |= SYS_CTRL_TXSTRT; /* Start Transmission Now Bit */

    logger->output("SYS_CTRL %x", system_ctrl);
    writeBytes(SYS_CTRL_ID, NO_SUB_ADDRESS, system_ctrl);
}

void DW1000::setFrameLength(FrameLength frame_length)
{
    uint8_t sys_cfg[SYS_CFG_LEN] = {0};
    readBytes(SYS_CFG_ID, NO_SUB_ADDRESS, sys_cfg, SYS_CFG_LEN);
    sys_cfg[2] &= ~(0x3); // reset PHR value
    sys_cfg[2] |= static_cast<uint8_t>(frame_length); // update PHR value
    writeBytes(SYS_CFG_ID, 0, sys_cfg, SYS_CFG_LEN);
}

void DW1000::enableInterrupts(enum InterruptTable table)
{
    writeBytes(SYS_MASK_ID, NO_SUB_ADDRESS, static_cast<uint32_t>(table));
    writeBytes(SYS_MASK_ID, NO_SUB_ADDRESS, static_cast<uint32_t>(table)); //todo crazy, but when writing twice, it works, otherwise not (fuck you dwm1000)
    
    logger->addBuffer("sys_mask should be now! %x", static_cast<uint32_t>(table));


    uint32_t sys_mask = 0;
    readBytes(SYS_MASK_ID, NO_SUB_ADDRESS, &sys_mask);
    if(logger!=nullptr) logger->addBuffer("sys_mask is now! %x", sys_mask);
}

void DW1000::writeBytes(uint8_t reg, uint16_t offset, uint8_t* data, uint32_t length)
{
    /* Build SPI Transaction Header according to 2.2.1.2 p4 DW1000 User Manual */
    SPI.begin();
    dw1000_spi_cmd_t cmd = {
        .reg = reg,
        .subindex = offset != 0,
        .operation = (offset != 0 ? WRITE_SUB >> 7 : WRITE >> 7),
        .extended = offset > 0x7F,
        .subaddress = offset
    };

    uint8_t header[] = {
        [0] = cmd.operation << 7 | cmd.subindex << 6 | cmd.reg,
        [1] = cmd.extended << 7 | (uint8_t)(offset),
        [2] = (uint8_t)(offset >> 7)
    };

    /* do we have a 1,2 or 3 octet header? */
    uint8_t header_len = cmd.subindex ? (cmd.extended ? 3 : 2) : 1;
    //spi_transceive(header, header_len, data, length);
    //noInterrupts();
    SPI.beginTransaction(spiSettings);
    digitalWrite(chip_select, LOW);
    for(uint16_t i = 0; i < header_len; i++) {
	    SPI.transfer(header[i]); // send header
	}
    for(uint16_t i = 0; i < length; i++) {
		SPI.transfer(data[i]); // write values
	}

    delayMicroseconds(5);
	digitalWrite(chip_select, HIGH);
	SPI.endTransaction();
    SPI.end();
    //interrupts();
    delayMicroseconds(10);

}
/**
 * @brief Write data to a specific register using an offset and data of type uint32_t
 * Converts uint32_t data to the correct uint8_t[] format (little endian)
 */
void DW1000::writeBytes(uint8_t reg, uint16_t offset, uint32_t data)
{
    //data = __builtin_bswap32(data); // swap to little endian decoding before writing as uint8_t array
    writeBytes(reg, offset, (uint8_t*)&data, sizeof(uint32_t));
}

void DW1000::getPrintableDeviceIdentifier(char message[])
{
    uint8_t data[DEV_ID_LEN];
	readBytes(DEV_ID_ID, NO_SUB_ADDRESS, data, DEV_ID_LEN);
    uint32_t device_id = (data[3] << 24) | (data[2] << 16) | (data[1] << 8) | data[0];
    sprintf(message, "0x%08X - %02X - model: %d, version: %d, revision: %d", device_id,
        (uint16_t)((data[3] << 8) | data[2]), data[1], (data[0] >> 4) & 0x0F, data[0] & 0x0F);
}

void DW1000::readNetworkIdAndDeviceAddress(uint8_t* data)
{
	readBytes(PANADR_ID, NO_SUB_ADDRESS, data, PANADR_LEN);
}
void DW1000::writeNetworkIdAndDeviceAddress(uint8_t* data)
{
    writeBytes(PANADR_ID, NO_SUB_ADDRESS, data, PANADR_LEN);
}
void DW1000::setDeviceID(uint16_t id)
{
    writeBytes(PANADR_ID, NO_SUB_ADDRESS, (uint8_t*) &id, 2);
}

uint16_t DW1000::getDeviceID()
{
    uint16_t deviceID;
    readBytes(PANADR_ID, NO_SUB_ADDRESS, (uint8_t*) &deviceID, 2);
    return deviceID;
}

void DW1000::setPANAdress(uint16_t address)
{
    writeBytes(PANADR_ID, PANADR_PAN_ID_OFFSET, (uint8_t*) &(address), 2);
}

void DW1000::startReceiving()
{
    forceIdle();
    uint32_t sys_ctrl = 0;
    readBytes(SYS_CTRL_ID, NO_SUB_ADDRESS, &sys_ctrl);
    sys_ctrl |= SYS_CTRL_RXENAB;
    writeBytes(SYS_CTRL_ID, NO_SUB_ADDRESS, sys_ctrl);
}


void DW1000::handleInterrupt()
{
    uint32_t sys_status = 0;
    readBytes(SYS_STATUS_ID, NO_SUB_ADDRESS, &sys_status);
    if(logger!=nullptr) logger->addBuffer("Gotcha! %x", sys_status);
    logger->addBuffer("customInterruptCallbackTable %x", customInterruptCallbackTable);
    logger->addBuffer("customInterruptCallback %x", customInterruptCallback);
    if( (uint32_t) customInterruptCallbackTable & sys_status)
    {
        if(customInterruptCallback)
        {
            customInterruptCallback(sys_status);
        } 
    }

    clearStatusRegister(); //!Dont clear everything!

    if(sys_status & SYS_STATUS_RXDFR)
    {
        if(logger!=nullptr) logger->addBuffer("Knock knock!! %x", sys_status);
    }
}

void DW1000::idle() {

}

void DW1000::setDataRate(uint8_t rate)
{
    //TODO
}
/* clear all SYS_Status Register by writing 1 in every bit*/
void DW1000::clearStatusRegister()
{
    uint8_t data[SYS_STATUS_LEN] = {0};
    memset(data, 0b1100110111011001111111111111111110, SYS_STATUS_LEN);
    writeBytes(SYS_STATUS_ID, NO_SUB_ADDRESS, data, SYS_STATUS_LEN);

    uint8_t test;
    readBytes(SYS_STATUS_ID, 0x4, &test, 1);
    logger->addBuffer("test is %x", test);
    readBytes(SYS_STATUS_ID, 0x0, &test, 1);
    logger->addBuffer("test is %x", test);
}

void DW1000::setClock(ClockSpeed clock){
    uint8_t pmsc_ctrl0[PMSC_CTRL0_LEN] = {0};
    readBytes(PMSC_ID, NO_SUB_ADDRESS, pmsc_ctrl0, PMSC_CTRL0_LEN);

    logger->output("clock speed pmsc %x %x %x %x", pmsc_ctrl0[3], pmsc_ctrl0[2], pmsc_ctrl0[1], pmsc_ctrl0[0]);

    pmsc_ctrl0[0] &= ~(0x3); // reset current clock settings
    pmsc_ctrl0[0] |= clock.pmsc0_clock;
    spiSettings = clock.spiSettings;
    logger->output("clock speed pmsc %x %x %x %x", pmsc_ctrl0[3], pmsc_ctrl0[2], pmsc_ctrl0[1], pmsc_ctrl0[0]);

    //writeBytes(PMSC_ID, NO_SUB_ADDRESS, pmsc_ctrl0, PMSC_CTRL0_LEN);
    delay(5);
    
}
void DW1000::setDiagnostic(bool enable)
{
    uint32_t value = 0;
    value |= enable;
    writeBytes(DIG_DIAG_ID, EVC_CTRL_OFFSET, value); //write all 4 bytes as stated in 7.2.48.1 Sub-Register 0x2F:00 – Event Counter Control
}
void DW1000::log(){
    uint32_t reg;

    reg = 0;
    readBytes(DIG_DIAG_ID, EVC_PHE_OFFSET, (uint8_t*)&reg, EVC_PHE_LEN);
    if(logger != nullptr) logger->addBuffer("DIG_DIAG_ID | EVC_PHE_OFFSET %x", reg);

    reg = 0;
    readBytes(DIG_DIAG_ID, EVC_RSE_OFFSET, (uint8_t*)&reg, EVC_RSE_LEN);
    if(logger != nullptr) logger->addBuffer("DIG_DIAG_ID | EVC_RSE_OFFSET %x", reg);

    reg = 0;
    readBytes(DIG_DIAG_ID, EVC_FCG_OFFSET, (uint8_t*)&reg, EVC_FCG_LEN);
    if(logger != nullptr) logger->addBuffer("DIG_DIAG_ID | EVC_FCG_OFFSET %x", reg);

    reg = 0;
    readBytes(DIG_DIAG_ID, EVC_FCE_OFFSET, (uint8_t*)&reg, EVC_FCE_LEN);
    if(logger != nullptr) logger->addBuffer("DIG_DIAG_ID | EVC_FCE_OFFSET %x", reg);

    reg = 0;
    readBytes(DIG_DIAG_ID, EVC_FFR_OFFSET, (uint8_t*)&reg, EVC_FFR_LEN);
    if(logger != nullptr) logger->addBuffer("DIG_DIAG_ID | EVC_FFR_OFFSET %x", reg);

    reg = 0;
    readBytes(DIG_DIAG_ID, EVC_OVR_OFFSET, (uint8_t*)&reg, EVC_OVR_LEN);
    if(logger != nullptr) logger->addBuffer("DIG_DIAG_ID | EVC_OVR_OFFSET %x", reg);

    reg = 0;
    readBytes(DIG_DIAG_ID, EVC_STO_OFFSET, (uint8_t*)&reg, EVC_STO_LEN);
    if(logger != nullptr) logger->addBuffer("DIG_DIAG_ID | EVC_STO_OFFSET %x", reg);

    reg = 0;
    readBytes(DIG_DIAG_ID, EVC_PTO_OFFSET, (uint8_t*)&reg, EVC_PTO_LEN);
    if(logger != nullptr) logger->addBuffer("DIG_DIAG_ID | EVC_PTO_OFFSET %x", reg);

    reg = 0;
    readBytes(DIG_DIAG_ID, EVC_FWTO_OFFSET, (uint8_t*)&reg, EVC_FWTO_LEN);
    if(logger != nullptr) logger->addBuffer("DIG_DIAG_ID | EVC_FWTO_OFFSET %x", reg);

    reg = 0;
    readBytes(DIG_DIAG_ID, EVC_TXFS_OFFSET, (uint8_t*)&reg, EVC_TXFS_LEN);
    if(logger != nullptr) logger->addBuffer("DIG_DIAG_ID | EVC_TXFS_OFFSET %x", reg);

    reg = 0;
    readBytes(DIG_DIAG_ID, EVC_HPW_OFFSET, (uint8_t*)&reg, EVC_HPW_LEN);
    if(logger != nullptr) logger->addBuffer("DIG_DIAG_ID | EVC_HPW_OFFSET %x", reg);

    reg = 0;
    readBytes(DIG_DIAG_ID, EVC_TPW_OFFSET, (uint8_t*)&reg, EVC_TPW_LEN);
    if(logger != nullptr) logger->addBuffer("DIG_DIAG_ID | EVC_TPW_OFFSET %x", reg);

}
void DW1000::forceIdle() {
    uint32_t sys_ctrl = SYS_CTRL_TRXOFF;

    //this->_dev_mode = IDLE_MODE;

    writeBytes(SYS_CTRL_ID, NO_SUB_ADDRESS, (uint8_t*)&sys_ctrl , SYS_CTRL_LEN);
}

/**
 * @brief Read len bytes of received data from the RX buffer of the DW1000
 * 
 * @param data Pointer to the buffer to store the received data
 * @param n Pointer to the variable to store the length of the received data
 *
 */
void DW1000::readReceivedData(uint8_t** data, uint16_t* length)
{
    /* get length of received data from Frame Info register */
    uint16_t len = getReceivedDataLength();

    uint32_t sys_status = 0;
    readBytes(SYS_STATUS_ID, NO_SUB_ADDRESS, &sys_status);
    if(logger!=nullptr) logger->addBuffer("sys_status is currently! %x", sys_status);

    uint32_t decid = 0;
    readBytes(DEV_ID_ID, NO_SUB_ADDRESS, &decid);
    if(logger!=nullptr) logger->addBuffer("decid is currently! %x", decid);

    uint32_t sys_mask = 0;
    readBytes(SYS_MASK_ID, NO_SUB_ADDRESS, &sys_mask);
    if(logger!=nullptr) logger->addBuffer("sys_mask is currently! %x", sys_mask);


    uint32_t sys_cfg = 0;
    readBytes(SYS_CFG_ID, NO_SUB_ADDRESS, &sys_cfg);
    if(logger!=nullptr) logger->addBuffer("sys_cfg is currently! %x", sys_cfg);

    uint32_t log = 0;
    readBytes(DRX_CONF_ID, DRX_TUNE2_OFFSET, &log);
    if(logger!=nullptr) logger->addBuffer("DRX_CONF_ID DRX_TUNE2_OFFSET is currently! %x", log);

    readBytes(SYS_CTRL_ID, NO_SUB_ADDRESS, &log);
    if(logger!=nullptr) logger->addBuffer("SYS_CTRL_ID is currently! %x", log);

    log = 0;
    readBytes(RX_FWTO_ID, NO_SUB_ADDRESS, &log);
    if(logger!=nullptr) logger->addBuffer("RX_FWTO_ID is currently! %x", log);

    //log = 0xFFF;
    //writeBytes(RX_FWTO_ID, NO_SUB_ADDRESS, log);


    Serial.println(len);
    //len-=2; //TODO check if this is correct
    /* TODO: Check if FCS is good */

    /* */
    uint8_t* rx_data = new uint8_t[len];
    
    if (rx_data == nullptr) {
        if(logger != nullptr) logger->addBuffer("Failed to allocate memory for received data with length 0x%x", *length);
        return;
    }
    /* Read received data from RX_BUFFER of DW1000 */
    readBytes(RX_BUFFER_ID, NO_SUB_ADDRESS, rx_data, len);

    /* update the length of the received buffer */
    *length = len;
    *data = rx_data;
    
    /* delete received data length to prevent double read accesses */
    deleteReceivedDataLength();
}
uint16_t DW1000::getReceivedDataLength()
{
    uint32_t data = 0;
    readBytes(RX_FINFO_ID, NO_SUB_ADDRESS, &data);

    /* Get the length of the received data */
    uint16_t len = data & (RX_FINFO_RXFLE_MASK | RX_FINFO_RXFLEN_MASK);
    return len;
}

void DW1000::deleteReceivedDataLength()
{
    uint32_t data = 0;
    
    data = ~(RX_FINFO_RXFLE_MASK | RX_FINFO_RXFLEN_MASK);
    writeBytes(RX_FINFO_ID, NO_SUB_ADDRESS, data);
}

void DW1000::get_tx_timestamp(DW1000Time& time)
{
    uint8_t data[TX_STAMP_LEN] = {0};
    readBytes(TX_TIME_ID, TX_TIME_TX_STAMP_OFFSET, data, TX_STAMP_LEN);

    /* Get the TX timestamp */
    time.set_timestamp(data);
}
void DW1000::get_rx_timestamp(DW1000Time& time)
{
    uint8_t data[RX_STAMP_LEN] = {0};
    readBytes(RX_TIME_ID, RX_TIME_RX_STAMP_OFFSET, data, RX_STAMP_LEN);

    /* Get the RX timestamp */
    time.set_timestamp(data);
}

void DW1000::addCustomInterruptHandler(InterruptTable interruptTable, std::function<void(uint32_t)> callback)
{ 
    customInterruptCallbackTable = interruptTable;
    customInterruptCallback = callback;
}

void DW1000::removeCustomInterruptHandler()
{
    customInterruptCallbackTable = (InterruptTable) 0;
    customInterruptCallback = nullptr;
}

