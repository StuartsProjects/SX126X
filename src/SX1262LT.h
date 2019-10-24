#ifndef SX1262LT_h
#define SX1262LT_h

#include "Arduino.h"
//#include <SX1262LT_Includes.h>

/**************************************************************************

  ToDO
  
  DONE - Tidy Order of routines
  DONE - Move spiInit out of class instance constructor
  DONE - Change GetFreqInt() to getFreqInt()
  
  
  
 Check for packet reception overwriting TXbuffer and RXbuffer
 For single byte commands remove need for buffer - writeCommand(RADIO_CALIBRATE, buffer, 1);
 Check SX1262LT.readPacketAddressed(RXBUFFER, RXBUFFER_SIZE) is not reading uneccssary data - RXpacketL
 Check SX1262LT.readPacketAddressed(RXBUFFER, RXBUFFER_SIZE) for packet length 256
 Check for passing TXbuffer and RX buffer size allows for 256
 Change set frequency to offset varient
 Change readPacketAddressed to readPacketAddressedLoRa
 Check rxEnable and txenable are working
 Why is setSleep(uint8_t config) commented out
 Check writeUint8 readUint8 works with characters
 Check bytes sent for writeInt16 vs writeUInt16
 Check bytes sent for writeInt32 vs writeUInt32
 Add not to library about use of rxEnable and txEnable
 Check setSleep()
 
 
 
 
 DONE - Check packet length setting in send packet routines sendPacketAddressedLoRa
  DONE - Check packet length setting in send packet routines sendPacketAddressedFLRC
  DONE - Check packet length setting in send packet routines sendPacketLoRa
  DONE - Check packet length setting in send packet routines sendPacketFLRC
  
  DONE - Check that send packet routines are sendPacketAddressedLoRa sendPacketAddressedFLRC
  DONE - Remove need for setPacketParamsFLRC( and setPacketParamsLoRa 
  DONE - Remove need for setModulationParamsFLRC, setModulationParamsLoRa
  DONE - Is _SavedLoRaBandwidth needed ? - No
  DONE - Why readpacket and readPacketLoRa - Just in case differences in packet treatmentr in future
  DONE - Check for bool sendPacket - Returns status of send
  DONE - sendPacketAddressedLoRa/FLRC and sendPacketLoRa/FLRC - is it necessary to trap zero length packets ? - Yes
  DONE - Does printASCIIPacket(uint8_t *buff, uint8_t tsize) need size--; - No
  DONE - printASCIIPacket( printing one byte too many ? - No prints null character present at end of buffer
  
  DONE - SendPacketAddressedLoRa remove startmS, timemS
  DONE - Change sendBufferLoRa to sendFIFOLoRa
  DONE - Add packet implicit mode support - Added readPacketLoRaImplicit and examples
  REJECTED - Remove need for <SX1280LT_Includes.h> - cannot see the need
  
  DONE - Check for packet reception overwriting TXbuffer and RXbuffer ends
  DONE - Check changes to detect _RXPacketL > buffer size are replicated in implicit reception
  DONE - Remove definitions for TXBUFFER_SIZE, RXBUFFER_SIZE
  REJECTED - Check readPacketAddressed(RXBUFFER, RXBUFFER_SIZE) for packet length 256 - packet limited to 255bytes
  REJECTED - Remove need for readPacketReceptionLoRa() - cannot as packet reception flagged outside of library
  DONE - Check for uint32_t txtimeout - all changed from unit16_t to uint32_t
  DONE - Change all TXtimeout, txbuffer, rx buffer in function calls to TX and RX
  DONE - Check if printPacketStatus2 printPacketStatus3 are FLRC only - not currently used anyway
  DONE - Check Whitening for FLRC #define   WHITENING   0x08 - removed, use RADIO_WHITENING_OFF
  DONE - Save settings to allow for a SX1280_Config recovery from busy timeout error
  DONE - Change function of setSleep() to setSleep(uint8_t config);
  DONE - Check setSleep(Retain_Data_RAM) retains register comments in sleep - Yes, setSleep(Retain_None) retains nothing
  DONE - Check setSleep puts device in low current mode, additional sleep current of SX1280 about 0.1uA in register retention mode
  DONE - Check recovery from busy timeout error
  DONE - Check frequency error output
  DONE - Check validity of offset calc to setfrequency
  DONE - Add frequency offset in setFrequency to all examples
  DONE - Check that checkBusy() only used in functions where there is SPI access
  DONE - Put  clearIrqStatus(IRQ_RADIO_ALL) in setTX() and SetRX()
  DONE - Set all examples to SF7 and LORA_BW_0400 - apart from ranging which is SF10
  DONE - Test all example programs
  DONE - Remove SX1280DEBUG2 defines
  DONE - Check RX and TX enable pin functions

  
 
 
 
 
 
**************************************************************************/

class SX1262Class  {
  public:

    SX1262Class();

    bool begin(int8_t pinNSS, int8_t pinNRESET, int8_t pinRFBUSY, int8_t pinDIO1, int8_t pinDIO2, int8_t pinDIO3);
    void pinInit(int8_t _NSS, int8_t _NRESET, int8_t _RFBUSY, int8_t _DIO1, int8_t _DIO2, int8_t _DIO3);
    void spiInit(uint8_t MSBOrder, uint8_t ClockDiv, uint8_t Mode);
    void resetDevice();
    void setStandby(uint8_t StdbyConfig);
    void checkBusy();
    void setSleep(uint8_t config);
    void setRegulatorMode(uint8_t mode);
    bool checkDevice();
    bool config();
    //-------------------------------------------------------------------------------
    uint8_t readsavedRegulatorMode();
    void setDIO2AsRfSwitchCtrl();
    void setDIO3AsTCXOCtrl(uint8_t tcxoVoltage);
    void clearDeviceErrors();
    void calibrateDevice(uint8_t devices);
    void printDeviceErrors();
    void wakeSleep();
    uint8_t getOperatingMode();


    void writeCommand(uint8_t Opcode, uint8_t *buffer, uint16_t size );
    void readCommand( uint8_t Opcode, uint8_t *buffer, uint16_t size );
    void writeRegisters( uint16_t address, uint8_t *buffer, uint16_t size );
    void writeRegister( uint16_t address, uint8_t value );
    void readRegisters( uint16_t address, uint8_t *buffer, uint16_t size );
    uint8_t readRegister( uint16_t address );
    void printRegisters(uint16_t Start, uint16_t End);
    void setPacketParams(uint16_t packetParam1, uint8_t  packetParam2, uint8_t packetParam3, uint8_t packetParam4, uint8_t packetParam5);
    void setModulationParams(uint8_t modParam1, uint8_t modParam2, uint8_t  modParam3, uint8_t  modParam4);
    void printSavedModulationParams();


    void setRfFrequency( uint32_t frequency, int32_t offset );
    uint32_t getFreqInt();
    void setTxParams(int8_t TXpower, uint8_t RampTime);
    void setTx(uint32_t timeout);
    bool readTXDone();
    void setRx(uint32_t timeout);
    void setLowPowerRX();
    void setHighSensitivity();
    bool readRXDone();
    void readRXBufferStatus();
    uint8_t readRXPacketL();
    uint8_t readPacketRSSI();
    uint8_t readPacketSNR();
    //-------------------------------------------------------------------------------
    void setPaConfig(uint8_t device, uint8_t dutycycle);
    void setRx2(uint32_t timeout);
    void setRxDutyCycle( uint32_t rxTime, uint32_t sleepTime );

    uint8_t readRXBufferPointer();
    uint32_t readsavedFrequency();
    uint32_t readsavedOffset();


    void setBufferBaseAddress(uint8_t txBaseAddress, uint8_t rxBaseAddress);
    void setPacketType(uint8_t PacketType);
    //-------------------------------------------------------------------------------
    uint8_t readsavedPacketType();


    void clearIrqStatus( uint16_t irq );
    uint16_t readIrqStatus();
    void setDioIrqParams(uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask );
    void printIrqStatus();

    bool readPacketCRCError();
    bool readPacketHeaderValid();
    bool readPacketHeaderError();
    uint16_t returnBandwidth(uint8_t data);
    uint8_t returnSF(uint8_t data);
    uint8_t readsavedPower();

    void printASCIIorHEX(uint8_t temp);
    void printHEXByte(uint8_t temp);
    void printHEXByte0x(uint8_t temp);
    bool packetOK(uint16_t mask);
    void printASCIIPacket(uint8_t *buff, uint8_t tsize);
    void printHEXPacket(uint8_t *buff, uint8_t tsize);

    uint8_t readsavedModParam1();
    uint8_t readsavedModParam2();
    uint8_t readsavedModParam3();
    uint8_t readsavedModParam4();


    //*******************************************************************************
    //Packet addressing routines
    //*******************************************************************************

    uint8_t readRXPacketType();
    uint8_t readRXDestination();
    uint8_t readRXSource();
    void printAddressInfo();

    //*******************************************************************************
    //LoRa specific Routines
    //*******************************************************************************

    bool sendPacketAddressedLoRa(uint8_t *txbuffer, uint8_t size, char txpackettype, char txdestination, char txsource, uint32_t txtimeout, int8_t txpower, uint8_t _DIO);
    bool sendPacketLoRa(uint8_t *txbuffer, uint8_t size, uint32_t TXTimeout, int8_t TXPower, uint8_t _DIO);
    uint8_t readPacketLoRa(uint8_t *rxbuffer, uint8_t size);
    void readPacketReceptionLoRa();
    void printReceptionInfoLoRa();
    uint32_t returnbandwidth(uint8_t BWregvalue);
    

    //-------------------------------------------------------------------------------
    bool sendFIFOLoRa(int32_t txtimeout, int8_t txpower, uint8_t _DIO);
    uint8_t readPacketAddressedLoRa(uint8_t *rxbuffer, uint8_t size);
    uint8_t getOptimisation(uint8_t SpreadingFactor, uint8_t Bandwidth);
    float calcSymbolTime(float Bandwidth, uint8_t SpreadingFactor);
    void printLoraSettings();

    //*******************************************************************************
    //Read Write Buffer commands
    //*******************************************************************************

    void startWriteFIFO();
    void endWriteFIFO();
    void startReadFIFO();
    void endReadFIFO();

    void writeUint8(uint8_t x);
    uint8_t readUint8();

    void writeInt8(int8_t x);
    int8_t readInt8();

    void writeInt16(int16_t x);
    int16_t readInt16();

    void writeUint16(uint16_t x);
    uint16_t readUint16();

    void writeInt32(int32_t x);
    int32_t readInt32();

    void writeUint32(uint32_t x);
    uint32_t readUint32();

    void writeFloat(float x);
    float readFloat();

    //*******************************************************************************
    //RXTX Switch routines
    //*******************************************************************************  

    void rxtxpinInit(int8_t pinRXEN, int8_t pinTXEN);
    void rxEnable();
    void txEnable();

    private:

    int8_t _NSS, _NRESET, _RFBUSY, _DIO1, _DIO2, _DIO3;
    int8_t _RXEN, _TXEN;
    uint8_t _SavedLoRaBandwidth;   //used to record the LoRa bandwidth configured, needed for FLRC
    uint8_t _RXPacketL;             //length of packet received
    uint8_t _RXPacketType;          //type number of received packet
    uint8_t _RXDestination;         //destination address of received packet
    uint8_t _RXSource;              //source address of received packet
    int8_t  _PacketRSSI;            //RSSI of received packet
    int8_t  _PacketSNR;             //signal to noise ratio of received packet
    int8_t  _TXPacketL;
    uint8_t _RXcount;               //used to keep track of the bytes read from SX1262 buffer during readFloat() etc
    uint8_t _TXcount;               //used to keep track of the bytes written to SX1262 buffer during writeFloat() etc
    uint8_t _RXBufferPointer;       //pointer to first byte of packet in buffer
    uint8_t _OperatingMode;         //current operating mode

    //config variables 36 bytes, allows for device to be reset and reconfigured via confg();
    uint32_t savedFrequency;
    uint32_t savedOffset;
    uint8_t  savedPacketType;
    uint8_t  savedRegulatorMode;


    uint8_t  savedModParam1, savedModParam2, savedModParam3, savedModParam4;
    uint16_t savedPacketParam1;
    uint8_t  savedPacketParam2, savedPacketParam3, savedPacketParam4, savedPacketParam5;
    uint16_t savedIrqMask, savedDio1Mask, savedDio2Mask, savedDio3Mask;
    int8_t   savedTXPower;


};
#endif




//SX1262LT Includes


//**********************************************************************************************
// Start defines from SX126x.h
//**********************************************************************************************

#define XTAL_FREQ                                   32000000
#define FREQ_DIV                                    33554432
#define FREQ_STEP                                   0.95367431640625 // ( ( double )( XTAL_FREQ / ( double )FREQ_DIV ) )
#define FREQ_ERR                                    0.47683715820312

#define AUTO_RX_TX_OFFSET                           2

#define CRC_IBM_SEED                                0xFFFF
#define CRC_CCITT_SEED                              0x1D0F
#define CRC_POLYNOMIAL_IBM                          0x8005
#define CRC_POLYNOMIAL_CCITT                        0x1021

//Registers
#define REG_LR_WHITSEEDBASEADDR_MSB                 0x06B8
#define REG_LR_WHITSEEDBASEADDR_LSB                 0x06B9
#define REG_LR_CRCSEEDBASEADDR                      0x06BC
#define REG_LR_CRCPOLYBASEADDR                      0x06BE
#define REG_LR_SYNCWORDBASEADDRESS                  0x06C0
#define REG_LR_PAYLOADLENGTH                        0x0702
#define REG_LR_PACKETPARAMS                         0x0704
#define REG_LR_SYNCWORD                             0x0740
#define REG_FREQUENCY_ERRORBASEADDR                 0x076B
#define RANDOM_NUMBER_GENERATORBASEADDR             0x0819
#define REG_RX_GAIN                                 0x08AC
#define REG_OCP                                     0x08E7
#define REG_XTA_TRIM                                0x0911
#define REG_TX_MODULATION                           0x0889


#define LORA_MAC_PRIVATE_SYNCWORD                   0x1424
#define LORA_MAC_PUBLIC_SYNCWORD                    0x3444

//radio operatine modes 
#define MODE_SLEEP                                  0x07
#define MODE_STDBY_RC                               0x00   //Device running on RC13M, set STDBY_RC mode 
#define MODE_STDBY_XOSC                             0x01   //Device running on XTAL 32MHz, set STDBY_XOSC mode
#define MODE_FS                                     0x02       // The radio is in frequency synthesis mode
#define MODE_TX                                     0x03       //TX mode
#define MODE_RX                                     0x04       //RX mode
#define MODE_RX_DC                                  0x05       //RX duty cycle mode
#define MODE_CAD                                    0x06       //RX CAD mode 


//Sleep Mode Definition
#define COLD_START 0x00
#define WARM_START  0x04
#define RTC_TIMEOUT_DISABLE  0x00
#define RTC_TIMEOUT_ENABLE   0x01 
 


#define USE_LDO                                     0x00   //default
#define USE_DCDC                                    0x01


#define    PACKET_TYPE_GFSK                         0x00
#define    PACKET_TYPE_LORA                         0x01
#define    PACKET_TYPE_NONE                         0x0F


#define    RADIO_RAMP_10_US                         0x00
#define    RADIO_RAMP_20_US                         0x01
#define    RADIO_RAMP_40_US                         0x02
#define    RADIO_RAMP_80_US                         0x03
#define    RADIO_RAMP_200_US                        0x04
#define    RADIO_RAMP_800_US                        0x05
#define    RADIO_RAMP_1700_US                       0x06
#define    RADIO_RAMP_3400_US                       0x07

#define    LORA_CAD_01_SYMBOL                       0x00
#define    LORA_CAD_02_SYMBOL                       0x01
#define    LORA_CAD_04_SYMBOL                       0x02
#define    LORA_CAD_08_SYMBOL                       0x03
#define    LORA_CAD_16_SYMBOL                       0x04

#define    LORA_CAD_ONLY                            0x00
#define    LORA_CAD_RX                              0x01
#define    LORA_CAD_LBT                             0x10

#define    LORA_SF5                                 0x05
#define    LORA_SF6                                 0x06
#define    LORA_SF7                                 0x07
#define    LORA_SF8                                 0x08
#define    LORA_SF9                                 0x09
#define    LORA_SF10                                0x0A
#define    LORA_SF11                                0x0B
#define    LORA_SF12                                0x0C

#define    LORA_BW_500                              6      //actual 500000hz
#define    LORA_BW_250                              5      //actual 250000hz
#define    LORA_BW_125                              4      //actual 125000hz
#define    LORA_BW_062                              3      //actual  62500hz 
#define    LORA_BW_041                              10     //actual  41670hz
#define    LORA_BW_031                              2      //actual  31250hz 
#define    LORA_BW_020                              9      //actual  20830hz
#define    LORA_BW_015                              1      //actual  15630hz
#define    LORA_BW_010                              8      //actual  10420hz 
#define    LORA_BW_007                              0      //actual   7810hz

#define    LORA_CR_4_5                              0x01
#define    LORA_CR_4_6                              0x02
#define    LORA_CR_4_7                              0x03
#define    LORA_CR_4_8                              0x04

#define    RADIO_PREAMBLE_DETECTOR_OFF              0x00         //!< Preamble detection length off
#define    RADIO_PREAMBLE_DETECTOR_08_BITS          0x04         //!< Preamble detection length 8 bits
#define    RADIO_PREAMBLE_DETECTOR_16_BITS          0x05         //!< Preamble detection length 16 bits
#define    RADIO_PREAMBLE_DETECTOR_24_BITS          0x06         //!< Preamble detection length 24 bits
#define    RADIO_PREAMBLE_DETECTOR_32_BITS          0x07         //!< Preamble detection length 32 bit

#define    RADIO_ADDRESSCOMP_FILT_OFF               0x00         //!< No correlator turned on i.e. do not search for SyncWord
#define    RADIO_ADDRESSCOMP_FILT_NODE              0x01
#define    RADIO_ADDRESSCOMP_FILT_NODE_BROAD        0x02

#define    RADIO_PACKET_FIXED_LENGTH                0x00         //!< The packet is known on both sides no header included in the packet
#define    RADIO_PACKET_VARIABLE_LENGTH             0x01         //!< The packet is on variable size header included

#define    RADIO_CRC_OFF                            0x01         //!< No CRC in use
#define    RADIO_CRC_1_BYTES                        0x00
#define    RADIO_CRC_2_BYTES                        0x02
#define    RADIO_CRC_1_BYTES_INV                    0x04
#define    RADIO_CRC_2_BYTES_INV                    0x06
#define    RADIO_CRC_2_BYTES_IBM                    0xF1
#define    RADIO_CRC_2_BYTES_CCIT                   0xF2

#define    RADIO_DC_FREE_OFF                        0x00
#define    RADIO_DC_FREEWHITENING                   0x01

#define    LORA_PACKET_VARIABLE_LENGTH              0x00         //!< The packet is on variable size header included
#define    LORA_PACKET_FIXED_LENGTH                 0x01         //!< The packet is known on both sides no header included in the packet
#define    LORA_PACKET_EXPLICIT                     LORA_PACKET_VARIABLE_LENGTH
#define    LORA_PACKET_IMPLICIT                     LORA_PACKET_FIXED_LENGTH


#define    LORA_CRC_ON                              0x01         //!< CRC activated
#define    LORA_CRC_OFF                             0x00         //!< CRC not used

#define    LORA_IQ_NORMAL                           0x00
#define    LORA_IQ_INVERTED                         0x01

#define    TCXO_CTRL_1_6V                           0x00
#define    TCXO_CTRL_1_7V                           0x01
#define    TCXO_CTRL_1_8V                           0x02
#define    TCXO_CTRL_2_2V                           0x03
#define    TCXO_CTRL_2_4V                           0x04
#define    TCXO_CTRL_2_7V                           0x05
#define    TCXO_CTRL_3_0V                           0x06
#define    TCXO_CTRL_3_3V                           0x07

#define    IRQ_RADIO_NONE                           0x0000
#define    IRQ_TX_DONE                              0x0001
#define    IRQ_RX_DONE                              0x0002
#define    IRQ_PREAMBLE_DETECTED                    0x0004
#define    IRQ_SYNCWORD_VALID                       0x0008
#define    IRQ_HEADER_VALID                         0x0010
#define    IRQ_HEADER_ERROR                         0x0020
#define    IRQ_CRC_ERROR                            0x0040
#define    IRQ_CAD_DONE                             0x0080
#define    IRQ_CAD_ACTIVITY_DETECTED                0x0100
#define    IRQ_RX_TX_TIMEOUT                        0x0200
#define    IRQ_RADIO_ALL                            0xFFFF

#define    RADIO_GET_STATUS                         0xC0
#define    RADIO_WRITE_REGISTER                     0x0D
#define    RADIO_READ_REGISTER                      0x1D
#define    RADIO_WRITE_BUFFER                       0x0E
#define    RADIO_READ_BUFFER                        0x1E
#define    RADIO_SET_SLEEP                          0x84
#define    RADIO_SET_STANDBY                        0x80
#define    RADIO_SET_FS                             0xC1
#define    RADIO_SET_TX                             0x83
#define    RADIO_SET_RX                             0x82
#define    RADIO_SET_RXDUTYCYCLE                    0x94
#define    RADIO_SET_CAD                            0xC5
#define    RADIO_SET_TXCONTINUOUSWAVE               0xD1
#define    RADIO_SET_TXCONTINUOUSPREAMBLE           0xD2
#define    RADIO_SET_PACKETTYPE                     0x8A
#define    RADIO_GET_PACKETTYPE                     0x11
#define    RADIO_SET_RFFREQUENCY                    0x86
#define    RADIO_SET_TXPARAMS                       0x8E
#define    RADIO_SET_PACONFIG                       0x95
#define    RADIO_SET_CADPARAMS                      0x88
#define    RADIO_SET_BUFFERBASEADDRESS              0x8F
#define    RADIO_SET_MODULATIONPARAMS               0x8B
#define    RADIO_SET_PACKETPARAMS                   0x8C
#define    RADIO_GET_RXBUFFERSTATUS                 0x13
#define    RADIO_GET_PACKETSTATUS                   0x14
#define    RADIO_GET_RSSIINST                       0x15
#define    RADIO_GET_STATS                          0x10
#define    RADIO_RESET_STATS                        0x00
#define    RADIO_CFG_DIOIRQ                         0x08
#define    RADIO_GET_IRQSTATUS                      0x12
#define    RADIO_CLR_IRQSTATUS                      0x02
#define    RADIO_CALIBRATE                          0x89
#define    RADIO_CALIBRATEIMAGE                     0x98
#define    RADIO_SET_REGULATORMODE                  0x96
#define    RADIO_GET_ERROR                          0x17
#define    RADIO_CLEAR_ERRORS                       0x07
#define    RADIO_SET_TCXOMODE                       0x97
#define    RADIO_SET_TXFALLBACKMODE                 0x93
#define    RADIO_SET_RFSWITCHMODE                   0x9D
#define    RADIO_SET_STOPRXTIMERONPREAMBLE          0x9F
#define    RADIO_SET_LORASYMBTIMEOUT                0xA0

#define REG_RFFrequency31_24              0x88B
#define REG_RFFrequency23_16              0x88C
#define REG_RFFrequency15_8               0x88D
#define REG_RFFrequency7_0                0x88E


#define  LDRO_OFF     0x00
#define  LDRO_ON      0x01
#define  LDRO_AUTO    0x02               //when set causes LDRO to be automatically calculated 

#define DEVICE_SX1261  0x01
#define DEVICE_SX1262  0x00


//**********************************************************************************************
// End defines from SX126x.h
//**********************************************************************************************


//#define TXBUFFER_SIZE_DEFAULT  128
//#define RXBUFFER_SIZE_DEFAULT  128
//#define BUFFER_SIZE_DEFAULT    128

/*
#ifndef TXBUFFER_SIZE
#define TXBUFFER_SIZE 16
#endif

#ifndef RXBUFFER_SIZE
#define RXBUFFER_SIZE 16
#endif
*/

#ifndef POWER
#define POWER 1
#endif


#ifndef RAMP_TIME
#define RAMP_TIME RADIO_RAMP_10_US
#endif

/*
//default setTX Parameters
#ifndef PERIODBASE
#define PERIODBASE PERIOBASE_01_MS
#endif

#ifndef PERIODBASE_COUNT_15_8
#define PERIODBASE_COUNT_15_8 0
#endif

#ifndef PERIODBASE_COUNT_7_0
#define PERIODBASE_COUNT_7_0 0
#endif
*/

#define RC64KEnable       0x01                   //calibrate RC64K clock
#define RC13MEnable       0x02                   //calibrate RC13M clock
#define PLLEnable         0x04                   //calibrate PLL
#define ADCPulseEnable    0x08                   //calibrate ADC Pulse
#define ADCBulkNEnable    0x10                   //calibrate ADC bulkN
#define ADCBulkPEnable    0x20                   //calibrate ADC bulkP
#define ImgEnable         0x40                   //calibrate image
#define ALLDevices        0x7F                   //calibrate all devices 

#define RC64K_CALIB_ERR   0x0001
#define RC13M_CALIB_ERR   0x0002 
#define PLL_CALIB_ERR     0x0004
#define ADC_CALIB_ERR     0x0008
#define IMG_CALIB_ERR     0x0010
#define XOSC_START_ERR    0x0020
#define PLL_LOCK_ERR      0x0040 
#define RFU               0x0080
#define PA_RAMP_ERR       0x0100



