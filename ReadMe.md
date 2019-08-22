# SX1280 - Semtech 2.4GHz LoRa Transceivers

This is a a repository for my Arduino library for the Semtech SX1280 LoRa device. This device is available in modules from NiceRF and EByte. The code will support the Ebyte 14 pin modules but not the 16 pin modules that require RX and TX switching. These modules are all 3.3V devices, do not use directly with 5V Arduinos.

The SX1280 operates in the 2.4GHz band. In addition to having a LoRa modem the SX1280 can send GFSK and FLRC (Fast Long Range Communication) packets. 
These SX1280 devices have been available since 2017 and can be used for both point to point applications and distance measurements using the built in ranging function.

The SX1280 device supports LoRa, FLRC, GFSK and BLE type communications, see the device data sheet for full details. The library here currently only supports LoRa and FLRC mode. FLRC (Fast Long Range Communication) mode is specified as having the same sensitivity as the 203kbps LoRa mode but with an improved data rate of 975kbps.  

The library is a Work in progress, and though the provided examples do work, there are still some issues to attend to and changes to be made, see the section 'Changes Required to Library' at the bottom of this document. 



### Distance measurements

As well as providing point to point LoRa communications the SX1280 has a ranging function which measures the time of flight of of a packet exchange between the initiator and receiver and this can be converted to a distance. 

Distances of up to **85km** were recorded for ranging and **89km** for point to point. 

### Testing reports

There is a GITHUB repository where there are reports of the distance testing of the SX1280 device, its located here; 

[https://github.com/LoRaTracker/SX1280_Testing](https://github.com/LoRaTracker/SX1280_Testing)

### Program examples

The \examples folder of the library contains basic transmitter and receiver programs for both LoRa and soon FLRC. There are link test transmitter programs as well, these program allows the performance of a link to be measured.

### SX1280 connections  

The SX1280 can operate with a UART or SPI based interface. All the example programs use the SPI interface. The SX1280 will need pin connections for NSS (select) NRESET (reset) RFBUSY (busy) and one of the interrupt out pins, DIO1 is used in the examples. The SPI connections for the SPI interface, SCK, MOSI and MISO are needed also. Most of the testing and evaluation of the SX1280 was carried out using Mikrobus compatible boards, see the boards folder in the testing GITHUB link given above for details. 

### Library installation

To install the library select the 'Clone or download' button on the main Github page, then select 'Download Zip'. In the Arduino IDE select 'Sketch' then 'Include Library'. Next select 'Add .ZIP library' and browse to and select the ZIP file you downloaded, it's called 'SX1280-master.zip'.

### Compatibility

Tested on 3.3V 8Mhz ATMega328P and ATMega1284P. 

<br>
<br>

### Changes Required to Library

Check save settings to allow for a SX1280_Config recovery from busy timeout error

Remove need for setPacketParamsFLRC( and setPacketParamsLoRa(

Remove need for setModulationParamsFLRC, setModulationParamsLoRa<br>

Is _SavedLoRaBandwidth needed ?

Does printASCIIPacket(uint8_t *buff, uint8_t tsize) need size--;

Check Whitening for FLRC #define   WHITENING   0x08          //No Whitnening avaialble in FLRC, must be set to off

PrintASCIIPacket maybe printing one byte too many ?

SendPacketAddressedLoRa needs to trap zero length packet

SendPacketAddressedLoRa remove startmS, timemS;

Remove need for readPacketReceptionLoRa()

Change function of setSleep(uint8_t config);

Change sendBufferLoRa to sendFIFOLoRa

Remove need for <SX1280LT_Includes.h>

Remove definitions for TXBUFFER_SIZE, RXBUFFER_SIZE

Check for uint32_t txtimeout

Check readPacketAddressed(RXBUFFER, RXBUFFER_SIZE) for packet length 256

Check for bool sendPacket

Check for passing TXbuffer and RX buffer size allows for 256

Add packet implicit mode support

Check frequency error output

<br>
<br>


### Stuart Robinson
### August 2019