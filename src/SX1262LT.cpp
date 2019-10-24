#include <SX1262LT.h>
#include <SPI.h>

//#define SX1262DEBUG             //enable level 1 debug messages
//#define SX1262DEBUG2              //enable level 2 debug messages
//#define SX1262DEBUG3             //enable level 2 debug messages

/*
****************************************************************************
 To Do:
 
 configuration of the register holding the "SX_RX_GAIN_BOOSTED" configuration is not retained in 
 case of a warm reset sleep mode. It means that, when the chip wakes up from sleep mode, the 
 register will have the reset value - i.e. "standard" Rx.
 
 DONE - Add routines for auto calculation of low data rate optimisation setting
 Check frequency bytes versus NiceRF code
 
 Implement changes in datesheet 15. Known Limitations
 Test wakeup with 64khz RTC clock.

****************************************************************************
*/

SX1262Class::SX1262Class()
{
  //Anything you need when instantiating your object goes here
}


bool SX1262Class::begin(int8_t pinNSS, int8_t pinNRESET, int8_t pinRFBUSY, int8_t pinDIO1, int8_t pinDIO2, int8_t pinDIO3)
{
#ifdef SX1262DEBUG
  Serial.println(F("begin()"));
  Serial.println(F("SX1262Class constructor instantiated successfully"));
  Serial.print(F("NSS "));
  Serial.println(_NSS);
  Serial.print(F("NRESET "));
  Serial.println(_NRESET);
  Serial.print(F("RFBUSY "));
  Serial.println(_RFBUSY);
  Serial.print(F("DIO1 "));
  Serial.println(_DIO1);
  Serial.print(F("DIO2 "));
  Serial.println(_DIO2);
  Serial.print(F("DIO3 "));
  Serial.println(_DIO3);
#endif

  pinInit(pinNSS, pinNRESET, pinRFBUSY, pinDIO1, pinDIO2, pinDIO3);

  //assign the passed pins to the class private variabled
  _NSS = pinNSS;
  _NRESET = pinNRESET;
  _RFBUSY = pinRFBUSY;
  _DIO1 = pinDIO1;
  _DIO2 = pinDIO2;
  _DIO3 = pinDIO3;


  resetDevice();
  if (checkDevice())
  {
    return true;
  }

  return false;
}


void SX1262Class::pinInit(int8_t _NSS, int8_t _NRESET, int8_t _RFBUSY, int8_t _DIO1, int8_t _DIO2, int8_t _DIO3)
{

#ifdef SX1262DEBUG
  Serial.println(F("Pin_Init()"));
#endif

  pinMode(_NSS, OUTPUT);
  digitalWrite(_NSS, HIGH);
  pinMode(_NRESET, OUTPUT);
  digitalWrite(_NRESET, LOW);
  pinMode(_RFBUSY, INPUT);

  if (_DIO1 >= 0)
  {
    pinMode( _DIO1, INPUT);
  }
  else
  {
    //Serial.println(F("DIO1 not used"));
  }

  if (_DIO2 >= 0)
  {
    pinMode( _DIO2, INPUT);
  }
  else
  {
    //Serial.println(F("DIO2 not used"));
  }

  if (_DIO3 >= 0)
  {
    pinMode( _DIO3, INPUT);
  }
  else
  {
    //Serial.println(F("DIO3 not used"));
  }
}


void SX1262Class::spiInit(uint8_t msborder, uint8_t clockdiv, uint8_t mode)
{
#ifdef SX1262DEBUG
  Serial.println(F("spiInit()"));
#endif
  SPI.begin();
  SPI.setBitOrder(msborder);           //depends on SX1262 spi timing


  SPI.setClockDivider(clockdiv);       //too fast may cause error
  SPI.setDataMode(mode);
}


void SX1262Class::rxtxpinInit(int8_t pinRXEN, int8_t pinTXEN)
{
#ifdef SX1262DEBUG
  Serial.println(F("rxtxpinInit()"));
#endif

  _RXEN = pinRXEN;
  _TXEN = pinTXEN;

  pinMode(pinRXEN, OUTPUT);
  digitalWrite(pinRXEN, LOW);           //pins needed for E28-2G4M20S
  pinMode(pinTXEN, OUTPUT);
  digitalWrite(pinTXEN, LOW);           //pins needed for E28-2G4M20S
}


void SX1262Class::resetDevice()
{
#ifdef SX1262DEBUG
  Serial.println(F("resetDevice()"));
#endif

  //timings from Semtech library
  //delay(20);
  digitalWrite(_NRESET, LOW);
  delay(2);
  digitalWrite(_NRESET, HIGH);
  delay(20);
}


void SX1262Class::setStandby(uint8_t standbyconfig)
{
#ifdef SX1262DEBUG
  Serial.println(F("setStandby()"));
#endif

  checkBusy();
  digitalWrite(_NSS, LOW);
  SPI.transfer(RADIO_SET_STANDBY);
  SPI.transfer(standbyconfig);
  digitalWrite(_NSS, HIGH);
  _OperatingMode = standbyconfig;
}


void SX1262Class::checkBusy()
{
#ifdef SX1262DEBUG
  Serial.println(F("checkBusy()"));
#endif

  uint8_t busy_timeout_cnt;
  busy_timeout_cnt = 0;

  while (digitalRead(_RFBUSY))
  {
    delay(1);
    busy_timeout_cnt++;

    if (busy_timeout_cnt > 5) //wait 5mS for busy to complete
    {
      busy_timeout_cnt = 0;
      Serial.println(F("ERROR - Busy Timeout!"));
      //resetDevice();          //reset device
      //setStandby(MODE_STDBY_RC);
      //config();               //re-run saved config
      break;
    }
  }
}


void SX1262Class::setPacketType(uint8_t packettype )
{
#ifdef SX1262DEBUG
  Serial.println(F("setPacketType()"));
#endif
  savedPacketType = packettype;

  writeCommand(RADIO_SET_PACKETTYPE, &packettype, 1);
}


void SX1262Class::writeCommand(uint8_t Opcode, uint8_t *buffer, uint16_t size)
{
#ifdef SX1262DEBUG
  Serial.print(F("writeCommand "));
  Serial.println(Opcode, HEX);
#endif

  uint8_t index;
  checkBusy();
  digitalWrite(_NSS, LOW);
  SPI.transfer(Opcode);

  for (index = 0; index < size; index++)
  {
    SPI.transfer(buffer[index]);
  }
  digitalWrite(_NSS, HIGH);

  if (Opcode != RADIO_SET_SLEEP)
  {
    checkBusy();
  }
}


void SX1262Class::printASCIIPacket(uint8_t *buffer, uint8_t size)
{
#ifdef SX1262DEBUG
  Serial.println(F("printASCIIPacket()"));
#endif

  uint8_t index;

  for (index = 0; index < size; index++)
  {
    Serial.write(buffer[index]);
    //Serial.print(F("Index "));
    //Serial.println(index);
  }

}


void SX1262Class::printHEXPacket(uint8_t *buffer, uint8_t size)
{
#ifdef SX1262DEBUG
  Serial.println(F("printHEXPacket()"));
#endif

  uint8_t index;
  
  for (index = 0; index < size; index++)
  {
    Serial.print(F("["));
    Serial.print(index);
    Serial.print(F("],"));
    Serial.print(buffer[index], HEX);
    Serial.print(F("  "));
  }
}


void SX1262Class::setRegulatorMode(uint8_t mode)
{
#ifdef SX1262DEBUG
  Serial.println(F("setRegulatorMode()"));
#endif

  savedRegulatorMode = mode;

  writeCommand(RADIO_SET_REGULATORMODE, &mode, 1);
}



void SX1262Class::printIrqStatus()
{
#ifdef SX1262DEBUG
  Serial.println(F("printIrqStatus()"));
#endif

  uint16_t _IrqStatus;
  _IrqStatus = readIrqStatus();

  //0x0001
  if (_IrqStatus & IRQ_TX_DONE)
  {
    Serial.print(F(",IRQ_TX_DONE"));
  }

  //0x0002
  if (_IrqStatus & IRQ_RX_DONE)
  {
    Serial.print(F(",IRQ_RX_DONE"));
  }

  //0x0004
  if (_IrqStatus & IRQ_PREAMBLE_DETECTED)
  {
    Serial.print(F(",IRQ_PREAMBLE_DETECTED"));
  }

  //0x0008
  if (_IrqStatus & IRQ_SYNCWORD_VALID)
  {
    Serial.print(F(",IRQ_SYNCWORD_VALID"));
  }

  //0x0010
  if (_IrqStatus & IRQ_HEADER_VALID)
  {
    Serial.print(F(",IRQ_HEADER_VALID"));
  }

  //0x0020
  if (_IrqStatus & IRQ_HEADER_ERROR)
  {
    Serial.print(F(",IRQ_HEADER_ERROR"));
  }

  //0x0040
  if (_IrqStatus & IRQ_CRC_ERROR)
  {
    Serial.print(F(",IRQ_CRC_ERROR"));
  }

  //0x0080
  if (_IrqStatus & IRQ_CAD_DONE)
  {
    Serial.print(F(",IRQ_CAD_DONE"));
  }

  //0x0100
  if (_IrqStatus & IRQ_CAD_ACTIVITY_DETECTED)
  {
    Serial.print(",IRQ_CAD_ACTIVITY_DETECTED");
  }

  //0x0200
  if (_IrqStatus & IRQ_RX_TX_TIMEOUT)
  {
    Serial.print(F(",IRQ_RX_TX_TIMEOUT"));
  }
  
}


void SX1262Class::printRegisters(uint16_t Start, uint16_t End)
{
  //prints the contents of SX1262 registers to serial monitor

#ifdef SX1262DEBUG
  Serial.println(F("printRegisters()"));
#endif

  uint16_t Loopv1, Loopv2, RegData;

  Serial.print(F("Reg    0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F"));
  Serial.println();

  for (Loopv1 = Start; Loopv1 <= End;)           //32 lines
  {
    Serial.print(F("0x"));
    Serial.print((Loopv1), HEX);                 //print the register number
    Serial.print(F("  "));
    for (Loopv2 = 0; Loopv2 <= 15; Loopv2++)
    {
      RegData = readRegister(Loopv1);
      if (RegData < 0x10)
      {
        Serial.print(F("0"));
      }
      Serial.print(RegData, HEX);                //print the register number
      Serial.print(F(" "));
      Loopv1++;
    }
    Serial.println();
  }
}


bool SX1262Class::checkDevice()
{
  //check there is a device out there, writes a register and reads back
#ifdef SX1262DEBUG
  Serial.println(F("checkDevice()"));
#endif

  uint8_t Regdata1, Regdata2;
  Regdata1 = readRegister(0x0907);               //mid byte of frequency setting
  writeRegister(0x0907, (Regdata1 + 1));
  Regdata2 = readRegister(0x0907);               //read changed value back
  writeRegister(0x0907, Regdata1);               //restore register to original value

  if (Regdata2 == (Regdata1 + 1))
  {
    return true;
  }
  else
  {
    return false;
  }
}


void SX1262Class::printHEXByte(uint8_t temp)
{
  if (temp < 0x10)
  {
    Serial.print(F("0"));
  }
  Serial.print(temp, HEX);
}


void SX1262Class::printHEXByte0x(uint8_t temp)
{
  //print a byte, adding 0x
  Serial.print(F("0x"));
  if (temp < 0x10)
  {
    Serial.print(F("0"));
  }
  Serial.print(temp, HEX);
}


void SX1262Class::printASCIIorHEX(uint8_t temp)
{
  if ((temp < 0x10) || (temp > 0x7E))
  {
    Serial.print(F(" ("));
    printHEXByte(temp);
    Serial.print(F(") "));
  }
  else
  {
    Serial.write(temp);
  }
}


void SX1262Class::printAddressInfo()
{
  //print the information for packet last received
#ifdef SX1262DEBUG
  Serial.println(F("printAddressInfo()"));
#endif
  Serial.print(F("RXType,"));
  printASCIIorHEX(_RXPacketType);
  Serial.print(F(",Destination,"));
  printASCIIorHEX(_RXDestination);
  Serial.print(F(",Source,"));
  printASCIIorHEX(_RXSource);
}


void SX1262Class::printReceptionInfoLoRa()
{
  //print the information for packet last received
  //note, _PacketSNR has already been converted into a signed value
  //_PacketRSSI is a signed value also
#ifdef SX1262DEBUG
  Serial.println(F("printReceptionInfoLoRa()"));
#endif

  Serial.print(F("SNR,"));
  Serial.print(_PacketSNR);
  Serial.print(F("dB"));

  Serial.print(F(",RSSI,"));
  Serial.print(_PacketRSSI);
  Serial.print(F("dBm"));
}


bool SX1262Class::readPacketCRCError()
{

#ifdef SX1262DEBUG
  Serial.println(F("readPacketCRCError()"));
#endif

  uint16_t IRQreg;
  IRQreg = readIrqStatus();

  if (IRQreg & IRQ_CRC_ERROR)
  {
    return true;
  }
  else
  {
    return false;
  }
}


bool SX1262Class::readPacketHeaderValid()
{
#ifdef SX1262DEBUG
  Serial.println(F("readPacketHeaderValid()"));
#endif

  uint16_t IRQreg;
  IRQreg = readIrqStatus();

  if (IRQreg & IRQ_HEADER_VALID)
  {
    return true;
  }
  else
  {
    return false;
  }
}


bool SX1262Class::readPacketHeaderError()
{
#ifdef SX1262DEBUG
  Serial.println(F("readPacketHeaderError()"));
#endif

  uint16_t IRQreg;
  IRQreg = readIrqStatus();

  if (IRQreg & IRQ_HEADER_ERROR)
  {
    return true;
  }
  else
  {
    return false;
  }
}


bool SX1262Class::readRXDone()
{
#ifdef SX1262DEBUG
  Serial.println(F("readRXDone()"));
#endif

  uint16_t IRQreg;
  IRQreg = readIrqStatus();

  if (IRQreg & IRQ_RX_DONE)
  {
    return true;
  }
  else
  {
    return false;
  }
}


bool SX1262Class::readTXDone()
{
#ifdef SX1262DEBUG
  Serial.println(F("readTXDone()"));
#endif

  uint16_t IRQreg;
  IRQreg = readIrqStatus();

  if (IRQreg & IRQ_TX_DONE)
  {
    return true;
  }
  else
  {
    return false;
  }
}


void SX1262Class::readRXBufferStatus()
{
#ifdef SX1262DEBUG
  Serial.println(F("readRXBufferStatus()"));
#endif

  uint8_t buffer[2];
  readCommand(RADIO_GET_RXBUFFERSTATUS, buffer, 2);
  _RXPacketL = buffer[0];
  _RXBufferPointer = buffer[1];
  }
  


uint8_t SX1262Class::readPacketAddressedLoRa(uint8_t *rxbuffer, uint8_t size)
{
#ifdef SX1262DEBUG
  Serial.println(F("readPacketAddressed()"));
#endif

  uint8_t buffer[2];
  uint8_t index, RegData, _RXStart, _RXEnd;
  uint8_t maxindex = (size - 1);

  readCommand(RADIO_GET_RXBUFFERSTATUS, buffer, 2);

  _RXPacketL = (buffer[0] - 3);
  _RXStart = buffer[1];
  _RXEnd = (_RXPacketL - 1);                    //calculate the new end of the packet in the buffer, to allow for addressing bits

#ifdef SX1262DEBUG2
  Serial.print(F("RXPacketL,"));
  Serial.print(_RXPacketL);
  Serial.print(F(",RXEnd,"));
  Serial.println(_RXEnd);
#endif

  digitalWrite(_NSS, LOW);                      //start the burst read
  SPI.transfer(RADIO_READ_BUFFER);
  SPI.transfer(_RXStart);
  SPI.transfer(0xFF);

  _RXPacketType = SPI.transfer(0);
  _RXDestination = SPI.transfer(0);
  _RXSource = SPI.transfer(0);

  //now fill RXBUFF
  for (index = _RXStart; index <= _RXEnd; index++)
  {
    if (index > maxindex)
    {
      break;
    }

    RegData = SPI.transfer(0);
    rxbuffer[index] = RegData;
  }
  digitalWrite(_NSS, HIGH);

  return _RXPacketL;
}



uint8_t SX1262Class::readPacketLoRa(uint8_t *rxbuffer, uint8_t size)
{
#ifdef SX1262DEBUG
  Serial.println(F("readPacket()"));
#endif

  uint8_t index, regdata, RXstart, RXend;
  uint8_t buffer[2];
  uint8_t counter = 0;
  
  readCommand(RADIO_GET_RXBUFFERSTATUS, buffer, 2);
  _RXPacketL = buffer[0];
  
  Serial.print(F("_RXPacketL "));
  Serial.println(_RXPacketL);
  
  if (_RXPacketL > size)               //check passed buffer is big enough for packet
  {
  _RXPacketL = size;                   //truncate packet if not enough space
  }
  
  RXstart = buffer[1];
  
  Serial.print(F("RXstart "));
  Serial.println(RXstart);
   
  //RXEnd = RXStart + _RXPacketL;        //calculate RXEend
   
  RXend = RXstart + _RXPacketL;

  digitalWrite(_NSS, LOW);               //start the burst read
  SPI.transfer(RADIO_READ_BUFFER);
  SPI.transfer(RXstart);
  SPI.transfer(0xFF);

  for (index = RXstart; index < RXend; index++)
  {
    regdata = SPI.transfer(0);
    rxbuffer[index] = regdata;
    //Serial.print(F("Index "));
    //Serial.println(index);
  }

  digitalWrite(_NSS, HIGH);
  return _RXPacketL;                     //so we can check for packet having enough buffer space
}



bool SX1262Class::sendPacketLoRa(uint8_t *txbuffer, uint8_t size, uint32_t txtimeout, int8_t txpower, uint8_t _DIO)
{
#ifdef SX1262DEBUG
  Serial.println(F("sendPacketLoRa()"));
#endif
  uint8_t index;
  uint8_t maxindex = (size - 1);
  uint8_t bufferdata;

  setStandby(MODE_STDBY_RC);
  setBufferBaseAddress(0, 0);
  checkBusy();
  digitalWrite(_NSS, LOW);
  SPI.transfer(RADIO_WRITE_BUFFER);
  SPI.transfer(0);

  for (index = 0; index <= maxindex; index++)
  {
    bufferdata = txbuffer[index];
    SPI.transfer(bufferdata);

  }

  digitalWrite(_NSS, HIGH);
  
  writeRegister(REG_LR_PAYLOADLENGTH, size);
  setTxParams(txpower, RAMP_TIME);
  setTx(txtimeout);                                                //this starts the TX

  //Serial.println(F("Waiting DIO high "));
  
  while (!digitalRead(_DIO));                                      //Wait for DIO to go high
   
  
   
  if (readIrqStatus() & IRQ_RX_TX_TIMEOUT )                         //check for timeout
  {
    return false;
  }
  else
  {
    return true;
  }
}


bool SX1262Class::sendPacketAddressedLoRa(uint8_t *txbuffer, uint8_t size, char txpackettype, char txdestination, char txsource, uint32_t txtimeout, int8_t txpower, uint8_t _DIO)
{
#ifdef SX1262DEBUG
  Serial.println(F("sendPacketAddressedLoRa()"));
#endif
  uint8_t index;
  uint8_t maxindex = (size - 1);
  uint8_t txpacketL = 0;
  uint8_t bufferdata;
  //uint32_t startmS, timemS;


  txpacketL = size + 3;                           //need to check for minimum of 6 max of 127
  setStandby(MODE_STDBY_RC);
  setBufferBaseAddress(0, 0);
  checkBusy();
  digitalWrite(_NSS, LOW);
  SPI.transfer(RADIO_WRITE_BUFFER);
  SPI.transfer(0);
  SPI.transfer(txpackettype);                     //Write the packet type
  SPI.transfer(txdestination);                    //Destination node
  SPI.transfer(txsource);                         //Source node
  txpacketL = 3 + size;                           //we have added 3 header bytes to size

  for (index = 0; index <= maxindex; index++)
  {
    bufferdata = txbuffer[index];
    SPI.transfer(bufferdata);
  }

  digitalWrite(_NSS, HIGH);
  writeRegister(REG_LR_PAYLOADLENGTH, txpacketL);
  setTxParams(txpower, RADIO_RAMP_10_US);
  //startmS = millis();
  setTx(txtimeout);                               //this starts the TX

  while (!digitalRead(_DIO));                     //Wait for DIO to go high

  //startmS = millis() - startmS;

  if (readIrqStatus() & IRQ_RX_TX_TIMEOUT )        //check for timeout
  {
    return false;
  }
  else
  {
    return true;
  }
}



/*
void SX1262Class::rxEnable(int8_t pinRXEN, int8_t pinTXEN)
{
  //Enable RX mode on Ebyte E28-2G4M20S
#ifdef SX1262DEBUG
  Serial.println(F("rxEnable()"));
#endif

  digitalWrite(pinRXEN, HIGH);
  digitalWrite(pinTXEN, LOW);
}


void SX1262Class::txEnable(int8_t pinRXEN, int8_t pinTXEN)
{
  //Enable TX mode on Ebyte E28-2G4M20S
#ifdef SX1262DEBUG
  Serial.println(F("txEnable()"));
#endif

  digitalWrite(pinRXEN, LOW);
  digitalWrite(pinTXEN, HIGH);
}

*/

/*
void SX1262Class::setRfFrequency(uint32_t frequency, int32_t offset)
{
#ifdef SX1262DEBUG
  Serial.println(F("setRfFrequency()"));
#endif

  savedFrequency = frequency;
  savedOffset = offset;

  frequency = frequency + offset;
  uint8_t buffer[3];
  uint32_t freqtemp = 0;
  freqtemp = ( uint32_t )( (double) frequency / (double)FREQ_STEP);
  buffer[0] = ( uint8_t )( ( freqtemp >> 16 ) & 0xFF );
  buffer[1] = ( uint8_t )( ( freqtemp >> 8 ) & 0xFF );
  buffer[2] = ( uint8_t )( freqtemp & 0xFF );
  writeCommand(RADIO_SET_RFFREQUENCY, buffer, 3);
}
*/

/*
//RF_Freq = freq_reg*32M/(2^25)-----> freq_reg = (RF_Freq * (2^25))/32
void SX1262Class::setRfFrequency( uint32_t frequency, int32_t offset)
{
	uint8_t Opcode;
	uint8_t Rf_Freq[4];
	uint32_t RfFreq = 0;
	
	RfFreq = ( uint32_t )( ( float )frequency / ( float )FREQ_STEP );
	
	//checkBusy();
	
	Opcode = RADIO_SET_RFFREQUENCY;	//0x86
	
	Rf_Freq[0] = (RfFreq>>24)&0xFF;//MSB
	Rf_Freq[1] = (RfFreq>>16)&0xFF;
	Rf_Freq[2] = (RfFreq>>8)&0xFF;
	Rf_Freq[3] = RfFreq&0xFF;//LSB
	
	Serial.print("Frequency bytes ");
	Serial.print(Rf_Freq[0],HEX);
	Serial.print("  ");
	Serial.print(Rf_Freq[1],HEX);
	Serial.print("  ");
	Serial.print(Rf_Freq[2],HEX);
	Serial.print("  ");
	Serial.print(Rf_Freq[3],HEX);
	Serial.println();
	
	
	writeCommand(RADIO_SET_RFFREQUENCY, Rf_Freq, 4);
	//SPI_NSS_LOW();
	//spi_rw(SET_RF_FREQUENCY);
	//spi_rw(Rf_Freq[0]);
	//spi_rw(Rf_Freq[1]);
	//spi_rw(Rf_Freq[2]);
	//spi_rw(Rf_Freq[3]);
	//SPI_NSS_HIGH();
}
*/


void SX1262Class::setRfFrequency( uint32_t frequency, int32_t offset )
{
//Note RF_Freq = freq_reg*32M/(2^25)-----> freq_reg = (RF_Freq * (2^25))/32

  uint8_t Opcode;
  uint8_t Rf_Freq[4];
  uint32_t RfFreq;

  savedFrequency = frequency;
  savedOffset = offset;

  RfFreq = frequency + offset;
  
  RfFreq = ( uint32_t )( ( double )frequency / ( double )FREQ_STEP );

  checkBusy();

  Rf_Freq[0] = (RfFreq>>24)&0xFF;//MSB
  Rf_Freq[1] = (RfFreq>>16)&0xFF;
  Rf_Freq[2] = (RfFreq>>8)&0xFF;
  Rf_Freq[3] = RfFreq&0xFF;//LSB

  #ifdef SX1262DEBUG2
  Serial.print("Frequency bytes ");
  Serial.print(Rf_Freq[0],HEX);
  Serial.print("  ");
  Serial.print(Rf_Freq[1],HEX);
  Serial.print("  ");
  Serial.print(Rf_Freq[2],HEX);
  Serial.print("  ");
  Serial.print(Rf_Freq[3],HEX);
  Serial.println();
  #endif
  
  writeCommand(RADIO_SET_RFFREQUENCY, Rf_Freq, 4);
}


void SX1262Class::setTxParams(int8_t TXpower, uint8_t RampTime)
{
#ifdef SX1262DEBUG
  Serial.println(F("setTxParams()"));
#endif

  uint8_t buffer[2];

  savedTXPower = TXpower;

  buffer[0] = TXpower;
  buffer[1] = (uint8_t)RampTime;
  writeCommand(RADIO_SET_TXPARAMS, buffer, 2);
}


void SX1262Class::setBufferBaseAddress(uint8_t txBaseAddress, uint8_t rxBaseAddress)
{
#ifdef SX1262DEBUG
  Serial.println(F("setBufferBaseAddress()"));
#endif

  uint8_t buffer[2];

  buffer[0] = txBaseAddress;
  buffer[1] = rxBaseAddress;
  writeCommand(RADIO_SET_BUFFERBASEADDRESS, buffer, 2);
}


void SX1262Class::setDioIrqParams(uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask )
{
#ifdef SX1262DEBUG
  Serial.println(F("setDioIrqParams()"));
#endif

  savedIrqMask = irqMask;
  savedDio1Mask = dio1Mask;
  savedDio2Mask = dio2Mask;
  savedDio3Mask = dio3Mask;

  uint8_t buffer[8];

  buffer[0] = (uint8_t) (irqMask >> 8);
  buffer[1] = (uint8_t) (irqMask & 0xFF);
  buffer[2] = (uint8_t) (dio1Mask >> 8);
  buffer[3] = (uint8_t) (dio1Mask & 0xFF);
  buffer[4] = (uint8_t) (dio2Mask >> 8);
  buffer[5] = (uint8_t) (dio2Mask & 0xFF);
  buffer[6] = (uint8_t) (dio3Mask >> 8);
  buffer[7] = (uint8_t) (dio3Mask & 0xFF);
  writeCommand(RADIO_CFG_DIOIRQ, buffer, 8);
}



void SX1262Class::setHighSensitivity()
{
//set RX Boosted gain mode
#ifdef SX1262DEBUG
  Serial.println(F("setHighSensitivity()"));
#endif
  writeRegister( REG_RX_GAIN, 0x96 ); // max LNA gain, increase current by ~2mA for around ~3dB in sensivity
  //writeRegister(REG_LNA_REGIME, (readRegister(REG_LNA_REGIME) | 0xC0));
}


void SX1262Class::setLowPowerRX()
{
//set RX power saving mode
#ifdef SX1262DEBUG
  Serial.println(F("setLowPowerRX()"));
#endif

  //writeRegister(REG_LNA_REGIME, (readRegister(REG_LNA_REGIME) & 0x3F));
  writeRegister( REG_RX_GAIN, 0x94 ); // min LNA gain, reduce current by 2mA for around 3dB loss in sensivity
}


void SX1262Class::setRx(uint32_t timeout)
{
 //SX126x base timeout in units of 15.625 µs
 //timeout passed to function in mS
 //range is 1mS to 67108 seconds
 
#ifdef SX1262DEBUG
  Serial.println(F("setRx()"));
#endif
  uint8_t buffer[3];

  timeout = timeout << 6;         //timeout passed in mS, multiply by 64 to convert units of 15.625us to 1mS
  
  //Serial.print(F("Timeout "));
  //Serial.println(timeout);
  
  checkBusy();
  buffer[0] = (timeout>>16)&0xFF;
  buffer[1] = (timeout>>8)&0xFF;
  buffer[2] = timeout&0xFF;
  
  ///Serial.print(buffer[0],HEX);
  //Serial.print(F(" "));
  //Serial.print(buffer[1],HEX);
  //Serial.print(F(" "));
  //Serial.print(buffer[2],HEX);
  //Serial.println(F(" "));
  writeCommand(RADIO_SET_RX, buffer, 3 );
}



void SX1262Class::setRx2(uint32_t timeout)
{
 //SX126x base timeout in units of 15.625 µs
 //timeout passed to function in mS
 //range is 1mS to 262 seconds
 
#ifdef SX1262DEBUG
  Serial.println(F("setRx()"));
#endif
  uint8_t buffer[3];

  timeout = timeout << 6;         //timeout passed in mS, multiply by 64 to convert units of 15.625us to 1mS
  
  //Serial.print(F("Timeout "));
  //Serial.println(timeout);
  
  checkBusy();
  buffer[0] = (timeout>>16)&0xFF;
  buffer[1] = (timeout>>8)&0xFF;
  buffer[2] = timeout&0xFF;
  
  //Serial.print(buffer[0],HEX);
  //Serial.print(F(" "));
  //Serial.print(buffer[1],HEX);
  //Serial.print(F(" "));
  //Serial.print(buffer[2],HEX);
  //Serial.println(F(" "));
  writeCommand(RADIO_SET_RX, buffer, 3 );
  _OperatingMode = MODE_RX;
}



uint16_t SX1262Class::readIrqStatus()
{
#ifdef SX1262DEBUG
  Serial.print(F("readIrqStatus()"));
#endif

  uint16_t temp;
  uint8_t buffer[2];

  readCommand(RADIO_GET_IRQSTATUS, buffer, 2);
  temp = ((buffer[0] << 8) + buffer[1]);
  return temp;
}


bool SX1262Class::packetOK(uint16_t mask)
{
#ifdef SX1262DEBUG
  Serial.print(F("packetOK()"));
#endif


  uint16_t temp;
  temp = readIrqStatus();

  if (temp == mask)
  {
    return true;
  }
  else
  {
    return false;
  }
}


void SX1262Class::readRegisters(uint16_t address, uint8_t *buffer, uint16_t size)
{
#ifdef SX1262DEBUG
  Serial.println(F("readRegisters()"));
#endif

  uint16_t index;
  uint8_t addr_l, addr_h;

  addr_h = address >> 8;
  addr_l = address & 0x00FF;
  checkBusy();

#ifdef SX1262DEBUG
  Serial.println(F("ReadRegisters "));
  Serial.print(addr_h, HEX);
  Serial.print(addr_l, HEX);
#endif

  digitalWrite(_NSS, LOW);
  SPI.transfer(RADIO_READ_REGISTER);
  SPI.transfer(addr_h);               //MSB
  SPI.transfer(addr_l);               //LSB
  SPI.transfer(0xFF);
  for (index = 0; index < size; index++)
  {
#ifdef SX1262DEBUG
    Serial.println(F(" "));
    Serial.print(*(buffer + index));
#endif
    *(buffer + index) = SPI.transfer(0xFF);
  }
#ifdef SX1262DEBUG
  Serial.println();
#endif
  digitalWrite(_NSS, HIGH);
  checkBusy();
}


uint8_t SX1262Class::readRegister(uint16_t address)
{
#ifdef SX1262DEBUG
  Serial.println(F("readRegister()"));
#endif

  uint8_t data;

  readRegisters(address, &data, 1);
  return data;
}


void SX1262Class::clearIrqStatus(uint16_t irqMask)
{
#ifdef SX1262DEBUG
  Serial.println(F("clearIrqStatus()"));
#endif

  uint8_t buffer[2];

  buffer[0] = (uint8_t) (irqMask >> 8);
  buffer[1] = (uint8_t) (irqMask & 0xFF);
  writeCommand(RADIO_CLR_IRQSTATUS, buffer, 2);
}


void SX1262Class::readCommand(uint8_t Opcode, uint8_t *buffer, uint16_t size)
{
#ifdef SX1262DEBUG
  Serial.println(F("readCommand()"));
#endif

  uint8_t i;
  checkBusy();

  digitalWrite(_NSS, LOW);
  SPI.transfer(Opcode);
  SPI.transfer(0xFF);

  for ( i = 0; i < size; i++ )
  {

    *(buffer + i) = SPI.transfer(0xFF);
  }
  digitalWrite(_NSS, HIGH);

  checkBusy();
}


void SX1262Class::readPacketReceptionLoRa()
{
#ifdef SX1262DEBUG
  Serial.println(F("readPacketReceptionLoRa()"));
#endif

  uint8_t status[5];

  readCommand(RADIO_GET_PACKETSTATUS, status, 5) ;
  _PacketRSSI = -status[0] / 2;

  if ( status[1] < 128 )
  {
    _PacketSNR = status[1] / 4 ;
  }
  else
  {
    _PacketSNR = (( status[1] - 256 ) / 4);
  }

}


void SX1262Class::writeRegisters(uint16_t address, uint8_t *buffer, uint16_t size)
{
#ifdef SX1262DEBUG
  Serial.println(F("writeRegisters()"));
#endif
  uint8_t addr_l, addr_h;
  uint8_t i;

  addr_l = address & 0xff;
  addr_h = address >> 8;
  checkBusy();

#ifdef SX1262DEBUG
  Serial.println(F("WriteRegisters "));
  Serial.print(addr_h, HEX);
  Serial.print(addr_l, HEX);
#endif

  digitalWrite(_NSS, LOW);
  SPI.transfer(RADIO_WRITE_REGISTER);
  SPI.transfer(addr_h);   //MSB
  SPI.transfer(addr_l);   //LSB

  for (i = 0; i < size; i++)
  {
#ifdef SX1262DEBUG
    Serial.println(F(" "));
    Serial.print(buffer[i]);
#endif
    SPI.transfer(buffer[i]);
  }

#ifdef SX1262DEBUG
  Serial.println();
#endif
  digitalWrite(_NSS, HIGH);
  checkBusy();
}


void SX1262Class::writeRegister(uint16_t address, uint8_t value)
{
#ifdef SX1262DEBUG
  Serial.println(F("writeRegisters()"));
#endif

  writeRegisters( address, &value, 1 );
}

/*
void SX1262Class::setSleep(uint8_t config)
{
#ifdef SX1262DEBUG
  Serial.println(F("setSleep()"));
#endif

  //uint8_t sleepConfig;

  //Opcode = 0x84;
  //sleepConfig = 0x07; //bit2: 1: ; bit0: 0:
  checkBusy();
  digitalWrite(_NSS, LOW);
  SPI.transfer(RADIO_SET_SLEEP);
  SPI.transfer(config);
  digitalWrite(_NSS, HIGH);
  delay(1);           //allow time for shutdown
}
*/

void SX1262Class::setTx(uint32_t timeout)
{
 //SX126x base timeout in units of 15.625 µs
 //timeout passed to function in mS
 
#ifdef SX1262DEBUG
  Serial.println(F("setTx()"));
#endif
  uint8_t buffer[3];
    
  timeout = timeout << 6;         //timeout passed in mS, convert to units of 15.625us
  
  //Serial.print(F("Timeout value "));
  //Serial.print(timeout);
  //Serial.print(F("  "));
  
  checkBusy();
  buffer[0] = (timeout>>16)&0xFF;
  buffer[1] = (timeout>>8)&0xFF;
  buffer[2] = timeout&0xFF;
  writeCommand(RADIO_SET_TX, buffer, 3 );
  _OperatingMode = MODE_TX;
}


/*from niceRF libe
void SX1262::SetTx(uint32_t timeout)
{
	uint8_t Opcode;
	uint8_t time_out[3];
	
	CheckBusy();
	Opcode = SET_TX;	//0x83
	time_out[0] = (timeout>>16)&0xFF;//MSB
	time_out[1] = (timeout>>8)&0xFF;
	time_out[2] = timeout&0xFF;//LSB
	
	SPI_NSS_LOW();
	spi_rw(Opcode);
	spi_rw(time_out[0]);
	spi_rw(time_out[1]);
	spi_rw(time_out[2]);
	SPI_NSS_HIGH();
}
*/

uint8_t SX1262Class::readRXPacketL()
{
#ifdef SX1262DEBUG
  Serial.println(F("readRXPacketL()"));
#endif

  uint8_t buffer[2];

  readCommand(RADIO_GET_RXBUFFERSTATUS, buffer, 2);
  _RXPacketL = buffer[0];
  return _RXPacketL;
}


uint8_t SX1262Class::readPacketRSSI()
{
  readPacketReceptionLoRa();
  return _PacketRSSI;
}


uint8_t SX1262Class::readPacketSNR()
{
  readPacketReceptionLoRa();
  return _PacketSNR;
}


uint8_t SX1262Class::readRXPacketType()
{
  return _RXPacketType;
}


uint8_t SX1262Class::readRXDestination()
{
  return _RXDestination;
}


uint8_t SX1262Class::readRXSource()
{
  return _RXSource;
}


bool SX1262Class::sendFIFOLoRa(int32_t txtimeout, int8_t txpower, uint8_t _DIO)
{
#ifdef SX1262DEBUG
  Serial.println(F("sendFIFOLoRa()"));
#endif

  setPacketParams(savedPacketParam1, savedPacketParam2, _TXPacketL, savedPacketParam4, savedPacketParam5);
  setTxParams(txpower, RAMP_TIME);
  setTx(txtimeout);                            //this starts the TX

  while (!digitalRead(_DIO));                                      //Wait for DIO to go high

  if (readIrqStatus() & IRQ_RX_TX_TIMEOUT )                         //check for timeout
  {
    return false;
  }
  else
  {
    return true;
  }
}


void SX1262Class::setPacketParams(uint16_t packetParam1, uint8_t  packetParam2, uint8_t packetParam3, uint8_t packetParam4, uint8_t packetParam5)
{
#ifdef SX1262DEBUG
  Serial.println(F("SetPacketParams()"));
#endif
    
  uint8_t preambleMSB, preambleLSB;

  preambleMSB = packetParam1 >> 8;
  preambleLSB = packetParam1 & 0xFF;

  savedPacketParam1 = packetParam1;
  savedPacketParam2 = packetParam2;
  savedPacketParam3 = packetParam3;
  savedPacketParam4 = packetParam4;
  savedPacketParam5 = packetParam5;

  uint8_t buffer[9];
  buffer[0] = preambleMSB;
  buffer[1] = preambleLSB;
  buffer[2] = packetParam2;
  buffer[3] = packetParam3;
  buffer[4] = packetParam4;
  buffer[5] = packetParam5;
  buffer[6] = 0xFF;
  buffer[7] = 0xFF;
  buffer[8] = 0xFF;
  writeCommand(RADIO_SET_PACKETPARAMS, buffer, 9);

}


void SX1262Class::setModulationParams(uint8_t modParam1, uint8_t modParam2, uint8_t  modParam3, uint8_t  modParam4)
{
#ifdef SX1262DEBUG
  Serial.println(F("setModulationParams()"));
#endif
  uint8_t regvalue;
  uint8_t buffer[4];

regvalue = readRegister(REG_TX_MODULATION);

//Serial.print("REG_TX_MODULATION 0x889 = ");
//Serial.println(regvalue,HEX);


if (modParam2 == LORA_BW_500)
{
writeRegister(REG_TX_MODULATION, (regvalue & 0xFB));         //if bandwidth is 500k set bit 2 to 0, see datasheet 15.1.1
}
else
{
writeRegister(REG_TX_MODULATION, (regvalue | 0x04));         //if bandwidth is < 500k set bit 2 to 0 see datasheet 15.1.1
}

if (modParam4 == LDRO_AUTO)
{
modParam4 = getOptimisation(modParam1, modParam2);          //pass Spreading factor then bandwidth to optimisation calc
}
 
 savedModParam1 = modParam1;
 savedModParam2 = modParam2;
 savedModParam3 = modParam3;
 savedModParam4 = modParam4;
  
 buffer[0] = modParam1;
 buffer[1] = modParam2;
 buffer[2] = modParam3;
 buffer[3] = modParam4;
  
#ifdef SX1262DEBUG2
  Serial.println();
  Serial.print(F("modParam1="));
  Serial.print(modParam1, HEX);
  Serial.print(F(" modParam2="));
  Serial.print(modParam2, HEX);
  Serial.print(F(" modParam3="));
  Serial.print(modParam3, HEX);
  Serial.print(F(" modParam4="));
  Serial.print(modParam4, HEX);
  Serial.println();
#endif

writeCommand(RADIO_SET_MODULATIONPARAMS, buffer, 4);
}


bool SX1262Class::config()
{
#ifdef SX1262DEBUG
  Serial.println(F("config()"));
#endif

  resetDevice();
  setStandby(MODE_STDBY_RC);
  setRegulatorMode(savedRegulatorMode);
  setPacketType(savedPacketType);
  setRfFrequency(savedFrequency, savedOffset);
  setModulationParams(savedModParam1, savedModParam2, savedModParam3, LDRO_ON);
  setPacketParams(savedPacketParam1, savedPacketParam2, savedPacketParam3, savedPacketParam4, savedPacketParam5);
  setDioIrqParams(savedIrqMask, savedDio1Mask, savedDio2Mask, savedDio3Mask);       //set for IRQ on RX done on DIO1
  return true;
}


void SX1262Class::printSavedModulationParams()
{
  Serial.print(F("SavedModulationParams "));
  Serial.print(savedModParam1, HEX);
  Serial.print(F(" "));
  Serial.print(savedModParam2, HEX);
  Serial.print(F(" "));
  Serial.print(savedModParam3, HEX);
  Serial.print(F(" "));
  Serial.print(savedModParam4, HEX);
}


//***************************************************************************
//
//Direct FIFO buffer access routines, these read and write data directly to an from
//the SX1262 buffer. A start address of 0 in the buffer is assumed.
//The startWriteFIFO() routine opens up the SPI interface for direct transfers and
//leaves the bus open. Do not perform other accesses to the SX1262 until the
//endWriteFIFO() function is called.
//
//***************************************************************************


void SX1262Class::startWriteFIFO()
{
#ifdef SX1262DEBUG
  Serial.println(F("startWriteFIFO()"));
#endif

  _TXcount = 0;           //this variable used to keep track of bytes written
  setStandby(MODE_STDBY_RC);
  setBufferBaseAddress(0, 0);
  checkBusy();
  digitalWrite(_NSS, LOW);
  SPI.transfer(RADIO_WRITE_BUFFER);
  SPI.transfer(0);
  //SPI interface ready for byte to write to buffer
}


void SX1262Class::endWriteFIFO()
{
#ifdef SX1262DEBUG
  Serial.println(F("endWriteFIFO()"));
#endif
  _TXPacketL = _TXcount;
  digitalWrite(_NSS, HIGH);
  checkBusy();
}


void SX1262Class::startReadFIFO()
{
#ifdef SX1262DEBUG
  Serial.println(F("startReadFIFO()"));
#endif

  uint8_t rxstart;
  uint8_t buffer[2];

  _RXcount = 0;
  readCommand(RADIO_GET_RXBUFFERSTATUS, buffer, 2);
  _RXPacketL = buffer[0];
  rxstart = buffer[1];

  digitalWrite(_NSS, LOW);               //start the burst read
  SPI.transfer(RADIO_READ_BUFFER);
  SPI.transfer(rxstart);
  SPI.transfer(0xFF);

  //next line would be data = SPI.transfer(0);
  //SPI interface ready for byte to read from
}


void SX1262Class::endReadFIFO()
{
#ifdef SX1262DEBUG
  Serial.println(F("endReadFIFO()"));
#endif

  digitalWrite(_NSS, HIGH);
  checkBusy();
}



void SX1262Class::writeUint8(uint8_t x)
{
#ifdef SX1262DEBUG
  Serial.println(F("writeUint8()"));
#endif

  SPI.transfer(x);

  _TXcount++;                     //increment count of bytes written
}

uint8_t SX1262Class::readUint8()
{
#ifdef SX1262DEBUG
  Serial.println(F("readUint8()"));
#endif
  byte x;

  x = SPI.transfer(0);

  _RXcount++;                      //increment count of bytes read
  return (x);
}


void SX1262Class::writeInt8(int8_t x)
{
#ifdef SX1262DEBUG
  Serial.println(F("writeInt8()"));
#endif

  SPI.transfer(x);

  _TXcount++;                     //increment count of bytes written
}


int8_t SX1262Class::readInt8()
{
#ifdef SX1262DEBUG
  Serial.println(F("readInt8()"));
#endif
  int8_t x;

  x = SPI.transfer(0);

  _RXcount++;                      //increment count of bytes read
  return (x);
}


void SX1262Class::writeInt16(int16_t x)
{
#ifdef SX1262DEBUG
  Serial.println(F("writeInt16()"));
#endif

  SPI.transfer(lowByte(x));
  SPI.transfer(highByte(x));

  _TXcount = _TXcount + 2;         //increment count of bytes written
}


int16_t SX1262Class::readInt16()
{
#ifdef SX1262DEBUG
  Serial.println(F("readInt16()"));
#endif
  byte lowbyte, highbyte;

  lowbyte = SPI.transfer(0);
  highbyte = SPI.transfer(0);

  _RXcount = _RXcount + 2;         //increment count of bytes read
  return ((highbyte << 8) + lowbyte);
}


void SX1262Class::writeUint16(uint16_t x)
{
#ifdef SX1262DEBUG
  Serial.println(F("writeUint16()"));
#endif

  SPI.transfer(lowByte(x));
  SPI.transfer(highByte(x));

  _TXcount = _TXcount + 2;         //increment count of bytes written
}


uint16_t SX1262Class::readUint16()
{
#ifdef SX1262DEBUG
  Serial.println(F("writeUint16()"));
#endif
  byte lowbyte, highbyte;

  lowbyte = SPI.transfer(0);
  highbyte = SPI.transfer(0);

  _RXcount = _RXcount + 2;         //increment count of bytes read
  return ((highbyte << 8) + lowbyte);
}


void SX1262Class::writeInt32(int32_t x)
{
#ifdef SX1262DEBUG
  Serial.println(F("writeInt32()"));
#endif

  byte i, j;

  union
  {
    byte b[4];
    int32_t f;
  } data;
  data.f = x;

  for (i = 0; i < 4; i++)
  {
    j = data.b[i];
    SPI.transfer(j);
  }

  _TXcount = _TXcount + 4;         //increment count of bytes written
}


int32_t SX1262Class::readInt32()
{
#ifdef SX1262DEBUG
  Serial.println(F("readInt32()"));
#endif

  byte i, j;

  union
  {
    byte b[4];
    int32_t f;
  } readdata;

  for (i = 0; i < 4; i++)
  {
    j = SPI.transfer(0);
    readdata.b[i] = j;
  }
  _RXcount = _RXcount + 4;         //increment count of bytes read
  return readdata.f;
}


void SX1262Class::writeUint32(uint32_t x)
{
#ifdef SX1262DEBUG
  Serial.println(F("writeUint32()"));
#endif

  byte i, j;

  union
  {
    byte b[4];
    uint32_t f;
  } data;
  data.f = x;

  for (i = 0; i < 4; i++)
  {
    j = data.b[i];
    SPI.transfer(j);
  }

  _TXcount = _TXcount + 4;         //increment count of bytes written
}


uint32_t SX1262Class::readUint32()
{
#ifdef SX1262DEBUG
  Serial.println(F("readUint32()"));
#endif

  byte i, j;

  union
  {
    byte b[4];
    uint32_t f;
  } readdata;

  for (i = 0; i < 4; i++)
  {
    j = SPI.transfer(0);
    readdata.b[i] = j;
  }
  _RXcount = _RXcount + 4;         //increment count of bytes read
  return readdata.f;
}


void SX1262Class::writeFloat(float x)
{
#ifdef SX1262DEBUG
  Serial.println(F("writeFloat()"));
#endif

  byte i, j;

  union
  {
    byte b[4];
    float f;
  } data;
  data.f = x;

  for (i = 0; i < 4; i++)
  {
    j = data.b[i];
    SPI.transfer(j);
  }

  _TXcount = _TXcount + 4;         //increment count of bytes written
}


float SX1262Class::readFloat()
{
#ifdef SX1262DEBUG
  Serial.println(F("readFloat()"));
#endif

  byte i, j;

  union
  {
    byte b[4];
    float f;
  } readdata;

  for (i = 0; i < 4; i++)
  {
    j = SPI.transfer(0);
    readdata.b[i] = j;
  }
  _RXcount = _RXcount + 4;         //increment count of bytes read
  return readdata.f;
}


//***************************************************************************
//End Direct buffer access routines
//***************************************************************************


uint8_t SX1262Class::readsavedModParam1()
{
  return savedModParam1;
}


uint8_t SX1262Class::readsavedModParam2()
{
  return savedModParam2;
}


uint8_t SX1262Class::readsavedModParam3()
{
  return savedModParam3;
}


uint8_t SX1262Class::readsavedModParam4()
{
  return savedModParam4;
}

uint8_t SX1262Class::readsavedPower()
{
  return savedTXPower;
}




uint32_t SX1262Class::getFreqInt()
{
  //get the current set device frequency from registers, return as long integer
  uint8_t MsbH, MsbL, Mid, Lsb;
  uint32_t uinttemp;
  float floattemp;
  MsbH = readRegister(REG_RFFrequency31_24);
  //Serial.println(MsbH,HEX);
  MsbL = readRegister(REG_RFFrequency23_16);
  //Serial.println(MsbL,HEX);
  Mid = readRegister(REG_RFFrequency15_8);
  //Serial.println(Mid,HEX);
  Lsb = readRegister(REG_RFFrequency7_0);
  //Serial.println(Lsb,HEX);
  floattemp = ( (MsbH * 0x1000000ul) + (MsbL * 0x10000ul) + (Mid * 0x100ul) + Lsb);
  floattemp = ((floattemp * FREQ_STEP) / 1000000ul);
  uinttemp = (uint32_t)(floattemp * 1000000);
  return uinttemp;
}

/*
uint8_t SX1262Class::returnSF(uint8_t data)
{
  return (data >> 4);
}
*/

void SX1262Class::setDIO2AsRfSwitchCtrl()
{
  //uint8_t buffer[1];
  uint8_t mode = 0x01;
  checkBusy();

  //buffer[0] = 0x01;   //DIO2 is selected to be used to control an RF switch; DIO2 = 1 in TX mode
  
  writeCommand(RADIO_SET_RFSWITCHMODE, &mode, 1);
}

/*
void SX1262Class::setDIO2AsRfSwitchCtrl()
{
  uint8_t buffer[1];
  checkBusy();

  buffer[0] = 0x01;   //DIO2 is selected to be used to control an RF switch; DIO2 = 1 in TX mode
  
  writeCommand(RADIO_SET_RFSWITCHMODE, buffer, 1);
}
*/



void SX1262Class::setDIO3AsTCXOCtrl(uint8_t tcxoVoltage)
{
  uint8_t buffer[4];
  checkBusy();

  buffer[0] = tcxoVoltage; 
  buffer[1] = 0x00;
  buffer[2] = 0x00;
  buffer[3] = 0x64;
  
  writeCommand(RADIO_SET_TCXOMODE, buffer, 4);
}


void SX1262Class::setPaConfig(uint8_t device, uint8_t dutycycle)
{
  uint8_t buffer[4];
  checkBusy();

  buffer[0] = dutycycle;   //paDutyCycle
  buffer[1] = 0x07;        //hpMax:0x00~0x07; 7:22dbm
  //buffer[1] = 0x00;        //hpMax:0x00~0x07; 7:22dbm
  buffer[2] = device;      //deviceSel: 0 = SX1262; 1 = SX1261;
  buffer[3] = 0x01;        //reserved, always 0x01
  
  writeCommand(RADIO_SET_PACONFIG, buffer, 4);
}


uint8_t SX1262Class::getOptimisation(uint8_t SpreadingFactor, uint8_t Bandwidth)
{
  //from the passed bandwidth (bandwidth) and spreading factor this routine
  //calculates whether low data rate optimisation should be on or off

  uint32_t tempBandwidth;
  float symbolTime;
  tempBandwidth = returnbandwidth(Bandwidth);
  symbolTime = calcSymbolTime(tempBandwidth, SpreadingFactor);

#ifdef SX1262DEBUG3
  Serial.print(F("Symbol Time "));
  Serial.print(symbolTime, 3);
  Serial.println(F("mS"));
#endif

  if (symbolTime > 16)
  {
#ifdef SX1262DEBUG3
    Serial.println(F("LDR Opt on"));
#endif
    return LDRO_ON;
  }
  else
  {
#ifdef SX1262DEBUG3
    Serial.println(F("LDR Opt off"));
#endif
    return LDRO_OFF;
  }
}


uint32_t SX1262Class::returnbandwidth(uint8_t BWregvalue)
{
  switch (BWregvalue)
  {
    case 0:
      return 7800;

    case 8:
      return 10400;

    case 1:
      return 15600;

    case 9:
      return 20800;

    case 2:
      return 31200;

    case 10:
      return 41700;

    case 3:
      return 62500;

    case 4:
      return 125000;

    case 5:
      return 250000;

    case 6:
      return 500000;

    default:
      break;
  }
  return 0xFFFF;                      //so that a bandwidth not set can be identified
}


float SX1262Class::calcSymbolTime(float Bandwidth, uint8_t SpreadingFactor)
{
  //calculates symbol time from passed bandwidth (lbandwidth) and Spreading factor (lSF)and returns in mS
  float symbolTimemS;
  symbolTimemS = (Bandwidth / pow(2, SpreadingFactor));
  symbolTimemS = (1000 / symbolTimemS);
  return symbolTimemS;
}


uint8_t SX1262Class::readRXBufferPointer()
{
#ifdef SX1262DEBUG
  Serial.println(F("readRXBufferPointer()"));
#endif

  uint8_t buffer[2];

  readCommand(RADIO_GET_RXBUFFERSTATUS, buffer, 2);
  _RXBufferPointer = buffer[1];
  return _RXBufferPointer;
}


void SX1262Class::clearDeviceErrors()
{
  uint8_t buffer[2];
  checkBusy();

  buffer[0] = 0x00;                     //can only clear all errors
  buffer[1] = 0x00;
  
  writeCommand(RADIO_CLEAR_ERRORS, buffer, 2);

}


void SX1262Class::calibrateDevice(uint8_t devices)
{
  checkBusy();
  writeCommand(RADIO_CALIBRATE, &devices, 1);
  delay(4);                              //calibration time for all devices is 3.5mS

}

/*
void SX1262Class::calibrateDevice(uint8_t devices)
{
  uint8_t buffer[1];
  checkBusy();

  buffer[0] = devices;                    
  
  writeCommand(RADIO_CALIBRATE, buffer, 1);
  delay(4);                              //calibration time for all devices is 3.5mS

}
*/



void SX1262Class::printDeviceErrors()
{
#ifdef SX1262DEBUG
  Serial.println(F("printDeviceErrors()"));
#endif

 uint16_t errors;
 uint8_t buffer[2];
 
 readCommand(RADIO_GET_ERROR, buffer, 2);

 //Serial.print(F("Errors "));
 //Serial.print(buffer[0],HEX);
 //Serial.print(F("  "));
 //Serial.print(buffer[1],HEX);
 //Serial.println();
 
 //buffer[0] = 0x01;
 //buffer[1] = 0x01;
 
 errors = (buffer[0] << 8) + buffer[1];
 
  //0x0001
  if (errors & RC64K_CALIB_ERR)
  {
    Serial.print(F(",RC64K_CALIB_ERR"));
  }

  //0x0002
  if (errors & RC13M_CALIB_ERR)
  {
    Serial.print(F(",RC13M_CALIB_ERR"));
  }

  //0x0004
  if (errors & PLL_CALIB_ERR)
  {
    Serial.print(F(",PLL_CALIB_ERR"));
  }

  //0x0008
  if (errors & ADC_CALIB_ERR)
  {
    Serial.print(F(",ADC_CALIB_ERR"));
  }

  //0x0010
  if (errors & IMG_CALIB_ERR)
  {
    Serial.print(F(",IMG_CALIB_ERR"));
  }

  //0x0020
  if (errors & XOSC_START_ERR)
  {
    Serial.print(F(",XOSC_START_ERR"));
  }

  //0x0040
  if (errors & PLL_LOCK_ERR)
  {
    Serial.print(F(",PLL_LOCK_ERR"));
  }

  //0x0080
  if (errors & RFU)
  {
    Serial.print(F(",RFU"));
  }

  //0x0100
  if (errors & PA_RAMP_ERR)
  {
    Serial.print(",PA_RAMP_ERR");
  }
 
  
}


uint32_t SX1262Class::readsavedFrequency()
{
return savedFrequency;
}


uint32_t SX1262Class::readsavedOffset()
{
return savedOffset;
}


uint8_t SX1262Class::readsavedRegulatorMode()
{
return savedRegulatorMode;
}


uint8_t SX1262Class::readsavedPacketType()
{
return savedPacketType;
}
 

void SX1262Class::setRxDutyCycle( uint32_t rxTime, uint32_t sleepTime )
{
    uint8_t buffer[6];

    rxTime = rxTime << 6;         //timeout passed in mS, multiply by 64 to convert units of 15.625us to 1mS
    sleepTime = sleepTime << 6;         //timeout passed in mS, multiply by 64 to convert units of 15.625us to 1mS
    
    buffer[0] = ( uint8_t )( ( rxTime >> 16 ) & 0xFF );
    buffer[1] = ( uint8_t )( ( rxTime >> 8 ) & 0xFF );
    buffer[2] = ( uint8_t )( rxTime & 0xFF );
    buffer[3] = ( uint8_t )( ( sleepTime >> 16 ) & 0xFF );
    buffer[4] = ( uint8_t )( ( sleepTime >> 8 ) & 0xFF );
    buffer[5] = ( uint8_t )( sleepTime & 0xFF );
    writeCommand( RADIO_SET_RXDUTYCYCLE, buffer, 6 );
    _OperatingMode = MODE_RX_DC;
}


uint8_t SX1262Class::getOperatingMode()
{
return _OperatingMode;
}


void SX1262Class::setSleep(uint8_t sleepConfig)
{
    writeCommand(RADIO_SET_SLEEP, &sleepConfig, 1);
    _OperatingMode = MODE_SLEEP;
    delay(1);           //allow time for shutdown
}

void SX1262Class::wakeSleep()
{
digitalWrite(_NSS, LOW);
digitalWrite(_NSS, HIGH);
delay(1);
}


void SX1262Class::printLoraSettings()
{
Serial.print(F("Frequency,"));
Serial.print(readsavedFrequency());
Serial.print(F("hz,SpreadingFactor,"));
Serial.print(readsavedModParam1());
Serial.print(F(",Bandwidth,")); 
Serial.print(returnbandwidth(readsavedModParam2()));
Serial.print(F(",CodingRate,4:"));
Serial.print((readsavedModParam3()+4));
Serial.print(F(",LDROptimisation,"));
Serial.print(readsavedModParam4());   
}

