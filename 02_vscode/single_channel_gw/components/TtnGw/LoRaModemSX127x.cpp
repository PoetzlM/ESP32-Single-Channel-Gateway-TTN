/*=======================================================================
Lib to interface with the semtech sx127x chips
Lib is based on arduino-LoRa (https://github.com/sandeepmistry/arduino-LoRa)
Library should work Semtech SX1276/77/78/79, but only tested on sx12767
Requirements:
- esp-idf
- arduino framework
- freertos
Version: 0.1
Author: MPO
Date: 30-21-2021
=======================================================================*/

#include "LoRaModemSX127x.h"

// registers
#define REG_FIFO 0x00
#define REG_OP_MODE 0x01
#define REG_FRF_MSB 0x06
#define REG_FRF_MID 0x07
#define REG_FRF_LSB 0x08
#define REG_PA_CONFIG 0x09
#define REG_OCP 0x0b
#define REG_LNA 0x0c
#define REG_FIFO_ADDR_PTR 0x0d
#define REG_FIFO_TX_BASE_ADDR 0x0e
#define REG_FIFO_RX_BASE_ADDR 0x0f
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS 0x12
#define REG_RX_NB_BYTES 0x13
#define REG_MODEM_STAT 0x13
#define REG_PKT_SNR_VALUE 0x19
#define REG_PKT_RSSI_VALUE 0x1a
#define REG_RSSI_VALUE 0x1b
#define REG_MODEM_CONFIG_1 0x1d
#define REG_MODEM_CONFIG_2 0x1e
#define REG_PREAMBLE_MSB 0x20
#define REG_PREAMBLE_LSB 0x21
#define REG_PAYLOAD_LENGTH 0x22
#define REG_MODEM_CONFIG_3 0x26
#define REG_FREQ_ERROR_MSB 0x28
#define REG_FREQ_ERROR_MID 0x29
#define REG_FREQ_ERROR_LSB 0x2a
#define REG_RSSI_WIDEBAND 0x2c
#define REG_DETECTION_OPTIMIZE 0x31
#define REG_INVERTIQ 0x33
#define REG_DETECTION_THRESHOLD 0x37
#define REG_SYNC_WORD 0x39
#define REG_INVERTIQ2 0x3b
#define REG_DIO_MAPPING_1 0x40 // Mapping of pins DIO0 to DIO3
#define REG_VERSION 0x42
#define REG_PA_DAC 0x4d
//added
#define REG_IRQ_MAKS 0x11
#define REG_DIO_MAPPING_2 0x41 // Mapping of pins DIO4 and DIO5, ClkOut frequency

#define REG_SYMB_TIMEOUT_LSB 0x1F

// modes
//RegOpMode
#define MODE_LONG_RANGE_MODE 0x80
#define MODE_SLEEP 0x00
#define MODE_STDBY 0x01
#define MODE_TX 0x03
#define MODE_RX_CONTINUOUS 0x05
#define MODE_RX_SINGLE 0x06
//added
#define MODE_CAD 0x07

// PA config
#define PA_BOOST 0x80

// IRQ masks
#define IRQ_TX_DONE_MASK 0x08           // End of Transmission
#define IRQ_PAYLOAD_CRC_ERROR_MASK 0x20 // CRC error detected. Note that RXDONE will also be set
#define IRQ_RX_DONE_MASK 0x40           // RXDONE after receiving the header and CRC, we receive payload part
#define IRQ_RX_TOUT_MASK 0x80           // RXTOUT
#define IRQ_HEADER_MASK 0x10            // valid HEADER mask. This interrupt is first when receiving a message
#define IRQ_CD_DONE_MASK 0x04           // CDDONE
#define IRQ_FHSSCH_MASK 0x02            //request frequency hopping
#define IRQ_CD_DETD_MASK 0x01           // Detect preamble channel

//Modem status masks
#define STATUS_SIG_DETECT_MASK 0x01
#define STATUS_SIG_SYNC_MASK 0x02
#define STATUS_RX_ONGOING_MASK 0x04
#define STATUS_HEADER_VALID_MASK 0x08
#define STATUS_MODEM_CLEAR_MASK 0x10

// DIO function mappings 		 D0D1D2D3
#define MAP_DIO0_LORA_RXDONE 0x00  // 00------ bit 7 and 6
#define MAP_DIO0_LORA_TXDONE 0x40  // 01------
#define MAP_DIO0_LORA_CADDONE 0x80 // 10------
#define MAP_DIO0_LORA_NOP 0xC0     // 11------

#define MAP_DIO1_LORA_RXTOUT 0x00    // --00---- bit 5 and 4
#define MAP_DIO1_LORA_FCC 0x10       // --01----
#define MAP_DIO1_LORA_CADDETECT 0x20 // --10----
#define MAP_DIO1_LORA_NOP 0x30       // --11----

// FSK specific
#define MAP_DIO0_FSK_READY 0x00   // 00------ (packet sent / payload ready)
#define MAP_DIO1_FSK_NOP 0x30     // --11----
#define MAP_DIO2_FSK_TXNOP 0x04   // ----01--
#define MAP_DIO2_FSK_TIMEOUT 0x08 // ----10--

#define RF_MID_BAND_THRESHOLD 525E6
#define RSSI_OFFSET_HF_PORT 157
#define RSSI_OFFSET_LF_PORT 164

#define MAX_PKT_LENGTH 255

#if (ESP8266 || ESP32)
#define ISR_PREFIX ICACHE_RAM_ATTR
#else
#define ISR_PREFIX
#endif

/**
 * dereffered service intterupt task, necessary to use mutexes in combination with interrupts
 * function is started on interrupt and function sets itself to sleep if interrupt task is processed.
 */
void isrTask(void *parameter)
{
  //Serial.println("Startup+");
  //uint32_t exeFlags;
  irqData irqData;
  uint16_t exeDio0;
  uint16_t exeDio1;

  while (true)
  {
    if (uxQueueMessagesWaiting(LoRaExt[0].isrTaskQueue) == 0)
    {
      vTaskSuspend(LoRaExt[0].isrServiceTaskHndl);
    }
    else
    {
      xQueueReceive(LoRaExt[0].isrTaskQueue, &irqData, portMAX_DELAY);

      exeDio0 = irqData.exeFlags & 0xFFFF;
      exeDio1 = (irqData.exeFlags & 0xFFFF0000) >> 16;

      if (exeDio0 != 0)
      {
        exeDio0 -= 1;
        LoRaExt[exeDio0].handleDio0Rise(irqData.timestamp, irqData.pinDio0, irqData.pinDio1);
        exeDio0 = 0;
      }

      if (exeDio1 != 0)
      {
        exeDio1 -= 1;
        LoRaExt[exeDio1].handleDio0Rise(irqData.timestamp, irqData.pinDio0, irqData.pinDio1);
        exeDio1 = 0;
      }
    }
  }
  vTaskDelete(NULL);
}

int LoRaModemSX127x::gbl_instance = 0;

SemaphoreHandle_t LoRaModemSX127x::spiHndlMutex = xSemaphoreCreateMutex();

/**
* @brief constructor for LoRaModemSX127x, setup the instance number.
*
* @exceptsafe This function does not throw exceptions.
*/
LoRaModemSX127x::LoRaModemSX127x() : _spiSettings(LORA_DEFAULT_SPI_FREQUENCY, MSBFIRST, SPI_MODE0),
                                     _spi(&LORA_DEFAULT_SPI),
                                     _ss(LORA_DEFAULT_SS_PIN), _reset(LORA_DEFAULT_RESET_PIN), _dio0(LORA_DEFAULT_DIO0_PIN),
                                     _implicitHeaderMode(0),
                                     _onReceive(NULL),
                                     _onTxDone(NULL),
                                     _onCadDone(NULL),
                                     _onCadDetected(NULL),
                                     _onRxTimeout(NULL)

{

  _regOpMode = MODE_LONG_RANGE_MODE;

  //reset all pending interrupts
  writeRegister(REG_IRQ_FLAGS, 0xFF, true);

  instance = gbl_instance;

  if (instance == 0)
  {
    //einhängen der isrTask

    xTaskCreatePinnedToCore(
        isrTask,             /* Task function. */
        "isrTask",           /* name of task. */
        10000,               /* Stack size of task */
        NULL,                /* parameter of the task */
        4,                   /* priority of the task */
        &isrServiceTaskHndl, /* Task handle to keep track of created task */
        0);                  //CPU 0

    //aufsetzten der queue in die task
    isrTaskQueue = xQueueCreate(30, sizeof(irqData_t));
  }

  gbl_instance++;
}

/**
* @brief check the interrupt flags for cad detected request. 
*
* @param flags actual contens of the interrupt register.
*
* @return true if interrupt is requested.
*
* @exceptsafe This function does not throw exceptions.
*/
bool LoRaModemSX127x::isIrqCadDetect(uint8_t flags)
{
  if ((flags & IRQ_CD_DETD_MASK) != 0)
  {
    return true;
  }
  return false;
}

/**
* @brief check the interrupt flags for rx timeout request. 
*
* @param flags actual contens of the interrupt register.
*
* @return true if interrupt is requested.
*
* @exceptsafe This function does not throw exceptions.
*/
bool LoRaModemSX127x::isIrqRxTout(uint8_t flags)
{
  if ((flags & IRQ_RX_TOUT_MASK) != 0)
  {
    return true;
  }
  return false;
}

/**
* @brief check the interrupt flags for valid header request. 
*
* @param flags actual contens of the interrupt register.
*
* @return true if interrupt is requested.
*
* @exceptsafe This function does not throw exceptions.
*/
bool LoRaModemSX127x::isIrqValidHeader(uint8_t flags)
{
  if ((flags & IRQ_HEADER_MASK) != 0)
  {
    return true;
  }
  return false;
}

/**
* @brief check the interrupt flags for cad done request. 
*
* @param flags actual contens of the interrupt register.
*
* @return true if interrupt is requested.
*
* @exceptsafe This function does not throw exceptions.
*/
bool LoRaModemSX127x::isIrqCadDone(uint8_t flags)
{
  if ((flags & IRQ_CD_DONE_MASK) != 0)
  {
    return true;
  }
  return false;
}

/**
* @brief check the interrupt flags for Fhssch request. 
*
* @param flags actual contens of the interrupt register.
*
* @return true if interrupt is requested.
*
* @exceptsafe This function does not throw exceptions.
*/
bool LoRaModemSX127x::isIrqFhsschRequest(uint8_t flags)
{
  if ((flags & IRQ_FHSSCH_MASK) != 0)
  {
    return true;
  }
  return false;
}

/**
* @brief check the interrupt flags for crc error request. 
*
* @param flags actual contens of the interrupt register.
*
* @return true if interrupt is requested.
*
* @exceptsafe This function does not throw exceptions.
*/
bool LoRaModemSX127x::isIrqCrcError(uint8_t flags)
{
  if ((flags & IRQ_PAYLOAD_CRC_ERROR_MASK) != 0)
  {
    return true;
  }
  return false;
}

/**
* @brief check the interrupt flags for rx done request. 
*
* @param flags actual contens of the interrupt register.
*
* @return true if interrupt is requested.
*
* @exceptsafe This function does not throw exceptions.
*/
bool LoRaModemSX127x::isIrqRxDone(uint8_t flags)
{
  if ((flags & IRQ_RX_DONE_MASK) != 0)
  {
    return true;
  }
  return false;
}

/**
* @brief check the interrupt flags for tx done request. 
*
* @param flags actual contens of the interrupt register.
*
* @return true if interrupt is requested.
*
* @exceptsafe This function does not throw exceptions.
*/
bool LoRaModemSX127x::isIrqTxDone(uint8_t flags)
{
  if ((flags & IRQ_TX_DONE_MASK) != 0)
  {
    return true;
  }
  return false;
}

/**
* @brief read the modem status register. 
*
* @param takeMutex use the mutex wich blocks the spi access for other tasks, if used in critical area this has to be set to false.
*
* @return contense of modem satus register
*
* @exceptsafe This function does not throw exceptions.
*/
uint8_t LoRaModemSX127x::getModemStatusRegister(bool takeMutex)
{
  return readRegister(REG_MODEM_STAT, takeMutex);
}

/**
* @brief check the given status register if signal was detected. 
*
* @param reg_data actual contens of the status register.
*
* @return true if signal was detected.
*
* @exceptsafe This function does not throw exceptions.
*/
bool LoRaModemSX127x::isStatusSignlaDetected(uint8_t reg_data)
{
  if ((reg_data & STATUS_SIG_DETECT_MASK) != 0)
  {
    return true;
  }
  return false;
}

/**
* @brief check the given status register if signal is synchronized. 
*
* @param reg_data actual contens of the status register.
*
* @return true if signal is synchronized.
*
* @exceptsafe This function does not throw exceptions.
*/
bool LoRaModemSX127x::isStatusSignlaSynchronized(uint8_t reg_data)
{
  if ((reg_data & STATUS_SIG_SYNC_MASK) != 0)
  {
    return true;
  }
  return false;
}

/**
* @brief check the given status register if the header information is valid. 
*
* @param reg_data actual contens of the status register.
*
* @return true if header information is valid.
*
* @exceptsafe This function does not throw exceptions.
*/
bool LoRaModemSX127x::isStatusHeaderInfoValid(uint8_t reg_data)
{
  if ((reg_data & STATUS_RX_ONGOING_MASK) != 0)
  {
    return true;
  }
  return false;
}

/**
* @brief check the given status register if message receive is ongoing. 
*
* @param reg_data actual contens of the status register.
*
* @return true if message receive is ongoing.
*
* @exceptsafe This function does not throw exceptions.
*/
bool LoRaModemSX127x::isStatusRxOngoing(uint8_t reg_data)
{
  if ((reg_data & STATUS_HEADER_VALID_MASK) != 0)
  {
    return true;
  }
  return false;
}

/**
* @brief check the given status register if modem is clear to operate. 
*
* @param reg_data actual contens of the status register.
*
* @return true if modem is clear to operate.
*
* @exceptsafe This function does not throw exceptions.
*/
bool LoRaModemSX127x::isStatusModemClear(uint8_t reg_data)
{
  if ((reg_data & STATUS_MODEM_CLEAR_MASK) != 0)
  {
    return true;
  }
  return false;
}

/**
* @brief initialize modem on an specific frequency
*
* @param frequency in Hz
*
* @return nothing.
*
* @exceptsafe This function does not throw exceptions.
*/
int LoRaModemSX127x::begin(long frequency)
{
  //set mode default to lora
  _modemConfig.modulation = LoRaModulation_t::LORA;

#if defined(ARDUINO_SAMD_MKRWAN1300) || defined(ARDUINO_SAMD_MKRWAN1310)
  pinMode(LORA_IRQ_DUMB, OUTPUT);
  digitalWrite(LORA_IRQ_DUMB, LOW);

  // Hardware reset
  pinMode(LORA_BOOT0, OUTPUT);
  digitalWrite(LORA_BOOT0, LOW);

  pinMode(LORA_RESET, OUTPUT);
  digitalWrite(LORA_RESET, HIGH);
  delay(200);
  digitalWrite(LORA_RESET, LOW);
  delay(200);
  digitalWrite(LORA_RESET, HIGH);
  delay(50);
#endif

  // setup pins
  pinMode(_ss, OUTPUT);
  // set SS high
  digitalWrite(_ss, HIGH);

  if (_reset != -1)
  {
    pinMode(_reset, OUTPUT);

    // perform reset
    digitalWrite(_reset, LOW);
    delay(10);
    digitalWrite(_reset, HIGH);
    delay(10);
  }

  // start SPI
  xSemaphoreTake(spiHndlMutex, portMAX_DELAY);
  _spi->begin();
  xSemaphoreGive(spiHndlMutex);

  // check version
  uint8_t version = readRegister(REG_VERSION, true);
  if (version != 0x12)
  {
    return 0;
  }

  // put in sleep mode
  sleep(true);

  //default values from datasheet
  // set frequency
  setFrequency(frequency, true);
  setSpreadingFactor(7, true);
  setPreambleLength(8, true);
  setCodingRate4(5, true);
  setSignalBandwidth(125E3, true);
  disableCrc(true);

  // set base addresses
  writeRegister(REG_FIFO_TX_BASE_ADDR, 0, true);
  writeRegister(REG_FIFO_RX_BASE_ADDR, 0, true);

  // set LNA boost
  writeRegister(REG_LNA, readRegister(REG_LNA, true) | 0x03, true);

  // set auto AGC
  //writeRegister(REG_MODEM_CONFIG_3, 0x04, true);
  setGain(0, true);

  // set output power to 17 dBm
  setTxPower(17, true);

  // put in standby mode
  stdby(true);

  return 1;
}

/**
* @brief shut down spi bus
*
* @return nothing.
*
* @exceptsafe This function does not throw exceptions.
*/
void LoRaModemSX127x::end()
{
  // put in sleep mode
  sleep(true);

  // stop SPI
  xSemaphoreTake(spiHndlMutex, portMAX_DELAY);
  _spi->end();
  xSemaphoreGive(spiHndlMutex);
}

/**
* @brief check if transmission is ongoing.
*
* @param takeMutex use the mutex wich blocks the spi access for other tasks, if used in critical area this has to be set to false.
*
* @return true if transmission is ongoing.
*
* @exceptsafe This function does not throw exceptions.
*/
bool LoRaModemSX127x::isTransmitting(bool takeMutex)
{
  if ((readRegister(REG_OP_MODE, takeMutex) & MODE_TX) == MODE_TX)
  {
    return true;
  }

  if (readRegister(REG_IRQ_FLAGS, takeMutex) & IRQ_TX_DONE_MASK)
  {
    // clear IRQ's
    reconfigRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK, takeMutex);
  }

  return false;
}

/**
* @brief read the Received Signal Strength Indicator of the received data packet 
*
* @param takeMutex use the mutex wich blocks the spi access for other tasks, if used in critical area this has to be set to false.
*
* @return integer with the rssi value.
*
* @exceptsafe This function does not throw exceptions.
*/
int LoRaModemSX127x::packetRssi(bool takeMutex)
{
  return (readRegister(REG_PKT_RSSI_VALUE, takeMutex) - (_modemConfig.freq_hz < RF_MID_BAND_THRESHOLD ? RSSI_OFFSET_LF_PORT : RSSI_OFFSET_HF_PORT));
}

/**
* @brief read the signal to noise ratio of the received data packet 
*
* @param takeMutex use the mutex wich blocks the spi access for other tasks, if used in critical area this has to be set to false.
*
* @return float with the snr value.
*
* @exceptsafe This function does not throw exceptions.
*/
float LoRaModemSX127x::packetSnr(bool takeMutex)
{
  return ((int8_t)readRegister(REG_PKT_SNR_VALUE, takeMutex)) * 0.25;
}

/**
* @brief read the frequency error, because the crystal reference oscillator has only a finite frequency 
*
* @param takeMutex use the mutex wich blocks the spi access for other tasks, if used in critical area this has to be set to false.
*
* @return long with the computed frequency error.
*
* @exceptsafe This function does not throw exceptions.
*/
long LoRaModemSX127x::packetFrequencyError(bool takeMutex)
{
  int32_t freqError = 0;
  freqError = static_cast<int32_t>(readRegister(REG_FREQ_ERROR_MSB, takeMutex) & B111);
  freqError <<= 8L;
  freqError += static_cast<int32_t>(readRegister(REG_FREQ_ERROR_MID, takeMutex));
  freqError <<= 8L;
  freqError += static_cast<int32_t>(readRegister(REG_FREQ_ERROR_LSB, takeMutex));

  if (readRegister(REG_FREQ_ERROR_MSB, takeMutex) & B1000)
  {                      // Sign bit is on
    freqError -= 524288; // B1000'0000'0000'0000'0000
  }

  const float fXtal = 32E6;                                                                                                  // FXOSC: crystal oscillator (XTAL) frequency (2.5. Chip Specification, p. 14)
  const float fError = ((static_cast<float>(freqError) * (1L << 24)) / fXtal) * (getSignalBandwidth(takeMutex) / 500000.0f); // p. 37

  return static_cast<long>(fError);
}

/**
* @brief read the current Signal Strength Indicator over the air
*
* @param takeMutex use the mutex wich blocks the spi access for other tasks, if used in critical area this has to be set to false.
*
* @return integer with the rssi value.
*
* @exceptsafe This function does not throw exceptions.
*/
int LoRaModemSX127x::rssi(bool takeMutex)
{
  return (readRegister(REG_RSSI_VALUE, takeMutex) - (_modemConfig.freq_hz < RF_MID_BAND_THRESHOLD ? RSSI_OFFSET_LF_PORT : RSSI_OFFSET_HF_PORT));
}

/**
* @brief get the number of received bytes.
*
* @param takeMutex use the mutex wich blocks the spi access for other tasks, if used in critical area this has to be set to false.
*
* @return number of received bytes.
*
* @exceptsafe This function does not throw exceptions.
*/
int LoRaModemSX127x::recNoBytes(bool takeMutex)
{
  return (readRegister(REG_RX_NB_BYTES, takeMutex));
}

/**
* @brief read the received message from the FiFo
*
* @param buffer pointer to uint8_t buffer, where the message should be copied to 
* @param len how many bytes should be read from the FiFo
* @param takeMutex use the mutex wich blocks the spi access for other tasks, if used in critical area this has to be set to false.
*
* @return nothing.
*
* @exceptsafe This function does not throw exceptions.
*/
void LoRaModemSX127x::readMsg(uint8_t *buffer, int len, bool takeMutex)
{
  writeRegister(REG_FIFO_ADDR_PTR, LoRaExt[0].readRegister(REG_FIFO_RX_CURRENT_ADDR, takeMutex), takeMutex);
  for (int i = 0; i < len; i++)
  {
    buffer[i] = readRegister(REG_FIFO, takeMutex); // 0x00, FIFO will auto shift register
  }
}

/**
* @brief transfer payload to FiFo of modem.
*
* @param buffer source of the payload.
* @param size used size of the buffer.
* @param takeMutex use the mutex wich blocks the spi access for other tasks, if used in critical area this has to be set to false.
*
* @return nothing.
*
* @exceptsafe This function does not throw exceptions.
*/
void LoRaModemSX127x::putPayloadFiFo(uint8_t *buffer, size_t size, bool takeMutex)
{
  int currentLength = readRegister(REG_PAYLOAD_LENGTH, takeMutex);

  // check size
  if ((currentLength + size) > MAX_PKT_LENGTH)
  {
    size = MAX_PKT_LENGTH - currentLength;
  }

  // write data
  for (size_t i = 0; i < size; i++)
  {
    writeRegister(REG_FIFO, buffer[i], takeMutex);
  }

  // update length
  writeRegister(REG_PAYLOAD_LENGTH, currentLength + size, takeMutex);
}

/**
* @brief put given message on transmit FiFo.
*
* @param buffer message wich should be transmitted, pointer to uint8_t buffer 
* @param len how many bytes should be transfered.
* @param takeMutex use the mutex wich blocks the spi access for other tasks, if used in critical area this has to be set to false.
*
* @return true if operation was successfull.
*
* @exceptsafe This function does not throw exceptions.
*/
bool LoRaModemSX127x::prepareTransmitMsg(uint8_t *buffer, int len, bool takeMutex)
{

  // put in standby mode
  stdby(takeMutex);

  // put in standby mode
  stdby(takeMutex);

  if (_implicitHeaderMode)
  {
    implicitHeaderMode(takeMutex);
  }
  else
  {
    explicitHeaderMode(takeMutex);
  }

  // reset FIFO address and paload length
  reconfigRegister(REG_FIFO_ADDR_PTR, 0, takeMutex);
  reconfigRegister(REG_PAYLOAD_LENGTH, 0, takeMutex);

  //put Data in fifo
  putPayloadFiFo(buffer, len, takeMutex);

  //transmit
  IrqTxDone(takeMutex);

  return 1;
}

/**
* @brief put given message on transmit FiFo and transmit the message.
*
* @param buffer message wich should be transmitted, pointer to uint8_t buffer 
* @param len how many bytes should be transfered.
* @async if function is blocking until the message was sent.
* @param takeMutex use the mutex wich blocks the spi access for other tasks, if used in critical area this has to be set to false.
*
* @return true if operation was successfull.
*
* @exceptsafe This function does not throw exceptions.
*/
bool LoRaModemSX127x::transmitMsg(uint8_t *buffer, int len, bool async, bool takeMutex)
{
  if (isTransmitting(takeMutex))
  {
    return 0;
  }

  // put in standby mode
  stdby(takeMutex);

  if (_implicitHeaderMode)
  {
    implicitHeaderMode(takeMutex);
  }
  else
  {
    explicitHeaderMode(takeMutex);
  }

  // reset FIFO address and paload length
  reconfigRegister(REG_FIFO_ADDR_PTR, 0, takeMutex);
  reconfigRegister(REG_PAYLOAD_LENGTH, 0, takeMutex);

  //put Data in fifo
  putPayloadFiFo(buffer, len, takeMutex);

  //transmit
  IrqTxDone(takeMutex);
  transmit(takeMutex);

  if (!async)
  {
    // wait for TX done
    while ((readRegister(REG_IRQ_FLAGS, takeMutex) & IRQ_TX_DONE_MASK) == 0)
    {
      yield();
    }
    // clear IRQ's
    reconfigRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK, takeMutex);
  }

  return 1;
}

#ifndef ARDUINO_SAMD_MKRWAN1300
////////////////////////////////////////////////

/// @brief enable interrupt on Dio0
void LoRaModemSX127x::_enableDio0Irq()
{
  switch (instance)
  {
  case 0:
    attachInterrupt(digitalPinToInterrupt(_dio0), LoRaModemSX127x::Instance1onDio0Rise, RISING);
    break;
  case 1:
    attachInterrupt(digitalPinToInterrupt(_dio0), LoRaModemSX127x::Instance2onDio0Rise, RISING);
    break;
  case 2:
    attachInterrupt(digitalPinToInterrupt(_dio0), LoRaModemSX127x::Instance3onDio0Rise, RISING);
    break;
  case 3:
    attachInterrupt(digitalPinToInterrupt(_dio0), LoRaModemSX127x::Instance4onDio0Rise, RISING);
    break;
  case 4:
    attachInterrupt(digitalPinToInterrupt(_dio0), LoRaModemSX127x::Instance5onDio0Rise, RISING);
    break;
  case 5:
    attachInterrupt(digitalPinToInterrupt(_dio0), LoRaModemSX127x::Instance6onDio0Rise, RISING);
    break;
  case 6:
    attachInterrupt(digitalPinToInterrupt(_dio0), LoRaModemSX127x::Instance7onDio0Rise, RISING);
    break;
  case 7:
    attachInterrupt(digitalPinToInterrupt(_dio0), LoRaModemSX127x::Instance8onDio0Rise, RISING);
    break;
  case 8:
    attachInterrupt(digitalPinToInterrupt(_dio0), LoRaModemSX127x::Instance9onDio0Rise, RISING);
    break;
  case 9:
    attachInterrupt(digitalPinToInterrupt(_dio0), LoRaModemSX127x::Instance10onDio0Rise, RISING);
    break;
  case 10:
    attachInterrupt(digitalPinToInterrupt(_dio0), LoRaModemSX127x::Instance11onDio0Rise, RISING);
    break;
  case 11:
    attachInterrupt(digitalPinToInterrupt(_dio0), LoRaModemSX127x::Instance12onDio0Rise, RISING);
    break;

  default:
    break;
  }
}

/**
* @brief register callback on the event of received message.
*
* @param callback function to be called on event
*
* @return nothing
*
* @exceptsafe This function does not throw exceptions.
*/
void LoRaModemSX127x::onReceive(void (*callback)(int, int64_t, uint32_t, uint8_t))
{
  _onReceive = callback;

  if (callback)
  {
    pinMode(_dio0, INPUT);
#ifdef SPI_HAS_NOTUSINGINTERRUPT
    xSemaphoreTake(spiHndlMutex, portMAX_DELAY);
    SPI.usingInterrupt(digitalPinToInterrupt(_dio0));
    xSemaphoreTake(spiHndlMutex, portMAX_DELAY);
#endif
    _enableDio0Irq();
  }
  else
  {
    detachInterrupt(digitalPinToInterrupt(_dio0));
#ifdef SPI_HAS_NOTUSINGINTERRUPT
    xSemaphoreTake(spiHndlMutex, portMAX_DELAY);
    SPI.notUsingInterrupt(digitalPinToInterrupt(_dio0));
    xSemaphoreTake(spiHndlMutex, portMAX_DELAY);
#endif
  }
}

/**
* @brief register callback on the event of transmission completed.
*
* @param callback function to be called on event
*
* @return nothing
*
* @exceptsafe This function does not throw exceptions.
*/
void LoRaModemSX127x::onTxDone(void (*callback)(int, int64_t))
{
  _onTxDone = callback;

  if (callback)
  {
    pinMode(_dio0, INPUT);
#ifdef SPI_HAS_NOTUSINGINTERRUPT
    xSemaphoreTake(spiHndlMutex, portMAX_DELAY);
    SPI.usingInterrupt(digitalPinToInterrupt(_dio0));
    xSemaphoreTake(spiHndlMutex, portMAX_DELAY);
#endif
    _enableDio0Irq();
  }
  else
  {
    detachInterrupt(digitalPinToInterrupt(_dio0));
#ifdef SPI_HAS_NOTUSINGINTERRUPT
    xSemaphoreTake(spiHndlMutex, portMAX_DELAY);
    SPI.notUsingInterrupt(digitalPinToInterrupt(_dio0));
    xSemaphoreGive(spiHndlMutex);
#endif
  }
}

/**
* @brief register callback on the event of channel activity detection completed.
*
* @param callback function to be called on event
*
* @return nothing
*
* @exceptsafe This function does not throw exceptions.
*/
void LoRaModemSX127x::onCadDone(void (*callback)(int, int64_t, uint8_t))
{
  _onCadDone = callback;

  if (callback)
  {
    pinMode(_dio0, INPUT);
#ifdef SPI_HAS_NOTUSINGINTERRUPT
    xSemaphoreTake(spiHndlMutex, portMAX_DELAY);
    SPI.usingInterrupt(digitalPinToInterrupt(_dio0));
    xSemaphoreGive(spiHndlMutex);
#endif
    _enableDio0Irq();
  }
  else
  {
    detachInterrupt(digitalPinToInterrupt(_dio0));
#ifdef SPI_HAS_NOTUSINGINTERRUPT
    xSemaphoreTake(spiHndlMutex, portMAX_DELAY);
    SPI.notUsingInterrupt(digitalPinToInterrupt(_dio0));
    xSemaphoreGive(spiHndlMutex);
#endif
  }
}

/// @brief enable interrupt on Dio1
void LoRaModemSX127x::_enableDio1Irq()
{
  switch (instance)
  {
  case 0:
    attachInterrupt(digitalPinToInterrupt(_dio1), LoRaModemSX127x::Instance1onDio1Rise, RISING);
    break;
  case 1:
    attachInterrupt(digitalPinToInterrupt(_dio1), LoRaModemSX127x::Instance2onDio1Rise, RISING);
    break;
  case 2:
    attachInterrupt(digitalPinToInterrupt(_dio1), LoRaModemSX127x::Instance3onDio1Rise, RISING);
    break;
  case 3:
    attachInterrupt(digitalPinToInterrupt(_dio1), LoRaModemSX127x::Instance4onDio1Rise, RISING);
    break;
  case 4:
    attachInterrupt(digitalPinToInterrupt(_dio1), LoRaModemSX127x::Instance5onDio1Rise, RISING);
    break;
  case 5:
    attachInterrupt(digitalPinToInterrupt(_dio1), LoRaModemSX127x::Instance6onDio1Rise, RISING);
    break;
  case 6:
    attachInterrupt(digitalPinToInterrupt(_dio1), LoRaModemSX127x::Instance7onDio1Rise, RISING);
    break;
  case 7:
    attachInterrupt(digitalPinToInterrupt(_dio1), LoRaModemSX127x::Instance8onDio1Rise, RISING);
    break;
  case 8:
    attachInterrupt(digitalPinToInterrupt(_dio1), LoRaModemSX127x::Instance9onDio1Rise, RISING);
    break;
  case 9:
    attachInterrupt(digitalPinToInterrupt(_dio1), LoRaModemSX127x::Instance10onDio1Rise, RISING);
    break;
  case 10:
    attachInterrupt(digitalPinToInterrupt(_dio1), LoRaModemSX127x::Instance11onDio1Rise, RISING);
    break;
  case 11:
    attachInterrupt(digitalPinToInterrupt(_dio1), LoRaModemSX127x::Instance12onDio1Rise, RISING);
    break;

  default:
    break;
  }
}

/**
* @brief register callback on the event of channel activity detected.
*
* @param callback function to be called on event
*
* @return nothing
*
* @exceptsafe This function does not throw exceptions.
*/
void LoRaModemSX127x::onCadDetected(void (*callback)(int, int64_t))
{
  _onCadDetected = callback;

  if (callback)
  {
    pinMode(_dio1, INPUT);
#ifdef SPI_HAS_NOTUSINGINTERRUPT
    xSemaphoreTake(spiHndlMutex, portMAX_DELAY);
    SPI.usingInterrupt(digitalPinToInterrupt(_dio1));
    xSemaphoreGive(spiHndlMutex);
#endif
    _enableDio1Irq();
  }
  else
  {
    detachInterrupt(digitalPinToInterrupt(_dio1));
#ifdef SPI_HAS_NOTUSINGINTERRUPT
    xSemaphoreTake(spiHndlMutex, portMAX_DELAY);
    SPI.notUsingInterrupt(digitalPinToInterrupt(_dio1));
    xSemaphoreGive(spiHndlMutex);
#endif
  }
}

/**
* @brief register callback on the event of received message with valid header.
*
* @param callback function to be called on event
*
* @return nothing
*
* @exceptsafe This function does not throw exceptions.
*/
void LoRaModemSX127x::onValidHeader(void (*callback)(int, int64_t))
{
  _onValidHeader = callback;
}

/**
* @brief register callback on the event of received timeout.
*
* @param callback function to be called on event
*
* @return nothing
*
* @exceptsafe This function does not throw exceptions.
*/
void LoRaModemSX127x::onRxTimeout(void (*callback)(int, int64_t))
{
  _onRxTimeout = callback;

  if (callback)
  {
    pinMode(_dio1, INPUT);
#ifdef SPI_HAS_NOTUSINGINTERRUPT
    xSemaphoreTake(spiHndlMutex, portMAX_DELAY);
    SPI.usingInterrupt(digitalPinToInterrupt(_dio1));
    xSemaphoreGive(spiHndlMutex);
#endif
    _enableDio1Irq();
  }

  else
  {
    detachInterrupt(digitalPinToInterrupt(_dio1));
#ifdef SPI_HAS_NOTUSINGINTERRUPT
    xSemaphoreTake(spiHndlMutex, portMAX_DELAY);
    SPI.notUsingInterrupt(digitalPinToInterrupt(_dio1));
    xSemaphoreGive(spiHndlMutex);
#endif
  }
}

#endif

/// set interrupt pins to rx done
void LoRaModemSX127x::IrqRxDone(bool takeMutex)
{
  writeRegister(REG_IRQ_FLAGS, 0xFF, takeMutex);

  if (_dio0IrqMode != MAP_DIO0_LORA_RXDONE)
  {
    reconfigRegister(REG_DIO_MAPPING_1, (MAP_DIO0_LORA_RXDONE | _dio1IrqMode | _dio2IrqMode | _dio3IrqMode), takeMutex); // DIO0 => RXDONE
    _dio0IrqMode = MAP_DIO0_LORA_RXDONE;
  }
}

/// set interrupt pins to tx done
void LoRaModemSX127x::IrqTxDone(bool takeMutex)
{
  writeRegister(REG_IRQ_FLAGS, 0xFF, takeMutex);

  if (_dio0IrqMode != MAP_DIO0_LORA_TXDONE)
  {
    reconfigRegister(REG_DIO_MAPPING_1, (MAP_DIO0_LORA_TXDONE | _dio1IrqMode | _dio2IrqMode | _dio3IrqMode), takeMutex); // DIO0 => TXDONE
    _dio0IrqMode = MAP_DIO0_LORA_TXDONE;
  }
}

/// set interrupt pins to cad done
void LoRaModemSX127x::IrqCadDone(bool takeMutex)
{
  writeRegister(REG_IRQ_FLAGS, 0xFF, takeMutex);

  if (_dio0IrqMode != MAP_DIO0_LORA_CADDONE)
  {
    reconfigRegister(REG_DIO_MAPPING_1, (MAP_DIO0_LORA_CADDONE | _dio1IrqMode | _dio2IrqMode | _dio3IrqMode), takeMutex); // DIO0 => CADDONE
    _dio0IrqMode = MAP_DIO0_LORA_CADDONE;
  }
}

/// set interrupt pins to cad detect
void LoRaModemSX127x::IrqCadDetect(bool takeMutex)
{
  writeRegister(REG_IRQ_FLAGS, 0xFF, takeMutex);

  if (_dio1IrqMode != MAP_DIO1_LORA_CADDETECT)
  {
    reconfigRegister(REG_DIO_MAPPING_1, (_dio0IrqMode | MAP_DIO1_LORA_CADDETECT | _dio2IrqMode | _dio3IrqMode), takeMutex); // DIO1 => CADDETECT
    _dio1IrqMode = MAP_DIO1_LORA_CADDETECT;
  }
}

/// set interrupt pins to rx timeout
void LoRaModemSX127x::IrqRxTimeout(bool takeMutex)
{
  writeRegister(REG_IRQ_FLAGS, 0xFF, takeMutex);

  if (_dio1IrqMode != MAP_DIO1_LORA_RXTOUT)
  {
    reconfigRegister(REG_DIO_MAPPING_1, (_dio0IrqMode | MAP_DIO1_LORA_RXTOUT | _dio2IrqMode | _dio3IrqMode), takeMutex); // DIO1 => RXTIMEOUT
    _dio1IrqMode = MAP_DIO1_LORA_RXTOUT;
  }
}

/**
* @brief set the modem in channel activiy detection mode, including the configuration of the interrupts.
*
* @param takeMutex use the mutex wich blocks the spi access for other tasks, if used in critical area this has to be set to false.
*
* @return nothing.
*
* @exceptsafe This function does not throw exceptions.
*/
void LoRaModemSX127x::cadScan(bool takeMutex)
{
  IrqCadDetect(takeMutex);
  //IrqRxTimeout(takeMutex);
  IrqCadDone(takeMutex);
  cad(takeMutex);
}

/**
* @brief set the modem in receive continous mode, including the configuration of the interrupts function is blocking if size != 0, blocks how long it takes to recieve the requested data.
*
* @param takeMutex use the mutex wich blocks the spi access for other tasks, if used in critical area this has to be set to false.
* @param size how many bytes should be read
*
* @return nothing.
*
* @exceptsafe This function does not throw exceptions.
*/
void LoRaModemSX127x::recContinuous(int size, bool takeMutex)
{
  _modemConfig.mode = ModemMode_t::RXCONTINUOUS;
  if (size > 0)
  {
    implicitHeaderMode(takeMutex);
    reconfigRegister(REG_PAYLOAD_LENGTH, size & 0xff, takeMutex);
  }
  else
  {
    explicitHeaderMode(takeMutex);
  }

  IrqRxDone(takeMutex);
  IrqRxTimeout(takeMutex);

  //gesetzten Modes ausschalten
  _regOpMode = _regOpMode & 0xF8;
  //cadmode einschalten
  _regOpMode = _regOpMode | MODE_RX_CONTINUOUS;
  reconfigRegister(REG_OP_MODE, _regOpMode, takeMutex);
}

/**
* @brief set the modem in receive single message mode, including the configuration of the interrupts
*
* @param takeMutex use the mutex wich blocks the spi access for other tasks, if used in critical area this has to be set to false.
*
* @return nothing.
*
* @exceptsafe This function does not throw exceptions.
*/
void LoRaModemSX127x::recSingleMsg(bool takeMutex)
{
  _modemConfig.mode = ModemMode_t::RXSINGLE;

  //alle änderungen dürfen nur über stby oder sleep gesetzt werden
  cfg_stdby(takeMutex);

  //interrupt Rx done setzten
  writeRegister(REG_DIO_MAPPING_1, (MAP_DIO0_LORA_RXDONE | _dio1IrqMode | _dio2IrqMode | _dio3IrqMode), takeMutex); // DIO0 => RXDONE
  _dio0IrqMode = MAP_DIO0_LORA_RXDONE;

  //interrupt Rx timeout setzten
  writeRegister(REG_DIO_MAPPING_1, (_dio0IrqMode | MAP_DIO1_LORA_RXTOUT | _dio2IrqMode | _dio3IrqMode), takeMutex); // DIO1 => RXTIMEOUT
  _dio1IrqMode = MAP_DIO1_LORA_RXTOUT;

  //zurücksetzten aller interrupt flags
  //writeRegister(REG_IRQ_FLAGS, 0xFF, takeMutex);

  //gesetzten Modes ausschalten
  _regOpMode = _regOpMode & 0xF8;
  //cadmode einschalten
  _regOpMode = _regOpMode | MODE_RX_SINGLE;

  //Serial.println(_regOpMode);

  writeRegister(REG_OP_MODE, _regOpMode, takeMutex);
}

/**
* @brief set the modem in transmit mode.
*
* @param takeMutex use the mutex wich blocks the spi access for other tasks, if used in critical area this has to be set to false.
*
* @return nothing.
*
* @exceptsafe This function does not throw exceptions.
*/
void LoRaModemSX127x::transmit(bool takeMutex)
{
  _modemConfig.mode = ModemMode_t::M_TX;
  _regOpMode = _regOpMode & 0xF8;
  _regOpMode = _regOpMode | MODE_TX;
  writeRegister(REG_OP_MODE, _regOpMode, takeMutex);
}

/**
* @brief set the modem in standby mode, without changing the last set mode, helper function to reconfigure modem registers 
*
* @param takeMutex use the mutex wich blocks the spi access for other tasks, if used in critical area this has to be set to false.
*
* @return nothing.
*
* @exceptsafe This function does not throw exceptions.
*/
void LoRaModemSX127x::cfg_stdby(bool takeMutex)
{
  uint8_t temp = _regOpMode & 0xF8;
  temp = temp | MODE_STDBY;
  writeRegister(REG_OP_MODE, temp, takeMutex);
}

/**
* @brief set the modem in standby mode
*
* @param takeMutex use the mutex wich blocks the spi access for other tasks, if used in critical area this has to be set to false.
*
* @return nothing.
*
* @exceptsafe This function does not throw exceptions.
*/
void LoRaModemSX127x::stdby(bool takeMutex)
{
  _modemConfig.mode = ModemMode_t::STBY;
  _regOpMode = _regOpMode & 0xF8;
  _regOpMode = _regOpMode | MODE_STDBY;
  writeRegister(REG_OP_MODE, _regOpMode, takeMutex);
}

/**
* @brief set the modem in sleep mode
*
* @param takeMutex use the mutex wich blocks the spi access for other tasks, if used in critical area this has to be set to false.
*
* @return nothing.
*
* @exceptsafe This function does not throw exceptions.
*/
void LoRaModemSX127x::sleep(bool takeMutex)
{
  _modemConfig.mode = ModemMode_t::SLEEP;
  _regOpMode = _regOpMode & 0xF8;
  _regOpMode = _regOpMode | MODE_SLEEP;
  writeRegister(REG_OP_MODE, _regOpMode, takeMutex);
}

/**
* @brief set the modem in channel activity detection mode.
*
* @param takeMutex use the mutex wich blocks the spi access for other tasks, if used in critical area this has to be set to false.
*
* @return nothing.
*
* @exceptsafe This function does not throw exceptions.
*/
void LoRaModemSX127x::cad(bool takeMutex)
{
  _modemConfig.mode = ModemMode_t::CAD;
  _regOpMode = _regOpMode & 0xF8;
  _regOpMode = _regOpMode | MODE_CAD;
  reconfigRegister(REG_OP_MODE, _regOpMode, takeMutex);
}

/**
* @brief configure modem mode by ModeModem_t enum.
*
* @param mode enum entry to set the modem mode.
* @param takeMutex use the mutex wich blocks the spi access for other tasks, if used in critical area this has to be set to false.
*
* @return nothing.
*
* @exceptsafe This function does not throw exceptions.
*/
void LoRaModemSX127x::setModemMode(ModemMode_t mode, bool takeMutex)
{
  switch (mode)
  {
  case ModemMode_t::M_TX:
    transmit(takeMutex);
    break;
  case ModemMode_t::CAD:
    cad(takeMutex);
    break;
  case ModemMode_t::SLEEP:
    sleep(takeMutex);
    break;
  case ModemMode_t::STBY:
    stdby(takeMutex);
    break;

  default:
    break;
  }
}

/**
* @brief set transmit power.
*
* @param level level of the transmit power, in dBm, 0 = auto adjust by modem.
* @param takeMutex use the mutex wich blocks the spi access for other tasks, if used in critical area this has to be set to false.
* @param outputPin addition hardware needed for the operation over 17dBm, see datasheet section 3.4.
*
* @return nothing.
*
* @exceptsafe This function does not throw exceptions.
*/
void LoRaModemSX127x::setTxPower(int level, bool takeMutex, int outputPin)
{
  _modemConfig.tx_power = level;

  if (PA_OUTPUT_RFO_PIN == outputPin)
  {
    // RFO
    if (level < 0)
    {
      level = 0;
    }
    else if (level > 14)
    {
      level = 14;
    }

    //Serial.println(takeMutex);
    reconfigRegister(REG_PA_CONFIG, 0x70 | level, takeMutex);
  }
  else
  {
    // PA BOOST
    if (level > 17)
    {
      if (level > 20)
      {
        level = 20;
      }

      // subtract 3 from level, so 18 - 20 maps to 15 - 17
      level -= 3;

      // High Power +20 dBm Operation (Semtech SX1276/77/78/79 5.4.3.)
      reconfigRegister(REG_PA_DAC, 0x87, takeMutex);
      setOCP(140, takeMutex);
    }
    else
    {
      if (level < 2)
      {
        level = 2;
      }
      //Default value PA_HF/LF or +17dBm
      reconfigRegister(REG_PA_DAC, 0x84, takeMutex);
      setOCP(100, takeMutex);
    }
    reconfigRegister(REG_PA_CONFIG, PA_BOOST | (level - 2), takeMutex);
  }
}

/**
* @brief set frequency.
*
* @param frequency of the channel in Hz.
* @param takeMutex use the mutex wich blocks the spi access for other tasks, if used in critical area this has to be set to false.
*
* @return nothing.
*
* @exceptsafe This function does not throw exceptions.
*/
void LoRaModemSX127x::setFrequency(long frequency, bool takeMutex)
{
  _modemConfig.freq_hz = frequency;

  uint64_t frf = ((uint64_t)frequency << 19) / 32000000;

  reconfigRegister(REG_FRF_MSB, (uint8_t)(frf >> 16), takeMutex);
  reconfigRegister(REG_FRF_MID, (uint8_t)(frf >> 8), takeMutex);
  reconfigRegister(REG_FRF_LSB, (uint8_t)(frf >> 0), takeMutex);

  //Section 4.3

  if (frequency < 779E6)
  {
    //low frequency flag setzen
    _regOpMode = _regOpMode | 0x08;
  }
  else
  {
    //high frequency flag setzten
    _regOpMode = _regOpMode & 0xF7;
  }

  reconfigRegister(REG_OP_MODE, _regOpMode, takeMutex);
}

/**
* @brief get the currently configured spreading factor.
*
* @param takeMutex use the mutex wich blocks the spi access for other tasks, if used in critical area this has to be set to false.
*
* @return integer representation of the spreading factor.
*
* @exceptsafe This function does not throw exceptions.
*/
int LoRaModemSX127x::getSpreadingFactor(bool takeMutex)
{
  return readRegister(REG_MODEM_CONFIG_2, takeMutex) >> 4;
}

/**
* @brief set spreading factor.
*
* @param spreading factor from SF 6-12.
* @param takeMutex use the mutex wich blocks the spi access for other tasks, if used in critical area this has to be set to false.
*
* @return nothing.
*
* @exceptsafe This function does not throw exceptions.
*/
void LoRaModemSX127x::setSpreadingFactor(int sf, bool takeMutex)
{
  _modemConfig.spreadFact = sf;

  if (sf < 6)
  {
    sf = 6;
  }
  else if (sf > 12)
  {
    sf = 12;
  }

  if (sf == 6)
  {
    reconfigRegister(REG_DETECTION_OPTIMIZE, 0xc5, takeMutex);
    reconfigRegister(REG_DETECTION_THRESHOLD, 0x0c, takeMutex);
  }
  else
  {
    reconfigRegister(REG_DETECTION_OPTIMIZE, 0xc3, takeMutex);
    reconfigRegister(REG_DETECTION_THRESHOLD, 0x0a, takeMutex);
  }

  reconfigRegister(REG_MODEM_CONFIG_2, (readRegister(REG_MODEM_CONFIG_2, takeMutex) & 0x0f) | ((sf << 4) & 0xf0), takeMutex);
  setLdoFlag(takeMutex);

  uint8_t timeout = 10;

  switch (sf)
  {
  case 6:
    timeout = 12;
    break;
  case 7:
    timeout = 11;
    break;
  case 8:
    timeout = 7;
    break;
  case 9:
    timeout = 6;
    break;
  case 10:
    timeout = 5;
    break;
  case 11:
    timeout = 4;
    break;
  case 12:
    timeout = 4;
    break;
  }

  setSymbolTimeout(timeout, takeMutex);
}

/**
* @brief set symbol timeout. Timeout is active if the modem is in single rx mode.
*
* @param timeout defines the timeout in symbols-time.
* @param takeMutex use the mutex wich blocks the spi access for other tasks, if used in critical area this has to be set to false.
*
* @return nothing.
*
* @exceptsafe This function does not throw exceptions.
*/
void LoRaModemSX127x::setSymbolTimeout(uint8_t timeout, bool takeMutex)
{
  _modemConfig.symbolTimeout = timeout;
  reconfigRegister(REG_SYMB_TIMEOUT_LSB, timeout, takeMutex);
}

/**
* @brief get the currently configured signal bandwidth.
*
* @param takeMutex use the mutex wich blocks the spi access for other tasks, if used in critical area this has to be set to false.
*
* @return long representation of the bandwidth in Hz.
*
* @exceptsafe This function does not throw exceptions.
*/
long LoRaModemSX127x::getSignalBandwidth(bool takeMutex)
{
  byte bw = (readRegister(REG_MODEM_CONFIG_1, takeMutex) >> 4);

  switch (bw)
  {
  case 0:
    return 7.8E3;
  case 1:
    return 10.4E3;
  case 2:
    return 15.6E3;
  case 3:
    return 20.8E3;
  case 4:
    return 31.25E3;
  case 5:
    return 41.7E3;
  case 6:
    return 62.5E3;
  case 7:
    return 125E3;
  case 8:
    return 250E3;
  case 9:
    return 500E3;
  }

  return -1;
}

/**
* @brief set signal bandwidth.
*
* @param sbw signal bandwidth in Hz.
* @param takeMutex use the mutex wich blocks the spi access for other tasks, if used in critical area this has to be set to false.
*
* @return nothing.
*
* @exceptsafe This function does not throw exceptions.
*/
void LoRaModemSX127x::setSignalBandwidth(long sbw, bool takeMutex)
{
  _modemConfig.bandwidth_hz = sbw;

  int bw;

  if (sbw <= 7.8E3)
  {
    bw = 0;
  }
  else if (sbw <= 10.4E3)
  {
    bw = 1;
  }
  else if (sbw <= 15.6E3)
  {
    bw = 2;
  }
  else if (sbw <= 20.8E3)
  {
    bw = 3;
  }
  else if (sbw <= 31.25E3)
  {
    bw = 4;
  }
  else if (sbw <= 41.7E3)
  {
    bw = 5;
  }
  else if (sbw <= 62.5E3)
  {
    bw = 6;
  }
  else if (sbw <= 125E3)
  {
    bw = 7;
  }
  else if (sbw <= 250E3)
  {
    bw = 8;
  }
  else /*if (sbw <= 250E3)*/
  {
    bw = 9;
  }

  reconfigRegister(REG_MODEM_CONFIG_1, (readRegister(REG_MODEM_CONFIG_1, takeMutex) & 0x0f) | (bw << 4), takeMutex);
  setLdoFlag(takeMutex);
}

/**
* @brief set the low data rate optimization.
*
* @param takeMutex use the mutex wich blocks the spi access for other tasks, if used in critical area this has to be set to false.
*
* @return true if transmission is ongoing.
*
* @exceptsafe This function does not throw exceptions.
*/
void LoRaModemSX127x::setLdoFlag(bool takeMutex)
{
  // Section 4.1.1.5
  long symbolDuration = 1000 / (getSignalBandwidth(takeMutex) / (1L << getSpreadingFactor(takeMutex)));

  // Section 4.1.1.6
  boolean ldoOn = symbolDuration > 16;

  uint8_t config3 = readRegister(REG_MODEM_CONFIG_3, takeMutex);
  bitWrite(config3, 3, ldoOn);
  reconfigRegister(REG_MODEM_CONFIG_3, config3, takeMutex);
}

/**
* @brief set signal coding rate.
*
* @param denominator, valid values 5-8, represents 4/5, 4/6, 4/7, 4/8
* @param takeMutex use the mutex wich blocks the spi access for other tasks, if used in critical area this has to be set to false.
*
* @return nothing.
*
* @exceptsafe This function does not throw exceptions.
*/
void LoRaModemSX127x::setCodingRate4(int denominator, bool takeMutex)
{
  _modemConfig.codingRate = denominator;

  if (denominator < 5)
  {
    denominator = 5;
  }
  else if (denominator > 8)
  {
    denominator = 8;
  }

  int cr = denominator - 4;

  reconfigRegister(REG_MODEM_CONFIG_1, (readRegister(REG_MODEM_CONFIG_1, takeMutex) & 0xf1) | (cr << 1), takeMutex);
}

/**
* @brief set preamble length.
*
* @param length length of the preamble, number of symbols the receiver hsa to synchronize to the signal.
* @param takeMutex use the mutex wich blocks the spi access for other tasks, if used in critical area this has to be set to false.
*
* @return nothing.
*
* @exceptsafe This function does not throw exceptions.
*/
void LoRaModemSX127x::setPreambleLength(uint32_t length, bool takeMutex)
{
  _modemConfig.preambleLength = length;

  reconfigRegister(REG_PREAMBLE_MSB, (uint8_t)(length >> 8), takeMutex);
  reconfigRegister(REG_PREAMBLE_LSB, (uint8_t)(length >> 0), takeMutex);
}

/**
* @brief set syncword.
*
* @param sw syncword 0-255.
* @param takeMutex use the mutex wich blocks the spi access for other tasks, if used in critical area this has to be set to false.
*
* @return nothing.
*
* @exceptsafe This function does not throw exceptions.
*/
void LoRaModemSX127x::setSyncWord(int sw, bool takeMutex)
{
  _modemConfig.syncWord = sw;

  reconfigRegister(REG_SYNC_WORD, sw, takeMutex);
}

/**
* @brief enable crc.
*
* @param takeMutex use the mutex wich blocks the spi access for other tasks, if used in critical area this has to be set to false.
*
* @return nothing.
*
* @exceptsafe This function does not throw exceptions.
*/
void LoRaModemSX127x::enableCrc(bool takeMutex)
{
  _modemConfig.crc = true;
  reconfigRegister(REG_MODEM_CONFIG_2, readRegister(REG_MODEM_CONFIG_2, takeMutex) | 0x04, takeMutex);
}

/**
* @brief disable crc.
*
* @param takeMutex use the mutex wich blocks the spi access for other tasks, if used in critical area this has to be set to false.
*
* @return nothing.
*
* @exceptsafe This function does not throw exceptions.
*/
void LoRaModemSX127x::disableCrc(bool takeMutex)
{
  _modemConfig.crc = false;
  reconfigRegister(REG_MODEM_CONFIG_2, readRegister(REG_MODEM_CONFIG_2, takeMutex) & 0xfb, takeMutex);
}

/**
* @brief enable or disable crc.
*
* @param mode true to enable crc, false to disable crc.
* @param takeMutex use the mutex wich blocks the spi access for other tasks, if used in critical area this has to be set to false.
*
* @return nothing.
*
* @exceptsafe This function does not throw exceptions.
*/
void LoRaModemSX127x::setCrc(bool mode, bool takeMutex)
{
  if (mode)
  {
    enableCrc(takeMutex);
  }
  else
  {
    disableCrc(takeMutex);
  }
}

/**
* @brief enable or disable inversion of polarity (I and Q path inversion).
*
* @param mode true to enable crc, false to disable crc.
* @param takeMutex use the mutex wich blocks the spi access for other tasks, if used in critical area this has to be set to false.
*
* @return nothing.
*
* @exceptsafe This function does not throw exceptions.
*/
void LoRaModemSX127x::setInvertPolarity(bool mode, bool takeMutex)
{
  if (mode)
  {
    enableInvertPolarity(takeMutex);
  }
  else
  {
    disableInvertPolarity(takeMutex);
  }
}

/**
* @brief disable inversion of polarity (I and Q path inversion).
*
* @param takeMutex use the mutex wich blocks the spi access for other tasks, if used in critical area this has to be set to false.
*
* @return nothing.
*
* @exceptsafe This function does not throw exceptions.
*/
void LoRaModemSX127x::disableInvertPolarity(bool takeMutex)
{
  _modemConfig.invertPolarity = false;
  enableInvertIqTx(takeMutex);
  disableInvertIqRx(takeMutex);
  disableInvertIq(takeMutex);
}

/**
* @brief enable inversion of polarity (I and Q path inversion).
*
* @param takeMutex use the mutex wich blocks the spi access for other tasks, if used in critical area this has to be set to false.
*
* @return nothing.
*
* @exceptsafe This function does not throw exceptions.
*/
void LoRaModemSX127x::enableInvertPolarity(bool takeMutex)
{
  _modemConfig.invertPolarity = true;
  disableInvertIqTx(takeMutex);
  enableInvertIqRx(takeMutex);
  disableInvertIq(takeMutex);
}

/**
* @brief disable inversion of polarity (I and Q path inversion) in Rx path. Activation of configuration needs @see enableInvertIq.
*
* @param takeMutex use the mutex wich blocks the spi access for other tasks, if used in critical area this has to be set to false.
*
* @return nothing.
*
* @exceptsafe This function does not throw exceptions.
*/
void LoRaModemSX127x::enableInvertIqRx(bool takeMutex)
{
  reconfigRegister(REG_INVERTIQ, readRegister(REG_INVERTIQ, takeMutex) | 0x40, takeMutex);
}

/**
* @brief disbale inversion of polarity (I and Q path inversion) in Rx path. Deactivation of configuration needs @see disableInvertIq.
*
* @param takeMutex use the mutex wich blocks the spi access for other tasks, if used in critical area this has to be set to false.
*
* @return nothing.
*
* @exceptsafe This function does not throw exceptions.
*/
void LoRaModemSX127x::disableInvertIqRx(bool takeMutex)
{
  reconfigRegister(REG_INVERTIQ, readRegister(REG_INVERTIQ, takeMutex) & 0xBF, takeMutex);
}

/**
* @brief disable inversion of polarity (I and Q path inversion) in Tx path. Activation of configuration needs @see enableInvertIq.
*
* @param takeMutex use the mutex wich blocks the spi access for other tasks, if used in critical area this has to be set to false.
*
* @return nothing.
*
* @exceptsafe This function does not throw exceptions.
*/
void LoRaModemSX127x::enableInvertIqTx(bool takeMutex)
{
  reconfigRegister(REG_INVERTIQ, readRegister(REG_INVERTIQ, takeMutex) | 0x01, takeMutex);
}

/**
* @brief disbale inversion of polarity (I and Q path inversion) in Tx path. Deactivation of configuration needs @see disableInvertIq.
*
* @param takeMutex use the mutex wich blocks the spi access for other tasks, if used in critical area this has to be set to false.
*
* @return nothing.
*
* @exceptsafe This function does not throw exceptions.
*/
void LoRaModemSX127x::disableInvertIqTx(bool takeMutex)
{
  reconfigRegister(REG_INVERTIQ, readRegister(REG_INVERTIQ, takeMutex) & 0xFE, takeMutex);
}

/**
* @brief enable configured inversion of polarity (I and Q path).
*
* @param takeMutex use the mutex wich blocks the spi access for other tasks, if used in critical area this has to be set to false.
*
* @return nothing.
*
* @exceptsafe This function does not throw exceptions.
*/
void LoRaModemSX127x::enableInvertIq(bool takeMutex)
{
  reconfigRegister(REG_INVERTIQ2, 0x19, takeMutex);
}

/**
* @brief disable configured inversion of polarity (I and Q path).
*
* @param takeMutex use the mutex wich blocks the spi access for other tasks, if used in critical area this has to be set to false.
*
* @return nothing.
*
* @exceptsafe This function does not throw exceptions.
*/
void LoRaModemSX127x::disableInvertIq(bool takeMutex)
{
  reconfigRegister(REG_INVERTIQ2, 0x1d, takeMutex);
}

/**
* @brief set over current protection. 
*
* @param mA over current protection in mA. 
* @param takeMutex use the mutex wich blocks the spi access for other tasks, if used in critical area this has to be set to false.
*
* @return nothing.
*
* @exceptsafe This function does not throw exceptions.
*/
void LoRaModemSX127x::setOCP(uint8_t mA, bool takeMutex)
{
  _modemConfig.OCPmA = mA;

  uint8_t ocpTrim = 27;

  if (mA <= 120)
  {
    ocpTrim = (mA - 45) / 5;
  }
  else if (mA <= 240)
  {
    ocpTrim = (mA + 30) / 10;
  }

  reconfigRegister(REG_OCP, 0x20 | (0x1F & ocpTrim), takeMutex);
}

/**
* @brief set over current protection. 
*
* @param gain value of 0 means auto adjust. Maximum value is 6.
* @param takeMutex use the mutex wich blocks the spi access for other tasks, if used in critical area this has to be set to false.
*
* @return nothing.
*
* @exceptsafe This function does not throw exceptions.
*/
void LoRaModemSX127x::setGain(uint8_t gain, bool takeMutex)
{
  _modemConfig.gain = gain;

  // check allowed range
  if (gain > 6)
  {
    gain = 6;
  }

  // set gain
  if (gain == 0)
  {
    // if gain = 0, enable AGC
    reconfigRegister(REG_MODEM_CONFIG_3, 0x04, takeMutex);
  }
  else
  {
    // disable AGC
    reconfigRegister(REG_MODEM_CONFIG_3, 0x00, takeMutex);

    // clear Gain and set LNA boost
    reconfigRegister(REG_LNA, 0x03, takeMutex);

    // set gain
    reconfigRegister(REG_LNA, readRegister(REG_LNA, takeMutex) | (gain << 5), takeMutex);
  }

  if (_modemConfig.modulation == LoRaModulation_t::LORA)
  {
    setLdoFlag(takeMutex);
  }
}

byte LoRaModemSX127x::random(bool takeMutex)
{
  return readRegister(REG_RSSI_WIDEBAND, takeMutex);
}

/**
* @brief set modem pin connection. 
*
* @param ss pin slave select.
* @param reset pin reset.
* @param dio0 pin dio0.
* @param dio1 pin dio1.
*
* @return nothing.
*
* @exceptsafe This function does not throw exceptions.
*/
void LoRaModemSX127x::setPins(int ss, int reset, int dio0, int dio1)
{
  _ss = ss;
  _reset = reset;
  _dio0 = dio0;
  _dio1 = dio1;
}

/**
* @brief set spi interface. 
*
* @param spi reference to spi interface.
*
* @return nothing.
*
* @exceptsafe This function does not throw exceptions.
*/
void LoRaModemSX127x::setSPI(SPIClass &spi)
{
  _spi = &spi;
}

/**
* @brief set maximum spi frequency. 
*
* @param frequency spi frequnecy in Hz, maximum supported from modem side is 10kHz.
*
* @return nothing.
*
* @exceptsafe This function does not throw exceptions.
*/
void LoRaModemSX127x::setSPIFrequency(uint32_t frequency)
{
  xSemaphoreTake(spiHndlMutex, portMAX_DELAY);
  _spiSettings = SPISettings(frequency, MSBFIRST, SPI_MODE0);
  xSemaphoreGive(spiHndlMutex);
}

/**
* @brief enable explicit header mode.
* feature description:
* - The payload length in bytes.
* - The forward error correction code rate
* - The presence of an optional 16-bits CRC for the payload.
*
* @param takeMutex use the mutex wich blocks the spi access for other tasks, if used in critical area this has to be set to false.
*
* @return nothing.
*
* @exceptsafe This function does not throw exceptions.
*/
void LoRaModemSX127x::explicitHeaderMode(bool takeMutex)
{
  _implicitHeaderMode = 0;

  reconfigRegister(REG_MODEM_CONFIG_1, readRegister(REG_MODEM_CONFIG_1, takeMutex) & 0xfe, takeMutex);
}

/**
* @brief implicit explicit header mode.
* feature description:
* - header is removed from payload
* - necessary if SF = 6 selected, only mode of operation
* - needs manual configuration oferror correction code rate
* - needs manual configuration of CRC for the payload.
*
* @param takeMutex use the mutex wich blocks the spi access for other tasks, if used in critical area this has to be set to false.
*
* @return nothing.
*
* @exceptsafe This function does not throw exceptions.
*/
void LoRaModemSX127x::implicitHeaderMode(bool takeMutex)
{
  _implicitHeaderMode = 1;

  reconfigRegister(REG_MODEM_CONFIG_1, readRegister(REG_MODEM_CONFIG_1, takeMutex) | 0x01, takeMutex);
}

/// @brief handle dio rise function
void LoRaModemSX127x::handleDio0Rise(int64_t timestamp, uint8_t dio0Irq, uint8_t dio1Irq)
{
  ///CIRITICAL REGION START
  //Blocking interrupt functions
  //Serial.print("+");
  xSemaphoreTake(spiHndlMutex, portMAX_DELAY);
  //Serial.print("*");
  //_irqTaskTiks = micros();

  //read Irq flags
  uint8_t irqFlags = readRegister(REG_IRQ_FLAGS, false);

  //all read interrupts clear irq's
  writeRegister(REG_IRQ_FLAGS, irqFlags, false);

  if (isIrqRxDone(irqFlags))
  {
    if (_onReceive)
    {
      uint32_t packetLength = _implicitHeaderMode ? readRegister(REG_PAYLOAD_LENGTH, false) : readRegister(REG_RX_NB_BYTES, false);

      _onReceive(instance, timestamp, packetLength, irqFlags);
    }
  }

  if (isIrqTxDone(irqFlags))
  {
    if (_onTxDone)
    {
      _onTxDone(instance, timestamp);
    }
  }

  if (isIrqCadDone(irqFlags))
  {
    if (_onCadDone)
    {
      _onCadDone(instance, timestamp, irqFlags);
    }
  }

  if (isIrqRxTout(irqFlags))
  {
    if (_onRxTimeout)
    {
      _onRxTimeout(instance, timestamp);
    }
  }

  if (isIrqCadDetect(irqFlags))
  {
    if (_onCadDetected)
    {
      _onCadDetected(instance, timestamp);
    }
  }

  if (isIrqValidHeader(irqFlags))
  {
    //Serial.println("irqOnValid");
    if (_onValidHeader)
    {
      //Serial.println("irqOnValidFncall");
      _onValidHeader(instance, timestamp);
    }
  }

  xSemaphoreGive(spiHndlMutex);

  ///CIRITICAL REGION END
}

/// @brief read modem register
uint8_t LoRaModemSX127x::readRegister(uint8_t address, bool takeMutex)
{
  return singleTransfer(address & 0x7f, 0x00, takeMutex);
}

/// @brief set modem to standby, then set the requested register
void LoRaModemSX127x::reconfigRegister(uint8_t address, uint8_t value, bool takeMutex)
{
  //modem has to be set in standby or in sleep mode to change configuration registers

  cfg_stdby(takeMutex);

  writeRegister(address, value, takeMutex);
}

/// @brief write the requested register
void LoRaModemSX127x::writeRegister(uint8_t address, uint8_t value, bool takeMutex)
{
  singleTransfer(address | 0x80, value, takeMutex);
}

/**
* @brief backup the current modem configuration.
*
* @param modemCfg referenc to the data struct where the backup should be stored.
* @param takeMutex use the mutex wich blocks the spi access for other tasks, if used in critical area this has to be set to false.
*
* @return nothing.
*
* @exceptsafe This function does not throw exceptions.
*/
void LoRaModemSX127x::backupModemConfig(ModemConfig_t &modemCfg)
{
  memcpy(&modemCfg, &_modemConfig, sizeof(ModemConfig_t));
}

/**
* @brief print the current modem config to serial. 
*
* @return nothing.
*
* @exceptsafe This function does not throw exceptions.
*/
void LoRaModemSX127x::printModemConfig()
{
  Serial.print("freq_hz:");
  Serial.println(_modemConfig.freq_hz);
  Serial.print("tx_power: ");
  Serial.println(_modemConfig.tx_power);
  Serial.print("spreadFact: ");
  Serial.println(_modemConfig.spreadFact);
  Serial.print("bandwidth_hz: ");
  Serial.println(_modemConfig.bandwidth_hz);
  Serial.print("codingRate: ");
  Serial.println(_modemConfig.codingRate);
  Serial.print("preambleLength: ");
  Serial.println(_modemConfig.preambleLength);
  Serial.print("syncWord: ");
  Serial.println(_modemConfig.syncWord);
  Serial.print("crc:");
  Serial.println(_modemConfig.crc);
  Serial.print("invertPolarity: ");
  Serial.println(_modemConfig.invertPolarity);
  Serial.print("symbolTimeout:");
  Serial.println(_modemConfig.symbolTimeout);
  Serial.print("OCPmA:");
  Serial.println(_modemConfig.OCPmA);
  Serial.print("gain: ");
  Serial.println(_modemConfig.gain);
}

/**
* @brief restore the given modem configuration.
*
* @param modemCfg modem configuration wich should be restored.
* @param takeMutex use the mutex wich blocks the spi access for other tasks, if used in critical area this has to be set to false.
*
* @return nothing.
*
* @exceptsafe This function does not throw exceptions.
*/
void LoRaModemSX127x::restoreModemConfig(ModemConfig_t &modemCfg, bool takeMutex)
{
  //Serial.println("rest::freq_hz");
  setFrequency(modemCfg.freq_hz, takeMutex);
  //Serial.println("rest::gain");
  setGain(modemCfg.gain, takeMutex);
  //Serial.println("rest::tx_power");
  setTxPower(modemCfg.tx_power, takeMutex);
  //Serial.println("rest::spreadFact");
  setSpreadingFactor(modemCfg.spreadFact, takeMutex);
  //Serial.println("rest::bandwidth_hz");
  setSignalBandwidth(modemCfg.bandwidth_hz, takeMutex);
  //Serial.println("rest::codingRate");
  setCodingRate4(modemCfg.codingRate, takeMutex);
  //Serial.println("rest::preambleLength");
  setPreambleLength(modemCfg.preambleLength, takeMutex);
  //Serial.println("rest::syncWord");
  setSyncWord(modemCfg.syncWord, takeMutex);
  //Serial.println("rest::takeMutex");
  setCrc(modemCfg.crc, takeMutex);
  //Serial.println("rest::invertIQ");
  setInvertPolarity(modemCfg.invertPolarity, takeMutex);
  //Serial.println("rest::symbolTimeout");
  setSymbolTimeout(modemCfg.symbolTimeout, takeMutex);
  //Serial.println("rest::OCPmA");
  setOCP(modemCfg.OCPmA, takeMutex);
}

/// @brief read or write to an register
uint8_t LoRaModemSX127x::singleTransfer(uint8_t address, uint8_t value, bool takeMutex)
{
  uint8_t response;
  //esp_task_wdt_reset();

  if (takeMutex)
  {
    xSemaphoreTake(spiHndlMutex, portMAX_DELAY);
  }

  digitalWrite(_ss, LOW);

  _spi->beginTransaction(_spiSettings);
  _spi->transfer(address);
  response = _spi->transfer(value);
  _spi->endTransaction();

  digitalWrite(_ss, HIGH);

  if (takeMutex)
  {
    xSemaphoreGive(spiHndlMutex);
  }

  return response;
}

// Interrupt DEMUX
// Irq Service Routine for DIO0
//============================
ISR_PREFIX void LoRaModemSX127x::Instance1onDio0Rise()
{
  irqData_t data;
  data.timestamp = esp_timer_get_time();
  data.exeFlags = 1;
  data.pinDio0 = LoRaExt[0]._dio0IrqMode;
  data.pinDio1 = LoRaExt[0]._dio1IrqMode;

  xQueueSend(LoRaExt[0].isrTaskQueue, &data, portMAX_DELAY);
  vTaskResume(LoRaExt[0].isrServiceTaskHndl);
}

ISR_PREFIX void LoRaModemSX127x::Instance2onDio0Rise()
{
  irqData_t data;
  data.timestamp = esp_timer_get_time();
  data.exeFlags = 2;
  data.pinDio0 = LoRaExt[0]._dio0IrqMode;
  data.pinDio1 = LoRaExt[0]._dio1IrqMode;

  xQueueSend(LoRaExt[0].isrTaskQueue, &data, portMAX_DELAY);
  vTaskResume(LoRaExt[0].isrServiceTaskHndl);
}
ISR_PREFIX void LoRaModemSX127x::Instance3onDio0Rise()
{
  irqData_t data;
  data.timestamp = esp_timer_get_time();
  data.exeFlags = 3;
  data.pinDio0 = LoRaExt[0]._dio0IrqMode;
  data.pinDio1 = LoRaExt[0]._dio1IrqMode;

  xQueueSend(LoRaExt[0].isrTaskQueue, &data, portMAX_DELAY);
  vTaskResume(LoRaExt[0].isrServiceTaskHndl);
}

ISR_PREFIX void LoRaModemSX127x::Instance4onDio0Rise()
{
  irqData_t data;
  data.timestamp = esp_timer_get_time();
  data.exeFlags = 4;
  data.pinDio0 = LoRaExt[0]._dio0IrqMode;
  data.pinDio1 = LoRaExt[0]._dio1IrqMode;

  xQueueSend(LoRaExt[0].isrTaskQueue, &data, portMAX_DELAY);
  vTaskResume(LoRaExt[0].isrServiceTaskHndl);
}

ISR_PREFIX void LoRaModemSX127x::Instance5onDio0Rise()
{
  irqData_t data;
  data.timestamp = esp_timer_get_time();
  data.exeFlags = 5;
  data.pinDio0 = LoRaExt[0]._dio0IrqMode;
  data.pinDio1 = LoRaExt[0]._dio1IrqMode;

  xQueueSend(LoRaExt[0].isrTaskQueue, &data, portMAX_DELAY);
  vTaskResume(LoRaExt[0].isrServiceTaskHndl);
}

ISR_PREFIX void LoRaModemSX127x::Instance6onDio0Rise()
{
  irqData_t data;
  data.timestamp = esp_timer_get_time();
  data.exeFlags = 6;
  data.pinDio0 = LoRaExt[0]._dio0IrqMode;
  data.pinDio1 = LoRaExt[0]._dio1IrqMode;

  xQueueSend(LoRaExt[0].isrTaskQueue, &data, portMAX_DELAY);
  vTaskResume(LoRaExt[0].isrServiceTaskHndl);
}

ISR_PREFIX void LoRaModemSX127x::Instance7onDio0Rise()
{
  irqData_t data;
  data.timestamp = esp_timer_get_time();
  data.exeFlags = 7;
  data.pinDio0 = LoRaExt[0]._dio0IrqMode;
  data.pinDio1 = LoRaExt[0]._dio1IrqMode;

  xQueueSend(LoRaExt[0].isrTaskQueue, &data, portMAX_DELAY);
  vTaskResume(LoRaExt[0].isrServiceTaskHndl);
}

ISR_PREFIX void LoRaModemSX127x::Instance8onDio0Rise()
{
  irqData_t data;
  data.timestamp = esp_timer_get_time();
  data.exeFlags = 8;
  data.pinDio0 = LoRaExt[0]._dio0IrqMode;
  data.pinDio1 = LoRaExt[0]._dio1IrqMode;

  xQueueSend(LoRaExt[0].isrTaskQueue, &data, portMAX_DELAY);
  vTaskResume(LoRaExt[0].isrServiceTaskHndl);
}

ISR_PREFIX void LoRaModemSX127x::Instance9onDio0Rise()
{
  irqData_t data;
  data.timestamp = esp_timer_get_time();
  data.exeFlags = 9;
  data.pinDio0 = LoRaExt[0]._dio0IrqMode;
  data.pinDio1 = LoRaExt[0]._dio1IrqMode;

  xQueueSend(LoRaExt[0].isrTaskQueue, &data, portMAX_DELAY);
  vTaskResume(LoRaExt[0].isrServiceTaskHndl);
}

ISR_PREFIX void LoRaModemSX127x::Instance10onDio0Rise()
{
  irqData_t data;
  data.timestamp = esp_timer_get_time();
  data.exeFlags = 10;
  data.pinDio0 = LoRaExt[0]._dio0IrqMode;
  data.pinDio1 = LoRaExt[0]._dio1IrqMode;

  xQueueSend(LoRaExt[0].isrTaskQueue, &data, portMAX_DELAY);
  vTaskResume(LoRaExt[0].isrServiceTaskHndl);
}

ISR_PREFIX void LoRaModemSX127x::Instance11onDio0Rise()
{
  irqData_t data;
  data.timestamp = esp_timer_get_time();
  data.exeFlags = 11;
  data.pinDio0 = LoRaExt[0]._dio0IrqMode;
  data.pinDio1 = LoRaExt[0]._dio1IrqMode;

  xQueueSend(LoRaExt[0].isrTaskQueue, &data, portMAX_DELAY);
  vTaskResume(LoRaExt[0].isrServiceTaskHndl);
}

ISR_PREFIX void LoRaModemSX127x::Instance12onDio0Rise()
{
  irqData_t data;
  data.timestamp = esp_timer_get_time();
  data.exeFlags = 12;
  data.pinDio0 = LoRaExt[0]._dio0IrqMode;
  data.pinDio1 = LoRaExt[0]._dio1IrqMode;

  xQueueSend(LoRaExt[0].isrTaskQueue, &data, portMAX_DELAY);
  vTaskResume(LoRaExt[0].isrServiceTaskHndl);
}

//Irq Service Routine for DIO1
//============================

ISR_PREFIX void LoRaModemSX127x::Instance1onDio1Rise()
{
  irqData_t data;
  data.timestamp = esp_timer_get_time();
  data.exeFlags = 1 << 16;
  data.pinDio0 = LoRaExt[0]._dio0IrqMode;
  data.pinDio1 = LoRaExt[0]._dio1IrqMode;

  xQueueSend(LoRaExt[0].isrTaskQueue, &data, portMAX_DELAY);
  vTaskResume(LoRaExt[0].isrServiceTaskHndl);
}

ISR_PREFIX void LoRaModemSX127x::Instance2onDio1Rise()
{
  irqData_t data;
  data.timestamp = esp_timer_get_time();
  data.exeFlags = 2 << 16;
  data.pinDio0 = LoRaExt[0]._dio0IrqMode;
  data.pinDio1 = LoRaExt[0]._dio1IrqMode;

  xQueueSend(LoRaExt[0].isrTaskQueue, &data, portMAX_DELAY);
  vTaskResume(LoRaExt[0].isrServiceTaskHndl);
}
ISR_PREFIX void LoRaModemSX127x::Instance3onDio1Rise()
{
  irqData_t data;
  data.timestamp = esp_timer_get_time();
  data.exeFlags = 3 << 16;
  data.pinDio0 = LoRaExt[0]._dio0IrqMode;
  data.pinDio1 = LoRaExt[0]._dio1IrqMode;

  xQueueSend(LoRaExt[0].isrTaskQueue, &data, portMAX_DELAY);
  vTaskResume(LoRaExt[0].isrServiceTaskHndl);
}

ISR_PREFIX void LoRaModemSX127x::Instance4onDio1Rise()
{
  irqData_t data;
  data.timestamp = esp_timer_get_time();
  data.exeFlags = 4 << 16;
  data.pinDio0 = LoRaExt[0]._dio0IrqMode;
  data.pinDio1 = LoRaExt[0]._dio1IrqMode;

  xQueueSend(LoRaExt[0].isrTaskQueue, &data, portMAX_DELAY);
  vTaskResume(LoRaExt[0].isrServiceTaskHndl);
}

ISR_PREFIX void LoRaModemSX127x::Instance5onDio1Rise()
{
  irqData_t data;
  data.timestamp = esp_timer_get_time();
  data.exeFlags = 5 << 16;
  data.pinDio0 = LoRaExt[0]._dio0IrqMode;
  data.pinDio1 = LoRaExt[0]._dio1IrqMode;

  xQueueSend(LoRaExt[0].isrTaskQueue, &data, portMAX_DELAY);
  vTaskResume(LoRaExt[0].isrServiceTaskHndl);
}

ISR_PREFIX void LoRaModemSX127x::Instance6onDio1Rise()
{
  irqData_t data;
  data.timestamp = esp_timer_get_time();
  data.exeFlags = 6 << 16;
  data.pinDio0 = LoRaExt[0]._dio0IrqMode;
  data.pinDio1 = LoRaExt[0]._dio1IrqMode;

  xQueueSend(LoRaExt[0].isrTaskQueue, &data, portMAX_DELAY);
  vTaskResume(LoRaExt[0].isrServiceTaskHndl);
}

ISR_PREFIX void LoRaModemSX127x::Instance7onDio1Rise()
{
  irqData_t data;
  data.timestamp = esp_timer_get_time();
  data.exeFlags = 7 << 16;
  data.pinDio0 = LoRaExt[0]._dio0IrqMode;
  data.pinDio1 = LoRaExt[0]._dio1IrqMode;

  xQueueSend(LoRaExt[0].isrTaskQueue, &data, portMAX_DELAY);
  vTaskResume(LoRaExt[0].isrServiceTaskHndl);
}

ISR_PREFIX void LoRaModemSX127x::Instance8onDio1Rise()
{
  irqData_t data;
  data.timestamp = esp_timer_get_time();
  data.exeFlags = 8 << 16;
  data.pinDio0 = LoRaExt[0]._dio0IrqMode;
  data.pinDio1 = LoRaExt[0]._dio1IrqMode;

  xQueueSend(LoRaExt[0].isrTaskQueue, &data, portMAX_DELAY);
  vTaskResume(LoRaExt[0].isrServiceTaskHndl);
}

ISR_PREFIX void LoRaModemSX127x::Instance9onDio1Rise()
{
  irqData_t data;
  data.timestamp = esp_timer_get_time();
  data.exeFlags = 9 << 16;
  data.pinDio0 = LoRaExt[0]._dio0IrqMode;
  data.pinDio1 = LoRaExt[0]._dio1IrqMode;

  xQueueSend(LoRaExt[0].isrTaskQueue, &data, portMAX_DELAY);
  vTaskResume(LoRaExt[0].isrServiceTaskHndl);
}

ISR_PREFIX void LoRaModemSX127x::Instance10onDio1Rise()
{
  irqData_t data;
  data.timestamp = esp_timer_get_time();
  data.exeFlags = 10 << 16;
  data.pinDio0 = LoRaExt[0]._dio0IrqMode;
  data.pinDio1 = LoRaExt[0]._dio1IrqMode;

  xQueueSend(LoRaExt[0].isrTaskQueue, &data, portMAX_DELAY);
  vTaskResume(LoRaExt[0].isrServiceTaskHndl);
}

ISR_PREFIX void LoRaModemSX127x::Instance11onDio1Rise()
{
  irqData_t data;
  data.timestamp = esp_timer_get_time();
  data.exeFlags = 11 << 16;
  data.pinDio0 = LoRaExt[0]._dio0IrqMode;
  data.pinDio1 = LoRaExt[0]._dio1IrqMode;

  xQueueSend(LoRaExt[0].isrTaskQueue, &data, portMAX_DELAY);
  vTaskResume(LoRaExt[0].isrServiceTaskHndl);
}

ISR_PREFIX void LoRaModemSX127x::Instance12onDio1Rise()
{
  irqData_t data;
  data.timestamp = esp_timer_get_time();
  data.exeFlags = 12 << 16;
  data.pinDio0 = LoRaExt[0]._dio0IrqMode;
  data.pinDio1 = LoRaExt[0]._dio1IrqMode;

  xQueueSend(LoRaExt[0].isrTaskQueue, &data, portMAX_DELAY);
  vTaskResume(LoRaExt[0].isrServiceTaskHndl);
}

//==================================

LoRaModemSX127x LoRaExt[12];
