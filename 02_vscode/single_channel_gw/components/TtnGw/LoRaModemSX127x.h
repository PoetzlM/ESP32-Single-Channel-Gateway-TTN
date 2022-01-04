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

#ifndef LORAMODEMSX127x_H
#define LORAMODEMSX127x_H

#include <Arduino.h>
#include <SPI.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "TtnGwDataTypes.h"
#include "esp_timer.h"
#include <arduino_base64.hpp>

// global defines
#define LORA_DEFAULT_SPI SPI
#define LORA_DEFAULT_SPI_FREQUENCY 8E6
#define LORA_DEFAULT_SS_PIN 10
#define LORA_DEFAULT_RESET_PIN 9
#define LORA_DEFAULT_DIO0_PIN 2
#define LORA_DEFAULT_DIO1_PIN 3

#define PA_OUTPUT_RFO_PIN 0
#define PA_OUTPUT_PA_BOOST_PIN 1
// global defines end

/**
 * dereffered service intterupt task, necessary to use mutexes in combination with interrupts
 * function is started on interrupt and function sets itself to sleep if interrupt task is processed.
 */
void isrTask(void *parameter);

/**
 * enum to represent the configured mode of the modem.
 */
typedef enum ModemMode
{
  MODE_UNDEFINED = -1,
  SLEEP = 0,        ///< modem in sleep mode
  STBY = 1,         ///< modem in standby mode
  FSTX = 2,         ///< Frequency synthesis transmit channel
  M_TX = 3,         ///< transmit message in FiFo
  FSRX = 4,         ///< Frequency synthesis receive channel
  RXCONTINUOUS = 5, ///< conitnous receiving of messages
  RXSINGLE = 6,     ///< receive single LoRa message
  CAD = 7           ///< cahnnel activity detection (monitor the airchannel)

} ModemMode_t;

/**
 * struct to collect all modem settings
 */
typedef struct ModemConfig
{
  uint32_t freq_hz;        ///< receive and transmit frequency
  uint32_t tx_power;       ///< configured transmit power
  int spreadFact;          ///< configureed spreading factor
  double bandwidth_hz;     ///< configured data rate bandwidth
  int syncWord;            ///< syncword used in the preamble
  int codingRate;          ///< coding rate used for error correction
  uint32_t preambleLength; ///< length of the preamble
  bool crc;                ///< if cyclic redundancy check is enabled
  bool invertPolarity;     ///< invert polarity, how the i- and q- signals of the receive frequency is interpreted
  ModemMode_t mode;        ///< mode of the lora modem
  uint8_t symbolTimeout;   ///< configured symbol timeout
  uint8_t OCPmA;           ///< Over current portecion in mA
  uint8_t gain;            //< configured antenna gain

  //prepared for FSK Mode
  LoRaModulation_t modulation;
  float fsk_dev;         ///< fsk deviation
  uint16_t fsk_datarate; ///< fsk datarate
  uint16_t preamble;     ///< preamble for fsk mode

} ModemConfig_t;

/**
 * struct to collect the configuration of the hardware interrupt
 */
typedef struct irqData
{
  uint32_t exeFlags;
  uint8_t pinDio0;
  uint8_t pinDio1;
  int64_t timestamp;
} irqData_t;

/**
 * class to interface with the semtec SX127x modems
 */
class LoRaModemSX127x
{
public:
  /**
  * @brief constructor for LoRaModemSX127x, setup the instance number.
  *
  * @exceptsafe This function does not throw exceptions.
  */
  LoRaModemSX127x();

  /**
  * @brief initialize modem on an specific frequency
  *
  * @param frequency in Hz
  *
  * @return nothing.
  *
  * @exceptsafe This function does not throw exceptions.
  */
  int begin(long frequency);

  /**
  * @brief shut down spi bus
  *
  * @return nothing.
  *
  * @exceptsafe This function does not throw exceptions.
  */
  void end();

  /**
  * @brief read the Received Signal Strength Indicator of the received data packet 
  *
  * @param takeMutex use the mutex wich blocks the spi access for other tasks, if used in critical area this has to be set to false.
  *
  * @return integer with the rssi value.
  *
  * @exceptsafe This function does not throw exceptions.
  */
  int packetRssi(bool takeMutex);

  /**
  * @brief read the signal to noise ratio of the received data packet 
  *
  * @param takeMutex use the mutex wich blocks the spi access for other tasks, if used in critical area this has to be set to false.
  *
  * @return float with the snr value.
  *
  * @exceptsafe This function does not throw exceptions.
  */
  float packetSnr(bool takeMutex);

  /**
  * @brief read the frequency error, because the crystal reference oscillator has only a finite frequency 
  *
  * @param takeMutex use the mutex wich blocks the spi access for other tasks, if used in critical area this has to be set to false.
  *
  * @return long with the computed frequency error.
  *
  * @exceptsafe This function does not throw exceptions.
  */
  long packetFrequencyError(bool takeMutex);

  /**
  * @brief read the current Signal Strength Indicator over the air
  *
  * @param takeMutex use the mutex wich blocks the spi access for other tasks, if used in critical area this has to be set to false.
  *
  * @return integer with the rssi value.
  *
  * @exceptsafe This function does not throw exceptions.
  */
  int rssi(bool takeMutex);

  /**
  * @brief register callback on the event of received message.
  *
  * @param callback function to be called on event
  *
  * @return nothing
  *
  * @exceptsafe This function does not throw exceptions.
  */
  void onReceive(void (*callback)(int, int64_t, uint32_t, uint8_t));

  /**
  * @brief register callback on the event of transmission completed.
  *
  * @param callback function to be called on event
  *
  * @return nothing
  *
  * @exceptsafe This function does not throw exceptions.
  */
  void onTxDone(void (*callback)(int, int64_t));

  /**
  * @brief register callback on the event of channel activity detection completed.
  *
  * @param callback function to be called on event
  *
  * @return nothing
  *
  * @exceptsafe This function does not throw exceptions.
  */
  void onCadDone(void (*callback)(int, int64_t, uint8_t));

  /**
  * @brief register callback on the event of channel activity detected.
  *
  * @param callback function to be called on event
  *
  * @return nothing
  *
  * @exceptsafe This function does not throw exceptions.
  */
  void onCadDetected(void (*callback)(int, int64_t));

  /**
  * @brief register callback on the event of received message with valid header.
  *
  * @param callback function to be called on event
  *
  * @return nothing
  *
  * @exceptsafe This function does not throw exceptions.
  */
  void onValidHeader(void (*callback)(int, int64_t));

  /**
  * @brief register callback on the event of received timeout.
  *
  * @param callback function to be called on event
  *
  * @return nothing
  *
  * @exceptsafe This function does not throw exceptions.
  */
  void onRxTimeout(void (*callback)(int, int64_t));

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
  void recContinuous(int size = 0, bool takeMutex = true);

  /**
  * @brief set the modem in receive single message mode, including the configuration of the interrupts
  *
  * @param takeMutex use the mutex wich blocks the spi access for other tasks, if used in critical area this has to be set to false.
  *
  * @return nothing.
  *
  * @exceptsafe This function does not throw exceptions.
  */
  void recSingleMsg(bool takeMutex);

  /**
  * @brief set the modem in channel activiy detection mode, including the configuration of the interrupts.
  *
  * @param takeMutex use the mutex wich blocks the spi access for other tasks, if used in critical area this has to be set to false.
  *
  * @return nothing.
  *
  * @exceptsafe This function does not throw exceptions.
  */
  void cadScan(bool takeMutex);

  /**
  * @brief set the modem in standby mode
  *
  * @param takeMutex use the mutex wich blocks the spi access for other tasks, if used in critical area this has to be set to false.
  *
  * @return nothing.
  *
  * @exceptsafe This function does not throw exceptions.
  */
  void stdby(bool takeMutex);

  /**
  * @brief set the modem in standby mode, without changing the last set mode, helper function to reconfigure modem registers 
  *
  * @param takeMutex use the mutex wich blocks the spi access for other tasks, if used in critical area this has to be set to false.
  *
  * @return nothing.
  *
  * @exceptsafe This function does not throw exceptions.
  */
  void cfg_stdby(bool takeMutex);

  /**
  * @brief set the modem in sleep mode
  *
  * @param takeMutex use the mutex wich blocks the spi access for other tasks, if used in critical area this has to be set to false.
  *
  * @return nothing.
  *
  * @exceptsafe This function does not throw exceptions.
  */
  void sleep(bool takeMutex);

  /**
  * @brief set the modem in channel activity detection mode.
  *
  * @param takeMutex use the mutex wich blocks the spi access for other tasks, if used in critical area this has to be set to false.
  *
  * @return nothing.
  *
  * @exceptsafe This function does not throw exceptions.
  */
  void cad(bool takeMutex);

  /**
  * @brief set the modem in transmit mode.
  *
  * @param takeMutex use the mutex wich blocks the spi access for other tasks, if used in critical area this has to be set to false.
  *
  * @return nothing.
  *
  * @exceptsafe This function does not throw exceptions.
  */
  void transmit(bool takeMutex);

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
  void putPayloadFiFo(uint8_t *buffer, size_t size, bool takeMutex);

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
  void setModemMode(ModemMode_t mode, bool takeMutex);

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
  void setTxPower(int level, bool takeMutex, int outputPin = PA_OUTPUT_PA_BOOST_PIN);

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
  void setFrequency(long frequency, bool takeMutex);

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
  void setSpreadingFactor(int sf, bool takeMutex);

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
  void setSignalBandwidth(long sbw, bool takeMutex);

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
  void setCodingRate4(int denominator, bool takeMutex);

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
  void setPreambleLength(uint32_t length, bool takeMutex);

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
  void setSyncWord(int sw, bool takeMutex);

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
  void setCrc(bool mode, bool takeMutex);

  /**
  * @brief enable crc.
  *
  * @param takeMutex use the mutex wich blocks the spi access for other tasks, if used in critical area this has to be set to false.
  *
  * @return nothing.
  *
  * @exceptsafe This function does not throw exceptions.
  */
  void enableCrc(bool takeMutex);

  /**
  * @brief disable crc.
  *
  * @param takeMutex use the mutex wich blocks the spi access for other tasks, if used in critical area this has to be set to false.
  *
  * @return nothing.
  *
  * @exceptsafe This function does not throw exceptions.
  */
  void disableCrc(bool takeMutex);

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
  void setInvertPolarity(bool mode, bool takeMutex);

  /**
  * @brief enable inversion of polarity (I and Q path inversion).
  *
  * @param takeMutex use the mutex wich blocks the spi access for other tasks, if used in critical area this has to be set to false.
  *
  * @return nothing.
  *
  * @exceptsafe This function does not throw exceptions.
  */
  void enableInvertPolarity(bool takeMutex);

  /**
  * @brief disable inversion of polarity (I and Q path inversion).
  *
  * @param takeMutex use the mutex wich blocks the spi access for other tasks, if used in critical area this has to be set to false.
  *
  * @return nothing.
  *
  * @exceptsafe This function does not throw exceptions.
  */
  void disableInvertPolarity(bool takeMutex);

  /**
  * @brief disable inversion of polarity (I and Q path inversion) in Rx path. Activation of configuration needs @see enableInvertIq.
  *
  * @param takeMutex use the mutex wich blocks the spi access for other tasks, if used in critical area this has to be set to false.
  *
  * @return nothing.
  *
  * @exceptsafe This function does not throw exceptions.
  */
  void enableInvertIqRx(bool takeMutex);

  /**
  * @brief disbale inversion of polarity (I and Q path inversion) in Rx path. Deactivation of configuration needs @see disableInvertIq.
  *
  * @param takeMutex use the mutex wich blocks the spi access for other tasks, if used in critical area this has to be set to false.
  *
  * @return nothing.
  *
  * @exceptsafe This function does not throw exceptions.
  */
  void disableInvertIqRx(bool takeMutex);

  /**
  * @brief disable inversion of polarity (I and Q path inversion) in Tx path. Activation of configuration needs @see enableInvertIq.
  *
  * @param takeMutex use the mutex wich blocks the spi access for other tasks, if used in critical area this has to be set to false.
  *
  * @return nothing.
  *
  * @exceptsafe This function does not throw exceptions.
  */
  void enableInvertIqTx(bool takeMutex);

  /**
  * @brief disbale inversion of polarity (I and Q path inversion) in Tx path. Deactivation of configuration needs @see disableInvertIq.
  *
  * @param takeMutex use the mutex wich blocks the spi access for other tasks, if used in critical area this has to be set to false.
  *
  * @return nothing.
  *
  * @exceptsafe This function does not throw exceptions.
  */
  void disableInvertIqTx(bool takeMutex);

  /**
  * @brief enable configured inversion of polarity (I and Q path).
  *
  * @param takeMutex use the mutex wich blocks the spi access for other tasks, if used in critical area this has to be set to false.
  *
  * @return nothing.
  *
  * @exceptsafe This function does not throw exceptions.
  */
  void enableInvertIq(bool takeMutex);

  /**
  * @brief disable configured inversion of polarity (I and Q path).
  *
  * @param takeMutex use the mutex wich blocks the spi access for other tasks, if used in critical area this has to be set to false.
  *
  * @return nothing.
  *
  * @exceptsafe This function does not throw exceptions.
  */
  void disableInvertIq(bool takeMutex);

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
  void setSymbolTimeout(uint8_t timeout, bool takeMutex);

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
  void setOCP(uint8_t mA, bool takeMutex); // Over Current Protection control

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
  void setGain(uint8_t gain, bool takeMutex); // Set LNA gain

  /**
  * @brief print the current modem config to serial. 
  *
  * @return nothing.
  *
  * @exceptsafe This function does not throw exceptions.
  */
  void printModemConfig();

  /**
  * @brief generate random value by reading the background noise in the air. 
  *
  * @param takeMutex use the mutex wich blocks the spi access for other tasks, if used in critical area this has to be set to false.
  *
  * @return randome value.
  *
  * @exceptsafe This function does not throw exceptions.
  */
  byte random(bool takeMutex);

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
  void setPins(int ss = LORA_DEFAULT_SS_PIN, int reset = LORA_DEFAULT_RESET_PIN, int dio0 = LORA_DEFAULT_DIO0_PIN, int dio1 = LORA_DEFAULT_DIO1_PIN);

  /**
  * @brief set spi interface. 
  *
  * @param spi reference to spi interface.
  *
  * @return nothing.
  *
  * @exceptsafe This function does not throw exceptions.
  */
  void setSPI(SPIClass &spi);

  /**
  * @brief set maximum spi frequency. 
  *
  * @param frequency spi frequnecy in Hz, maximum supported from modem side is 10kHz.
  *
  * @return nothing.
  *
  * @exceptsafe This function does not throw exceptions.
  */
  void setSPIFrequency(uint32_t frequency);

  /**
  * @brief check the interrupt flags for Fhssch request. 
  *
  * @param flags actual contens of the interrupt register.
  *
  * @return true if interrupt is requested.
  *
  * @exceptsafe This function does not throw exceptions.
  */
  bool isIrqFhsschRequest(uint8_t flags);

  /**
  * @brief check the interrupt flags for cad done request. 
  *
  * @param flags actual contens of the interrupt register.
  *
  * @return true if interrupt is requested.
  *
  * @exceptsafe This function does not throw exceptions.
  */
  bool isIrqCadDone(uint8_t flags);

  /**
  * @brief check the interrupt flags for valid header request. 
  *
  * @param flags actual contens of the interrupt register.
  *
  * @return true if interrupt is requested.
  *
  * @exceptsafe This function does not throw exceptions.
  */
  bool isIrqValidHeader(uint8_t flags);

  /**
  * @brief check the interrupt flags for rx timeout request. 
  *
  * @param flags actual contens of the interrupt register.
  *
  * @return true if interrupt is requested.
  *
  * @exceptsafe This function does not throw exceptions.
  */
  bool isIrqRxTout(uint8_t flags);

  /**
  * @brief check the interrupt flags for cad detected request. 
  *
  * @param flags actual contens of the interrupt register.
  *
  * @return true if interrupt is requested.
  *
  * @exceptsafe This function does not throw exceptions.
  */
  bool isIrqCadDetect(uint8_t flags);

  /**
  * @brief check the interrupt flags for crc error request. 
  *
  * @param flags actual contens of the interrupt register.
  *
  * @return true if interrupt is requested.
  *
  * @exceptsafe This function does not throw exceptions.
  */
  bool isIrqCrcError(uint8_t flags);

  /**
  * @brief check the interrupt flags for rx done request. 
  *
  * @param flags actual contens of the interrupt register.
  *
  * @return true if interrupt is requested.
  *
  * @exceptsafe This function does not throw exceptions.
  */
  bool isIrqRxDone(uint8_t flags);

  /**
  * @brief check the interrupt flags for tx done request. 
  *
  * @param flags actual contens of the interrupt register.
  *
  * @return true if interrupt is requested.
  *
  * @exceptsafe This function does not throw exceptions.
  */
  bool isIrqTxDone(uint8_t flags);

  /**
  * @brief read the modem status register. 
  *
  * @param takeMutex use the mutex wich blocks the spi access for other tasks, if used in critical area this has to be set to false.
  *
  * @return contense of modem satus register
  *
  * @exceptsafe This function does not throw exceptions.
  */
  uint8_t getModemStatusRegister(bool takeMutex);

  /**
  * @brief check the given status register if signal was detected. 
  *
  * @param reg_data actual contens of the status register.
  *
  * @return true if signal was detected.
  *
  * @exceptsafe This function does not throw exceptions.
  */
  bool isStatusSignlaDetected(uint8_t reg_data);

  /**
  * @brief check the given status register if signal is synchronized. 
  *
  * @param reg_data actual contens of the status register.
  *
  * @return true if signal is synchronized.
  *
  * @exceptsafe This function does not throw exceptions.
  */
  bool isStatusSignlaSynchronized(uint8_t reg_data);

  /**
  * @brief check the given status register if message receive is ongoing. 
  *
  * @param reg_data actual contens of the status register.
  *
  * @return true if message receive is ongoing.
  *
  * @exceptsafe This function does not throw exceptions.
  */
  bool isStatusRxOngoing(uint8_t reg_data);

  /**
  * @brief check the given status register if the header information is valid. 
  *
  * @param reg_data actual contens of the status register.
  *
  * @return true if header information is valid.
  *
  * @exceptsafe This function does not throw exceptions.
  */
  bool isStatusHeaderInfoValid(uint8_t reg_data);

  /**
  * @brief check the given status register if modem is clear to operate. 
  *
  * @param reg_data actual contens of the status register.
  *
  * @return true if modem is clear to operate.
  *
  * @exceptsafe This function does not throw exceptions.
  */
  bool isStatusModemClear(uint8_t reg_data);

  /**
  * @brief get the number of received bytes.
  *
  * @param takeMutex use the mutex wich blocks the spi access for other tasks, if used in critical area this has to be set to false.
  *
  * @return number of received bytes.
  *
  * @exceptsafe This function does not throw exceptions.
  */
  int recNoBytes(bool takeMutex);

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
  void backupModemConfig(ModemConfig_t &modemCfg);

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
  void restoreModemConfig(ModemConfig_t &modemCfg, bool takeMutex);

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
  void readMsg(uint8_t *buffer, int len, bool takeMutex);

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
  bool transmitMsg(uint8_t *buffer, int len, bool async, bool takeMutex);

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
  bool prepareTransmitMsg(uint8_t *buffer, int len, bool takeMutex);

  /**
  * @brief get the currently configured spreading factor.
  *
  * @param takeMutex use the mutex wich blocks the spi access for other tasks, if used in critical area this has to be set to false.
  *
  * @return integer representation of the spreading factor.
  *
  * @exceptsafe This function does not throw exceptions.
  */
  int getSpreadingFactor(bool takeMutex);

  /**
  * @brief get the currently configured signal bandwidth.
  *
  * @param takeMutex use the mutex wich blocks the spi access for other tasks, if used in critical area this has to be set to false.
  *
  * @return long representation of the bandwidth in Hz.
  *
  * @exceptsafe This function does not throw exceptions.
  */
  long getSignalBandwidth(bool takeMutex);

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
  void explicitHeaderMode(bool takeMutex);

  /**
  * @brief implicit explicit header mode.
  * feature description:
  * - header is removed from payload
  * - necessary if SF = 6 selected, only mode of operation
  * - needs manual configuration of  error correction code rate
  * - needs manual configuration of CRC for the payload.
  *
  * @param takeMutex use the mutex wich blocks the spi access for other tasks, if used in critical area this has to be set to false.
  *
  * @return nothing.
  *
  * @exceptsafe This function does not throw exceptions.
  */
  void implicitHeaderMode(bool takeMutex);

  /**
  * @brief check if transmission is ongoing.
  *
  * @param takeMutex use the mutex wich blocks the spi access for other tasks, if used in critical area this has to be set to false.
  *
  * @return true if transmission is ongoing.
  *
  * @exceptsafe This function does not throw exceptions.
  */
  bool isTransmitting(bool takeMutex);

  /**
  * @brief set the low data rate optimization.
  *
  * @param takeMutex use the mutex wich blocks the spi access for other tasks, if used in critical area this has to be set to false.
  *
  * @return true if transmission is ongoing.
  *
  * @exceptsafe This function does not throw exceptions.
  */
  void setLdoFlag(bool takeMutex);

private:
  //Function section

  void _enableDio0Irq(); ///< @brief enable interrupt on Dio0
  void _enableDio1Irq(); ///< @brief enable interrupt on Dio1

  uint8_t readRegister(uint8_t address, bool takeMutex);                 ///< @brief read modem register
  void reconfigRegister(uint8_t address, uint8_t value, bool takeMutex); ///< @brief set modem to standby, then set the requested register

  void writeRegister(uint8_t address, uint8_t value, bool takeMutex = true);     ///< @brief write the requested register
  uint8_t singleTransfer(uint8_t address, uint8_t value, bool takeMutex = true); ///< @brief read or write to an register

  static void Instance1onDio0Rise();  ///< @brief static interrupt function for instance 1 on dio0
  static void Instance2onDio0Rise();  ///< @brief static interrupt function for instance 2 on dio0
  static void Instance3onDio0Rise();  ///< @brief static interrupt function for instance 3 on dio0
  static void Instance4onDio0Rise();  ///< @brief static interrupt function for instance 4 on dio0
  static void Instance5onDio0Rise();  ///< @brief static interrupt function for instance 5 on dio0
  static void Instance6onDio0Rise();  ///< @brief static interrupt function for instance 6 on dio0
  static void Instance7onDio0Rise();  ///< @brief static interrupt function for instance 7 on dio0
  static void Instance8onDio0Rise();  ///< @brief static interrupt function for instance 8 on dio0
  static void Instance9onDio0Rise();  ///< @brief static interrupt function for instance 9 on dio0
  static void Instance10onDio0Rise(); ///< @brief static interrupt function for instance 10 on dio0
  static void Instance11onDio0Rise(); ///< @brief static interrupt function for instance 11 on dio0
  static void Instance12onDio0Rise(); ///< @brief static interrupt function for instance 12 on dio0

  static void Instance1onDio1Rise();  ///< @brief static interrupt function for instance 1 on dio1
  static void Instance2onDio1Rise();  ///< @brief static interrupt function for instance 2 on dio0
  static void Instance3onDio1Rise();  ///< @brief static interrupt function for instance 3 on dio0
  static void Instance4onDio1Rise();  ///< @brief static interrupt function for instance 4 on dio0
  static void Instance5onDio1Rise();  ///< @brief static interrupt function for instance 5 on dio0
  static void Instance6onDio1Rise();  ///< @brief static interrupt function for instance 6 on dio0
  static void Instance7onDio1Rise();  ///< @brief static interrupt function for instance 7 on dio0
  static void Instance8onDio1Rise();  ///< @brief static interrupt function for instance 8 on dio0
  static void Instance9onDio1Rise();  ///< @brief static interrupt function for instance 9 on dio0
  static void Instance10onDio1Rise(); ///< @brief static interrupt function for instance 10 on dio0
  static void Instance11onDio1Rise(); ///< @brief static interrupt function for instance 11 on dio0
  static void Instance12onDio1Rise(); ///< @brief static interrupt function for instance 12 on dio0

private:
  //Variable section
  SPISettings _spiSettings; ///< spi configuration
  SPIClass *_spi;           ///< reference to spi interface
  int _ss;                  ///< slave select pin
  int _reset;               ///< reset pin
  int _dio0;                ///< dio0 interrupt pin
  int _dio1;                ///< dio1 interrupt pin

  static SemaphoreHandle_t spiHndlMutex; ///< mutex to access the spi interface exlusivly

  //config of interrupts
  void IrqRxDone(bool takeMutex);    ///< set interrupt pins to rx done
  void IrqTxDone(bool takeMutex);    ///< set interrupt pins to tx done
  void IrqCadDone(bool takeMutex);   ///< set interrupt pins to cad done
  void IrqCadDetect(bool takeMutex); ///< set interrupt pins to cad detect
  void IrqRxTimeout(bool takeMutex); ///< set interrupt pins to tx timeout

  //register backups
  uint8_t _dio0IrqMode = 0; ///< last configuration of the dio0 register
  uint8_t _dio1IrqMode = 0; ///< last configuration of the dio1 register
  uint8_t _dio2IrqMode = 0; ///< last configuration of the dio2 register
  uint8_t _dio3IrqMode = 0; ///< last configuration of the dio3 register
  uint8_t _regOpMode = 0;   ///< last configuration of the operation mode register

  //konfigurierte Paramter
  ModemConfig_t _modemConfig; ///< contains the complete modem configuration

  int _implicitHeaderMode; ///< last configuration of the implicit header register

  void (*_onReceive)(int, int64_t, uint32_t, uint8_t); ///< function pointer wich is called on message receive
  void (*_onTxDone)(int, int64_t);                     ///< function pointer wich is called on transmission completed
  void (*_onCadDone)(int, int64_t, uint8_t);           ///< function pointer wich is called on cad done
  void (*_onCadDetected)(int, int64_t);                ///< function pointer wich is called on cad detected
  void (*_onValidHeader)(int, int64_t);                ///< function pointer wich is called on valid message header
  void (*_onRxTimeout)(int, int64_t);                  ///< function pointer wich is called on receive timeout


public:
  static int gbl_instance;                       ///< global instance counter variable
  int instance = 0;                              ///< current instance
  
  //used in derefferd interrupt task
  inline static TaskHandle_t isrServiceTaskHndl; ///< task handle for the seperate dereffered interrupt task
  inline static QueueHandle_t isrTaskQueue;      ///< seperate queue to interface with the dereffered interrupt task
  void handleDio0Rise(int64_t, uint8_t, uint8_t); ///< @brief handle dio rise function

};

extern LoRaModemSX127x LoRaExt[12];

#endif
