/*=======================================================================
Lib to interface with a SX127x modem. It setus up automated callbacks to 
implement CAD and respect the use of the air channel (cad implementation
reduces crosstalk). If the air channel is free it can be used to transmit 
data. Implements automatic modem parameter change and restoration
(parameters for transmitting and receiving can be diffrent). If data is 
received it can be offloaded to seperate tasks. All information wich are 
required by the semtech udp protocoll will be collected and straped 
together. 

Requirements:
  - esp-idf
  - arduino framework
  - freertos
  - LoRaModemSX127x.h
Version: 0.1
Author: MPO
Date: 30-21-2021
=======================================================================*/

#ifndef LORAMODEMSX127XHANDLER_H
#define LORAMODEMSX127XHANDLER_H

#include <Arduino.h>

#include "LoRaModemSX127x.h"
#include "TtnGwDataTypes.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_task_wdt.h"

#define MAX_SIZE_IRQ_MODEMHANDLER 12
//#include <arduino_base64.hpp>

/**
 * struct to communicate in to the CadDone dereffered isr
 */
typedef struct OnCadDoneDataIn
{
  uint8_t instance;
  bool txRequest;

} OnCadDoneDataIn_t;

/**
 * struct to communicate out of the CadDone dereffered isr
 */
typedef struct OnCadDoneDataOut
{
  uint8_t instance;
  bool txReady;

} OnCadDoneDataOut_t;

/**
 * struct to communicate out of the OnReceive dereffered isr
 */
typedef struct OnReceiveDataOut
{
  uint8_t instance;
  uint32_t msg_len;
  uint32_t rx_timestamp;
  bool crcOk;
  bool headerOk;
  ModemConfig_t modemCfg;

} OnReceiveDataOut_t;

/**
 * struct to communicate in to the TxDone dereffered isr
 */
typedef struct OnTxDoneDataIn
{
  uint8_t instance;
  ModemConfig_t modemCfg;

} OnTxDoneDataIn_t;

/**
 * class to setup the callbacks to the SX127x modem
 */
class LoRaModemSX127xHandler
{
private:
  ModemConfig_t _modemTxConfig; ///< temporary modem configuration

  int _cpyInstance; ///< copy of the registerd instance of the modem, will be used to interface with the correct modem

  inline static QueueHandle_t _queueOnCadDoneOut;    ///< queue to communicate out of CadDone isr task
  inline static QueueHandle_t _queueOnCadDoneIn;     ///< queue to communicate in the CadDone isr task
  inline static QueueHandle_t _queueOnRecieveOut;    ///< queue to communicate out of OnRecieve isr task
  inline static QueueHandle_t _queueOnTxDoneIn;      ///< queue to communicate in the TxDone isr task
  

  int _availTxFrequency[20];      ///< available range of acceptable tx frequencies
  uint8_t _availSpreadFactTx[20]; ///< available range of acceptable spreading factors

  

  uint16_t _if_channel; ///< descriptiv information interface channel
  uint16_t _rf_chain;   ///< descriptiv information radio frequency chain

  bool _ignoreIfChannel = false;    ///< configuration of the transmit acceptance filters, filter by interface channel
  bool _ignoreRfChain = false;      ///< configuration of the transmit acceptance filters, filter by radio frequency chain
  bool _ignoreTxPower = false;      ///< configuration of the transmit acceptance filters, ignore tx power in request
  bool _ignorePolarization = false; ///< configuration of the transmit acceptance filters, ignor polarization in request
  bool _disableAllTxFlt = false;    ///< configuration of the transmit acceptance filters, disable all tx filters
  bool _noTx = false;               ///< configuration of the transmit acceptance filters, disable all transmissions

private:
  // static interrupt functions, extension of the critical area.
  // communication out an in this function should only take place via queues
  // extension of the critical area
  static void _onReceive0(int instance, int64_t deltaTicks, uint32_t packetLength, uint8_t irqFlags); ///< @brief registerd callback for onReceive
  static void _onCadDetect0(int instance, int64_t deltaTicks);                                        ///< @brief registerd callback for CadDetect
  static void _onCadDone0(int instance, int64_t deltaTicks, uint8_t irqFlags);                        ///< @brief registerd callback for CadDone
  static void _onRxTout0(int instance, int64_t deltaTicks);                                           ///< @brief registerd callback for RxTout
  static void _onTxDone0(int instance, int64_t deltaTicks);                                           ///< @brief registerd callback for TxDone

public:
  // structures exposed so the timer interrupt can access it.
  inline static LoRaModemSX127x *interface[12]; ///< global instace holder to make it available in the dereffered isr
  LoRaDataPktRx_t loraDataPacketRx; ///< temporary data collection of received data
  inline static QueueHandle_t queueTimerInterruptIn; ///< queue to communicate in the Timer Interrupt isr task
  
  uint32_t msg_len;             ///< message length
  uint8_t msg[TTN_MAX_PAYLOAD]; ///< actual message

public:
  /**
  * @brief constructor for LoRaModemSX127xHandler, setup the instance number.
  *
  * @exceptsafe This function does not throw exceptions.
  */
  LoRaModemSX127xHandler();

  /**
  * @brief initialize modem on an specific frequency
  *
  * @param interface wich instance of LoRaModemSX127x should be used
  *
  * @return nothing.
  *
  * @exceptsafe This function does not throw exceptions.
  */
  void begin(LoRaModemSX127x *interface);

  /**
  * @brief process open tasks. This function hast to be called periodicly.
  *
  * @return nothing.
  *
  * @exceptsafe This function does not throw exceptions.
  */
  void process();

  /**
  * @brief configure additional descriptiv information to distinguish diffrent modem instances.
  *
  * @param ifChan interface channel as integer
  * @param rfChain radio frequency chain as integer
  *
  * @return nothing.
  *
  * @exceptsafe This function does not throw exceptions.
  */
  void setAdditionalInfo(uint16_t ifChan, uint16_t rfChain);

  /**
  * @brief configure if transmit filters should be used.
  *
  * @param state true if enabled, false if disabled
  *
  * @return nothing.
  *
  * @exceptsafe This function does not throw exceptions.
  */
  void setIgnorAllTxFilter(bool state);

  /**
  * @brief configure if requestes inteface channel should be ignored.
  *
  * @param state true if enabled, false if disabled
  *
  * @return nothing.
  *
  * @exceptsafe This function does not throw exceptions.
  */
  void setIgnoreReqIfChan(bool state);

  /**
  * @brief configure if requested radio frequency chain should be ignored.
  *
  * @param state true if enabled, false if disabled
  *
  * @return nothing.
  *
  * @exceptsafe This function does not throw exceptions.
  */
  void setIgnoreReqRfChain(bool state);

  /**
  * @brief configure if requested transmit power should be ignored.
  *
  * @param state true if enabled, false if disabled
  *
  * @return nothing.
  *
  * @exceptsafe This function does not throw exceptions.
  */
  void setIgnoreReqTxPower(bool state);

  /**
  * @brief configure if requested polarization should be ignored.
  *
  * @param state true if enabled, false if disabled
  *
  * @return nothing.
  *
  * @exceptsafe This function does not throw exceptions.
  */
  void setIgnoreReqPolarization(bool state);

  /**
  * @brief configure if transmission with this instance is disabled.
  *
  * @param state true if enabled, false if disabled
  *
  * @return nothing.
  *
  * @exceptsafe This function does not throw exceptions.
  */
  void setDisableTx(bool state);

  /**
  * @brief configure if filter of availabel transmit frequencies.
  *
  * @param data array of availabel frequencies in Hz.
  * @param noEnt length of the array.
  *
  * @return nothing.
  *
  * @exceptsafe This function does not throw exceptions.
  */
  void setAvailTxFrequency(int *data, int noEnt);

  /**
  * @brief configure if filter of availabel spread factors.
  *
  * @param data array of availabel spread factors.
  * @param noEnt length of the array.
  *
  * @return nothing.
  *
  * @exceptsafe This function does not throw exceptions.
  */
  void setAvailSpreadFactTx(uint8_t *data, int noEnt);

  /**
  * @brief check if the specified spread factor is permitted within this modem handler instance.
  *
  * @param sf requested spreding factor
  *
  * @return nothing.
  *
  * @exceptsafe This function does not throw exceptions.
  */
  bool isSpreadFactTxEnabled(uint8_t sf);

  /**
  * @brief check if the specified frequency is permitted within this modem handler instance.
  *
  * @param sf requested spreding factor
  *
  * @return nothing.
  *
  * @exceptsafe This function does not throw exceptions.
  */
  bool isTxFrequencyEnabled(int frequency);

  /**
  * @brief function is used to que in a transmission between channel detection and receiving a message.
  *
  * @param packetTx packet to be transmitted.
  *
  * @return nothing.
  *
  * @exceptsafe This function does not throw exceptions.
  */
  bool requestTransmit(LoRaDataPktTx_t *packetTx);
};

#endif
