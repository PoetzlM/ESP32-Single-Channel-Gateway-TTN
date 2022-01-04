//Simple Lib to parse pakets in udp semtec protcol style
//for mor information see:
//https://github.com/Lora-net/packet_forwarder/blob/master/PROTOCOL.TXT
//needs the Arduino framework and the ArduinoJson framework
//Version 0.1
//Written by MPO
//License MIT

#ifndef TTNGWUDPPARSER_H
#define TTNGWUDPPARSER_H

#include <Arduino.h>
#include <ArduinoJson.h>
#include <arduino_base64.hpp>
#include "TtnGwDataTypes.h"
#include "TtnGwTime.h"

// define section
#define PROTOCOL_VERSION 2

#define PCKG_PUSH_DATA 0x00
#define PCKG_PUSH_ACK 0x01
#define PCKG_PULL_DATA 0x02
#define PCKG_PULL_RESP 0x03
#define PCKG_PULL_ACK 0x04
#define PCKG_TX_ACK 0x05

#define TTNUDPPACKAGEBUFFER 2048
#define JSONBUFFER 1024
// define section end

/**
 * struct to hold the correct formatted udp msg wich should be sent to the ttn server.
 */
typedef struct TtnUdpPacketOut
{
  char outBuffer[TTNUDPPACKAGEBUFFER]; ///< Buffer wich holds the actual message
  uint32_t outMsgLen;                  ///< length of actual message, how manny bytes are used of the outBuffer
  uint32_t retrys;                     ///< no of retrys of sending the information to the server
} TtnUdpPacketOut_t;

/**
* @brief convert the DataRateIdentifier as c-string provided by the semtech udp protocoll to only the bandwith as integer
*
* @return integer representation of the bandwidth
*
* @exceptsafe This function does not throw exceptions.
*/
int convertDataRateIdentifierToBandwidth(char *value);

/**
* @brief convert the DataRateIdentifier as const c-string provided by the semtech udp protocoll to only the bandwith as integer.
*
* @return integer representation of the bandwidth.
*
* @exceptsafe This function does not throw exceptions.
*/
int convertDataRateIdentifierToBandwidth(const char *value);

/**
* @brief convert the DataRateIdentifier as c-string provided by the semtech udp protocoll to only the spreadingfactor.
*
* @return integer representation of the spreading factor.
*
* @exceptsafe This function does not throw exceptions.
*/
LoRaSpreadingFactor_t convertDataRateIdentifierToSpreadFact(char *value);

/**
* @brief convert the DataRateIdentifier as const c-string provided by the semtech udp protocoll to only the spreadingfactor.
*
* @return integer representation of the spreading factor.
*
* @exceptsafe This function does not throw exceptions.
*/
LoRaSpreadingFactor_t convertDataRateIdentifierToSpreadFact(const char *value);

/**
 * class to parse and process the incomming and outgoing data packets
 */
class TtnGwUdpParser
{
private:
  StaticJsonDocument<JSONBUFFER> _jsonDoc; ///< temporary buffer ot parse the json string
  char _serializedJsonDoc[JSONBUFFER];     ///< temporary buffer to create the json string
  char _b64[TTN_MAX_PAYLOAD];              ///< temporary buffer for the b64 encoded data

  char tempDataRateString[20]; ///< temporary buffer to hold the datarate and bandwidth encoding

  uint8_t _inTokenH; ///< the last received token (highbyte)
  uint8_t _inTokenL; ///< the last received token (lowbyte)

  uint8_t _outTokenH; ///< the last generated token for the output message (highbyte)
  uint8_t _outTokenL; ///< the last generated token for the output message (lowbyte)

  LoRaGwInfo_t _gwInfo;           ///< contains all the gateway descriptiv data
  LoRaGwStatistic_t _gwStatistic; ///< contains all information about recieved and transmited packages
  LoRaGwGeoData_t _gwGeoData;     ///< separeate data structur with the information of the geo position

public:
  // general information how much data can be read in one go
  int bufferSize = TTNUDPPACKAGEBUFFER; ///< maximum buffer size
	/// shared data structs with other interfaces
  char inBuffer[TTNUDPPACKAGEBUFFER];   ///< temporary input buffer
  char outBuffer[TTNUDPPACKAGEBUFFER];  ///< temporary output buffer
  uint32_t outMsgLen;                   ///< used size of the output buffer
  uint32_t inMsgLen;                    ///< used size of the input buffer

  LoRaDataPktTx_t loraDataPacketTx; ///< temporary struct to hold all necessary data for the upstream message
  LoRaDataPktRx_t loraDataPacketRx; ///< temporary struct to hold all necessary data for the downstream message
  
private:
  /**
  * @brief generate udp data rate identifiere from LoRaSpreadingFactor_t and integere representation of bandwidth in khz
  *
  * @return c-string pointer to the generated c-string
  *
  * @exceptsafe This function does not throw exceptions.
  */
  char *_createDataRateIdentifier(LoRaSpreadingFactor_t spreadFact, int bandwidth_khz);

public:
  /**
  * @brief constructor of TtnGwUdpParser
  *
  * @return nothing
  *
  * @exceptsafe This function does not throw exceptions.
  */
  TtnGwUdpParser();


  /**
  * @brief print gateway identifier to serial interface
  *
  * @return nothing
  *
  * @exceptsafe This function does not throw exceptions.
  */
  void printIdentifier();

  /**
  * @brief generate the transmit acknowledge message
  *
  * @param error of type TxAckError_t to set the additional information field
  *
  * @return nothing
  *
  * @exceptsafe This function does not throw exceptions.
  */
  void genTxAckMsg(TxAckError_t error);

  /**
  * @brief parse input buffer for the pull response message
  *
  * @return true if message type is pull response message
  *
  * @exceptsafe This function does not throw exceptions.
  */
  bool parsePullRespMsg();

  /**
  * @brief parse input buffer for the pull acknowledge message
  *
  * @return true if message type is pull acknowledge message
  *
  * @exceptsafe This function does not throw exceptions.
  */
  bool parsePullAckMsg();

  /**
  * @brief generate pull message and put it in the output buffer
  *
  * @return nothing
  *
  * @exceptsafe This function does not throw exceptions.
  */
  void genPullDataMsg();

  /**
  * @brief generate status message and put it in the output buffer
  *
  * @return nothing
  *
  * @exceptsafe This function does not throw exceptions.
  */
  void genStatMsg();

  /**
  * @brief parse input buffer for the push acknowledge message
  *
  * @return true if message type is push acknowledge message
  *
  * @exceptsafe This function does not throw exceptions.
  */
  bool parsePushAckMsg();

  /**
  * @brief generate psuh message and put it in the output buffer
  *
  * @return nothing
  *
  * @exceptsafe This function does not throw exceptions.
  */
  void genPushDataMsg();

  /**
  * @brief generate unique identifiere from 8-unsigned char array
  *
  * @return nothing
  *
  * @exceptsafe This function does not throw exceptions.
  */
  void setUniqueIdentifier(uint8_t *ident);

  /**
  * @brief generate unique identifiere from 6-unsigned char array, like ethernet mac adress
  *
  * @return nothing
  *
  * @exceptsafe This function does not throw exceptions.
  */
  void setUniqueIdentFromMacEth(byte *mac);

  /**
  * @brief check if the last received token matches the given tokens
  *
  * @return true if the tokens match, else false
  *
  * @exceptsafe This function does not throw exceptions.
  */
  bool compareToken(uint8_t TokenL, uint8_t TokenH);

  /**
  * @brief update the LoRaDataPktRx_t datastruct for the received lora message
  *
  * @return nothing
  *
  * @exceptsafe This function does not throw exceptions.
  */
  void update(LoRaDataPktRx_t &data);

  /**
  * @brief update the LoRaDataPktTx_t datastruct for the lora message to be transmitted
  *
  * @return nothing
  *
  * @exceptsafe This function does not throw exceptions.
  */
  void update(LoRaDataPktTx_t &data);

  /**
  * @brief update the LoRaGwInfo_t datastruct to use the newest values for the status update message
  *
  * @return nothing
  *
  * @exceptsafe This function does not throw exceptions.
  */
  void update(LoRaGwInfo_t &data);

  /**
  * @brief update the LoRaGwStatistic_t datastruct to use the newest values for the status update message
  *
  * @return nothing
  *
  * @exceptsafe This function does not throw exceptions.
  */
  void update(LoRaGwStatistic_t &data);

  /**
  * @brief update the LoRaGwGeoData_t datastruct to use the newest values for the status update message
  *
  * @return nothing
  *
  * @exceptsafe This function does not throw exceptions.
  */
  void update(LoRaGwGeoData_t &data);

  /**
  * @brief update the input buffer from the udp socket, if the input buffer is updated, messages can be parsed out
  *
  * @return nothing
  *
  * @exceptsafe This function does not throw exceptions.
  */
  void updateInBuffer(char *buffer, int len);

  /**
  * @brief get pointer to the outbuffer. Carefull this function is not threadsafe!
  *
  * @return nothing
  *
  * @exceptsafe This function does not throw exceptions.
  */
  char *getOutBuffer();

  /**
  * @brief get the used length of the output buffer.
  *
  * @return nothing
  *
  * @exceptsafe This function does not throw exceptions.
  */
  int getOutBufferLen();

  /**
  * @brief coppy the generated out message to a seperate datastructure.
  * function is more save than @see getOutBuffer
  *
  * @return nothing
  *
  * @exceptsafe This function does not throw exceptions.
  */
  void getOutMsg(TtnUdpPacketOut_t *outDataStruct);
};

#endif
