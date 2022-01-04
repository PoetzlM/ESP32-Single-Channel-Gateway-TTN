//Simple Lib to define basic datatypes of the gateway
//needs the Arduino framework
//Version 0.1
//Written by MPO
//License MIT

#ifndef TTNGWDATATYPES_H
#define TTNGWDATATYPES_H

#include <Arduino.h>

//see https://lora-developers.semtech.com/documentation/tech-papers-and-guides/the-book/packet-size-considerations
#define TTN_MAX_PAYLOAD 222

/**
 * enum for the udp semtech protocoll.
 * used in the pull response packet
 */
typedef enum TxAckError
{

  ERROR_UNDEFINED = -1, ///<  undefined state, used to mark conversation errors from or to the enum
  ERROR_NONE = 0,       ///<  Packet has been programmed for downlink
  TOO_LATE = 1,         ///<  Rejected because it was already too late to program this packet for downlink
  TOO_EARLY = 2,        ///<  Rejected because downlink packet timestamp is too much in advance
  COLLISION_PACKET = 3, ///<  Rejected because there was already a packet programmed in requested timeframe
  COLLISION_BEACON = 4, ///<  Rejected because there was already a beacon planned in requested timeframe
  TX_FREQUENCY = 5,     ///<  Rejected because requested frequency is not supported by TX RF chain
  TX_POWER = 6,         ///<  Rejected because requested power is not supported by gateway
  GPS_UNLOCK = 7        ///<  Rejected because GPS is unlocked, so GPS timestamp cannot be used

} TxAckError_t;

/**
* @brief convert TxAckError_t enum value to c-string representation
*
* @param value enum value to be converted
*
* @return constant c-string as the written representation of the enum.
*
* @exceptsafe This function does not throw exceptions.
*/
const char *TxAckErrorToString(TxAckError_t value);

/**
 * enum to represent the configured spreading factor.
 */
typedef enum LoRaSpreadingFactor
{
  SF_UNDEFINED = -2,
  SF_ALL = -1,
  SF_MIN = 6,
  SF6 = SF_MIN,
  SF7 = 7,
  SF8 = 8,
  SF9 = 9,
  SF10 = 10,
  SF11 = 11,
  SF_MAX = 12,
  SF12 = SF_MAX
} LoRaSpreadingFactor_t;

/**
* @brief convert LoRaSpreadingFactor_t to integer representation
*
* @param value enum value to be converted
*
* @return integer representation of the enum entry.
*
* @exceptsafe This function does not throw exceptions.
*/
int LoRaSpreadingFactorToInt(LoRaSpreadingFactor_t value);

/**
* @brief convert integer to LoRaSpreadingFactor_t representation
*
* @param value integer value to be converted
*
* @return enum representation of the enum entry.
*
* @exceptsafe This function does not throw exceptions.
*/
LoRaSpreadingFactor_t Int2LoRaSpreadingFactor(int value);

/**
* @brief convert LoRaSpreadingFactor_t to c-string representation
*
* @param value enum value to be converted
*
* @return c-string representation of the enum entry.
*
* @exceptsafe This function does not throw exceptions.
*/
const char *LoRaSpreadingFactorToString(LoRaSpreadingFactor_t value);

/**
* @brief convert c-string to LoRaSpreadingFactor_t representation
*
* @param value c-string to be converted
*
* @return LoRaSpreadingFactor_t enum representation of the c-string.
*
* @exceptsafe This function does not throw exceptions.
*/
LoRaSpreadingFactor_t StringToLoRaSpreadingFactor(char *value);

/**
 * enum to represent the configured modulation.
 */
typedef enum LoRaModulation
{
  LRM_UNDEFINED = -1,
  LORA = 0,
  FSK = 1
} LoRaModulation_t;

/**
* @brief convert LoRaModulation_t to c-string representation
*
* @param value enum value to be converted
*
* @return c-string representation of the LoRaModulation_t value.
*
* @exceptsafe This function does not throw exceptions.
*/
const char *LoRaModulationToString(LoRaModulation_t value);

/**
* @brief convert c-string to LoRaModulation_t representation
*
* @param value c-string to be converted
*
* @return LoRaModulation_t representation of the c-string value.
*
* @exceptsafe This function does not throw exceptions.
*/

LoRaModulation_t StringToLoRaModulation(char *value);

/**
* @brief convert const c-string to LoRaModulation_t representation
*
* @param value c-string to be converted
*
* @return LoRaModulation_t representation of the c-string value.
*
* @exceptsafe This function does not throw exceptions.
*/
LoRaModulation_t StringToLoRaModulation(const char *value);

/**
 * enum to represent the configured crc setting.
 */
typedef enum LoRaCrcConfig
{
  CRC_OK = 1,
  CRC_FAIL = -1,
  CRC_OFF = 0
} LoRaCrcConfig_t;

/**
 * enum to represent the configured coding rate.
 * default is 4_5
 */
typedef enum LoRaCodingRate
{
  LCR_UNDEFINED = -1,
  CR_MIN = 5,
  CR_4_5 = CR_MIN,
  CR_4_6 = 6,
  CR_4_7 = 7,
  CR_MAX = 8,
  CR_4_8 = CR_MAX
} LoRaCodingRate_t;

/**
* @brief convert LoRaCodingRate_t to integer representation
*
* @param value enum value to be converted
*
* @return integer representation of the enum entry.
*
* @exceptsafe This function does not throw exceptions.
*/
int LoRaCodingRateToInt(LoRaCodingRate_t value);

/**
* @brief convert integer to LoRaCodingRate_t representation
*
* @param integer value to be converted
*
* @return renum representation of the integer value.
*
* @exceptsafe This function does not throw exceptions.
*/
LoRaCodingRate_t IntToLoRaCodingRate(int value);

/**
* @brief convert LoRaCodingRate_t to c-string representation
*
* @param value enum value to be converted
*
* @return c-string representation of the enum entry.
*
* @exceptsafe This function does not throw exceptions.
*/
const char *LoRaCodingRateToString(LoRaCodingRate_t value);

/**
* @brief convert c-string to LoRaCodingRate_t representation
*
* @param value c-string to be converted
*
* @return enum representation of the c-string representation.
*
* @exceptsafe This function does not throw exceptions.
*/
LoRaCodingRate_t StringToLoRaCodingRate(char *value);

/**
* @brief convert const c-string to LoRaCodingRate_t representation
*
* @param value c-string to be converted
*
* @return enum representation of the c-string representation.
*
* @exceptsafe This function does not throw exceptions.
*/
LoRaCodingRate_t StringToLoRaCodingRate(const char *value);

/**
 * struct to collect all gw statistical data.
 * used in the semtech udp protocol.
 */
typedef struct LoRaGwStatistic
{

  uint32_t rxnb; ///< Number of radio packets received (unsigned integer)
  uint32_t rxok; ///< Number of radio packets received with a valid PHY CRC
  uint32_t rxfw; ///< Number of radio packets forwarded (unsigned integer)
  uint32_t ackr; ///< Percentage of upstream datagrams that were acknowledged
  uint32_t dwnb; ///< Number of downlink datagrams received (unsigned integer)
  uint32_t txnb; ///< Number of packets emitted (unsigned integer)
  char time_expanded[30];

  //out of spec, helper and debugging information
  uint32_t rxnok;      ///< Number of radio packets received with a defectiv PHY CRC
  uint32_t rxcrcoff;   ///< Number of radio packets received with disabled PHY CRC
  uint32_t ackr_nom;   ///< Number of actual upstream datagrams
  uint32_t ackr_denom; ///< Number of acknowledged datagrams

} LoRaGwStatistic_t;

/**
 * struct to collect all information regarding the received lora packet
 */
typedef struct LoRaDataPktRx
{
  char time_compact[30];
  uint32_t gps_time_rx;
  uint32_t timestamp_rx;
  double freq_mhz;
  uint16_t if_channel;
  uint16_t rf_chain;
  LoRaCrcConfig_t crc;
  LoRaModulation_t modulation;

  LoRaSpreadingFactor_t spreadFact;
  double bandwidth_khz;
  uint16_t fsk_datarate;

  LoRaCodingRate_t codingRate;
  float rssi;
  float snr;
  uint32_t msg_len;
  uint8_t msg[TTN_MAX_PAYLOAD];

} LoRaDataPktRx_t;

/**
 * struct to collect all information for the transmission of the lora packet
 */
typedef struct LoRaDataPktTx
{
  bool immediat;
  uint32_t gps_time_tx;
  uint32_t timestamp_tx;
  uint32_t unix_timestamp;
  double freq_mhz;
  uint16_t if_channel;
  uint16_t rf_chain;
  uint32_t tx_power;
  LoRaModulation_t modulation; ///< LoRa or FSK Mode

  LoRaSpreadingFactor_t spreadFact;
  double bandwidth_khz;
  uint16_t fsk_datarate;

  LoRaCodingRate_t codingRate;
  float fsk_dev;
  bool modul_pol_inv;
  uint16_t preamble;
  uint32_t msg_len;
  uint8_t msg[TTN_MAX_PAYLOAD];

  bool no_crc;

} LoRaDataPktTx_t;

/**
 * struct to collect possition data
 * seperated in a extra struct to use it later with an actual gps receiver
 */
typedef struct LoRaGwGeoData
{
  float latitude;
  float longitude;
  int altitude_meters;

} LoRaGwGeoData_t;

/**
 * struct for basic gateway identification
 */
typedef struct LoRaGwInfo
{

  char identifier[10]; ///< Gateway EUI

  char platform_definition[25];  ///< out of spec, but well knowen additional information, simple name of the device (without spaces)
  char platform_email[41];       ///< out of spec, but well knowen additional information, email adress of the owner
  char platform_description[65]; ///< out of spec, but well knowen additional information, description of the device (with spaces)

} LoRaGwInfo_t;

#endif
