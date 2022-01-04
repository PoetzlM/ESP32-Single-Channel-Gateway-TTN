//Simple Lib to define basic datatypes of the gateway
//needs the Arduino framework
//Version 0.1
//Written by MPO
//License MIT

#include "TtnGwDataTypes.h"

/**
* @brief convert TxAckError_t enum value to c-string representation
*
* @param value enum value to be converted
*
* @return constant c-string as the written representation of the enum.
*
* @exceptsafe This function does not throw exceptions.
*/
const char *TxAckErrorToString(TxAckError_t value)
{
  switch (value)
  {
  case TxAckError_t::ERROR_NONE:
    return "NONE";
  case TxAckError_t::TOO_LATE:
    return "TOO_LATE";
  case TxAckError_t::TOO_EARLY:
    return "TOO_EARLY";
  case TxAckError_t::COLLISION_PACKET:
    return "COLLISION_PACKET";
  case TxAckError_t::COLLISION_BEACON:
    return "COLLISION_BEACON";
  case TxAckError_t::TX_FREQUENCY:
    return "TX_FREQUENCY";
  case TxAckError_t::TX_POWER:
    return "TX_POWER";
  case TxAckError_t::GPS_UNLOCK:
    return "GPS_UNLOCK";
  default:
    return "ERROR_UNDEFINED";
  }
  return "ERROR_UNDEFINED";
}

/**
* @brief convert LoRaSpreadingFactor_t to integer representation
*
* @param value enum value to be converted
*
* @return integer representation of the enum entry.
*
* @exceptsafe This function does not throw exceptions.
*/
int LoRaSpreadingFactorToInt(LoRaSpreadingFactor_t value)
{
  switch (value)
  {
  case LoRaSpreadingFactor_t::SF6:
    return 6;
  case LoRaSpreadingFactor_t::SF7:
    return 7;
  case LoRaSpreadingFactor_t::SF8:
    return 8;
  case LoRaSpreadingFactor_t::SF9:
    return 9;
  case LoRaSpreadingFactor_t::SF10:
    return 10;
  case LoRaSpreadingFactor_t::SF11:
    return 11;
  case LoRaSpreadingFactor_t::SF12:
    return 12;
  default:
    return -2;
  }
  return -2;
}

/**
* @brief convert integer to LoRaSpreadingFactor_t representation
*
* @param value integer value to be converted
*
* @return enum representation of the enum entry.
*
* @exceptsafe This function does not throw exceptions.
*/
LoRaSpreadingFactor_t Int2LoRaSpreadingFactor(int value)
{
  switch (value)
  {
  case 6:
    return LoRaSpreadingFactor_t::SF6;
  case 7:
    return LoRaSpreadingFactor_t::SF7;
  case 8:
    return LoRaSpreadingFactor_t::SF8;
  case 9:
    return LoRaSpreadingFactor_t::SF9;
  case 10:
    return LoRaSpreadingFactor_t::SF10;
  case 11:
    return LoRaSpreadingFactor_t::SF11;
  case 12:
    return LoRaSpreadingFactor_t::SF12;
  default:
    return LoRaSpreadingFactor_t::SF_UNDEFINED;
  }
  return LoRaSpreadingFactor_t::SF_UNDEFINED;
}

/**
* @brief convert LoRaSpreadingFactor_t to c-string representation
*
* @param value enum value to be converted
*
* @return c-string representation of the enum entry.
*
* @exceptsafe This function does not throw exceptions.
*/
const char *LoRaSpreadingFactorToString(LoRaSpreadingFactor_t value)
{
  switch (value)
  {
  case LoRaSpreadingFactor_t::SF6:
    return "SF6";
  case LoRaSpreadingFactor_t::SF7:
    return "SF7";
  case LoRaSpreadingFactor_t::SF8:
    return "SF8";
  case LoRaSpreadingFactor_t::SF9:
    return "SF9";
  case LoRaSpreadingFactor_t::SF10:
    return "SF10";
  case LoRaSpreadingFactor_t::SF11:
    return "SF11";
  case LoRaSpreadingFactor_t::SF12:
    return "SF12";
  default:
    return "SF_UNDEFINED";
  }
  return "SF_UNDEFINED";
}

/**
* @brief convert c-string to LoRaSpreadingFactor_t representation
*
* @param value c-string to be converted
*
* @return LoRaSpreadingFactor_t enum representation of the c-string.
*
* @exceptsafe This function does not throw exceptions.
*/
LoRaSpreadingFactor_t StringToLoRaSpreadingFactor(char *value)
{

  if (strncmp(value, "SF6", 3) == 0)
  {
    return LoRaSpreadingFactor_t::SF6;
  }
  if (strncmp(value, "SF7", 3) == 0)
  {
    return LoRaSpreadingFactor_t::SF7;
  }
  if (strncmp(value, "SF8", 3) == 0)
  {
    return LoRaSpreadingFactor_t::SF8;
  }
  if (strncmp(value, "SF9", 3) == 0)
  {
    return LoRaSpreadingFactor_t::SF9;
  }
  if (strncmp(value, "SF10", 4) == 0)
  {
    return LoRaSpreadingFactor_t::SF10;
  }
  if (strncmp(value, "SF11", 4) == 0)
  {
    return LoRaSpreadingFactor_t::SF11;
  }
  if (strncmp(value, "SF12", 4) == 0)
  {
    return LoRaSpreadingFactor_t::SF12;
  }

  return LoRaSpreadingFactor_t::SF_UNDEFINED;
}

/**
* @brief convert LoRaModulation_t to c-string representation
*
* @param value enum value to be converted
*
* @return c-string representation of the LoRaModulation_t value.
*
* @exceptsafe This function does not throw exceptions.
*/
const char *LoRaModulationToString(LoRaModulation_t value)
{
  switch (value)
  {
  case LoRaModulation_t::LORA:
    return "LORA";
  case LoRaModulation_t::FSK:
    return "FSK";
  default:
    return "LRM_UNDEFINED";
  }
}

/**
* @brief convert c-string to LoRaModulation_t representation
*
* @param value c-string to be converted
*
* @return LoRaModulation_t representation of the c-string value.
*
* @exceptsafe This function does not throw exceptions.
*/

LoRaModulation_t StringToLoRaModulation(char *value)
{

  if (strncmp(value, "LOR", 3) == 0)
  {
    return LoRaModulation_t::LORA;
  }
  if (strncmp(value, "FSK", 3) == 0)
  {
    return LoRaModulation_t::FSK;
  }

  return LoRaModulation_t::LRM_UNDEFINED;
}

/**
* @brief convert const c-string to LoRaModulation_t representation
*
* @param value c-string to be converted
*
* @return LoRaModulation_t representation of the c-string value.
*
* @exceptsafe This function does not throw exceptions.
*/
LoRaModulation_t StringToLoRaModulation(const char *value)
{

  if (strncmp(value, "LOR", 3) == 0)
  {
    return LoRaModulation_t::LORA;
  }
  if (strncmp(value, "FSK", 3) == 0)
  {
    return LoRaModulation_t::FSK;
  }

  return LoRaModulation_t::LRM_UNDEFINED;
}

/**
* @brief convert LoRaCodingRate_t to integer representation
*
* @param value enum value to be converted
*
* @return integer representation of the enum entry.
*
* @exceptsafe This function does not throw exceptions.
*/
int LoRaCodingRateToInt(LoRaCodingRate_t value)
{
  switch (value)
  {
  case LoRaCodingRate_t::CR_4_5:
    return 5;
  case LoRaCodingRate_t::CR_4_6:
    return 6;
  case LoRaCodingRate_t::CR_4_7:
    return 7;
  case LoRaCodingRate_t::CR_4_8:
    return 8;
  default:
    return -1;
  }
}

/**
* convert integer to LoRaCodingRate_t representation
*
* @param integer value to be converted
*
* @return renum representation of the integer value.
*
* @exceptsafe This function does not throw exceptions.
*/
LoRaCodingRate_t IntToLoRaCodingRate(int value)
{
  switch (value)
  {
  case 5:
    return LoRaCodingRate_t::CR_4_5;
  case 6:
    return LoRaCodingRate_t::CR_4_6;
  case 7:
    return LoRaCodingRate_t::CR_4_7;
  case 8:
    return LoRaCodingRate_t::CR_4_8;
  default:
    return LoRaCodingRate_t::LCR_UNDEFINED;
  }
}

/**
* convert LoRaCodingRate_t to c-string representation
*
* @param value enum value to be converted
*
* @return c-string representation of the enum entry.
*
* @exceptsafe This function does not throw exceptions.
*/
const char *LoRaCodingRateToString(LoRaCodingRate_t value)
{
  switch (value)
  {
  case LoRaCodingRate_t::CR_4_5:
    return "4/5";
  case LoRaCodingRate_t::CR_4_6:
    return "4/6";
  case LoRaCodingRate_t::CR_4_7:
    return "4/7";
  case LoRaCodingRate_t::CR_4_8:
    return "4/8";
  default:
    return "LCR_UNDEFINED";
  }
}

/**
* convert c-string to LoRaCodingRate_t representation
*
* @param value c-string to be converted
*
* @return enum representation of the c-string representation.
*
* @exceptsafe This function does not throw exceptions.
*/
LoRaCodingRate_t StringToLoRaCodingRate(char *value)
{
  if (strncmp(value, "4/5", strlen("4/5")) == 0)
  {
    return LoRaCodingRate_t::CR_4_5;
  }

  if (strncmp(value, "4/6", strlen("4/6")) == 0)
  {
    return LoRaCodingRate_t::CR_4_6;
  }

  if (strncmp(value, "4/7", strlen("4/7")) == 0)
  {
    return LoRaCodingRate_t::CR_4_7;
  }

  if (strncmp(value, "4/8", strlen("4/8")) == 0)
  {
    return LoRaCodingRate_t::CR_4_8;
  }

  return LoRaCodingRate_t::LCR_UNDEFINED;
}

/**
* convert const c-string to LoRaCodingRate_t representation
*
* @param value c-string to be converted
*
* @return enum representation of the c-string representation.
*
* @exceptsafe This function does not throw exceptions.
*/
LoRaCodingRate_t StringToLoRaCodingRate(const char *value)
{
  if (strncmp(value, "4/5", strlen("4/5")) == 0)
  {
    return LoRaCodingRate_t::CR_4_5;
  }

  if (strncmp(value, "4/6", strlen("4/6")) == 0)
  {
    return LoRaCodingRate_t::CR_4_6;
  }

  if (strncmp(value, "4/7", strlen("4/7")) == 0)
  {
    return LoRaCodingRate_t::CR_4_7;
  }

  if (strncmp(value, "4/8", strlen("4/8")) == 0)
  {
    return LoRaCodingRate_t::CR_4_8;
  }

  return LoRaCodingRate_t::LCR_UNDEFINED;
}
