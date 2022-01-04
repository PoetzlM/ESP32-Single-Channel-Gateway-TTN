//Simple Lib to parse pakets in udp semtec protcol style
//for mor information see:
//https://github.com/Lora-net/packet_forwarder/blob/master/PROTOCOL.TXT
//needs the Arduino framework and the ArduinoJson framework
//Version 0.1
//Written by MPO
//License MIT

#include "TtnGwUdpParser.h"

/**
* @brief convert the DataRateIdentifier as c-string provided by the semtech udp protocoll to only the bandwith as integer
*
* @return integer representation of the bandwidth
*
* @exceptsafe This function does not throw exceptions.
*/
int convertDataRateIdentifierToBandwidth(char *value)
{
	char tempString[20];
	if (value[4] == 'W')
	{
		memcpy(tempString, value + 5, strlen(value) - 5);
		tempString[strlen(value) - 4] = '\0';
		return atoi(tempString);
	}
	if (value[5] == 'W')
	{
		memcpy(tempString, value + 6, strlen(value) - 6);
		tempString[strlen(value) - 5] = '\0';
		return atoi(tempString);
	}
	return 0;
}

/**
* @brief convert the DataRateIdentifier as const c-string provided by the semtech udp protocoll to only the bandwith as integer
*
* @return integer representation of the bandwidth
*
* @exceptsafe This function does not throw exceptions.
*/
int convertDataRateIdentifierToBandwidth(const char *value)
{
	char tempString[20];
	if (value[4] == 'W')
	{
		memcpy(tempString, value + 5, strlen(value) - 5);
		tempString[strlen(value) - 4] = '\0';
		return atoi(tempString);
	}
	if (value[5] == 'W')
	{
		memcpy(tempString, value + 6, strlen(value) - 6);
		tempString[strlen(value) - 5] = '\0';
		return atoi(tempString);
	}
	return 0;
}

/**
* @brief convert the DataRateIdentifier as c-string provided by the semtech udp protocoll to only the spreadingfactor.
*
* @return integer representation of the spreading factor.
*
* @exceptsafe This function does not throw exceptions.
*/
LoRaSpreadingFactor_t convertDataRateIdentifierToSpreadFact(char *value)
{
	return StringToLoRaSpreadingFactor(value);
}

/**
* @brief convert the DataRateIdentifier as const c-string provided by the semtech udp protocoll to only the spreadingfactor.
*
* @return integer representation of the spreading factor.
*
* @exceptsafe This function does not throw exceptions.
*/
LoRaSpreadingFactor_t convertDataRateIdentifierToSpreadFact(const char *value)
{
	char tempString[10];
	memcpy(tempString, value, 4);

	return StringToLoRaSpreadingFactor(tempString);
}

/**
* @brief constructor of TtnGwUdpParser
*
* @return nothing
*
* @exceptsafe This function does not throw exceptions.
*/
TtnGwUdpParser::TtnGwUdpParser()
{
	for (int i = 0; i < TTNUDPPACKAGEBUFFER; i++)
	{
		inBuffer[i] = 0;
		outBuffer[i] = 0;
	}
}

/**
* @brief print gateway identifier to serial interface
*
* @return nothing
*
* @exceptsafe This function does not throw exceptions.
*/
void TtnGwUdpParser::printIdentifier()
{
	Serial.println("GW Unique identifier");
	for (int i = 0; i < 8; i++)
	{
		Serial.print(_gwInfo.identifier[i], HEX);
		Serial.print(".");
	}
	Serial.println();
}

/**
* @brief update the LoRaDataPktRx_t datastruct for the received lora message
*
* @return nothing
*
* @exceptsafe This function does not throw exceptions.
*/
void TtnGwUdpParser::update(LoRaDataPktRx_t &data)
{
	memcpy(&loraDataPacketRx, &data, sizeof(LoRaDataPktRx_t));
}

/**
* @brief update the LoRaDataPktTx_t datastruct for the lora message to be transmitted
*
* @return nothing
*
* @exceptsafe This function does not throw exceptions.
*/
void TtnGwUdpParser::update(LoRaDataPktTx_t &data)
{
	memcpy(&loraDataPacketTx, &data, sizeof(LoRaDataPktTx_t));
}

/**
* @brief update the LoRaGwInfo_t datastruct to use the newest values for the status update message
*
* @return nothing
*
* @exceptsafe This function does not throw exceptions.
*/
void TtnGwUdpParser::update(LoRaGwInfo_t &data)
{
	memcpy(&_gwInfo, &data, sizeof(LoRaGwInfo_t));
}

/**
* @brief update the LoRaGwStatistic_t datastruct to use the newest values for the status update message
*
* @return nothing
*
* @exceptsafe This function does not throw exceptions.
*/
void TtnGwUdpParser::update(LoRaGwStatistic_t &data)
{
	memcpy(&_gwStatistic, &data, sizeof(LoRaGwStatistic_t));
}

/**
* @brief update the LoRaGwGeoData_t datastruct to use the newest values for the status update message
*
* @return nothing
*
* @exceptsafe This function does not throw exceptions.
*/
void TtnGwUdpParser::update(LoRaGwGeoData_t &data)
{
	memcpy(&_gwGeoData, &data, sizeof(LoRaGwGeoData_t));
}

/**
* @brief update the input buffer from the udp socket, if the input buffer is updated, messages can be parsed out
*
* @return nothing
*
* @exceptsafe This function does not throw exceptions.
*/
void TtnGwUdpParser::updateInBuffer(char *buffer, int len)
{
	inMsgLen = len;
	memcpy(inBuffer, buffer, sizeof(LoRaGwStatistic_t));
}

/**
* @brief get pointer to the outbuffer. Carefull this function is not threadsafe!
*
* @return nothing
*
* @exceptsafe This function does not throw exceptions.
*/
char *TtnGwUdpParser::getOutBuffer()
{
	return outBuffer;
}

/**
* @brief get the used length of the output buffer.
*
* @return nothing
*
* @exceptsafe This function does not throw exceptions.
*/
int TtnGwUdpParser::getOutBufferLen()
{
	int tempLen = outMsgLen;
	outMsgLen = 0;
	return tempLen;
}

/**
* @brief generate udp data rate identifiere from LoRaSpreadingFactor_t and integere representation of bandwidth in khz
*
* @return c-string pointer to the generated c-string
*
* @exceptsafe This function does not throw exceptions.
*/
char *TtnGwUdpParser::_createDataRateIdentifier(LoRaSpreadingFactor_t spreadFact, int bandwidth_khz)
{
	uint16_t pos = 0;

	memset(tempDataRateString, 0, 20 * (sizeof(tempDataRateString[0])));
	strcpy(tempDataRateString, LoRaSpreadingFactorToString(spreadFact));

	pos += strlen(tempDataRateString);
	tempDataRateString[pos] = 'B';
	pos++;
	tempDataRateString[pos] = 'W';
	pos++;
	itoa(bandwidth_khz, tempDataRateString + pos, 10);

	return tempDataRateString;
}

/**
* @brief generate unique identifiere from 6-unsigned char array, like ethernet mac adress
*
* @return nothing
*
* @exceptsafe This function does not throw exceptions.
*/
void TtnGwUdpParser::setUniqueIdentFromMacEth(byte *mac)
{
	_gwInfo.identifier[0] = mac[0];
	_gwInfo.identifier[1] = mac[1];
	_gwInfo.identifier[2] = mac[2];
	_gwInfo.identifier[3] = 0xFF;
	_gwInfo.identifier[4] = 0xFF;
	_gwInfo.identifier[5] = mac[3];
	_gwInfo.identifier[6] = mac[4];
	_gwInfo.identifier[7] = mac[5];
}

/**
* @brief generate unique identifiere from 8-unsigned char array
*
* @return nothing
*
* @exceptsafe This function does not throw exceptions.
*/
void TtnGwUdpParser::setUniqueIdentifier(uint8_t *ident)
{
	memcpy(_gwInfo.identifier, ident, 8);
}

/**
* @brief generate psuh message and put it in the output buffer
*
* @return nothing
*
* @exceptsafe This function does not throw exceptions.
*/
void TtnGwUdpParser::genPushDataMsg()
{
	/*
That packet type is used by the gateway mainly to forward the RF packets 
received, and associated metadata, to the server.

 Bytes| Function
:------:|---------------------------------------------------------------------
 0| protocol version = 2
 1-2| random token
 3| PUSH_DATA identifier 0x00
 4-11 | Gateway unique identifier (MAC address)
 12-end | JSON object, starting with {, ending with }, see section 4

Upstream Data Structure

That array contains at least one JSON object, each object contain a RF packet 
and associated metadata with the following fields:

 Name |Type| Function
:----:|:------:|--------------------------------------------------------------
 time | string | UTC time of pkt RX, us precision, ISO 8601 'compact' format
 tmms | number | GPS time of pkt RX, number of milliseconds since 06.Jan.1980
 tmst | number | Internal timestamp of "RX finished" event (32b unsigned)
 freq | number | RX central frequency in MHz (unsigned float, Hz precision)
 chan | number | Concentrator "IF" channel used for RX (unsigned integer)
 rfch | number | Concentrator "RF chain" used for RX (unsigned integer)
 stat | number | CRC status: 1 = OK, -1 = fail, 0 = no CRC
 modu | string | Modulation identifier "LORA" or "FSK"
 datr | string | LoRa datarate identifier (eg. spreadFact12BW500)
 datr | number | FSK datarate (unsigned, in bits per second)
 codr | string | LoRa ECC coding rate identifier
 rssi | number | RSSI in dBm (signed integer, 1 dB precision)
 lsnr | number | Lora SNR ratio in dB (signed float, 0.1 dB precision)
 size | number | RF packet payload size in bytes (unsigned integer)
 data | string | Base64 encoded RF packet payload, padded

 {"rxpk":[{ JSON_OBJECT }]}
*/

	_outTokenL = ((uint8_t)rand());
	_outTokenH = ((uint8_t)rand());

	outBuffer[0] = PROTOCOL_VERSION;
	outBuffer[1] = _outTokenL;
	outBuffer[2] = _outTokenH;
	outBuffer[3] = PCKG_PUSH_DATA;
	outMsgLen = 4;

	memcpy(outBuffer + outMsgLen, _gwInfo.identifier, 8);
	outMsgLen += 8;

	strcpy(outBuffer + outMsgLen, "{\"rxpk\":[");
	outMsgLen += 9;

	_jsonDoc.clear();
	_jsonDoc["time"] = loraDataPacketRx.time_compact;
	_jsonDoc["tmms"] = loraDataPacketRx.gps_time_rx;
	_jsonDoc["tmst"] = loraDataPacketRx.timestamp_rx;
	_jsonDoc["freq"] = loraDataPacketRx.freq_mhz;
	_jsonDoc["chan"] = loraDataPacketRx.if_channel;
	_jsonDoc["rfch"] = loraDataPacketRx.rf_chain;
	_jsonDoc["stat"] = loraDataPacketRx.crc;
	_jsonDoc["modu"] = LoRaModulationToString(loraDataPacketRx.modulation);

	if (loraDataPacketRx.modulation == LoRaModulation_t::FSK)
	{
		_jsonDoc["datr"] = loraDataPacketRx.fsk_datarate;
	}

	if (loraDataPacketRx.modulation == LoRaModulation_t::LORA)
	{
		_jsonDoc["datr"] = _createDataRateIdentifier(loraDataPacketRx.spreadFact, loraDataPacketRx.bandwidth_khz);
	}

	_jsonDoc["codr"] = LoRaCodingRateToString(loraDataPacketRx.codingRate);
	_jsonDoc["rssi"] = loraDataPacketRx.rssi;
	_jsonDoc["lsnr"] = loraDataPacketRx.snr;
	_jsonDoc["size"] = loraDataPacketRx.msg_len;

	memset(_b64, 0, TTN_MAX_PAYLOAD * (sizeof(_b64[0])));
	int base64_length = encode_base64(loraDataPacketRx.msg, loraDataPacketRx.msg_len, (unsigned char *)_b64);
	_jsonDoc["data"] = _b64;

	memset(_serializedJsonDoc, 0, JSONBUFFER);
	serializeJson(_jsonDoc, _serializedJsonDoc);

	strcpy(outBuffer + outMsgLen, _serializedJsonDoc);
	outMsgLen += strlen(_serializedJsonDoc);

	outBuffer[outMsgLen] = ']';
	outMsgLen++;
	outBuffer[outMsgLen] = '}';
	outMsgLen++;
}

/**
* @brief parse input buffer for the push acknowledge message
*
* @return true if message type is push acknowledge message
*
* @exceptsafe This function does not throw exceptions.
*/
bool TtnGwUdpParser::parsePushAckMsg()
{
	/*
That packet type is used by the server to acknowledge immediately all the 
PUSH_DATA packets received.

 Bytes| Function
:------:|---------------------------------------------------------------------
 0| protocol version = 2
 1-2| same token as the PUSH_DATA packet to acknowledge
 3| PUSH_ACK identifier 0x01
 */

	if (inBuffer[3] != PCKG_PUSH_ACK)
	{
		return false;
	}
	if (inMsgLen != 4)
	{
		return false;
	}

	_inTokenL = inBuffer[1];
	_inTokenH = inBuffer[2];

	return true;
}

/**
* @brief check if the last received token matches the given tokens
*
* @return true if the tokens match, else false
*
* @exceptsafe This function does not throw exceptions.
*/
bool TtnGwUdpParser::compareToken(uint8_t TokenL, uint8_t TokenH)
{
	return ((_inTokenL == TokenL) && (_inTokenH == TokenH));
}

/**
* @brief generate status message and put it in the output buffer
*
* @return nothing
*
* @exceptsafe This function does not throw exceptions.
*/
void TtnGwUdpParser::genStatMsg()
{
	/*
That packet type is used by the gateway mainly to forward status information of the gateway

 Bytes| Function
:------:|---------------------------------------------------------------------
 0| protocol version = 2
 1-2| random token
 3| PUSH_DATA identifier 0x00
 4-11 | Gateway unique identifier (MAC address)
 12-end | JSON object, starting with {, ending with }, see section 4


Name |Type| Function
:----:|:------:|--------------------------------------------------------------
 time | string | UTC 'system' time of the gateway, ISO 8601 'expanded' format
 lati | number | GPS latitude of the gateway in degree (float, N is +)
 long | number | GPS latitude of the gateway in degree (float, E is +)
 alti | number | GPS altitude of the gateway in meter RX (integer)
 rxnb | number | Number of radio packets received (unsigned integer)
 rxok | number | Number of radio packets received with a valid PHY CRC
 rxfw | number | Number of radio packets forwarded (unsigned integer)
 ackr | number | Percentage of upstream datagrams that were acknowledged
 dwnb | number | Number of downlink datagrams received (unsigned integer)
 txnb | number | Number of packets emitted (unsigned integer)

{"stat":{ JSON_OBJECT }}


 */
	_outTokenL = ((uint8_t)rand());
	_outTokenH = ((uint8_t)rand());

	outBuffer[0] = PROTOCOL_VERSION;
	outBuffer[1] = _outTokenL;
	outBuffer[2] = _outTokenH;
	outBuffer[3] = PCKG_PUSH_DATA;
	outMsgLen = 4;

	memcpy(outBuffer + outMsgLen, _gwInfo.identifier, 8);
	outMsgLen += 8;

	//strcpy(outBuffer + outMsgLen, "{\"stat\":");
	//outMsgLen += 8;

	_jsonDoc.clear();
	_jsonDoc["stat"]["time"] = _gwStatistic.time_expanded;
	_jsonDoc["stat"]["lati"] = _gwGeoData.latitude;
	_jsonDoc["stat"]["long"] = _gwGeoData.longitude;
	_jsonDoc["stat"]["alti"] = _gwGeoData.altitude_meters;
	_jsonDoc["stat"]["rxnb"] = _gwStatistic.rxnb;
	_jsonDoc["stat"]["rxok"] = _gwStatistic.rxok;
	_jsonDoc["stat"]["rxfw"] = _gwStatistic.rxfw;
	_jsonDoc["stat"]["ackr"] = _gwStatistic.ackr;
	_jsonDoc["stat"]["dwnb"] = _gwStatistic.dwnb;
	_jsonDoc["stat"]["txnb"] = _gwStatistic.txnb;

	memset(_serializedJsonDoc, 0, JSONBUFFER);
	serializeJson(_jsonDoc, _serializedJsonDoc);
	strcpy(outBuffer + outMsgLen, _serializedJsonDoc);
	outMsgLen += strlen(_serializedJsonDoc);

	//outMsgLen++;
	//outBuffer[outMsgLen] = '}';

	//proper terminate the strin (if the string is printed for debugging)
	//outBuffer[outMsgLen] = '\0';
}

/**
* @brief generate pull message and put it in the output buffer
*
* @return nothing
*
* @exceptsafe This function does not throw exceptions.
*/
void TtnGwUdpParser::genPullDataMsg()
{
	/*
That packet type is used by the gateway to poll data from the server.

This data exchange is initialized by the gateway because it might be 
impossible for the server to send packets to the gateway if the gateway is 
behind a NAT.

 Bytes| Function
:------:|---------------------------------------------------------------------
 0| protocol version = 2
 1-2| random token
 3| PULL_DATA identifier 0x02
 4-11 | Gateway unique identifier (MAC address)


*/

	_outTokenL = ((uint8_t)rand());
	_outTokenH = ((uint8_t)rand());

	outBuffer[0] = PROTOCOL_VERSION;
	outBuffer[1] = _outTokenL;
	outBuffer[2] = _outTokenH;
	outBuffer[3] = PCKG_PULL_DATA;
	outMsgLen = 4;

	memcpy(outBuffer + outMsgLen, _gwInfo.identifier, 8);
	outMsgLen += 8;

	//proper terminate the strin (if the string is printed for debugging)
	//outBuffer[outMsgLen] = '\0';
}

/**
* @brief parse input buffer for the pull acknowledge message
*
* @return true if message type is pull acknowledge message
*
* @exceptsafe This function does not throw exceptions.
*/
bool TtnGwUdpParser::parsePullAckMsg()
{
	/*
That packet type is used by the server to confirm that the network route is 
open and that the server can send PULL_RESP packets at any time.

 Bytes| Function
:------:|---------------------------------------------------------------------
 0| protocol version = 2
 1-2| same token as the PULL_DATA packet to acknowledge
 3| PULL_ACK identifier 0x04


*/

	if (inBuffer[3] != PCKG_PULL_ACK)
	{
		return false;
	}
	if (inMsgLen != 4)
	{
		return false;
	}

	_inTokenL = outBuffer[1];
	_inTokenH = outBuffer[2];

	return true;
}

/**
* @brief parse input buffer for the pull response message
*
* @return true if message type is pull response message
*
* @exceptsafe This function does not throw exceptions.
*/
bool TtnGwUdpParser::parsePullRespMsg()
{
	/*
That packet type is used by the server to send RF packets and associated 
metadata that will have to be emitted by the gateway.

 Bytes| Function
:------:|---------------------------------------------------------------------
 0| protocol version = 2
 1-2| random token
 3| PULL_RESP identifier 0x03
 4-end| JSON object, starting with {, ending with }


 Name |Type| Function
:----:|:------:|--------------------------------------------------------------
 imme | bool | Send packet immediately (will ignore tmst & time)
 tmst | number | Send packet on a certain timestamp value (will ignore time)
 tmms | number | Send packet at a certain GPS time (GPS synchronization required)
 freq | number | TX central frequency in MHz (unsigned float, Hz precision)
 rfch | number | Concentrator "RF chain" used for TX (unsigned integer)
 powe | number | TX output power in dBm (unsigned integer, dBm precision)
 modu | string | Modulation identifier "LORA" or "FSK"
 datr | string | LoRa datarate identifier (eg. spreadFact12BW500)
 datr | number | FSK datarate (unsigned, in bits per second)
 codr | string | LoRa ECC coding rate identifier
 fdev | number | FSK frequency deviation (unsigned integer, in Hz) 
 ipol | bool | Lora modulation polarization inversion
 prea | number | RF preamble size (unsigned integer)
 size | number | RF packet payload size in bytes (unsigned integer)
 data | string | Base64 encoded RF packet payload, padding optional
 ncrc | bool | If true, disable the CRC of the physical layer (optional)
 
{"txpk": {JSON_OBJECT}}

*/

	if (inMsgLen < 10)
	{
		return false;
	}
	if (inBuffer[3] != PCKG_PULL_RESP)
	{
		return false;
	}

	//txpk string wegschmeißen damit es einfacher zum parsen ist

	//inBuffer[inMsgLen-1] = '\0';

	deserializeJson(_jsonDoc, inBuffer + 4);

	JsonVariant value = _jsonDoc["txpk"]["imme"];
	if (!value.isNull())
	{
		loraDataPacketTx.immediat = _jsonDoc["txpk"]["imme"];
	}

	//internal timestamp in us
	value = _jsonDoc["txpk"]["tmst"];
	if (!value.isNull())
	{
		loraDataPacketTx.timestamp_tx = _jsonDoc["txpk"]["tmst"];
	}
	else
	{
		loraDataPacketTx.timestamp_tx = 0;
	}

	//gps timestamp in ms
	value = _jsonDoc["txpk"]["tmms"];
	if (!value.isNull())
	{
		loraDataPacketTx.gps_time_tx = _jsonDoc["txpk"]["tmms"];
	}
	else
	{
		loraDataPacketTx.gps_time_tx = 0;
	}

	//convert function form timestring to unix timestamp in seconds
	value = _jsonDoc["txpk"]["time"];
	if (!value.isNull())
	{
		const char *temp = _jsonDoc["txpk"]["time"];
		int tempLen = strlen(temp);

		loraDataPacketTx.unix_timestamp = timeStringToUnixTimestamp(temp, tempLen);
	}
	else
	{
		loraDataPacketTx.unix_timestamp = 0;
	}

	value = _jsonDoc["txpk"]["freq"];
	if (!value.isNull())
	{
		loraDataPacketTx.freq_mhz = _jsonDoc["txpk"]["freq"];
	}

	value = _jsonDoc["txpk"]["rfch"];
	if (!value.isNull())
	{
		loraDataPacketTx.rf_chain = _jsonDoc["txpk"]["rfch"];
	}

	value = _jsonDoc["txpk"]["powe"];
	if (!value.isNull())
	{
		loraDataPacketTx.tx_power = _jsonDoc["txpk"]["powe"];
	}

	value = _jsonDoc["txpk"]["modu"];
	if (!value.isNull())
	{
		const char *temp = _jsonDoc["txpk"]["modu"];
		loraDataPacketTx.modulation = StringToLoRaModulation(temp);
	}

	value = _jsonDoc["txpk"]["datr"];
	if (!value.isNull())
	{
		if (loraDataPacketTx.modulation == LoRaModulation_t::FSK)
		{
			loraDataPacketTx.fsk_datarate = _jsonDoc["txpk"]["datr"];
		}

		if (loraDataPacketTx.modulation == LoRaModulation_t::LORA)
		{
			const char *tempPtr = _jsonDoc["txpk"]["datr"];
			loraDataPacketTx.spreadFact = convertDataRateIdentifierToSpreadFact(tempPtr);

			loraDataPacketTx.bandwidth_khz = convertDataRateIdentifierToBandwidth(tempPtr);
		}
	}

	value = _jsonDoc["txpk"]["codr"];
	if (!value.isNull())
	{
		const char *tempPtr = _jsonDoc["txpk"]["codr"];
		loraDataPacketTx.codingRate = StringToLoRaCodingRate(tempPtr);
	}

	value = _jsonDoc["txpk"]["fdev"];
	if (!value.isNull())
	{
		loraDataPacketTx.fsk_dev = _jsonDoc["txpk"]["fdev"];
	}

	value = _jsonDoc["txpk"]["ipol"];
	if (!value.isNull())
	{
		loraDataPacketTx.modul_pol_inv = _jsonDoc["txpk"]["ipol"];
	}

	value = _jsonDoc["txpk"]["prea"];
	if (!value.isNull())
	{
		loraDataPacketTx.preamble = _jsonDoc["txpk"]["prea"];
	}

	value = _jsonDoc["txpk"]["size"];
	if (!value.isNull())
	{
		loraDataPacketTx.msg_len = _jsonDoc["txpk"]["size"];
	}

	value = _jsonDoc["txpk"]["data"];
	if (!value.isNull())
	{

		//empty datastruct
		memset(_b64, 0, TTN_MAX_PAYLOAD * (sizeof(_b64[0])));

		const char *tempPtr = _jsonDoc["txpk"]["data"].as<const char *>();

		//Serial.println("extract B64");
		//Serial.println(tempPtr);

		//memcpy(_b64, tempPtr, );
		//_b64 = _jsonDoc["data"].as<char*>();

		//_jsonDoc["data"].printTo((char*)_b64, _jsonDoc["data"].measureLength() + 1);
		//size_t len = measureJson(_jsonDoc["data"]);

		//serializeJson(_jsonDoc["data"], _b64);

		///////b64_to_bin(tempPtr, TTN_MAX_PAYLOAD, loraDataPacketTx.msg, loraDataPacketTx.msg_len);
		int decoded_length = decode_base64((unsigned char *)tempPtr, loraDataPacketTx.msg);
	}

	value = _jsonDoc["txpk"]["ncrc"];
	if (!value.isNull())
	{
		loraDataPacketTx.no_crc = _jsonDoc["txpk"]["ncrc"];
	}

	return true;
}

/**
* @brief generate the transmit acknowledge message
*
* @param error of type TxAckError_t to set the additional information field
*
* @return nothing
*
* @exceptsafe This function does not throw exceptions.
*/
void TtnGwUdpParser::genTxAckMsg(TxAckError_t error)
{
	/*
That packet type is used by the gateway to send a feedback to the server
to inform if a downlink request has been accepted or rejected by the gateway.

 Bytes| Function
:------:|---------------------------------------------------------------------
 0| protocol version = 2
 1-2| same token as the PULL_RESP packet to acknowledge
 3| TX_ACK identifier 0x05
 4-11 | Gateway unique identifier (MAC address)
 12-end | [optional] JSON object, starting with {, ending with }


 Name |Type| Function
:----:|:------:|------------------------------------------------------------------------------
error | string | Indication about success or type of failure that occured for downlink request.

The possible values of "error" field are:

Value | Definition
:-----------------:|---------------------------------------------------------------------
NONE| Packet has been programmed for downlink
TOO_LATE| Rejected because it was already too late to program this packet for downlink
TOO_EARLY | Rejected because downlink packet timestamp is too much in advance
COLLISION_PACKET| Rejected because there was already a packet programmed in requested timeframe
COLLISION_BEACON| Rejected because there was already a beacon planned in requested timeframe
TX_FREQ | Rejected because requested frequency is not supported by TX RF chain
TX_POWER| Rejected because requested power is not supported by gateway
GPS_UNLOCKED| Rejected because GPS is unlocked, so GPS timestamp cannot be used



{"txpk_ack": {JSON_OBJECT}}
*/

	outBuffer[0] = PROTOCOL_VERSION;
	outBuffer[1] = _inTokenL;
	outBuffer[2] = _inTokenH;
	outBuffer[3] = PCKG_TX_ACK;
	outMsgLen = 4;

	memcpy(outBuffer + outMsgLen, _gwInfo.identifier, 8);
	outMsgLen += 8;

	strcpy(outBuffer + outMsgLen, "{\"txpk_ack\":");
	outMsgLen += 12;

	_jsonDoc.clear();
	_jsonDoc["error"] = TxAckErrorToString(error);

	memset(_serializedJsonDoc, 0, JSONBUFFER);
	serializeJson(_jsonDoc, _serializedJsonDoc);
	strcpy(outBuffer + outMsgLen, _serializedJsonDoc);
	outMsgLen += strlen(_serializedJsonDoc);

	outMsgLen++;
	outBuffer[outMsgLen] = '}';
}

/**
* @brief coppy the generated out message to a seperate datastructure.
* function is more save than @see getOutBuffer
*
* @return nothing
*
* @exceptsafe This function does not throw exceptions.
*/
void TtnGwUdpParser::getOutMsg(TtnUdpPacketOut_t *outDataStruct)
{
	outDataStruct->outMsgLen = outMsgLen;
	memcpy(outDataStruct->outBuffer, outBuffer, sizeof(uint8_t) * outMsgLen);
	outDataStruct->retrys = 0;
}
