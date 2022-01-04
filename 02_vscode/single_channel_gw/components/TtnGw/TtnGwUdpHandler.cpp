/*=======================================================================
Lib to interface with the udp interface and the udp semtech protocoll 
(semtech protocoll is implemented in the TtnGwUdpParser.h)
Requirements:
  - esp-idf
  - arduino framework
  - TtnGwUdpParser.h
  - TtnGwTime.h
Version: 0.1
Author: MPO
Date: 30-21-2021
=======================================================================*/

#include "TtnGwUdpHandler.h"

/**
  * @brief constructor of TtnGwUdpHandler.
  *
  * @return nothing
  *
  * @exceptsafe This function does not throw exceptions.
  */
TtnGwUdpHandler::TtnGwUdpHandler()
{
    //to do
}

/**
* @brief init all helper structs.
*
* @return nothing.
*
* @exceptsafe This function does not throw exceptions.
*/
void TtnGwUdpHandler::begin()
{
    udpIf.begin(_localPort); //setup local receive Port to get UDP data

    memset(&_gwStats, 0, sizeof(LoRaGwStatistic_t));
    _gwStats.ackr_nom = 1;
    _gwStats.ackr_denom = 1;

    memset(&_loraPktRx, 0, sizeof(LoRaDataPktRx_t));
    memset(&_loraPktTx, 0, sizeof(LoRaDataPktTx_t));

    ttnPkgParser.update(_gwGeoData);
    ttnPkgParser.update(_gwStats);
    ttnPkgParser.update(_gwInfo);
    ttnPkgParser.update(_loraPktRx);
    ttnPkgParser.update(_loraPktTx);

    memset(&_udpMsgBuffer, 0, sizeof(TtnUdpPacketOut_t));
}

/**
* @brief setup the refrences to all used queues.
*
* @param txQueue reference to txQueue.
* @param rxQueue reference to rxQueue.
* @param schedQueue reference to schedQueue.
* @param txUdpToutQueue reference to txUdpToutQueue.
*
* @return nothing.
*
* @exceptsafe This function does not throw exceptions.
*/
void TtnGwUdpHandler::configQueue(QueueHandle_t *txQueue, QueueHandle_t *rxQueue, QueueHandle_t *schedQueue, QueueHandle_t *txUdpToutQueue)
{
    _txQueue = txQueue;
    _rxQueue = rxQueue;
    _schedQueue = schedQueue;
    _txUdpToutQueue = txUdpToutQueue;
}

/**
* @brief setup the safety checks for all used queues.
*
* @param txQueueSize size of txQueue.
* @param rxQueueSize size of rxQueue.
* @param schedQueueSize size of schedQueue.
* @param txUdpToutQueueSize size of txUdpToutQueue.
*
* @return nothing.
*
* @exceptsafe This function does not throw exceptions.
*/
void TtnGwUdpHandler::configInfoQueueSizes(uint32_t txQueueSize, uint32_t rxQueueSize, uint32_t schedQueueSize, uint32_t txUdpToutQueueSize)
{
    _txQueueSize = txQueueSize;
    _rxQueueSize = rxQueueSize;
    _schedQueueSize = schedQueueSize;
    _txUdpToutQueueSize = txUdpToutQueueSize;
}

/**
* @brief setup how far in adavace the data should be available in the high priority task.
*
* @param txTmstPreload preload time in ms for the tmst marker.
* @param txTmmsPreload size of rxQueue in ms for the tmms marker.
* @param txTimePreload size of schedQueue in ms for the time marker.
*
* @return nothing.
*
* @exceptsafe This function does not throw exceptions.
*/
void TtnGwUdpHandler::configTxPreload(uint32_t txTmstPreload, uint32_t txTmmsPreload, uint32_t txTimePreload)
{
    _txTmstPreload = txTmstPreload;
    _txTmmsPreload = txTmmsPreload;
    _txTimePreload = txTimePreload;
}

/**
* @brief config the udp interface settings
*
* @param remoteIp target ip of the ttn server.
* @param localPort local port of the gatway.
* @param remotePort target port of the ttn server.
* @param transmitRetrys how often not acknowledged packets should be resent.
*
* @return nothing.
*
* @exceptsafe This function does not throw exceptions.
*/
void TtnGwUdpHandler::configUDP(const char *remoteIp, const int localPort, const int remotePort, const int transmitRetrys)
{
    _remoteIp = (char *)remoteIp;
    _remotePort = remotePort;
    _localPort = localPort;
    _transmitRetrys = transmitRetrys;
}

/**
* @brief config the debug udp interface settings
*
* @param udpDbgIf pointer to the udp interface wich should be used.
* @param remoteDbgIp target ip, for the listener.
* @param remoteDbgPort target port for the listener. 
*
* @return nothing.
*
* @exceptsafe This function does not throw exceptions.
*/
void TtnGwUdpHandler::configDebugUDP(WiFiUDP *udpDbgIf, const char *remoteDbgIp, const int remoteDbgPort)
{
    _udpDbgIf = udpDbgIf;
    _remoteDbgIp = (char *)remoteDbgIp;
    _remoteDbgPort = remoteDbgPort;
}

/**
* @brief how fare the timestamp is allowed to differe from the internal timestamp.
*
* @param pktMaxFutur maximum difference in the future, in seconds.
* @param pktMaxPast maximum differenc ein the past, in seconds.
*
* @return nothing.
*
* @exceptsafe This function does not throw exceptions.
*/
void TtnGwUdpHandler::configPacketTimeframe(int pktMaxFutur, int pktMaxPast)
{
    _pktMaxFutur = pktMaxFutur;
    _pktMaxPast = pktMaxPast;
}

/**
* @brief configure the automatic retry intervalls for diffrent message types.
*
* @param gwUpdateIntervall invervall defines how often (in time base) the status message should be sent to ttn server.
* @param pullDataIntervall inverall defines how often (in time base) the pull message should be sent to ttn server.
* @param udpRetryIntervall intervall defines how often (in time base) the push message should be repeated, if it is not acknowledged.
*
* @return nothing.
*
* @exceptsafe This function does not throw exceptions.
*/
void TtnGwUdpHandler::configIntervall(uint32_t gwUpdateIntervall, uint32_t pullDataIntervall, uint32_t udpRetryIntervall)
{
    _gwUpdateIntervall = gwUpdateIntervall;
    _pullDataIntervall = pullDataIntervall;
    _udpRetryIntervall = udpRetryIntervall;
}

/**
* @brief update basic information, used in status message
*
* @param gwGeoData basic data of the gateway about the geo location.
* @param gwInfo general information abaout the gateway.
*
* @return nothing.
*
* @exceptsafe This function does not throw exceptions.
*/
void TtnGwUdpHandler::updateBasicInformation(LoRaGwGeoData_t &gwGeoData, LoRaGwInfo_t &gwInfo)
{
    memcpy(&_gwGeoData, &gwGeoData, sizeof(LoRaGwGeoData_t));
    memcpy(&_gwInfo, &gwInfo, sizeof(LoRaGwInfo_t));

    ttnPkgParser.update(gwGeoData);
    ttnPkgParser.update(gwInfo);
}

/**
* @brief read available data from udp interface
*
* @return nothing.
*
* @exceptsafe This function does not throw exceptions.
*/
void TtnGwUdpHandler::readUdp()
{
    //process the incomming data
    int packetSize = udpIf.parsePacket();

    ttnPkgParser.inMsgLen = udpIf.read(ttnPkgParser.inBuffer, ttnPkgParser.bufferSize);

    if (ttnPkgParser.inMsgLen != 0)
    {

        if (_udpDbgIf != NULL)
        {

            _udpDbgIf->beginPacket(_remoteDbgIp, _remoteDbgPort);
            _udpDbgIf->print("In Traffic: timestamp=");
            _udpDbgIf->print(esp_timer_get_time());

            _udpDbgIf->endPacket();

            _udpDbgIf->beginPacket(_remoteDbgIp, _remoteDbgPort);
            _udpDbgIf->write((uint8_t *)ttnPkgParser.inBuffer, ttnPkgParser.inMsgLen);
            _udpDbgIf->endPacket();
        }
    }
}

/**
* @brief transmit available to the udp interface
*
* @return nothing.
*
* @exceptsafe This function does not throw exceptions.
*/
void TtnGwUdpHandler::transmitUdp()
{
    int outLen = ttnPkgParser.getOutBufferLen();
    if (outLen != 0)
    {
        udpIf.beginPacket(_remoteIp, _remotePort);

        udpIf.write((uint8_t *)ttnPkgParser.outBuffer, outLen);
        udpIf.endPacket();

        if (_udpDbgIf != NULL)
        {
            _udpDbgIf->beginPacket(_remoteDbgIp, _remoteDbgPort);
            _udpDbgIf->print("Out Traffic:");
            _udpDbgIf->endPacket();

            _udpDbgIf->beginPacket(_remoteDbgIp, _remoteDbgPort);
            _udpDbgIf->write((uint8_t *)ttnPkgParser.outBuffer, outLen);
            _udpDbgIf->endPacket();
        }
    }
}

/**
* @brief handle all status message related functions
*
* @return nothing.
*
* @exceptsafe This function does not throw exceptions.
*/
void TtnGwUdpHandler::statusUpdate()
{
    ttnTimeData.update();

    //send status update to ttn server
    //================================
    if (((ttnTimeData.unixTimeStamp_s() % _gwUpdateIntervall) == 0) && (_toggleGwUpdate == false))
    {
        Serial.println(">>send status update");
        //Serial.print("unix:");
        //Serial.println(ttnTimeData.unixTimeStamp_s());
        //Serial.print("gps time:");
        //Serial.println(ttnTimeData.getGpsTime());

        if (_gwStats.ackr_nom != 0)
        {
            _gwStats.ackr = (_gwStats.ackr_nom / _gwStats.ackr_denom) * 100;
        }

        //update status information
        memset(_gwStats.time_expanded, 0, sizeof(_gwStats.time_expanded));
        ttnTimeData.getTimeExpandedSystem(_gwStats.time_expanded);

        ttnPkgParser.update(_gwStats);

        //Serial.println("Send GW Update");
        ttnPkgParser.genStatMsg();
        transmitUdp();
        _toggleGwUpdate = true;
    }

    if ((ttnTimeData.unixTimeStamp_s() % _gwUpdateIntervall) == 1)
    {
        _toggleGwUpdate = false;
    }

    //send pull data message and signalize the ttn server that the gw is ready to transmit
    //================================

    if (((ttnTimeData.unixTimeStamp_s() % _pullDataIntervall) == 0) && (_togglePullData == false))
    {
        ttnPkgParser.genPullDataMsg();
        transmitUdp();
        _togglePullData = true;
    }

    if ((ttnTimeData.unixTimeStamp_s() % _pullDataIntervall) == 1)
    {
        _togglePullData = false;
    }
}

/**
* @brief check for not acknowledged messages, an resend them, if necessary.
*
* @return nothing.
*
* @exceptsafe This function does not throw exceptions.
*/
void TtnGwUdpHandler::checkNotAckMsg()
{
    //re-send not acknoledeged packets
    //================================
    if (((ttnTimeData.unixTimeStamp_s() % _udpRetryIntervall) == 0) && (_togglUdpResend == false))
    {
        for (int i = 0; i < uxQueueMessagesWaiting(*_txUdpToutQueue); i++)
        {
            //get stored msg
            xQueueReceive(*_txUdpToutQueue, &_udpMsgBuffer, portMAX_DELAY);

            //restore msg
            ttnPkgParser.outMsgLen = _udpMsgBuffer.outMsgLen;
            memcpy(ttnPkgParser.outBuffer, _udpMsgBuffer.outBuffer, sizeof(uint8_t) * ttnPkgParser.outMsgLen);

            _udpMsgBuffer.retrys++;

            _gwStats.ackr_denom++;
            //send current udp msg
            transmitUdp();

            //reschedule msg only if it is not tryed to often
            if (_udpMsgBuffer.retrys < _transmitRetrys)
            {
                //put backuped msg back
                if (uxQueueMessagesWaiting(*_txUdpToutQueue) < _txQueueSize)
                {
                    Serial.println("packet in tx resend queue");
                    //put packet in queue to be send
                    xQueueSend(*_txUdpToutQueue, &_udpMsgBuffer, portMAX_DELAY);
                }
            }
            else
            {
                Serial.println("delete udp package, no ack rec");
            }
        }
        _togglUdpResend = true;
    }

    if ((ttnTimeData.unixTimeStamp_s() % _udpRetryIntervall) == 1)
    {
        _togglUdpResend = false;
    }
}

/**
* @brief check for upsteam messages from lora modem
*
* @return nothing.
*
* @exceptsafe This function does not throw exceptions.
*/
void TtnGwUdpHandler::upadteRxQueue()
{
    //get Data from *_rxQueue, send to ttn server
    //data transfer from high prio task to low prio task
    //================================
    if (uxQueueMessagesWaiting(*_rxQueue) != 0)
    {
        Serial.println(">>lora rx push to ttn server");
        xQueueReceive(*_rxQueue, &_loraPktRx, portMAX_DELAY);

        switch (_loraPktRx.crc)
        {
        case LoRaCrcConfig_t::CRC_OK:
            _gwStats.rxnb++;
            _gwStats.rxok++;
            break;
        case LoRaCrcConfig_t::CRC_FAIL:
            _gwStats.rxnb++;
            _gwStats.rxnok++;
            //hier müsste ein abbruch rein für die subroutine

            break;
        case LoRaCrcConfig_t::CRC_OFF:
            _gwStats.rxnb++;
            _gwStats.rxcrcoff++;
            break;
        default:
            break;
        }

        _gwStats.rxfw++;
        memset(_loraPktRx.time_compact, 0, sizeof(_loraPktRx.time_compact));
        ttnTimeData.getTimeCompactGmt(_loraPktRx.time_compact);

        _loraPktRx.gps_time_rx = ttnTimeData.getGpsTime();

        ttnPkgParser.update(_loraPktRx);
        ttnPkgParser.genPushDataMsg();

        //backup msg
        _udpMsgBuffer.outMsgLen = ttnPkgParser.outMsgLen;
        memcpy(_udpMsgBuffer.outBuffer, ttnPkgParser.outBuffer, sizeof(uint8_t) * ttnPkgParser.outMsgLen);
        _udpMsgBuffer.retrys = 0;
        //put backuped msg in resend queue
        if (uxQueueMessagesWaiting(*_txUdpToutQueue) < _txUdpToutQueueSize)
        {
            Serial.println("packet in tx resend queue");
            //put packet in queue to be send
            xQueueSend(*_txUdpToutQueue, &_udpMsgBuffer, portMAX_DELAY);
        }

        _gwStats.ackr_denom++;
        //send current udp msg
        transmitUdp();
    }
}

/**
* @brief check wich data should be put in TxQueue
*
* @return nothing.
*
* @exceptsafe This function does not throw exceptions.
*/
void TtnGwUdpHandler::updateTxQueue()
{
    //check wich data should be put in TxQueue
    //unload the schedule queu to high prio task if the right time has com
    //=======================================
    for (int i = 0; i < uxQueueMessagesWaiting(*_schedQueue); i++)
    {
        xQueueReceive(*_schedQueue, &_loraPktTx, portMAX_DELAY);

        if (_loraPktTx.timestamp_tx != 0)
        {
            if (ttnTimeData.isInPastTime_us(_loraPktTx.timestamp_tx - (_txTmstPreload * 1000)))
            {
                //put packet in queue to be send
                Serial.println("put from schedule queue to tx queue");
                xQueueSend(*_txQueue, &_loraPktTx, portMAX_DELAY);
                _gwStats.txnb++;
            }
            else
            {
                //put packet in queue to be scheduled
                xQueueSend(*_schedQueue, &_loraPktTx, portMAX_DELAY);
            }
        }
        else if (_loraPktTx.gps_time_tx != 0)
        {
            if (ttnTimeData.isInPastGpsTime_ms(_loraPktTx.gps_time_tx - _txTmmsPreload))
            {
                //put packet in queue to be send
                xQueueSend(*_txQueue, &_loraPktTx, portMAX_DELAY);
                _gwStats.txnb++;
            }
            else
            {
                //put packet in queue to be scheduled
                xQueueSend(*_schedQueue, &_loraPktTx, portMAX_DELAY);
            }
        }

        else if (_loraPktTx.unix_timestamp != 0)
        {
            double temp = (double)_loraPktTx.unix_timestamp - (double)(_txTimePreload / 1000.000);
            if (ttnTimeData.isInPastUnixTime_s(temp))
            {
                //put packet in queue to be send
                xQueueSend(*_txQueue, &_loraPktTx, portMAX_DELAY);
                _gwStats.txnb++;
            }
            else
            {
                //put packet in queue to be scheduled
                xQueueSend(*_schedQueue, &_loraPktTx, portMAX_DELAY);
            }
        }
    }
}

/**
* @brief check for downstream messages from the ttn server
*
* @return nothing.
*
* @exceptsafe This function does not throw exceptions.
*/
void TtnGwUdpHandler::checkPullMsg()
{
    if (ttnPkgParser.inMsgLen != 0)
    {
        //data transfer from udp interface to high prio task, to transmit the data
        //or store the data in a queue, to transmit the data if the time arrises
        //================================
        if (ttnPkgParser.parsePullRespMsg())
        {
            _gwStats.dwnb++;
            Serial.println("pull packet detected");

            //server requestet data transmit to lora wan
            if (ttnPkgParser.loraDataPacketTx.immediat)
            {
                Serial.println("immediate");

                //chekc if space is avialable to store the message
                if (uxQueueMessagesWaiting(*_txQueue) < _txQueueSize)
                {
                    Serial.println("packet in tx queue");
                    //put packet in queue to be send
                    xQueueSend(*_txQueue, &ttnPkgParser.loraDataPacketTx, portMAX_DELAY);
                    _gwStats.txnb++;
                    ttnPkgParser.genTxAckMsg(TxAckError_t::ERROR_NONE);
                }
                else
                {
                    Serial.println("queue is full");
                    //queue is full, send error to ttn server
                    ttnPkgParser.genTxAckMsg(TxAckError_t::COLLISION_PACKET);
                }
            }
            else
            {
                Serial.println("packet in scheduler");
                //check if message should be scheduled
                if (uxQueueMessagesWaiting(*_schedQueue) < _schedQueueSize)
                {
                    if (ttnPkgParser.loraDataPacketTx.timestamp_tx != 0)
                    {
                        Serial.println("schedule with internal timestamp");

                        if (ttnTimeData.isInTimeFrame_us(ttnPkgParser.loraDataPacketTx.timestamp_tx, _pktMaxFutur, _pktMaxPast) > 0)
                        {
                            Serial.println("packet in correct timeframe->schedule packet");
                            //put packet in queue to be scheduled
                            xQueueSend(*_schedQueue, &ttnPkgParser.loraDataPacketTx, portMAX_DELAY);
                            ttnPkgParser.genTxAckMsg(TxAckError_t::ERROR_NONE);
                        }
                        else
                        {
                            if (ttnTimeData.isInTimeFrame_us(ttnPkgParser.loraDataPacketTx.timestamp_tx, _pktMaxFutur, _pktMaxPast) == -1)
                            {
                                Serial.println("packet too late");
                                ttnPkgParser.genTxAckMsg(TxAckError_t::TOO_LATE);
                            }
                            else
                            {
                                Serial.println("packet too early");
                                ttnPkgParser.genTxAckMsg(TxAckError_t::TOO_EARLY);
                            }
                        }
                    }

                    else if (ttnPkgParser.loraDataPacketTx.unix_timestamp != 0)
                    {
                        Serial.println("schedule with timestamp");

                        if (ttnTimeData.isInUnixTimeFrame_s((double)ttnPkgParser.loraDataPacketTx.unix_timestamp, _pktMaxFutur, _pktMaxPast) > 0)
                        {
                            Serial.println("packet in correct timeframe->schedule packet");
                            //put packet in queue to be scheduled
                            xQueueSend(*_schedQueue, &ttnPkgParser.loraDataPacketTx, portMAX_DELAY);
                            ttnPkgParser.genTxAckMsg(TxAckError_t::ERROR_NONE);
                        }
                        else
                        {
                            if (ttnTimeData.isInUnixTimeFrame_s((double)ttnPkgParser.loraDataPacketTx.unix_timestamp, _pktMaxFutur, _pktMaxPast) == -1)
                            {
                                Serial.println("packet too late");
                                ttnPkgParser.genTxAckMsg(TxAckError_t::TOO_LATE);
                            }
                            else
                            {
                                Serial.println("packet too early");
                                ttnPkgParser.genTxAckMsg(TxAckError_t::TOO_EARLY);
                            }
                        }
                    }

                    else if (ttnPkgParser.loraDataPacketTx.gps_time_tx != 0)
                    {
                        Serial.println("schedule with gps timestamp");
                        if (ttnTimeData.isInGpsTimeFrame_ms(ttnPkgParser.loraDataPacketTx.gps_time_tx, _pktMaxFutur, _pktMaxPast) > 0)
                        {
                            Serial.println("packet in correct timeframe->schedule packet");
                            //put packet in queue to be scheduled
                            xQueueSend(*_schedQueue, &ttnPkgParser.loraDataPacketTx, portMAX_DELAY);
                            ttnPkgParser.genTxAckMsg(TxAckError_t::ERROR_NONE);
                        }
                        else
                        {
                            if (ttnTimeData.isInGpsTimeFrame_ms(ttnPkgParser.loraDataPacketTx.gps_time_tx, _pktMaxFutur, _pktMaxPast) == -1)
                            {
                                Serial.println("packet too early");
                                ttnPkgParser.genTxAckMsg(TxAckError_t::TOO_LATE);
                            }
                            else
                            {
                                Serial.println("packet too late");
                                ttnPkgParser.genTxAckMsg(TxAckError_t::TOO_EARLY);
                            }
                        }
                    }
                }
                else
                {
                    //queue is full, send error to ttn server
                    Serial.println("schedule queue is full");
                    ttnPkgParser.genTxAckMsg(TxAckError_t::COLLISION_PACKET);
                }
            }

            transmitUdp();
        }
    }
}

/**
* @brief check for acknowledge of transmitted data
*
* @return nothing.
*
* @exceptsafe This function does not throw exceptions.
*/
void TtnGwUdpHandler::checkPushMsg()
{

    //Acknowledge of transmitted data
    //data can be deleted from the retry queue
    //=============================
    if (ttnPkgParser.parsePushAckMsg())
    {
        Serial.println("<<push ack msg");
        for (int i = 0; i < uxQueueMessagesWaiting(*_txUdpToutQueue); i++)
        {
            //get stored msg
            xQueueReceive(*_txUdpToutQueue, &_udpMsgBuffer, portMAX_DELAY);

            if (!(ttnPkgParser.compareToken(_udpMsgBuffer.outBuffer[1], _udpMsgBuffer.outBuffer[2])))
            {
                //put backuped msg back
                if (uxQueueMessagesWaiting(*_txUdpToutQueue) < _txUdpToutQueueSize)
                {
                    Serial.println("packet in tx resend queue, not acknowledged");
                    //put packet in queue to be send
                    xQueueSend(*_txUdpToutQueue, &_udpMsgBuffer, portMAX_DELAY);
                }
            }
            else
            {
                Serial.println("Remove Packet, ack");
                _gwStats.ackr_nom++;
            }
        }
    }
}
