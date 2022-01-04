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

#include "LoRaModemSX127xHandler.h"

/**
* @brief constructor for LoRaModemSX127xHandler, setup the instance number.
*
* @exceptsafe This function does not throw exceptions.
*/
LoRaModemSX127xHandler::LoRaModemSX127xHandler()
{
    memset(&loraDataPacketRx, 0, sizeof(LoRaDataPktRx_t));

    for (int i = 0; i < sizeof(_availTxFrequency); i++)
        _availTxFrequency[i] = -1;

    for (int i = 0; i < sizeof(_availSpreadFactTx); i++)
        _availSpreadFactTx[i] = -1;

    //init queue
    _queueOnCadDoneOut = xQueueCreate(MAX_SIZE_IRQ_MODEMHANDLER, sizeof(OnCadDoneDataOut_t));
    _queueOnCadDoneIn = xQueueCreate(MAX_SIZE_IRQ_MODEMHANDLER, sizeof(OnCadDoneDataIn_t));
    _queueOnRecieveOut = xQueueCreate(MAX_SIZE_IRQ_MODEMHANDLER, sizeof(OnReceiveDataOut_t));
    _queueOnTxDoneIn = xQueueCreate(MAX_SIZE_IRQ_MODEMHANDLER, sizeof(OnReceiveDataOut_t));
    queueTimerInterruptIn = xQueueCreate(MAX_SIZE_IRQ_MODEMHANDLER, sizeof(uint8_t));
}

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
void LoRaModemSX127xHandler::setAvailTxFrequency(int *data, int noEnt)
{
    for (int i = 0; i < sizeof(_availTxFrequency); i++)
        _availTxFrequency[i] = -1;

    memcpy(_availTxFrequency, data, sizeof(int) * noEnt);
}

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
void LoRaModemSX127xHandler::setAvailSpreadFactTx(uint8_t *data, int noEnt)
{
    for (int i = 0; i < sizeof(_availSpreadFactTx); i++)
        _availSpreadFactTx[i] = 0xFF;

    memcpy(_availSpreadFactTx, data, sizeof(uint8_t) * noEnt);
}

/**
* @brief check if the specified spread factor is permitted within this modem handler instance.
*
* @param sf requested spreding factor
*
* @return nothing.
*
* @exceptsafe This function does not throw exceptions.
*/
bool LoRaModemSX127xHandler::isSpreadFactTxEnabled(uint8_t sf)
{
    for (int i = 0; i < sizeof(_availSpreadFactTx); i++)
    {
        if (_availSpreadFactTx[i] == sf)
        {
            return true;
        }
    }

    return false;
}

/**
* @brief check if the specified frequency is permitted within this modem handler instance.
*
* @param sf requested spreding factor
*
* @return nothing.
*
* @exceptsafe This function does not throw exceptions.
*/
bool LoRaModemSX127xHandler::isTxFrequencyEnabled(int frequency)
{
    for (int i = 0; i < sizeof(_availTxFrequency); i++)
    {
        if (_availTxFrequency[i] == frequency)
        {
            return true;
        }
    }

    return false;
}

//settings for packet request to transmit
void LoRaModemSX127xHandler::setAdditionalInfo(uint16_t ifChan, uint16_t rfChain)
{
    _if_channel = ifChan;
    _rf_chain = rfChain;
}

/**
* @brief configure if transmit filters should be used.
*
* @param state true if enabled, false if disabled
*
* @return nothing.
*
* @exceptsafe This function does not throw exceptions.
*/
void LoRaModemSX127xHandler::setIgnorAllTxFilter(bool state)
{
    _disableAllTxFlt = state;
}

/**
* @brief configure if requestes inteface channel should be ignored.
*
* @param state true if enabled, false if disabled
*
* @return nothing.
*
* @exceptsafe This function does not throw exceptions.
*/
void LoRaModemSX127xHandler::setIgnoreReqIfChan(bool state)
{
    _ignoreIfChannel = state;
}

/**
* @brief configure if requested radio frequency chain should be ignored.
*
* @param state true if enabled, false if disabled
*
* @return nothing.
*
* @exceptsafe This function does not throw exceptions.
*/
void LoRaModemSX127xHandler::setIgnoreReqRfChain(bool state)
{
    _ignoreRfChain = state;
}

/**
* @brief configure if requested transmit power should be ignored.
*
* @param state true if enabled, false if disabled
*
* @return nothing.
*
* @exceptsafe This function does not throw exceptions.
*/
void LoRaModemSX127xHandler::setIgnoreReqTxPower(bool state)
{
    _ignoreTxPower = state;
}

/**
* @brief configure if requested polarization should be ignored.
*
* @param state true if enabled, false if disabled
*
* @return nothing.
*
* @exceptsafe This function does not throw exceptions.
*/
void LoRaModemSX127xHandler::setIgnoreReqPolarization(bool state)
{
    _ignorePolarization = state;
}

/**
* @brief configure if transmission with this instance is disabled.
*
* @param state true if enabled, false if disabled
*
* @return nothing.
*
* @exceptsafe This function does not throw exceptions.
*/
void LoRaModemSX127xHandler::setDisableTx(bool state)
{
    _noTx = state;
}

/**
* @brief function is used to que in a transmission between channel detection and receiving a message.
*
* @param packetTx packet to be transmitted.
*
* @return nothing.
*
* @exceptsafe This function does not throw exceptions.
*/
bool LoRaModemSX127xHandler::requestTransmit(LoRaDataPktTx_t *packetTx)
{
    //request a Transmit
    //Transmit is executet with the process function
    //enable in the process function is set via the irq task, after cad done ist detectec
    //air channel = temporary free

    if (_noTx)
    {
        Serial.println("rej: No Tx");
        return false;
    }

    //take care of float precision
    double tempValue = packetTx->freq_mhz;
    tempValue *= 1E3;

    uint32_t txFrequency = round(tempValue);
    txFrequency *= 1E3;

    if (!_disableAllTxFlt)
    {
        Serial.println("flt enable");

        //correct interface channel?
        if (!_ignoreIfChannel)
        {
            if (packetTx->if_channel != _if_channel)
            {
                Serial.println("rej: if channel");
                return false;
            }
        }

        //correct rf chain?
        if (!_ignoreRfChain)
        {
            if (packetTx->rf_chain != _rf_chain)
            {
                Serial.println("rej: rf chain");
                return false;
            }
        }

        //correct spreading factor
        if (!isSpreadFactTxEnabled((uint8_t)packetTx->spreadFact))
        {
            //Serial.println("rej: spread fac");
            return false;
        }
        //correct frequency
        if (!isTxFrequencyEnabled(txFrequency))
        {
            //Serial.println("rej: frequency");
            return false;
        }
    }

    //Serial.println("prep modem and msg config");

    msg_len = packetTx->msg_len;
    memcpy(msg, packetTx->msg, sizeof(uint8_t) * msg_len);

    //get default settings
    interface[_cpyInstance]->backupModemConfig(_modemTxConfig);

    //convert everything in modem config container
    _modemTxConfig.freq_hz = txFrequency;

    if (!(_ignoreTxPower))
    {
        _modemTxConfig.tx_power = packetTx->tx_power;
    }

    _modemTxConfig.spreadFact = packetTx->spreadFact;

    _modemTxConfig.bandwidth_hz = packetTx->bandwidth_khz * 1000.0;

    _modemTxConfig.crc = !packetTx->no_crc;

    if (!(_ignorePolarization))
    {
        _modemTxConfig.invertPolarity = packetTx->modul_pol_inv;
    }

    _modemTxConfig.fsk_dev = packetTx->fsk_dev;
    _modemTxConfig.fsk_datarate = packetTx->fsk_datarate;
    _modemTxConfig.preamble = packetTx->preamble;

    _modemTxConfig.codingRate = LoRaCodingRateToInt(packetTx->codingRate);
    _modemTxConfig.modulation = packetTx->modulation;

    //set request for transmit

    OnCadDoneDataIn_t dataIn;

    dataIn.instance = _cpyInstance;
    dataIn.txRequest = true;

    xQueueSend(_queueOnCadDoneIn, &dataIn, portMAX_DELAY);

    return true;
}

/**
* @brief initialize modem on an specific frequency
*
* @param interface wich instance of LoRaModemSX127x should be used
*
* @return nothing.
*
* @exceptsafe This function does not throw exceptions.
*/
void LoRaModemSX127xHandler::begin(LoRaModemSX127x *set_interface)
{
    _cpyInstance = set_interface->instance;

    //Serial.print("RegisterInstance:");
    //Serial.println(_cpyInstance);

    interface[_cpyInstance] = set_interface;

    memset(&_modemTxConfig, 0, sizeof(ModemConfig_t));
    memset(&loraDataPacketRx, 0, sizeof(LoRaDataPktRx_t));

    //set interrupt routines

    switch (_cpyInstance)
    {
    case 0:
        interface[_cpyInstance]->onReceive(_onReceive0);
        interface[_cpyInstance]->onCadDetected(_onCadDetect0);
        interface[_cpyInstance]->onCadDone(_onCadDone0);
        interface[_cpyInstance]->onRxTimeout(_onRxTout0);
        interface[_cpyInstance]->onTxDone(_onTxDone0);
        break;

    default:
        break;
    }
}

/// @brief registerd callback for onReceive
void LoRaModemSX127xHandler::_onReceive0(int instance, int64_t timestamp, uint32_t packetLength, uint8_t irqFlags)
{
    esp_task_wdt_reset();

    OnReceiveDataOut_t dataOut;

    dataOut.instance = instance;
    dataOut.msg_len = interface[instance]->recNoBytes(false);
    dataOut.rx_timestamp = uint32_t(0x00000000FFFFFFFF & timestamp);

    if (interface[instance]->isIrqCrcError(irqFlags))
    {
        dataOut.crcOk = false;
        //Serial.println("Crc n.Ok.");
    }
    else
    {
        dataOut.crcOk = true;

        if (interface[instance]->isIrqValidHeader(irqFlags))
        {
            dataOut.headerOk = true;
        }
        else
        {
            dataOut.headerOk = false;
            //Serial.println("Header Not Ok.");
        }
    }

    //get current Modem configuration, needed later to build the message to ttn server
    interface[instance]->backupModemConfig(dataOut.modemCfg);

    //put all data in queue
    xQueueSend(_queueOnRecieveOut, &dataOut, portMAX_DELAY);

    interface[instance]->cadScan(false);
}

/// @brief registerd callback for RxTout
void LoRaModemSX127xHandler::_onRxTout0(int instance, int64_t timestamp)
{
    esp_task_wdt_reset();
    //Serial.println("RxTout");
    interface[instance]->cadScan(false);
}

/// @brief registerd callback for TxDone
void LoRaModemSX127xHandler::_onTxDone0(int instance, int64_t timestamp)
{
    esp_task_wdt_reset();
    //Serial.println("TxDone");

    OnTxDoneDataIn_t dataIn;

    for (int i = 0; i < uxQueueMessagesWaiting(_queueOnTxDoneIn); i++)
    {
        //Serial.println("check open queue::::TxDone");
        xQueueReceive(_queueOnTxDoneIn, &dataIn, portMAX_DELAY);

        if (dataIn.instance == instance)
        {
            //Serial.println("restore Modem config");
            //put modem in the old config
            //interface[instance]->printModemConfig();

            interface[instance]->restoreModemConfig(dataIn.modemCfg, false);

            //Serial.println("restored Config: ");
            //interface[instance]->printModemConfig();
        }
        else
        {

            //put data back
            xQueueSend(_queueOnTxDoneIn, &dataIn, portMAX_DELAY);
        }
    }

    //Serial.println("enable cad scan after tx done");

    //after interrupt, the modem should go back to cadScan
    interface[instance]->cadScan(false);
}

/// @brief registerd callback for CadDetect
void LoRaModemSX127xHandler::_onCadDetect0(int instance, int64_t timestamp)
{
    esp_task_wdt_reset();
    //Serial.println("cadDetected");
    interface[instance]->recSingleMsg(false);
    return;
}

/// @brief registerd callback for CadDone
void LoRaModemSX127xHandler::_onCadDone0(int instance, int64_t timestamp, uint8_t irqFlags)
{
    esp_task_wdt_reset();
    //Serial.println("cadDone");
    if (interface[instance]->isIrqCadDetect(irqFlags))
    {
        //Serial.println("cadDone->CadDetected");
        return;
    }

    OnCadDoneDataOut_t dataOut;
    OnCadDoneDataIn_t dataIn;

    for (int i = 0; i < uxQueueMessagesWaiting(_queueOnCadDoneIn); i++)
    {
        //Serial.println("check open queue::::CadDone");
        
        xQueueReceive(_queueOnCadDoneIn, &dataIn, portMAX_DELAY);

        if (dataIn.instance == instance)
        {
            dataOut.instance = instance;
            dataOut.txReady = true;
            xQueueSend(_queueOnCadDoneOut, &dataOut, portMAX_DELAY);

            //put modem in standby, so no timeout error can occoure
            interface[instance]->stdby(false);
            return;
        }
        else
        {
            //put it back, not the current interrupt
            xQueueSend(_queueOnCadDoneIn, &dataIn, portMAX_DELAY);
        }
    }

    interface[instance]->cadScan(false);
}

/**
* @brief process open tasks. This function hast to be called periodicly.
*
* @return nothing.
*
* @exceptsafe This function does not throw exceptions.
*/
void LoRaModemSX127xHandler::process()
{
    //Serial.println("process");

    if (uxQueueMessagesWaiting(_queueOnRecieveOut) != 0)
    {
        OnReceiveDataOut_t dataOut;
        xQueueReceive(_queueOnRecieveOut, &dataOut, portMAX_DELAY);

        //handling information
        // time_compact, gps_time is set, if the msg is transmitted via udp

        //packet data
        loraDataPacketRx.timestamp_rx = dataOut.rx_timestamp;
        loraDataPacketRx.freq_mhz = (double)(dataOut.modemCfg.freq_hz / 1E6);

        //msg data
        loraDataPacketRx.msg_len = dataOut.msg_len;
        interface[dataOut.instance]->readMsg(loraDataPacketRx.msg, loraDataPacketRx.msg_len, true);

        //handler and modem overhead information
        loraDataPacketRx.if_channel = _if_channel;
        loraDataPacketRx.rf_chain = _rf_chain;

        //crc
        if (dataOut.modemCfg.crc == true)
        {
            if (dataOut.crcOk)
            {
                loraDataPacketRx.crc = LoRaCrcConfig_t::CRC_OK;
            }
            else
            {
                loraDataPacketRx.crc = LoRaCrcConfig_t::CRC_FAIL;
            }
        }
        else
        {
            loraDataPacketRx.crc = LoRaCrcConfig_t::CRC_OFF;
        }

        //modulation
        loraDataPacketRx.modulation = dataOut.modemCfg.modulation;

        //spreadFactor
        //enum values are the same as the modem config integers
        loraDataPacketRx.spreadFact = Int2LoRaSpreadingFactor(dataOut.modemCfg.spreadFact);

        //bandwidth
        loraDataPacketRx.bandwidth_khz = dataOut.modemCfg.bandwidth_hz / 1000.0;

        //fsk datarate
        loraDataPacketRx.fsk_datarate = dataOut.modemCfg.fsk_datarate;

        //coding rate
        //enum values are the same as the modem config integers
        loraDataPacketRx.codingRate = IntToLoRaCodingRate(dataOut.modemCfg.codingRate);

        //read packet rssi
        loraDataPacketRx.rssi = interface[dataOut.instance]->packetRssi(true);

        //read packet snr
        loraDataPacketRx.snr = interface[dataOut.instance]->packetSnr(true);

        //fsk - not yet implemented
        loraDataPacketRx.fsk_datarate = dataOut.modemCfg.fsk_datarate;

        //debug out of rec packet
        //=================================================
        //Serial.write(loraDataPacketRx.msg, loraDataPacketRx.msg_len);
        //Serial.println();
        //=================================================
    }

    if (uxQueueMessagesWaiting(_queueOnCadDoneOut) != 0)
    {
        OnTxDoneDataIn_t dataIn;
        OnReceiveDataOut_t dataOut;

        xQueueReceive(_queueOnCadDoneOut, &dataOut, portMAX_DELAY);

        dataIn.instance = dataOut.instance;

        //backup von der modemconfig erstellen
        interface[dataOut.instance]->backupModemConfig(dataIn.modemCfg);

        //modemconfig an die Interrupt Routine für die wiederherstellung senden
        xQueueSend(_queueOnTxDoneIn, &dataIn, portMAX_DELAY);

        //sender Configurieren
        interface[dataOut.instance]->restoreModemConfig(_modemTxConfig, true);

        //interface[dataOut.instance]->printModemConfig();

        //bool temp = interface[dataOut.instance]->transmitMsg(msg, msg_len, true, true);
        interface[dataOut.instance]->prepareTransmitMsg(msg, msg_len, true);
        
        xQueueSend(queueTimerInterruptIn, &dataOut.instance, portMAX_DELAY);
    }
}
