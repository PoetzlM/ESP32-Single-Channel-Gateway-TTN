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

#ifndef TTNGWUDPHANDLER_H
#define TTNGWUDPHANDLER_H

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "esp_timer.h"
#include "TtnGwUdpParser.h"
#include "TtnGwTime.h"

/**
 * class to take care of the udp communication from an to the ttn server
 */
class TtnGwUdpHandler
{
private:
  WiFiUDP udpIf;             ///< WIfi Udp Interface
  WiFiUDP *_udpDbgIf = NULL; ///< pointer to second WiFi Udp Interface for debugging

  /// Queues to handle Rx and Tx packets
  QueueHandle_t *_txQueue;        ///< queue for all packets wich should be transmitted from ttn server to the lora modem
  QueueHandle_t *_rxQueue;        ///< queue for all packetes wich are received by the lora modem and should be pushed to ttn server
  QueueHandle_t *_schedQueue;     ///< packets with future timestamp are stored here.
  QueueHandle_t *_txUdpToutQueue; ///< packets wich are not acknowledget are store here.

  uint32_t _gwUpdateIntervall; ///< invervall defines how often (in time base) the status message should be sent to ttn server.
  uint32_t _pullDataIntervall; ///< inverall defines how often (in time base) the pull message should be sent to ttn server.
  uint32_t _udpRetryIntervall; ///< intervall defines how often (in time base) the push message should be repeated, if it is not acknowledged.

  char *_remoteDbgIp;  ///< ip adress for debugging
  int _remoteDbgPort;  ///< port for debugging
  char *_remoteIp;     ///< ip adress of the ttn server
  int _remotePort;     ///< remote port of the ttn server
  int _localPort;      ///< local port of the gateway
  int _transmitRetrys; ///< how often (absolut) a push message should be reapeated before it is deleted.

  ///helper variables to some actions only once in a cycle
  bool _toggleGwUpdate = false;
  bool _togglUdpResend = false;
  bool _togglePullData = false;

  int _pktMaxFutur = 0; ///< how big the delta can be for timestamps with future reference.
  int _pktMaxPast = 0;  ///< how big the delta can be for timestamps with past refrence.

  /// temporary helper variables to store data from and to the ttn server (and vice versa)
  TtnUdpPacketOut_t _udpMsgBuffer;
  LoRaDataPktRx_t _loraPktRx;
  LoRaDataPktTx_t _loraPktTx;

  LoRaGwStatistic_t _gwStats;
  LoRaGwGeoData_t _gwGeoData;
  LoRaGwInfo_t _gwInfo;

  /// how fare in advance the packets should be sent from one queue to an other.
  /// needed so the data is acessable to be processed in the correct timeframe.
  uint32_t _txTmstPreload;
  uint32_t _txTmmsPreload;
  uint32_t _txTimePreload;

  /// define the upper limits of the queues, so a safe acess is guaranteed.
  uint32_t _txQueueSize;
  uint32_t _rxQueueSize;
  uint32_t _schedQueueSize;
  uint32_t _txUdpToutQueueSize;

public:
  /// TTN specific interface
  TtnGwUdpParser ttnPkgParser; ///< TTN UDP Packet parser
  TtnGwTime ttnTimeData;       ///< TTN Time data, loaded via Ethernet interface

  /**
  * @brief constructor of TtnGwUdpHandler.
  *
  * @return nothing
  *
  * @exceptsafe This function does not throw exceptions.
  */
  TtnGwUdpHandler();

  /**
     * @brief init all helper structs.
     *
     * @return nothing.
     *
     * @exceptsafe This function does not throw exceptions.
     */
  void begin();

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
  void configQueue(QueueHandle_t *txQueue, QueueHandle_t *rxQueue, QueueHandle_t *schedQueue, QueueHandle_t *txUdpToutQueue);

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
  void configInfoQueueSizes(uint32_t txQueueSize, uint32_t rxQueueSize, uint32_t schedQueueSize, uint32_t txUdpToutQueueSize);

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
  void configTxPreload(uint32_t txTmstPreload, uint32_t txTmmsPreload, uint32_t txTimePreload);

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
  void configUDP(const char *remoteIp, const int localPort, const int remotePort, const int transmitRetrys);

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
  void configDebugUDP(WiFiUDP *udpDbgIf, const char *remoteDbgIp, const int remoteDbgPort);

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
  void configPacketTimeframe(int pktMaxFutur, int pktMaxPast);

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
  void configIntervall(uint32_t gwUpdateIntervall, uint32_t pullDataIntervall, uint32_t udpRetryIntervall);

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
  void updateBasicInformation(LoRaGwGeoData_t &gwGeoData, LoRaGwInfo_t &gwInfo);

  /**
     * @brief read available data from udp interface
     *
     * @return nothing.
     *
     * @exceptsafe This function does not throw exceptions.
     */
  void readUdp();

  /**
     * @brief transmit available to the udp interface
     *
     * @return nothing.
     *
     * @exceptsafe This function does not throw exceptions.
     */
  void transmitUdp();

  /**
     * @brief handle all status message related functions
     *
     * @return nothing.
     *
     * @exceptsafe This function does not throw exceptions.
     */
  void statusUpdate();

  /**
     * @brief check for not acknowledged messages, an resend them, if necessary.
     *
     * @return nothing.
     *
     * @exceptsafe This function does not throw exceptions.
     */
  void checkNotAckMsg();

  /**
     * @brief check for upsteam messages from lora modem
     *
     * @return nothing.
     *
     * @exceptsafe This function does not throw exceptions.
     */
  void upadteRxQueue();

  /**
     * @brief check wich data should be put in TxQueue
     *
     * @return nothing.
     *
     * @exceptsafe This function does not throw exceptions.
     */
  void updateTxQueue();

  /**
     * @brief check for downstream messages from the ttn server
     *
     * @return nothing.
     *
     * @exceptsafe This function does not throw exceptions.
     */
  void checkPullMsg();

  /**
     * @brief check for acknowledge of transmitted data
     *
     * @return nothing.
     *
     * @exceptsafe This function does not throw exceptions.
     */
  void checkPushMsg();
};

#endif
