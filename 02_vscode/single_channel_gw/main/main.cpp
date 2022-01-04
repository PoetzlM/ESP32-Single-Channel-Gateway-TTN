#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "esp_task_wdt.h"

#include "esp_log.h"

#include <LoRaModemSX127x.h>
#include <LoRaModemSX127xHandler.h>
#include <TtnGwUdpHandler.h>
#include <OneShotTimerInterrupt.h>
#include <TtnGwSerialUiConfig.h>
#include <TtnGwDataTypes.h>

#include <ezTime.h>

//==================================== DEFINES
#define GW_VERSION "0.1a"
#define SERIAL_CFG //comment this line out, if serial configuration should not be used
#define GW_DESCRIPTION "LORA GW"
#define GW_DEFINITION "single_channel_gw"


#ifndef SERIAL_CFG
    #define LOCAL_PORT 1700
    #define REMOTE_PORT 1700
    //#define REMOTE_IP "eu1.cloud.thethings.network"
    #define REMOTE_IP "52.212.223.226"

    #define REMOTE_DBG //comment this line out, if remote debug interface should not be active
    #define REMOTE_DBG_PORT 555
    #define REMOTE_DBG_IP "192.168.xx.xx"

    #define WIFI_SSID "xxxxx"
    #define WIFI_PASSWD "xxxxx"

    #define GW_EMAIL "xxxxx@xxxxx.xxxxx"

    #define GW_IDENTIFIER                                  \
        {                                                  \
            0x01, 0x23, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xCD \
        } //GW EUID


    //#define GW_IDENT_FROM_MAC //comment this line out, if the preset GW_IDENTIFIER should not be used -> gw identifier is generated from wifi mac adress of the esp32

    #define GW_LAT 48.15352170654199
    #define GW_LONG 11.552633669311625
    #define GW_ALT 517

#endif

//Interval (in seconds) of status message sent to ttn server
//after 1 minute the gateway is offline for the ttn console
//minimal interval 2, units in seconds
#define GW_UPDATE_INTERVAL 55
//size how many packets can be processed in a short interval
#define RX_QUEUE_SIZE 10
#define TX_QUEUE_SIZE 10
//how many messages can be stored to be sent at a specified timeframe
#define SCHED_QUEUE_SIZE 10

#define TX_UDP_TOUT_QUEUE_SIZE 10

//upper and lower limits to determin, if a packed schould be scheduled
//or thrown away
#define PKT_MAX_PAST 1     //packet maximal time delta in past frame, in seconds
#define PKT_MAX_FUTUR 3600 //Packet maximal time delta in future frame, in seconds

//how often to signalize the server that the gateway is ready to transmit data
//only after the first pull data message tx request are sent to the gw
//minimal interval 2, units seconds
#define PULL_DATA_INTERVAL 20

//if a sent udp message is not acknowledged, these are the paramters
//wich define the retry process
//minimal interval 2, units seconds
#define GW_UDP_RETRY_INTERVAL 10
#define GW_UDP_RETRY_DELETE_AFTER 20

//set the system timezone of the gateway
#define TIMEZONE "de"

//preload time: how long before the actual transmit the package should be sent
//from low prioity task to high prioity task
//time in ms
#define TX_TMST_PRELOAD 800 //programm process accuracy in us
#define TX_TMST_SEND_BEFORE 50
#define TX_TMMS_PRELOAD 800 //programm process accuracy in ms
#define TX_TMMS_SEND_BEFORE 50
#define TX_TIME_PRELOAD 1000 //programm process accurary in seconds
#define TX_TIME_SEND_BEFORE 50

// LoRa-module connection pinout
#define SS 5
#define RST 14
#define DIO0 12
#define DIO1 13
#define MISO 19
#define SCK 18
#define MOSI 23

// LoRa Radio config
#define LORA_LISTENING_SF 0x07
#define LORA_LISTENING_FREQU 868300000
#define LORA_SYNCWORD 0x34
#define LORA_PREAM_LEN 0x08
//Sx1276 max 10MHz
#define SPI_FREQUENCY 8E6
//============================================



// WiFi login data
//const char *ssid = WIFI_SSID;       // WiFi Name
//const char *password = WIFI_PASSWD; // WiFi Password

//WIfi Udp Interface
WiFiUDP udpIf;

#if defined REMOTE_DBG
//WiFi Udp Interface for debugging
WiFiUDP udpDbgIf;
#endif

//TTN specific interface
//TtnUdpPackageProcessor ttnPkgParser; // TTN UDP Packet parser
//TtnTime ttnTimeData;                 // TTN Time data, loaded via Ethernet interface

//Queue for Rx and Tx packets
QueueHandle_t txQueue;
QueueHandle_t rxQueue;
QueueHandle_t schedQueue;
QueueHandle_t txUdpToutQueue;

//High precision Timer interrupt
OneShotTimerInterrupt oneShotTimerInterrupt;
//helper variables to callibrate the execution time
int64_t callTime = 0;
bool calibrate = false;
bool setupComplete = false;
int64_t usStep = 80;
int64_t gblTransTime = 0;

//Serial configuration interface
TtnGWSerialUiConfig gwSerialUiConf;

//LoRa HW Interface
LoRaModemSX127xHandler ModemHandler1;

//Setup LoRa Modem================================================================
//================================================================================
void initLoRa()
{
    Serial.println("Setup LoRa modem");

    // setup LoRa-module
    LoRaExt[0].setPins(SS, RST, DIO0, DIO1);

    while (!LoRaExt[0].begin(LORA_LISTENING_FREQU))
    {
        Serial.print(".");
        delay(500);
    }

    Serial.println("Setup LoRa modem done");

    LoRaExt[0].setSpreadingFactor(LORA_LISTENING_SF, true);
    LoRaExt[0].setSyncWord(LORA_SYNCWORD, true);
    LoRaExt[0].setPreambleLength(LORA_PREAM_LEN, true);
    LoRaExt[0].setSPIFrequency(SPI_FREQUENCY);

    ModemHandler1.begin(&LoRaExt[0]);

    ModemHandler1.setAdditionalInfo(1, 2);
    ModemHandler1.setDisableTx(false);
    ModemHandler1.setIgnorAllTxFilter(true);
    ModemHandler1.setIgnoreReqTxPower(true);
    ModemHandler1.setIgnoreReqPolarization(false);

    LoRaExt[0].cadScan(false);
    Serial.println("==============================================");
    LoRaExt[0].printModemConfig();
    Serial.println("==============================================");

    Serial.println("Setup of ESP-LoRa-Receiver is finished!");
}

//Setup WiFi connection===========================================================
//================================================================================

void initWifi()
{
    Serial.print("Connecting to ");
    
    #ifdef  SERIAL_CFG
        Serial.print(gwSerialUiConf.storedData.wifiSSID);
        WiFi.begin(gwSerialUiConf.storedData.wifiSSID, gwSerialUiConf.storedData.wifiPW);
    #else
        Serial.print(WIFI_SSID);
        WiFi.begin(WIFI_SSID, WIFI_PASSWD);
    #endif
    
    
    //Serial.println(ssid);
    //WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }

    Serial.println();
    Serial.println("WiFi connected!");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

    delay(1500);
}

void DogFeedDelay(int delay_ms)
{
    vTaskDelay(delay_ms / portTICK_PERIOD_MS);
    yield();
}

//LOW PRIO TASK ==================================================================
//UDP Packet handling
//================================================================================

TtnGwUdpHandler twUdpHndlr;

void lowPriorityTask(void *parameter)
{
    // Setup basic config of the GW

    LoRaGwGeoData_t gwGeoData;
    LoRaGwInfo_t gwInfo;

        char def[] = GW_DEFINITION;
        memcpy(gwInfo.platform_definition, def, strlen(def));

        char desc[] = GW_DESCRIPTION;
        memcpy(gwInfo.platform_description, desc, strlen(desc));    

    #ifdef  SERIAL_CFG

        memcpy(gwInfo.platform_email, gwSerialUiConf.storedData.email, strlen(gwSerialUiConf.storedData.email));

        gwGeoData.latitude = gwSerialUiConf.storedData.latitude;
        gwGeoData.longitude = gwSerialUiConf.storedData.longitude;
        gwGeoData.altitude_meters = gwSerialUiConf.storedData.altitude;
    #else

        char email[] = GW_EMAIL;
        memcpy(gwInfo.platform_email, email, strlen(email));

        gwGeoData.latitude = GW_LAT;
        gwGeoData.longitude = GW_LONG;
        gwGeoData.altitude_meters = GW_ALT;
    #endif
    

    twUdpHndlr.configQueue(&txQueue, &rxQueue, &schedQueue, &txUdpToutQueue);
    twUdpHndlr.configInfoQueueSizes(TX_QUEUE_SIZE, RX_QUEUE_SIZE, SCHED_QUEUE_SIZE, TX_UDP_TOUT_QUEUE_SIZE);
    twUdpHndlr.configTxPreload(TX_TMST_PRELOAD, TX_TMMS_PRELOAD, TX_TIME_PRELOAD);

    #ifdef  SERIAL_CFG
    twUdpHndlr.configUDP(gwSerialUiConf.storedData.remoteAdress, gwSerialUiConf.storedData.localPort, gwSerialUiConf.storedData.remotePort, GW_UDP_RETRY_DELETE_AFTER);
    #else
    twUdpHndlr.configUDP(REMOTE_IP, LOCAL_PORT, REMOTE_PORT, GW_UDP_RETRY_DELETE_AFTER);
    #endif

#if defined REMOTE_DBG
    twUdpHndlr.configDebugUDP(&udpDbgIf, REMOTE_DBG_IP, REMOTE_DBG_PORT);
#endif
    twUdpHndlr.configPacketTimeframe(PKT_MAX_FUTUR, PKT_MAX_PAST);
    twUdpHndlr.configIntervall(GW_UPDATE_INTERVAL, PULL_DATA_INTERVAL, GW_UDP_RETRY_INTERVAL);

    twUdpHndlr.updateBasicInformation(gwGeoData, gwInfo);

    twUdpHndlr.ttnTimeData.begin();               //setup timezones and date/time strings for the Ttn Udp Package parsing
    twUdpHndlr.ttnTimeData.setTimeZone(TIMEZONE); //set system to defined Time Zone

    twUdpHndlr.begin();

    #if defined GW_IDENTIFIER
    //set unique identifier
    uint8_t ident[] = GW_IDENTIFIER;
    twUdpHndlr.ttnPkgParser.setUniqueIdentifier(ident);
    #endif
    
//generieren eine automatische UID abhängig von der Mac Adresse
#if defined (GW_IDENT_FROM_MAC) || (defined SERIAL_CFG)
    byte WiFiMac[6];
    WiFi.macAddress(WiFiMac);

    twUdpHndlr.ttnPkgParser.setUniqueIdentFromMacEth(WiFiMac);
#endif

    Serial.println("==============================================");
    twUdpHndlr.ttnPkgParser.printIdentifier();
    Serial.println("==============================================");

    while (true)
    {
        //send status update to ttn server
        //================================
        twUdpHndlr.statusUpdate();

        //re-send not acknoledeged packets
        //================================
        twUdpHndlr.checkNotAckMsg();

        //get Data from *rxQueue, send to ttn server
        //data transfer from high prio task to low prio task
        //================================
        twUdpHndlr.upadteRxQueue();

        //check wich data should be put in TxQueue
        //unload the schedule queu to high prio task if the right time has com
        //=======================================
        twUdpHndlr.updateTxQueue();

        //UDP Interface
        //====================================
        if (setupComplete)
        {
            twUdpHndlr.readUdp();
        }

        //data transfer from udp interface to high prio task, to transmit the data
        //or store the data in a queue, to transmit the data if the time arrises
        //================================
        twUdpHndlr.checkPullMsg();

        //Acknoledge of transmitted data
        //data can be deleted from the retry queue
        //=============================
        twUdpHndlr.checkPushMsg();

        DogFeedDelay(10);
    }
    vTaskDelete(NULL);
}

//Timer interrupt funciton========================================================
//================================================================================
static void highPrioTimerInterrupt(void *arg)
{
    
    uint8_t dataIn;
    xQueueReceive(ModemHandler1.queueTimerInterruptIn, &dataIn, portMAX_DELAY);

    callTime = esp_timer_get_time();
    if (calibrate)
    {
        return;
    }

    ModemHandler1.interface[dataIn]->transmit(true);
    int64_t tStamp = esp_timer_get_time();
    //Serial.print("beginTransmit: ");
    //Serial.println(tStamp);
    Serial.print("TimeError us: ");
    int64_t tError = gblTransTime - tStamp;
    int tErrorSmall = tError;
    Serial.println(tErrorSmall);

    //hast to be in range of 10ms or the change will be rejected
    //protects against timer overrun
    if ((tErrorSmall < 10000) && (tErrorSmall < -10000))
    {
        usStep -= gblTransTime - tStamp;
    }

    //ModemHandler1._interface[dataIn]->printModemConfig();
}

//HIGH PRIO TASK ==================================================================

void highPriorityTask(void *parameter)
{
    //init empty receiver paket structure
    LoRaDataPktTx_t loraPktTx;
    memset(&loraPktTx, 0, sizeof(LoRaDataPktTx_t));
    //muss noch überprüft werden warum msg_len nicht mit 0 init wird....
    //ModemHandler1._loraDataPacketRx.msg_len = 0;

    TtnGwTime ttnTimeData;
    ttnTimeData.setTimeZone(TIMEZONE);

    while (true)
    {
        ModemHandler1.process();

        //process the reiceived data
        if (ModemHandler1.loraDataPacketRx.msg_len != 0)
        {
            //Serial.print("rec msg len: ");
            //Serial.println(ModemHandler1._loraDataPacketRx.msg_len);

            //send packet to reiceiver Task
            xQueueSend(rxQueue, &ModemHandler1.loraDataPacketRx, portMAX_DELAY);
            ModemHandler1.loraDataPacketRx.msg_len = 0;
        }

        //check wich data should be put up for transfer
        //=======================================
        for (int i = 0; i < uxQueueMessagesWaiting(txQueue); i++)
        {
            xQueueReceive(txQueue, &loraPktTx, portMAX_DELAY);

            if (loraPktTx.timestamp_tx != 0)
            {
                //if (ttnTimeData.isInPastTime_us(loraPktTx.timestamp_tx - (TX_TMST_SEND_BEFORE * 1000) - (msStep * 1000)))
                if (ttnTimeData.isInPastTime_us(loraPktTx.timestamp_tx - (TX_TMST_SEND_BEFORE * 1000)))
                {

                    //generate valid 64 bit timestamp
                    bool temp = ModemHandler1.requestTransmit(&loraPktTx);
                    int64_t tempStamp = esp_timer_get_time();
                    tempStamp = tempStamp & 0xFFFFFFFF00000000;
                    tempStamp += loraPktTx.timestamp_tx;

                    gblTransTime = tempStamp;
                    oneShotTimerInterrupt.callOn(tempStamp - usStep);
                }
                else
                {
                    //put packet in queue to be scheduled
                    xQueueSend(txQueue, &loraPktTx, portMAX_DELAY);
                }
            }
            //not properly implemented. see documentation of the project.
            else if (loraPktTx.gps_time_tx != 0)
            {
                if (ttnTimeData.isInPastGpsTime_ms(loraPktTx.gps_time_tx - TX_TMMS_SEND_BEFORE))
                {
                    ModemHandler1.requestTransmit(&loraPktTx);
                }
                else
                {

                    //put packet in queue to be scheduled
                    xQueueSend(txQueue, &loraPktTx, portMAX_DELAY);
                }
            }
            //not properly implemented. see documentation of the project.
            else if (loraPktTx.unix_timestamp != 0)
            {
                double temp = (double)loraPktTx.unix_timestamp + (double)(TX_TIME_SEND_BEFORE / 1000.000);
                if (ttnTimeData.isInPastUnixTime_s(temp))
                {
                    ModemHandler1.requestTransmit(&loraPktTx);
                }
                else
                {
                    //put packet in queue to be scheduled
                    xQueueSend(txQueue, &loraPktTx, portMAX_DELAY);
                }
            }
        }

        DogFeedDelay(10);
    }
    vTaskDelete(NULL);
}

void setup()
{
    initArduino();
    Serial.begin(112500);

    Timezone dummyCompile;
    dummyCompile.setTime(compileTime());

    Serial.println("Single Channel Gateway========================");
    Serial.print("Version: ");
    Serial.println(GW_VERSION);
    Serial.print("Compile date/time: ");
    Serial.println(dummyCompile.dateTime(ISO8601));
    Serial.println("==============================================");

#if defined SERIAL_CFG
    //Serial configuration user interface
    gwSerialUiConf.begin();
    gwSerialUiConf.printConfig();


        //simple check if data is loaded correctly from flash
        if (gwSerialUiConf.checkStorage())
        {
            gwSerialUiConf.readUntilTimeout("reconfig gateway (y/n), timeout after 10s", 10);
            if (gwSerialUiConf.checkYes())
            {
                gwSerialUiConf.getNewConfig();
            }
        }
        else
        {
            Serial.println("no valid configuration found, enter new configuration");
            Serial.println("=====================================================");
            
            gwSerialUiConf.readUntilTimeout("reconfig gateway, enter (y)", 10);
            
            while (!gwSerialUiConf.checkYes())
            {
                gwSerialUiConf.readUntilTimeout("reconfig gateway, enter (y)", 10);
            }
            gwSerialUiConf.getNewConfig();

        }

    

#endif

    initWifi(); //connect to wifi and get IP Adress

#if defined REMOTE_DBG
    udpDbgIf.begin(REMOTE_DBG_PORT);
#endif

    //queue aufsetzten
    rxQueue = xQueueCreate(RX_QUEUE_SIZE, sizeof(LoRaDataPktRx_t));
    txQueue = xQueueCreate(TX_QUEUE_SIZE, sizeof(LoRaDataPktTx_t));
    schedQueue = xQueueCreate(SCHED_QUEUE_SIZE, sizeof(LoRaDataPktTx_t));

    txUdpToutQueue = xQueueCreate(TX_UDP_TOUT_QUEUE_SIZE, sizeof(TtnUdpPacketOut_t));

    //setup all the LoRa Moduls and variables
    //before they should be used in the task!
    initLoRa();

    xTaskCreate(
        lowPriorityTask,   /* Task function. */
        "lowPriorityTask", /* name of task. */
        10000,             /* Stack size of task */
        NULL,              /* parameter of the task */
        1,                 /* priority of the task */
        NULL);             /* Task handle to keep track of created task */

    /* let lowPriorityTask run first then create highPriorityTask */
    xTaskCreate(
        highPriorityTask,   /* Task function. */
        "highPriorityTask", /* name of task. */
        10000,              /* Stack size of task */
        NULL,               /* parameter of the task */
        4,                  /* priority of the task */
        NULL);              /* Task handle to keep track of created task */

    //setup interrupt timer and callibrate timer callbacks

    oneShotTimerInterrupt.begin(&highPrioTimerInterrupt);
    oneShotTimerInterrupt.configCalibrateVar(&callTime, &calibrate);

    //fill up queue, needed for testruns of the callibration
    for (int i = 0; i < 10; i++)
    {
        xQueueSend(ModemHandler1.queueTimerInterruptIn, &i, portMAX_DELAY);
    }

    Serial.println("start callibrate timer interrupts");
    //callibrate for a timeframe of 200ms, do 10 retrys for the callibration
    oneShotTimerInterrupt.calibrateCallOnDelta(200000, 10);

    for (int i = 0; i < 10; i++)
    {
        xQueueSend(ModemHandler1.queueTimerInterruptIn, &i, portMAX_DELAY);
    }

    oneShotTimerInterrupt.calibrateCallOn(200000, 10);
    Serial.println("callibrate Timer interrupt done");

    Serial.print("offset call on: ");
    Serial.print(oneShotTimerInterrupt.callOffset);
    Serial.print(" offset call on delta: ");
    Serial.println(oneShotTimerInterrupt.callOffsetDelta);

    setupComplete = true;
}

void loop()
{
    //uses delay with yield to feed the watchdog timer
    DogFeedDelay(1000);
}

extern "C" void app_main()
{
    setup();

    while (true)
    {
        loop();
    }
}
