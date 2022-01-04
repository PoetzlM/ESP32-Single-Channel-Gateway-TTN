//Simple Lib to get the basic config for the gw from the serial terminal.
//needs the Arduino framework
//Version 0.1
//Written by MPO

#ifndef TTNGWSERIALCONFIG_H
#define TTNGWSERIALCONFIG_H

#include <Arduino.h>
#include <EEPROM.h>
#include "mbedtls/md.h"

//increment this counter if you change the flashData Structure
#define STRUCT_STORAGE_VERSION 2

/**
 * SerialInterface class to get c-strings from the serial interface.
 * with basic comparisions for yes and no inputs from the user
 */

class SerialInterface
{

public:
    /// buffer wich contains the user input
    char input[1024];

public:
    /// empty constructor
    SerialInterface();

    /**
     * @brief get user input within defined timeout, function is blocking until the timeout is completed or user input is made.
     *
     * @param msg Message wich is displayed before the requested input.
     * @param timeout_s how long the function is blocking and the user gets to input the data.
     *
     * @return true if user input was made, false if no userinput was made.
     *
     * @exceptsafe This function does not throw exceptions.
     */
    bool readUntilTimeout(const char *msg, int timeout_s);

    /**
     * @brief get user input without timeout, function is blocking user input is made.
     *
     * @param msg Message wich is displayed before the requested input.
     *
     * @return true if user input was made, false if no userinput was made.
     *
     * @exceptsafe This function does not throw exceptions.
     */

    bool readUntilInput(const char *msg);

    /**
     * @brief check the userinput if it is declining
     *
     * @return true if user input if it is declining, else return false.
     *
     * @exceptsafe This function does not throw exceptions.
     */

    bool checkNo();

    /**
     * @brief check the userinput if it is accepting
     *
     * @return true if user input if it is accepting, else return false.
     *
     * @exceptsafe This function does not throw exceptions.
     */
    bool checkYes();
};
/**
 * simple strucure to store user input.
 *
 * Values are later used to configure the gateway
 */
typedef struct flashData
{
    /// HAMAC of the stored data
    uint8_t hmac[32];
    /// version of the structure, changes on extension of the structure to maintain compatibility.
    uint8_t storageVersion;
    /// SSID of the WiFi
    char wifiSSID[30];
    /// Password of the WiFi
    char wifiPW[100];
    /// email adress wich is connected to the gateway
    char email[100];
    /// listening frequency of the single channel gateway
    uint32_t frequency;
    /// listening spread factor fo the single channel gateway
    uint8_t spreadFact;
    /// remote adress of the ttn server to connect
    char remoteAdress[100];
    /// remote port of the ttn server to connect
    uint16_t remotePort;
    /// local port for the udp semtech protocol
    uint16_t localPort;
    /// latitude, descreptiv position of the gateway
    float latitude;
    /// longitude, descreptiv position of the gateway
    float longitude;
    /// altitude, descreptiv position of the gateway
    uint32_t altitude;
} flashData_t;

/**
 * TtnGWSerialUiConfig class to get specific configuration data from the user
 */
class TtnGWSerialUiConfig
{
private:
    /// serial interface to get the data from Serial
    SerialInterface _serialIF;
    /// data structure to store the configured values
    char hmacKey[8] = "mpo-key";

public:
    flashData_t storedData;

    ///empty constructor
    TtnGWSerialUiConfig();

    /**
     * @brief initialize and read the previously stored data on flash
     *
     * @return nothing.
     *
     * @exceptsafe This function does not throw exceptions.
     */
    void begin();

    /**
     * @brief print the loaded config to serial
     *
     * @return nothing.
     *
     * @exceptsafe This function does not throw exceptions.
     */
    void printConfig();

    /**
     * @brief get user input within defined timeout, function is blocking until the timeout is completed or user input is made.
     *
     * @param msg Message wich is displayed before the requested input.
     * @param timeout_s how long the function is blocking and the user gets to input the data.
     *
     * @return true if user input was made, false if no userinput was made.
     *
     * @exceptsafe This function does not throw exceptions.
     */

    bool readUntilTimeout(char *msg, int timeout);

    /**
     * @brief check the userinput if it is accepting
     *
     * @return true if user input if it is accepting, else return false.
     *
     * @exceptsafe This function does not throw exceptions.
     */
    bool checkYes();

    /**
     * check the userinput if it is declining
     *
     * @return true if user input if it is declining, else return false.
     *
     * @exceptsafe This function does not throw exceptions.
     */

    bool checkNo();

    /**
     * @brief check if the loaded storage version is matching the current structure
     *
     * @return true if the structure is compatible
     *
     * @exceptsafe This function does not throw exceptions.
     */
    bool checkStorage();

    /**
     * @brief check if the integrity of the loaded data is ok
     *
     * @return true if the integrity is ok
     *
     * @exceptsafe This function does not throw exceptions.
     */
    bool checkDataIntegrity();

    /**
     * generate checksum for the current data structure
     *
     * @return true if creation of checksum was sucessfully
     *
     * @exceptsafe This function does not throw exceptions.
     */
    bool updateChecksum();

    /**
     * @brief create new configuration, get all the neccessary user inputs to reconfigure the gateway
     * and save it to the flash storage.
     * ================ caution, at the moment no sanity checks are applied! ================
     * @return nothing.
     *
     * @exceptsafe This function does not throw exceptions.
     */
    void getNewConfig();
};

#endif
