//Simple Lib to get the basic config for the gw from the serial terminal.
//needs the Arduino framework
//Version 0.1
//Written by MPO
//License MIT

#include "TtnGwSerialUiConfig.h"

/// empty constructor
SerialInterface::SerialInterface()
{
    //unuesd at the moment
}

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
bool SerialInterface::readUntilTimeout(const char *msg, int timeout_s)
{
    Serial.setTimeout(100);
    Serial.println(msg);

    memset(input, 0, sizeof(input));

    int loops = (timeout_s * 10) / 2;

    for (int i = 0; i < loops; i++)
    {
        String sinput = Serial.readString();
        if (sinput.length() != 0)
        {
            //Serial.println(input);
            memcpy(input, sinput.c_str(), strlen(sinput.c_str()));
            return true;
        }
        delay(200);
        yield();
    }

    return false;
}

/**
 * @brief get user input without timeout, function is blocking user input is made.
 *
 * @param msg Message wich is displayed before the requested input.
 *
 * @return true if user input was made, false if no userinput was made.
 *
 * @exceptsafe This function does not throw exceptions.
 */

bool SerialInterface::readUntilInput(const char *msg)
{
    Serial.setTimeout(100);
    Serial.println(msg);
    memset(input, 0, sizeof(input));

    while (true)
    {
        String sinput = Serial.readString();
        if (sinput.length() != 0)
        {
            memcpy(input, sinput.c_str(), strlen(sinput.c_str()));
            return true;
        }
        delay(200);
        yield();
    }

    return false;
}

/**
 * @brief check the userinput if it is declining
 *
 * @return true if user input if it is declining, else return false.
 *
 * @exceptsafe This function does not throw exceptions.
 */

bool SerialInterface::checkNo()
{
    String sinput = String(input);

    if (sinput.indexOf("n") > -1)
    {
        return true;
    }

    if (sinput.indexOf("N") > -1)
    {
        return true;
    }

    return false;
}

/**
 * @brief check the userinput if it is accepting
 *
 * @return true if user input if it is accepting, else return false.
 *
 * @exceptsafe This function does not throw exceptions.
 */
bool SerialInterface::checkYes()
{
    String sinput = String(input);

    if (sinput.indexOf("y") > -1)
    {
        return true;
    }

    if (sinput.indexOf("Y") > -1)
    {
        return true;
    }

    return false;
}

///empty constructor
TtnGWSerialUiConfig::TtnGWSerialUiConfig()
{
    ////unuesd at the moment
}

/**
 * @brief initialize and read the previously stored data on flash
 *
 * @return nothing.
 *
 * @exceptsafe This function does not throw exceptions.
 */
void TtnGWSerialUiConfig::TtnGWSerialUiConfig::begin()
{
    EEPROM.begin(sizeof(flashData_t));
    EEPROM.get(0, storedData);
}

/**
 * @brief print the loaded config to serial
 *
 * @return nothing.
 *
 * @exceptsafe This function does not throw exceptions.
 */
void TtnGWSerialUiConfig::printConfig()
{
    Serial.print("HAMAC:                      ");
    for (uint8_t i = 0; i < 32; i++)
    {
        Serial.print(storedData.hmac[i]);
    }
    Serial.println();

    Serial.print("Storage Version:            ");
    Serial.println(storedData.storageVersion);

    Serial.print("Wifi SSID:                  ");
    Serial.println(storedData.wifiSSID);

    Serial.print("listening frequency (Hz):   ");
    Serial.println(storedData.frequency);

    Serial.print("listening spreading factor: ");
    Serial.println(storedData.spreadFact);

    Serial.print("Server adress:              ");
    Serial.println(storedData.remoteAdress);

    Serial.print("Server port:                ");
    Serial.println(storedData.remotePort);

    Serial.print("local port:                 ");
    Serial.println(storedData.localPort);

    Serial.print("email:                      ");
    Serial.println(storedData.email);

    Serial.print("latitude:                   ");
    Serial.println(storedData.latitude);

    Serial.print("longitude:                  ");
    Serial.println(storedData.longitude);

    Serial.print("altitude:                   ");
    Serial.println(storedData.altitude);
}

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

bool TtnGWSerialUiConfig::readUntilTimeout(char *msg, int timeout)
{
    return _serialIF.readUntilTimeout(msg, timeout);
}

/**
 * @brief check the userinput if it is accepting
 *
 * @return true if user input if it is accepting, else return false.
 *
 * @exceptsafe This function does not throw exceptions.
 */
bool TtnGWSerialUiConfig::checkYes()
{
    return _serialIF.checkYes();
}

/**
 * @brief check the userinput if it is declining
 *
 * @return true if user input if it is declining, else return false.
 *
 * @exceptsafe This function does not throw exceptions.
 */
bool TtnGWSerialUiConfig::checkNo()
{
    return _serialIF.checkNo();
}

/**
 * @brief check if the loaded storage version is matching the current structure
 *
 * @return true if the structure is compatible
 *
 * @exceptsafe This function does not throw exceptions.
 */
bool TtnGWSerialUiConfig::checkStorage()
{
    if (storedData.storageVersion != STRUCT_STORAGE_VERSION)
    {
        Serial.println("Data struct version not compatible.");
        //return false;
    }

    if (checkDataIntegrity())
    {
        return true;
    }
    else
    {
         Serial.println("Data integrity not ok.");
        return false;
    }
    return false;
}

/**
 * @brief check if the integrity of the loaded data is ok
 *
 * @return true if the integrity is ok
 *
 * @exceptsafe This function does not throw exceptions.
 */

bool TtnGWSerialUiConfig::checkDataIntegrity()
{
    uint8_t hmac[32];

    //startup the crypto engine
    mbedtls_md_context_t ctx;
    mbedtls_md_type_t md_type = MBEDTLS_MD_SHA256;
    const size_t keyLength = strlen(hmacKey);
    mbedtls_md_init(&ctx);

    mbedtls_md_setup(&ctx, mbedtls_md_info_from_type(md_type), 1);
    mbedtls_md_hmac_starts(&ctx, (const unsigned char *)hmacKey, keyLength);
    mbedtls_md_hmac_update(&ctx, (const uint8_t *)&storedData + 32, sizeof(flashData_t) - 32);

    //save the generated HMAC in the correct data field
    bool fail = mbedtls_md_hmac_finish(&ctx, hmac);
    mbedtls_md_free(&ctx);

    //hmac creation failed, invalid parameters
    if (fail != 0)
    {
        Serial.println("hmac creation failed.");
        return false;
    }

    // HMAC created
    for (int i = 0; i < 32; i++)
    {
       if (storedData.hmac[i] != hmac[i])
       {
           return false;
       }
    }

    return true;
    
    /*
    if (memcmp(storedData.hmac, hmac, 32))
    {
        //ceated HMAC matches stored HMAC
        return true;
    }
    else
    {
        return false;
    }

    return false;

    */

}

/**
 * @brief generate checksum for the current data structure
 *
 * @return true if creation of checksum was sucessfully
 *
 * @exceptsafe This function does not throw exceptions.
 */

bool TtnGWSerialUiConfig::updateChecksum()
{

    //startup the crypto engine
    mbedtls_md_context_t ctx;
    mbedtls_md_type_t md_type = MBEDTLS_MD_SHA256;
    const size_t keyLength = strlen(hmacKey);
    mbedtls_md_init(&ctx);

    mbedtls_md_setup(&ctx, mbedtls_md_info_from_type(md_type), 1);
    mbedtls_md_hmac_starts(&ctx, (const unsigned char *)hmacKey, keyLength);
    mbedtls_md_hmac_update(&ctx, (const uint8_t *)&storedData + 32, sizeof(flashData_t) - 32);

    //save the generated HMAC in the correct data field
    bool fail = mbedtls_md_hmac_finish(&ctx, storedData.hmac);
    mbedtls_md_free(&ctx);

    //hmac creation failed, invalid parameters
    if (fail != 0)
    {
        Serial.println("hmac creation failed (update checksum).");
        return false;
    }
    else
    {
        return true;
    }
}

/**
 * @brief create new configuration, get all the neccessary user inputs to reconfigure the gateway
 * and save it to the flash storage.
 * ================ caution, at the moment no sanity checks are applied! ================
 * @return nothing.
 *
 * @exceptsafe This function does not throw exceptions.
 */
void TtnGWSerialUiConfig::getNewConfig()
{
    memset(&storedData, 0, sizeof(flashData_t));

    _serialIF.readUntilInput("Set Wifi SSID:");
    Serial.print("New Wifi SSID: ");
    memcpy(storedData.wifiSSID, _serialIF.input, strlen(_serialIF.input));
    Serial.println(_serialIF.input);

    _serialIF.readUntilInput("Wifi password:");
    memcpy(storedData.wifiPW, _serialIF.input, strlen(_serialIF.input));
    Serial.println("Password Input OK");

    _serialIF.readUntilInput("Listening frequency (in MHz):");
    Serial.print("New Listening Frequncy in Hz: ");
    float frequency_mhz = String(_serialIF.input).toFloat();
    frequency_mhz *= 1000;
    uint32_t frequency = round(frequency_mhz);
    frequency *= 1000;
    storedData.frequency = frequency;
    Serial.println(frequency);

    _serialIF.readUntilInput("Listening SpreadFactor (7-12):");
    Serial.print("Listening Spreading Factor: ");
    uint8_t spreadFactor = String(_serialIF.input).toInt();
    Serial.println(spreadFactor);
    storedData.spreadFact = spreadFactor;

    _serialIF.readUntilInput("Set server adress (IP):");
    Serial.print("Server adress: ");
    memcpy(storedData.remoteAdress, _serialIF.input, strlen(_serialIF.input));
    Serial.println(_serialIF.input);

    _serialIF.readUntilInput("Set server port (TTN Server requires 1700):");
    Serial.print("Server port: ");
    storedData.remotePort = String(_serialIF.input).toInt();
    Serial.println(_serialIF.input);

    _serialIF.readUntilInput("Set local port (TTN Server requires 1700):");
    Serial.print("local port: ");
    storedData.localPort = String(_serialIF.input).toInt();
    Serial.println(_serialIF.input);

    _serialIF.readUntilInput("Set email adress:");
    Serial.print("New email adress: ");
    memcpy(storedData.email, _serialIF.input, strlen(_serialIF.input));
    Serial.println(_serialIF.input);

    _serialIF.readUntilInput("Set latitude of the GW (as float):");
    Serial.print("Latitude: ");
    float latitude = String(_serialIF.input).toFloat();
    Serial.println(latitude);
    storedData.latitude = latitude;

    _serialIF.readUntilInput("Set longitude of the GW (as float):");
    Serial.print("Longitude: ");
    float longitude = String(_serialIF.input).toFloat();
    Serial.println(longitude);
    storedData.longitude = longitude;

    _serialIF.readUntilInput("Set altitude of the GW (as int in meter):");
    Serial.print("Altitude: ");
    int altitude = String(_serialIF.input).toInt();
    Serial.println(altitude);
    storedData.altitude = altitude;

    updateChecksum();

    Serial.println("write to flash");
    storedData.storageVersion = STRUCT_STORAGE_VERSION;
    EEPROM.put(0, storedData);
    EEPROM.end();
}
