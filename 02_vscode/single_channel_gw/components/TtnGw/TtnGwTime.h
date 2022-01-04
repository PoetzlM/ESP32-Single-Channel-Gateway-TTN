//Simple Lib to generate timestrings necessary for the udp semtech protocoll.
//requirements:
// - esp-idf
// - freertos
// - Arduino framework
// - eztime lib
//Version 0.1
//Written by MPO

#ifndef TTGWNTIME_H
#define TTGWNTIME_H

#include <Arduino.h>
#include <ezTime.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

/**
* @brief convert c-string wich contains time to unix timestamp
*
* @return integeer representation of the unix timestamp
*
* @exceptsafe This function does not throw exceptions.
*/
int timeStringToUnixTimestamp(const char *timestring, int strLen);

/**
 * class to generate c-strings for the diffrent time formats wich are used in the semtech udp protocoll
 */

class TtnGwTime
{
private:
    Timezone _systemTZ; ///< time structure to hold the current system timezone
    Timezone _gmtTZ;    ///< time structure to hold the greenwich mean time

    uint32_t _millis = 0;                     ///< helper variable, contain the milliseconds
    uint32_t _gpsTime = 0;                    ///< updated gps timestamp in seconds
    uint32_t _unixTime = 0;                   ///< updated unix timestamp in seconds
    uint32_t _compSec = 0;                    ///< actual estimated leap seconds
    uint32_t _scaleFactLeapSec = 73505222;    ///< scaling factor to determine an estimate for the past leap seconds from the time since the start of GPS.
    uint32_t _gpsStartTimeAsUnix = 315964800; ///< timestamp for 1980-01-06 00:00:00
    uint32_t _diffGpsUnixTime = 0;            ///< helper variable, difference from current timestamp to gps start time

    char _timeCompact[40];  ///< time compact format: e.g. "2021-12-26T17:17:14.6340Z"
    char _timeExpanded[40]; ///< time expanded format: e.g. "2021-12-26 18:15:55 GMT+01"

    SemaphoreHandle_t _updateTimeMutex; ///< semaphore to make the update of the c-strings thread save

public:
    /**
     * @brief constructor for the TtnGwTime class
     *
     * @exceptsafe This function does not throw exceptions.
     */
    TtnGwTime();

    /**
     * @brief init all timestructs and sync to ntp server
     *
     * @return nothing.
     *
     * @exceptsafe This function does not throw exceptions.
     */
    void begin();

    /**
     * @brief update all diffrent time formats
     *
     * @return nothing.
     *
     * @exceptsafe This function does not throw exceptions.
     */
    void update();

    /**
     * @brief check if given gps timestamp is within timeframe of future and past delta
     *
     * @param timeStamp gps timestamp in milliseconds to compare to.
     * @param frameFutur define the valid future timedelta in seconds.
     * @param framePast define the valid past timedelta in seconds.
     *
     * @return 1: valid in past, 2: valid in futur, -1: valid in past, -2: invalid in future
     *
     * @exceptsafe This function does not throw exceptions.
     */
    int8_t isInGpsTimeFrame_ms(uint32_t timeStamp, uint32_t frameFutur, uint32_t framePast);

    /**
     * @brief check if given unix timestamp is within timeframe of future and past delta
     *
     * @param timeStamp unix timestamp in seconds to compare to.
     * @param frameFutur define the valid future timedelta in seconds.
     * @param framePast define the valid past timedelta in seconds.
     *
     * @return 1: valid in past, 2: valid in futur, -1: valid in past, -2: invalid in future
     *
     * @exceptsafe This function does not throw exceptions.
     */
    int8_t isInUnixTimeFrame_s(double timeStamp, uint32_t frameFutur, uint32_t framePast);

    /**
     * @brief check if given internal timestamp is within timeframe of future and past delta
     *
     * @param timeStamp internal timestamp in micorseconds to compare to.
     * @param frameFutur define the valid future timedelta in seconds.
     * @param framePast define the valid past timedelta in seconds.
     *
     * @return 1: valid in past, 2: valid in futur, -1: valid in past, -2: invalid in future
     *
     * @exceptsafe This function does not throw exceptions.
     */
    int8_t isInTimeFrame_us(uint32_t timeStamp, uint32_t frameFutur, uint32_t framePast);

    /**
     * @brief check if given gps timestamp is in past
     *
     * @param gps timeStamp in milliseconds to compare to.
     *
     * @return true if in past, else false
     *
     * @exceptsafe This function does not throw exceptions.
     */
    bool isInPastGpsTime_ms(uint32_t timeStamp);

    /**
     * @brief check if given unix timestamp is in past
     *
     * @param timeStamp unix timestamp in seconds to compare to.
     *
     * @return true if in past, else false
     *
     * @exceptsafe This function does not throw exceptions.
     */
    bool isInPastUnixTime_s(double timeStamp);

    /**
     * @brief check if given internal timestamp is in past
     *
     * @param timeStamp internal timestamp in micorseconds to compare to.
     *
     * @return true if in past, else false
     *
     * @exceptsafe This function does not throw exceptions.
     */
    bool isInPastTime_us(uint32_t timeStamp);

    /**
     * @brief configure the system timezone via c-string abbrevation
     *
     * @param buffer c-string abbrevation e.g. "DE" for germany.
     *
     * @return nothing
     *
     * @exceptsafe This function does not throw exceptions.
     */
    void setTimeZone(const char* buffer);

    /**
     * @brief get last updated gps time, @see update
     *
     * @return gps time in seconds
     *
     * @exceptsafe This function does not throw exceptions.
     */
    uint32_t getGpsTime();

    /**
     * @brief get last updated gps time, @see update
     *
     * @return gps time in milliseconds
     *
     * @exceptsafe This function does not throw exceptions.
     */
    uint32_t getGpsTime_ms();

    /**
     * @brief get last updated unix timestamp, @see update
     *
     * @return unix time in seconds
     *
     * @exceptsafe This function does not throw exceptions.
     */
    uint32_t unixTimeStamp_s();

    /**
     * @brief get last updated unix timestamp, @see update
     *
     * @return unix time in milliseconds
     *
     * @exceptsafe This function does not throw exceptions.
     */
    uint32_t unixTimeStamp_ms();

    /**
     * @brief get current gps timestamp
     *
     * @return gps time in seconds
     *
     * @exceptsafe This function does not throw exceptions.
     */
    uint32_t actGpsTime();

    /**
     * @brief get current gps timestamp
     *
     * @return gps time in milliseconds
     *
     * @exceptsafe This function does not throw exceptions.
     */
    uint32_t actGpsTime_ms();

    /**
     * @brief get current unix timestamp
     *
     * @return unix time in seconds
     *
     * @exceptsafe This function does not throw exceptions.
     */
    uint32_t actUnixTimeStamp_s();

    /**
     * @brief get current unix timestamp
     *
     * @return unix time in milliseconds
     *
     * @exceptsafe This function does not throw exceptions.
     */
    uint32_t actUnixTimeStamp_ms();

    /**
     * @brief copy last updated time in ISO8601 compact format in a given buffer
     *
     * @return nothing
     *
     * @exceptsafe This function does not throw exceptions.
     */
    void getTimeCompactGmt(char *buffer);

    /**
     * @brief copy last updated time in ISO8601 expanded format in a given buffer
     *
     * @return nothing
     *
     * @exceptsafe This function does not throw exceptions.
     */
    void getTimeExpandedSystem(char *buffer);

    /**
     * @brief get last updated time in ISO8601 comapact format as a c-string pointer
     *
     * @return c-string pointer to buffer wich contains the ISO8601 compact formated c-string
     *
     * @exceptsafe This function does not throw exceptions.
     */
    const char *getTimeCompactGmt();

    /**
     * @brief get last updated time in ISO8601 expanded format as a c-string pointer
     *
     * @return c-string pointer to buffer wich contains the ISO8601 expanded formated c-string
     *
     * @exceptsafe This function does not throw exceptions.
     */
    const char *getTimeExpandedSystem();
};

#endif
