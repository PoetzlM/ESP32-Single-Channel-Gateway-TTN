//Simple Lib to setup and callibrate esp32 one shot timer interrupts
//requires:
//  - esp-idf framework
//Version 0.1
//Written by MPO
//License MIT

#ifndef ONESHOTTIMERINTERRUPT_H
#define ONESHOTTIMERINTERRUPT_H

#include "esp_timer.h"
#include "esp_log.h"
#include <unistd.h>

/**
 * simple class to setup the esp32 oneshot timer system
 */
class OneShotTimerInterrupt
{
private:
    int64_t *_callTime; ///< used in the callibrate function to get the actual call time
    bool *_calibrate;   ///< set the function to call in an calibrate mode to meassure the execution time up to this point

public:
    esp_timer_handle_t oneshot_timer;           ///< the actual one shot timer provided by the esp-idf framework
    esp_timer_create_args_t oneshot_timer_args; ///< configuration structure for the one shot timer
    int64_t callOffsetDelta = 0;                ///< callibrated offset in us, if the timer is executed relative to the current timestamp
    int64_t callOffset = 0;                     ///< callibrated offset in us, if the timer is absolute to the current timestamp
    inline static int gblInstance = 0;          ///< global instance counter
    int Instance = 0;                           ///< current instance counter, used for naming the counter configuration, informative if class needs debugging

    /**
    * @brief constructor for OneShotTimerInterrupt, setup the instance number.
    *
    * @exceptsafe This function does not throw exceptions.
    */
    OneShotTimerInterrupt();

    /**
    * @brief set up the references to the necessary callibration variables, if the callibration routine is used
    *
    * @param callTime pointer to the generated timestamp as reference to the execution time.
    * @param calibrate pointer to the flag to set the execution routine in a dryrun state.
    *
    * @return nothing.
    *
    * @exceptsafe This function does not throw exceptions.
    */
    void configCalibrateVar(int64_t *callTime, bool *calibrate);

    /**
    * @brief configure the function to be called on specified timerinterrupt
    *
    * @param callback function to call
    *
    * @return nothing.
    *
    * @exceptsafe This function does not throw exceptions.
    */
    void begin(void (*callback)(void *arg));

    /**
    * @brief delte the generated timer structures
    *
    * @return nothing.
    *
    * @exceptsafe This function does not throw exceptions.
    */
    void shutdown();

    /**
    * @brief set the next timerinterrupt relative to the current timestamp
    *
    * @return nothing.
    *
    * @exceptsafe This function does not throw exceptions.
    */
    void callOnDelta(uint64_t timestamp);

    /**
    * @brief set the next timerinterrupt absolut to the current timestamp
    *
    * @return nothing.
    *
    * @exceptsafe This function does not throw exceptions.
    */
    void callOn(uint64_t timestamp);

    /**
    * @brief callibrate the offset for callOnDelta function
    *
    * @param callTimeCallibrate expected timeframe between configuration of the interrupt and actual execution (use the shortest possible time if uncertain).
    * @param noRetrys how often the configuration should be testete.
    *
    * @return nothing.
    *
    * @exceptsafe This function does not throw exceptions.
    */
    void calibrateCallOnDelta(int64_t callTimeCallibrate = 3000000, int noRetrys = 5);

    /**
    * @brief callibrate the offset for callOn function, use the @see calibrateCallOnDelta function beforehand.
    *
    * @param callTimeCallibrate expected timeframe between configuration of the interrupt and actual execution (use the shortest possible time if uncertain).
    * @param noRetrys how often the configuration should be testete.
    *
    * @return nothing.
    *
    * @exceptsafe This function does not throw exceptions.
    */
    void calibrateCallOn(int64_t callTimeCallibrate = 3000000, int noRetrys = 5);
};

#endif
