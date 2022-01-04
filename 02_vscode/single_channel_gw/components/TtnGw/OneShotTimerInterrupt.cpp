//Simple Lib to setup and callibrate esp32 one shot timer interrupts
//requires:
//  - esp-idf framework
//Version 0.1
//Written by MPO
//License MIT

#include "OneShotTimerInterrupt.h"

/**
* @brief constructor for OneShotTimerInterrupt, setup the instance number.
*
* @exceptsafe This function does not throw exceptions.
*/
OneShotTimerInterrupt::OneShotTimerInterrupt()
{
   Instance = Instance;
   gblInstance++;
}

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
void OneShotTimerInterrupt::configCalibrateVar(int64_t *callTime, bool *calibrate)
{
   _callTime = callTime;
   _calibrate = calibrate;
}

/**
* @brief configure the function to be called on specified timerinterrupt
*
* @param callback function to call
*
* @return nothing.
*
* @exceptsafe This function does not throw exceptions.
*/
void OneShotTimerInterrupt::begin(void (*callback)(void *arg))
{
   oneshot_timer_args.callback = callback;
   char buffer[50];
   sprintf(buffer, "one-shot%i", Instance);
   oneshot_timer_args.name = buffer;

   ESP_ERROR_CHECK(esp_timer_create(&oneshot_timer_args, &oneshot_timer));
}

/**
* @brief delte the generated timer structures
*
* @return nothing.
*
* @exceptsafe This function does not throw exceptions.
*/
void OneShotTimerInterrupt::shutdown()
{
   ESP_ERROR_CHECK(esp_timer_delete(oneshot_timer));
}

/**
* @brief set the next timerinterrupt relative to the current timestamp
*
* @return nothing.
*
* @exceptsafe This function does not throw exceptions.
*/
void OneShotTimerInterrupt::callOnDelta(uint64_t timestamp)
{
   esp_timer_start_once(oneshot_timer, timestamp - callOffsetDelta);
}

/**
* @brief set the next timerinterrupt absolut to the current timestamp
*
* @return nothing.
*
* @exceptsafe This function does not throw exceptions.
*/
void OneShotTimerInterrupt::callOn(uint64_t timestamp)
{
   int64_t currentTime = esp_timer_get_time();
   callOnDelta(timestamp - currentTime - callOffset);
}

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
void OneShotTimerInterrupt::calibrateCallOnDelta(int64_t callTimeCallibrate, int noRetrys)
{

   callOffsetDelta = 0;
   int64_t delta = 0;

   *_calibrate = true;
   for (int i = 0; i < noRetrys; i++)
   {
      int64_t timeCurrent = esp_timer_get_time();
      callOnDelta(callTimeCallibrate);
      usleep(callTimeCallibrate + 100000);
      delta += *_callTime - timeCurrent - callTimeCallibrate;
   }

   callOffsetDelta = delta / noRetrys;
   *_calibrate = false;
}

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
void OneShotTimerInterrupt::calibrateCallOn(int64_t callTimeCallibrate, int noRetrys)
{
   int64_t delta = 0;

   *_calibrate = true;
   for (int i = 0; i < noRetrys; i++)
   {
      int64_t timeCall = esp_timer_get_time() + callTimeCallibrate;
      callOn(timeCall);
      usleep(callTimeCallibrate + 100000);
      delta += *_callTime - timeCall;
   }

   callOffset = delta / noRetrys;
   *_calibrate = false;
}
