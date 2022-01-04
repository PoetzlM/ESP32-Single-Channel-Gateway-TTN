//Simple Lib to generate timestrings necessary for the udp semtech protocoll.
//requirements:
// - esp-idf
// - freertos
// - Arduino framework
// - eztime lib
//Version 0.1
//Written by MPO

#include "TtnGwTime.h"

/**
* @brief convert c-string wich contains time to unix timestamp
*
* @return integeer representation of the unix timestamp
*
* @exceptsafe This function does not throw exceptions.
*/
int timeStringToUnixTimestamp(const char *timestring, int strLen)
{
	tmElements_t tm;

	char tempString[6];
	memset(tempString, 0, 6);

	int tempCntr = 0;
	int extractedNumbers = 0;

	for (int i = 0; i < strLen; i++)
	{
		//check if current character is a number
		if ((timestring[i] >= '0') && (timestring[i] <= '9'))
		{
			tempString[tempCntr] = timestring[i];
			tempCntr++;
		}
		else
		{
			//escape one character
			//convert the temporary string in to an integer
			int tempNumber = atoi(tempString);
			memset(tempString, 0, 6);
			tempCntr = 0;
			extractedNumbers++;

			switch (extractedNumbers)
			{
			case 1:
				tm.Year = tempNumber - 1970;
				break;
			case 2:
				tm.Month = tempNumber;
				break;
			case 3:
				tm.Day = tempNumber;
				break;
			case 4:
				tm.Hour = tempNumber;
				break;
			case 5:
				tm.Minute = tempNumber;
				break;
			case 6:
				tm.Second = tempNumber;
				break;
			case 7:
				//escape the for loop
				i = strLen;
			default:
				break;
			}
		}
	}

	time_t test = makeTime(tm);

	int retVal = test;

	return retVal;
}

/**
 * @brief constructor for the TtnGwTime class
 *
 * @exceptsafe This function does not throw exceptions.
 */
TtnGwTime::TtnGwTime()
{
	_updateTimeMutex = xSemaphoreCreateMutex();
	memset(_timeCompact, 0, sizeof(_timeCompact));
	memset(_timeExpanded, 0, sizeof(_timeExpanded));
}

/**
 * @brief update all diffrent time formats
 *
 * @return nothing.
 *
 * @exceptsafe This function does not throw exceptions.
 */
void TtnGwTime::update()
{
	xSemaphoreTake(_updateTimeMutex, portMAX_DELAY);

	_unixTime = _gmtTZ.now();
	_millis = millis();
	_diffGpsUnixTime = _unixTime - 315964800;
	_compSec = _diffGpsUnixTime / _scaleFactLeapSec;
	_gpsTime = _diffGpsUnixTime + _compSec;

	memset(_timeCompact, 0, sizeof(_timeCompact));
	memset(_timeExpanded, 0, sizeof(_timeExpanded));

	String temp = _gmtTZ.dateTime("Y-m-d~TH:i:s.v~0~Z");
	memcpy(_timeCompact, temp.c_str(), strlen(temp.c_str()));

	String temp2 = _systemTZ.dateTime("Y-m-d H:i:s ~G~M~TP");

	//ohne die letzten 3 zeichen, sind die Minuten des Timezoen offsets.
	//diese werden beim TTN Server nicht zugelassen
	memcpy(_timeExpanded, temp2.c_str(), strlen(temp2.c_str()) - 3);

	xSemaphoreGive(_updateTimeMutex);
}

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
int8_t TtnGwTime::isInUnixTimeFrame_s(double timeStamp, uint32_t frameFutur, uint32_t framePast)
{
	double unixTime = (double)actUnixTimeStamp_ms() / 1000.00;

	if (timeStamp > unixTime)
	{
		//future
		double temp = timeStamp - unixTime;
		if (frameFutur >= temp)
		{
			//return ok in Futur
			return 2;
		}
		else
		{
			return -2;
		}
	}
	else
	{
		//past
		double temp = unixTime - timeStamp;
		if (framePast >= temp)
		{
			//return ok in past
			return 1;
		}
		else
		{
			return -1;
		}
	}

	return 0;
}

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
int8_t TtnGwTime::isInGpsTimeFrame_ms(uint32_t timeStamp, uint32_t frameFutur, uint32_t framePast)
{
	uint32_t gpsTime = actGpsTime_ms();

	if (timeStamp > gpsTime)
	{
		//future
		uint32_t temp = timeStamp - gpsTime;
		if ((frameFutur * 1000) >= temp)
		{
			//return ok in Futur
			return 2;
		}
		else
		{
			return -2;
		}
	}
	else
	{
		//past
		uint32_t temp = gpsTime - timeStamp;
		if ((framePast * 1000) >= temp)
		{
			//return ok in past
			return 1;
		}
		else
		{
			return -1;
		}
	}

	return 0;
}

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
int8_t TtnGwTime::isInTimeFrame_us(uint32_t timeStamp, uint32_t frameFutur, uint32_t framePast)
{
	uint32_t currentTs = esp_timer_get_time();

	bool pastTimeFrame = isInPastTime_us(timeStamp);
	//overrun handling
	//check the correct time Frame:

	if (pastTimeFrame)
	{
		if (currentTs >= timeStamp)
		{
			uint32_t delta = currentTs - timeStamp;
			if (delta < framePast * 1E6)
			{
				return 1;
			}
			else
			{
				return -1;
			}
		}
		else
		{
			//timer overflow handling
			uint32_t delta = currentTs + (0xFFFFFFFF - timeStamp);

			if (delta < framePast * 1E6)
			{
				return 1;
			}
			else
			{
				return -1;
			}
		}
	}
	else
	{
		if (currentTs <= timeStamp)
		{
			uint32_t delta = timeStamp - currentTs;
			if (delta < frameFutur * 1E6)
			{
				return 2;
			}
			else
			{
				return -2;
			}
		}
		else
		{
			//timer overflow handling
			uint32_t delta = (0xFFFFFFFF - currentTs) + timeStamp;

			if (delta < frameFutur * 1E6)
			{
				return 2;
			}
			else
			{
				return -2;
			}
		}
	}

	return 0;
}

/**
 * @brief check if given gps timestamp is in past
 *
 * @param gps timeStamp in milliseconds to compare to.
 *
 * @return true if in past, else false
 *
 * @exceptsafe This function does not throw exceptions.
 */
bool TtnGwTime::isInPastGpsTime_ms(uint32_t timeStamp)
{
	return actGpsTime_ms() >= timeStamp;
}

/**
 * @brief check if given unix timestamp is in past
 *
 * @param timeStamp unix timestamp in seconds to compare to.
 *
 * @return true if in past, else false
 *
 * @exceptsafe This function does not throw exceptions.
 */
bool TtnGwTime::isInPastUnixTime_s(double timeStamp)
{
	double unixTime = (double)actUnixTimeStamp_ms() / 1000.00;
	return unixTime >= timeStamp;
}

/**
 * @brief check if given internal timestamp is in past
 *
 * @param timeStamp internal timestamp in micorseconds to compare to.
 *
 * @return true if in past, else false
 *
 * @exceptsafe This function does not throw exceptions.
 */
bool TtnGwTime::isInPastTime_us(uint32_t timeStamp)
{
	uint32_t tempStamp = esp_timer_get_time();

	uint32_t delta1;
	uint32_t delta2;

	if (timeStamp <= tempStamp)
	{
		delta1 = tempStamp - timeStamp;
		if (delta1 < 0x7FFFFFFF)
		{
			//normaler compare
			//is in the past
			return true;
		}
		else
		{
			//overrun check
			//always in the futur
			return false;
		}
	}
	else
	{
		delta1 = timeStamp - tempStamp;
		if (delta1 < 0x7FFFFFFF)
		{
			//normal compare
			//is in the futur
			return false;
		}
		else
		{
			//overrun check
			//alsways in the past
			return true;
		}
	}

	return false;
}

/**
 * @brief get last updated gps time, @see update
 *
 * @return gps time in seconds
 *
 * @exceptsafe This function does not throw exceptions.
 */
uint32_t TtnGwTime::getGpsTime()
{
	return _gpsTime;
}

/**
 * @brief get last updated gps time, @see update
 *
 * @return gps time in milliseconds
 *
 * @exceptsafe This function does not throw exceptions.
 */
uint32_t TtnGwTime::getGpsTime_ms()
{
	return (_gpsTime * 1000) + _millis;
}

/**
 * @brief get current unix timestamp
 *
 * @return unix time in seconds
 *
 * @exceptsafe This function does not throw exceptions.
 */
uint32_t TtnGwTime::unixTimeStamp_s()
{
	return _unixTime;
}

/**
 * @brief get current unix timestamp
 *
 * @return unix time in milliseconds
 *
 * @exceptsafe This function does not throw exceptions.
 */
uint32_t TtnGwTime::unixTimeStamp_ms()
{
	return (_unixTime * 1000) + _millis;
}

/**
 * @brief get current gps timestamp
 *
 * @return gps time in seconds
 *
 * @exceptsafe This function does not throw exceptions.
 */
uint32_t TtnGwTime::actGpsTime()
{
	uint32_t diffGpsUnixTime = _gmtTZ.now() - 315964800;
	uint32_t compSec = _diffGpsUnixTime / _scaleFactLeapSec;
	return diffGpsUnixTime + compSec;
}

/**
 * @brief get current gps timestamp
 *
 * @return gps time in milliseconds
 *
 * @exceptsafe This function does not throw exceptions.
 */
uint32_t TtnGwTime::actGpsTime_ms()
{
	uint32_t diffGpsUnixTime = _gmtTZ.now() - 315964800;
	uint32_t compSec = _diffGpsUnixTime / _scaleFactLeapSec;
	return ((diffGpsUnixTime + compSec) * 1000) + millis();
}

/**
 * @brief get current unix timestamp
 *
 * @return unix time in seconds
 *
 * @exceptsafe This function does not throw exceptions.
 */
uint32_t TtnGwTime::actUnixTimeStamp_s()
{
	return _gmtTZ.now();
}

/**
 * @brief get current unix timestamp
 *
 * @return unix time in milliseconds
 *
 * @exceptsafe This function does not throw exceptions.
 */
uint32_t TtnGwTime::actUnixTimeStamp_ms()
{
	return (_gmtTZ.now() * 1000) + millis();
}

/**
 * @brief init all timestructs and sync to ntp server
 *
 * @return nothing.
 *
 * @exceptsafe This function does not throw exceptions.
 */
void TtnGwTime::begin()
{
	ezt::waitForSync();
}

/**
 * @brief configure the system timezone via c-string abbrevation
 *
 * @param buffer c-string abbrevation e.g. "DE" for germany.
 *
 * @return nothing
 *
 * @exceptsafe This function does not throw exceptions.
 */
void TtnGwTime::setTimeZone(const char *buffer)
{
	_systemTZ.setLocation(F(buffer));
}

/**
 * @brief get last updated time in ISO8601 comapact format as a c-string pointer
 *
 * @return c-string pointer to buffer wich contains the ISO8601 compact formated c-string
 *
 * @exceptsafe This function does not throw exceptions.
 */
const char *TtnGwTime::getTimeCompactGmt()
{
	return _timeCompact;
}

/**
 * @brief get last updated time in ISO8601 expanded format as a c-string pointer
 *
 * @return c-string pointer to buffer wich contains the ISO8601 expanded formated c-string
 *
 * @exceptsafe This function does not throw exceptions.
 */
const char *TtnGwTime::getTimeExpandedSystem()
{
	return _timeExpanded;
}

/**
 * @brief copy last updated time in ISO8601 compact format in a given buffer
 *
 * @return nothing
 *
 * @exceptsafe This function does not throw exceptions.
 */
void TtnGwTime::getTimeCompactGmt(char *buffer)
{
	xSemaphoreTake(_updateTimeMutex, portMAX_DELAY);
	memcpy(buffer, _timeCompact, strlen(_timeCompact));
	xSemaphoreGive(_updateTimeMutex);
}

/**
 * @brief copy last updated time in ISO8601 expanded format in a given buffer
 *
 * @return nothing
 *
 * @exceptsafe This function does not throw exceptions.
 */
void TtnGwTime::getTimeExpandedSystem(char *buffer)
{
	xSemaphoreTake(_updateTimeMutex, portMAX_DELAY);
	memcpy(buffer, _timeExpanded, strlen(_timeExpanded));
	xSemaphoreGive(_updateTimeMutex);
}
