// sbgCommonLib headers
#include "sbgCommon.h"

//----------------------------------------------------------------------//
//- Include specific header for WIN32 and UNIX platforms               -//
//----------------------------------------------------------------------//
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

//----------------------------------------------------------------------//
//- Global singleton for the log callback							   -//
//----------------------------------------------------------------------//

/*!
 * Unique singleton used to log error messages.
 */
SbgCommonLibOnLogFunc	gLogCallback = NULL;
const AP_HAL::HAL& hal_instance = AP_HAL::get_HAL();
//----------------------------------------------------------------------//
//- Public functions                                                   -//
//----------------------------------------------------------------------//

SBG_COMMON_LIB_API uint32_t sbgGetTime(void)
{	
	GCS_SEND_TEXT(MAV_SEVERITY_INFO, "sbgGetTime");			
    return AP_HAL::millis();
}

SBG_COMMON_LIB_API void sbgSleep(uint32_t ms)
{
	GCS_SEND_TEXT(MAV_SEVERITY_INFO, "sbgSleep: %u Start", (uint16_t)ms);			
	hal_instance.scheduler->delay(ms);
	GCS_SEND_TEXT(MAV_SEVERITY_INFO, "sbgSleep: %u Stop", (uint16_t)ms);			
}

SBG_COMMON_LIB_API void sbgCommonLibSetLogCallback(SbgCommonLibOnLogFunc logCallback)
{
	//
	// TODO: should we implement lock / sync mechanisms ?
	//
	gLogCallback = logCallback;
}

SBG_COMMON_LIB_API void sbgPlatformDebugLogMsg(const char *pFileName, const char *pFunctionName, uint32_t line, const char *pCategory, SbgDebugLogType logType, SbgErrorCode errorCode, const char *pFormat, ...)
{
	return;
	char errorMsg[SBG_CONFIG_LOG_MAX_SIZE];
	va_list	args;

	if (pFileName && pFunctionName && pCategory && pFormat){

	//
	// Initialize the list of variable arguments on the latest function argument
	//
	va_start(args, pFormat);

	//
	// Generate the error message string
	//
	vsnprintf(errorMsg, sizeof(errorMsg), pFormat, args);

	//
	// Close the list of variable arguments
	//
	va_end(args);

	//
	// Check if there is a valid logger callback if not use a default output
	//
	if (gLogCallback) {
		gLogCallback(pFileName, pFunctionName, line, pCategory, logType, errorCode, errorMsg);
	}
	else
	{
		//
		// Log the correct message according to the log type
		//
		switch (logType)
		{
		case SBG_DEBUG_LOG_TYPE_ERROR:
			GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "*ERROR* %s(%u): %s\n\r", pFunctionName, (uint16_t)line, errorMsg);
			break;
		case SBG_DEBUG_LOG_TYPE_WARNING:
			GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "*WARN* %s(%u): %s\n\r", pFunctionName, (uint16_t)line, errorMsg);
			break;
		case SBG_DEBUG_LOG_TYPE_INFO:
			GCS_SEND_TEXT(MAV_SEVERITY_INFO, "*INFO* %s(%u): %s\n\r", pFunctionName, (uint16_t)line, errorMsg);
			break;
		case SBG_DEBUG_LOG_TYPE_DEBUG:
			GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "*DEBUG* %s(%u): %s\n\r", pFunctionName, (uint16_t)line, errorMsg);
			break;
		default:
			GCS_SEND_TEXT(MAV_SEVERITY_INFO, "*INFO* %s(%u): %s\n\r", pFunctionName, (uint16_t)line, errorMsg);			
			break;
		}
	}
}
}
