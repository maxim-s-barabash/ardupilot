/*!
 * \file			sbgEComBinaryLogDvl.h
 * \ingroup			binaryLogs
 * \author			SBG Systems
 * \date			05 June 2013
 *
 * \brief			Parse received DVL (Doppler Velocity Logger) measurement logs.
 *
 * \copyright		Copyright (C) 2022, SBG Systems SAS. All rights reserved.
 * \beginlicense	The MIT license
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * 
 * \endlicense
 */

#ifndef SBG_ECOM_BINARY_LOG_DVL_H
#define SBG_ECOM_BINARY_LOG_DVL_H

// sbgCommonLib headers
#include "sbgCommon.h"
#include "sbgStreamBuffer.h"

#ifdef __cplusplus
extern "C" {
#endif

//----------------------------------------------------------------------//
//- Log DVL status definitions                                         -//
//----------------------------------------------------------------------//

/*!
 * DVL status mask definitions
 */
#define	SBG_ECOM_DVL_VELOCITY_VALID		(0x0001u << 0)			/*!< Set to 1 if the DVL equipment was able to measure a valid velocity. */
#define SBG_ECOM_DVL_TIME_SYNC			(0x0001u << 1)			/*!< Set to 1 if the DVL data is correctly synchronized. */

//----------------------------------------------------------------------//
//- Log structure definitions                                          -//
//----------------------------------------------------------------------//

/*!
 * Log structure for DVL data.
 */
typedef struct _SbgLogDvlData
{
	uint32_t	timeStamp;				/*!< Time in us since the sensor power up. */
	uint16_t	status;					/*!< DVL status bitmask. */
	float		velocity[3];			/*!< X, Y, Z velocities in m.s^-1 expressed in the DVL instrument frame. */
	float		velocityQuality[3];		/*!< X, Y, Z velocities quality indicators as provided by the DVL sensor and expressed in m.s^-1.
											WARNING: This is typically just a residual information and not a real standard deviation. */
} SbgLogDvlData;

//----------------------------------------------------------------------//
//- Public methods                                                     -//
//----------------------------------------------------------------------//

/*!
 * Parse data for the SBG_ECOM_LOG_DVL_BOTTOM_TRACK / SBG_ECOM_LOG_DVL_WATER_TRACK message and fill the corresponding structure.
 * 
 * \param[in]	pInputStream				Input stream buffer to read the payload from.
 * \param[out]	pOutputData					Pointer on the output structure that stores parsed data.
 * \return									SBG_NO_ERROR if the payload has been parsed.
 */
SbgErrorCode sbgEComBinaryLogParseDvlData(SbgStreamBuffer *pInputStream, SbgLogDvlData *pOutputData);

/*!
 * Write data for the SBG_ECOM_LOG_DVL_BOTTOM_TRACK / SBG_ECOM_LOG_DVL_WATER_TRACK message to the output stream buffer from the provided structure.
 * 
 * \param[out]	pOutputStream				Output stream buffer to write the payload to.
 * \param[in]	pInputData					Pointer on the input structure that stores data to write.
 * \return									SBG_NO_ERROR if the message has been generated in the provided buffer.
 */
SbgErrorCode sbgEComBinaryLogWriteDvlData(SbgStreamBuffer *pOutputStream, const SbgLogDvlData *pInputData);

#ifdef __cplusplus
}
#endif

#endif // SBG_ECOM_BINARY_LOG_DVL_H
