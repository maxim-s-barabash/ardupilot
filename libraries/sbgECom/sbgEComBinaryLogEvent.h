/*!
 * \file			sbgEComBinaryLogEvent.h
 * \ingroup			binaryLogs
 * \author			SBG Systems
 * \date			28 October 2013
 *
 * \brief			Parse event markers logs used to timestamp external signals.
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

#ifndef SBG_ECOM_BINARY_LOG_EVENT_H
#define SBG_ECOM_BINARY_LOG_EVENT_H

// sbgCommonLib headers
#include "sbgCommon.h"
#include "sbgStreamBuffer.h"

#ifdef __cplusplus
extern "C" {
#endif

//----------------------------------------------------------------------//
//- Log marker events definitions                                      -//
//----------------------------------------------------------------------//

/*!
 * Log market events status mask definitions
 */
#define	SBG_ECOM_EVENT_OVERFLOW			(0x00000001u << 0)		/*!< Set to 1 if we have received events at a higher rate than 1 kHz. */
#define	SBG_ECOM_EVENT_OFFSET_0_VALID	(0x00000001u << 1)		/*!< Set to 1 if at least two events have been received. */
#define	SBG_ECOM_EVENT_OFFSET_1_VALID	(0x00000001u << 2)		/*!< Set to 1 if at least three events have been received. */
#define	SBG_ECOM_EVENT_OFFSET_2_VALID	(0x00000001u << 3)		/*!< Set to 1 if at least four events have been received. */
#define	SBG_ECOM_EVENT_OFFSET_3_VALID	(0x00000001u << 4)		/*!< Set to 1 if at least five events have been received. */

//----------------------------------------------------------------------//
//- Log structure definitions                                          -//
//----------------------------------------------------------------------//

/*!
 * Structure that stores data for the SBG_ECOM_LOG_EVENT_# message.
 */
typedef struct _SbgLogEvent
{
	uint32_t	timeStamp;					/*!< Measurement time since the sensor power up. */
	uint16_t	status;						/*!< Events status bitmask. */
	uint16_t	timeOffset0;				/*!< Time offset for the second received event. */
	uint16_t	timeOffset1;				/*!< Time offset for the third received event. */
	uint16_t	timeOffset2;				/*!< Time offset for the fourth received event. */
	uint16_t	timeOffset3;				/*!< Time offset for the fifth received event. */
} SbgLogEvent;

//----------------------------------------------------------------------//
//- Public methods                                                     -//
//----------------------------------------------------------------------//

/*!
 * Parse data for the SBG_ECOM_LOG_EVENT_# message and fill the corresponding structure.
 * 
 * \param[in]	pInputStream				Input stream buffer to read the payload from.
 * \param[out]	pOutputData					Pointer on the output structure that stores parsed data.
 * \return									SBG_NO_ERROR if the payload has been parsed.
 */
SbgErrorCode sbgEComBinaryLogParseEvent(SbgStreamBuffer *pInputStream, SbgLogEvent *pOutputData);

/*!
 * Write data for the SBG_ECOM_LOG_EVENT_# message to the output stream buffer from the provided structure.
 * 
 * \param[out]	pOutputStream				Output stream buffer to write the payload to.
 * \param[in]	pInputData					Pointer on the input structure that stores data to write.
 * \return									SBG_NO_ERROR if the message has been generated in the provided buffer.
 */
SbgErrorCode sbgEComBinaryLogWriteEvent(SbgStreamBuffer *pOutputStream, const SbgLogEvent *pInputData);

#ifdef __cplusplus
}
#endif

#endif // SBG_ECOM_BINARY_LOG_EVENT_H
