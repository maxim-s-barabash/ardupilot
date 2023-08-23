/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  suppport for serial connected AHRS systems
 */

#pragma once

#include "AP_ExternalAHRS_backend.h"

#ifndef HAL_EXTERNAL_AHRS_SBG_ENABLED
#define HAL_EXTERNAL_AHRS_SBG_ENABLED HAL_EXTERNAL_AHRS_ENABLED
#endif

#if HAL_EXTERNAL_AHRS_SBG_ENABLED

#include <GCS_MAVLink/GCS_MAVLink.h>
#include <sbgECom/sbgEComLib.h>
#include <sbgECom/sbgInterface.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>

class AP_ExternalAHRS_SBG: public AP_ExternalAHRS_backend
{
public:

    AP_ExternalAHRS_SBG(AP_ExternalAHRS *frontend, AP_ExternalAHRS::state_t &state);

    // get serial port number, -1 for not enabled
    int8_t get_port(void) const override;

    // Get model/type name
    const char* get_name() const override;

    // accessors for AP_AHRS
    bool healthy(void) const override;
    bool initialised(void) const override;
    bool pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const override;
    void get_filter_status(nav_filter_status &status) const override;
    void send_status_report(class GCS_MAVLINK &link) const override;

    // check for new data
    void update() override {
        post_data();
    };

private:
    void update_thread();

    AP_HAL::UARTDriver *uart;
    HAL_Semaphore sem;

    uint32_t baudrate;
    int8_t port_num;
    bool port_open = false;

    uint32_t last_ins_pkt;
    uint32_t last_gps_pkt;
    uint32_t last_filter_pkt;

    struct {
        Vector3f accel;
        Vector3f gyro;
        Vector3f mag;
        Quaternion quat;
        float pressure;
        float temperature;
    } imu_data;

    struct {
        uint16_t state;
        uint16_t mode;
        uint16_t flags;
    } filter_status;

    struct {
        GPS_FIX_TYPE fix_type;
        uint8_t satellites;
        float hdop;
        float vdop;
        uint16_t week;
        uint32_t tow_ms;
        float horizontal_position_accuracy;
        float vertical_position_accuracy;
        int32_t lon;
        int32_t lat;
        int32_t hae_altitude;
        int32_t msl_altitude;
        float ned_velocity_north;
        float ned_velocity_east;
        float ned_velocity_down;
        float speed_accuracy;
    } gps_data;

    Vector3f populate_vector3f(const float* data) const;
    Vector3f populate_vector3f(const float *data, float (*pred)(float)) const;
    Quaternion populate_quaternion(const float* data) const;

	void receive();
    void post_data();
    GPS_FIX_TYPE fix_convertion(const SbgEComGpsPosType fixType);

	static SbgErrorCode readCallback(SbgInterface *p_interface, void *p_buffer, size_t *p_read_bytes, size_t bytes_to_read);
	SbgErrorCode read(SbgInterface *p_interface, void *p_buffer, size_t *p_read_bytes, size_t bytes_to_read);

	static SbgErrorCode onLogReceivedCallback(SbgEComHandle *p_handle, SbgEComClass msg_class, SbgEComMsgId msg, const SbgBinaryLogData *p_log_data, void *p_user_arg);
	void onLogReceived(SbgEComClass msg_class, SbgEComMsgId msg, const SbgBinaryLogData &ref_sbg_data, uint64_t system_timestamp);
	void processGpsVel(const SbgLogGpsVel *p_gps_vel);
	void processGpsPos(const SbgLogGpsPos *p_gps_pos, uint64_t system_timestamp);
	void processGpsHdt(const SbgLogGpsHdt *p_gps_hdt);
	void processUtc(const SbgLogUtcData *p_utc);
    void processStatusData(const SbgLogStatusData *statusData);
    void processImuData(const SbgLogImuData *imuData);
    void processImuShort(const SbgLogImuShort *imuShort);
    void processEkfEulerData(const SbgLogEkfEulerData *ekfEulerData);
    void processEkfQuatData(const SbgLogEkfQuatData *ekfQuatData);
    void processEkfNavData(const SbgLogEkfNavData *ekfNavData);
    void processShipMotionData(const SbgLogShipMotionData *shipMotionData);
    void processOdometerData(const SbgLogOdometerData *odometerData);
    void processMag(const SbgLogMag *magData);
    void processMagCalib(const SbgLogMagCalib *magCalibData);
    void processDvlData(const SbgLogDvlData *dvlData);
    void processAirData(const SbgLogAirData *airData);
    void processUsblData(const SbgLogUsblData *usblData);
    void processDepth(const SbgLogDepth *depthData);
    void processEvent(const SbgLogEvent *eventMarker);
    void processDiagData(const SbgLogDiagData *diagData);
    void processSatGroupData(const SbgLogSatGroupData *satGroupData);


	SbgInterface _sbg_interface;
    SbgEComHandle _com_handle;

	float _heading_offset;
	uint64_t _utc_timestamp;

    uint16_t counter = 0;
};

#endif // HAL_EXTERNAL_AHRS_ENABLED

