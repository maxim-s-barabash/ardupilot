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
  suppport for SBG serially connected AHRS Systems
 */

#define ALLOW_DOUBLE_MATH_FUNCTIONS

#include "AP_ExternalAHRS_SBG.h"
#if HAL_EXTERNAL_AHRS_SBG_ENABLED
#include <AP_Baro/AP_Baro.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL &hal;

AP_ExternalAHRS_SBG::AP_ExternalAHRS_SBG(AP_ExternalAHRS *_frontend,
        AP_ExternalAHRS::state_t &_state): AP_ExternalAHRS_backend(_frontend, _state)
{
    auto &sm = AP::serialmanager();
    uart = sm.find_serial(AP_SerialManager::SerialProtocol_AHRS, 0);

    baudrate = sm.find_baudrate(AP_SerialManager::SerialProtocol_AHRS, 0);
    port_num = sm.find_portnum(AP_SerialManager::SerialProtocol_AHRS, 0);

    if (!uart) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "ExternalAHRS no UART");
        return;
    } else {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ExternalAHRS UART found");
    }
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_ExternalAHRS_SBG::update_thread, void), "AHRS", 16384, AP_HAL::Scheduler::PRIORITY_SPI, 0)) {
        AP_HAL::panic("Failed to start ExternalAHRS update thread");
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SBG ExternalAHRS initialised");
}

void AP_ExternalAHRS_SBG::update_thread(void)
{
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "update_thread input");

    _sbg_interface.handle = this;
    _sbg_interface.type    = SBG_IF_TYPE_SERIAL;
    _sbg_interface.pReadFunc = readCallback;

    if (!port_open) {
        port_open = true;
        uart->begin(baudrate);
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "UART opened");
    hal.scheduler->delay(2000);
    SbgErrorCode error_code = sbgEComInit(&_com_handle, &_sbg_interface);

    if (error_code == SBG_NO_ERROR) {
        // Attach the callback that handle received log
        sbgEComSetReceiveLogCallback(&_com_handle, onLogReceivedCallback, this);
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "sbgECom initialized succesfully");
    } else {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Couldn't initialize sbgECom");
    }

    while(1){
        hal.scheduler->delay(1);
        receive();
    }
}

int8_t AP_ExternalAHRS_SBG::get_port(void) const
{
    if (!uart) {
        return -1;
    }
    return port_num;
}

// Get model/type name
const char* AP_ExternalAHRS_SBG::get_name() const
{
    return "SBG";
}

bool AP_ExternalAHRS_SBG::healthy(void) const
{
    uint32_t now = AP_HAL::millis();
    return (now - last_ins_pkt < 40 && now - last_gps_pkt < 500 && now - last_filter_pkt < 500);
}

bool AP_ExternalAHRS_SBG::initialised(void) const
{
    return last_ins_pkt != 0 && last_gps_pkt != 0 && last_filter_pkt != 0;
}

bool AP_ExternalAHRS_SBG::pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const
{
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "pre arm check");
    if (!healthy()) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s, %u, %s", failure_msg, failure_msg_len, "SBG unhealthy");
        return false;
    }
    if (gps_data.fix_type < 3) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s, %u, %s", failure_msg, failure_msg_len, "SBG no GPS lock");
        return false;
    }
    if (filter_status.state != 0x02) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s, %u, %s", failure_msg, failure_msg_len, "SBG filter not running");
        return false;
    }

    return true;
}

void AP_ExternalAHRS_SBG::get_filter_status(nav_filter_status &status) const
{
    //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "get_filter_status");
 
    memset(&status, 0, sizeof(status));
    if (last_ins_pkt != 0 && last_gps_pkt != 0) {
        status.flags.initalized = 1;
    }
    if (healthy() && last_ins_pkt != 0) {
        status.flags.attitude = 1;
        status.flags.vert_vel = 1;
        status.flags.vert_pos = 1;

        if (gps_data.fix_type >= 3) {
            status.flags.horiz_vel = 1;
            status.flags.horiz_pos_rel = 1;
            status.flags.horiz_pos_abs = 1;
            status.flags.pred_horiz_pos_rel = 1;
            status.flags.pred_horiz_pos_abs = 1;
            status.flags.using_gps = 1;
        }
    }
}

void AP_ExternalAHRS_SBG::send_status_report(GCS_MAVLINK &link) const
{
    // prepare flags
    uint16_t flags = 0;
    nav_filter_status filterStatus;
    get_filter_status(filterStatus);
    if (filterStatus.flags.attitude) {
        flags |= EKF_ATTITUDE;
    }
    if (filterStatus.flags.horiz_vel) {
        flags |= EKF_VELOCITY_HORIZ;
    }
    if (filterStatus.flags.vert_vel) {
        flags |= EKF_VELOCITY_VERT;
    }
    if (filterStatus.flags.horiz_pos_rel) {
        flags |= EKF_POS_HORIZ_REL;
    }
    if (filterStatus.flags.horiz_pos_abs) {
        flags |= EKF_POS_HORIZ_ABS;
    }
    if (filterStatus.flags.vert_pos) {
        flags |= EKF_POS_VERT_ABS;
    }
    if (filterStatus.flags.terrain_alt) {
        flags |= EKF_POS_VERT_AGL;
    }
    if (filterStatus.flags.const_pos_mode) {
        flags |= EKF_CONST_POS_MODE;
    }
    if (filterStatus.flags.pred_horiz_pos_rel) {
        flags |= EKF_PRED_POS_HORIZ_REL;
    }
    if (filterStatus.flags.pred_horiz_pos_abs) {
        flags |= EKF_PRED_POS_HORIZ_ABS;
    }
    if (!filterStatus.flags.initalized) {
        flags |= EKF_UNINITIALIZED;
    }

    // send message
    const float vel_gate = 5; // represents hz value data is posted at
    const float pos_gate = 5; // represents hz value data is posted at
    const float hgt_gate = 5; // represents hz value data is posted at
    const float mag_var = 0; //we may need to change this to be like the other gates, set to 0 because mag is ignored by the ins filter in vectornav
    mavlink_msg_ekf_status_report_send(link.get_chan(), flags,
                                       gps_data.speed_accuracy/vel_gate, gps_data.horizontal_position_accuracy/pos_gate, gps_data.vertical_position_accuracy/hgt_gate,
                                       mag_var, 0, 0);
}

Vector3f AP_ExternalAHRS_SBG::populate_vector3f(const float *data) const
{
    return Vector3f{data[0], data[1], data[2]};
}

Vector3f AP_ExternalAHRS_SBG::populate_vector3f(const float *data, float (*pred)(float)) const
{
    return Vector3f{pred(data[0]), pred(data[1]), pred(data[2])};
}

Quaternion AP_ExternalAHRS_SBG::populate_quaternion(const float *data) const
{
    return Quaternion{data[0], data[1], data[2], data[3]};
}

void AP_ExternalAHRS_SBG::receive()
{
    // Handle incoming log
    sbgEComHandleOneLog(&_com_handle);
    counter++;
}

SbgErrorCode AP_ExternalAHRS_SBG::readCallback(SbgInterface *p_interface, void *p_buffer, size_t *p_read_bytes, size_t bytes_to_read)
{
    AP_ExternalAHRS_SBG *p_sbg_driver = (AP_ExternalAHRS_SBG *)p_interface->handle;
    return p_sbg_driver->read(p_interface, p_buffer, p_read_bytes, bytes_to_read);
}

SbgErrorCode AP_ExternalAHRS_SBG::read(SbgInterface *p_interface, void *p_buffer, size_t *p_read_bytes, size_t bytes_to_read){
    SbgErrorCode error_code = SBG_NOT_READY;
    int result = uart->read((uint8_t *)p_buffer, bytes_to_read);
    if (result >= 0) {
        *p_read_bytes = result;
        error_code = SBG_NO_ERROR;
    }
    return error_code;
}

GPS_FIX_TYPE AP_ExternalAHRS_SBG::fix_convertion(const SbgEComGpsPosType fixType){
    GPS_FIX_TYPE result = GPS_FIX_TYPE_ENUM_END;
    switch (fixType){
        case SBG_ECOM_POS_NO_SOLUTION : result = GPS_FIX_TYPE_NO_GPS; break;
        case SBG_ECOM_POS_RTK_FLOAT: result = GPS_FIX_TYPE_RTK_FLOAT; break;
        case SBG_ECOM_POS_RTK_INT: result = GPS_FIX_TYPE_RTK_FIXED; break;
        case SBG_ECOM_POS_PPP_FLOAT: result = GPS_FIX_TYPE_PPP; break;
        case SBG_ECOM_POS_PSRDIFF:
        case SBG_ECOM_POS_SBAS:
        case SBG_ECOM_POS_OMNISTAR:
        case SBG_ECOM_POS_PPP_INT:
        case SBG_ECOM_POS_SINGLE:
        case SBG_ECOM_POS_FIXED:
        case SBG_ECOM_POS_UNKNOWN_TYPE:
        default: result = GPS_FIX_TYPE_3D_FIX; break;
    }
    return result;
}

void AP_ExternalAHRS_SBG::onLogReceived(SbgEComClass msg_class, SbgEComMsgId msg, const SbgBinaryLogData &ref_sbg_data, uint64_t system_timestamp)
{
    last_filter_pkt = system_timestamp;

    if (msg_class == SBG_ECOM_CLASS_LOG_ECOM_0)
    {
        switch (msg)
        {
        case SBG_ECOM_LOG_UTC_TIME:
            processUtc(&ref_sbg_data.utcData);
            break;
        case SBG_ECOM_LOG_GPS1_VEL:
        case SBG_ECOM_LOG_GPS2_VEL:
            processGpsVel(&ref_sbg_data.gpsVelData);
            break;
        case SBG_ECOM_LOG_GPS1_POS:
        case SBG_ECOM_LOG_GPS2_POS:
            processGpsPos(&ref_sbg_data.gpsPosData, system_timestamp);
            break;
        case SBG_ECOM_LOG_GPS1_HDT:
        case SBG_ECOM_LOG_GPS2_HDT:
            processGpsHdt(&ref_sbg_data.gpsHdtData);
            break;
        case SBG_ECOM_LOG_GPS1_SAT:
        case SBG_ECOM_LOG_GPS2_SAT:
            processSatGroupData(&ref_sbg_data.satGroupData);
        break;
        case SBG_ECOM_LOG_EKF_EULER:
            processEkfEulerData(&ref_sbg_data.ekfEulerData);
        break;
        case SBG_ECOM_LOG_EKF_QUAT:
            processEkfQuatData(&ref_sbg_data.ekfQuatData);
        break;
        case SBG_ECOM_LOG_IMU_DATA:
            processImuData(&ref_sbg_data.imuData);
        break;
        case SBG_ECOM_LOG_MAG:
            processMag(&ref_sbg_data.magData);
        break;
        case SBG_ECOM_LOG_SHIP_MOTION:
            processShipMotionData(&ref_sbg_data.shipMotionData);
        break;
        case SBG_ECOM_LOG_EKF_NAV:
            processEkfNavData(&ref_sbg_data.ekfNavData);
        break;
        case SBG_ECOM_LOG_IMU_SHORT:
            processImuShort(&ref_sbg_data.imuShort);
        break;
        default:
            break;
        }
    }
}

SbgErrorCode AP_ExternalAHRS_SBG::onLogReceivedCallback(SbgEComHandle *p_handle, SbgEComClass msg_class, SbgEComMsgId msg, const SbgBinaryLogData *p_log_data, void* p_user_arg)
{
    if (p_user_arg){
        SBG_UNUSED_PARAMETER(p_handle);
        uint64_t system_timestamp = AP_HAL::millis();
        AP_ExternalAHRS_SBG *p_sbg_driver = (AP_ExternalAHRS_SBG *)p_user_arg;
        p_sbg_driver->onLogReceived(msg_class, msg, *p_log_data, system_timestamp);
    }
    return SBG_NO_ERROR;
}

void AP_ExternalAHRS_SBG::processGpsVel(const SbgLogGpsVel *p_gps_vel)
{
    gps_data.ned_velocity_north = p_gps_vel->velocity[0];
    gps_data.ned_velocity_east = p_gps_vel->velocity[1];
    gps_data.ned_velocity_down = p_gps_vel->velocity[2];
    //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Velocity:[ N:%f, E:%f, D:%f]", gps_data.ned_velocity_north, gps_data.ned_velocity_east, gps_data.ned_velocity_down);
    
    gps_data.speed_accuracy = sqrtf(powf(p_gps_vel->velocityAcc[0], 2) + powf(p_gps_vel->velocityAcc[1], 2));
    gps_data.vdop = gps_data.speed_accuracy;
}

void AP_ExternalAHRS_SBG::processStatusData(const SbgLogStatusData *statusData) {

}

void AP_ExternalAHRS_SBG::processImuData(const SbgLogImuData *imuData) {
    imu_data.accel = populate_vector3f(imuData->accelerometers);
    imu_data.gyro = populate_vector3f(imuData->gyroscopes);
    imu_data.temperature = imuData->temperature;
    // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Accel: [%f,%f,%f]", imu_data.accel[0], imu_data.accel[1], imu_data.accel[2]);
    // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Gyro: [%f,%f,%f]", imu_data.gyro[0], imu_data.gyro[1], imu_data.gyro[2]);
    last_ins_pkt = AP_HAL::millis();
}
void AP_ExternalAHRS_SBG::processImuShort(const SbgLogImuShort *imuShort) {

}

void AP_ExternalAHRS_SBG::processEkfEulerData(const SbgLogEkfEulerData *ekfEulerData) {

}

void AP_ExternalAHRS_SBG::processEkfQuatData(const SbgLogEkfQuatData *ekfQuatData) {
    imu_data.quat = populate_quaternion(ekfQuatData->quaternion);
//    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Quat: [%f, %f, %f, %f]", imu_data.quat[0], imu_data.quat[1], imu_data.quat[2], imu_data.quat[3]);
}

void AP_ExternalAHRS_SBG::processEkfNavData(const SbgLogEkfNavData *ekfNavData) {

}

void AP_ExternalAHRS_SBG::processShipMotionData(const SbgLogShipMotionData *shipMotionData) {

}

void AP_ExternalAHRS_SBG::processOdometerData(const SbgLogOdometerData *odometerData) {

}

void AP_ExternalAHRS_SBG::processMag(const SbgLogMag *magData) {
    imu_data.mag = populate_vector3f((const float*)magData->magnetometers);
    //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Mag: [%f, %f, %f]", imu_data.mag[0], imu_data.mag[1], imu_data.mag[2]);
}

void AP_ExternalAHRS_SBG::processMagCalib(const SbgLogMagCalib *magCalibData) {

}

void AP_ExternalAHRS_SBG::processDvlData(const SbgLogDvlData *dvlData) {

}

void AP_ExternalAHRS_SBG::processAirData(const SbgLogAirData *airData) {

}

void AP_ExternalAHRS_SBG::processUsblData(const SbgLogUsblData *usblData) {

}

void AP_ExternalAHRS_SBG::processDepth(const SbgLogDepth *depthData) {

}

void AP_ExternalAHRS_SBG::processEvent(const SbgLogEvent *eventMarker) {

}

void AP_ExternalAHRS_SBG::processDiagData(const SbgLogDiagData *diagData) {

}

void AP_ExternalAHRS_SBG::processSatGroupData(const SbgLogSatGroupData *satGroupData) {
//    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SbgLogSatGroupData");
}

void AP_ExternalAHRS_SBG::processGpsPos(const SbgLogGpsPos *p_pos, uint64_t system_timestamp)
{
//    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Pos: [%f, %f, %f]", p_pos->latitude, p_pos->longitude, p_pos->altitude);
    gps_data.lon = (int32_t)(p_pos->longitude * 1.0E7);
    gps_data.lat = (int32_t)(p_pos->latitude * 1.0E7);
    gps_data.hae_altitude = (int32_t)(p_pos->altitude * 1.0E2);
    gps_data.msl_altitude = (int32_t)(p_pos->altitude * 1.0E2);
    gps_data.horizontal_position_accuracy = sqrtf(powf(p_pos->longitudeAccuracy, 2) + powf(p_pos->latitudeAccuracy, 2));
    gps_data.vertical_position_accuracy = p_pos->altitudeAccuracy;
    gps_data.satellites = p_pos->numSvUsed;
    gps_data.fix_type = fix_convertion(sbgEComLogGpsPosGetType(p_pos->status));
    last_gps_pkt = system_timestamp;
}

void AP_ExternalAHRS_SBG::post_data(){
    AP_ExternalAHRS::gps_data_message_t gps {
        gps_week: gps_data.week,
        ms_tow: gps_data.tow_ms,
        fix_type: (uint8_t) gps_data.fix_type,
        satellites_in_view: gps_data.satellites,

        horizontal_pos_accuracy: gps_data.horizontal_position_accuracy,
        vertical_pos_accuracy: gps_data.vertical_position_accuracy,
        horizontal_vel_accuracy: gps_data.speed_accuracy,

        hdop: gps_data.hdop,
        vdop: gps_data.vdop,

        longitude: gps_data.lon,
        latitude: gps_data.lat,
        msl_altitude: gps_data.msl_altitude,

        ned_vel_north: gps_data.ned_velocity_north,
        ned_vel_east: gps_data.ned_velocity_east,
        ned_vel_down: gps_data.ned_velocity_down,
    };
    {
        WITH_SEMAPHORE(state.sem);
        state.velocity = Vector3f{gps_data.ned_velocity_north, gps_data.ned_velocity_east, gps_data.ned_velocity_down};
        state.have_velocity = true;

        state.location = Location{gps_data.lat, gps_data.lon, gps_data.msl_altitude, Location::AltFrame::ABSOLUTE};
        state.have_location = true;
    }

    {
        WITH_SEMAPHORE(state.sem);
        state.accel = imu_data.accel;
        state.gyro = imu_data.gyro;
        state.quat = imu_data.quat;
        state.have_quaternion = true;
    }

    {
        AP_ExternalAHRS::ins_data_message_t ins {
            accel: imu_data.accel,
            gyro: imu_data.gyro,

            temperature: imu_data.temperature
        };
        AP::ins().handle_external(ins);
    }
    {
        AP_ExternalAHRS::mag_data_message_t mag;
        mag.field = imu_data.mag;
        AP::compass().handle_external(mag);
    }

    if (gps.fix_type >= 3 && !state.have_origin) {
        WITH_SEMAPHORE(state.sem);
        state.origin = Location{int32_t(gps_data.lat),
                                int32_t(gps_data.lon),
                                int32_t(gps_data.msl_altitude),
                                Location::AltFrame::ABSOLUTE};
        state.have_origin = true;
    }

    AP::gps().handle_external(gps);
}

void AP_ExternalAHRS_SBG::processGpsHdt(const SbgLogGpsHdt *p_gps_hdt)
{
    gps_data.hdop = p_gps_hdt->headingAccuracy;
}

void AP_ExternalAHRS_SBG::processUtc(const SbgLogUtcData *p_utc)
{
    bool clock_stable = ((p_utc->status & SBG_ECOM_CLOCK_STABLE_INPUT) != 0);
    bool utc_sync = ((p_utc->status & SBG_ECOM_CLOCK_UTC_SYNC) != 0);
    uint8_t clock_status = sbgEComLogUtcGetClockStatus(p_utc->status);

    if (clock_stable && utc_sync && clock_status == SBG_ECOM_CLOCK_VALID)
    {
        time_t epoch;
        tm timeinfo{};

        timeinfo.tm_year = p_utc->year - 1900;
        timeinfo.tm_mon = p_utc->month - 1;
        timeinfo.tm_mday = p_utc->day;
        timeinfo.tm_hour = p_utc->hour;
        timeinfo.tm_min = p_utc->minute;
        timeinfo.tm_sec = p_utc->second;

        epoch = mktime(&timeinfo);

        gps_data.week = epoch / (AP_MSEC_PER_WEEK * 1000000ULL);
        gps_data.tow_ms = p_utc->gpsTimeOfWeek;
    }
}


#endif // HAL_EXTERNAL_AHRS_ENABLED

