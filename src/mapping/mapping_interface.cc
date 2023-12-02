/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.0
 * @Author: renjy
 * @Date: 2022-10-28 20:34:44
 * @LastEditTime: 2023-08-21 16:50:03
 */

#include <memory>

#include "mapping/mapping_interface.h"
#include "common/logger.h"

namespace gomros {
namespace data_process {
namespace mapping_and_location {

MappingInterface::MappingInterface(const RecordConfig& config) {
  sensor_data_recorder_ = std::make_shared<Record::SensorData>(config);
  is_qr_calibration_ = false;
}

void MappingInterface::SetMappingConfig(const MappingConfig& config) {
  mapping_config_ = config;
  RecordConfig record_config = config.record_config;
  record_config.is_need_record_data = false;
  sensor_data_recorder_->SetRecordConfig(record_config);
}

void MappingInterface::HandleOdomData(const OdometerMessage &data) {
  sensor_data_recorder_->RecordOdomData(data);
}

void MappingInterface::HandleLaserData(
  const RadarSensoryMessage &data, bool is_move) {
  sensor_data_recorder_->RecordLadarData(data, is_move);
}

void MappingInterface::HandleQRCodeData(const DMreaderSensoryMessage &data) {
  // 插入二维码数据信息
  sensor_data_recorder_->RecordQrData(data);
}


void MappingInterface::HandleIMUData(const ImuSensoryMessage &data) {
  // 插入imu信息
  sensor_data_recorder_->RecordImuData(data);
}

void MappingInterface::SetQrCalibrationFlag(bool is_qr_calibration) {
  is_qr_calibration_ = is_qr_calibration;
}

}  // namespace mapping_and_location
}  // namespace data_process
}  // namespace gomros
