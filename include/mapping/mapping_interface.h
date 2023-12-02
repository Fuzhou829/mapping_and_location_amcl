/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.0
 * @Date: 2022-08-05 17:09:35
 * @LastEditTime: 2023-08-21 16:50:41
 * @Author: lcfc-desktop
 */
#pragma once

#include <memory>
#include <string>
#include <list>

#include "message_lib/odometer_message.h"
#include "message_lib/radar_message.h"
#include "message_lib/position_message.h"
#include "message_lib/dmreader_message.h"
#include "message_lib/imu_message.h"
#include "include/config_struct.h"
#include "landmark_tool/landmark_center_calcuator.h"

#include "record_data/record_data.h"

namespace gomros {
namespace data_process {
namespace mapping_and_location {

class MappingInterface {
 public:
  using OdometerMessage = gomros::message::OdometerMessage;
  using RadarSensoryMessage = gomros::message::RadarSensoryMessage;
  using DMreaderSensoryMessage = gomros::message::DMreaderSensoryMessage;
  using ImuSensoryMessage = gomros::message::ImuSensoryMessage;
  using Position = gomros::message::Position;

 public:
  explicit MappingInterface(const RecordConfig& config);
  virtual ~MappingInterface() {}

  // 设置配置参数
  void SetMappingConfig(const MappingConfig& config);
  // 处理里程计数据
  virtual void HandleOdomData(const OdometerMessage &data);
  // 处理雷达数据 -- 雷达是否在移动
  virtual void HandleLaserData(const RadarSensoryMessage &data, bool is_move);
  // 处理二维码数据
  virtual void HandleQRCodeData(const DMreaderSensoryMessage &data);
  // 处理IMU数据
  virtual void HandleIMUData(const ImuSensoryMessage &data);
  // 对QR进行标定标志位
  virtual void SetQrCalibrationFlag(bool is_qr_calibration);
  // 开始建图
  virtual void StartMapping() = 0;

  /**
   * @brief: 停止建图
   * @param {string&} map_name 地图保存名字
   */  
  virtual void StopMapping(const std::string& map_name) = 0;

 protected:
  bool is_qr_calibration_;   // false 正常建图 true 标定qr
  MappingConfig mapping_config_;
  std::shared_ptr<Record::SensorData> sensor_data_recorder_;
};

}  // namespace mapping_and_location
}  // namespace data_process
}  // namespace gomros
