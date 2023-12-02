/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.0
 * @Date: 2022-08-05 17:09:35
 * @LastEditTime: 2023-06-29 11:16:35
 * @Author: renjy
 */
#pragma once

#include <string>
#include <memory>
#include <vector>

#include "message_lib/odometer_message.h"
#include "message_lib/radar_message.h"
#include "mapping/mapping_interface.h"
#include "landmark_tool/landmark_center_calcuator.h"
#include "include/config_struct.h"
#include "common/tool.h"
#include "reflector_mapping/reflector_map_builder.h"

namespace gomros {
namespace data_process {
namespace mapping_and_location {


class ReflectorMapping : public MappingInterface {
 public:
  using OdometerMessage = gomros::message::OdometerMessage;
  using RadarSensoryMessage = gomros::message::RadarSensoryMessage;
  using RadarSensoryInfo = gomros::message::RadarSensoryInfo;
  using Position = gomros::message::Position;

 public:
  explicit ReflectorMapping(const MappingConfig& config);
  virtual ~ReflectorMapping() {}

  /**
   * @brief: 开始建图
   * @return {*}
   */  
  void StartMapping() override;
  /**
   * @brief: 结束建图
   * @param {string} map_name 地图输出名字
   * @return {*}
   */  
  void StopMapping(const std::string& map_name) override;

 private:
  void _startReflectorMapping(const std::string& map_name);
  void _openOdomAndImuFile();
  void _closeOdomAndImuFile();
  void _transform2RadarSensoryInfo(const std::vector<std::string>& ladar_str,
                                    RadarSensoryInfo* ladar_message);
  void _dealOdomAndImuMessage(uint64_t time_stemp);

 private:
  std::shared_ptr<ReflectorMapBuilder> reflector_map_builder_;
  std::ifstream odom_file_;
  std::ifstream imu_file_;
  OdometerMessage last_odom_massage_;
  bool is_record_last_odom_;
};

}  // namespace mapping_and_location
}  // namespace data_process
}  // namespace gomros
