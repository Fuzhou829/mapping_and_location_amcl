/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.2
 * @Author: renjy
 * @Date: 2022-10-28 14:32:05
 * @LastEditTime: 2023-10-18 15:38:37
 */

#pragma once

#include <memory>
#include <vector>
#include <string>

#include "common/thread_pool.h"
#include "common/tool.h"
#include "include/config_struct.h"
#include "reflector_mapping/submap.h"
#include "reflector_mapping/optimizer.h"
#include "reflector_mapping/reflector_map_builder.h"
#include "landmark_tool/landmark_center_calcuator.h"
#include "landmark_tool/landmark_dbscan_calcuator.h"


#include "message_lib/odometer_message.h"
#include "message_lib/imu_message.h"
#include "message_lib/radar_message.h"

#include "display_result/display_result.h"

namespace gomros {
namespace data_process {
namespace mapping_and_location {


class ReflectorMapBuilder {
 public:
  using OdometerMessage = gomros::message::OdometerMessage;
  using ImuSensoryMessage = gomros::message::ImuSensoryMessage;
  using RadarSensoryInfo = gomros::message::RadarSensoryInfo;
  using Position = gomros::message::Position;

 public:
  ReflectorMapBuilder(const MappingConfig& config, int num_background_threads);
  virtual ~ReflectorMapBuilder() {
    if (optimizer_ != nullptr) delete optimizer_;
  }
  void Init();
  void DispatchOdomData(const OdometerMessage& odom_message);
  void DispatchLadarData(const RadarSensoryInfo& ladar_message);
  void DispatchImuData(const ImuSensoryMessage& imu_message);
  void FinishTrajectory();
  void SaveToFile(const std::string& map_name);

 private:
  std::shared_ptr<Submap> _createdNewSubMap(
    std::shared_ptr<Submap> last_submap);


 private:
  MappingConfig config_;

  std::vector<std::shared_ptr<Submap>> active_submaps_;
  std::vector<std::shared_ptr<Submap>> finished_submaps_;
  internal_common::ThreadPool thread_pool_;
  Optimizer* optimizer_;

  // for test
  std::shared_ptr<DisPlayResult::SimulationDataPublisher>
    display_reflector_ekf_;
};





}  // namespace mapping_and_location
}  // namespace data_process
}  // namespace gomros

