/*
 * @Descripttion: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @version: 1.0
 * @Author: renjy
 * @Date: 2022-12-14 10:42:13
 * @LastEditors: renjy
 * @LastEditTime: 2023-10-26 11:43:04
 */

#pragma once
#include <list>
#include <map>
#include <memory>
#include <mutex>  // NOLINT
#include <vector>
#include "Eigen/Dense"

#include "common/logger.h"
#include "common/run_time.h"
#include "common/transform.h"
#include "include/mapping_and_location_math.h"
#include "location/location_interface.h"

#include "ekf_calcuator/ekf_calcuator.h"
#include "ekf_calcuator/recursive_algorithm.h"
#include "ekf_calcuator/reflector_ekf.h"
#include "include/config_struct.h"
#include "landmark_tool/trilateration.h"

#ifdef TEST_DEBUG
#include "common_lib/node.h"
#endif

namespace gomros {
namespace data_process {
namespace mapping_and_location {

class ReflectorLocation : public LocationInterface {
 public:
  using OdometerMessage = gomros::message::OdometerMessage;
  using RadarSensoryMessage = gomros::message::RadarSensoryMessage;
  using RadarSensoryInfo = gomros::message::RadarSensoryInfo;
  using Position = gomros::message::Position;

 public:
  explicit ReflectorLocation(const LocationConfig& config);
  ~ReflectorLocation() {}

  void HandleLaserData(const RadarSensoryMessage& data,
                       const Position& forecast_pos, bool is_move) override;
  void HandleOdomData(const OdometerMessage& odom_data) override;
  void HandleImuData(const ImuSensoryMessage& imu_data) override;
  void SetInitPose(float x, float y, float theta) override;
  void SetGlobalLocationPos(const Position& pos) override;

  bool StartGlobalLocating() override;

  void SetMapData(
      const std::map<int, std::map<int, Eigen::Vector2d>>& map) override;
  bool GetCurrentPosition(Position* pos) override;

  bool IsFinishLocate() override;

  RadarSensoryInfo GetLaserData() { return RadarSensoryInfo(); }
  RadarSensoryInfo GetParticleData() { return RadarSensoryInfo(); }
  Position GetCurrentPose() { return Position(); }
  cv::Mat GetImage() { return cv::Mat(); }

 private:
  /**
   * @name: 全局定位线程
   * @param {void*} ptr
   * @return {*}
   */
  static void* _globalLocatingThreadFunction(void* ptr);

  /**
   * @name: 处理观测信息
   * @param {Observation&} obs 观测反光板信息
   * @return {*}
   */
  void _handleObservation(const DataFusion::Observation& obs);

 private:
  std::map<int, std::map<int, Eigen::Vector2d>> land_mark_map_;
  bool is_finish_global_location_;
  bool is_get_location_pos_;
  LocationConfig reflector_location_config_;
  std::shared_ptr<DataFusion::RecursiveAlgorithm> data_fusion_;
  // 反光柱全局计算器
  std::shared_ptr<LandMark::Trilateration> reflector_global_cal_;
  // 反光柱定位计算器
  std::shared_ptr<DataFusion::ReflectorEkfCalcuator> location_real_time_;

  // 为全局定位记录下的激光消息
  std::vector<RadarSensoryInfo> ladar_datas_for_global_location_;

  Position current_pose_;
  Eigen::Matrix<double, 2, 2, Eigen::DontAlign> current_cov_;
  // 线程锁
  pthread_t global_location_mutex_;
  // 递归锁
  std::recursive_mutex radar_data_mutex_;
  std::recursive_mutex imu_data_mutex_;
  std::recursive_mutex state_data_mutex_;

  // 记录imu数据
  std::list<ImuSensoryMessage> imu_list_;  //  imu待处理数据
};

}  // namespace mapping_and_location
}  // namespace data_process
}  // namespace gomros
