/*
 * @Descripttion: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @version: 1.0
 * @Author: renjy
 * @Date: 2023-04-03 13:17:18
 * @LastEditors: renjy
 * @LastEditTime: 2023-09-27 16:37:39
 */
#pragma once

#include "Eigen/Dense"

#include "include/config_struct.h"
#include "location/location_interface.h"

namespace gomros {
namespace data_process {
namespace mapping_and_location {

/**
 * @name: 为万集雷达提供接口
 * @return {*}
 */

class WanJiLocation : public LocationInterface {
 public:
  explicit WanJiLocation(const LocationConfig& config);
  virtual ~WanJiLocation() {}

  void HandleLaserData(const RadarSensoryMessage& data,
                       const Position& forecast_pos, bool is_move) override;

  void SetInitPose(float x, float y, float theta) override {}
  void SetGlobalLocationPos(const Position& pos) override {}
  bool StartGlobalLocating() override {}
  bool GetCurrentPosition(Position* pos) override;
  bool IsFinishLocate() override;
  gomros::message::RadarSensoryInfo GetLaserData() {
    return gomros::message::RadarSensoryInfo();
  }
  gomros::message::RadarSensoryInfo GetParticleData() {
    return gomros::message::RadarSensoryInfo();
  }
  gomros::message::Position GetCurrentPose() {
    return gomros::message::Position();
  }
  cv::Mat GetImage() { return cv::Mat(); }

 private:
  bool is_finished_location_;
  bool is_get_pos_;
  Position current_pos_;
  LocationConfig config_;
  Eigen::Matrix<double, 3, 3> ladar_mat_;
  int print_count_;
};

}  // namespace mapping_and_location
}  // namespace data_process
}  // namespace gomros
