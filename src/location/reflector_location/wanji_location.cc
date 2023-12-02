/*
 * @Descripttion: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @version: 1.0
 * @Author: renjy
 * @Date: 2023-04-03 13:25:07
 * @LastEditors: renjy
 * @LastEditTime: 2023-10-11 15:15:32
 */

#include "reflector_location/wanji_location.h"
#include "include/mapping_and_location_math.h"

#include "common/logger.h"

namespace gomros {
namespace data_process {
namespace mapping_and_location {



WanJiLocation::WanJiLocation(const LocationConfig& config) {
  config_ = config;
  is_finished_location_ = false;
  is_get_pos_ = false;
  ladar_mat_.setZero(3, 3);
  float ladar_theta = config_.sensor_mount.radar_position_theta;
  float ladar_pos_x = config_.sensor_mount.radar_position_x;
  float ladar_pos_y = config_.sensor_mount.radar_position_y;
  SLAM_INFO("ladar sensor mount %f %f %f",
            ladar_theta, ladar_pos_x, ladar_pos_y);
  ladar_mat_ <<
    cos(ladar_theta), -sin(ladar_theta), ladar_pos_x,
    sin(ladar_theta), cos(ladar_theta), ladar_pos_y,
    0, 0, 1;
  print_count_ = 0;
}


void WanJiLocation::HandleLaserData(const RadarSensoryMessage &data,
  const Position& forecast_pos, bool is_move) {
  is_get_pos_ = data.mstruRadarMessage.mstruRadarHeaderData.mcPosValid;
  if (!is_get_pos_) return;
  // 转换至robot坐标系下
  Eigen::Matrix3d ladar_mat(3, 3);
  float ladar_pos_theta = data.mstruRadarMessage.mstruRadarHeaderData.mfTheta;
  float ladar_pos_x = data.mstruRadarMessage.mstruRadarHeaderData.mfXPos;
  float ladar_pos_y = data.mstruRadarMessage.mstruRadarHeaderData.mfYPos;
  if (print_count_++ == 10) {
    print_count_ = 0;
    SLAM_INFO("move_ladar_global_pos %f %f %f",
      ladar_pos_x, ladar_pos_y, ladar_pos_theta);
  }
  if (std::isnan(ladar_pos_x) || std::isnan(ladar_pos_y) || std::isnan(ladar_pos_theta)) {
    SLAM_ERROR("pos is nan~~~~~~~~~~");
    is_get_pos_ = false;
    return;
  }

  ladar_mat <<
    cos(ladar_pos_theta), -sin(ladar_pos_theta), ladar_pos_x,
    sin(ladar_pos_theta), cos(ladar_pos_theta), ladar_pos_y,
    0, 0, 1;
  Eigen::Matrix3d robot_world(3, 3);
  robot_world = ladar_mat * ladar_mat_.inverse();

  current_pos_.mfX = robot_world(0, 2);
  current_pos_.mfY = robot_world(1, 2);
  current_pos_.mfTheta =
    SLAMMath::NormalizePITheta(atan2(robot_world(1, 0), robot_world(0, 0)));
  current_pos_.mlTimestamp =
    data.mstruRadarMessage.mstruRadarHeaderData.mlTimeStamp;
  if (is_get_pos_) is_finished_location_ = true;
  if (!is_move) is_get_pos_ = false;
  return;
}

bool WanJiLocation::GetCurrentPosition(Position* pos) {
  *pos = current_pos_;
  return is_get_pos_;
}

bool WanJiLocation::IsFinishLocate() {
  return is_finished_location_;
}



}  // namespace mapping_and_location
}  // namespace data_process
}  // namespace gomros
