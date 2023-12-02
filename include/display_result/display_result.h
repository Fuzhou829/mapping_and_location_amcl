/*
 * @Descripttion: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @version: 1.0
 * @Author: renjy
 * @Date: 2023-05-20 02:23:21
 * @LastEditors: renjy
 * @LastEditTime: 2023-10-18 14:48:12
 */
#pragma once
#include <vector>

#include "include/config_struct.h"
#include "message_lib/radar_message.h"
#include "message_lib/odometer_message.h"
#include "message_lib/simple_grid_map.h"

#include "common_lib/node.h"

namespace DisPlayResult {

using namespace gomros::data_process::mapping_and_location;  // NOLINT

enum ObsType {
  NEW_OBS = 0,  // 新增观测
  BEFORE_OBS = 1,  // 之前观测
  MAP_LANDMARK = 2   // 地图上的信息
};

class SimulationDataPublisher {
 public:
  using Node = gomros::common::Node;
  using RawPublisher = gomros::common::RawPublisher;
  using RadarSensoryInfo = gomros::message::RadarSensoryInfo;
  using RadarSensoryMessage = gomros::message::RadarSensoryMessage;
  using Position = gomros::message::Position;
  using SimpleGridMap = gomros::message::SimpleGridMap;

 public:
  explicit SimulationDataPublisher(const SensorMount& sensor_mount);
  virtual ~SimulationDataPublisher();

  /**
   * @name: 显示激光信息
   * @param message 激光点云信息 (原始)
   * @param current_pos 当前机器全局坐标
   * @return {*}
   */
  void DisplayLadarMessage(const RadarSensoryInfo& message);
  /**
   * @name: 显示激光信息及匹配的反光板信息
   * @param message 激光点云信息 （原始）
   * @param current_pos 当前机器全局坐标
   * @param obs 观测到的反光板信息 - 机器坐标系下
   * @param match_obs 与地图匹配 -- 全局坐标系、
   * @param cov 协方差矩阵
   * @return {*}
   */
  void DisPlayLadarAndObs(const RadarSensoryInfo& message,
                          const Position& current_pos,
                          const std::vector<Eigen::Vector2d>& obs,
                          const std::vector<Eigen::Vector2d>& match_obs,
                          const Eigen::Matrix2d& cov = Eigen::Matrix2d::Zero());

  /**
   * @name: 显示栅格地图
   * @param grid_map 栅格地图信息
   * @return
   */
  void DisPlayGridMap(SimpleGridMap& grid_map);  // NOLINT

  /**
   * @describes: 显示ekf的观测信息
   * @param state 状态 包括前三为当前坐标
   * @param new_obs_size 新增obs个数
   * @param match_id 匹配的id
   * @return {*}
   */
  void DisPlayReflectorEkf(const Eigen::VectorXd& state, int new_obs_size,
                           const std::vector<int>& match_id);

 private:
  SensorMount sensor_mount_;
  Node* node_;
  RawPublisher* publish_ladar_for_display_;
  RawPublisher* publish_reflector_ekf_;
  RawPublisher* grid_map_publisher_;
};


}  // namespace DisPlayResult

