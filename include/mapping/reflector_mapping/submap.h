/*
 * @Description: Copyright (C) 2023 山东亚历山大智能科技有限公司
 * @version: 1.2
 * @Author: renjy
 * @Data: 2023-06-01 02:02:50
 * @LastEditors: renjy
 * @LastEditTime: 2023-07-17 09:29:07
 */

#pragma once

#include <utility>
#include <memory>
#include "Eigen/Dense"

#include "include/config_struct.h"
#include "common/thread_pool.h"
#include "common/octree.h"
#include "ekf_calcuator/reflector_ekf.h"

#include "common/logger.h"

namespace gomros {
namespace data_process {
namespace mapping_and_location {

/**
 * @describes: 子图信息管理
 */
class Submap {
 public:
  using OdometerMessage = gomros::message::OdometerMessage;
  using ImuSensoryMessage = gomros::message::ImuSensoryMessage;

 public:
  Submap(const MappingConfig& config, internal_common::ThreadPool* thread_pool);
  virtual ~Submap() {}

  /**
   * @describes: 初始化submap中的反光板ekf
   * @param state 状态值
   * @return {*}
   */
  void InitReflectorEkf(const DataFusion::State& state);
  /**
   * @name: 处理预测信息
   * @param data 里程计数据
   * @return 
   */  
  void HandleOdomData(const OdometerMessage& data);
  /**
   * @name: 处理imu数据
   * @param imu_data imu数据
   * @return {*}
   */
  void HandleImuData(const ImuSensoryMessage& imu_data);
  /**
   * @name: 处理观测信息
   * @param obs 观测反光板信息 (雷达坐标系下)
   * @return int 子图中已经插入的数目
   */  
  int HandleObsData(const DataFusion::Observation& obs);
  /**
   * @describes: 设置子图id
   * @param submap_id
   * @return {*}
   */
  void SetSubMapId(int submap_id) { submap_id_ = submap_id;}
  /**
   * @describes: 获取子图id
   * @return {*}
   */  
  int GetSubMapId() { return submap_id_;}

  /**
   * @describes: 获取ekf-slam建图计算器
   * @return {*}
   */
  std::shared_ptr<DataFusion::ReflectorEkfCalcuator> GetReflectorEKFCal() {
    return reflector_ekf_cal_;
  }

  /**
   * @describes: 获取子图八叉树
   * @return {*}
   */  
  std::shared_ptr<Octree::Map> GetOctreeMap() {
    return octree_reflector_map_;
  }

  /**
   * @describes: 获取反光柱地图在submap坐标系下
   * @return {*}
   */
  DataFusion::ReflectorMap GetReflectorInSubMap() {
    return reflector_in_local_;
  }

  DataFusion::ReflectorMap GetReflectorInGlobal() {
    return reflectors_in_global_;
  }


  /**
   * @describes: 设置子图在全局坐标中的信息
   * @param pose 子图在全局坐标中的位子信息
   * @param cov 该位置对应的协方差
   * @return {*}
   */
  void SetGlobalPose(const Eigen::Vector3d &pose, const Eigen::Matrix3d &cov) {
    global_pose_.first = pose;
    global_pose_.second = cov;
    is_first_data_ = false;
  }

  /**
   * @describes: 获取子图在全局坐标中的信息
   * @return {*}
   */
  std::pair<Eigen::Vector3d, Eigen::Matrix3d> GetGlobalPose() {
    return global_pose_;
  }

  /**
   * @describes: 设置该子图在上一个子图的全局坐标的位姿关系
   * @param relative_pos 相对位姿关系
   * @return {*}
   */
  void SetRelativePose(const Eigen::Vector3d& relative_pos) {
    relative_pose_to_last_submap_ = relative_pos;
  }

  /**
   * @describes: 获取子图在上一个子图之间的位姿关系
   * @return {*}
   */
  Eigen::Vector3d GetRelativePose() {
    return relative_pose_to_last_submap_;
  }

  /**
   * @describes: 结束子图的建立
   * @return {*}
   */
  void FinishSubMap();

 private:
  /**
   * @describes: 填充八叉树
   * @return {*}
   */ 
  void _fillInOctree();
  /**
   * @describes: 判断是否可作为一个三角形插入八叉树中
   * @param {float} side_1
   * @param {float} side_2
   * @param {float} side_3
   * @return {*}
   */  
  bool _checkTriangle(float side_1, float side_2, float side_3);

  /**
   * @describes: 计算子图在地图中行走的轨迹（形成box）方便日志显示
   * @param state 机器当前状态
   * @return 
   */
  void _calStateBox(const DataFusion::State& state);


 private:
  bool is_first_data_;
  int num_range_data_;
  int submap_id_;
  MappingConfig config_;
  DataFusion::State last_state_;  // 上一时刻的状态 ekf
  internal_common::ThreadPool* const thread_pool_;
  std::pair<Eigen::Vector3d, Eigen::Matrix3d> global_pose_;
  Eigen::Vector3d relative_pose_to_last_submap_;
  // 子图中反光柱中心及其协方差矩阵 (全局坐标系)
  DataFusion::ReflectorMap reflectors_in_global_;
  // 子图中反光柱中心及其协方差矩阵 (局部坐标系)
  DataFusion::ReflectorMap reflector_in_local_;
  std::shared_ptr<Octree::Map> octree_reflector_map_;
  std::shared_ptr<DataFusion::ReflectorEkfCalcuator> reflector_ekf_cal_;
  // for test
  float min_x;
  float min_y;
  float max_x;
  float max_y;
};






}  // namespace mapping_and_location
}  // namespace data_process
}  // namespace gomros




