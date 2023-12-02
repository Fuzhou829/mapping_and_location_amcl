/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.0
 * @Author: renjy
 * @Date: 2022-11-07 09:46:10
 * @LastEditTime: 2023-10-23 20:45:30
 */
#pragma once
#include <stdint.h>
#include <mutex>  // NOLINT
#include <list>
#include <string>

#include "Eigen/Dense"

#include "message_lib/odometer_message.h"
#include "message_lib/position_message.h"
#include "message_lib/imu_message.h"

#include "include/config_struct.h"
#include "landmark_tool/landmark_center_calcuator.h"

namespace DataFusion {

/**
 * @brief: 用于融合imu和odom数据信息 
 *         两种模式：1.以imu数据为测量，odom数据为观测2.只利用imu的角度信息
 *         目前只融合odom的角度信息
 */
enum class DataFusionModel : uint8_t {
  IMU_ODOM,
  ODOM_IMU_THETA
};


class EKFCalcuator {
 public:
  using OdometerMessage = gomros::message::OdometerMessage;
  using ImuSensoryMessage = gomros::message::ImuSensoryMessage;
  using Position = gomros::message::Position;

 public:
  EKFCalcuator(
          const gomros::data_process::mapping_and_location::EKFConfig& config,
          const std::string& name);
  virtual ~EKFCalcuator() {}
  // 设置初始pos
  void SetInitPos(const Position& pos, const Eigen::Matrix3d& covariance);
  void ClearImuData() {
    std::lock_guard<std::recursive_mutex> lk(ekf_mutex_);
    imu_list_.clear();
  }
  // 处理接收到的odom信息
  void HandleOdomData(const OdometerMessage& data);
  // 处理接收到的imu信息
  void HandleImuData(const ImuSensoryMessage& data, bool is_need_clean = true);
  // 获取滤波结果
  void GetCurrentState(Position* current_pos, Eigen::Matrix3d* covariance);
  void GetCurrentState(Position* current_pos);
  // 更新
  void Update();

  // void Update(uint64_t time_stamp);

 private:
  /**
   * @name: 双轮差速运动模型
   * @return {*}
   */  
  void _calOdomMessageDiffWheel(const OdometerMessage& data);
  /**
   * @name: 双舵轮运动模型
   * @return {*}
   */  
  void _calOdomMessageDoubleSteer(const OdometerMessage& data);
  /**
   * @name: 单舵轮运动模型
   * @return {*}
   */
  void _calOdomMessageSingleSteer(const OdometerMessage& data);


 private:
  Position previous_pos_;   // 上一时刻的ekf求解的位姿
  Position current_pos_;    // EKF 求得位姿信息
  Eigen::Matrix<double, 3, 3, Eigen::DontAlign> state_covariance_;
  gomros::data_process::mapping_and_location::EKFConfig config_;
  std::recursive_mutex ekf_mutex_;
  std::recursive_mutex imu_mutex_;

  std::string call_back_name_;
  Eigen::Matrix<double, 2, 2, Eigen::DontAlign> control_noise_covariance_;
  std::list<ImuSensoryMessage> imu_list_;          //  imu待处理数据
  bool is_need_update_;
  bool is_get_init_pos_;
  Eigen::Matrix<float, 4, 3, Eigen::DontAlign> config_mat_;  // 舵轮配置矩阵
};



}  // namespace DataFusion



