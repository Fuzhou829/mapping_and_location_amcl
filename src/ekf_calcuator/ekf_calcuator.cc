/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.0
 * @Author: renjy
 * @Date: 2022-11-07 09:45:40
 * @LastEditTime: 2023-10-23 20:45:34
 */
#include "ekf_calcuator/ekf_calcuator.h"
#include "include/mapping_and_location_math.h"

#include "common/logger.h"
namespace DataFusion {

EKFCalcuator::EKFCalcuator(
  const gomros::data_process::mapping_and_location::EKFConfig& config,
  const std::string& name) {
  config_ = config;
  call_back_name_ = name;
  previous_pos_.mfX = 0.0f;
  previous_pos_.mfY = 0.0f;
  previous_pos_.mfTheta = 0.0f;
  state_covariance_.setZero(3, 3);  // 协方差矩阵初始化
  control_noise_covariance_.setZero(2, 2);
  control_noise_covariance_(0, 0) =
    std::pow(config_.control_left_error, 2);
  control_noise_covariance_(1, 1) =
    std::pow(config_.control_right_error, 2);
  is_need_update_ = false;
  is_get_init_pos_ = false;

  // SLAM_INFO("config length (%f %f), aplha (%f %f)",
            // config_.sensor_mount.back_wheel_length,
            // config_.sensor_mount.front_wheel_length,
            // config_.sensor_mount.back_wheel_alpha,
            // config_.sensor_mount.front_wheel_alpha);
  config_mat_ <<
    1.0f, 0.0f, -config_.sensor_mount.back_wheel_length *
                sin(config_.sensor_mount.back_wheel_alpha),
    0.0f, 1.0f, -config_.sensor_mount.back_wheel_length *
                cos(config_.sensor_mount.back_wheel_alpha),
    1.0f, 0.0f, config_.sensor_mount.front_wheel_length *
                sin(config_.sensor_mount.front_wheel_alpha),
    0.0f, 1.0f, config_.sensor_mount.front_wheel_length *
                cos(config_.sensor_mount.front_wheel_alpha);
}

void EKFCalcuator::SetInitPos(const Position& pos,
  const Eigen::Matrix3d& covariance) {
  std::lock_guard<std::recursive_mutex> lk(ekf_mutex_);
  current_pos_ = pos;
  previous_pos_ = pos;
  state_covariance_ = covariance;
  is_get_init_pos_ = true;
}


void EKFCalcuator::HandleOdomData(const OdometerMessage& data) {
  std::lock_guard<std::recursive_mutex> lk(ekf_mutex_);
  if (!is_get_init_pos_) return;
  if (gomros::data_process::mapping_and_location::OdomType::DiffWheelModel ==
      config_.odom_type) {
    _calOdomMessageDiffWheel(data);
  } else if (
    gomros::data_process::mapping_and_location::OdomType::DoubleSteerModel ==
    config_.odom_type) {
    _calOdomMessageDoubleSteer(data);
  } else if (
    gomros::data_process::mapping_and_location::OdomType::SingleSteerModel ==
    config_.odom_type) {
    _calOdomMessageSingleSteer(data);
  }
  return;
}

void EKFCalcuator::HandleImuData(const ImuSensoryMessage& data,
  bool is_need_clean) {
  std::lock_guard<std::recursive_mutex> lk(ekf_mutex_);
  if (!is_get_init_pos_) return;
  if (imu_list_.size() > 0) {
    if (data.time_stamp <= imu_list_.back().time_stamp) return;
  }
  if (is_need_clean && imu_list_.size() > 200) imu_list_.clear();
  imu_list_.push_back(data);
  return;
}


void EKFCalcuator::GetCurrentState(Position* current_pos,
  Eigen::Matrix3d* covariance) {
  std::lock_guard<std::recursive_mutex> lk(ekf_mutex_);
  *current_pos = current_pos_;
  *covariance = state_covariance_;
  return;
}

void EKFCalcuator::GetCurrentState(Position* current_pos) {
  std::lock_guard<std::recursive_mutex> lk(ekf_mutex_);
  *current_pos = current_pos_;
  return;
}

void EKFCalcuator::Update() {
  std::lock_guard<std::recursive_mutex> lk(ekf_mutex_);
  // TODO(r) 暂时屏蔽单舵轮融合imu信息
  if (gomros::data_process::mapping_and_location::OdomType::SingleSteerModel ==
      config_.odom_type) imu_list_.clear();
  if (imu_list_.empty()) return;
  if (current_pos_.mlTimestamp < previous_pos_.mlTimestamp) return;
  double observation = 0.0f;
  float vel_theta = 0.0;
  int count = 0;
  for (auto iter = imu_list_.begin(); iter != imu_list_.end(); iter++) {
    if (iter->time_stamp < previous_pos_.mlTimestamp) continue;
    if (iter->time_stamp > current_pos_.mlTimestamp) break;
    count++;
    vel_theta += iter->z_omega;
  }
  while (!imu_list_.empty()) {
    if (imu_list_.front().time_stamp > current_pos_.mlTimestamp) break;
    imu_list_.pop_front();
  }
  if (count <= 0) return;
  vel_theta = vel_theta / (count * 1.0f);
  observation = previous_pos_.mfTheta + vel_theta *
                (current_pos_.mlTimestamp - previous_pos_.mlTimestamp) * 1e-6;
  if (fabs(observation) > 100) {
    SLAM_ERROR("theta %f %f %lu %lu", previous_pos_.mfTheta, vel_theta,
                current_pos_.mlTimestamp, previous_pos_.mlTimestamp);
  }
  observation = SLAMMath::NormalizePITheta(observation);
  // 更新卡尔曼增益
  // 观测矩阵 H
  Eigen::Matrix<double, 1, 3> H = {0.0, 0.0, 1.0};
  double denominator = H * state_covariance_ * (H.transpose()) +
                      std::pow(config_.imu_theta_noise, 2);
  Eigen::Vector3d K_k = state_covariance_ * (H.transpose()) / denominator;

  Eigen::Vector3d delta_pos =
    K_k * SLAMMath::NormalizePITheta(observation - current_pos_.mfTheta);
  delta_pos(2) = SLAMMath::NormalizePITheta(delta_pos(2));
  // current_pos_.mfX += delta_pos(0);
  // current_pos_.mfY += delta_pos(1);
  current_pos_.mfTheta += delta_pos(2);
  current_pos_.mfTheta = SLAMMath::NormalizePITheta(current_pos_.mfTheta);
  // 更新协方差
  Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();
  state_covariance_ = (identity - K_k * H) * state_covariance_;
  is_need_update_ = false;
  // // SLAM_DEBUG("current pos = %f %f %f\n",
  //             current_pos_.mfX, current_pos_.mfY, current_pos_.mfTheta);
  return;
}

void EKFCalcuator::_calOdomMessageDiffWheel(const OdometerMessage& data) {
  // static FILE* ekfpos_file = fopen("./allodompos.txt", "w+");
  double vel_left = data.mstruDiffSteerSts.mfLeftLinearVel;
  double vel_right = data.mstruDiffSteerSts.mfRightLinearVel;
  uint64_t odom_time = data.mclDeltaPosition.mlTimestamp;
  uint64_t current_time = current_pos_.mlTimestamp;
  if (data.mclDeltaPosition.mlTimestamp < current_pos_.mlTimestamp) return;
  double delta_time =
      1e-6 * (data.mclDeltaPosition.mlTimestamp - current_pos_.mlTimestamp);
  if (delta_time > 0.1) {
    SLAM_ERROR("call_back_name_ %s odom too long %f before"
              " %lu %lu after %lu, %lu",
                call_back_name_.c_str(), delta_time, odom_time, current_time,
                data.mclDeltaPosition.mlTimestamp, current_pos_.mlTimestamp);
  }
  // 双轮差速模型下
  double delta_left_distance = vel_left * delta_time;
  double delta_right_distance = vel_right * delta_time;
  double delta_distance = 0.5 * (delta_left_distance + delta_right_distance);
  double delta_theta = SLAMMath::NormalizePITheta(
        (delta_right_distance - delta_left_distance) /
        config_.sensor_mount.wheel_distance);
  double sin_theta = sin(current_pos_.mfTheta + 0.5 * delta_theta);
  double cos_theta = cos(current_pos_.mfTheta + 0.5 * delta_theta);
  double delta_x = delta_distance * cos_theta;
  double delta_y = delta_distance * sin_theta;
  previous_pos_ = current_pos_;
  // 预测 位姿及路标点
  // 预测 预测状态矩阵(位姿部分)
  current_pos_.mfX += delta_x;
  current_pos_.mfY += delta_y;
  current_pos_.mfTheta += delta_theta;
  current_pos_.mfTheta = SLAMMath::NormalizePITheta(current_pos_.mfTheta);
  current_pos_.mlTimestamp = data.mclDeltaPosition.mlTimestamp;

  // 预测 预测协方差矩阵
  // 位姿雅克比矩阵 state_/state_
  Eigen::MatrixXd G_pos = Eigen::MatrixXd::Identity(3, 3);
  G_pos << 1.0, 0.0, -delta_distance * sin_theta,
           0.0, 1.0, delta_distance * cos_theta,
           0.0, 0.0, 1.0;
  // 控制雅克比矩阵 state_/u[delta_left_distance, delta_right_distance]
  Eigen::MatrixXd G_u = Eigen::MatrixXd::Zero(3, 2);
  float wheel_distance = config_.sensor_mount.wheel_distance;
  G_u <<
    0.5 * (cos_theta - delta_distance * sin_theta / wheel_distance),
    0.5 * (cos_theta + delta_distance * sin_theta / wheel_distance),
    0.5 * (sin_theta + delta_distance * cos_theta / wheel_distance),
    0.5 * (sin_theta - delta_distance * cos_theta / wheel_distance),
    1.0 / wheel_distance, - 1.0 / wheel_distance;
  // 预测协方差矩阵更新
  state_covariance_ = G_pos * state_covariance_ * G_pos.transpose() +
                      G_u * control_noise_covariance_ * G_u.transpose();
  is_need_update_ = true;
  return;
}

void EKFCalcuator::_calOdomMessageDoubleSteer(const OdometerMessage& data) {
  if (data.mclDeltaPosition.mlTimestamp < current_pos_.mlTimestamp) return;
  double delta_time =
      1e-6 * (data.mclDeltaPosition.mlTimestamp - current_pos_.mlTimestamp);

  float fvel1 = data.mstruDualSteerSts.mdFrontLinearVel;
  float bvel1 = data.mstruDualSteerSts.mdBackLinearVel;
  float theta_f1 = data.mstruDualSteerSts.mdFrontRotAngle;
  float theta_b1 = data.mstruDualSteerSts.mdBackRotAngle;
  // SLAM_INFO("vel (%f %f), theta (%f %f)", fvel1, bvel1, theta_f1, theta_b1);
  float cf1, sf1, cb1, sb1;
  cf1 = cos(theta_f1);
  sf1 = sin(theta_f1);
  cb1 = cos(theta_b1);
  sb1 = sin(theta_b1);

  Eigen::Matrix<float, 4, 1> matrix_41;
  matrix_41<<
  bvel1 * cb1, bvel1 * sb1, fvel1 * cf1, fvel1 * sf1;

  Eigen::Matrix<float, 3, 3> matrix_33 =
    config_mat_.transpose() * config_mat_;
  Eigen::Matrix<float, 3, 4> matrix_btbnbt =
    matrix_33.inverse() * config_mat_.transpose();
  Eigen::Matrix<float, 3, 1> result =
    matrix_btbnbt * matrix_41;

  float vel_x = result(0);
  float vel_y = result(1);
  float w = result(2);

  previous_pos_ = current_pos_;

  Position delta_pos;
  delta_pos.mfX = delta_time * result(0);
  delta_pos.mfY = delta_time * result(1);
  delta_pos.mfTheta = delta_time * result(2);

  // 预测 预测协方差矩阵
  // 位姿雅克比矩阵 state_/state_(t-1)
  Eigen::MatrixXd G_xi = Eigen::MatrixXd::Identity(3, 3);
  float s_theta = sin(current_pos_.mfTheta);
  float c_theta = cos(current_pos_.mfTheta);
  float s_delta_sum = sin(current_pos_.mfTheta + delta_pos.mfTheta);
  float c_delta_sum = cos(current_pos_.mfTheta + delta_pos.mfTheta);
  // 直行 和 旋转进行区分
  float v_r = std::sqrt(std::pow(result(0), 2) + std::pow(result(1), 2));
  if (fabs(result(2)) < 0.000001) {
    G_xi <<
      1, 0, -v_r * delta_time * s_theta,
      0, 1, v_r * delta_time * c_theta,
      0, 0, 1;
  } else {
    G_xi <<
      1, 0, v_r * (c_delta_sum - c_theta) / result(2),
      0, 1, v_r * (s_delta_sum - s_theta) / result(2),
      0, 0, 1;
  }

  // 控制雅克比矩阵 state_/u[v, w]
  Eigen::MatrixXd G_u = Eigen::MatrixXd::Zero(3, 2);
  if (fabs(result(2)) < 0.000001) {
    G_u <<
      delta_time * c_theta, 0,
      delta_time * s_theta, 0,
      0, 0;
  } else {
    G_u <<
      (-s_theta + s_delta_sum) / result(2),
      v_r * (s_theta - s_delta_sum) / (result(2) * result(2)) +
      v_r * c_delta_sum * delta_time / result(2),
      (c_theta - c_delta_sum) / result(2),
      -v_r * (c_theta - c_delta_sum) / (result(2) * result(2)) +
      v_r * s_delta_sum * delta_time / result(2),
      0, delta_time;
  }


  Eigen::MatrixXd control_noise_covariance = Eigen::MatrixXd::Identity(2, 2);
  if (fabs(result(2)) < 0.000001) {
    control_noise_covariance(0, 0) =
    config_.control_left_error * std::pow(v_r, 2);
    control_noise_covariance(1, 1) =
    config_.control_right_error * std::pow(v_r, 2);
  } else {
    control_noise_covariance(0, 0) =
      config_.control_left_error *
      (std::pow(v_r, 2) + std::pow(result(2), 2));
    control_noise_covariance(1, 1) =
      config_.control_right_error *
      (std::pow(v_r, 2) + std::pow(result(2), 2));
  }
  current_pos_ = current_pos_ * delta_pos;

  current_pos_.mfTheta = SLAMMath::NormalizePITheta(current_pos_.mfTheta);
  current_pos_.mlTimestamp = data.mclDeltaPosition.mlTimestamp;
  // 预测协方差矩阵更新
  state_covariance_ = G_xi * state_covariance_ * G_xi.transpose() +
                      G_u * control_noise_covariance_ * G_u.transpose();
  is_need_update_ = true;
  return;
}


void EKFCalcuator::_calOdomMessageSingleSteer(const OdometerMessage& data) {
  if (data.mclDeltaPosition.mlTimestamp < current_pos_.mlTimestamp) return;
  double delta_time =
      1e-6 * (data.mclDeltaPosition.mlTimestamp - current_pos_.mlTimestamp);
  float ldLineVel = data.mstruSingleSteerSts.mdLinearVel;
  float ldAngularVel = data.mstruSingleSteerSts.mdAngularVel;
  float ldTurnAng = data.mstruSingleSteerSts.mdRotAngle;

  if (std::isnan(ldLineVel) || std::isnan(ldAngularVel) || std::isnan(ldTurnAng)) {
    SLAM_ERROR("get vel is nan %f %f %f", ldLineVel, ldAngularVel, ldTurnAng);
    return;
  }

  double ldCarLineVel = ldLineVel * (cos(ldTurnAng));
  double ldCarAngleVel = ldLineVel * (sin(ldTurnAng)) /
                        config_.sensor_mount.to_centre_distance;
  // SLAM_INFO("get vel %f %f", ldCarLineVel, ldCarAngleVel);

  double delta_theta = ldCarAngleVel * delta_time;

  double delta_x = ldCarLineVel * cos(delta_theta) * delta_time;
  double delta_y = ldCarLineVel * sin(delta_theta) * delta_time;
  Position delta_pos;
  delta_pos.mfX = delta_x;
  delta_pos.mfY = delta_y;
  delta_pos.mfTheta = delta_theta;
  current_pos_ = current_pos_ * delta_pos;
  current_pos_.mfTheta = SLAMMath::NormalizePITheta(current_pos_.mfTheta);
  current_pos_.mlTimestamp = data.mclDeltaPosition.mlTimestamp;
  return;
}



}  // namespace DataFusion
