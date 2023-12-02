/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.0
 * @Date: 2022-01-14 15:52:31
 * @LastEditTime: 2023-10-26 11:50:39
 * @Author: renjy
 */

#include "qr_location/qr_location.h"
#include "Eigen/Dense"
#include "include/mapping_and_location_math.h"
#include "common/logger.h"

namespace gomros {
namespace data_process {
namespace mapping_and_location {


void QrLocation::HandleOdomData(const OdometerMessage& data) {
  // pthread_mutex_lock(&robot_pos_mutex_);
  // current_pose_ = current_pose_ * data.mclDeltaPosition;
  // pthread_mutex_unlock(&robot_pos_mutex_);

  ekf_cal_->HandleOdomData(data);
  ekf_cal_->Update();
}
void QrLocation::HandleImuData(const ImuSensoryMessage& data) {
  ekf_cal_->HandleImuData(data);
  return;
}

bool QrLocation::HandleQrData(const DMreaderSensoryMessage& data,
  Position* qr_pos) {
  if (!data.read_success || !qr_map_.count(data.tag_num)) {
    last_qr_id_ = -1;
    return false;
  }

  QRInfo qr_info;
  qr_info.pos_x = data.x;
  qr_info.pos_y = data.y;
  qr_info.pos_theta =
    SLAMMath::NormalizePITheta(-data.theta * M_PI / 180.f);
  qr_info.tag_num = data.tag_num;

  // 相机在二维码坐标系下的坐标
  Eigen::Matrix3d camera_qr(3, 3);
  camera_qr <<
    cos(qr_info.pos_theta), -sin(qr_info.pos_theta), qr_info.pos_x,
    sin(qr_info.pos_theta), cos(qr_info.pos_theta), qr_info.pos_y,
    0, 0, 1;
  // 相机在车体中的坐标
  Eigen::Matrix3d camera_robot(3, 3);
  float theta = config_.sensor_mount.camera_position_theta * M_PI;
  camera_robot <<
    cos(theta), -sin(theta), config_.sensor_mount.camera_position_x,
    sin(theta), cos(theta), config_.sensor_mount.camera_position_y,
    0, 0, 1;
  // 二维码在全局中的坐标
  QRCoordinate qr_coor = qr_map_[data.tag_num];
  Eigen::Matrix3d qr_world(3, 3);
  qr_world <<
    cos(qr_coor.theta), -sin(qr_coor.theta), qr_coor.mx,
    sin(qr_coor.theta), cos(qr_coor.theta), qr_coor.my,
    0, 0, 1;
  Eigen::Matrix3d robot_world(3, 3);
  robot_world = qr_world * camera_qr * camera_robot.inverse();

  qr_pos->mlTimestamp = data.time_stamp;
  qr_pos->mfX = robot_world(0, 2);
  qr_pos->mfY = robot_world(1, 2);
  qr_pos->mfTheta =
    SLAMMath::NormalizePITheta(atan2(robot_world(1, 0), robot_world(0, 0)));
  if (last_qr_id_ != data.tag_num) {
    last_qr_id_ = data.tag_num;
    Position current_pose;
    GetCurrentPose(&current_pose);
    SLAM_INFO("DR pose %f %f %f %ld -> qr location %f %f %f %ld %d \n",
      current_pose.mfX, current_pose.mfY,
      current_pose.mfTheta, current_pose.mlTimestamp,
      qr_pos->mfX, qr_pos->mfY, qr_pos->mfTheta,
      qr_pos->mlTimestamp, data.tag_num);
  }
  // TODO(r) 那出去处理
  // SetInitPos(current_pose);
  return true;
}

void QrLocation::SetInitPos(const Position& pos) {
  Eigen::Matrix3d covariance;
  covariance.setZero(3, 3);
  ekf_cal_->SetInitPos(pos, covariance);
  // pthread_mutex_lock(&robot_pos_mutex_);
  // current_pose_ = pos;
  // pthread_mutex_unlock(&robot_pos_mutex_);
}

bool QrLocation::GetCurrentPose(Position* current_pose) {
  Eigen::Matrix3d covariance;
  ekf_cal_->Update();
  ekf_cal_->GetCurrentState(current_pose, &covariance);
  return true;
  // pthread_mutex_lock(&robot_pos_mutex_);
  // *current_pose = current_pose_;
  // pthread_mutex_unlock(&robot_pos_mutex_);
}





}  // namespace mapping_and_location
}  // namespace data_process
}  // namespace gomros
