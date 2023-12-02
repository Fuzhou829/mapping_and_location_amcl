/*
 * @Description: Copyright (C) 2023 山东亚历山大智能科技有限公司
 * @version: 1.2
 * @Author: renjy
 * @Data: Do not edit
 * @LastEditors: renjy
 * @LastEditTime: 2023-09-06 09:03:41
 */
#pragma once
#include "Eigen/Dense"
#include "include/mapping_and_location_math.h"

/**
 * @description: 坐标转换
 * @return {*}
 */

namespace Coordinate {

class Transform {
 public:
  Transform() {
    global_pos_1_.resize(3, 3);
    global_pos_1_.setZero();
    global_pos_2_.resize(3, 3);
    global_pos_2_.setZero();
    pos2_in_pos1_.resize(3, 3);
    pos2_in_pos1_.setZero();
  }
  virtual ~Transform() {}
  void SetPose1InGlobal(const Eigen::Vector3d& pose1) {
    global_pos_1_ <<
      cos(pose1(2)), -sin(pose1(2)), pose1(0),
      sin(pose1(2)), cos(pose1(2)), pose1(1),
      0, 0, 1;
  }
  void SetPose2InGlobal(const Eigen::Vector3d& pose2) {
    global_pos_1_ <<
      cos(pose2(2)), -sin(pose2(2)), pose2(0),
      sin(pose2(2)), cos(pose2(2)), pose2(1),
      0, 0, 1;
  }
  void SetPose2InPose1(const Eigen::Vector3d& relative_pos) {
    pos2_in_pos1_ <<
      cos(relative_pos(2)), -sin(relative_pos(2)), relative_pos(0),
      sin(relative_pos(2)), cos(relative_pos(2)), relative_pos(1),
      0, 0, 1;
  }

  void GetPose1InGlobal(Eigen::Vector3d* pose1) const {
    Eigen::Matrix3d result;
    result = global_pos_2_ * pos2_in_pos1_.inverse();
    (*pose1)(0) = result(0, 2);
    (*pose1)(1) = result(1, 2);
    (*pose1)(2) = SLAMMath::NormalizePITheta(atan2(result(1, 0), result(0, 0)));
    return;
  }
  void GetPose2InGlobal(Eigen::Vector3d* pose2) const {
    Eigen::Matrix3d result;
    result = global_pos_1_ * pos2_in_pos1_;
    (*pose2)(0) = result(0, 2);
    (*pose2)(1) = result(1, 2);
    (*pose2)(2) = SLAMMath::NormalizePITheta(atan2(result(1, 0), result(0, 0)));
    return;
  }
  void GetPose2InPose1(Eigen::Vector3d* relative_pos) const {
    Eigen::Matrix3d result;
    result = global_pos_1_.inverse() * global_pos_2_;
    (*relative_pos)(0) = result(0, 2);
    (*relative_pos)(1) = result(1, 2);
    (*relative_pos)(2) =
      SLAMMath::NormalizePITheta(atan2(result(1, 0), result(0, 0)));
    return;
  }

 private:
  Eigen::Matrix3d global_pos_1_;
  Eigen::Matrix3d global_pos_2_;
  Eigen::Matrix3d pos2_in_pos1_;
};




}  // namespace Coordinate






