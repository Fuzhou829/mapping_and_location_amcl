/*
 * @Descripttion: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @version: 1.0
 * @Author: renjy
 * @Date: 2023-03-19 02:33:34
 * @LastEditors: renjy
 * @LastEditTime: 2023-10-23 10:57:39
 */
#include <assert.h>

#include "location/location_result_calculator.h"
#include "include/mapping_and_location_math.h"

namespace DataFusion {

LocationResult::LocationResult(
  const MAL::LocationType& type): global_location_type_(type) {
  std::lock_guard<std::recursive_mutex> lk(location_result_mutex_);
  location_type_.clear();
  if (global_location_type_ != MAL::LocationType::FusionLocation ||
      global_location_type_ != MAL::LocationType::WanJiAndQR) {
    if (MAL::LocationType::Trilateral == global_location_type_) {
      location_type_.insert(
        std::make_pair(MAL::LocationType::LandMarkLocation, 0));
    } else {
      location_type_.insert(
          std::make_pair(global_location_type_, 0));
    }
  } else {
    // 暂时初始化按照AMCL
    location_type_.insert(
        std::make_pair(MAL::LocationType::AMCL, 0));
  }
  last_location_type_ = location_type_;
}

void LocationResult::SetLocationArea(
  const std::map<MAL::LocationType, std::vector<MAL::LocationArea>>& areas) {
  location_areas_.clear();
  intersecting_area_.clear();
  location_areas_ = areas;
  _getIntersectingAreas();
  std::lock_guard<std::recursive_mutex> lk(location_result_mutex_);
  location_type_.clear();
  if (global_location_type_ != MAL::LocationType::FusionLocation ||
      global_location_type_ != MAL::LocationType::WanJiAndQR) {
    if (MAL::LocationType::Trilateral == global_location_type_) {
      location_type_.insert(
        std::make_pair(MAL::LocationType::LandMarkLocation, 0));
    } else {
      location_type_.insert(
          std::make_pair(global_location_type_, 0));
    }
  } else {
    // 暂时初始化按照AMCL
    location_type_.insert(
        std::make_pair(MAL::LocationType::AMCL, 0));
  }
  last_location_type_ = location_type_;
}

std::map<MAL::LocationType, int> LocationResult::GetLocationResult(
  const std::map<MAL::LocationType, PoseInfo>& pos_candidate,
  PoseInfo* result, bool is_get_qr) {
  assert(pos_candidate.count(MAL::LocationType::QR));
  assert(pos_candidate.count(MAL::LocationType::LandMarkLocation));
  assert(pos_candidate.count(MAL::LocationType::AMCL));
  std::lock_guard<std::recursive_mutex> lk(location_result_mutex_);

  // 万集雷达定位 直接获取反光板定位结果
  if (MAL::LocationType::WanJiLandMarkLocation == global_location_type_) {
    *result = pos_candidate.at(MAL::LocationType::LandMarkLocation);
    last_location_type_ = location_type_;
    return location_type_;
  }
  // 三边定位 同样是反光板定位
  if (MAL::LocationType::Trilateral == global_location_type_) {
    *result = pos_candidate.at(MAL::LocationType::LandMarkLocation);
    last_location_type_ = location_type_;
    return location_type_;
  }

  // 判断全局是否采用融合
  if (MAL::LocationType::FusionLocation != global_location_type_ &&
      global_location_type_ != MAL::LocationType::WanJiAndQR) {
    *result = pos_candidate.at(global_location_type_);
    last_location_type_ = location_type_;
    return location_type_;
  }
  location_type_.clear();
  for (auto iter : last_location_type_) {
    if (iter.second == 0) {
      location_type_.insert(std::make_pair(iter.first, 0));
    }
  }
  PoseInfo amcl_pos = pos_candidate.at(MAL::LocationType::AMCL);
  PoseInfo qr_pos = pos_candidate.at(MAL::LocationType::QR);
  PoseInfo landmark_pos =
    pos_candidate.at(MAL::LocationType::LandMarkLocation);
  IntersectingAreas interscecting_area;
  bool is_get_area = false;
  for (int i = 0; i < intersecting_area_.size(); i++) {
    if (intersecting_area_[i].area.IsInArea(
        amcl_pos.pose_info(0), amcl_pos.pose_info(1)) ||
        intersecting_area_[i].area.IsInArea(
        qr_pos.pose_info(0), qr_pos.pose_info(1)) ||
        intersecting_area_[i].area.IsInArea(
          landmark_pos.pose_info(0), landmark_pos.pose_info(1))) {
      // 更新交叉区域信息--offset
      // TODO(r) 考虑机器未扫到二维码的情况
      interscecting_area = intersecting_area_[i];
      is_get_area = true;
      if (is_get_qr && intersecting_area_[i].IsQrIntersectingArea()) {
       PoseInfo pos =
          intersecting_area_[i].location_type_1 == MAL::LocationType::QR ?
          pos_candidate.at(intersecting_area_[i].location_type_2) :
          pos_candidate.at(intersecting_area_[i].location_type_1);
        interscecting_area = intersecting_area_[i];
        std::vector<MAL::LocationArea>* qr_area =
          &location_areas_.at(MAL::LocationType::QR);
        for (int i = 0; i < qr_area->size(); i++) {
          if ((*qr_area)[i].CheckContainArea(interscecting_area.area)) {
            SLAM_INFO("now qr pos (%f %f %f) other pos (%f %f %f)",
                  qr_pos.pose_info(0), qr_pos.pose_info(1), qr_pos.pose_info(2),
                  pos.pose_info(0), pos.pose_info(1), pos.pose_info(2));
            (*qr_area)[i].qr_offset = _calQrOffset(qr_pos, pos);
            interscecting_area.area.qr_offset = _calQrOffset(qr_pos, pos);
            SLAM_INFO("cal qr offset~~~~~~~~~~~~~~~~~~~~");
            break;
          }
        }
      }
      break;
    }
  }
  if (!is_get_area) {
    // 根据目前的位姿所在区域要求的位姿进行选择定位结果
    _chooseResult(pos_candidate, is_get_qr, result);
    if (!location_type_.count(result->location_type)) {
      location_type_.insert(std::make_pair(result->location_type, 1));
    }
    last_location_type_.clear();
    last_location_type_.insert(std::make_pair(result->location_type, 0));
    return location_type_;
  }

  if (is_get_qr) {
    // 更新qr位姿
    std::map<MAL::LocationType, PoseInfo>
      pos_candidate_tmp = pos_candidate;
     _calNewQrPos(interscecting_area.area.qr_offset,
     &pos_candidate_tmp.at(MAL::LocationType::QR));
    _intersectingAreasPoseFusion(interscecting_area, pos_candidate_tmp, result);
  } else {
    _intersectingAreasPoseFusion(interscecting_area, pos_candidate, result);
  }

  if (result->location_type == MAL::FusionLocation) {
    if (!location_type_.count(interscecting_area.location_type_1)) {
      location_type_.insert(std::make_pair(
        interscecting_area.location_type_1, 1));
    }
    if (!location_type_.count(interscecting_area.location_type_2)) {
      location_type_.insert(std::make_pair(
        interscecting_area.location_type_2, 2));
    }
    last_location_type_.clear();
    last_location_type_.insert(std::make_pair(
      interscecting_area.location_type_1, 0));
    last_location_type_.insert(std::make_pair(
      interscecting_area.location_type_2, 0));
  } else {
    if (!location_type_.count(result->location_type)) {
      location_type_.insert(std::make_pair(result->location_type, 1));
    }
    last_location_type_.clear();
    last_location_type_.insert(std::make_pair(result->location_type, 1));
  }

  return location_type_;
}

void LocationResult::_getIntersectingAreas() {
  IntersectingAreas intersecting_area;
  if (location_areas_.count(MAL::LocationType::AMCL)) {
      intersecting_area.location_type_1 = MAL::LocationType::AMCL;
    std::vector<MAL::LocationArea> amcl_area =
      location_areas_.at(MAL::LocationType::AMCL);
    if (location_areas_.count(MAL::LocationType::QR)) {
      intersecting_area.location_type_2 = MAL::LocationType::QR;
      std::vector<MAL::LocationArea> qr_area =
        location_areas_.at(MAL::LocationType::QR);
      for (int i = 0; i < amcl_area.size(); i++) {
        for (int j = 0; j < qr_area.size(); j++) {
          if (amcl_area[i].CheckIntersecting(qr_area[j],
              &intersecting_area.area)) {
              intersecting_area_.push_back(intersecting_area);
          }
        }
      }
    }

  if (location_areas_.count(MAL::LocationType::LandMarkLocation)) {
      intersecting_area.location_type_2 = MAL::LocationType::LandMarkLocation;
      std::vector<MAL::LocationArea> landmark_area =
        location_areas_.at(MAL::LocationType::LandMarkLocation);
      for (int i = 0; i < amcl_area.size(); i++) {
        for (int j = 0; j < landmark_area.size(); j++) {
          if (amcl_area[i].CheckIntersecting(landmark_area[j],
              &intersecting_area.area)) {
              intersecting_area_.push_back(intersecting_area);
          }
        }
      }
    }
  }

  if (location_areas_.count(MAL::LocationType::LandMarkLocation)) {
      intersecting_area.location_type_1 = MAL::LocationType::LandMarkLocation;
    std::vector<MAL::LocationArea> landmark_area =
      location_areas_.at(MAL::LocationType::LandMarkLocation);
    if (location_areas_.count(MAL::LocationType::QR)) {
      intersecting_area.location_type_2 = MAL::LocationType::QR;
      std::vector<MAL::LocationArea> qr_area =
        location_areas_.at(MAL::LocationType::QR);
      for (int i = 0; i < landmark_area.size(); i++) {
        for (int j = 0; j < qr_area.size(); j++) {
          if (landmark_area[i].CheckIntersecting(qr_area[j],
              &intersecting_area.area)) {
              intersecting_area_.push_back(intersecting_area);
          }
        }
      }
    }
  }
  return;
}


void LocationResult::_intersectingAreasPoseFusion(
  const IntersectingAreas& interscecting_area,
  const std::map<MAL::LocationType, PoseInfo>& pos_candidate,
  PoseInfo* result) {
  // 如果内部存在qr_pos 利用offset更新
  PoseInfo pos_1 = pos_candidate.at(interscecting_area.location_type_1);
  PoseInfo pos_2 = pos_candidate.at(interscecting_area.location_type_2);
  if (interscecting_area.location_type_1 == MAL::AMCL) {
    if (last_location_type_.count(MAL::AMCL) &&
        last_location_type_.at(MAL::AMCL) != 0) {
        // 不做任何操作向下进行
    } else {
      *result = pos_2;
      return;
    }
  }
  if (interscecting_area.location_type_2 == MAL::AMCL) {
    if (last_location_type_.count(MAL::AMCL) &&
        last_location_type_.at(MAL::AMCL) != 0) {
        // 不做任何操作向下进行
    } else {
      *result = pos_1;
      return;
    }
  }
  // TODO(r) 不能单纯的加权平均,需要递归~~~~
  float error_1 = 0.02;
  float error_2 = 0.03;
  float k = std::pow(error_1, 2) / (std::pow(error_1, 2) +
            std::pow(error_2, 2));
  result->pose_info(0) = pos_1.pose_info(0) +
                        k * (pos_2.pose_info(0) - pos_1.pose_info(0));
  result->pose_info(1) = pos_1.pose_info(1) +
                        k * (pos_2.pose_info(1) - pos_1.pose_info(1));
  result->pose_info(2) = pos_1.pose_info(2) +
                        k * (pos_2.pose_info(2) - pos_1.pose_info(2));
  result->pose_info(2) = SLAMMath::NormalizePITheta(result->pose_info(2));
  result->time_stamp = pos_1.time_stamp;
  result->location_type = MAL::LocationType::FusionLocation;
  return;
}


bool LocationResult::_chooseResult(
  const std::map<MAL::LocationType, PoseInfo>& pos_candidate,
  bool is_get_qr,
  PoseInfo* result) {
  // 首先检查AMCL
  std::vector<MAL::LocationArea> areas;
  if (location_areas_.count(MAL::LocationType::AMCL)) {
    areas.clear();
    areas = location_areas_.at(MAL::LocationType::AMCL);
    for (int i = 0; i < areas.size(); i++) {
     PoseInfo pos = pos_candidate.at(MAL::LocationType::AMCL);
      if (areas[i].IsInArea(pos.pose_info(0), pos.pose_info(1))) {
        *result = pos;
        return true;
      }
    }
  }

  if (location_areas_.count(MAL::LocationType::QR)) {
    areas.clear();
    areas = location_areas_.at(MAL::LocationType::QR);
    for (int i = 0; i < areas.size(); i++) {
     PoseInfo pos = pos_candidate.at(MAL::LocationType::QR);
      if (areas[i].IsInArea(pos.pose_info(0), pos.pose_info(1))) {
        if (is_get_qr) {
          _calNewQrPos(areas[i].qr_offset, &pos);
        }
        *result = pos;
        return true;
      }
    }
  }

  if (location_areas_.count(MAL::LocationType::LandMarkLocation)) {
    areas.clear();
    areas = location_areas_.at(MAL::LocationType::LandMarkLocation);
    for (int i = 0; i < areas.size(); i++) {
     PoseInfo pos = pos_candidate.at(MAL::LocationType::LandMarkLocation);
      if (areas[i].IsInArea(pos.pose_info(0), pos.pose_info(1))) {
        *result = pos;
        return true;
      }
    }
  }

  // 获取融合区域失败 选择amcl作为默认定位
  if (global_location_type_ == MAL::LocationType::WanJiAndQR) {
    *result = pos_candidate.at(MAL::LocationType::LandMarkLocation);
    return false;
  }
  *result = pos_candidate.at(MAL::LocationType::AMCL);
  // SLAM_ERROR("not in any target area ~~~~");
  return false;
}


Eigen::Matrix<double, 3, 3> LocationResult::_calQrOffset(
  const PoseInfo& qr_pos, const PoseInfo& pos) {
  Eigen::Matrix3d qr_mat(3, 3);
  qr_mat <<
    cos(qr_pos.pose_info(2)), -sin(qr_pos.pose_info(2)), qr_pos.pose_info(0),
    sin(qr_pos.pose_info(2)), cos(qr_pos.pose_info(2)), qr_pos.pose_info(1),
    0, 0, 1;
  Eigen::Matrix3d world_mat(3, 3);
  world_mat <<
    cos(pos.pose_info(2)), -sin(pos.pose_info(2)), pos.pose_info(0),
    sin(pos.pose_info(2)), cos(pos.pose_info(2)), pos.pose_info(1),
    0, 0, 1;
  Eigen::Matrix<double, 3, 3>
  qr_offset = world_mat * qr_mat.inverse();
  return qr_offset;
}


void LocationResult::_calNewQrPos(
  const Eigen::Matrix<double, 3, 3>& qr_offset,
 PoseInfo* qr_pos) {
  static int count = 0;
  Eigen::Matrix3d qr_mat(3, 3);
  qr_mat <<
    cos(qr_pos->pose_info(2)), -sin(qr_pos->pose_info(2)), qr_pos->pose_info(0),
    sin(qr_pos->pose_info(2)), cos(qr_pos->pose_info(2)), qr_pos->pose_info(1),
    0, 0, 1;
  Eigen::Matrix3d world_pos;
  world_pos = qr_offset * qr_mat;
  qr_pos->pose_info(0) = world_pos(0, 2);
  qr_pos->pose_info(1) = world_pos(1, 2);
  qr_pos->pose_info(2) =
    SLAMMath::NormalizePITheta(atan2(world_pos(1, 0), world_pos(0, 0)));
  if (++count % 10 == 0) {
    count = 1;
    SLAM_INFO("qr pos (%f %f %f)", qr_pos->pose_info(0),
              qr_pos->pose_info(1), qr_pos->pose_info(2));
  }

  return;
}

}  // namespace DataFusion
