/*
 * @Description: Copyright (C) 2023 山东亚历山大智能科技有限公司
 * @version: 1.2
 * @Author: renjy
 * @Data: 2023-09-29 13:43:40
 * @LastEditors: renjy
 * @LastEditTime: 2023-10-19 22:09:20
 */

#include "karto_mapping/Macros.h"
#include "karto_mapping/landmark_constraints.h"
#include "karto_mapping/Mapper.h"

#include "Eigen/Dense"

#include "include/mapping_and_location_math.h"

namespace gomros {
namespace data_process {
namespace mapping_and_location {
namespace karto {

LandmarkMatcher::~LandmarkMatcher() {}

LandmarkMatcher* LandmarkMatcher::Create(Mapper* pMapper) {
  LandmarkMatcher* pLandmarkMatcher = new LandmarkMatcher(pMapper);
  return pLandmarkMatcher;
}
void LandmarkMatcher::SetQrInRobot(float x, float y, float t) {
  // TODO(r) 配置文件中读取
  SLAM_INFO("qr offset %f %f %f", x, y, t);
  qr_robot_ <<
    cos(t), -sin(t), x,
    sin(t), cos(t), y,
    0, 0, 1;
}

void LandmarkMatcher::SetLadarInRobot(float x, float y, float t) {
  // TODO(r) 配置文件中读取
  SLAM_INFO("ladar offset %f %f %f", x, y, t);
  ladar_robot_ <<
    cos(t), -sin(t), x,
    sin(t), cos(t), y,
    0, 0, 1;
}

bool LandmarkMatcher::MatchScan(LocalizedRangeScan* pScan,
  const LocalizedRangeScanVector& rBaseScans,
  Pose2& rMean, Matrix3& rCovariance) {
  LandmarkConstraintsInfo landmark_info_base, landmark_info;
  if (!pScan->IsGetLandmarkConstraints(&landmark_info)) {
    return false;
  }

  const_forEach(LocalizedRangeScanVector, &rBaseScans) {
    if ((*iter)->IsGetLandmarkConstraints(&landmark_info_base)) {
      if (landmark_info_base.landmark_id != landmark_info.landmark_id) continue;
      Pose2 pose_base = (*iter)->GetSensorPose();
      Matrix3 base_covariance = (*iter)->GetCovariance();
      _calNewScanInfo(landmark_info_base, pose_base, base_covariance,
                      landmark_info, rMean, rCovariance);
      Pose2 last_base = pScan->GetSensorPose();
      SLAM_DEBUG("(%f %f %f) -> (%f %f %f)",
            last_base.GetX(), last_base.GetY(), last_base.GetHeading(),
            rMean.GetX(), rMean.GetY(), rMean.GetHeading());
      return true;
    }
  }
  return false;
}


bool LandmarkMatcher::MatchScan(LocalizedRangeScan* pScan,
  const LocalizedRangeScan& rBaseScan,
  Pose2& rMean, Matrix3& rCovariance) {
  LandmarkConstraintsInfo landmark_info_base, landmark_info;
  if (!pScan->IsGetLandmarkConstraints(&landmark_info) ||
      !rBaseScan.IsGetLandmarkConstraints(&landmark_info_base)) {
    return false;
  }
  if (landmark_info.landmark_id != landmark_info_base.landmark_id) return false;
  Pose2 pose_base = rBaseScan.GetSensorPose();
  Matrix3 base_covariance = rBaseScan.GetCovariance();
  _calNewScanInfo(landmark_info_base, pose_base, base_covariance,
                  landmark_info, rMean, rCovariance);
  Pose2 last_base = pScan->GetSensorPose();
  SLAM_DEBUG("(%f %f %f) -> (%f %f %f)",
      last_base.GetX(), last_base.GetY(), last_base.GetHeading(),
      rMean.GetX(), rMean.GetY(), rMean.GetHeading());
  return true;
}

bool LandmarkMatcher::IsGetLoopScan(LocalizedRangeScan* pScan,
  const LocalizedRangeScan* comparison_scan,
  Pose2& rMean, Matrix3& rCovariance) {
  LandmarkConstraintsInfo landmark_info_base, landmark_info;
  if (!pScan->IsGetLandmarkConstraints(&landmark_info) ||
      !comparison_scan->IsGetLandmarkConstraints(&landmark_info_base)) {
    return false;
  }
  if (landmark_info_base.landmark_id != landmark_info.landmark_id) {
    return false;
  }
  Pose2 pose_base = comparison_scan->GetSensorPose();
  Matrix3 base_covariance = comparison_scan->GetCovariance();
  _calNewScanInfo(landmark_info_base, pose_base, base_covariance,
                  landmark_info, rMean, rCovariance);
  return true;
}

LocalizedRangeScanVector LandmarkMatcher::FindLoopClosureLadar(
  LocalizedRangeScan* pScan) {
  LocalizedRangeScanVector result;
  kt_int32u nScans = static_cast<kt_int32u>(
                    m_pMapper->m_pMapperSensorManager->GetScans(
                      pScan->GetSensorName()).size());
  int rStartNum = 0;
  LandmarkConstraintsInfo constraints_info_base;
  if (!pScan->IsGetLandmarkConstraints(&constraints_info_base)) {
    return result;
  }
  for (; rStartNum < nScans; rStartNum++) {
    LocalizedRangeScan* pCandidateScan = m_pMapper->m_pMapperSensorManager->
                                  GetScan(pScan->GetSensorName(), rStartNum);
    LandmarkConstraintsInfo constraints_info;
    if (pCandidateScan->IsGetLandmarkConstraints(&constraints_info)) {
      if (constraints_info.landmark_id == constraints_info_base.landmark_id) {
        result.push_back(pCandidateScan);
      }
    }
  }
  return result;
}


void LandmarkMatcher::_calNewScanInfo(const LandmarkConstraintsInfo& base_info,
  const Pose2& base_pose, const Matrix3& base_covariance,
  const LandmarkConstraintsInfo& current_scan,
  Pose2& rMean, Matrix3& rCovariance) {
  Eigen::Matrix3d in_qr_1(3, 3);
  in_qr_1 <<
    cos(base_info.theta), -sin(base_info.theta), base_info.x,
    sin(base_info.theta), cos(base_info.theta), base_info.y,
    0, 0, 1;
  Eigen::Matrix3d in_qr_2(3, 3);
  in_qr_2 <<
    cos(current_scan.theta), -sin(current_scan.theta), current_scan.x,
    sin(current_scan.theta), cos(current_scan.theta), current_scan.y,
    0, 0, 1;
  Eigen::Matrix3d ladar_1(3, 3);
  ladar_1 <<
    cos(base_pose.GetHeading()), -sin(base_pose.GetHeading()), base_pose.GetPosition().GetX(),
    sin(base_pose.GetHeading()), cos(base_pose.GetHeading()),  base_pose.GetPosition().GetY(),
    0, 0, 1;
  Eigen::Matrix3d ladar_2(3, 3);
  ladar_2 = ladar_1 * (ladar_robot_.inverse() * qr_robot_ * in_qr_1.inverse() *
                      in_qr_2 * qr_robot_.inverse() * ladar_robot_);
  rMean.SetX(ladar_2(0, 2));
  rMean.SetY(ladar_2(1, 2));
  rMean.SetHeading(atan2(ladar_2(1, 0), ladar_2(0, 0)));
  // 只利用旋转
  float t = SLAMMath::NormalizePITheta(
      base_pose.GetHeading() + current_scan.theta - base_info.theta);
  rMean.SetHeading(t);
  // TODO(r) 协方差求解需要根据上述公式推导
  // rCovariance(0, 0) = base_info.cov(0, 0);
  // rCovariance(1, 1) = base_info.cov(1, 1);
  // rCovariance(2, 2) = base_info.cov(2, 2);
  rCovariance(0, 0) = base_info.cov(0, 0);
  rCovariance(1, 1) = base_info.cov(1, 1);
  rCovariance(2, 2) = base_info.cov(2, 2);
  return;
}

}  // namespace karto
}  // namespace mapping_and_location
}  // namespace data_process
}  // namespace gomros
