/*
 * @Description: Copyright (C) 2023 山东亚历山大智能科技有限公司
 * @version: 1.2
 * @Author: renjy
 * @Data: 2023-09-12 10:57:51
 * @LastEditors: renjy
 * @LastEditTime: 2023-10-19 21:59:59
 */
#pragma once
#include <stdint.h>

#include "Eigen/Dense"
#include "karto_mapping/Karto.h"

namespace gomros {
namespace data_process {
namespace mapping_and_location {
namespace karto {


class Mapper;

class LandmarkMatcher {
 public:
  virtual ~LandmarkMatcher();

 public:
  static LandmarkMatcher* Create(Mapper* pMapper);
  void SetQrInRobot(float x, float y, float t);
  void SetLadarInRobot(float x, float y, float t);

  // 前端 -- 帧间匹配
  bool MatchScan(LocalizedRangeScan* pScan,
                 const LocalizedRangeScanVector& rBaseScans,
                 Pose2& rMean, Matrix3& rCovariance);

  // 前端 -- 帧间匹配 只匹配前一帧
  bool MatchScan(LocalizedRangeScan* pScan,
                 const LocalizedRangeScan& rBaseScan,
                 Pose2& rMean, Matrix3& rCovariance);

  // 后端 -- 闭环判断
  bool IsGetLoopScan(LocalizedRangeScan* pScan,
                     const LocalizedRangeScan* comparison_scan,
                     Pose2& rMean, Matrix3& rCovariance);

  /**
   * @description: 查找和该帧数据具有一样landmark的雷达组
   * @return {*}
   */
  LocalizedRangeScanVector FindLoopClosureLadar(LocalizedRangeScan* pScan);

 protected:
  /**
   * Default constructor
   */
  LandmarkMatcher(Mapper* pMapper)
    : m_pMapper(pMapper) {}
 private:
  void _calNewScanInfo(const LandmarkConstraintsInfo& base_info,
                       const Pose2& base_pose, const Matrix3& base_covariance,
                       const LandmarkConstraintsInfo& current_scan,
                       Pose2& rMean, Matrix3& rCovariance);
 private:
  Mapper* m_pMapper;
  Eigen::Matrix<double, 3, 3> ladar_robot_;
  Eigen::Matrix<double, 3, 3> qr_robot_;
};







}  // namespace karto
}  // namespace mapping_and_location
}  // namespace data_process
}  // namespace gomros







