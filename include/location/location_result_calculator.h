/*
 * @Descripttion: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @version: 1.0
 * @Author: renjy
 * @Date: 2023-03-19 02:33:34
 * @LastEditors: renjy
 * @LastEditTime: 2023-10-13 16:05:20
 */
#pragma once

#include <map>
#include <vector>
#include <mutex>  // NOLINT

#include "Eigen/Dense"

#include "include/config_struct.h"
#include "location/location_area.h"
#include "common/logger.h"


namespace DataFusion {

namespace MAL = gomros::data_process::mapping_and_location;

struct IntersectingAreas {
  MAL::LocationArea area;
  MAL::LocationType location_type_1;
  MAL::LocationType location_type_2;
  IntersectingAreas() {
    location_type_1 = MAL::LocationType::FusionLocation;
    location_type_2 = MAL::LocationType::FusionLocation;
  }
  bool IsQrIntersectingArea() {
    if (MAL::LocationType::QR == location_type_1 ||
        MAL::LocationType::QR == location_type_2)
      return true;
    return false;
  }
};


struct PoseInfo {
  PoseInfo() {}
  PoseInfo(const Eigen::Vector3f& pose_info, uint64_t time_stamp,
          float score, MAL::LocationType location_type) {
    this->location_type = location_type;
    this->pose_info = pose_info;
    this->score = score;
    this->time_stamp = time_stamp;
  }
  Eigen::Vector3f pose_info;   // 位姿信息
  uint64_t time_stamp;        // 时间戳
  float score;                // 定位置信得分
  MAL::LocationType location_type;   // 定位模式
};

class LocationResult {
 public:
  explicit LocationResult(const MAL::LocationType& type);
  ~LocationResult() {}
  /**
   * @name: 设置定位区域信息
   */
  void SetLocationArea(
    const std::map<MAL::LocationType, std::vector<MAL::LocationArea>>& areas);
  /**
   * @name: 获取数据融合结果
   */
  std::map<MAL::LocationType, int> GetLocationResult(
    const std::map<MAL::LocationType, PoseInfo>& pos_candidate,
    PoseInfo* result, bool is_get_qr = false);

  std::map<MAL::LocationType, int> GetLocationType() {
    std::lock_guard<std::recursive_mutex> lk(location_result_mutex_);
    return last_location_type_;
  }

 private:
  /**
   * @name: 获取当前区域的交叉区域
   */ 
  void _getIntersectingAreas();
  /**
   * @name: 融合交叉区域位姿
   */  
  void _intersectingAreasPoseFusion(
      const IntersectingAreas& interscecting_area,
      const std::map<MAL::LocationType, PoseInfo>& pos_candidate,
     PoseInfo* result);
  /**
   * @name: 根据定位结果及定位区域进行选择定位结果
   */
  bool _chooseResult(
    const std::map<MAL::LocationType, PoseInfo>& pos_candidate,
    bool is_get_qr,
   PoseInfo* result);

  /**
   * @name:计算offset信息
   */
  Eigen::Matrix<double, 3, 3>
    _calQrOffset(const PoseInfo& qr_pos, const PoseInfo& pos);

  /**
  * @name: 获取转换之后的qr_pos
  */
  void _calNewQrPos(
    const Eigen::Matrix<double, 3, 3>& qr_offset,
   PoseInfo* qr_pos);

 private:
  std::map<MAL::LocationType, std::vector<MAL::LocationArea>> location_areas_;
  // 是否为相交区域
  std::vector<IntersectingAreas> intersecting_area_;
  MAL::LocationType global_location_type_;
  // 定位模式 int 0 - 上一时刻选择 1,2... - 当前选择
  std::map<MAL::LocationType, int> location_type_;
  // 上一时刻选择的定位模式
  std::map<MAL::LocationType, int> last_location_type_;
  std::recursive_mutex location_result_mutex_;
};

}  // namespace DataFusion
