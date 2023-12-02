/*
 * @Descripttion: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @version: 1.0
 * @Author: renjy
 * @Date: 2023-03-17 07:42:32
 * @LastEditors: renjy
 * @LastEditTime: 2023-08-29 10:02:42
 */
#pragma once

#include <float.h>
#include <algorithm>

#include "Eigen/Dense"


namespace gomros {
namespace data_process {
namespace mapping_and_location {


/**
 * @name: 获取到的定位区域
 */
struct LocationArea {
  float min_mx;    // 最左侧边 单位：m
  float min_my;    // 最下侧边 单位：m
  float max_mx;    // 最右侧边 单位：m
  float max_my;    // 最上测边 单位：m
  bool is_get_area;  // 是否获取定位区域
  LocationArea(float minx, float miny, float maxx, float maxy) {
    is_get_area = true;
    min_mx = minx;
    min_my = miny;
    max_mx = maxx;
    max_my = maxy;
    qr_offset.setIdentity(3, 3);
    qr_offset << 1, 0, 0,
                0, 1, 0,
                0, 0, 1;
  }
  LocationArea() {
    min_mx = FLT_MAX;
    min_my = FLT_MAX;
    max_mx = -FLT_MAX;
    max_my = -FLT_MAX;
    is_get_area = false;
    qr_offset.setIdentity(3, 3);
    qr_offset << 1, 0, 0,
                0, 1, 0,
                0, 0, 1;
  }
  bool IsInArea(float mx, float my) {
    if (mx < this->min_mx) return false;
    if (my < this->min_my) return false;
    if (mx > this->max_mx) return false;
    if (my > this->max_my) return false;
    return true;
  }
  // 获取交叉区域
  bool CheckIntersecting(const LocationArea& area, LocationArea* result) {
    if (area.min_mx > max_mx) return false;
    if (area.max_mx < min_mx) return false;
    if (area.min_my > max_my) return false;
    if (area.max_my < min_my) return false;
    result->min_mx = std::max(area.min_mx, min_mx);
    result->max_mx = std::min(area.max_mx, max_mx);
    result->min_my = std::max(area.min_my, min_my);
    result->max_my = std::min(area.max_my, max_my);
    return true;
  }
  // 是否包含area
  bool CheckContainArea(const LocationArea& area) {
    if (area.min_mx < this->min_mx) return false;
    if (area.min_my < this->min_my) return false;
    if (area.max_mx > this->max_mx) return false;
    if (area.max_my > this->max_my) return false;
    return true;
  }
  void RestArea() {
    min_mx = FLT_MAX;
    min_my = FLT_MAX;
    max_mx = -FLT_MAX;
    max_my = -FLT_MAX;
    is_get_area = false;
  }
  Eigen::Matrix<double, 3, 3> qr_offset;
};

}  // namespace mapping_and_location
}  // namespace data_process
}  // namespace gomros
