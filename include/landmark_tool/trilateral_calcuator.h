/*
 * @Description: Copyright (C) 2023 山东亚历山大智能科技有限公司
 * @version: 1.2
 * @Author: renjy
 * @Data: 2023-06-07 03:02:15
 * @LastEditors: renjy
 * @LastEditTime: 2023-07-22 09:02:18
 */

#pragma once
#include <vector>

#include "Eigen/Dense"


namespace LandMark {
/**
 * @describes: 三边定位
 * @return {*}
 */

class TrilateralCal {
 public:
  TrilateralCal() {}
  virtual ~TrilateralCal() {}
  /**
   * @describes: 
   * @param map_triangle 地图中三边信息
   * @param observation_triangle 观测的三边信息
   * @param result 三边定位结果
   * @return {*}
   */  
  bool GetGlobalPos(const std::vector<Eigen::Vector2d>& map_triangle,
                    const std::vector<Eigen::Vector2d>& observation_triangle,
                    Eigen::Vector3d* result);
};


}  // namespace LandMark





