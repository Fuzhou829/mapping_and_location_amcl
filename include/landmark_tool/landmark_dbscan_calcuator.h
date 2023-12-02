/*
 * @Description: Copyright (C) 2023 山东亚历山大智能科技有限公司
 * @version: 1.2
 * @Author: renjy
 * @Data: 2023-06-12 09:34:11
 * @LastEditors: renjy
 * @LastEditTime: 2023-06-28 04:41:03
 */

#pragma once

#include <vector>



namespace LandMark {


enum DbscanState {
  UNCLASSIFIED = -1,
  CORE_POINT = 1,
  BORDER_POINT = 2,
  NOISE = -2,
  SUCCESS = 0,
  FAILURE_ = -3
};



struct DbscanPoint {
  double x;
  double y;
  int clusterID;  // clustered ID
};


class ReflectorDbscan {
 public:
  ReflectorDbscan(int minPts, float eps,
                  const std::vector<DbscanPoint>& points) {
      m_minPoints = minPts;
      m_epsilon = eps;
      m_points = points;
      m_pointSize = points.size();
  }
  ~ReflectorDbscan() {}

  std::vector<DbscanPoint> Run();
  std::vector<int> CalculateCluster(DbscanPoint point);
  int ExpandCluster(DbscanPoint point, int clusterID);

  int GetTotalPointSize() {
    return m_pointSize;
  }
  int getMinimumClusterSize() {
    return m_minPoints;
  }
  int getEpsilonSize() {
    return m_epsilon;
  }
  std::vector<DbscanPoint> get_points() {
    return m_points;
  }

 private:
  std::vector<DbscanPoint> m_points;
  int m_pointSize;
  int m_minPoints;
  float m_epsilon;
};

}  // namespace LandMark
