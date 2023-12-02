/*
 * @Description: Copyright (C) 2023 山东亚历山大智能科技有限公司
 * @version: 1.2
 * @Author: renjy
 * @Data: 2023-06-12 09:38:14
 * @LastEditors: renjy
 * @LastEditTime: 2023-06-29 11:13:29
 */

#include "landmark_tool/landmark_dbscan_calcuator.h"
#include "include/mapping_and_location_math.h"


namespace LandMark {




std::vector<DbscanPoint> ReflectorDbscan::Run() {
  int clusterID = 1;
  std::vector<DbscanPoint>::iterator iter;
  for (iter = m_points.begin(); iter != m_points.end(); ++iter) {
    if (iter->clusterID == UNCLASSIFIED) {
      if (ExpandCluster(*iter, clusterID) != FAILURE_) clusterID++;
    }
  }

  return m_points;
}

std::vector<int> ReflectorDbscan::CalculateCluster(DbscanPoint point) {
  int index = 0;
  std::vector<DbscanPoint>::iterator iter;
  std::vector<int> clusterIndex;
  for (iter = m_points.begin(); iter != m_points.end(); ++iter) {
      if (SLAMMath::Dist(point.x, point.y, iter->x, iter->y) <= m_epsilon) {
        clusterIndex.push_back(index);
      }
      index++;
  }
  return clusterIndex;
}

int ReflectorDbscan::ExpandCluster(DbscanPoint point, int clusterID) {
  // 找到该点邻域内的点
  std::vector<int> clusterSeeds = CalculateCluster(point);
  // 如果小于min则归于噪音点
  if (clusterSeeds.size() < m_minPoints) {
    point.clusterID = NOISE;
    return FAILURE_;
  }
  int index = 0, indexCorePoint = 0;
  std::vector<int>::iterator iterSeeds;
  for (iterSeeds = clusterSeeds.begin();
    iterSeeds != clusterSeeds.end(); ++iterSeeds) {
    m_points.at(*iterSeeds).clusterID = clusterID;
    if (m_points.at(*iterSeeds).x == point.x &&
        m_points.at(*iterSeeds).y == point.y) {
        indexCorePoint = index;
    }
    ++index;
  }

  clusterSeeds.erase(clusterSeeds.begin()+indexCorePoint);

  for (std::vector<int>::size_type i = 0, n = clusterSeeds.size(); i < n; ++i) {
    // 计算该点邻域内除自身外每个点的邻域的点
    std::vector<int> clusterNeighors =
      CalculateCluster(m_points.at(clusterSeeds[i]));
      if (clusterNeighors.size() >= m_minPoints) {
        std::vector<int>::iterator iterNeighors;
          for (iterNeighors = clusterNeighors.begin();
               iterNeighors != clusterNeighors.end(); ++iterNeighors) {
              if (m_points.at(*iterNeighors).clusterID == UNCLASSIFIED ||
                  m_points.at(*iterNeighors).clusterID == NOISE) {
                  if (m_points.at(*iterNeighors).clusterID == UNCLASSIFIED) {
                      clusterSeeds.push_back(*iterNeighors);
                      n = clusterSeeds.size();
                  }
                  m_points.at(*iterNeighors).clusterID = clusterID;
              }
          }
      }
  }

  return SUCCESS;
}

}  // namespace LandMark
