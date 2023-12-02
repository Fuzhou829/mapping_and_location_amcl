/*
 * @Descripttion: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @version: 1.0
 * @Author: renjy
 * @Date: 2023-05-06 15:32:57
 * @LastEditors: renjy
 * @LastEditTime: 2023-09-21 15:57:34
 */
#include "landmark_tool/trilateration.h"

namespace LandMark {

Trilateration::Trilateration(
  const MAL::LandMarkConfig& config)
: config_(config) {}

std::map<int, std::map<int, Eigen::Vector2d>>
  Trilateration::Transform(const std::vector<Eigen::Vector2d>& map_info) {
  std::map<int, std::map<int, Eigen::Vector2d>> result;
  std::map<int, Eigen::Vector2d> reflectors;
  for (int i = 0; i < map_info.size(); i++) {
    reflectors.insert(std::make_pair(i, map_info[i]));
  }
  result.insert(std::make_pair(-1, reflectors));
  return result;
}

void Trilateration::SetLandMarkMap(
  const std::map<int, std::map<int, Eigen::Vector2d>>& map_info) {
  octree_reflector_maps_.clear();
  land_mark_map_.clear();
  land_mark_map_ = map_info;
  match_id_.clear();
  _buildOctreeReflectorMap();
  match_map_id_ = -1;
}

bool Trilateration::IsGetGlobalPos(
  const std::vector<Eigen::Vector2d>& observation_info,
  Eigen::Vector3d* global_pos) {
  match_map_id_ = -1;
  if (observation_info.size() < 3) return false;
  if (!land_mark_map_.count(-1)) {
    SLAM_ERROR("there is no global map info, please check your map info");
    return false;
  }

  // 先在全局地图中进行匹配
  std::vector<int> global_id;
  if (!_getGlobalPosFromMap(-1, observation_info, global_pos, &global_id)) {
    SLAM_WARN("global location fail");
    return false;
  } else {
    // 检索应该转移到哪张子图 并在该子图中进行全局定位
    if (land_mark_map_.size() <= 2) return true;
    match_map_id_ = _matchMapId(global_id);
    if (match_map_id_ != -1) {
      Eigen::Vector3d last_global_pos = *global_pos;
      if (_getGlobalPosFromMap(match_map_id_, observation_info,
        global_pos, &global_id)) {
        SLAM_INFO("global pos %f %f %f, submap id %d pos %f %f %f",
          last_global_pos(0), last_global_pos(1), last_global_pos(2),
          match_map_id_, (*global_pos)(0), (*global_pos)(1), (*global_pos)(2));
      } else {
        match_map_id_ = -1;
      }
    }
    return true;
  }
}

void Trilateration::GetMatchId(std::map<int, int>* result) {
  // if (land_mark_map_.size() > 1) return;
  *result = match_id_;
  return;
}


void Trilateration::_findSimilarTriangles(
  float side_1, float side_2, float side_3, int map_id,
  std::vector<Octree::OctreePoint>* similar_triangles) {
  float error_threshold_for_loop = config_.error_threshold_for_loop;
  Octree::Sides3 side_max(
                side_1 + error_threshold_for_loop,
                side_2 + error_threshold_for_loop,
                side_3 + error_threshold_for_loop);
  Octree::Sides3 side_min(
                side_1 - error_threshold_for_loop,
                side_2 - error_threshold_for_loop,
                side_3 - error_threshold_for_loop);
  octree_reflector_maps_.at(map_id)->GetPointsInsideBox(
    side_min, side_max, similar_triangles);
  return;
}
void Trilateration::_checkMatchResult(const std::map<int, int>& match_id,
  const std::vector<Eigen::Vector2d>& observation_info,
  int map_id, std::vector<int>* global_id,
  std::vector<Eigen::Vector2d>* triangle_points,
  std::vector<Eigen::Vector2d>* observation_triangle_points) {
  std::map<int, int> map_count, obs_count;
  for (auto iter : match_id) {
    if (!map_count.count(iter.first)) {
      map_count.insert(std::make_pair(iter.first, 1));
    } else {
      map_count.at(iter.first)++;
    }
    if (!obs_count.count(iter.second)) {
      obs_count.insert(std::make_pair(iter.second, 1));
    } else {
      obs_count.at(iter.second)++;
    }
  }
  for (auto iter : match_id) {
    if (map_count.at(iter.first) != 1 || obs_count.at(iter.second) != 1)
      continue;
    global_id->push_back(iter.first);
    triangle_points->push_back(
            land_mark_map_.at(map_id).at(iter.first));
    observation_triangle_points->push_back(observation_info[iter.second]);
  }
  return;
}

bool Trilateration::_doLocation(
  const std::vector<Eigen::Vector2d>& map_triangle,
  const std::vector<Eigen::Vector2d>& observation_triangle,
  Eigen::Vector3d* global_pos) {
  assert(map_triangle.size() >= 3);
  assert(observation_triangle.size() >= 3);
  std::vector<Eigen::Vector2d> maps, obs;
  for (int a = 0; a < map_triangle.size(); a++) {
    // SLAM_INFO("%d map(%f %f), obs(%f %f)",
    //   a, map_triangle[a](0), map_triangle[a](1),
    //   observation_triangle[a](0), observation_triangle[a](1));
    maps.push_back(Eigen::Vector2d(map_triangle[a](0), map_triangle[a](1)));
    obs.push_back(Eigen::Vector2d(observation_triangle[a](0),
                                  observation_triangle[a](1)));
  }
  LandMark::TrilateralCal trilater_cal;
  return trilater_cal.GetGlobalPos(maps, obs, global_pos);
}




void Trilateration::_buildOctreeReflectorMap() {
  float half_dimension = 0.5 * config_.reflector_max_distance;
  Octree::Sides3 half_sides(half_dimension, half_dimension, half_dimension);
  for (auto iter : land_mark_map_) {
    int land_mark_size = static_cast<int>(iter.second.size());
    if (land_mark_size < 3) {
      SLAM_ERROR("mapid %d land mark %d is not enough",
        iter.first, land_mark_size);
    }
    std::shared_ptr<Octree::Map> octree_map =
      std::make_shared<Octree::Map>(half_sides, half_sides);
    float side_distance_ij, side_distance_ik, side_distance_jk;
    int real_triangle_count = 0;
    // 预计放入三角形个数
    int estimate_triangle_size =
      land_mark_size * (land_mark_size - 1) * (land_mark_size - 2) / 6;
    Octree::OctreePoint *octreePoints =
      new Octree::OctreePoint[estimate_triangle_size];

    for (auto i_iter : iter.second) {
      for (auto j_iter : iter.second) {
        if (i_iter.first >= j_iter.first) continue;
        side_distance_ij = SLAMMath::Dist(
                  i_iter.second(0), i_iter.second(1),
                  j_iter.second(0), j_iter.second(1));
        for (auto k_iter : iter.second) {
          if (i_iter.first >= k_iter.first ||
              j_iter.first >= k_iter.first) continue;
          side_distance_ik = SLAMMath::Dist(
                  i_iter.second(0), i_iter.second(1),
                  k_iter.second(0), k_iter.second(1));
          side_distance_jk = SLAMMath::Dist(
                  j_iter.second(0), j_iter.second(1),
                  k_iter.second(0), k_iter.second(1));
          // 判断是满足放入八叉树的条件
          if (!_checkTriangle(
              side_distance_ij, side_distance_ik, side_distance_jk))
              continue;
          Octree::Sides3 side(side_distance_ij,
                              side_distance_ik, side_distance_jk);
          std::vector<int> landmark_local_ids{
            i_iter.first, j_iter.first, k_iter.first};
          octreePoints[real_triangle_count].setValue(side, landmark_local_ids);
          octree_map->Insert(octreePoints + real_triangle_count);
          real_triangle_count++;
        }
      }
    }
    octree_reflector_maps_.insert(std::make_pair(iter.first, octree_map));
  }
  return;
}

bool Trilateration::_getGlobalPosFromMap(int map_id,
  const std::vector<Eigen::Vector2d>& observation_info,
  Eigen::Vector3d* global_pos, std::vector<int>* global_id) {
  int observation_info_size = static_cast<int>(observation_info.size());
  // 计算三边边长
  float side_distance_ij, side_distance_ik, side_distance_jk;
  std::map<int, int> match_id;
  // bool is_finish_location = false;
  for (int i = 0; i < observation_info_size; i++) {
    for (int j = 0; j < observation_info_size; j++) {
      if (i == j) continue;
      side_distance_ij = SLAMMath::Dist(observation_info[i](0),
                                        observation_info[i](1),
                                        observation_info[j](0),
                                        observation_info[j](1));
      for (int k = 0; k < observation_info_size; k++) {
        if (i == k || j == k) continue;
        side_distance_ik = SLAMMath::Dist(observation_info[i](0),
                                          observation_info[i](1),
                                          observation_info[k](0),
                                          observation_info[k](1));
        side_distance_jk = SLAMMath::Dist(observation_info[j](0),
                                          observation_info[j](1),
                                          observation_info[k](0),
                                          observation_info[k](1));
        if (!_checkTriangle(
            side_distance_ij, side_distance_ik, side_distance_jk))
            continue;
        // 三边定位
        // 寻找相似的三角形
        std::vector<Octree::OctreePoint> similar_triangles;
        _findSimilarTriangles(side_distance_ij, side_distance_ik,
                              side_distance_jk, map_id, &similar_triangles);
        if (similar_triangles.size() != 1) {
          // SLAM_DEBUG("find similar_triangles size %d, can not do location",
          //             similar_triangles.size());
          continue;
        }
        std::vector<int> landmark_ids = similar_triangles[0].getLandmarkids();

        if (!match_id.count(landmark_ids[0])) {
          match_id.insert(std::make_pair(landmark_ids[0], i));
        }
        if (!match_id.count(landmark_ids[1])) {
          match_id.insert(std::make_pair(landmark_ids[1], j));
        }
        if (!match_id.count(landmark_ids[2])) {
          match_id.insert(std::make_pair(landmark_ids[2], k));
        }
        // is_finish_location =
        //   _doLocation(triangle_points,
        //   observation_triangle_points, global_pos);
        // if (is_finish_location) {
        //   SLAM_DEBUG("find similar triangles, finish reflection location");
        //   break;
        // }
      }
      // if (is_finish_location) break;
    }
    // if (is_finish_location) break;
  }
  std::vector<Eigen::Vector2d> observation_triangle_points;
  std::vector<Eigen::Vector2d> triangle_points;
  // 检查是否出现同一个反光柱重复匹配的情况
  _checkMatchResult(match_id, observation_info, map_id, global_id,
                    &triangle_points, &observation_triangle_points);
  if (triangle_points.size() < 3 || observation_triangle_points.size() < 3)
    return false;
  match_id_.clear();
  match_id_ = match_id;
  return _doLocation(triangle_points, observation_triangle_points, global_pos);
}


int Trilateration::_matchMapId(const std::vector<int>& global_map_id) {
  std::map<int, int> match_id_count;
  for (auto iter : global_map_id) {
    for (auto submap_iter : land_mark_map_) {
      if (submap_iter.first == -1) continue;
      if (submap_iter.second.count(iter)) {
        if (match_id_count.count(submap_iter.first)) {
          match_id_count.at(submap_iter.first)++;
        } else {
          match_id_count.insert(std::make_pair(submap_iter.first, 1));
        }
      }
    }
  }
  int match_id = -1;
  int max_count = 2;
  for (auto iter : match_id_count) {
    if (iter.second > max_count) {
      max_count = iter.second;
      match_id = iter.first;
    }
  }
  return match_id;
}




bool Trilateration::_checkTriangle(
  float side_1, float side_2, float side_3) {
  float diff_threshold =
    config_.distance_diff_threshold;
  float distance_threshold =
    config_.reflector_max_distance;
  float theta_1 = std::acos(0.5 * (std::pow(side_1, 2) + std::pow(side_2, 2) -
                                   std::pow(side_3, 2)) / (side_1 * side_2));
  float theta_2 = std::acos(0.5 * (std::pow(side_1, 2) + std::pow(side_3, 2) -
                                   std::pow(side_2, 2)) / (side_1 * side_3));
  float theta_3 = std::acos(0.5 * (std::pow(side_2, 2) + std::pow(side_3, 2) -
                                   std::pow(side_1, 2)) / (side_2 * side_3));
  if (fabs(theta_1) < 0.09 || fabs(theta_2) < 0.09 || fabs(theta_3) < 0.09)
    return false;
  if (side_1 > distance_threshold) return false;
  if (side_2 > distance_threshold) return false;
  if (side_3 > distance_threshold) return false;
  if (fabs(side_1 - side_2) < diff_threshold) return false;
  if (fabs(side_1 - side_3) < diff_threshold) return false;
  if (fabs(side_2 - side_3) < diff_threshold) return false;
  return true;
}




}  // namespace LandMark
