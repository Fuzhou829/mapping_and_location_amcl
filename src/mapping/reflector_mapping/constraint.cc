/*
 * @Description: Copyright (C) 2023 山东亚历山大智能科技有限公司
 * @version: 1.2
 * @Author: renjy
 * @Data: 2023-06-04 08:27:09
 * @LastEditors: renjy
 * @LastEditTime: 2023-08-02 10:34:24
 */

#include <cfloat>

#include "reflector_mapping/constraint.h"
#include "common/logger.h"

namespace gomros {
namespace data_process {
namespace mapping_and_location {



bool SubMapConstraint::ComputeSubMapConstraint(
  const std::map<int, std::vector<Eigen::Vector2d>>& reflectors_in_submaps,
  const std::map<int, Eigen::Vector3d>& submap_global_pos,
  const std::vector<std::shared_ptr<Submap>>& finished_submap) {
  assert(submap_global_pos.size() == reflectors_in_submaps.size());
  assert(reflectors_in_submaps.size() == finished_submap.size());
  // constraints_.clear();
  int submap_size = submap_global_pos.size();
  if (submap_size < 2) return false;
  bool is_get_loop = false;
  const Eigen::Vector3d last_submap_global_pos =
    submap_global_pos.at(submap_size - 1);
  const std::vector<Eigen::Vector2d> reflectors_in_last_submap =
    reflectors_in_submaps.at(submap_size - 1);
  std::shared_ptr<Submap> new_submap = finished_submap.back();
  for (int i = submap_size - 2; i >= 0; i--) {
    const Eigen::Vector3d current_submap_global_pos = submap_global_pos.at(i);
    const std::vector<Eigen::Vector2d> reflectors_in_current_submap =
      reflectors_in_submaps.at(i);
    double distance =
          SLAMMath::Dist(last_submap_global_pos(0), last_submap_global_pos(1),
                  current_submap_global_pos(0), current_submap_global_pos(1));

    if (i > submap_size - 3 && distance <
        config_.land_mark_config.distance_for_constraint) {
      _calContraintForTimenearSubmap(submap_size - 1, i,
          last_submap_global_pos, current_submap_global_pos,
          reflectors_in_last_submap, reflectors_in_current_submap);
          // is_get_loop = true;
      } else if (distance < config_.land_mark_config.distance_for_loop) {
        if (_isGetConstraintForLoopSubMap(new_submap, finished_submap[i])) {
          is_get_loop = true;
        }
      }
  }
  SLAM_INFO("is find loop %d", is_get_loop);
  return is_get_loop;
}

bool SubMapConstraint::_calContraintForTimenearSubmap(
  int new_submap_id, int current_submap_id,
  const Eigen::Vector3d& new_submap_pos,
  const Eigen::Vector3d& current_submap_pos,
  const std::vector<Eigen::Vector2d>& new_submap_reflectors,
  const std::vector<Eigen::Vector2d>& current_submap_reflectors) {
  std::vector<std::pair<int, int>> initial_association_result;
  _getInitialAssociationResult(new_submap_pos, current_submap_pos,
                              new_submap_reflectors, current_submap_reflectors,
                              &initial_association_result);
  if (initial_association_result.size() < 3) {
    for (int i = 0; i < initial_association_result.size(); i++) {
      constraints_.push_back(
        Constraint(new_submap_id, initial_association_result[i].first,
              current_submap_id, initial_association_result[i].second,
              config_.land_mark_config.error_threshold_for_loop));
    }
    return true;
  }

  // 对齐之后。再进行匹配
  std::vector<Eigen::Vector2d> map_triangle;
  std::vector<Eigen::Vector2d> observation_triangle;
  for (int i = 0; i < initial_association_result.size(); i++) {
    observation_triangle.push_back(
      new_submap_reflectors[initial_association_result[i].first]);
    map_triangle.push_back(
      current_submap_reflectors[initial_association_result[i].second]);
  }

  LandMark::TrilateralCal trilater_cal;
  Eigen::Vector3d result;
  if (trilater_cal.GetGlobalPos(map_triangle, observation_triangle, &result)) {
    // 计算出新的 new_submap_pos
    initial_association_result.clear();
    Eigen::Vector3d correct_pos;
    correct_pos(0) = result(0) * std::cos(current_submap_pos(2)) -
                     result(1) * std::sin(current_submap_pos(2)) +
                     current_submap_pos(0);
    correct_pos(1) = result(0) * std::sin(current_submap_pos(2)) +
                     result(1) * std::cos(current_submap_pos(2)) +
                     current_submap_pos(1);
    correct_pos(2) = current_submap_pos(2) + result(2);
    correct_pos(2) = SLAMMath::NormalizePITheta(correct_pos(2));
    SLAM_INFO("old new_submap_pos (%f %f %f), now new_submap_pos (%f %f %f)",
              new_submap_pos(0), new_submap_pos(1), new_submap_pos(2),
              correct_pos(0), correct_pos(1), correct_pos(2));
    _getInitialAssociationResult(correct_pos, current_submap_pos,
                              new_submap_reflectors, current_submap_reflectors,
                              &initial_association_result);
    for (int i = 0; i < initial_association_result.size(); i++) {
      constraints_.push_back(
        Constraint(new_submap_id, initial_association_result[i].first,
              current_submap_id, initial_association_result[i].second,
              config_.land_mark_config.error_threshold_for_loop));
    }
  }


  // LandMark::Trilateration trilateration(config_.land_mark_config);
  // std::vector<Eigen::Vector2d> maps;
  // for (int i = 0; i < current_submap_reflectors.size(); i++) {
  //   maps.push_back(_local2Global(current_submap_reflectors[i],
  //                                 current_submap_pos));
  // }


  // trilateration.SetLandMarkMap(maps);
  // Eigen::Vector3d corrected_pos;
  // if (trilateration.IsGetGlobalPos(new_submap_reflectors, &corrected_pos)) {
  //   SLAM_INFO("new_submap_pos %f %f %f, corrected_pos %f %f %f",
  //             new_submap_pos(0), new_submap_pos(1), new_submap_pos(2),
  //             corrected_pos(0), corrected_pos(1), corrected_pos(2));
  //   // std::vector<std::tuple<int, int>> result;
  //   // trilateration.GetMatchId(&result);
  //   // for (auto iter : result) {
  //   //   current_submap_reflectors[iter->first];
  //   // }

  //   _getContinstrint(new_submap_id, current_submap_id,
  //                    corrected_pos, current_submap_pos,
  //             new_submap_reflectors, current_submap_reflectors);
  //   return true;
  // }
  return false;
}

bool SubMapConstraint::_isGetConstraintForLoopSubMap(
  const std::shared_ptr<Submap>& new_submap,
  const std::shared_ptr<Submap>& current_submap) {
  bool is_get_loop = false;
  // 利用三边定位进行
  int new_submap_id = new_submap->GetSubMapId();
  int current_submap_id = current_submap->GetSubMapId();
  const Eigen::Vector3d new_submap_pos = new_submap->GetGlobalPose().first;

  const std::vector<Eigen::Vector2d> new_submap_reflectors =
    new_submap->GetReflectorInSubMap().reflectors_;

  // 获取当前submap的八叉树
  std::shared_ptr<Octree::Map> current_octree =
    current_submap->GetOctreeMap();

  for (int i = 0; i < new_submap_reflectors.size(); i++) {
    for (int j = 0 ; j < new_submap_reflectors.size(); j++) {
      if (i == j) continue;
      for (int k = 0; k < new_submap_reflectors.size(); k++) {
        if (k == i || k == j) continue;
        double side_length_ij = SLAMMath::Dist(
                new_submap_reflectors[i](0), new_submap_reflectors[i](1),
                new_submap_reflectors[j](0), new_submap_reflectors[j](1));
        double side_length_ik = SLAMMath::Dist(
                new_submap_reflectors[i](0), new_submap_reflectors[i](1),
                new_submap_reflectors[k](0), new_submap_reflectors[k](1));
        double side_length_kj = SLAMMath::Dist(
                new_submap_reflectors[k](0), new_submap_reflectors[k](1),
                new_submap_reflectors[j](0), new_submap_reflectors[j](1));
        std::vector<Octree::OctreePoint> similar_triangles;
        _findSimilarTriangles(side_length_ij, side_length_ik, side_length_kj,
                              current_octree, &similar_triangles);
        if (similar_triangles.size() != 1) continue;
        std::vector<int> obs_id{i, j, k};
        SLAM_DEBUG("obs (%f %f) (%f %f) (%f %f)",
                    new_submap_reflectors[i](0),
                    new_submap_reflectors[i](1),
                    new_submap_reflectors[j](0),
                    new_submap_reflectors[j](1),
                    new_submap_reflectors[k](0),
                    new_submap_reflectors[k](1));
        if (_isLoop(obs_id, new_submap, similar_triangles, current_submap)) {
          is_get_loop = true;
        }
      }
    }
  }

  return is_get_loop;
}


Eigen::Vector2d SubMapConstraint::_local2Global(
  const Eigen::Vector2d &localcenter,
  const Eigen::Vector3d &submap_global_pos) {
  double x =
    localcenter(0) * std::cos(submap_global_pos(2)) -
    localcenter(1) * std::sin(submap_global_pos(2)) + submap_global_pos(0);
  double y =
    localcenter(0) * std::sin(submap_global_pos(2)) +
    localcenter(1) * std::cos(submap_global_pos(2)) + submap_global_pos(1);
  return Eigen::Vector2d(x, y);
}


void SubMapConstraint::_getInitialAssociationResult(
  const Eigen::Vector3d& new_submap_pos,
  const Eigen::Vector3d& current_submap_pos,
  const std::vector<Eigen::Vector2d>& new_submap_reflectors,
  const std::vector<Eigen::Vector2d>& current_submap_reflectors,
  std::vector<std::pair<int, int>>* initial_association_result) {
  float theta_theshold = 1.0 * M_PI / 180.0;

  for (int i = 0; i < new_submap_reflectors.size(); i++) {
    float distance_threshold =
        config_.land_mark_config.distance_for_same_reflector +
        std::sin(theta_theshold) * 2 * SLAMMath::Dist(
        new_submap_reflectors.at(i)(0),
        new_submap_reflectors.at(i)(1), 0.0, 0.0);
    // float distance_threshold =
    //   config_.land_mark_config.distance_for_same_reflector;
    const Eigen::Vector2d global_start =
        _local2Global(new_submap_reflectors[i], new_submap_pos);
    int best_match_in_current = -1;
    float min_distance = FLT_MAX;
    for (int j = 0; j < current_submap_reflectors.size(); j++) {
      const Eigen::Vector2d global_end =
        _local2Global(current_submap_reflectors[j], current_submap_pos);
      float distance = SLAMMath::Dist(global_start(0), global_start(1),
                                      global_end(0), global_end(1));
      if (distance < distance_threshold && distance < min_distance) {
          min_distance = distance;
          best_match_in_current = j;
      }
    }
    if (best_match_in_current != -1) {
      initial_association_result->push_back({i, best_match_in_current});
    }
  }
  return;
}

void SubMapConstraint::_getContinstrint(
  int new_submap_id, int current_submap_id,
  const Eigen::Vector3d& new_submap_pos,
  const Eigen::Vector3d& current_submap_pos,
  const std::vector<Eigen::Vector2d>& new_submap_reflectors,
  const std::vector<Eigen::Vector2d>& current_submap_reflector) {
  float theta_theshold = 1.0 * M_PI / 180.0;

  for (int i = 0; i < new_submap_reflectors.size(); i++) {
    float distance_threshold =
          config_.land_mark_config.distance_for_same_reflector +
          std::sin(theta_theshold) * 2 * SLAMMath::Dist(
        new_submap_reflectors.at(i)(0),
        new_submap_reflectors.at(i)(1), 0.0, 0.0);
    // float distance_threshold =
    //       config_.land_mark_config.distance_for_same_reflector;
    const Eigen::Vector2d global_start =
        _local2Global(new_submap_reflectors[i], new_submap_pos);
    int best_match_in_current = -1;
    float min_distance = FLT_MAX;
    for (int j = 0; j < current_submap_reflector.size(); j++) {
      const Eigen::Vector2d global_end =
        _local2Global(current_submap_reflector[j], current_submap_pos);
      float distance = SLAMMath::Dist(global_start(0), global_start(1),
                                      global_end(0), global_end(1));
      if (distance < distance_threshold && distance < min_distance) {
        min_distance = distance;
        best_match_in_current = j;
      }
    }
    if (best_match_in_current != -1) {
      _checkBestConstraints(Constraint(new_submap_id, i,
                    current_submap_id, best_match_in_current, min_distance));
    }
  }
  return;
}

void SubMapConstraint::_findSimilarTriangles(
  double side_1, double side_2, double side_3,
  std::shared_ptr<Octree::Map> octree,
  std::vector<Octree::OctreePoint>* similar_triangles) {
  float error_threshold_for_loop = 0.1;
  Octree::Sides3 side_max(
                side_1 + error_threshold_for_loop,
                side_2 + error_threshold_for_loop,
                side_3 + error_threshold_for_loop);
  Octree::Sides3 side_min(
                side_1 - error_threshold_for_loop,
                side_2 - error_threshold_for_loop,
                side_3 - error_threshold_for_loop);
  octree->GetPointsInsideBox(
    side_min, side_max, similar_triangles);
  return;
}

bool SubMapConstraint::_isLoop(const std::vector<int>& obs_id,
  const std::shared_ptr<Submap>& new_submap,
  const std::vector<Octree::OctreePoint>& similar_triangles,
  const std::shared_ptr<Submap>& current_submap) {
  const std::vector<Eigen::Vector2d> new_submap_reflectors =
    new_submap->GetReflectorInSubMap().reflectors_;
  const std::vector<Eigen::Vector2d> current_submap_reflectors =
    current_submap->GetReflectorInSubMap().reflectors_;
  std::vector<Eigen::Vector2d> maps;
  const Eigen::Vector3d current_submap_pos =
    current_submap->GetGlobalPose().first;
  const Eigen::Vector3d new_submap_pos =
    new_submap->GetGlobalPose().first;
  std::vector<Eigen::Vector2d> obs;
  for (int i = 0; i < obs_id.size(); i++) {
    // obs.push_back(_local2Global(new_submap_reflectors[obs_id[i]],
    //                             new_submap_pos));
    obs.push_back(new_submap_reflectors.at(obs_id[i]));
    SLAM_DEBUG("obs i %d (%f %f)", i,
                new_submap_reflectors.at(obs_id[i])(0),
                new_submap_reflectors.at(obs_id[i])(1));
  }

  for (auto m : similar_triangles[0].getLandmarkids()) {
    maps.push_back(_local2Global(current_submap_reflectors.at(m),
                                 current_submap_pos));
  }
  LandMark::TrilateralCal trilateral_cal;
  Eigen::Vector3d corrected_new_submap_pos;
  if (!trilateral_cal.GetGlobalPos(maps, obs, &corrected_new_submap_pos))
    return false;
  float verify_triangle_threhold_dis = 0.1;
  float verify_triangle_threhold_theta = 0.1;

  float delta_x = fabs(corrected_new_submap_pos(0) - new_submap_pos(0));
  float delta_y = fabs(corrected_new_submap_pos(1) - new_submap_pos(1));
  float delta_theta = fabs(corrected_new_submap_pos(2) - new_submap_pos(2));

  SLAM_DEBUG("change delta (%f %f %f)", delta_x, delta_y, delta_theta);

  if (delta_x > verify_triangle_threhold_dis ||
      delta_y > verify_triangle_threhold_dis ||
      delta_theta > verify_triangle_threhold_theta)  return false;
  const std::vector<Eigen::Vector2d> current_submap_global_reflectors =
                    current_submap->GetReflectorInGlobal().reflectors_;
  int start_submap_id = new_submap->GetSubMapId();
  int end_submap_id = current_submap->GetSubMapId();
  bool is_have_extra = false;
  for (int i = 0; i < new_submap_reflectors.size(); i++) {
    float min_dis = FLT_MAX;
    int best_match_id = -1;
    Eigen::Vector2d new_reflector_global =
      _local2Global(new_submap_reflectors[i], corrected_new_submap_pos);
    for (int j = 0; j < current_submap_global_reflectors.size(); j++) {
      double dis = SLAMMath::Dist(
                    new_reflector_global(0), new_reflector_global(1),
                    current_submap_global_reflectors[j](0),
                    current_submap_global_reflectors[j](1));
      if (dis < min_dis &&
          dis < config_.land_mark_config.error_threshold_for_loop) {
        best_match_id = j;
        min_dis = dis;
      }
    }
    if (best_match_id != -1) {
      is_have_extra = true;
      _checkBestConstraints(Constraint(start_submap_id, i,
                              end_submap_id, best_match_id, min_dis));
    }
  }
  return is_have_extra;
}


bool SubMapConstraint::_checkBestConstraints(const Constraint& constraint) {
  for (int i = 0; i < constraints_.size(); i++) {
    if (constraint.start_submap_id == constraints_[i].start_submap_id &&
        constraint.end_submap_id == constraints_[i].end_submap_id &&
        constraint.end_reflector_id == constraints_[i].end_reflector_id &&
        constraint.distance < constraints_[i].distance) {
        constraints_[i].start_reflector_id = constraint.start_reflector_id;
        constraints_[i].distance = constraint.distance;
        return true;
    }
  }
  constraints_.push_back(constraint);
  return false;
}





}  // namespace mapping_and_location
}  // namespace data_process
}  // namespace gomros
