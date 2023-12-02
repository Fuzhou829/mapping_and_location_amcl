/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.0
 * @Date: 2022-08-05 17:09:35
 * @LastEditTime: 2023-09-21 15:32:21
 * @Author: renjy
 */

#include <float.h>

#include "reflector_mapping/trilateral_mapping.h"
#include "ekf_calcuator/reflector_ekf.h"
#include "landmark_tool/trilateration.h"
#include "include/mapping_and_location_math.h"
#include "common/tool.h"
#include "common/logger.h"

namespace gomros {
namespace data_process {
namespace mapping_and_location {

MergeReflectors::MergeReflectors(const MappingConfig& config) {
  mapping_config_ = config;
  sub_maps_.clear();
  actual_sub_maps_.clear();
  global_pose_.clear();
  constraint_.clear();
}

void MergeReflectors::SetReflectorMapInfo(
  const std::vector<Eigen::Vector2d>& map) {
  if (map.size() < 3) return;
  SubmapGlobalPose pos;
  if (sub_maps_.empty()) {
    SLAM_DEBUG("push fist submap~~~");
    sub_maps_.insert(std::make_pair(sub_maps_.size(), map));
    actual_sub_maps_.insert(std::make_pair(actual_sub_maps_.size(), map));
    pos.is_get_global_pose = true;
    global_pose_.insert(std::make_pair(global_pose_.size(), pos));
  } else {
    // 遍历寻找约束
    if (_isLoop(map, &pos)) {
      pos.is_get_global_pose = true;
      global_pose_.insert(std::make_pair(global_pose_.size(), pos));
      actual_sub_maps_.insert(std::make_pair(actual_sub_maps_.size(), map));
      sub_maps_.insert(std::make_pair(sub_maps_.size(), map));
      _runOptimizer();
    } else {
      pos.is_get_global_pose = false;
      global_pose_.insert(std::make_pair(global_pose_.size(), pos));
      sub_maps_.insert(std::make_pair(sub_maps_.size(), map));
      actual_sub_maps_.insert(std::make_pair(actual_sub_maps_.size(), map));
    }
  }
}

bool MergeReflectors::GetMapResult(
  std::map<int, std::map<int, Eigen::Vector2d>>* result) {
  if (sub_maps_.empty()) return false;
  // 最后统一判断一下未有全局坐标的submap
  if (_refleashAllSubmap()) {
    _runOptimizer();
  }
  // 保存总图信息
  std::map<int, Eigen::Vector2d> global_map;
  std::vector<Eigen::Vector2d> map_temp;
  _calGlobalMap(&map_temp);
  for (int i = 0; i < map_temp.size(); i++) {
    global_map.insert(std::make_pair(i, map_temp[i]));
    SLAM_INFO("reflector_global_pos %d %f %f",
              i, map_temp[i](0), map_temp[i](1));
  }

  result->insert(std::make_pair(-1, global_map));
  // 保存子图以及对应的全局地图中的id
  LandMark::Trilateration trilateration(mapping_config_.land_mark_config);
  trilateration.SetLandMarkMap(*result);
  for (auto submap_iter : actual_sub_maps_) {
    if (!global_pose_.at(submap_iter.first).is_get_global_pose) continue;
    Eigen::Vector3d global_pos_temp;
    if (!trilateration.IsGetGlobalPos(submap_iter.second, &global_pos_temp))
      continue;
    std::map<int, int> match_id;
    trilateration.GetMatchId(&match_id);
    std::array<double, 3> global_pos =
      global_pose_.at(submap_iter.first).pose_info;
    std::map<int, Eigen::Vector2d> sub_maps;
    for (int i = 0; i < submap_iter.second.size(); i++) {
      int id_in_global = -1;
      for (auto iter_match : match_id) {
        if (iter_match.second == i) {
          id_in_global = iter_match.first;
          break;
        }
      }
      std::array<double, 3> global_center =
        _local2Global(Eigen::Vector3d(submap_iter.second[i](0),
                      submap_iter.second[i](1), 0.0), global_pos);
      sub_maps.insert(std::make_pair(id_in_global,
        Eigen::Vector2d(global_center[0], global_center[1])));
      SLAM_INFO("submap %d, global_pos %d %f %f",
                submap_iter.first, id_in_global,
                global_center[0], global_center[1]);
    }
    result->insert(std::make_pair(submap_iter.first, sub_maps));
  }
  return true;
}

bool MergeReflectors::_isLoop(const std::vector<Eigen::Vector2d>& map,
  SubmapGlobalPose* global_pose) {
  int new_submap_id = sub_maps_.size();
  bool is_loop = false;
  LandMark::Trilateration trilateration(mapping_config_.land_mark_config);
  for (int i = 0; i < sub_maps_.size(); i++) {
    // 转换数据保存格式
    trilateration.SetLandMarkMap(trilateration.Transform(sub_maps_.at(i)));
    Eigen::Vector3d pos_in_submap;
    if (!global_pose_.at(i).is_get_global_pose) continue;
    if (trilateration.IsGetGlobalPos(map, &pos_in_submap)) {
      global_pose->pose_info = _local2Global(pos_in_submap,
                                global_pose_.at(i).pose_info);
      std::map<int, int> result;
      trilateration.GetMatchId(&result);
      is_loop = true;
      for (auto iter : result) {
        Constraint constraint(new_submap_id, iter.second, i, iter.first, 0.0);
        constraint.in_end_map_pos = pos_in_submap;
        // SLAM_DEBUG("add constraint %d %d %d %d %f %f %f",
        //            new_submap_id, iter.second, i, iter.first,
        //            pos_in_submap(0), pos_in_submap(1), pos_in_submap(2));
        constraint_.push_back(constraint);
      }
    }
  }
  return is_loop;
}

std::array<double, 3> MergeReflectors::_local2Global(
  const Eigen::Vector3d& local, const std::array<double, 3>& global) {
  std::array<double, 3> reslut;
  reslut[0] = local(0) * std::cos(global[2]) -
              local[1] * std::sin(global[2]) + global[0];
  reslut[1] = local(0) * std::sin(global[2]) +
              local[1] * std::cos(global[2]) + global[1];
  reslut[2] = global[2] + local[2];
  reslut[2] = SLAMMath::NormalizePITheta(reslut[2]);
  return reslut;
}

bool MergeReflectors::_refleashAllSubmap() {
  bool is_get_map = false;
  return is_get_map;
}

void MergeReflectors::_calGlobalMap(
  std::vector<Eigen::Vector2d>* global_map) {
  std::vector<LandMark::DbscanPoint> unclustered_centers;
  for (auto submap_iter : sub_maps_) {
    bool is_get_global_pose =
          global_pose_.at(submap_iter.first).is_get_global_pose;
    SLAM_DEBUG("record %d submap %d size %d",
              is_get_global_pose, submap_iter.first, submap_iter.second.size());
    if (!is_get_global_pose) continue;
    // 转换至全局坐标
    std::array<double, 3> global_pos =
      global_pose_.at(submap_iter.first).pose_info;
    for (int i = 0; i < submap_iter.second.size(); i++) {
      std::array<double, 3> global_center =
        _local2Global(Eigen::Vector3d(submap_iter.second[i](0),
                      submap_iter.second[i](1), 0.0), global_pos);
      LandMark::DbscanPoint tmp;
      tmp.x = global_center[0];
      tmp.y = global_center[1];
      tmp.clusterID = LandMark::UNCLASSIFIED;
      unclustered_centers.push_back(tmp);
    }
  }

  int landmark_id = 0;

  int minPts = 2;
  float eps = mapping_config_.land_mark_config.landmark_radius * 4;
  LandMark::ReflectorDbscan dbscan(minPts, eps, unclustered_centers);
  std::vector<LandMark::DbscanPoint> middle = dbscan.Run();  // 存在复制数据较多

  std::unordered_map<int, std::vector<Eigen::Vector2d>> clustered_centers;
  for (auto point : middle) {
    if (point.clusterID > 0) {
      clustered_centers[point.clusterID].push_back(
        Eigen::Vector2d(point.x, point.y));
    } else {
      // single_centers.push_back(tmp);
      global_map->push_back(Eigen::Vector2d(point.x, point.y));
    }
  }
  for (const auto &cluster : clustered_centers) {
    Eigen::Vector2d tmp_pose(0, 0);
    int n = cluster.second.size();
    for (const auto &center : cluster.second) {
      tmp_pose(0) += center(0) / n;
      tmp_pose(1) += center(1) / n;
    }
    global_map->push_back(Eigen::Vector2d(tmp_pose(0), tmp_pose(1)));
  }
  return;
}

void MergeReflectors::_runOptimizer() {
  // 转换至ceres需要的数据类型
  ceres::Problem problem;
  std::map<int, std::array<double, 3>> submap_pos;
  std::map<int, std::vector<std::array<double, 2>>> submap_landmarks;

  // 将需要优化的子图位姿设置为优化参数
  for (const auto &submap_id_pose : global_pose_) {
    // 将子图的global_pose放入submap_pos中
    submap_pos[submap_id_pose.first] =
      std::array<double, 3>({submap_id_pose.second.pose_info[0],
                             submap_id_pose.second.pose_info[1],
                             submap_id_pose.second.pose_info[2]});
    // c++11: std::array::data() 返回指向数组对象中第一个元素的指针
    // 添加需要优化的数据 这里显式添加参数块,会进行额外的参数块正确性检查
    problem.AddParameterBlock(submap_pos.at(submap_id_pose.first).data(), 3);

    if (submap_id_pose.first == 0 ||
        !submap_id_pose.second.is_get_global_pose) {
      // 如果是第一幅子图, 或者不具备全局坐标信息 不优化这个子图位姿
      problem.SetParameterBlockConstant(
        submap_pos.at(submap_id_pose.first).data());
    }
  }

  for (const auto &landmark_id_poses : sub_maps_) {
    std::vector<Eigen::Vector2d> center = landmark_id_poses.second;
    bool is_get_global_pose =
      global_pose_.at(landmark_id_poses.first).is_get_global_pose;
    for (int id = 0; id < center.size(); id++) {
      submap_landmarks[landmark_id_poses.first].push_back(
        std::array<double, 2>({center[id](0), center[id](1)}));
      problem.AddParameterBlock(
        submap_landmarks.at(landmark_id_poses.first)[id].data(), 2);
      if (!is_get_global_pose) {
        // 不具备全局坐标信息 不优化这个子图信息
        problem.SetParameterBlockConstant(
            submap_landmarks.at(landmark_id_poses.first)[id].data());
      }
    }
  }

  // 第一种约束 ： 反光柱局部位置和初始位姿
  for (const auto &landmark_id_poses : submap_landmarks) {
    const auto &landmarkcenters = landmark_id_poses.second;
    for (unsigned int id = 0; id < landmarkcenters.size(); id++) {
      ceres::CostFunction *cost_function =
          new ceres::AutoDiffCostFunction<LocalcentorCostFunction, 2, 2>(
              new LocalcentorCostFunction(
                submap_landmarks.at(landmark_id_poses.first).at(id)));
      // 设置残差项
      problem.AddResidualBlock(cost_function, NULL,
        submap_landmarks.at(landmark_id_poses.first).at(id).data());
    }
  }
  // 第二种约束 ：子图的全局位姿和构成回环子图得到的全局位姿的约束
  // 第三种约束 ：同一个反光柱在不同子图中的位置
  for (const auto &constraint : constraint_) {
    std::array<double, 3> in_end_map_pos{constraint.in_end_map_pos(0),
                                        constraint.in_end_map_pos(1),
                                        constraint.in_end_map_pos(2)};
    ceres::CostFunction *cost_function_1 =
    new ceres::AutoDiffCostFunction<SubmapConstraintCostFunction, 3, 3, 3>(
          new SubmapConstraintCostFunction(in_end_map_pos));
    problem.AddResidualBlock(cost_function_1, NULL,
            submap_pos.at(constraint.start_submap_id).data(),
            submap_pos.at(constraint.end_submap_id).data());

    ceres::CostFunction *cost_function_2 =
    new ceres::AutoDiffCostFunction<LandmarkCostFunction, 2, 3, 3, 2, 2>(
          new LandmarkCostFunction());
    // 设置残差项
    problem.AddResidualBlock(cost_function_2, NULL,
      submap_pos.at(constraint.start_submap_id).data(),
      submap_pos.at(constraint.end_submap_id).data(),
      submap_landmarks.at(constraint.start_submap_id).
      at(constraint.start_reflector_id).data(),
      submap_landmarks.at(constraint.end_submap_id).
      at(constraint.end_reflector_id).data());
  }

  // 处理问题
  ceres::Solver::Options options;
  options.max_num_iterations = 200;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  // 更新优化后的位姿信息
  for (const auto &C_submap_id_data : submap_pos) {
    if (!global_pose_.at(C_submap_id_data.first).is_get_global_pose) continue;
    SLAM_DEBUG("id %d used %f %f %f now %f %f %f", C_submap_id_data.first,
              global_pose_.at(C_submap_id_data.first).pose_info[0],
              global_pose_.at(C_submap_id_data.first).pose_info[1],
              global_pose_.at(C_submap_id_data.first).pose_info[2],
              C_submap_id_data.second[0],
              C_submap_id_data.second[1],
              C_submap_id_data.second[2]);
    global_pose_.at(C_submap_id_data.first).pose_info =
      C_submap_id_data.second;
  }
  // 更新优化后的反光柱信息
  // for (const auto &landmarks : submap_landmarks) {
  //   for (unsigned int i = 0; i < landmarks.second.size(); i++) {
  //     sub_maps_[landmarks.first][i][0] = landmarks.second[i][0];
  //     sub_maps_[landmarks.first][i][1] = landmarks.second[i][1];
  //   }
  // }
  return;
}

TrilateralMapping::TrilateralMapping(const MappingConfig& config)
: MappingInterface(config.record_config) {
  mapping_config_ = config;
  SetMappingConfig(mapping_config_);
}

void TrilateralMapping::StartMapping() {
  sensor_data_recorder_->StartRecord();
}

void TrilateralMapping::StopMapping(const std::string& map_name) {
  sensor_data_recorder_->FinishRecord();
  std::ifstream ladar_file;
  ladar_file.open(sensor_data_recorder_->GetLadarFileDir());
  std::string ladar_line;
  int count = 0;
  MergeReflectors merge_reflector_map(mapping_config_);
  SensorMount sensor_mount;
  sensor_mount.radar_position_x = 0;
  sensor_mount.radar_position_y = 0;
  sensor_mount.radar_position_theta = 0;
  while (getline(ladar_file, ladar_line)) {
    // if (count++ > 15) break;  // 单次建图数据不超过15帧数据
    std::vector<std::string> ladar_data =
      internal_common::SplitCString(ladar_line, " ");
    RadarSensoryInfo ladar_message;
    _transform2RadarSensoryInfo(ladar_data, &ladar_message);
    DataFusion::Observation obs;
    obs.GetObsFromLadarData(sensor_mount,
        mapping_config_.land_mark_config, ladar_message);
    SLAM_INFO("obs.centers size %d", obs.centers.size());
    if (obs.centers.size() < 3) continue;
    merge_reflector_map.SetReflectorMapInfo(obs.centers);
    // 拿到一帧具有三个反光柱的就退出建图
    break;
  }

  std::map<int, std::map<int, Eigen::Vector2d>> result;
  if (!merge_reflector_map.GetMapResult(&result)) {
    SLAM_ERROR("there is no reflector, please check!!!!!");
    return;
  }
  std::string full_name = mapping_config_.map_data_file_path + "/" +
                      internal_common::CreatedMapFileName(map_name);
  _saveToFile(result, full_name);

  return;
}


void TrilateralMapping::_transform2RadarSensoryInfo(
  const std::vector<std::string>& ladar_str, RadarSensoryInfo* ladar_message) {
  ladar_message->mstruRadarHeaderData.mlTimeStamp = atof(ladar_str[0].c_str());
  ladar_message->mstruSingleLayerData.mvPoints.clear();
  ladar_message->mstruSingleLayerData.mvIntensities.clear();
  float theta_acc = mapping_config_.mapping_end_angle -
                    mapping_config_.mapping_start_angle;
  int record_count = theta_acc / mapping_config_.laser_resolution + 1;
  int read_id = 2;
  float xp, yp;
  float theta = mapping_config_.mapping_start_angle * M_PI / 180.0f;
  float delta_theta = mapping_config_.laser_resolution * M_PI / 180.0f;
  for (int i = 0; i < record_count; i++) {
    float len = atof(ladar_str[read_id++].c_str());
    xp = len * cos(theta);
    yp = len * sin(theta);
    theta += delta_theta;
    Eigen::Vector2d temp(xp, yp);
    ladar_message->mstruSingleLayerData.mvPoints.push_back(temp);
    ladar_message->mstruSingleLayerData.mvIntensities.push_back(
                                          atof(ladar_str[read_id++].c_str()));
  }
  return;
}

void TrilateralMapping::_saveToFile(
  const std::map<int, std::map<int, Eigen::Vector2d>>& result,
  const std::string& map_name) {
  float min_x = FLT_MAX;
  float min_y = FLT_MAX;
  float max_x = -FLT_MAX;
  float max_y = -FLT_MAX;

  Json::Value map_json;
  Json::Value map_header;
  Json::Value atring_add;
  map_header["mapType"] = "Reflector-Map";
  map_header["mapName"] = map_name;

  map_header["resolution"] = mapping_config_.mapping_resolution;
  map_header["version"] = "1.2";

  internal_common::SaveLandmarkInfoToJson(result,
    mapping_config_.land_mark_config.landmark_radius,
    &map_json, &min_x, &min_y, &max_x, &max_y);

  atring_add["x"] = min_x - 1.0;
  atring_add["y"] = min_y - 1.0;
  map_header["minPos"] = atring_add;
  atring_add["x"] = max_x + 1.0;
  atring_add["y"] = max_y + 1.0;
  map_header["maxPos"] = atring_add;
  map_json["header"] = map_header;


  SLAM_INFO("save reflector map to %s", map_name.c_str());
  internal_common::Save(map_name.c_str(), map_json);
}




}  // namespace mapping_and_location
}  // namespace data_process
}  // namespace gomros
