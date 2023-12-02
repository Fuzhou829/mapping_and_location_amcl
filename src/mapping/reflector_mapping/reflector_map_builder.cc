/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.2
 * @Author: renjy
 * @Date: 2022-10-28 14:32:05
 * @LastEditTime: 2023-10-18 15:44:35
 */

#include <float.h>


#include "reflector_mapping/reflector_map_builder.h"
#include "include/mapping_and_location_math.h"

namespace gomros {
namespace data_process {
namespace mapping_and_location {

ReflectorMapBuilder::ReflectorMapBuilder(const MappingConfig& config,
  int num_background_threads)
: config_(config), thread_pool_(num_background_threads) {
  optimizer_ = new Optimizer(config_, &thread_pool_);
  finished_submaps_.clear();
  active_submaps_.clear();
  // 创建第一个活跃的子图
  std::shared_ptr<Submap> first_subamp =
    std::make_shared<Submap>(config_, &thread_pool_);
  active_submaps_.push_back(first_subamp);
  display_reflector_ekf_ = std::make_shared<
    DisPlayResult::SimulationDataPublisher>(config_.sensor_mount);
}

void ReflectorMapBuilder::Init() {
  SLAM_INFO("init ReflectorMapBuilder~~~~~~");
  active_submaps_.clear();
  // 创建第一个活跃的子图
  std::shared_ptr<Submap> first_subamp =
    std::make_shared<Submap>(config_, &thread_pool_);
  active_submaps_.push_back(first_subamp);
  finished_submaps_.clear();
  optimizer_->Init();
}


void ReflectorMapBuilder::DispatchOdomData(
  const OdometerMessage& odom_message) {
  // TODO(r) assert()??
  if (active_submaps_.size() == 1) {
    active_submaps_[0]->HandleOdomData(odom_message);
    assert(finished_submaps_.size() == 0);
  } else {
    active_submaps_[0]->HandleOdomData(odom_message);
    active_submaps_[1]->HandleOdomData(odom_message);
  }
  return;
}

void ReflectorMapBuilder::DispatchLadarData(
  const RadarSensoryInfo& ladar_message) {
  // 处理雷达数据-获取观测反光柱(机器坐标系)
  DataFusion::Observation obs;
  obs.GetObsFromLadarData(config_.sensor_mount,
      config_.land_mark_config, ladar_message);
  if (active_submaps_.size() == 1) {
    assert(finished_submaps_.size() == 0);
    int deal_ladar_size = active_submaps_[0]->HandleObsData(obs);
    if (deal_ladar_size ==
        config_.land_mark_config.max_ladar_size_in_sub_map / 2) {
      active_submaps_.push_back(_createdNewSubMap(active_submaps_[0]));
    }
  } else {
    int deal_size_submap0 = active_submaps_[0]->HandleObsData(obs);
    int deal_size_submap1 = active_submaps_[1]->HandleObsData(obs);
    if (deal_size_submap0 ==
        config_.land_mark_config.max_ladar_size_in_sub_map) {
      finished_submaps_.push_back(active_submaps_[0]);
      std::shared_ptr<Submap> new_submap =
        _createdNewSubMap(active_submaps_[1]);
      active_submaps_[0] = active_submaps_[1];
      active_submaps_[1] = new_submap;
      finished_submaps_.back()->FinishSubMap();
      optimizer_->AddSubMap(finished_submaps_.back());
    }
  }

#ifdef TEST_DEBUG
  DataFusion::State state;
  active_submaps_[0]->GetReflectorEKFCal()->GetState(&state);
  Position current_pos;
  current_pos.mfX = state.mu(0);
  current_pos.mfY = state.mu(1);
  current_pos.mfTheta = state.mu(2);
  // display_reflector_ekf_->DisplayLadarMessage(ladar_message, current_pos);
#endif

  return;
}

void ReflectorMapBuilder::DispatchImuData(
  const ImuSensoryMessage& imu_message) {
  if (active_submaps_.size() == 1) {
    active_submaps_[0]->HandleImuData(imu_message);
    assert(finished_submaps_.size() == 0);
  } else {
    active_submaps_[0]->HandleImuData(imu_message);
    active_submaps_[1]->HandleImuData(imu_message);
  }
  return;
}


void ReflectorMapBuilder::FinishTrajectory() {
  // 寻找回环并进行优化 放入目前最完整的submap1即可,处理最后一个子图
  finished_submaps_.push_back(active_submaps_[0]);
  active_submaps_[0]->FinishSubMap();
  optimizer_->AddSubMap(active_submaps_[0]);
  optimizer_->RunFinalOptimization();
}

void ReflectorMapBuilder::SaveToFile(const std::string& map_name) {
  Json::Value map_json;
  Json::Value landmark_point;
  Json::Value map_header;
  Json::Value atring_add;
  map_header["mapType"] = "Reflector-Map";
  map_header["mapName"] = map_name;

  map_header["resolution"] = config_.mapping_resolution;
  map_header["version"] = "1.2";

  const std::map<int, Eigen::Vector3d> global_submap_poses =
    optimizer_->GetSubMapsGlobalPose();
  const std::map<int, std::vector<Eigen::Vector2d>> local_landmark_centers =
    optimizer_->GetSubMapsReflectors();
  // TODO(r) 通过半径滤波进行聚类
  LandMark::Calcuator landmark_cal(config_.land_mark_config);
  std::vector<LandMark::DbscanPoint> unclustered_centers;

  for (const auto &submap_id_pose : global_submap_poses) {
    for (const auto &local_center :
      local_landmark_centers.at(submap_id_pose.first)) {
      const auto &global_submap_pose = submap_id_pose.second;
      LandMark::DbscanPoint tmp;  // 将局部坐标转换为全局坐标
      tmp.x = local_center(0) * std::cos(global_submap_pose(2)) -
              local_center(1) * std::sin(global_submap_pose(2)) +
              global_submap_pose(0);
      tmp.y = local_center(0) * std::sin(global_submap_pose(2)) +
              local_center(1) * std::cos(global_submap_pose(2)) +
              global_submap_pose(1);
      tmp.clusterID = LandMark::UNCLASSIFIED;
      unclustered_centers.push_back(tmp);
      landmark_cal.AddLandMarkCenter(LandMark::Point(tmp.x, tmp.y));
    }
  }

  float min_x = FLT_MAX;
  float min_y = FLT_MAX;
  float max_x = -FLT_MAX;
  float max_y = -FLT_MAX;

  int landmark_id = 0;

  std::string full_map_data_path = config_.map_data_file_path + "/" +
                      internal_common::CreatedMapFileName(map_name);

  // TODO(r) 配置参数 通过密度聚类判断那些点是一个反光柱
  int minPts = 2;
  float eps = config_.land_mark_config.landmark_radius * 4;
  LandMark::ReflectorDbscan dbscan(minPts, eps, unclustered_centers);
  std::vector<LandMark::DbscanPoint> middle = dbscan.Run();  // 存在复制数据较多

  int id = 0;

  std::unordered_map<int, std::vector<Eigen::Vector2d>> clustered_centers;
  for (auto point : middle) {
    SLAM_INFO("point.clusterID: %d (%f %f)", point.clusterID, point.x, point.y);
    if (point.clusterID > 0) {
      clustered_centers[point.clusterID].push_back(
        Eigen::Vector2d(point.x, point.y));
    } else {
      if (point.x < min_x) min_x = point.x;
      if (point.y < min_y) min_y = point.y;
      if (point.x > max_x) max_x = point.x;
      if (point.y > max_y) max_y = point.y;
      // single_centers.push_back(tmp);
      Json::Value landmark_info;
      landmark_info["x"] = point.x;
      landmark_info["y"] = point.y;
      landmark_info["radius"] = config_.land_mark_config.landmark_radius;
      landmark_info["landmark_id"] = id;
      landmark_point[id] = landmark_info;
      SLAM_INFO("reflector_mapping index %d %f %f", id, point.x, point.y);
      id++;
    }
  }
  for (const auto &cluster : clustered_centers) {
    Eigen::Vector2d tmp_pose(0, 0);
    int n = cluster.second.size();
    for (const auto &center : cluster.second) {
      tmp_pose(0) += center(0) / n;
      tmp_pose(1) += center(1) / n;
    }
    if (tmp_pose(0) < min_x) min_x = tmp_pose(0);
    if (tmp_pose(1) < min_y) min_y = tmp_pose(1);
    if (tmp_pose(0) > max_x) max_x = tmp_pose(0);
    if (tmp_pose(1) > max_y) max_y = tmp_pose(1);
    // single_centers.push_back(tmp);
    Json::Value landmark_info;
    landmark_info["x"] = tmp_pose(0);
    landmark_info["y"] = tmp_pose(1);
    landmark_info["radius"] = config_.land_mark_config.landmark_radius;
    landmark_info["landmark_id"] = id;
    landmark_point[id] = landmark_info;
    SLAM_INFO("reflector_mapping index %d %f %f", id, tmp_pose(0), tmp_pose(1));
    id++;
  }

  if (landmark_point.size() <= 0) {
    min_x = min_y = max_x = max_y = 0;
  }
  atring_add["x"] = min_x - 1.0;
  atring_add["y"] = min_y - 1.0;
  map_header["minPos"] = atring_add;
  atring_add["x"] = max_x + 1.0;
  atring_add["y"] = max_y + 1.0;
  map_header["maxPos"] = atring_add;
  map_json["header"] = map_header;

  map_json["landmarks_pos_list"] = landmark_point;

  SLAM_INFO("save reflector map to %s", full_map_data_path.c_str());
  internal_common::Save(full_map_data_path.c_str(), map_json);
  return;
}



std::shared_ptr<Submap> ReflectorMapBuilder::_createdNewSubMap(
  std::shared_ptr<Submap> last_submap) {
  std::shared_ptr<Submap> new_submap_ptr =
    std::make_shared<Submap>(config_, &thread_pool_);
  new_submap_ptr->SetSubMapId(last_submap->GetSubMapId() + 1);
  // auto ekf_cal = new_submap_ptr->GetReflectorEKFCal();
  // *ekf_cal = (*(last_submap->GetReflectorEKFCal()));
  // 配置新子图全局坐标等
  DataFusion::State state;
  last_submap->GetReflectorEKFCal()->GetState(&state);
  new_submap_ptr->InitReflectorEkf(state);
  new_submap_ptr->SetGlobalPose(
    state.mu.topRows(3), state.sigma.block(0, 0, 3, 3));
  // 求解新子图到上一个子图坐标系下的坐标
  Eigen::Vector3d relative_pose;
  double last_global_x = last_submap->GetGlobalPose().first.x();
  double last_global_y = last_submap->GetGlobalPose().first.y();
  double last_global_theta = last_submap->GetGlobalPose().first.z();
  double new_global_x = new_submap_ptr->GetGlobalPose().first.x();
  double new_global_y = new_submap_ptr->GetGlobalPose().first.y();
  double new_global_theta = new_submap_ptr->GetGlobalPose().first.z();
  relative_pose.x() =
    (new_global_x - last_global_x) * std::cos(last_global_theta) +
    (new_global_y - last_global_y) * std::sin(last_global_theta);
  relative_pose.y() =
    -(new_global_x - last_global_x) * std::sin(last_global_theta) +
    (new_global_y - last_global_y) * std::cos(last_global_theta);
  relative_pose.z() =
    SLAMMath::NormalizePITheta(new_global_theta - last_global_theta);
  new_submap_ptr->SetRelativePose(relative_pose);
  return new_submap_ptr;
}



}  // namespace mapping_and_location
}  // namespace data_process
}  // namespace gomros
