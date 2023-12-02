/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.0
 * @Date: 2022-07-25 09:59:27
 * @LastEditTime: 2023-10-31 17:56:43
 * @Author: lcfc-desktop
 */

#include "reflector_location/reflector_location.h"


namespace gomros {
namespace data_process {
namespace mapping_and_location {

ReflectorLocation::ReflectorLocation(
  const LocationConfig& config)
: reflector_location_config_(config) {
  // ekf_cal_ = ekf_cal;
  is_finish_global_location_ = true;
  is_get_location_pos_ = false;
  LandMarkConfig land_mark_config = config.land_mark_config;
  land_mark_config.landmark_size_threshold /= 5;
  reflector_global_cal_ =
    std::make_shared<LandMark::Trilateration>(config.land_mark_config);
  location_real_time_ = std::make_shared<DataFusion::ReflectorEkfCalcuator>(
      config.ekf_config, config.sensor_mount, config.land_mark_config, false);
  display_ladar_ = std::make_shared<
    DisPlayResult::SimulationDataPublisher>(config.sensor_mount);
  data_fusion_ = std::make_shared<DataFusion::RecursiveAlgorithm>(0.01, 0.01);
}

void ReflectorLocation::HandleLaserData(const RadarSensoryMessage &data,
  const Position& forecast_pos,
  bool is_move) {
  is_get_location_pos_ = false;
  {
    std::lock_guard<std::recursive_mutex> lk(radar_data_mutex_);
    if ((!is_finish_global_location_ && !is_move) ||
        is_finish_global_location_) {
      ladar_datas_for_global_location_.push_back(data.mstruRadarMessage);
    } else if (!is_finish_global_location_ && is_move) {
      ladar_datas_for_global_location_.clear();
    }
  }
  if (!is_finish_global_location_) return;
  RadarSensoryInfo radar_info;
  {
    std::lock_guard<std::recursive_mutex> lk(radar_data_mutex_);
    if (ladar_datas_for_global_location_.empty()) return;
    radar_info = ladar_datas_for_global_location_.back();
    ladar_datas_for_global_location_.clear();
  }

  SensorMount sensor_mount;
  sensor_mount.radar_position_x = 0;
  sensor_mount.radar_position_y = 0;
  sensor_mount.radar_position_theta = 0;
  if (!is_move) return;
  if (reflector_location_config_.location_type == LocationType::Trilateral) {
    if (!is_move) {
      // 获取静止情况下 激光雷达在全局坐标中的位置
      std::vector<Eigen::Vector2d> obs_centers;
      DataFusion::Observation obs;
      obs.GetObsFromLadarData(reflector_location_config_.sensor_mount,
      reflector_location_config_.land_mark_config, radar_info);
      data_fusion_->AddPoint(obs.centers);
      Eigen::Vector3d ladar_global_pos;
      if (!data_fusion_->GetResult(&obs_centers) ||
          obs_centers.size() < 3) return;
      if (reflector_global_cal_->IsGetGlobalPos(
          obs_centers, &ladar_global_pos)) {
        Coordinate::Transform transform;
        transform.SetPose1InGlobal(ladar_global_pos);
        transform.SetPose2InPose1(Eigen::Vector3d(
          reflector_location_config_.sensor_mount.radar_position_x,
          reflector_location_config_.sensor_mount.radar_position_y,
          reflector_location_config_.sensor_mount.radar_position_theta));
        Eigen::Vector3d result;
        transform.GetPose2InGlobal(&result);
        SLAM_INFO("ladar_global_pos %f %f %f",
          result(0), result(1), result(2));
        is_get_location_pos_ = true;
        std::lock_guard<std::recursive_mutex> lk(state_data_mutex_);
        current_pose_.mfX = ladar_global_pos(0);
        current_pose_.mfY = ladar_global_pos(1);
        current_pose_.mfTheta = ladar_global_pos(2);
        current_pose_.mlTimestamp =
          radar_info.mstruRadarHeaderData.mlTimeStamp;
        SetGlobalLocationPos(current_pose_);
      }
      return;
    } else {
      data_fusion_->Init();
      DataFusion::Observation obs;
      Eigen::Vector3d ladar_global_pos;
      obs.GetObsFromLadarData(sensor_mount,
        reflector_location_config_.land_mark_config, radar_info);
      if (reflector_global_cal_->IsGetGlobalPos(
          obs.centers, &ladar_global_pos)) {
        SLAM_INFO("move_ladar_global_pos %f %f %f",
          ladar_global_pos(0), ladar_global_pos(1), ladar_global_pos(2));
      }
    }
  }
  DataFusion::Observation obs;
  obs.GetObsFromLadarData(reflector_location_config_.sensor_mount,
      reflector_location_config_.land_mark_config, radar_info);
  if (reflector_location_config_.location_type == LocationType::Trilateral) {
    if (obs.centers.size() >= 3) {
      // 遍历观测到的反光柱信息 进行定位
      Eigen::Vector3d global_pos;
      if (reflector_global_cal_->IsGetGlobalPos(obs.centers, &global_pos)) {
        is_get_location_pos_ = true;
        std::lock_guard<std::recursive_mutex> lk(state_data_mutex_);
        current_pose_.mfX = global_pos(0);
        current_pose_.mfY = global_pos(1);
        current_pose_.mfTheta = global_pos(2);
        current_pose_.mlTimestamp = radar_info.mstruRadarHeaderData.mlTimeStamp;
        SetGlobalLocationPos(current_pose_);
        SLAM_INFO("golbal_reflector_pos (%f %f %f)",
            current_pose_.mfX, current_pose_.mfY, current_pose_.mfTheta);
      }
    }
  } else {
    _handleObservation(obs);
  }

#ifdef TEST_DEBUG
  if (!is_get_location_pos_) return;
  std::vector<Eigen::Vector2d> match_obs;
  for (int i = 0; i < location_real_time_->GetMapMatch().size(); i++) {
    match_obs.push_back(
      land_mark_map_.at(-1)[location_real_time_->GetMapMatch()[i]]);
  }
  display_ladar_->DisPlayLadarAndObs(data.mstruRadarMessage, current_pose_,
                                     obs.centers, match_obs, current_cov_);
#endif
}

void ReflectorLocation::HandleOdomData(const OdometerMessage& odom_data) {
  // record_sensor_data_->RecordOdomData(odom_data);
  if (!is_finish_global_location_) return;
  location_real_time_->HandleOdomData(odom_data);
  return;
}

void ReflectorLocation::HandleImuData(const ImuSensoryMessage& imu_data) {
  // record_sensor_data_->RecordImuData(imu_data);
  std::lock_guard<std::recursive_mutex> lk(imu_data_mutex_);
  imu_list_.push_back(imu_data);
  if (imu_list_.size() > 100) imu_list_.clear();
}


void ReflectorLocation::SetInitPose(float x, float y, float theta) {
  DataFusion::State state;
  state.sigma.resize(3, 3);
  state.sigma.setZero(3, 3);
  state.mu.resize(3, 1);  // 初始化
  state.mu(0) = x;
  state.mu(1) = y;
  state.mu(2) = theta;
  return;
}

void ReflectorLocation::SetGlobalLocationPos(const Position& pos) {
  is_finish_global_location_ = true;
  {
    std::lock_guard<std::recursive_mutex> lk(state_data_mutex_);
    current_pose_ = pos;
  }
  DataFusion::State state;
  state.sigma.resize(3, 3);
  state.sigma.setZero(3, 3);
  state.mu.resize(3, 1);
  state.mu(0) = pos.mfX;
  state.mu(1) = pos.mfY;
  state.mu(2) = pos.mfTheta;
  state.time = pos.mlTimestamp;
  location_real_time_->SetInitState(state);
}

bool ReflectorLocation::StartGlobalLocating() {
  if (!is_finish_global_location_) {
    SLAM_WARN("now is reflector location, please wait......");
    return false;
  }
  is_finish_global_location_ = false;
  // 构建全局定位线程
  pthread_create(&global_location_mutex_, nullptr,
                  _globalLocatingThreadFunction, this);
  return true;
}

void ReflectorLocation::SetMapData(
  const std::map<int, std::map<int, Eigen::Vector2d>>& map) {
  land_mark_map_ = map;
  DataFusion::ReflectorMap reflector_map;
  reflector_map.reflectors_.clear();
  if (!land_mark_map_.count(-1)) {
    SLAM_WARN("reflector map error, please check map!");
  }
  for (auto iter : land_mark_map_.at(-1)) {
    SLAM_INFO("landmark map id:%d (%f %f)", iter.first,
              iter.second(0), iter.second(1));
    reflector_map.reflectors_.push_back(iter.second);
    reflector_map.reflector_map_coviarance_.push_back(
      Eigen::Matrix2d::Identity());
  }
  reflector_global_cal_->SetLandMarkMap(map);
  location_real_time_->SetMapDate(reflector_map);
}


bool ReflectorLocation::GetCurrentPosition(Position* pos) {
  std::lock_guard<std::recursive_mutex> lk(state_data_mutex_);
  *pos = current_pose_;
  return is_get_location_pos_;
}

bool ReflectorLocation::IsFinishLocate() {
  return is_finish_global_location_;
}

void* ReflectorLocation::_globalLocatingThreadFunction(void* ptr) {
  pthread_detach(pthread_self());
  ReflectorLocation *pgl = reinterpret_cast<ReflectorLocation *>(ptr);
  SLAM_INFO("begin redflector location thread.......");
  internal_common::RunTime run_time;
  while (!pgl->is_finish_global_location_) {
    if (pgl->ladar_datas_for_global_location_.size() < 50) continue;
    std::vector<RadarSensoryInfo> radar_info;
    {
      std::lock_guard<std::recursive_mutex> lk(pgl->radar_data_mutex_);
      radar_info = pgl->ladar_datas_for_global_location_;
      pgl->ladar_datas_for_global_location_.clear();
    }

#ifndef TEST_DEBUG
    if (run_time.ElapsedSecond() > 70) {
      SLAM_ERROR(">>>>>>reflector global location TimeOut<<<<<<<");
            // 更新当前位姿的时间戳
      pgl->current_pose_.mlTimestamp =
        radar_info.back().mstruRadarHeaderData.mlTimeStamp;
      SLAM_DEBUG("reflector location result is (%f %f %f)",
                pgl->current_pose_.mfX, pgl->current_pose_.mfY,
                pgl->current_pose_.mfTheta);
      pgl->is_get_location_pos_ = true;
      pgl->is_finish_global_location_ = true;
      break;
    }
#endif
    // pgl->_flushedCurrentPose(radar_info.mstruRadarHeaderData.mlTimeStamp);
    if (pgl->land_mark_map_.at(-1).size() < 3) {
      SLAM_ERROR("landmark in map size is less 3, can not do global location");
            // 更新当前位姿的时间戳
      pgl->current_pose_.mlTimestamp =
        radar_info.back().mstruRadarHeaderData.mlTimeStamp;
      SLAM_DEBUG("reflector location result is (%f %f %f)",
                pgl->current_pose_.mfX, pgl->current_pose_.mfY,
                pgl->current_pose_.mfTheta);
      pgl->is_get_location_pos_ = true;
      pgl->is_finish_global_location_ = true;
      break;
    }
    // 反光柱 拟合
    DataFusion::RecursiveAlgorithm data_fusion(0.01, 0.01);
    std::vector<Eigen::Vector2d> obs_centers;
    for (int i = 0; i < radar_info.size(); i++) {
      DataFusion::Observation obs;
      obs.GetObsFromLadarData(pgl->reflector_location_config_.sensor_mount,
          pgl->reflector_location_config_.land_mark_config, radar_info[i]);
      data_fusion.AddPoint(obs.centers);
    }
    data_fusion.GetResult(&obs_centers);
    if (obs_centers.size() < 3) continue;
    // 显示全局定位用的反光柱信息
    for (int i = 0; i < obs_centers.size(); i++) {
      SLAM_INFO("obs %d %f %f", i, obs_centers[i](0), obs_centers[i](1));
    }
    // 遍历观测到的反光柱信息 进行定位
    Eigen::Vector3d global_pos;
    bool get_pos =
      pgl->reflector_global_cal_->IsGetGlobalPos(obs_centers, &global_pos);
    if (get_pos) {
      std::lock_guard<std::recursive_mutex> lk(pgl->state_data_mutex_);
      pgl->current_pose_.mfX = global_pos(0);
      pgl->current_pose_.mfY = global_pos(1);
      pgl->current_pose_.mfTheta = global_pos(2);
      // 更新当前位姿的时间戳
      pgl->current_pose_.mlTimestamp =
        radar_info.back().mstruRadarHeaderData.mlTimeStamp;
      SLAM_DEBUG("reflector location result is (%f %f %f)",
                pgl->current_pose_.mfX, pgl->current_pose_.mfY,
                pgl->current_pose_.mfTheta);
      pgl->SetGlobalLocationPos(pgl->current_pose_);
      break;
    }
  }
  SLAM_INFO("exit reflector global location thread.......");
  pthread_exit(NULL);
}


void ReflectorLocation::_handleObservation(
  const DataFusion::Observation& obs) {
  if (obs.centers.empty()) return;
  location_real_time_->HandleObsData(obs);
  DataFusion::State state;
  is_get_location_pos_ = location_real_time_->GetState(&state);
  current_cov_ << state.sigma(0, 0), state.sigma(0, 1),
                  state.sigma(1, 0), state.sigma(1, 1);
  if (!is_get_location_pos_ && obs.centers.size() >= 3) {
    Eigen::Vector3d global_pos;
    if (reflector_global_cal_->IsGetGlobalPos(obs.centers, &global_pos)) {
      // TODO(r) 不要出现大规模跳变
      is_get_location_pos_ = true;
      if (fabs(state.mu(0) - global_pos(0)) < 0.1 &&
          fabs(state.mu(1) - global_pos(1)) < 0.1 &&
          fabs(state.mu(2) - global_pos(2)) < 0.1) {
          current_pose_.mfX = global_pos(0);
          current_pose_.mfY = global_pos(1);
          current_pose_.mfTheta = global_pos(2);
          current_pose_.mlTimestamp = obs.time_;
          SetGlobalLocationPos(current_pose_);
          SLAM_INFO("use global_pose (%f %f %f)",
            current_pose_.mfX, current_pose_.mfY, current_pose_.mfTheta);
      }
    }
  }
  if (is_get_location_pos_) {
    std::lock_guard<std::recursive_mutex> lk(state_data_mutex_);
    current_pose_.mfX = state.mu(0);
    current_pose_.mfY = state.mu(1);
    current_pose_.mfTheta = state.mu(2);
    current_pose_.mlTimestamp = state.time;
  }

  return;
}

}  // namespace mapping_and_location
}  // namespace data_process
}  // namespace gomros
