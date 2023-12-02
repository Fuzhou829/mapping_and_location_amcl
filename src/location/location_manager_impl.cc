/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.0
 * @Date: 2022-08-18 09:21:41
 * @LastEditTime: 2023-11-07 19:18:49
 * @Author: lcfc-desktop
 */

#include <fstream>
#include <string>
#include <vector>

#include "common/tool.h"
#include "common_lib/message.h"
#include "common_lib/time_utils.h"
#include "include/mapping_and_location_math.h"
#include "location/location_area.h"
#include "location/location_manager_impl.h"
#include "message_lib/simple_grid_map.h"

namespace gomros {
namespace data_process {
namespace mapping_and_location {

LocationManagerImpl::LocationManagerImpl(
    const LocationConfig &config, std::shared_ptr<DeviceState> device_state) {
  pthread_mutex_init(&amcl_pos_data_mutex_, nullptr);
  pthread_mutex_init(&odom_data_mutex_, nullptr);
  pthread_mutex_init(&last_odom_mutex_, nullptr);
  pthread_mutex_init(&odom_data_for_amcl_mutex_, nullptr);
  pthread_mutex_init(&ladar_data_mutex_, nullptr);
  location_config_ = config;
  device_state_ = device_state;
  is_move_ = false;
  get_last_odom_ = false;
  is_get_total_odom_ = false;
  total_odom_ = 0.0;
  qr_distance_ = 0.0;
  ekf_for_amcl_ = std::make_shared<DataFusion::EKFCalcuator>(config.ekf_config,
                                                             "ekf_for_amcl_");
  location_data_fusion_cal_ = std::make_shared<DataFusion::LocationResult>(
      location_config_.location_type);
  grid_location_ = std::make_shared<GridLocation>(config);
  qr_location_ = std::make_shared<QrLocation>(config);

  reflector_location_ = std::make_shared<ReflectorLocation>(config);

  wan_ji_location_ = std::make_shared<WanJiLocation>(location_config_);

  SLAM_INFO("now location type is %d\n", location_config_.location_type);
  if (location_config_.location_type == LocationType::QR) {
    pthread_create(&odom_thread, NULL, _handleOdomFunction, this);
  }
  if (location_config_.location_type == LocationType::FusionLocation ||
      location_config_.location_type == LocationType::LandMarkLocation ||
      location_config_.location_type == LocationType::Trilateral) {
    pthread_create(&reflector_location_thread_, NULL,
                   _reflectorLocationFunction, this);
  }
  if (location_config_.location_type == LocationType::FusionLocation ||
      location_config_.location_type == LocationType::AMCL) {
    SLAM_INFO("创建amcl定位线程\n");
    pthread_create(&amcl_location_thread_, NULL, _amclLocationFunction, this);
  }

  if (location_config_.location_type == LocationType::WanJiLandMarkLocation ||
      location_config_.location_type == LocationType::WanJiAndQR) {
    if (location_config_.odom_type != OdomType::SingleSteerModel) {
      SLAM_ERROR("please choose odomtype SingleSteerModel %d, now choose is %d",
                 OdomType::SingleSteerModel, location_config_.odom_type);
    }
    pthread_create(&wanji_location_thread_, NULL, _wanJiLocationFunction, this);
  }

  is_reflector_deal_radar_data_ = false;
  is_amcl_deal_radar_data_ = false;
  is_finish_global_location = true;
}

LocationManagerImpl::~LocationManagerImpl() {}

void LocationManagerImpl::DealEmerge() {
  get_last_odom_ = false;
  qr_location_->ClearImuData();
  ekf_for_amcl_->ClearImuData();
}

/**
 * @brief 开始全局定位
 *
 */
void LocationManagerImpl::StartGlobalLocating(bool is_need_location) {
  if (!is_finish_global_location) {
    SLAM_WARN("last location is not finished, please wait ......");
    return;
  }
  is_finished_global_location_ = false;
  is_get_total_odom_ = false;
  if (location_config_.location_type == LocationType::QR) {
    // location_last_pose_.mlTimestamp = gomros::common::GetCurrentTime_us();
    qr_location_->SetInitPos(location_last_pose_);
    Eigen::Matrix3d covariance;
    covariance.setZero(3, 3);
    ekf_for_amcl_->SetInitPos(location_last_pose_, covariance);
    device_state_->mcChassisState = SYSTEM_STATE_FREE;
    is_finished_global_location_ = true;
    is_finish_global_location = true;
  } else if (location_config_.location_type == LocationType::LandMarkLocation ||
             location_config_.location_type == LocationType::Trilateral) {
    reflector_location_->StartGlobalLocating();
    pthread_create(&m_MonitorGlobalLocateThread, NULL,
                   _monitorGlobalLocateThreadFunction, this);
  } else if (location_config_.location_type ==
                 LocationType::WanJiLandMarkLocation ||
             location_config_.location_type == LocationType::WanJiAndQR) {
    pthread_create(&m_MonitorGlobalLocateThread, NULL,
                   _monitorGlobalLocateThreadFunction, this);
  } else {
    grid_location_->StartGlobalLocating(is_need_location);
    pthread_create(&m_MonitorGlobalLocateThread, NULL,
                   _monitorGlobalLocateThreadFunction, this);
  }
  pthread_mutex_lock(&odom_data_mutex_);
  odom_list_.clear();
  pthread_mutex_unlock(&odom_data_mutex_);
}

/**
 * @brief 监控定位是否完成线程
 *
 * @param param
 * @return void*
 */

void *LocationManagerImpl::_monitorGlobalLocateThreadFunction(void *param) {
  pthread_detach(pthread_self());
  LocationManagerImpl *pc = reinterpret_cast<LocationManagerImpl *>(param);
  pc->_doGlobalLocateMonitor();
  pthread_exit(NULL);
  return nullptr;
}

void *LocationManagerImpl::_reflectorLocationFunction(void *param) {
  pthread_detach(pthread_self());
  SLAM_INFO("begin reflector location thread<<<<<<<");
  LocationManagerImpl *pc = reinterpret_cast<LocationManagerImpl *>(param);
  float jump_distance = 0.0;
  Position last_ladar_pos;
  bool is_get_jump = false;
  while (1) {
    if (!pc->is_reflector_deal_radar_data_) {
      usleep(10000);
      continue;
    }
    pc->is_reflector_deal_radar_data_ = false;
    Position reflector_pose, last_pos, now_odom_pos, current_pos;
    pthread_mutex_lock(&pc->ladar_data_mutex_);
    RadarSensoryMessage radar_data = pc->radar_data_;
    pthread_mutex_unlock(&pc->ladar_data_mutex_);
    last_pos = pc->_refreshOdom(
        radar_data.mstruRadarMessage.mstruRadarHeaderData.mlTimeStamp);
    if (last_pos.mlTimestamp !=
            radar_data.mstruRadarMessage.mstruRadarHeaderData.mlTimeStamp &&
        pc->is_finish_global_location) {
      // uint64_t delta_time =
      //   radar_data.mstruRadarMessage.mstruRadarHeaderData.mlTimeStamp >
      //   last_pos.mlTimestamp ?
      //   radar_data.mstruRadarMessage.mstruRadarHeaderData.mlTimeStamp -
      //   last_pos.mlTimestamp :
      //   last_pos.mlTimestamp -
      //   radar_data.mstruRadarMessage.mstruRadarHeaderData.mlTimeStamp;
      // SLAM_WARN("time is not same delta time is %ld", delta_time);
      std::lock_guard<std::recursive_mutex> lk(pc->reflector_pos_data_mutex_);
      pc->reflector_current_pos_ = last_pos;
      pc->reflector_last_pos_ = last_pos;
      continue;
    }

    {
      std::lock_guard<std::recursive_mutex> lk(pc->reflector_pos_data_mutex_);
      pc->reflector_current_pos_ = last_pos;
      pc->reflector_last_pos_ = last_pos;
    }
    internal_common::RunTime run_time;
    bool is_can_do_location = pc->is_move_ && pc->is_finish_global_location;

    pc->reflector_location_->HandleLaserData(radar_data, last_pos,
                                             is_can_do_location);
    std::map<LocationType, int> types =
        pc->location_data_fusion_cal_->GetLocationType();
    if (!pc->is_finish_global_location ||
        !types.count(LocationType::LandMarkLocation))
      continue;
    if (pc->reflector_location_->GetCurrentPosition(&reflector_pose)) {
      if (pc->_isPosJump(last_pos, reflector_pose) && pc->is_move_) {
        if (is_get_jump) {
          jump_distance +=
              SLAMMath::Dist(last_pos.mfX, last_pos.mfY, last_ladar_pos.mfX,
                             last_ladar_pos.mfY);
        }
        pc->reflector_current_pos_ = last_pos;
      } else {
        jump_distance = 0.0;
        pc->reflector_current_pos_ = reflector_pose;
        SLAM_DEBUG("reflector_pose %f %f %f", reflector_pose.mfX,
                   reflector_pose.mfY, reflector_pose.mfTheta);
        if (pc->device_state_->mcChassisState == SYSTEM_STATE_OFFLINE) {
          SLAM_INFO("from %d to %d", pc->device_state_->mcChassisState, 1);
          pc->device_state_->mcChassisState = SYSTEM_STATE_FREE;
        }
      }
      last_ladar_pos = pc->reflector_current_pos_;
      is_get_jump = true;
      if (jump_distance > 2.0) {
        // 触发定位失效报警
        if (pc->device_state_->mcChassisState != SYSTEM_STATE_OFFLINE) {
          SLAM_INFO("from %d to %d", pc->device_state_->mcChassisState, 0);
          pc->device_state_->mcChassisState = SYSTEM_STATE_OFFLINE;
        }
      }
      SLAM_INFO("reflector run time %f ms",
                run_time.ElapsedMicroSecond() / 1000.0f);
      pc->_updateReflectorPos();
    }
  }
  pthread_exit(NULL);
}

void *LocationManagerImpl::_amclLocationFunction(void *param) {
  pthread_detach(pthread_self());
  LocationManagerImpl *pc = reinterpret_cast<LocationManagerImpl *>(param);
  SLAM_INFO("begin AMCL location thread<<<<<<<");
  float jump_distance = 0.0;
  Position last_ladar_pos;
  bool is_get_jump = false;
  while (1) {
    if (pc->is_amcl_deal_radar_data_ && pc->have_init_pose_ && pc->have_map_) {
      pc->is_amcl_deal_radar_data_ = false;
      std::map<LocationType, int> types =
          pc->location_data_fusion_cal_->GetLocationType();
      if (pc->is_finish_global_location && !types.count(LocationType::AMCL))
        continue;
      RadarSensoryMessage radar_data;
      pthread_mutex_lock(&pc->ladar_data_mutex_);
      radar_data = pc->radar_data_;
      pthread_mutex_unlock(&pc->ladar_data_mutex_);
      uint64_t radar_time =
          radar_data.mstruRadarMessage.mstruRadarHeaderData.mlTimeStamp;
      Position last_odom_pos = pc->_refreshOdomForAmcl(radar_time);
      Position last_pos = pc->_refreshOdom(radar_time);
      if (pc->is_finish_global_location &&
          last_odom_pos.mlTimestamp != radar_time) {
        pthread_mutex_lock(&pc->amcl_pos_data_mutex_);
        pc->amcl_last_pos_ = last_pos;
        pc->amcl_current_pos_ = last_pos;
        pthread_mutex_unlock(&pc->amcl_pos_data_mutex_);
        continue;
      }
      pthread_mutex_lock(&pc->amcl_pos_data_mutex_);
      pc->amcl_last_pos_ = last_pos;
      pc->amcl_current_pos_ = last_pos;
      pthread_mutex_unlock(&pc->amcl_pos_data_mutex_);
      // if ((last_pos.mlTimestamp != radar_time ||
      //   last_odom_pos.mlTimestamp != radar_time) &&
      //   pc->is_finish_global_location) {
      //   double delta_time = static_cast<double>(radar_time) -
      //                       static_cast<double>(last_pos.mlTimestamp);
      //   double delta_time_amcl = static_cast<double>(radar_time) -
      //                       static_cast<double>(last_odom_pos.mlTimestamp);
      //   if (delta_time > 0 || delta_time_amcl > 0) {
      //     SLAM_ERROR("amcl_delta_time %f, odom %f",
      //             delta_time_amcl, delta_time);
      //   } else {
      //     SLAM_WARN("amcl_delta_time %f, odom %f",
      //             delta_time_amcl, delta_time);
      //   }
      // }
      bool is_need_update =
          pc->is_finish_global_location && last_pos.mlTimestamp == radar_time;
      bool is_can_do_location = pc->is_move_ && pc->is_finish_global_location;
      internal_common::RunTime run_time;
      pc->grid_location_->HandleLaserData(radar_data, last_odom_pos,
                                          is_can_do_location);
      bool is_need_amcl_location =
          (pc->location_config_.location_type == LocationType::AMCL ||
           pc->location_config_.location_type == LocationType::FusionLocation);
      if (is_need_amcl_location && pc->is_finish_global_location &&
          is_need_update) {
        Position now_amcl_pos;
        pthread_mutex_lock(&pc->amcl_pos_data_mutex_);
        pc->amcl_last_pos_ = last_pos;
        pc->amcl_current_pos_ = last_pos;
        if (pc->grid_location_->GetCurrentPosition(&now_amcl_pos)) {
          if (pc->_isPosJump(last_pos, now_amcl_pos)) {
            if (!is_get_jump) last_ladar_pos = last_pos;
            is_get_jump = true;
            pc->amcl_current_pos_ = now_amcl_pos;  // 暂时不监跳变
          } else {
            is_get_jump = false;
            pc->amcl_current_pos_ = now_amcl_pos;
          }
          SLAM_INFO(
              "amcl run time %f ms jump_distance %f delta to now %ld",
              run_time.ElapsedMicroSecond() / 1000.0f, jump_distance,
              gomros::common::GetCurrentTime_us() - now_amcl_pos.mlTimestamp);
          pc->_updateAmclPos();
        }
        if (!is_get_jump) {
          jump_distance = 0;
        } else {
          jump_distance += SLAMMath::Dist(
              last_ladar_pos.mfX, last_ladar_pos.mfY, pc->amcl_current_pos_.mfX,
              pc->amcl_current_pos_.mfY);
        }

        last_ladar_pos = pc->amcl_current_pos_;
        pthread_mutex_unlock(&pc->amcl_pos_data_mutex_);
        if (jump_distance > 1.0) {
          // 触发定位失效报警
          // pc->device_state_->mcChassisState = SYSTEM_STATE_OFFLINE;
        }
      }
      if (!is_need_amcl_location || !pc->is_finish_global_location) {
        is_get_jump = false;
      }
    } else {
      usleep(20000);
    }
  }
  pthread_exit(NULL);
}

void *LocationManagerImpl::_wanJiLocationFunction(void *param) {
  pthread_detach(pthread_self());
  SLAM_INFO("begin wanJi location thread<<<<<<<");
  LocationManagerImpl *pc = reinterpret_cast<LocationManagerImpl *>(param);
  while (1) {
    usleep(10000);
    if (!pc->is_reflector_deal_radar_data_) continue;
    pc->is_reflector_deal_radar_data_ = false;
    Position reflector_pose, last_pos, now_odom_pos, current_pos;
    pthread_mutex_lock(&pc->ladar_data_mutex_);
    RadarSensoryMessage radar_data = pc->radar_data_;
    pthread_mutex_unlock(&pc->ladar_data_mutex_);
    last_pos = pc->_refreshOdom(
        radar_data.mstruRadarMessage.mstruRadarHeaderData.mlTimeStamp);
    if (last_pos.mlTimestamp !=
            radar_data.mstruRadarMessage.mstruRadarHeaderData.mlTimeStamp &&
        pc->is_finish_global_location) {
      // uint64_t delta_time =
      //   radar_data.mstruRadarMessage.mstruRadarHeaderData.mlTimeStamp >
      //   last_pos.mlTimestamp ?
      //   radar_data.mstruRadarMessage.mstruRadarHeaderData.mlTimeStamp -
      //   last_pos.mlTimestamp :
      //   last_pos.mlTimestamp -
      //   radar_data.mstruRadarMessage.mstruRadarHeaderData.mlTimeStamp;
      // SLAM_WARN("time is not same delta time is %ld", delta_time);
      std::lock_guard<std::recursive_mutex> lk(pc->reflector_pos_data_mutex_);
      pc->reflector_current_pos_ = last_pos;
      pc->reflector_last_pos_ = last_pos;
      continue;
    }
    {
      std::lock_guard<std::recursive_mutex> lk(pc->reflector_pos_data_mutex_);
      pc->reflector_current_pos_ = last_pos;
      pc->reflector_last_pos_ = last_pos;
    }
    internal_common::RunTime run_time;
    bool is_can_do_location = pc->is_move_ && pc->is_finish_global_location;
    pc->wan_ji_location_->HandleLaserData(radar_data, last_pos,
                                          is_can_do_location);

    if (!pc->wan_ji_location_->IsFinishLocate()) continue;
    if (pc->wan_ji_location_->GetCurrentPosition(&reflector_pose)) {
      {
        std::lock_guard<std::recursive_mutex> lk(pc->reflector_pos_data_mutex_);
        pc->reflector_last_pos_ = last_pos;
        pc->reflector_current_pos_ = reflector_pose;
        pc->_updateReflectorPos();
      }
      SLAM_INFO("wan ji reflector run time %f ms",
                run_time.ElapsedMicroSecond() / 1000.0f);
    }
  }
  pthread_exit(NULL);
}

void *LocationManagerImpl::_handleOdomFunction(void *param) {
  pthread_detach(pthread_self());
  LocationManagerImpl *pc = reinterpret_cast<LocationManagerImpl *>(param);
  while (1) {
    usleep(10000);
    if (pc->odom_list_.size() > 200) {
      pc->_refreshOdom();
    }
    OdometerMessage odom_data;
    pthread_mutex_lock(&pc->odom_data_mutex_);
    if (pc->odom_list_.empty()) {
      pthread_mutex_unlock(&pc->odom_data_mutex_);
      continue;
    }
    odom_data = pc->odom_list_.front();
    pc->odom_list_.pop_front();
    pc->qr_location_->HandleOdomData(odom_data);
    pthread_mutex_unlock(&pc->odom_data_mutex_);

    pthread_mutex_lock(&pc->odom_data_for_amcl_mutex_);
    if (pc->odom_list_for_amcl_.empty()) {
      pthread_mutex_unlock(&pc->odom_data_for_amcl_mutex_);
      continue;
    }
    odom_data = pc->odom_list_for_amcl_.front();
    pc->odom_list_for_amcl_.pop_front();
    pc->ekf_for_amcl_->HandleOdomData(odom_data);
    pc->ekf_for_amcl_->Update();
    pthread_mutex_unlock(&pc->odom_data_for_amcl_mutex_);
  }
  pthread_exit(NULL);
}

void LocationManagerImpl::SetMapData(SimpleGridMap *simple_grid_map) {
  grid_location_->SetMapData(simple_grid_map);
  SetHaveMap(true);
}

void LocationManagerImpl::SetLocationArea(
    const std::map<LocationType, std::vector<LocationArea>> &areas) {
  location_data_fusion_cal_->SetLocationArea(areas);
}

void LocationManagerImpl::SetQrData(const DMreaderSensoryMessage &data) {
  // uint64_t current_time = gomros::common::GetCurrentTime_us();
  bool is_need_qr_location =
      (location_config_.location_type == LocationType::QR ||
       location_config_.location_type == LocationType::FusionLocation ||
       location_config_.location_type == LocationType::WanJiAndQR);
  if (is_need_qr_location) {
    Position qr_pos;
    if (qr_location_->HandleQrData(data, &qr_pos)) {
      qr_distance_ = 0.0;
      // SLAM_DEBUG("qr get time %ld", current_time - qr_pos.mlTimestamp);
      _updateQrPos(qr_pos);
    }
  }
}

void LocationManagerImpl::SetIniPose(const Position &pose) {
  Position init_pose = pose;
  init_pose.mlTimestamp = gomros::common::GetCurrentTime_us();
  grid_location_->SetInitPose(init_pose.mfX, init_pose.mfY, init_pose.mfTheta);
  qr_location_->SetInitPos(init_pose);
  Eigen::Matrix3d covariance;
  covariance.setZero(3, 3);
  ekf_for_amcl_->SetInitPos(init_pose, covariance);
  location_last_pose_ = init_pose;
  {
    pthread_mutex_lock(&amcl_pos_data_mutex_);
    amcl_current_pos_ = init_pose;
    amcl_last_pos_ = init_pose;
    pthread_mutex_unlock(&amcl_pos_data_mutex_);
  }
  {
    std::lock_guard<std::recursive_mutex> lk(reflector_pos_data_mutex_);
    reflector_current_pos_ = init_pose;
    reflector_last_pos_ = init_pose;
  }
  SetHavePose(true);
  pthread_mutex_lock(&odom_data_mutex_);
  odom_list_.clear();
  pthread_mutex_unlock(&odom_data_mutex_);
}

void LocationManagerImpl::SetHaveMap(bool have_map) { have_map_ = have_map; }

bool LocationManagerImpl::FinishLocate() {
  return is_finished_global_location_;
}

void LocationManagerImpl::SetHavePose(bool have_pose) {
  have_init_pose_ = have_pose;
}

void LocationManagerImpl::SetRadarData(const RadarSensoryMessage &data) {
  radar_disconnect = false;
  pthread_mutex_lock(&ladar_data_mutex_);
  if (radar_data_.mstruRadarMessage.mstruRadarHeaderData.mlTimeStamp >=
      data.mstruRadarMessage.mstruRadarHeaderData.mlTimeStamp) {
    SLAM_ERROR("set radar is error %ld",
               radar_data_.mstruRadarMessage.mstruRadarHeaderData.mlTimeStamp -
                   data.mstruRadarMessage.mstruRadarHeaderData.mlTimeStamp);
  }
  radar_data_ = data;
  pthread_mutex_unlock(&ladar_data_mutex_);
  is_amcl_deal_radar_data_ = true;
  is_reflector_deal_radar_data_ = true;
}

void LocationManagerImpl::SetOdomData(const OdometerMessage &data) {
  pthread_mutex_lock(&odom_data_mutex_);
  odom_list_.push_back(data);
  pthread_mutex_unlock(&odom_data_mutex_);
  pthread_mutex_lock(&last_odom_mutex_);
  last_odom_data_ = data;
  pthread_mutex_unlock(&last_odom_mutex_);

  get_last_odom_ = true;
  if (location_config_.location_type == LocationType::FusionLocation ||
      location_config_.location_type == LocationType::AMCL) {
    pthread_mutex_lock(&odom_data_for_amcl_mutex_);
    odom_list_for_amcl_.push_back(data);
    pthread_mutex_unlock(&odom_data_for_amcl_mutex_);
  }

  if (location_config_.location_type == LocationType::FusionLocation ||
      location_config_.location_type == LocationType::LandMarkLocation ||
      location_config_.location_type == LocationType::Trilateral) {
    reflector_location_->HandleOdomData(data);
  }
}
void LocationManagerImpl::SetImuData(const ImuSensoryMessage &data) {
  qr_location_->HandleImuData(data);
  // reflector_location_->HandleImuData(data);
  // ekf_for_amcl_->HandleImuData(data);
}

void LocationManagerImpl::ResetPose() {
  Position pos;
  pos.mlTimestamp = gomros::common::GetCurrentTime_us();
  qr_location_->SetInitPos(pos);
  Eigen::Matrix3d covariance;
  covariance.setZero(3, 3);
  ekf_for_amcl_->SetInitPos(pos, covariance);
}

void LocationManagerImpl::SetTotalOdom(double total_odom) {
  SLAM_INFO("set total_odom %f", total_odom);
  total_odom_ = total_odom;
  is_get_total_odom_ = true;
}

void LocationManagerImpl::SetDeviceState(
    std::shared_ptr<gomros::message::DeviceState> device_state) {
  device_state_ = device_state;
}

double LocationManagerImpl::GetTotalOdom() { return total_odom_; }

double LocationManagerImpl::GetQrOdom() {
  if (LocationType::QR != location_config_.location_type) {
    qr_distance_ = 0.0;
  }
  return qr_distance_;
}

gomros::message::Position LocationManagerImpl::GetCurrentPose() {
  Position current_pos, amcl_pos, land_mark_pos;
  bool is_pop = false;
  if (device_state_->mcChassisState == SYSTEM_STATE_MAPPING) is_pop = true;
#ifndef TEST_DEBUG
  uint64_t current_time = gomros::common::GetCurrentTime_us();
  current_pos = _refreshOdom(current_time, is_pop);
#endif

#ifdef TEST_DEBUG
  current_pos = _refreshOdom(is_pop);
#endif
  {
    pthread_mutex_lock(&amcl_pos_data_mutex_);
    _calDaltaPos(amcl_last_pos_, current_pos, amcl_current_pos_, &amcl_pos);
    pthread_mutex_unlock(&amcl_pos_data_mutex_);
  }

  {
    std::lock_guard<std::recursive_mutex> lk(reflector_pos_data_mutex_);
    _calDaltaPos(reflector_last_pos_, current_pos, reflector_current_pos_,
                 &land_mark_pos);
  }
  std::map<LocationType, DataFusion::PoseInfo> pos_candidate;
  pos_candidate.insert(std::make_pair(
      LocationType::AMCL,
      DataFusion::PoseInfo(
          Eigen::Vector3f(amcl_pos.mfX, amcl_pos.mfY, amcl_pos.mfTheta),
          amcl_pos.mlTimestamp, 1.0, LocationType::AMCL)));
  pos_candidate.insert(std::make_pair(
      LocationType::QR,
      DataFusion::PoseInfo(Eigen::Vector3f(current_pos.mfX, current_pos.mfY,
                                           current_pos.mfTheta),
                           current_pos.mlTimestamp, 1.0, LocationType::QR)));
  pos_candidate.insert(std::make_pair(
      LocationType::LandMarkLocation,
      DataFusion::PoseInfo(Eigen::Vector3f(land_mark_pos.mfX, land_mark_pos.mfY,
                                           land_mark_pos.mfTheta),
                           land_mark_pos.mlTimestamp, 1.0,
                           LocationType::LandMarkLocation)));
  DataFusion::PoseInfo result_info;
  std::map<LocationType, int> types =
      location_data_fusion_cal_->GetLocationResult(pos_candidate, &result_info);
  Position result;
  result.mfX = result_info.pose_info(0);
  result.mfY = result_info.pose_info(1);
  result.mfTheta = result_info.pose_info(2);
  result.mlTimestamp = result_info.time_stamp;

  float dis = SLAMMath::Dist(result.mfX, result.mfY, location_last_pose_.mfX,
                             location_last_pose_.mfY);
  if (is_get_total_odom_) total_odom_ += dis;
  if (types.count(LocationType::AMCL)) {
    if (types.at(LocationType::AMCL) != 0) {
      // 即将进入AMCL区域 准备好切换
      grid_location_->SetGlobalLocationPos(result);
    }
  }
  qr_distance_ += dis;
  location_last_pose_ = current_pos;
  // pthread_mutex_lock(&last_odom_mutex_);
  // bool is_not_move =
  //   fabs(last_odom_data_.mstruDiffSteerSts.mfLeftLinearVel) < 0.001 &&
  //   fabs(last_odom_data_.mstruDiffSteerSts.mfRightLinearVel) < 0.001;
  // pthread_mutex_unlock(&last_odom_mutex_);
  // if (is_not_move) {
  //   SLAM_INFO("stop_current pose %f %f %f %ld now time %ld",
  //             current_pos.mfX, current_pos.mfY, current_pos.mfTheta,
  //             current_pos.mlTimestamp, current_time);
  // } else {
  //   SLAM_INFO("move_current pose %f %f %f %ld now time %ld",
  //             current_pos.mfX, current_pos.mfY, current_pos.mfTheta,
  //             current_pos.mlTimestamp, current_time);
  // }
  return current_pos;
}

LocationManagerImpl::Position LocationManagerImpl::_refreshOdom(bool is_pop) {
  Position current_pose, last_pose;
  pthread_mutex_lock(&odom_data_mutex_);
  qr_location_->GetCurrentPose(&last_pose);
  uint64_t time = last_pose.mlTimestamp;
  OdometerMessage odom_data;
  if (!is_pop) {
    for (auto data : odom_list_) {
      odom_data = data;
      qr_location_->HandleOdomData(odom_data);
    }
  } else {
    while (!odom_list_.empty()) {
      odom_data = odom_list_.front();
      odom_list_.pop_front();
      qr_location_->HandleOdomData(odom_data);
    }
  }
  qr_location_->GetCurrentPose(&current_pose);
  assert(last_pose.mlTimestamp == time);
  if (!is_pop) qr_location_->SetInitPos(last_pose);
  pthread_mutex_unlock(&odom_data_mutex_);
  return current_pose;
}

LocationManagerImpl::Position LocationManagerImpl::_refreshOdom(uint64_t time,
                                                                bool is_pop) {
  OdometerMessage odom_data;
  bool get_odom_data = false;
  Position current_pose, last_pose;
  pthread_mutex_lock(&odom_data_mutex_);
  qr_location_->GetCurrentPose(&last_pose);
  if (!is_pop) {
    for (auto data : odom_list_) {
      get_odom_data = true;
      odom_data = data;
      if (odom_data.mclDeltaPosition.mlTimestamp > time) break;
      qr_location_->HandleOdomData(odom_data);
    }
  } else {
    while (!odom_list_.empty()) {
      odom_data = odom_list_.front();
      get_odom_data = true;
      if (odom_data.mclDeltaPosition.mlTimestamp > time) break;
      odom_list_.pop_front();
      qr_location_->HandleOdomData(odom_data);
    }
  }

  if (get_odom_data) {
    odom_data.mclDeltaPosition.mlTimestamp = time;
    qr_location_->HandleOdomData(odom_data);
  } else if (get_last_odom_) {
    // get_last_odom_ = false;
    OdometerMessage data;
    pthread_mutex_lock(&last_odom_mutex_);
    data = last_odom_data_;
    data.mclDeltaPosition.mlTimestamp = time;
    pthread_mutex_unlock(&last_odom_mutex_);
    qr_location_->HandleOdomData(data);
  }
  qr_location_->GetCurrentPose(&current_pose);
  if (!is_pop) qr_location_->SetInitPos(last_pose);
  pthread_mutex_unlock(&odom_data_mutex_);
  // if (current_pose.mlTimestamp < time && !is_pop) {
  //   SLAM_ERROR("not the time pose %f %f %f %ld %ld",
  //     current_pose.mfX, current_pose.mfY, current_pose.mfTheta,
  //     current_pose.mlTimestamp, time);
  // }
  return current_pose;
}

LocationManagerImpl::Position LocationManagerImpl::_refreshOdomForAmcl(
    uint64_t time) {
  Position current_pose;
  ekf_for_amcl_->GetCurrentState(&current_pose);

  if (location_config_.location_type != LocationType::FusionLocation &&
      location_config_.location_type != LocationType::AMCL)
    return current_pose;

  OdometerMessage odom_data;
  bool is_get_odom = false;
  pthread_mutex_lock(&odom_data_for_amcl_mutex_);
  while (!odom_list_for_amcl_.empty()) {
    odom_data = odom_list_for_amcl_.front();
    is_get_odom = true;
    if (odom_data.mclDeltaPosition.mlTimestamp > time) break;
    odom_list_for_amcl_.pop_front();
    ekf_for_amcl_->HandleOdomData(odom_data);
    ekf_for_amcl_->Update();
  }
  if (is_get_odom) {
    odom_data.mclDeltaPosition.mlTimestamp = time;
    ekf_for_amcl_->HandleOdomData(odom_data);
    ekf_for_amcl_->Update();
  } else if (get_last_odom_) {
    // get_last_odom_ = false;
    OdometerMessage data;
    pthread_mutex_lock(&last_odom_mutex_);
    data = last_odom_data_;
    data.mclDeltaPosition.mlTimestamp = time;
    pthread_mutex_unlock(&last_odom_mutex_);
    ekf_for_amcl_->HandleOdomData(data);
    ekf_for_amcl_->Update();
  }
  ekf_for_amcl_->GetCurrentState(&current_pose);
  pthread_mutex_unlock(&odom_data_for_amcl_mutex_);
  return current_pose;
}

void LocationManagerImpl::_updateAmclPos() {
  Position current_pos, amcl_pos, qr_pos, land_mark_pos;
  current_pos = _refreshOdom(false);
  {
    _calDaltaPos(amcl_last_pos_, current_pos, amcl_current_pos_, &amcl_pos);
    // if (amcl_current_pos_.mlTimestamp >= current_pos.mlTimestamp) {
    //     SLAM_INFO("ladar time %ld > odom time %ld",
    //               amcl_current_pos_.mlTimestamp,
    //               current_pos.mlTimestamp);
    // } else {
    //   SLAM_INFO("ladar to now time delta %ld",
    //             current_pos.mlTimestamp - amcl_current_pos_.mlTimestamp);
    // }
  }

  {
    std::lock_guard<std::recursive_mutex> lk(reflector_pos_data_mutex_);
    _calDaltaPos(reflector_last_pos_, current_pos, reflector_current_pos_,
                 &land_mark_pos);
  }
  std::map<LocationType, DataFusion::PoseInfo> pos_candidate;
  pos_candidate.insert(std::make_pair(
      LocationType::AMCL,
      DataFusion::PoseInfo(
          Eigen::Vector3f(amcl_pos.mfX, amcl_pos.mfY, amcl_pos.mfTheta),
          amcl_pos.mlTimestamp, 1.0, LocationType::AMCL)));
  pos_candidate.insert(std::make_pair(
      LocationType::QR,
      DataFusion::PoseInfo(Eigen::Vector3f(current_pos.mfX, current_pos.mfY,
                                           current_pos.mfTheta),
                           current_pos.mlTimestamp, 1.0, LocationType::QR)));
  pos_candidate.insert(std::make_pair(
      LocationType::LandMarkLocation,
      DataFusion::PoseInfo(Eigen::Vector3f(land_mark_pos.mfX, land_mark_pos.mfY,
                                           land_mark_pos.mfTheta),
                           land_mark_pos.mlTimestamp, 1.0,
                           LocationType::LandMarkLocation)));
  DataFusion::PoseInfo result_info;
  location_data_fusion_cal_->GetLocationResult(pos_candidate, &result_info);
  Position result;
  result.mfX = result_info.pose_info(0);
  result.mfY = result_info.pose_info(1);
  result.mfTheta = result_info.pose_info(2);
  result.mlTimestamp = result_info.time_stamp;
  qr_location_->SetInitPos(result);
  uint64_t current_time = gomros::common::GetCurrentTime_us();
  SLAM_INFO("amcl update pose %f %f %f delta time %ld", result.mfX, result.mfY,
            result.mfTheta, current_time - result.mlTimestamp);

  {
    std::lock_guard<std::recursive_mutex> lk(reflector_pos_data_mutex_);
    reflector_current_pos_ = result;
    reflector_last_pos_ = result;
  }
  {
    amcl_last_pos_ = result;
    amcl_current_pos_ = result;
  }
}

void LocationManagerImpl::_updateReflectorPos() {
  Position current_pos, amcl_pos, qr_pos, land_mark_pos;
  current_pos = _refreshOdom(false);
  {
    pthread_mutex_lock(&amcl_pos_data_mutex_);
    _calDaltaPos(amcl_last_pos_, current_pos, amcl_current_pos_, &amcl_pos);
    pthread_mutex_unlock(&amcl_pos_data_mutex_);
  }
  {
    std::lock_guard<std::recursive_mutex> lk(reflector_pos_data_mutex_);
    _calDaltaPos(reflector_last_pos_, current_pos, reflector_current_pos_,
                 &land_mark_pos);
  }
  std::map<LocationType, DataFusion::PoseInfo> pos_candidate;
  pos_candidate.insert(std::make_pair(
      LocationType::AMCL,
      DataFusion::PoseInfo(
          Eigen::Vector3f(amcl_pos.mfX, amcl_pos.mfY, amcl_pos.mfTheta),
          amcl_pos.mlTimestamp, 1.0, LocationType::AMCL)));
  pos_candidate.insert(std::make_pair(
      LocationType::QR,
      DataFusion::PoseInfo(Eigen::Vector3f(current_pos.mfX, current_pos.mfY,
                                           current_pos.mfTheta),
                           current_pos.mlTimestamp, 1.0, LocationType::QR)));
  pos_candidate.insert(std::make_pair(
      LocationType::LandMarkLocation,
      DataFusion::PoseInfo(Eigen::Vector3f(land_mark_pos.mfX, land_mark_pos.mfY,
                                           land_mark_pos.mfTheta),
                           land_mark_pos.mlTimestamp, 1.0,
                           LocationType::LandMarkLocation)));
  DataFusion::PoseInfo result_info;
  std::map<LocationType, int> types =
      location_data_fusion_cal_->GetLocationResult(pos_candidate, &result_info);

  Position result;
  result.mfX = result_info.pose_info(0);
  result.mfY = result_info.pose_info(1);
  result.mfTheta = result_info.pose_info(2);
  result.mlTimestamp = result_info.time_stamp;
  qr_location_->SetInitPos(result);
  if (types.count(LocationType::AMCL)) {
    if (types.at(LocationType::AMCL) != 0) {
      // 即将进入AMCL区域 准备好切换
      grid_location_->SetGlobalLocationPos(result);
    }
  }
  {
    std::lock_guard<std::recursive_mutex> lk(reflector_pos_data_mutex_);
    reflector_current_pos_ = result;
    reflector_last_pos_ = result;
  }
  {
    pthread_mutex_lock(&amcl_pos_data_mutex_);
    amcl_last_pos_ = result;
    amcl_current_pos_ = result;
    pthread_mutex_unlock(&amcl_pos_data_mutex_);
  }
}

void LocationManagerImpl::_updateQrPos(const Position &qr_pos) {
  Position current_pos, amcl_pos, land_mark_pos;
  qr_location_->GetCurrentPose(&current_pos);
  {
    pthread_mutex_lock(&amcl_pos_data_mutex_);
    _calDaltaPos(amcl_last_pos_, current_pos, amcl_current_pos_, &amcl_pos);
    pthread_mutex_unlock(&amcl_pos_data_mutex_);
  }

  {
    std::lock_guard<std::recursive_mutex> lk(reflector_pos_data_mutex_);
    _calDaltaPos(reflector_last_pos_, current_pos, reflector_current_pos_,
                 &land_mark_pos);
  }

  std::map<LocationType, DataFusion::PoseInfo> pos_candidate;
  pos_candidate.insert(std::make_pair(
      LocationType::AMCL,
      DataFusion::PoseInfo(
          Eigen::Vector3f(amcl_pos.mfX, amcl_pos.mfY, amcl_pos.mfTheta),
          amcl_pos.mlTimestamp, 1.0, LocationType::AMCL)));
  pos_candidate.insert(std::make_pair(
      LocationType::QR,
      DataFusion::PoseInfo(
          Eigen::Vector3f(qr_pos.mfX, qr_pos.mfY, qr_pos.mfTheta),
          qr_pos.mlTimestamp, 1.0, LocationType::QR)));
  pos_candidate.insert(std::make_pair(
      LocationType::LandMarkLocation,
      DataFusion::PoseInfo(Eigen::Vector3f(land_mark_pos.mfX, land_mark_pos.mfY,
                                           land_mark_pos.mfTheta),
                           land_mark_pos.mlTimestamp, 1.0,
                           LocationType::LandMarkLocation)));
  DataFusion::PoseInfo result_info;
  std::map<LocationType, int> types =
      location_data_fusion_cal_->GetLocationResult(pos_candidate, &result_info,
                                                   true);
  Position result;
  result.mfX = result_info.pose_info(0);
  result.mfY = result_info.pose_info(1);
  result.mfTheta = result_info.pose_info(2);
  result.mlTimestamp = result_info.time_stamp;
  qr_location_->SetInitPos(result);
  if (types.count(LocationType::AMCL)) {
    if (types.at(LocationType::AMCL) != 0) {
      // 即将进入AMCL区域 准备好切换
      grid_location_->SetGlobalLocationPos(result);
    }
  }
  {
    std::lock_guard<std::recursive_mutex> lk(reflector_pos_data_mutex_);
    reflector_current_pos_ = result;
    reflector_last_pos_ = result;
  }
  {
    pthread_mutex_lock(&amcl_pos_data_mutex_);
    amcl_last_pos_ = result;
    amcl_current_pos_ = result;
    pthread_mutex_unlock(&amcl_pos_data_mutex_);
  }
}

void LocationManagerImpl::_calDaltaPos(const Position &last_pos,
                                       const Position &now_pos,
                                       const Position &now_amcl_pos,
                                       Position *current_pose) {
  assert(last_pos.mlTimestamp == now_amcl_pos.mlTimestamp);
  if (now_pos.mlTimestamp <= last_pos.mlTimestamp) {
    *current_pose = now_amcl_pos;
    return;
  }
  // assert(now_pos.mlTimestamp >= last_pos.mlTimestamp);
  Eigen::Matrix3d now_odom_pos_mat(3, 3);
  now_odom_pos_mat << cos(now_pos.mfTheta), -sin(now_pos.mfTheta), now_pos.mfX,
      sin(now_pos.mfTheta), cos(now_pos.mfTheta), now_pos.mfY, 0, 0, 1;
  Eigen::Matrix3d last_odom_pos_mat(3, 3);
  last_odom_pos_mat << cos(last_pos.mfTheta), -sin(last_pos.mfTheta),
      last_pos.mfX, sin(last_pos.mfTheta), cos(last_pos.mfTheta), last_pos.mfY,
      0, 0, 1;
  Eigen::Matrix3d delta_pos_mat(3, 3);
  delta_pos_mat = last_odom_pos_mat.inverse() * now_odom_pos_mat;
  Eigen::Matrix3d amcl_pos_mat(3, 3);
  amcl_pos_mat << cos(now_amcl_pos.mfTheta), -sin(now_amcl_pos.mfTheta),
      now_amcl_pos.mfX, sin(now_amcl_pos.mfTheta), cos(now_amcl_pos.mfTheta),
      now_amcl_pos.mfY, 0, 0, 1;
  Eigen::Matrix3d now_amcl_pos_mat(3, 3);
  now_amcl_pos_mat = amcl_pos_mat * delta_pos_mat;
  current_pose->mfX = now_amcl_pos_mat(0, 2);
  current_pose->mfY = now_amcl_pos_mat(1, 2);
  current_pose->mfTheta = SLAMMath::NormalizePITheta(
      atan2(now_amcl_pos_mat(1, 0), now_amcl_pos_mat(0, 0)));
  current_pose->mlTimestamp = now_pos.mlTimestamp;
  return;
}

bool LocationManagerImpl::_isPosJump(const Position &predictive_pos,
                                     const Position &real_pos) {
  // static FILE* jump_file = fopen("./jump_time.txt", "w+");

  if (predictive_pos.mlTimestamp != real_pos.mlTimestamp) {
    // SLAM_ERROR("not same time %d",
    //   predictive_pos.mlTimestamp - real_pos.mlTimestamp);
  }
  if (fabs(predictive_pos.mfX - real_pos.mfX) > 0.1 ||
      fabs(predictive_pos.mfY - real_pos.mfY) > 0.1 ||
      fabs(SLAMMath::NormalizeTheta(predictive_pos.mfTheta -
                                    real_pos.mfTheta)) > 0.1) {
    SLAM_WARN("position jump %f %f %f -> %f %f %f time %ld", predictive_pos.mfX,
              predictive_pos.mfY, predictive_pos.mfTheta, real_pos.mfX,
              real_pos.mfY, real_pos.mfTheta, real_pos.mlTimestamp);
    // fprintf(jump_file, "%ld\n", real_pos.mlTimestamp);
    return true;
  }
  return false;
}

/**
 * @brief 监控初始定位是否完成，完成则更新当期位姿
 *
 */
void LocationManagerImpl::_doGlobalLocateMonitor() {
  is_finish_global_location = false;
  int count = 0;
  while (radar_disconnect) {
    count++;
    if (count > 100) {
      SLAM_WARN("ladar disconnect !!!! please wait.....\n");
      count = 0;
    }
    usleep(100000);
  }
  Position locate_pose;
  if (location_config_.location_type == LocationType::WanJiLandMarkLocation ||
      location_config_.location_type == LocationType::WanJiAndQR) {
    while (!wan_ji_location_->IsFinishLocate()) {
      usleep(200000);
    }
    wan_ji_location_->GetCurrentPosition(&locate_pose);
    SLAM_INFO("finish WanJi Global Location~~~~~");
  } else if (location_config_.location_type == LocationType::LandMarkLocation ||
             location_config_.location_type == LocationType::Trilateral) {
    while (!reflector_location_->IsFinishLocate()) {
      usleep(200000);
    }
    reflector_location_->GetCurrentPosition(&locate_pose);
    SLAM_INFO("finish reflector Global Location~~~~~");
  } else {
    while (!grid_location_->IsFinishLocate()) {
      usleep(200000);
    }
    grid_location_->GetCurrentPosition(&locate_pose);
    SLAM_INFO("finish AMCL Global Location~~~~~");
  }

#ifndef TEST_DEBUG
  locate_pose.mlTimestamp = gomros::common::GetCurrentTime_us();
#endif

  location_last_pose_ = locate_pose;

  {
    pthread_mutex_lock(&amcl_pos_data_mutex_);
    amcl_current_pos_ = locate_pose;
    amcl_last_pos_ = locate_pose;
    pthread_mutex_unlock(&amcl_pos_data_mutex_);
  }

  {
    std::lock_guard<std::recursive_mutex> lk(reflector_pos_data_mutex_);
    reflector_current_pos_ = locate_pose;
    reflector_last_pos_ = locate_pose;
  }

  Eigen::Matrix3d covariance;
  covariance.setZero(3, 3);
  qr_location_->SetInitPos(locate_pose);
  ekf_for_amcl_->SetInitPos(locate_pose, covariance);
  reflector_location_->SetGlobalLocationPos(locate_pose);
  SLAM_INFO("global location pos(%f, %f, %f)\n", locate_pose.mfX,
            locate_pose.mfY, locate_pose.mfTheta);
  device_state_->mcChassisState = SYSTEM_STATE_FREE;
#ifndef TEST_DEBUG
  pthread_mutex_lock(&odom_data_mutex_);
  odom_list_.clear();
  odom_list_for_amcl_.clear();
  pthread_mutex_unlock(&odom_data_mutex_);
#endif

  is_finish_global_location = true;
  is_finished_global_location_ = true;
}

gomros::message::RadarSensoryInfo LocationManagerImpl::GetLaserData() {
  return grid_location_->GetLaserData();
}
gomros::message::RadarSensoryInfo LocationManagerImpl::GetParticleData() {
  return grid_location_->GetParticleData();
}
gomros::message::Position LocationManagerImpl::GetCurrentPos() {
  return grid_location_->GetCurrentPose();
}

cv::Mat LocationManagerImpl::GetImage() { return grid_location_->GetImage(); }

}  // namespace mapping_and_location
}  // namespace data_process
}  // namespace gomros
