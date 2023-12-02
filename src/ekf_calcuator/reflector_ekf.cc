/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.2
 * @Author: renjy
 * @Date: 2023-06-02 02:02:50
 * @LastEditTime: 2023-10-13 15:28:43
 */
#include <iostream>
#include <cfloat>
#include <map>
#include <set>

#include "ekf_calcuator/reflector_ekf.h"
#include "landmark_tool/landmark_center_calcuator.h"
#include "include/mapping_and_location_math.h"


namespace DataFusion {

void Observation::GetObsFromLadarData(const MAL::SensorMount& sensor_mount,
  const MAL::LandMarkConfig& landmark_config,
  const gomros::message::RadarSensoryInfo& radar_info) {
  centers.clear();
  float ladar_theta = sensor_mount.radar_position_theta;
  float ladar_x = sensor_mount.radar_position_x;
  float ladar_y = sensor_mount.radar_position_y;
  LandMark::Calcuator land_mark_calcuator(landmark_config);

  if (landmark_config.recognition_type == MAL::RecognitionType::Cluster) {
    // 根据设定半径判断是否为反光板
    int point_size =
    static_cast<int>(radar_info.mstruSingleLayerData.mvPoints.size());
    land_mark_calcuator.Init();
    bool is_have_landmark_data = false;
    std::vector<LandMark::Point> candidate_points;
    for (int i = 0; i < point_size; i++) {
      LandMark::Point candidate_point(
                      radar_info.mstruSingleLayerData.mvPoints[i](0),
                      radar_info.mstruSingleLayerData.mvPoints[i](1),
                      radar_info.mstruSingleLayerData.mvIntensities[i]);
      candidate_points.push_back(candidate_point);
      is_have_landmark_data = true;
    }
    land_mark_calcuator.AddValidPoints(candidate_points);
    time_ = radar_info.mstruRadarHeaderData.mlTimeStamp;
    centers.clear();

    if (!is_have_landmark_data) return;
    land_mark_ =
      land_mark_calcuator.GetLandMarkCenter(true);
    if (land_mark_.empty()) return;

    for (auto iter = land_mark_.begin(); iter != land_mark_.end(); iter++) {
      if (!iter->second.is_cal_center) continue;
      Eigen::Vector2d center;
      center(0) = iter->second.mx * cos(ladar_theta) -
                  iter->second.my * sin(ladar_theta) + ladar_x;
      center(1) = iter->second.mx * sin(ladar_theta) +
                  iter->second.my * cos(ladar_theta) + ladar_y;
      centers.push_back(center);
    }
  } else {
    time_ = radar_info.mstruRadarHeaderData.mlTimeStamp;
    centers.clear();
    std::vector<Eigen::Vector2d> land_marks1 =
      LandMark::CalLandMarkCentor(radar_info,
      landmark_config);
    for (auto iter = land_marks1.begin(); iter != land_marks1.end(); iter++) {
      Eigen::Vector2d center;
      center(0) = (*iter)(0) * cos(ladar_theta) -
                  (*iter)(1) * sin(ladar_theta) + ladar_x;
      center(1) = (*iter)(0) * sin(ladar_theta) +
                  (*iter)(1) * cos(ladar_theta) + ladar_y;
      centers.push_back(center);
    }
  }
  return;
}


ReflectorEkfCalcuator::ReflectorEkfCalcuator(
  const MAL::EKFConfig& ekf_config,
  const MAL::SensorMount& sensor_mount_config,
  const MAL::LandMarkConfig& landmark_config, bool is_mapping)
: ekf_config_(ekf_config), sensor_mount_config_(sensor_mount_config),
  landmark_config_(landmark_config), is_mapping_(is_mapping) {
  pthread_mutex_init(&state_mutex_, nullptr);
  pthread_mutex_init(&odom_mutex_, nullptr);
  pthread_mutex_init(&imu_mutex_, nullptr);
  show_map_id_.clear();
  config_mat_ <<
    1.0f, 0.0f, -sensor_mount_config_.back_wheel_length *
              sin(sensor_mount_config_.back_wheel_alpha),
    0.0f, 1.0f, -sensor_mount_config_.back_wheel_length *
              cos(sensor_mount_config_.back_wheel_alpha),
    1.0f, 0.0f, sensor_mount_config_.front_wheel_length *
              sin(sensor_mount_config_.front_wheel_alpha),
    0.0f, 1.0f, sensor_mount_config_.front_wheel_length *
              cos(sensor_mount_config_.front_wheel_alpha);
  double observation_error_2 =
          std::pow(landmark_config_.observation_error, 2);
  Qt_.setZero(2, 2);
  Qt_ << observation_error_2, 0,
        0, observation_error_2;
  is_get_location_pos_ = false;
  get_init_state_ = false;
  state_.Init();
  display_reflector_ekf_ = std::make_shared<
  DisPlayResult::SimulationDataPublisher>(sensor_mount_config_);
  get_odom_data_ = false;
}

void ReflectorEkfCalcuator::Copy(const ReflectorEkfCalcuator &obj) {}

void ReflectorEkfCalcuator::SetInitState(const State& state) {
  pthread_mutex_lock(&state_mutex_);
  state_ = state;
  pthread_mutex_unlock(&state_mutex_);
  get_init_state_ = true;
  is_get_location_pos_ = true;
  pthread_mutex_lock(&odom_mutex_);
  odom_list_.clear();
  get_odom_data_ = false;
  pthread_mutex_unlock(&odom_mutex_);
}

void ReflectorEkfCalcuator::SetMapDate(const ReflectorMap& map_info) {
  reflector_map_info_ = map_info;
}


void ReflectorEkfCalcuator::HandleOdomData(const OdometerMessage& data) {
  if (!get_init_state_) return;
  pthread_mutex_lock(&odom_mutex_);
  odom_list_.push_back(data);
  last_odom_data_ = odom_list_.front();
  get_odom_data_ = true;
  pthread_mutex_unlock(&odom_mutex_);
}


void ReflectorEkfCalcuator::HandleImuData(const ImuSensoryMessage& data) {
  if (!get_init_state_) return;
  pthread_mutex_lock(&imu_mutex_);
  if (imu_list_.size() > 0) {
     if (data.time_stamp <= imu_list_.back().time_stamp) {
        pthread_mutex_unlock(&imu_mutex_);
        return;
     }
  }
  imu_list_.push_back(data);
  pthread_mutex_unlock(&imu_mutex_);
  return;
}


void ReflectorEkfCalcuator::HandleObsData(const Observation& obs) {
  is_get_location_pos_ = false;
  // 刷新里程计
  _refreshOdom(obs.time_);
  // assert(state_.time == obs.time_);
  ReflectorMatchResult result = _calReflectorMatch(obs);
  _update(obs, result);
  return;
}

bool ReflectorEkfCalcuator::GetState(State* state) {
  pthread_mutex_lock(&state_mutex_);
  *state = state_;
  pthread_mutex_unlock(&state_mutex_);
  return is_get_location_pos_;
}


ReflectorMatchResult ReflectorEkfCalcuator::_calReflectorMatch(
  const Observation& obs) {
  ReflectorMatchResult reflector_match_result;
  reflector_match_result.map_obs_match_ids.clear();
  reflector_match_result.state_obs_match_ids.clear();
  reflector_match_result.new_ids.clear();
  if (obs.centers.empty()) {
    SLAM_INFO("there is no observation");
    return reflector_match_result;
  }
  // 建图时第一帧观测全部写入状态空间中
  // 传入地图中已有的反光柱个数
  const int reflector_map_size = reflector_map_info_.reflectors_.size();
  const int state_obs_size = (state_.mu.rows() - 3) / 2;
  if (state_obs_size == 0 && is_mapping_) {
    for (int i = 0; i < obs.centers.size(); ++i) {
      reflector_match_result.new_ids.push_back(i);
      int size = state_.obs_in_map_count.size();
      state_.obs_in_map_count.insert(std::make_pair(size, 1));
    }
    return reflector_match_result;
  }
  // 根据当前机器人位姿，找出地图中一定范围内的反光柱
  std::vector<int> index_for_data_association;
  for (int i = 0; i < reflector_map_size; i++) {
    float dist = SLAMMath::Dist(reflector_map_info_.reflectors_[i](0),
                                reflector_map_info_.reflectors_[i](1),
                                 state_.mu(0), state_.mu(1));
    // 增加角度上的数据关联
    float landmark_theta =
      atan2(reflector_map_info_.reflectors_[i](1) - state_.mu(1),
            reflector_map_info_.reflectors_[i](0) - state_.mu(0));
    float delta_theta =
      fabs(SLAMMath::NormalizePITheta(landmark_theta - state_.mu(2)));
    // if (dist < landmark_config_.data_association_max_dis &&
    //    delta_theta < 0.75 * M_PI) {
    if (dist < landmark_config_.data_association_max_dis) {
      index_for_data_association.push_back(i);
    }
  }
  if (index_for_data_association.empty() &&
      reflector_map_size == 0 && !is_mapping_) {
    SLAM_WARN("data association failed........");
    return reflector_match_result;
  }
  // agv pose cov
  Eigen::Matrix3d pose_sigma = state_.sigma.block(0, 0, 3, 3);
  for (int i = 0; i < obs.centers.size(); ++i) {
    // Match with old map_
    if (!is_mapping_) {
      if (_isMatchToMap(obs.centers[i], i, index_for_data_association,
                        &reflector_match_result)) continue;
    } else {
      // Match with state 定位时暂时不增加另外识别到的反光柱
      if (_isMatchToState(obs.centers[i], i, &reflector_match_result)) continue;
      reflector_match_result.new_ids.push_back(i);
      int size = state_.obs_in_map_count.size();
      state_.obs_in_map_count.insert(std::make_pair(size, 1));
    }
  }

  int map_obs = reflector_match_result.map_obs_match_ids.size();
  int state_obs = reflector_match_result.state_obs_match_ids.size();
  int new_obs = reflector_match_result.new_ids.size();
  for (int i = 0; i < state_obs; i++) {
    int state_id = reflector_match_result.state_obs_match_ids[i].match_id;
    if (state_.obs_in_map_count.count(state_id)) {
      state_.obs_in_map_count.at(state_id)++;
    }
  }
  // SLAM_DEBUG("sumcount %d, map_obs %d, state_obs %d, new_obs %d",
  //             obs.centers.size(), map_obs, state_obs, new_obs);
  return reflector_match_result;
}

void ReflectorEkfCalcuator::_update(const Observation& observation,
  const ReflectorMatchResult& result) {
  std::vector<int> obs_match_id;
  show_map_id_.clear();
  const int map_match_size = result.map_obs_match_ids.size();
  const int obs_match_size = result.state_obs_match_ids.size();
  // SLAM_INFO("obs size is %d", observation.centers.size());
  const int whole_match_size = obs_match_size + map_match_size;
  const int new_obs_size = result.new_ids.size();
  // SLAM_DEBUG("obs_size %d map_match %d, state_match %d new_id %d",
  //            observation.centers.size(),
  //            map_match_size, obs_match_size, new_obs_size);
  const int N = state_.mu.rows();  // 状态空间的行数
  if (whole_match_size > 1) {
    // N是状态空间的维度
    Eigen::MatrixXd H_t = Eigen::MatrixXd::Zero(2 * whole_match_size, N);
    // 实际观测值
    Eigen::VectorXd zt = Eigen::VectorXd::Zero(2 * whole_match_size);
    // 预测观测值
    Eigen::VectorXd zt_hat = Eigen::VectorXd::Zero(2 * whole_match_size);
    // 观测协方差
    Eigen::MatrixXd Q =
      Eigen::MatrixXd::Zero(2 * whole_match_size, 2 * whole_match_size);
    const double cos_theta = std::cos(state_.mu(2));
    const double sin_theta = std::sin(state_.mu(2));
    Eigen::Matrix2d B;
    B << cos_theta, sin_theta, -sin_theta, cos_theta;
    if (obs_match_size > 0) {  // 状态空间
      // lamda表达式,返回状态空间中反光柱的位置
      const auto xy = [&](const int &id) -> Eigen::Vector2d {
          return Eigen::Vector2d(state_.mu(3 + 2 * id),
                                 state_.mu(3 + 2 * id + 1));
      };
      for (int i = 0; i < obs_match_size; ++i) {
        const int local_id = result.state_obs_match_ids[i].obs_id;
        const int global_id = result.state_obs_match_ids[i].match_id;
        obs_match_id.push_back(global_id);
        // SLAM_DEBUG("local_id %d, global_id %d", local_id, global_id);
        // 实际观测值
        zt(2 * i) = observation.centers[local_id](0);
        zt(2 * i + 1) = observation.centers[local_id](1);
        const double delta_x = xy(global_id)(0) - state_.mu(0);
        const double delta_y = xy(global_id)(1) - state_.mu(1);
        // 预测观测值
        zt_hat(2 * i) = delta_x * cos_theta + delta_y * sin_theta;
        zt_hat(2 * i + 1) = -delta_x * sin_theta + delta_y * cos_theta;
        Eigen::MatrixXd A_i(2, 3);
        A_i << -cos_theta, -sin_theta,
              -delta_x * sin_theta + delta_y * cos_theta,
              sin_theta, -cos_theta,
              -delta_x * cos_theta - delta_y * sin_theta;
        H_t.block(2 * i, 0, 2, 3) = A_i;
        H_t.block(2 * i, 3 + 2 * global_id, 2, 2) = B;
        Q.block(2 * i, 2 * i, 2, 2) = Qt_;  // 观测协方差
      }
    }
    if (map_match_size > 0) {
      const auto xy = [&](const int &id) -> Eigen::Vector2d {
        return reflector_map_info_.reflectors_[id];
      };

      for (int i = 0; i < map_match_size; ++i) {
        const int local_id = result.map_obs_match_ids[i].obs_id;
        const int global_id = result.map_obs_match_ids[i].match_id;
        zt(2 * (obs_match_size + i)) = observation.centers[local_id](0);
        zt(2 * (obs_match_size + i) + 1) = observation.centers[local_id](1);
        const double delta_x = xy(global_id)(0) - state_.mu(0);
        const double delta_y = xy(global_id)(1) - state_.mu(1);
        // for test
        // SLAM_DEBUG("local_id %d, global_id %d", local_id, global_id);
        show_map_id_.push_back(global_id);
        zt_hat(2 * (obs_match_size + i)) =
          delta_x * cos_theta + delta_y * sin_theta;
        zt_hat(2 * (obs_match_size + i) + 1) =
          -delta_x * sin_theta + delta_y * cos_theta;

        Eigen::MatrixXd A_i(2, 3);
        A_i << -cos_theta, -sin_theta,
              -delta_x * sin_theta + delta_y * cos_theta,
              sin_theta, -cos_theta,
              -delta_x * cos_theta - delta_y * sin_theta;
        H_t.block(2 * (obs_match_size + i), 0, 2, 3) = A_i;
        Q.block(2 * (obs_match_size + i), 2 * (obs_match_size + i), 2, 2) = Qt_;
      }
    }
    const auto K_t = state_.sigma * H_t.transpose() *
                    ((H_t * state_.sigma * H_t.transpose() + Q).inverse());
    {
      pthread_mutex_lock(&state_mutex_);
      float x = state_.mu(0);
      float y = state_.mu(1);
      float t = state_.mu(2);
      state_.mu += K_t * (zt - zt_hat);
      state_.mu(2) = SLAMMath::NormalizePITheta(state_.mu(2));

      state_.sigma = state_.sigma - K_t * H_t * state_.sigma;
      if (state_.sigma(0, 0) > 1.0) {
        std::cout << state_.sigma << std::endl;
      }

      state_.time = observation.time_;
      if (fabs(state_.mu(0) - x) > 0.1 || fabs(state_.mu(1) - y) > 0.1 ||
          fabs(state_.mu(2) - t) > 0.087) {
        for (int i = 0; i < map_match_size; i++) {
          float aa = (zt - zt_hat)(2 * (obs_match_size + i));
          float bb = (zt - zt_hat)(2 * (obs_match_size + i) + 1);
          SLAM_ERROR("map_match zt - zt_hat %f %f", aa, bb);
        }
        for (int i = 0; i < obs_match_size; i++) {
          float aa = (zt - zt_hat)(2 * i);
          float bb = (zt - zt_hat)(2 * i + 1);
          SLAM_ERROR("obs_match zt - zt_hat %f %f", aa, bb);
        }
        // std::cout << "K_t " << K_t << std::endl;
        // std::cout << "H_t " << H_t << std::endl;
        // std::cout << "Q " << Q << std::endl;
        // std::cout << "state_.sigma " << state_.sigma << std::endl;
        // std::cout << "state_.mu " << state_.mu << std::endl;

        //  SLAM_ERROR("obs_match_size %d map_match_size %d, delta (%f %f %f)",
        //             obs_match_size, map_match_size, state_.mu(0) - x,
        //             state_.mu(1) - y, (state_.mu(2) - t) * 180 / M_PI);
      }
      is_get_location_pos_ = true;
      pthread_mutex_unlock(&state_mutex_);
    }
  }
  if (new_obs_size > 0) {
    const int M_e = N + 2 * new_obs_size;
    Eigen::VectorXd tmp_xe = Eigen::VectorXd::Zero(M_e);
    tmp_xe.topRows(N) = state_.mu.topRows(N);

    Eigen::MatrixXd tmp_sigma = Eigen::MatrixXd::Zero(M_e, M_e);
    tmp_sigma.block(0, 0, N, N) = state_.sigma;
    // 原协方差矩阵的前三行前三列
    const Eigen::Matrix3d sigma_xi = state_.sigma.block(0, 0, 3, 3);
    const double sin_theta = std::sin(state_.mu(2));
    const double cos_theta = std::cos(state_.mu(2));
    Eigen::Matrix2d G_zi;
    G_zi << cos_theta, -sin_theta, sin_theta, cos_theta;
    Eigen::MatrixXd G_p(2 * new_obs_size, 3);
    Eigen::MatrixXd G_z =
      Eigen::MatrixXd::Zero(2 * new_obs_size, 2 * new_obs_size);
    Eigen::MatrixXd Q =
      Eigen::MatrixXd::Zero(2 * new_obs_size, 2 * new_obs_size);
    Eigen::MatrixXd G_fx = Eigen::MatrixXd::Zero(2 * new_obs_size, N);

    for (int i = 0; i < new_obs_size; i++) {
      const int local_id = result.new_ids[i];
      const auto point = _ladar2GlobalFrame(observation.centers[local_id]);
      // update state vector
      tmp_xe(N + 2 * i) = point(0);
      tmp_xe(N + 2 * i + 1) = point(1);
      // for update cov
      const double rx = observation.centers[local_id](0);
      const double ry = observation.centers[local_id](1);
      Eigen::Matrix<double, 2, 3> Gp_i;
      Gp_i << 1., 0., -rx * sin_theta - ry * cos_theta,
              0., 1., rx * cos_theta - ry * sin_theta;
      G_p.block(2 * i, 0, 2, 3) = Gp_i;
      G_z.block(2 * i, 0, 2, 2) = G_zi;

      Q.block(2 * i, 2 * i, 2, 2) = Qt_;
      Eigen::MatrixXd G_fx_i = Eigen::MatrixXd::Zero(2, N);
      G_fx_i.topLeftCorner(2, 3) = Gp_i;
      G_fx.block(2 * i, 0, 2, N) = G_fx_i;
    }
    const auto sigma_mm =
      G_p * sigma_xi * G_p.transpose() + G_z * Q * G_z.transpose();
    const auto sigma_mx = G_fx * state_.sigma;
    tmp_sigma.block(N, 0, 2 * new_obs_size, N) = sigma_mx;
    tmp_sigma.block(0, N, N, 2 * new_obs_size) = sigma_mx.transpose();
    tmp_sigma.block(N, N, 2 * new_obs_size, 2 * new_obs_size) = sigma_mm;

    pthread_mutex_lock(&state_mutex_);
    state_.sigma.resize(M_e, M_e);
    state_.sigma = tmp_sigma;
    if (state_.sigma(0, 0) > 1.0) {
      std::cout << state_.sigma << std::endl;
    }
    state_.mu.resize(M_e);
    state_.mu = tmp_xe;
    pthread_mutex_unlock(&state_mutex_);
  }
#ifdef TEST_DEBUG
  display_reflector_ekf_->DisPlayReflectorEkf(
    state_.mu, new_obs_size, obs_match_id);
#endif
  return;
}

bool ReflectorEkfCalcuator::_isMatchToState(const Eigen::Vector2d& obs,
  int obs_id, ReflectorMatchResult* reflector_match_result) {
  // 将反光板转换到全局坐标系下
  const auto reflector = _ladar2GlobalFrame(obs);
  const int state_obs_size = (state_.mu.rows() - 3) / 2;
  int association_id = -1;
  float min_distance = FLT_MAX;
  bool is_same_state = false;
  float theta_theshold = 1.0 * M_PI / 180.0;
  float min_eu_dis = 0.05 + std::sin(theta_theshold) * 2 * SLAMMath::Dist(
        reflector(0), reflector(1), state_.mu(0), state_.mu(1));

  for (int j = 0; j < state_obs_size; ++j) {
    // 状态空间中第j个反光板
    Eigen::Vector2d global_reflector(state_.mu(3 + 2 * j),
                                      state_.mu(3 + 2 * j + 1));
    // agv pose cov
    Eigen::Matrix3d pose_sigma = state_.sigma.block(0, 0, 3, 3);
    // obs matrix
    Eigen::Matrix<double, 2, 3> Ht;
    Ht << -std::cos(state_.mu(2)), -std::sin(state_.mu(2)),

        -(global_reflector.x() - state_.mu(0)) * std::sin(state_.mu(2)) +
        (global_reflector.y() - state_.mu(1)) * std::cos(state_.mu(2)),
        std::sin(state_.mu(2)),
        -std::cos(state_.mu(2)),
        -(global_reflector.x() - state_.mu(0)) * std::cos(state_.mu(2)) -
        (global_reflector.y() - state_.mu(1)) * std::sin(state_.mu(2));

    Eigen::Matrix2d St = Ht * pose_sigma * Ht.transpose() + Qt_;

    // 预测的局部坐标系下的值
    Eigen::Vector2d pridect_reflector;
    pridect_reflector(0) = (global_reflector.x() - state_.mu(0)) *
                            std::cos(state_.mu(2)) +
                            (global_reflector.y() - state_.mu(1)) *
                            std::sin(state_.mu(2));
    pridect_reflector(1) = -(global_reflector.x() - state_.mu(0)) *
                            std::sin(state_.mu(2)) +
                            (global_reflector.y() - state_.mu(1)) *
                            std::cos(state_.mu(2));
    // 预测值-实际观测
    Eigen::Vector2d delta_state = pridect_reflector - obs;
    // Calculate Ma distance //计算马氏距离
    double ma_dist =
      std::sqrt(delta_state.transpose() * St.inverse() * delta_state);
    // Calculate Eu distance
    delta_state = reflector - global_reflector;
    double eu_dist = std::sqrt(delta_state.transpose() * delta_state);
    // if (eu_dist > 0.5) continue;
    if (eu_dist < min_eu_dis) {
      is_same_state = true;
    }
    if (ma_dist < landmark_config_.map_ma_match_dis_threshold &&
        ma_dist < min_distance) {
        min_distance = ma_dist;
        association_id = j;
        if (eu_dist > 1.0) {
          SLAM_ERROR("association error, please check!!!");
        }
        SLAM_DEBUG("association eu_dist %f ma_dist %f", eu_dist, ma_dist);
    }
  }
  // if (association_id == -1 && is_same_state) return true;
  if (association_id == -1) return false;
  for (auto it = reflector_match_result->state_obs_match_ids.begin();
      it != reflector_match_result->state_obs_match_ids.end(); it++) {
    if (it->match_id == association_id) {
      if (it->min_distance > min_distance) {
        it->obs_id = obs_id;
        it->min_distance = min_distance;
      }
      return true;
    }
  }
  reflector_match_result->state_obs_match_ids.push_back(
    MatchInfo(obs_id, association_id, min_distance));
  return true;
}

bool ReflectorEkfCalcuator::_isMatchToMap(const Eigen::Vector2d& obs,
  int obs_id, const std::vector<int>& index_for_data_association,
  ReflectorMatchResult* reflector_match_result) {
  const auto reflector = _ladar2GlobalFrame(obs);
  std::set<int> connected_id;
  float min_distance = FLT_MAX;
  int association_id = -1;
  for (int j : index_for_data_association) {
    // 地图中第j个反光板
    Eigen::Vector2d global_reflector;
    global_reflector(0) = (reflector_map_info_.reflectors_)[j](0);
    global_reflector(1) = (reflector_map_info_.reflectors_)[j](1);
    // obs matrix
    Eigen::Matrix<double, 2, 3> Ht;
    Ht <<
      -std::cos(state_.mu(2)), -std::sin(state_.mu(2)),
      -(global_reflector(0) - state_.mu(0)) * std::sin(state_.mu(2)) +
        (global_reflector(1) - state_.mu(1)) * std::cos(state_.mu(2)),
        std::sin(state_.mu(2)), -std::cos(state_.mu(2)),
      -(global_reflector(0) - state_.mu(0)) * std::cos(state_.mu(2)) -
      (global_reflector(1) - state_.mu(1)) * std::sin(state_.mu(2));
    Eigen::Matrix2d St =
      Ht * state_.sigma.block(0, 0, 3, 3) * Ht.transpose() + Qt_;
    Eigen::Vector2d pridect_reflector;  // 预测的局部坐标系下的值
    Eigen::Matrix3d state_mat(3, 3);
    state_mat <<
      cos(state_.mu(2)), -sin(state_.mu(2)), state_.mu(0),
      sin(state_.mu(2)), cos(state_.mu(2)), state_.mu(1),
      0, 0, 1;
    Eigen::Matrix3d global_mat(3, 3);
    global_mat <<
      1, 0, global_reflector(0),
      0, 1, global_reflector(1),
      0, 0, 1;
    Eigen::Matrix3d pridect_mat(3, 3);
    pridect_mat = state_mat.inverse() * global_mat;
    pridect_reflector(0) = pridect_mat(0, 2);
    pridect_reflector(1) = pridect_mat(1, 2);
    // 预测值-实际观测
    Eigen::Vector2d delta_state = pridect_reflector - obs;
    // 计算马氏距离
    float ma_dist =
      std::sqrt(delta_state.transpose() * St.inverse() * delta_state);
    // 计算欧式距离
    delta_state = reflector - global_reflector;
    float eu_dist = std::sqrt(delta_state.transpose() * delta_state);
    eu_dist = SLAMMath::Dist(obs(0), obs(1),
                            pridect_reflector(0), pridect_reflector(1));
    // if (eu_dist > 0.5) continue;
    if (ma_dist < landmark_config_.map_ma_match_dis_threshold &&
        ma_dist < min_distance) {
        min_distance = ma_dist;
        association_id = j;
    }
  }
  if (association_id == -1) return false;
  for (auto it = reflector_match_result->map_obs_match_ids.begin();
      it != reflector_match_result->map_obs_match_ids.end(); it++) {
    if (it->match_id == association_id) {
      if (it->min_distance > min_distance) {
        it->obs_id = obs_id;
        it->min_distance = min_distance;
      }
      return true;
    }
  }
  reflector_match_result->map_obs_match_ids.push_back(
    MatchInfo(obs_id, association_id, min_distance));
  return true;
}


void ReflectorEkfCalcuator::_refreshOdom(uint64_t time) {
  pthread_mutex_lock(&odom_mutex_);
  while (!odom_list_.empty()) {
    last_odom_data_ = odom_list_.front();
    get_odom_data_ = true;
    if (last_odom_data_.mclDeltaPosition.mlTimestamp > time) break;
    odom_list_.pop_front();
    _calOdomMessage(last_odom_data_);
  }
  if (get_odom_data_) {
    last_odom_data_.mclDeltaPosition.mlTimestamp = time;
    _calOdomMessage(last_odom_data_);
  }
  pthread_mutex_unlock(&odom_mutex_);
  return;
}
void ReflectorEkfCalcuator::_refreshImu(const DataFusion::State& previous_state,
  const DataFusion::State& current_state) {
  pthread_mutex_lock(&imu_mutex_);
  if (imu_list_.empty() || current_state.time < previous_state.time) {
    pthread_mutex_unlock(&imu_mutex_);
    return;
  }
  double observation = 0.0f;
  float vel_theta = 0.0;
  int count = 0;
  for (auto iter = imu_list_.begin(); iter != imu_list_.end(); iter++) {
    if (iter->time_stamp < previous_state.time) continue;
    if (iter->time_stamp > current_state.time) break;
    count++;
    vel_theta += iter->z_omega;
  }
  while (!imu_list_.empty()) {
    if (imu_list_.front().time_stamp > current_state.time) break;
    imu_list_.pop_front();
  }
  pthread_mutex_unlock(&imu_mutex_);

  if (count <= 0) return;
  vel_theta = vel_theta / (count * 1.0f);
  observation = previous_state.mu(2) + vel_theta *
                (current_state.time - previous_state.time) * 1e-6;
  observation = SLAMMath::NormalizePITheta(observation);
  // 更新卡尔曼增益
  // 观测矩阵 H
  Eigen::Matrix<double, 1, 3> H = {0.0, 0.0, 1.0};
  double denominator = H * state_.sigma.block(0, 0, 3, 3) * (H.transpose()) +
                      std::pow(ekf_config_.imu_theta_noise, 2);
  Eigen::Vector3d K_k =
    state_.sigma.block(0, 0, 3, 3) * (H.transpose()) / denominator;

  Eigen::Vector3d delta_pos =
    K_k * SLAMMath::NormalizePITheta(observation - current_state.mu(2));
  delta_pos(2) = SLAMMath::NormalizePITheta(delta_pos(2));
  if (fabs(delta_pos(2)) > 0.00005) {
    SLAM_INFO("it works~~~~~~~~~~~ %f", delta_pos(2));
  }
  // 更新协方差
  Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();
  Eigen::Matrix<double, 3, 3> state_covariance =
    (identity - K_k * H) * state_.sigma.block(0, 0, 3, 3);
  // std::cout << "state_covariance " << state_covariance << std::endl;
  // std::cout << "state_.sigma "
  //           << state_.sigma.block(0, 0, 3, 3) << std::endl;

  {
    pthread_mutex_lock(&state_mutex_);
    // state_.sigma.block(0, 0, 3, 3) = state_covariance;
    state_.mu(2) = current_state.mu(2) + delta_pos(2);
    state_.mu(2) = SLAMMath::NormalizePITheta(state_.mu(2));
    state_.time = current_state.time;
    pthread_mutex_unlock(&state_mutex_);
  }
  return;
}



void ReflectorEkfCalcuator::_calOdomMessage(const OdometerMessage& data) {
  DataFusion::State previous_state = state_;
  if (MAL::OdomType::DiffWheelModel == ekf_config_.odom_type) {
    _calOdomMessageDiffWheel(data);
  } else if (MAL::OdomType::DoubleSteerModel == ekf_config_.odom_type) {
    _calOdomMessageDoubleSteer(data);
  } else if (MAL::OdomType::SingleSteerModel == ekf_config_.odom_type) {
    _calOdomMessageSingleSteer(data);
  }
  _refreshImu(previous_state, state_);
}

void ReflectorEkfCalcuator::_calOdomMessageDiffWheel(
  const OdometerMessage& data) {
  double vel_left = data.mstruDiffSteerSts.mfLeftLinearVel;
  double vel_right = data.mstruDiffSteerSts.mfRightLinearVel;
  if (data.mclDeltaPosition.mlTimestamp < state_.time) return;
  double delta_time =
      1e-6 * (data.mclDeltaPosition.mlTimestamp - state_.time);
  if (delta_time > 0.06) {
    SLAM_WARN("delta time %f %lu %lu",
              delta_time, data.mclDeltaPosition.mlTimestamp, state_.time);
  }
  float wheel_distance = ekf_config_.sensor_mount.wheel_distance;
  double delta_left_distance = vel_left * delta_time;
  double delta_right_distance = vel_right * delta_time;
  double delta_distance = 0.5 * (delta_left_distance + delta_right_distance);
  double delta_theta = SLAMMath::NormalizePITheta(
        (delta_right_distance - delta_left_distance) / wheel_distance);

  double sin_theta = sin(state_.mu(2));
  double cos_theta = cos(state_.mu(2));
  double delta_x = delta_distance * cos_theta;
  double delta_y = delta_distance * sin_theta;
  /***** 更新协方差 1.0 *****/
  /* 构造 Gt */
  // int N = state_.mu.rows();
  // double s_theta = sin(state_.mu(2));
  // double c_theta = cos(state_.mu(2));
  // Eigen::MatrixXd G_xi = Eigen::MatrixXd::Identity(N, N);
  // G_xi(0, 2) = -(vel_right + vel_left) * delta_time / 2.0 * s_theta;
  // G_xi(1, 2) = (vel_right + vel_left) * delta_time / 2.0 * c_theta;
  // /* 构造 Gu' */
  // Eigen::MatrixXd G_u = Eigen::MatrixXd::Zero(N, 2);
  // Eigen::MatrixXd G_u_2(3, 2);
  // G_u_2 <<
  // delta_time * c_theta, 0,
  // delta_time * s_theta, 0,
  // 0, delta_time;

  // // G_u_2 <<
  // //     0.5 * delta_time * c_theta, 0.5 * delta_time * c_theta,
  // //     0.5 * delta_time * s_theta, 0.5 * delta_time * s_theta,
  // //     delta_time / wheel_distance, -delta_time / wheel_distance;
  // G_u.block(0, 0, 3, 2) = G_u_2;
  // Eigen::MatrixXd control_noise_covariance = Eigen::MatrixXd::Identity(2, 2);
  // control_noise_covariance <<
  //   std::pow(ekf_config_.control_left_error, 2), 0.0,
  //   0.0, std::pow(ekf_config_.control_right_error, 2);
  // {
  //   pthread_mutex_lock(&state_mutex_);
  //   state_.sigma = G_xi * state_.sigma * G_xi.transpose() +
  //                 G_u * control_noise_covariance * G_u.transpose();
  //   /***** 更新均值 *****/
  //   state_.mu(0) += delta_x;
  //   state_.mu(1) += delta_y;
  //   state_.mu(2) += delta_theta;
  //   state_.mu(2) = SLAMMath::NormalizePITheta(state_.mu(2));
  //   state_.time = data.mclDeltaPosition.mlTimestamp;
  //   pthread_mutex_unlock(&state_mutex_);
  // }
  // return;

  // 预测 预测协方差矩阵 1.1 统一预测公式
  float v_r = 0.5 * (vel_right + vel_left);
  float w = (vel_right - vel_left) / wheel_distance;

  _calPredictedState(v_r, w, delta_time, data.mclDeltaPosition.mlTimestamp,
      Eigen::Vector3d(delta_x, delta_y, delta_theta), true);
  return;
}

void ReflectorEkfCalcuator::_calOdomMessageDoubleSteer(
  const OdometerMessage& data) {
  if (data.mclDeltaPosition.mlTimestamp < state_.time) return;
  double delta_time =
      1e-6 * (data.mclDeltaPosition.mlTimestamp - state_.time);
  if (delta_time > 0.06) {
    SLAM_WARN("delta time %f %lu %lu",
              delta_time, data.mclDeltaPosition.mlTimestamp, state_.time);
  }
  float fvel1 = data.mstruDualSteerSts.mdFrontLinearVel;
  float bvel1 = data.mstruDualSteerSts.mdBackLinearVel;
  float theta_f1 = data.mstruDualSteerSts.mdFrontRotAngle;
  float theta_b1 = data.mstruDualSteerSts.mdBackRotAngle;
  float cf1, sf1, cb1, sb1;
  cf1 = cos(theta_f1);
  sf1 = sin(theta_f1);
  cb1 = cos(theta_b1);
  sb1 = sin(theta_b1);

  Eigen::Matrix<float, 4, 1> matrix_41;
  matrix_41<<
  bvel1 * cb1, bvel1 * sb1, fvel1 * cf1, fvel1 * sf1;

  Eigen::Matrix<float, 3, 3> matrix_33 =
    config_mat_.transpose() * config_mat_;
  Eigen::Matrix<float, 3, 4> matrix_btbnbt =
    matrix_33.inverse() * config_mat_.transpose();
  Eigen::Matrix<float, 3, 1> result =
    matrix_btbnbt * matrix_41;

  float vel_x = result(0);
  float vel_y = result(1);
  float w = result(2);

  float v_r = std::sqrt(std::pow(result(0), 2) + std::pow(result(1), 2));

  if (fabs(v_r) > 1.1 || fabs(w) > 0.41) {
    SLAM_ERROR("vel too fast %f %f", v_r, w);
    // return;
  }

  // Eigen::Vector3d delta_pos(delta_time * result(0), delta_time * result(1),
  //                           delta_time * result(2));
  // int N = state_.mu.rows();
  // Eigen::MatrixXd G_xi = Eigen::MatrixXd::Identity(N, N);
  // // G_xi.setZero(N, N);
  // float s_theta = std::sin(state_.mu(2));
  // float c_theta = std::cos(state_.mu(2));
  // float s_delta_sum = sin(state_.mu(2) + delta_pos(2));
  // float c_delta_sum = cos(state_.mu(2) + delta_pos(2));
  // // 直行 和 旋转进行区分
  // Eigen::MatrixXd G_xi_tmp = Eigen::MatrixXd::Zero(3, 3);
  // G_xi_tmp <<
  //     1, 0, -delta_pos(0) * s_theta - delta_pos(1) * c_theta,
  //     0, 1, delta_pos(0) * c_theta - delta_pos(1) * s_theta,
  //     0, 0, 1;

  // G_xi.block(0, 0, 3, 3) = G_xi_tmp;
  // // 控制雅克比矩阵 state_/u[v_x, v_y, w]
  // Eigen::MatrixXd G_u = Eigen::MatrixXd::Zero(N, 3);
  // G_u.setZero(N, 2);
  // Eigen::MatrixXd G_u_block = Eigen::MatrixXd::Zero(3, 3);
  // G_u_block <<
  //     c_theta * delta_time, -s_theta * delta_time, 0,
  //     s_theta * delta_time, c_theta * delta_time, 0,
  //     0, 0, delta_time;
  // G_u.block(0, 0, 3, 2) = G_u_block;

  // double error_2 =
  //   ekf_config_.control_left_error * ekf_config_.control_left_error;

  // Eigen::MatrixXd control_noise_covariance = Eigen::MatrixXd::Zero(3, 3);
  // control_noise_covariance <<
  //             error_2, 0, 0,
  //             0, error_2, 0,
  //             0, 0, error_2;

  // if (fabs(delta_pos(0)) > 0.15 || fabs(delta_pos(1)) > 0.15 ||
  //     fabs(delta_pos(2)) > 0.05) {
  //     SLAM_WARN("change too fast %f %f %f",
  //               delta_pos(0), delta_pos(1), delta_pos(2));
  // }

  // /* 更新协方差 */
  // {
  //   pthread_mutex_lock(&state_mutex_);
  //   state_.sigma = G_xi * state_.sigma * G_xi.transpose() +
  //                 G_u * control_noise_covariance * G_u.transpose();
  //   if (state_.sigma(0, 0) > 1.0) {
  //     std::cout << std::endl;
  //     std::cout << std::endl;
  //     std::cout << "G_xi " << G_xi << std::endl;
  //     std::cout << "state_.sigma " << state_.sigma << std::endl;
  //     std::cout << "G_u " << G_u << std::endl;
  //     std::cout << "control_noise_covariance "
  //     << control_noise_covariance << std::endl;
  //   }
  //   /***** 更新均值 *****/
  //     state_.mu(0) += delta_pos(0) * c_theta - delta_pos(1) * s_theta;
  //     state_.mu(1) += delta_pos(0) * s_theta + delta_pos(1) * c_theta;
  //     state_.mu(2) += delta_pos(2);
  //     state_.mu(2) = SLAMMath::NormalizePITheta(state_.mu(2));
  //     state_.time = data.mclDeltaPosition.mlTimestamp;
  //   pthread_mutex_unlock(&state_mutex_);
  // }
  // return;


  _calPredictedState(v_r, w, delta_time, data.mclDeltaPosition.mlTimestamp,
      Eigen::Vector3d(delta_time * result(0), delta_time * result(1),
                      delta_time * result(2)));
}

void ReflectorEkfCalcuator::_calOdomMessageSingleSteer(
  const OdometerMessage& data) {
  if (data.mclDeltaPosition.mlTimestamp < state_.time) return;
  double delta_time =
      1e-6 * (data.mclDeltaPosition.mlTimestamp - state_.time);
  if (delta_time > 0.06) {
    SLAM_WARN("delta time %f %lu %lu",
              delta_time, data.mclDeltaPosition.mlTimestamp, state_.time);
  }
  float ldLineVel = data.mstruSingleSteerSts.mdLinearVel;
  float ldAngularVel = data.mstruSingleSteerSts.mdAngularVel;
  float ldTurnAng = data.mstruSingleSteerSts.mdRotAngle;

  if (std::isnan(ldLineVel) || std::isnan(ldAngularVel) || std::isnan(ldTurnAng)) {
    SLAM_ERROR("get vel is nan %f %f %f", ldLineVel, ldAngularVel, ldTurnAng);
    return;
  }

  double v_r = ldLineVel * (cos(ldTurnAng));
  double w = ldLineVel * (sin(ldTurnAng)) /
                        sensor_mount_config_.to_centre_distance;
  double delta_theta = w * delta_time;

  double delta_x = v_r * cos(delta_theta) * delta_time;
  double delta_y = v_r * sin(delta_theta) * delta_time;

  if (!is_mapping_) {
    // 位姿雅克比矩阵 state_/state_(t-1)
    int N = state_.mu.rows();
    Eigen::MatrixXd G_xi = Eigen::MatrixXd::Identity(N, N);
    // G_xi.setZero(N, N);
    float s_theta = std::sin(state_.mu(2));
    float c_theta = std::cos(state_.mu(2));
    float s_delta_sum = sin(state_.mu(2) + delta_theta);
    float c_delta_sum = cos(state_.mu(2) + delta_theta);
    Eigen::MatrixXd G_xi_tmp = Eigen::MatrixXd::Zero(3, 3);
    // 单舵轮 ！！！！
    G_xi_tmp <<
      1, 0, -v_r * delta_time * s_delta_sum,
      0, 1, v_r * delta_time * c_delta_sum,
      0, 0, 1;
    G_xi.block(0, 0, 3, 3) = G_xi_tmp;
    // 控制雅克比矩阵 state_/u[v, w]
    Eigen::MatrixXd G_u = Eigen::MatrixXd::Zero(N, 2);
    G_u.setZero(N, 2);
    Eigen::MatrixXd G_u_block = Eigen::MatrixXd::Zero(3, 2);
    // 单舵轮 ！！！！！
    G_u_block <<
      delta_time * c_delta_sum, -v_r * delta_time * delta_time * s_delta_sum,
      delta_time * s_delta_sum, v_r * delta_time * delta_time * c_delta_sum,
      0, delta_time;
    G_u.block(0, 0, 3, 2) = G_u_block;

    Eigen::MatrixXd control_noise_covariance = Eigen::MatrixXd::Identity(2, 2);
    // 单舵轮 ！！！！
    control_noise_covariance <<
      ekf_config_.control_left_error * ekf_config_.control_left_error, 0,
      0, ekf_config_.control_left_error * ekf_config_.control_left_error;
    /* 更新协方差 */
    {
      pthread_mutex_lock(&state_mutex_);
      state_.sigma = G_xi * state_.sigma * G_xi.transpose() +
                    G_u * control_noise_covariance * G_u.transpose();
      state_.mu(0) += delta_x * c_theta - delta_y * s_theta;
      state_.mu(1) += delta_x * s_theta + delta_y * c_theta;
      state_.mu(2) += delta_theta;
      state_.mu(2) = SLAMMath::NormalizePITheta(state_.mu(2));
      state_.time = data.mclDeltaPosition.mlTimestamp;
      pthread_mutex_unlock(&state_mutex_);
    }
    return;
  }

  _calPredictedState(v_r, w, delta_time, data.mclDeltaPosition.mlTimestamp,
      Eigen::Vector3d(delta_x, delta_y, delta_theta));
}



void ReflectorEkfCalcuator::_calPredictedState(float v, float w,
  double delta_time, uint64_t time_stamp,
  const Eigen::Vector3d& delta_pos, bool in_global) {
  // 位姿雅克比矩阵 state_/state_(t-1)
  int N = state_.mu.rows();
  Eigen::MatrixXd G_xi = Eigen::MatrixXd::Identity(N, N);
  // G_xi.setZero(N, N);
  float s_theta = std::sin(state_.mu(2));
  float c_theta = std::cos(state_.mu(2));
  float s_delta_sum = sin(state_.mu(2) + delta_pos(2));
  float c_delta_sum = cos(state_.mu(2) + delta_pos(2));
  Eigen::MatrixXd G_xi_tmp = Eigen::MatrixXd::Zero(3, 3);
  // G_xi_tmp <<
  //     1, 0, -1.0 * s_theta * delta_pos(0) - c_theta * delta_pos(1),
  //     0, 1, c_theta * delta_pos(0) - s_theta * delta_pos(1),
  //     0, 0, 1;
  // 单舵轮 ！！！！
  // G_xi_tmp <<
  //   1, 0, -v * delta_time * s_delta_sum,
  //   0, 1, v * delta_time * c_delta_sum,
  //   0, 0, 1;
  // 直行 和 旋转进行区分 （通用）
  // if (fabs(w) > 0.01 && fabs(v) < 0.0001) {
  //   G_xi_tmp <<
  //     1, 0, v * (c_delta_sum - c_theta) / w,
  //     0, 1, v * (s_delta_sum - s_theta) / w,
  //     0, 0, 1;
  // } else {
  //   G_xi_tmp <<
  //     1, 0, -v * delta_time * s_theta,
  //     0, 1, v * delta_time * c_theta,
  //     0, 0, 1;
  // }
  float threshold = 0.000001;
  if (MAL::OdomType::DiffWheelModel == ekf_config_.odom_type) threshold = 0.001;
  if (fabs(w) < threshold) {
    // SLAM_DEBUG("vel_info %f %f %f", v, w, delta_time);
    G_xi_tmp <<
      1, 0, -v * delta_time * s_theta,
      0, 1, v * delta_time * c_theta,
      0, 0, 1;
  } else {
    // SLAM_ERROR("vel_info %f %f %f", v, w, delta_time);
    G_xi_tmp <<
      1, 0, v * (c_delta_sum - c_theta) / w,
      0, 1, v * (s_delta_sum - s_theta) / w,
      0, 0, 1;
  }
  G_xi.block(0, 0, 3, 3) = G_xi_tmp;
  // 控制雅克比矩阵 state_/u[v, w]
  Eigen::MatrixXd G_u = Eigen::MatrixXd::Zero(N, 2);
  G_u.setZero(N, 2);
  Eigen::MatrixXd G_u_block = Eigen::MatrixXd::Zero(3, 2);
  // 单舵轮 ！！！！！
  // G_u_block <<
  //   delta_time * c_delta_sum, -v * delta_time * delta_time * s_delta_sum,
  //   delta_time * s_delta_sum, v * delta_time * delta_time * c_delta_sum,
  //   0, delta_time;
  // if (fabs(w) > 0.01 && fabs(v) < 0.0001) {
  //   G_u_block <<
  //     (-s_theta + s_delta_sum) / w,
  //     v * (s_theta - s_delta_sum) / (w * w) +
  //     v * c_delta_sum * delta_time / w,
  //     (c_theta - c_delta_sum) / w,
  //     -v * (c_theta - c_delta_sum) / (w * w) +
  //     v * s_delta_sum * delta_time / w,
  //     0, delta_time;
  // } else {
  //   G_u_block <<
  //     delta_time * c_theta, 0,
  //     delta_time * s_theta, 0,
  //     0, 0;
  // }
  if (fabs(w) < threshold) {
    G_u_block <<
      delta_time * c_theta, 0,
      delta_time * s_theta, 0,
      0, 0;
  } else {
    G_u_block <<
      (-s_theta + s_delta_sum) / w,
      v * (s_theta - s_delta_sum) / (w * w) +
      v * c_delta_sum * delta_time / w,
      (c_theta - c_delta_sum) / w,
      -v * (c_theta - c_delta_sum) / (w * w) +
      v * s_delta_sum * delta_time / w,
      0, delta_time;
  }
  G_u.block(0, 0, 3, 2) = G_u_block;


  Eigen::MatrixXd control_noise_covariance = Eigen::MatrixXd::Identity(2, 2);
  // 单舵轮 ！！！！
  // control_noise_covariance <<
  //   ekf_config_.control_left_error * ekf_config_.control_left_error, 0,
  //   0, ekf_config_.control_left_error * ekf_config_.control_left_error;
  control_noise_covariance(0, 1) = 0.0;
  control_noise_covariance(1, 0) = 0.0;
  // 单舵轮 0.000001 差速 0.001
  // if (fabs(w) > 0.01 && fabs(v) < 0.0001) {
  //   control_noise_covariance(0, 0) =
  //     ekf_config_.control_left_error *
  //     (std::pow(v, 2) + std::pow(w, 2));
  //   control_noise_covariance(1, 1) =
  //     ekf_config_.control_right_error *
  //     (std::pow(v, 2) + std::pow(w, 2));
  // } else {
  //   control_noise_covariance(0, 0) =
  //   ekf_config_.control_left_error * std::pow(v, 2);
  //   control_noise_covariance(1, 1) =
  //   ekf_config_.control_right_error * std::pow(v, 2);
  // }
  if (fabs(w) < threshold) {
    control_noise_covariance(0, 0) =
    ekf_config_.control_left_error * std::pow(v, 2);
    control_noise_covariance(1, 1) =
    ekf_config_.control_right_error * std::pow(v, 2);
  } else {
    control_noise_covariance(0, 0) =
      ekf_config_.control_left_error *
      (std::pow(v, 2) + std::pow(w, 2));
    control_noise_covariance(1, 1) =
      ekf_config_.control_right_error *
      (std::pow(v, 2) + std::pow(w, 2));
  }

  if (fabs(delta_pos(0)) > 0.15 || fabs(delta_pos(1)) > 0.15 ||
      fabs(delta_pos(2)) > 0.05) {
      SLAM_WARN("change too fast %f %f %f",
                delta_pos(0), delta_pos(1), delta_pos(2));
  }

  /* 更新协方差 */
  {
    pthread_mutex_lock(&state_mutex_);
    state_.sigma = G_xi * state_.sigma * G_xi.transpose() +
                  G_u * control_noise_covariance * G_u.transpose();
    if (state_.sigma(0, 0) > 1.0) {
      std::cout << std::endl;
      std::cout << std::endl;
      std::cout << "G_xi " << G_xi << std::endl;
      std::cout << "state_.sigma " << state_.sigma << std::endl;
      std::cout << "G_u " << G_u << std::endl;
      std::cout << "control_noise_covariance "
      << control_noise_covariance << std::endl;
    }
    /***** 更新均值 *****/
    if (in_global) {
      state_.mu.topRows(3) += delta_pos;
    } else {
      state_.mu(0) += delta_pos(0) * c_theta - delta_pos(1) * s_theta;
      state_.mu(1) += delta_pos(0) * s_theta + delta_pos(1) * c_theta;
      state_.mu(2) += delta_pos(2);
    }
    state_.mu(2) = SLAMMath::NormalizePITheta(state_.mu(2));
    state_.time = time_stamp;
    pthread_mutex_unlock(&state_mutex_);
  }
  return;
}

Eigen::Vector2d ReflectorEkfCalcuator::_ladar2GlobalFrame(
  const Eigen::Vector2d& obs) {
  double x = obs(0) * std::cos(state_.mu(2)) -
             obs(1) * std::sin(state_.mu(2)) +
             state_.mu(0);
  double y = obs(0) * std::sin(state_.mu(2)) +
             obs(1) * std::cos(state_.mu(2)) +
             state_.mu(1);
  return Eigen::Vector2d(x, y);
}



}  // namespace DataFusion
