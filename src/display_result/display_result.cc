/*
 * @Descripttion: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @version: 1.0
 * @Author: renjy
 * @Date: 2023-05-20 02:39:17
 * @LastEditors: renjy
 * @LastEditTime: 2023-10-18 15:38:56
 */
#include "display_result/display_result.h"
#include "display_result/display_topic.h"



namespace DisPlayResult {

SimulationDataPublisher::SimulationDataPublisher(
  const SensorMount& sensor_mount)
  : sensor_mount_(sensor_mount) {
  node_ = new Node("display");
  publish_ladar_for_display_ = node_->AdvertiseTopic(raw_ladar_topic);
  grid_map_publisher_ = node_->AdvertiseTopic(grid_map_topic);
  publish_reflector_ekf_ = node_->AdvertiseTopic(reflector_ladar_topic);
}

SimulationDataPublisher::~SimulationDataPublisher() {}

void SimulationDataPublisher::DisplayLadarMessage(
  const RadarSensoryInfo& message) {
  RadarSensoryMessage message_result;
  message_result.mstruRadarMessage = message;
  std::vector<char> send_buff;
  message_result.to_char_array(&send_buff);
  publish_ladar_for_display_->Publish(send_buff.data(), send_buff.size());
}


void SimulationDataPublisher::DisPlayLadarAndObs(
  const RadarSensoryInfo& ladar_message,
  const Position& current_pos,
  const std::vector<Eigen::Vector2d>& obs,
  const std::vector<Eigen::Vector2d>& match_obs,
  const Eigen::Matrix2d& cov) {
  RadarSensoryMessage message;
  vector<Eigen::Vector2d> ladar_data =
    ladar_message.mstruSingleLayerData.mvPoints;
  float ladar_theta = sensor_mount_.radar_position_theta;
  float ladar_x = sensor_mount_.radar_position_x;
  float ladar_y = sensor_mount_.radar_position_y;
  for (int i = 0; i < ladar_data.size(); i++) {
    float ladar_data_x = ladar_data[i](0) * cos(ladar_theta) -
                        ladar_data[i](1) * sin(ladar_theta) + ladar_x;
    float ladar_data_y = ladar_data[i](0) * sin(ladar_theta) +
                        ladar_data[i](1) * cos(ladar_theta) + ladar_y;
    Eigen::Vector2d obs_vec;
    obs_vec(0) = ladar_data_x * cos(current_pos.mfTheta) -
                ladar_data_y * sin(current_pos.mfTheta) + current_pos.mfX;
    obs_vec(1) = ladar_data_x * sin(current_pos.mfTheta) +
                ladar_data_y * cos(current_pos.mfTheta) + current_pos.mfY;
    message.mstruRadarMessage.mstruSingleLayerData.mvPoints.push_back(obs_vec);
    message.mstruRadarMessage.mstruSingleLayerData.mvIntensities.push_back(4.0);
  }
  // 插入观测
  for (int i = 0; i < obs.size(); i++) {
    Eigen::Vector2d obs_vec;
    obs_vec(0) = obs[i](0) * cos(current_pos.mfTheta) -
                obs[i](1) * sin(current_pos.mfTheta) +current_pos.mfX;
    obs_vec(1) = obs[i](0) * sin(current_pos.mfTheta) +
                obs[i](1) * cos(current_pos.mfTheta) +current_pos.mfY;
    message.mstruRadarMessage.mstruSingleLayerData.mvPoints.push_back(obs_vec);
    message.mstruRadarMessage.mstruSingleLayerData.mvIntensities.push_back(0.0);
  }
  // 插入地图信息
  for (int i = 0; i < match_obs.size(); i++) {
    message.mstruRadarMessage.
        mstruSingleLayerData.mvPoints.push_back(match_obs[i]);
    message.mstruRadarMessage.
        mstruSingleLayerData.mvIntensities.push_back(1.1);
  }
  // 插入 协方差
  message.mstruRadarMessage.mstruSingleLayerData.mvPoints.push_back(
    Eigen::Vector2d(cov(0, 0), cov(0, 1)));
  message.mstruRadarMessage.mstruSingleLayerData.mvIntensities.push_back(10);
  message.mstruRadarMessage.mstruSingleLayerData.mvPoints.push_back(
    Eigen::Vector2d(cov(1, 0), cov(1, 1)));
  message.mstruRadarMessage.mstruSingleLayerData.mvIntensities.push_back(10);

  message.mstruRadarMessage.mstruSingleLayerData.mvPoints.push_back(
    Eigen::Vector2d(current_pos.mfX, current_pos.mfY));
  message.mstruRadarMessage.mstruSingleLayerData.
      mvIntensities.push_back(current_pos.mfTheta);
  std::vector<char> send_buff;
  message.to_char_array(&send_buff);
  publish_reflector_ekf_->Publish(send_buff.data(), send_buff.size());
  return;
}


void SimulationDataPublisher::DisPlayGridMap(SimpleGridMap& grid_map) {
  std::vector<char> send_buff;
  grid_map.to_json_char_array(&send_buff);
  grid_map_publisher_->Publish(send_buff.data(), send_buff.size());
}

void SimulationDataPublisher::DisPlayReflectorEkf(
  const Eigen::VectorXd& state, int new_obs_size,
  const std::vector<int>& match_id) {
  RadarSensoryMessage message;
  int state_obs_size = (state.rows() - 3) / 2;
  for (int i = 0; i < state_obs_size; i++) {
    Eigen::Vector2d temp(state(3 + 2 * i), state(3 + 2 * i + 1));
    message.mstruRadarMessage.mstruSingleLayerData.mvPoints.push_back(temp);
    if (i + new_obs_size < state_obs_size) {
      // 已存在
      bool is_match = false;
      for (auto id : match_id) {
        if (id == i) is_match = true;
      }
      if (is_match) {
        message.mstruRadarMessage.mstruSingleLayerData.
            mvIntensities.push_back(-3.0);
      } else {
        message.mstruRadarMessage.mstruSingleLayerData.
            mvIntensities.push_back(-1.0);
      }
    } else {
      // 新增
      message.mstruRadarMessage.mstruSingleLayerData.
          mvIntensities.push_back(1.0);
    }
  }
  message.mstruRadarMessage.mstruSingleLayerData.mvPoints.push_back(
    Eigen::Vector2d(state(0), state(1)));
  message.mstruRadarMessage.mstruSingleLayerData.mvIntensities.push_back(0.0);
  std::vector<char> send_buff;
  message.to_char_array(&send_buff);
  publish_reflector_ekf_->Publish(send_buff.data(), send_buff.size());
  return;
}



}  // namespace DisPlayResult

