/*
 * @Descripttion: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @version: 1.0
 * @Author: renjy
 * @Date: 2023-03-15 15:04:58
 * @LastEditors: renjy
 * @LastEditTime: 2023-10-23 11:51:10
 */

#include <unistd.h>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>

#include "Eigen/Eigen"

#include "mock/display_location_result.h"
#include "mock/bresenham_line.h"
#include "jsoncpp/json/json.h"
#include "common/logger.h"
#include "display_result/display_topic.h"
#include "common/transform.h"


namespace Display {
// static cv::Mat show_mat_;

LocationResult::LocationResult(
  const mapping_and_location::MappingAndLocationConfig& config) {
  pthread_mutex_init(&raw_ladar_data_mutex_, nullptr);
  pthread_mutex_init(&picture_data_mutex_, nullptr);

  pthread_mutex_init(&obs_pos_mutex_, nullptr);
  pthread_mutex_init(&ekf_obs_mutex_, nullptr);

  config_ = config;
  is_get_map_ = false;
  is_get_pos_ = false;

  // 加载地图用于可视化
  std::string file_dir =
    config_.mapping_config.map_data_file_path + "/" + "ale.smap";
  display_count_ = 0;
  if (!_loadMap(file_dir)) {
    SLAM_ERROR("can not load map !!!");
    return;
  }
  SLAM_INFO("load map successful!!!!!!!!!!!!!");
  // 订阅建图及定位结果
  node_ = new Node("display_node");
  CallBackEvent location_pose_event{this, _locationPosEvent};
  node_->SubscribeTopic(config_.ad_pose_topic, location_pose_event);

  CallBackEvent grid_map_event{this, _gridMapEvent};
  node_->SubscribeTopic(DisPlayResult::grid_map_topic, grid_map_event);


  pthread_create(&display_map_pthread, nullptr,
                 _displayMapPthread, this);

  CallBackEvent raw_ladar_event{this, _rawLadarEvent};
  node_->SubscribeTopic(DisPlayResult::raw_ladar_topic, raw_ladar_event);

  pthread_create(&display_ladar_pthread, nullptr,
                 _displayLadarInMapPthread, this);


  // 订阅定位结果
  // node_ = new Node("display");
  // CallBackEvent location_pose_event{this, _locationPosEvent};
  // node_->SubscribeTopic(config_.ad_pose_topic, location_pose_event);
  CallBackEvent obs_event{this, _observationEvent};
  node_->SubscribeTopic(DisPlayResult::reflector_ladar_topic, obs_event);

  CallBackEvent reflector_ekf_event{this, _reflectorEKFEvent};
  node_->SubscribeTopic(
    DisPlayResult::reflector_ekf_topic, reflector_ekf_event);
}

LocationResult::~LocationResult() {
  node_->UnsubscribeTopic(config_.ad_pose_topic);
  node_->UnsubscribeTopic(DisPlayResult::grid_map_topic);
  node_->UnsubscribeTopic(DisPlayResult::raw_ladar_topic);
  node_->UnsubscribeTopic(DisPlayResult::reflector_ladar_topic);
  node_->UnsubscribeTopic(DisPlayResult::reflector_ekf_topic);
  delete node_;
}


bool LocationResult::IsFinishedDisply() {
  return ladar_datas_.empty() && ekf_obs_datas_.empty();
}

bool LocationResult::_loadMap(const std::string& map_dir) {
  std::string map_data;
  if (!internal_common::Read(map_dir.c_str(), &map_data)) {
    SLAM_ERROR("read map failed.....");
    return false;
  }

  Json::Reader reader;
  Json::Value map_json;
  if (!reader.parse(map_data, map_json)) {
    SLAM_ERROR("parse map failed.....\n");
    return false;
  }
  pthread_mutex_lock(&picture_data_mutex_);
  map_info_.resolution = map_json["header"]["resolution"].asDouble();
  map_info_.origen_x = map_json["header"]["minPos"]["x"].asDouble();
  map_info_.origen_y = map_json["header"]["minPos"]["y"].asDouble();

  map_info_.width = static_cast<int>(ceil(
    (map_json["header"]["maxPos"]["x"].asDouble() -
    map_info_.origen_x) / map_info_.resolution));
  map_info_.height = static_cast<int>(ceil(
    (map_json["header"]["maxPos"]["y"].asDouble() -
    map_info_.origen_y) / map_info_.resolution));

  SLAM_INFO("location map info %d %d resolution %f",
            map_info_.width, map_info_.height, map_info_.resolution);
  show_mat_ = cv::Mat(cv::Size(map_info_.width, map_info_.height), CV_8UC3,
                            cv::Scalar(255, 255, 255));
  pthread_mutex_unlock(&picture_data_mutex_);
  for (int j = 0; j < map_json["normalPosList"].size(); j++) {
    float x = map_json["normalPosList"][j]["x"].asDouble();
    float y = map_json["normalPosList"][j]["y"].asDouble();
    int grid_x, grid_y;
    if (!_world2Grid(x, y, &grid_x, &grid_y)) continue;
    pthread_mutex_lock(&picture_data_mutex_);
    show_mat_.at<cv::Vec3b>(grid_y, grid_x)[0] = 0;
    show_mat_.at<cv::Vec3b>(grid_y, grid_x)[1] = 0;
    show_mat_.at<cv::Vec3b>(grid_y, grid_x)[2] = 0;
    pthread_mutex_unlock(&picture_data_mutex_);
  }
  // 显示landmark
  for (int i = 0; i < map_json["landmarks_pos_list"].size(); i++) {
    float x = map_json["landmarks_pos_list"][i]["x"].asDouble();
    float y = map_json["landmarks_pos_list"][i]["y"].asDouble();
    int grid_x, grid_y;
    if (!_world2Grid(x, y, &grid_x, &grid_y)) continue;
    landmark_map_.push_back(LandMarkGrid(grid_x, grid_y));
    _loadLandMarkMap(grid_x, grid_y);
  }

  // 显示qr
  for (int i = 0; i < map_json["qr_pos_list"].size(); i++) {
    float x = map_json["qr_pos_list"][i]["x"].asDouble();
    float y = map_json["qr_pos_list"][i]["y"].asDouble();
    int grid_x, grid_y;
    if (!_world2Grid(x, y, &grid_x, &grid_y)) continue;
    _loadQrMap(grid_x, grid_y);
  }

  return true;
}

void LocationResult::_loadLandMarkMap(int grid_x, int grid_y) {
  // 显示为红色的圆圈
  for (int xx = -3; xx <= 3; xx++) {
    for (int yy = -3; yy <= 3; yy++) {
      if (abs(xx) + abs(yy) != 3) continue;
      if (!_isInPicture(grid_x + xx, grid_y + yy)) continue;
      pthread_mutex_lock(&picture_data_mutex_);
      show_mat_.at<cv::Vec3b>(grid_y + yy, grid_x + xx)[0] = 0;
      show_mat_.at<cv::Vec3b>(grid_y + yy, grid_x + xx)[1] = 0;
      show_mat_.at<cv::Vec3b>(grid_y + yy, grid_x + xx)[2] = 255;
      pthread_mutex_unlock(&picture_data_mutex_);
    }
  }
}

void LocationResult::_loadQrMap(int grid_x, int grid_y) {
  // 显示为绿色的方块
  for (int xx = -3; xx <= 3; xx++) {
    for (int yy = -3; yy <= 3; yy++) {
      if (!_isInPicture(grid_x + xx, grid_y + yy)) continue;
      pthread_mutex_lock(&picture_data_mutex_);
      show_mat_.at<cv::Vec3b>(grid_y + yy, grid_x + xx)[0] = 0;
      show_mat_.at<cv::Vec3b>(grid_y + yy, grid_x + xx)[1] = 255;
      show_mat_.at<cv::Vec3b>(grid_y + yy, grid_x + xx)[2] = 0;
      pthread_mutex_unlock(&picture_data_mutex_);
    }
  }
}



bool LocationResult::_world2Grid(float x, float y, int* grid_x, int* grid_y) {
  pthread_mutex_lock(&picture_data_mutex_);
  *grid_x = ceil((x - map_info_.origen_x) / map_info_.resolution);
  *grid_y = ceil((y - map_info_.origen_y) / map_info_.resolution);
  pthread_mutex_unlock(&picture_data_mutex_);

  return _isInPicture(*grid_x, *grid_y);
}

bool LocationResult::_isInPicture(int grid_x, int grid_y) {
  pthread_mutex_lock(&picture_data_mutex_);
  if (grid_x < 0 || grid_x >= map_info_.width ||
      grid_y < 0 || grid_y >= map_info_.height) {
    pthread_mutex_unlock(&picture_data_mutex_);
    return false;
  }
  pthread_mutex_unlock(&picture_data_mutex_);
  return true;
}



void LocationResult::_ladarInMap(
  const RadarSensoryMessage& ladar_info, cv::Mat* map) {
  int current_grid_x, current_grid_y;
  float mfXPos = ladar_info.mstruRadarMessage.mstruRadarHeaderData.mfXPos;
  float mfYPos = ladar_info.mstruRadarMessage.mstruRadarHeaderData.mfYPos;
  float mfTheta = ladar_info.mstruRadarMessage.mstruRadarHeaderData.mfTheta;
  if (!_world2Grid(mfXPos, mfYPos, &current_grid_x, &current_grid_y)) return;

  Coordinate::Transform transform;
  transform.SetPose1InGlobal(Eigen::Vector3d(mfXPos, mfYPos, mfTheta));
  for (auto iter : ladar_info.mstruRadarMessage.mstruSingleLayerData.mvPoints) {
    // 将雷达点转换至全局坐标下
    transform.SetPose2InPose1(Eigen::Vector3d(iter(0), iter(1), 0.0));
    Eigen::Vector3d in_global;
    transform.GetPose2InGlobal(&in_global);
    int grid_x, grid_y;
    if (!_world2Grid(in_global(0), in_global(1), &grid_x, &grid_y)) continue;
    (*map).at<cv::Vec3b>(grid_y, grid_x)[0] = 0;
    (*map).at<cv::Vec3b>(grid_y, grid_x)[1] = 0;
    (*map).at<cv::Vec3b>(grid_y, grid_x)[2] = 255;
    // // 画线 灰色
    // LineMath::BresenhamLine line;
    // line.SetStartAndEndGrid(grid_x, grid_y, current_grid_x, current_grid_y);
    // int next_x, next_y;
    // while (line.GetNextGrid(&next_x, &next_y)) {
    //   if (!_isInPicture(next_x, next_y)) continue;
    //   // 显示为灰色
    //   (*map).at<cv::Vec3b>(next_y, next_x)[0] = 192;
    //   (*map).at<cv::Vec3b>(next_y, next_x)[1] = 192;
    //   (*map).at<cv::Vec3b>(next_y, next_x)[2] = 192;
    // }
  }
  // 显示粒子分布情况
  int sample_size = ladar_info.mstruRadarMessage.mstruMultiLayerData.size();
  cv::putText(*map, std::to_string(sample_size), cv::Point(10, 10),
              cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 0, 255), 1);
  for (auto iter : ladar_info.mstruRadarMessage.mstruMultiLayerData) {
    int grid_x, grid_y;
    if (!_world2Grid(iter.x, iter.y, &grid_x, &grid_y)) continue;
    // 显示为绿色
    (*map).at<cv::Vec3b>(grid_y, grid_x)[0] = 255;
    (*map).at<cv::Vec3b>(grid_y, grid_x)[1] = 0;
    (*map).at<cv::Vec3b>(grid_y, grid_x)[2] = 0;
    // 显示粒子的方向
    // float end_pos_x = iter.x + 0.02 * cos(iter.z);
    // float end_pos_y = iter.y + 0.02 * sin(iter.z);
    // int end_grid_x, end_grid_y;
    // if (!_world2Grid(end_pos_x, end_pos_y, &end_grid_x, &end_grid_y))
    //    continue;
    // cv::arrowedLine(*map, cv::Point(current_grid_x, current_grid_y),
    //                 cv::Point(end_grid_x, end_grid_y),
    //                 cv::Scalar(0, 0, 0), 1, 8, 0, 0.01);
  }


  // 显示当前位姿朝向
  float end_pos_x = mfXPos + 0.3 * cos(mfTheta);
  float end_pos_y = mfYPos + 0.3 * sin(mfTheta);
  int end_grid_x, end_grid_y;
  if (!_world2Grid(end_pos_x, end_pos_y, &end_grid_x, &end_grid_y)) {
    SLAM_ERROR("show pos theta is error......");
  } else {
    cv::arrowedLine(*map, cv::Point(current_grid_x, current_grid_y),
                    cv::Point(end_grid_x, end_grid_y),
                    cv::Scalar(0, 255, 0), 1, 8, 0, 0.1);
  }
  return;
}



void LocationResult::_locationPosEvent(
  void *object, char *buf, const int size) {
  LocationResult *lp_this = reinterpret_cast<LocationResult *>(object);
  gomros::common::Message<TotalOdomMessgae> location_pose;
  location_pose.FromCharArray(buf, size);
  OdometerMessage position = location_pose.pack.odom_oose;
  int grid_x, grid_y;
  if (!lp_this->_world2Grid(
      position.mclDeltaPosition.mfX, position.mclDeltaPosition.mfY,
      &grid_x, &grid_y)) {
    // SLAM_ERROR("location pos is error......");
    return;
  }
  pthread_mutex_lock(&lp_this->picture_data_mutex_);
  lp_this->is_get_pos_ = true;
  lp_this->show_mat_.at<cv::Vec3b>(grid_y, grid_x)[0] = 0;
  lp_this->show_mat_.at<cv::Vec3b>(grid_y, grid_x)[1] = 0;
  lp_this->show_mat_.at<cv::Vec3b>(grid_y, grid_x)[2] = 255;
  pthread_mutex_unlock(&lp_this->picture_data_mutex_);
  return;
}

void LocationResult::_gridMapEvent(void *object, char *buf, const int size) {
  LocationResult *lp_this = reinterpret_cast<LocationResult *>(object);
  SimpleGridMap simple_grid_map;
  simple_grid_map.from_json_char_array(buf, size);
  if (simple_grid_map.map_info.miMapWidth <= 0 ||
      simple_grid_map.map_info.miMapHeight <= 0) {
    SLAM_WARN("get grid map width %f, height %f",
                simple_grid_map.map_info.miMapWidth,
                simple_grid_map.map_info.miMapHeight);
    return;
  }
  lp_this->grid_map_ = simple_grid_map;
  lp_this->is_get_map_ = true;
}

void *LocationResult::_displayMapPthread(void *param) {
  pthread_detach(pthread_self());
  LocationResult *lp_this = reinterpret_cast<LocationResult *>(param);
  while (1) {
    if (lp_this->is_get_map_) {
      lp_this->is_get_map_ = false;
      lp_this->_showGridMap();
    }
    pthread_mutex_lock(&lp_this->picture_data_mutex_);
    cv::Mat grid_map_mat = lp_this->show_mat_.clone();
    if (grid_map_mat.rows <= 0 || grid_map_mat.cols <= 0) {
      pthread_mutex_unlock(&lp_this->picture_data_mutex_);
      continue;
    }
    cv::namedWindow("grid_map", 0);
    cv::resizeWindow("grid_map",
      lp_this->map_info_.width, lp_this->map_info_.height);
    cv::imshow("grid_map", grid_map_mat);
    pthread_mutex_unlock(&lp_this->picture_data_mutex_);
    cv::waitKey(10);
    usleep(100000);
  }
  pthread_exit(NULL);
}




void LocationResult::_rawLadarEvent(void *object, char *buf, const int size) {
  LocationResult *lp_this = reinterpret_cast<LocationResult *>(object);
  RadarSensoryMessage radar_data;
  radar_data.from_char_array(buf, size);
  pthread_mutex_lock(&lp_this->raw_ladar_data_mutex_);
  lp_this->raw_ladar_datas_.push_back(radar_data);
  pthread_mutex_unlock(&lp_this->raw_ladar_data_mutex_);
  return;
}

void *LocationResult::_displayLadarInMapPthread(void *param) {
  pthread_detach(pthread_self());
  LocationResult *lp_this = reinterpret_cast<LocationResult *>(param);
  while (1) {
    RadarSensoryMessage ladar_info;
    pthread_mutex_lock(&lp_this->raw_ladar_data_mutex_);
    if (lp_this->raw_ladar_datas_.empty()) {
      usleep(1000);
      pthread_mutex_unlock(&lp_this->raw_ladar_data_mutex_);
      continue;
    }
    ladar_info = lp_this->raw_ladar_datas_.front();
    lp_this->raw_ladar_datas_.pop_back();
    pthread_mutex_unlock(&lp_this->raw_ladar_data_mutex_);
    if (ladar_info.mstruRadarMessage.mstruRadarHeaderData.mcPosValid != 1) {
      usleep(1000);
      continue;
    }

    pthread_mutex_lock(&lp_this->picture_data_mutex_);
    cv::Mat show_mat = lp_this->show_mat_.clone();
    cv::namedWindow("ladar_in_map", 0);
    cv::resizeWindow("ladar_in_map",
      lp_this->map_info_.width, lp_this->map_info_.height);
    pthread_mutex_unlock(&lp_this->picture_data_mutex_);
    lp_this->_ladarInMap(ladar_info, &show_mat);

    cv::imshow("ladar_in_map", show_mat);
    cv::waitKey(20);
  }
  pthread_exit(NULL);
}



void LocationResult::_observationEvent(
  void *object, char *buf, const int size) {
  LocationResult *lp_this = reinterpret_cast<LocationResult *>(object);
  RadarSensoryMessage radar_data;
  radar_data.from_char_array(buf, size);

  pthread_mutex_lock(&lp_this->obs_pos_mutex_);
  lp_this->ladar_datas_.push_back(
    radar_data.mstruRadarMessage.mstruSingleLayerData);
  pthread_mutex_unlock(&lp_this->obs_pos_mutex_);
}


void LocationResult::_reflectorEKFEvent(
  void *object, char *buf, const int size) {
  LocationResult *lp_this = reinterpret_cast<LocationResult *>(object);
  RadarSensoryMessage radar_data;
  radar_data.from_char_array(buf, size);
  pthread_mutex_lock(&lp_this->ekf_obs_mutex_);
  lp_this->ekf_obs_datas_.push_back(
    radar_data.mstruRadarMessage.mstruSingleLayerData);
  pthread_mutex_unlock(&lp_this->ekf_obs_mutex_);
}

void LocationResult::_showObs() {
  std::vector<Eigen::Vector2d> obs_point;
  vector<float> mvIntensities;
  obs_point = ladar_datas_.front().mvPoints;
  mvIntensities = ladar_datas_.front().mvIntensities;
  ladar_datas_.pop_front();
  int current_grid_x, current_grid_y;
  float current_pos_x = obs_point.back()(0);
  float current_pos_y = obs_point.back()(1);
  float current_pos_theta = mvIntensities.back();
  if (!_world2Grid(current_pos_x, current_pos_y,
                   &current_grid_x, &current_grid_y)) return;

  pthread_mutex_lock(&picture_data_mutex_);
  cv::Mat show_mat = show_mat_.clone();
  pthread_mutex_unlock(&picture_data_mutex_);
  int obs_size = obs_point.size();

  for (int i = 0; i < obs_size - 3; i++) {
    float x = obs_point[i](0);
    float y = obs_point[i](1);
    int grid_x, grid_y;
    if (!_world2Grid(x, y, &grid_x, &grid_y)) continue;
    // 画点 显示为蓝色的方块
    for (int xx = -3; xx <= 3; xx++) {
      for (int yy = -3; yy <= 3; yy++) {
        if (!_isInPicture(grid_x + xx, grid_y + yy)) continue;
        if (mvIntensities[i] < 1.0) {
          show_mat.at<cv::Vec3b>(grid_y + yy, grid_x + xx)[0] = 255;
          show_mat.at<cv::Vec3b>(grid_y + yy, grid_x + xx)[1] = 0;
          show_mat.at<cv::Vec3b>(grid_y + yy, grid_x + xx)[2] = 0;
        } else if (mvIntensities[i] < 2.0) {
          show_mat.at<cv::Vec3b>(grid_y + yy, grid_x + xx)[0] = 0;
          show_mat.at<cv::Vec3b>(grid_y + yy, grid_x + xx)[1] = 0;
          show_mat.at<cv::Vec3b>(grid_y + yy, grid_x + xx)[2] = 255;
        }
      }
    }
    if (mvIntensities[i] < 2.0) continue;
    // 画线 灰色
    LineMath::BresenhamLine line;
    line.SetStartAndEndGrid(grid_x, grid_y, current_grid_x, current_grid_y);
    int next_x, next_y;
    while (line.GetNextGrid(&next_x, &next_y)) {
      if (!_isInPicture(next_x, next_y)) continue;
      show_mat.at<cv::Vec3b>(next_y, next_x)[0] = 192;
      show_mat.at<cv::Vec3b>(next_y, next_x)[1] = 192;
      show_mat.at<cv::Vec3b>(next_y, next_x)[2] = 192;
    }
  }
  for (auto iter = landmark_map_.begin(); iter != landmark_map_.end(); iter++) {
      // 显示为红色的圆圈
    for (int xx = -3; xx <= 3; xx++) {
      for (int yy = -3; yy <= 3; yy++) {
        if (abs(xx) + abs(yy) != 3) continue;
        if (!_isInPicture(iter->grid_x + xx, iter->grid_y + yy)) continue;
        show_mat.at<cv::Vec3b>(iter->grid_y + yy, iter->grid_x + xx)[0] = 0;
        show_mat.at<cv::Vec3b>(iter->grid_y + yy, iter->grid_x + xx)[1] = 0;
        show_mat.at<cv::Vec3b>(iter->grid_y + yy, iter->grid_x + xx)[2] = 255;
      }
    }
  }
  // 显示当前位姿朝向
  float end_pos_x = current_pos_x + 0.15 * cos(current_pos_theta);
  float end_pos_y = current_pos_y + 0.15 * sin(current_pos_theta);
  int end_grid_x, end_grid_y;
  if (!_world2Grid(end_pos_x, end_pos_y, &end_grid_x, &end_grid_y)) {
    SLAM_ERROR("show pos theta is error......");
  } else {
    cv::arrowedLine(show_mat, cv::Point(current_grid_x, current_grid_y),
                    cv::Point(end_grid_x, end_grid_y),
                    cv::Scalar(0, 255, 0), 2, 8, 0, 0.1);
  }

  Eigen::Matrix2d cov;
  cov << obs_point[obs_size - 3](0), obs_point[obs_size - 3](1),
         obs_point[obs_size - 2](0), obs_point[obs_size - 2](1);
  // _displayConfidenceEllipse(obs_point.back(), cov, show_mat);

  cv::namedWindow("location_result", 0);
  cv::resizeWindow("location_result", map_info_.width, map_info_.height);
  cv::imshow("location_result", show_mat);
  cv::waitKey(20);
}


void LocationResult::_showGridMap() {
  cv::Mat grid_map_mat =
    cv::Mat(cv::Size(grid_map_.map_info.miMapWidth,
                      grid_map_.map_info.miMapHeight),
    CV_8UC3, cv::Scalar(192, 192, 192));
  for (int y = 0; y < grid_map_.map_info.miMapHeight; y++) {
    for (int x = 0; x < grid_map_.map_info.miMapWidth; x++) {
    double mx = grid_map_.map_info.MapXToRealX(x);
    double my = grid_map_.map_info.MapYToRealY(y);
      char val = grid_map_.get(mx, my);
      switch (val) {
        case 100 :
          grid_map_mat.at<cv::Vec3b>(y, x)[0] = 0;
          grid_map_mat.at<cv::Vec3b>(y, x)[1] = 0;
          grid_map_mat.at<cv::Vec3b>(y, x)[2] = 0;
        break;
        case 0 :
          grid_map_mat.at<cv::Vec3b>(y, x)[0] = 255;
          grid_map_mat.at<cv::Vec3b>(y, x)[1] = 255;
          grid_map_mat.at<cv::Vec3b>(y, x)[2] = 255;
        break;
        default :
          grid_map_mat.at<cv::Vec3b>(y, x)[0] = 192;
          grid_map_mat.at<cv::Vec3b>(y, x)[1] = 192;
          grid_map_mat.at<cv::Vec3b>(y, x)[2] = 192;
        break;
      }
    }
  }

  pthread_mutex_lock(&picture_data_mutex_);
  map_info_.resolution =  grid_map_.map_info.mdResolution;
  map_info_.width = grid_map_.map_info.miMapWidth;
  map_info_.height = grid_map_.map_info.miMapHeight;
  map_info_.origen_x = grid_map_.map_info.mdOriginXInWorld;
  map_info_.origen_y = grid_map_.map_info.mdOriginYInWorld;
  SLAM_INFO("get new grid map (%f %f) width %d height %d",
            map_info_.origen_x, map_info_.origen_y,
            map_info_.width, map_info_.height);
  show_mat_ = grid_map_mat.clone();
  pthread_mutex_unlock(&picture_data_mutex_);

  cv::namedWindow("grid_map", 0);
  cv::resizeWindow("grid_map", map_info_.width, map_info_.height);
  cv::imshow("grid_map", grid_map_mat);
  cv::imwrite("new_map.png", grid_map_mat);
  cv::waitKey(10);
  return;
}

void LocationResult::_showReflectorEKFObs() {
  St2dLayerData points = ekf_obs_datas_.front();
  ekf_obs_datas_.pop_front();
  static int ekf_obs_count = 0;
  // SLAM_INFO("ekf_obs_count %d", ekf_obs_count++);
  if (ekf_obs_count == 265) {
    SLAM_INFO("watch out ~~~~");
  }
  float current_pos_x = points.mvPoints.back()(0);
  float current_pos_y = points.mvPoints.back()(1);
  int current_grid_x, current_grid_y;
  if (!_world2Grid(current_pos_x, current_pos_y,
                   &current_grid_x, &current_grid_y)) return;
  pthread_mutex_lock(&picture_data_mutex_);
  cv::Mat show_mat = show_mat_.clone();
  pthread_mutex_unlock(&picture_data_mutex_);
  bool is_have_new_obs = false;
  int new_count = 0;
  int match_count = 0;
  for (int i = 0; i < points.mvPoints.size() - 1; i++) {
    float intensites = points.mvIntensities[i];
    float x = points.mvPoints[i](0);
    float y = points.mvPoints[i](1);
    int grid_x, grid_y;
    if (!_world2Grid(x, y, &grid_x, &grid_y)) continue;
    // 画线 黑色
    LineMath::BresenhamLine line;
    line.SetStartAndEndGrid(grid_x, grid_y, current_grid_x, current_grid_y);
    if (intensites > 0) {
      is_have_new_obs = true;
      SLAM_DEBUG("lidar_count %d new obs grid (%d %d) %d", ekf_obs_count,
                  grid_x, grid_y, new_count++);
    } else if (intensites < -2) {
      match_count++;
    }
    int next_x, next_y;
    while (line.GetNextGrid(&next_x, &next_y)) {
      if (!_isInPicture(next_x, next_y)) continue;
      if (intensites > 0) {
        // 新增 红色
        show_mat.at<cv::Vec3b>(next_y, next_x)[0] = i;
        show_mat.at<cv::Vec3b>(next_y, next_x)[1] = 0;
        show_mat.at<cv::Vec3b>(next_y, next_x)[2] = 255;
      } else if (intensites > -2) {
        // 已有 绿色
        show_mat.at<cv::Vec3b>(next_y, next_x)[0] = 0;
        show_mat.at<cv::Vec3b>(next_y, next_x)[1] = 255;
        show_mat.at<cv::Vec3b>(next_y, next_x)[2] = i;
      } else if (intensites < -2) {
        // 匹配 蓝色
        show_mat.at<cv::Vec3b>(next_y, next_x)[0] = 255;
        show_mat.at<cv::Vec3b>(next_y, next_x)[1] = 0;
        show_mat.at<cv::Vec3b>(next_y, next_x)[2] = i;
      }
    }
  }
  if (is_have_new_obs) {
    SLAM_DEBUG("there have new obs, please check! %d", ekf_obs_count);
  }
  char picture_buff[512];
  snprintf(picture_buff, sizeof(picture_buff), \
          "./out/new_obs_recognition_%02d.png", ekf_obs_count);
  cv::imwrite(picture_buff, show_mat);

  ekf_obs_count++;
  cv::namedWindow("location_result", 0);
  cv::resizeWindow("location_result", map_info_.width, map_info_.height);
  cv::imshow("location_result", show_mat);
  cv::waitKey(20);
  return;
}

void LocationResult::_displayConfidenceEllipse(const Eigen::Vector2d& mean,
  const Eigen::Matrix2d& cov, cv::Mat& show_mat) {
  Eigen::EigenSolver<Eigen::Matrix2d> es(cov);
  // 特征值
  Eigen::Matrix2d value = es.pseudoEigenvalueMatrix();
  // 特征向量
  Eigen::Matrix2d vector = es.pseudoEigenvectors();

  float theta = value(0, 0) > value(1, 1) ?
              std::atan2(vector(1, 0), vector(0, 0)) :
              std::atan2(vector(1, 1), vector(0, 1));
  theta *= -180.0 / M_PI;
  float resolution = config_.mapping_config.mapping_resolution;
  int grid_x, grid_y;
  if (!_world2Grid(mean(0), mean(1), &grid_x, &grid_y)) return;

  cv::ellipse(show_mat, cv::Point(grid_x, grid_y),
      cv::Size(std::sqrt(value(0, 0)) * 10 / resolution,
      std::sqrt(value(1, 1)) * 10 / resolution),
      theta, 0, 360, cv::Scalar(0, 0, 255));
}

}  // namespace Display
