/*
 * @Descripttion: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @version: 1.0
 * @Author: renjy
 * @Date: 2023-03-15 15:04:48
 * @LastEditors: renjy
 * @LastEditTime: 2023-10-18 15:22:22
 */

#pragma once
#include <mutex>  // NOLINT
#include <list>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>

#include "Eigen/Dense"

#include "include/config_struct.h"

#include "common_lib/gomros.h"
#include "message_lib/pose_and_odom_message.h"
#include "message_lib/simple_grid_map.h"
#include "message_lib/radar_message.h"

#include "common/tool.h"

namespace Display {

/**
 * @name: 可视化定位结果
 */
namespace mapping_and_location = gomros::data_process::mapping_and_location;

struct LandMarkGrid {
  int grid_x;
  int grid_y;
  LandMarkGrid(int xx, int yy) {
    grid_x = xx;
    grid_y = yy;
  }
};

// 地图信息
struct MapInfo {
  float resolution;
  int width;
  int height;
  float origen_x;
  float origen_y;
};


class LocationResult {
 public:
  using Node = gomros::common::Node;
  using CallBackEvent = gomros::common::CallBackEvent;
  using TotalOdomMessgae = gomros::message::TotalOdomMessgae;
  using OdometerMessage = gomros::message::OdometerMessage;
  using Position = gomros::message::Position;
  using RadarSensoryMessage = gomros::message::RadarSensoryMessage;
  using SimpleGridMap = gomros::message::SimpleGridMap;
  using St2dLayerData = gomros::message::St2dLayerData;

 public:
  explicit LocationResult(
    const mapping_and_location::MappingAndLocationConfig& config);
  virtual ~LocationResult();

  bool IsFinishedDisply();

 private:
  /**
   * @description: 加载地图
   * @param {string&} map_dir
   * @return {*}
   */ 
  bool _loadMap(const std::string& map_dir);
  /**
   * @description: 加载landmark地图信息
   */  
  void _loadLandMarkMap(int grid_x, int grid_y);
  /**
   * @description: 加载qr地图信息
   */  
  void _loadQrMap(int grid_x, int grid_y);
  /**
   * @description: slam->栅格坐标
   * @return {*}
   */  
  bool _world2Grid(float x, float y, int* grid_x, int* grid_y);
  /**
   * @description: 判断是否在地图中
   * @return {*}
   */  
  bool _isInPicture(int grid_x, int grid_y);
  /**
   * @description: 将雷达信息画在地图上
   * @param ladar_info
   * @param  map
   * @return {*}
   */
  void _ladarInMap(const RadarSensoryMessage& ladar_info, cv::Mat* map);

  /**
   * @description: 订阅定位信息
   * @return {*}
   */
  static void _locationPosEvent(void *object, char *buf, const int size);

  /**
   * @description: 接受建图结果
   * @return {*}
   */
  static void _gridMapEvent(void *object, char *buf, const int size);
  /**
   * @description: 显示地图及定位信息
   * @return {*}
   */
  static void *_displayMapPthread(void *param);

  /**
   * @description: 接收雷达原始数据及其对应的全局坐标
   * @return {*}
   */  
  static void _rawLadarEvent(void *object, char *buf, const int size);

  /**
   * @description: 加载定位信息在地图的线程
   * @return {*}
   */
  static void *_displayLadarInMapPthread(void *param);

  /**
   * @description: 显示接受到的最新地图信息
   * @return {*}
   */
  void _showGridMap();



  void _showObs();
  // 显示反光板ekf的相关信息
  void _showReflectorEKFObs();
  /**
   * @describes: 绘制置信椭圆
   * @param mean 均值
   * @param cov 协方差
   * @return {*}
   */
  void _displayConfidenceEllipse(const Eigen::Vector2d& mean,
                                 const Eigen::Matrix2d& cov,
                                 cv::Mat& show_mat);  // NOLINT

  // 接受观测结果 -- 显示在全局坐标下
  static void _observationEvent(void *object, char *buf, const int size);

  /**
  * @describes: 显示ekf观测信息
  * @return {*}
  */
  static void _reflectorEKFEvent(void *object, char *buf, const int size);

 private:
  bool is_get_map_;
  bool is_get_pos_;
  SimpleGridMap grid_map_;
  mapping_and_location::MappingAndLocationConfig config_;

  Node* node_;

  pthread_mutex_t picture_data_mutex_;
  cv::Mat show_mat_;
  MapInfo map_info_;

  pthread_mutex_t raw_ladar_data_mutex_;
  // 雷达原始数据及雷达在全局中的坐标
  std::list<RadarSensoryMessage> raw_ladar_datas_;



  pthread_t display_ladar_pthread;
  pthread_t display_map_pthread;



  pthread_mutex_t obs_pos_mutex_;
  pthread_mutex_t ekf_obs_mutex_;

  Position current_pos_;
  std::list<St2dLayerData> ladar_datas_;
  std::list<St2dLayerData> ekf_obs_datas_;
  std::vector<LandMarkGrid> landmark_map_;

  int display_count_;
};


}  // namespace Display




