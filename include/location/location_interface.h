/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.0
 * @Date: 2022-07-25 10:37:44
 * @LastEditTime: 2023-08-19 15:51:04
 * @Author: lcfc-desktop
 */
#pragma once


#include <memory>
#include <vector>
#include <map>
#include <opencv2/opencv.hpp>

#include "message_lib/odometer_message.h"
#include "message_lib/position_message.h"
#include "message_lib/radar_message.h"
#include "message_lib/simple_grid_map.h"
#include "message_lib/imu_message.h"

#include "common/tool.h"
#include "landmark_tool/landmark_center_calcuator.h"

#include "display_result/display_result.h"


namespace gomros {
namespace data_process {
namespace mapping_and_location {

class LocationInterface {
 public:
  using OdometerMessage = gomros::message::OdometerMessage;
  using RadarSensoryMessage = gomros::message::RadarSensoryMessage;
  using ImuSensoryMessage = gomros::message::ImuSensoryMessage;
  using Position = gomros::message::Position;

  /**
   * @name: 处理雷达数据
   * @param data 雷达数据
   * @param forecast_pos odom位姿
   * @param is_move 是否在运动中，静止不进行定位操作
   * @return {*}
   */
  virtual void HandleLaserData(const RadarSensoryMessage &data,
                               const Position& forecast_pos,
                               bool is_move) {}

  /**
   * @name: 处理odom数据
   * @param odom_data odom数据
   * @return {*}
   */
  virtual void HandleOdomData(const OdometerMessage& odom_data) {}

  /**
   * @name: 处理imu数据
   * @param imu_data imu数据
   * @return {*}
   */
  virtual void HandleImuData(const ImuSensoryMessage& imu_data) {}

  /**
   * @name: 设置初始位姿
   * @param {float} x
   * @param {float} y
   * @param {float} theta
   * @return {*}
   */
  virtual void SetInitPose(float x, float y, float theta) = 0;

  /**
   * @name: 设置全局定位结果 -- 可终止全局定位，直接进去局部定位
   * @param {Position&} pos
   * @return {*}
   */  
  virtual void SetGlobalLocationPos(const Position& pos) = 0;

  /**
   * @name: 开始全局定位
   * @return {*}
   */  
  virtual bool StartGlobalLocating() = 0;

  /**
   * @name: 开始全局定位
   * @param is_relocation true 重新定位作为定位结果
   *                      false 直接采用传入的数据作为定位信息
   * @return {*}
   */  
  virtual bool StartGlobalLocating(bool is_relocation) {}

  /**
   * @name: 设置地图数据 栅格地图
   * @param {SimpleGridMap*} grid_map
   * @return {*}
   */  
  virtual void SetMapData(gomros::message::SimpleGridMap* grid_map) {}

  /**
   * @name: 设置地图数据 反光板地图
   * @return {*}
   */
  virtual void SetMapData(
    const std::map<int, std::map<int, Eigen::Vector2d>>& map) {}

  /**
   * @description: 设置slam地图 -- 非栅格地图
   * @return {*}
   */
  virtual void SetMapData(const std::vector<Eigen::Vector2d>& map) {}
  /**
   * @name: 获取当前位姿左边
   * @param {Position*} pos
   * @return {*}
   */ 
  virtual bool GetCurrentPosition(Position* pos) = 0;

  /**
   * @name: 获取全局定位是否完成标志位
   * @return {*}
   */  
  virtual bool IsFinishLocate() = 0;
  virtual  gomros::message::RadarSensoryInfo GetLaserData() = 0;
  virtual gomros::message::RadarSensoryInfo GetParticleData() = 0;
  virtual gomros::message::Position GetCurrentPose() = 0;
  virtual cv::Mat GetImage() = 0;

 protected:
  std::shared_ptr<DisPlayResult::SimulationDataPublisher> display_ladar_;
};

}  // namespace mapping_and_location
}  // namespace data_process
}  // namespace gomros

