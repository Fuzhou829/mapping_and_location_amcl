/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.0
 * @Author: renjy
 * @Date: 2022-11-14 19:33:37
 * @LastEditTime: 2023-10-11 20:25:14
 */
#pragma once
#include <sys/stat.h>
#include <memory>
#include <fstream>
#include <vector>
#include <string>
#include "Eigen/Dense"

#include "message_lib/odometer_message.h"

#include "ekf_calcuator/ekf_calcuator.h"

#include "fusion_mapping/fusion_map_info.h"
#include "include/mapping_and_location_math.h"
#include "common/run_time.h"

#include "mapping/mapping_interface.h"
#include "include/config_struct.h"

#include "karto_mapping/Karto.h"
#include "karto_mapping/Mapper.h"
#include "karto_mapping/spa_solver.h"

#include "common/tool.h"
#include "common/logger.h"

namespace gomros {
namespace data_process {
namespace mapping_and_location {


class FusionMapping : public MappingInterface {
 public:
  using OdometerMessage = gomros::message::OdometerMessage;

 public:
  explicit FusionMapping(const MappingConfig& config);
  virtual ~FusionMapping();

  // 开始建图
  void StartMapping() override;
  // 停止建图
  void StopMapping(const std::string& map_name) override;

 private:
  /**
   * @brief: 初始化karto mapper
   * @return {*}
   */ 
  void _initKartoMapper();
  /**
   * @brief: 开始融合进入离线建图模式
   * @param {string&} map_name 离线建图最终结果
   * @return {*}
   */  
  void _startFusionMapping(const std::string& map_name);

  /**
   * @brief: 判断机器是否行走足够多的距离
   * @param {Pose2&} pre_pos 上一课距离
   * @param {Pose2&} now_pos 当前距离
   * @return {*}
   */
  bool _isMoveEnough(const karto::Pose2& pre_pos, const karto::Pose2& now_pos);
  /**
   * @brief: 获取当前odom的位姿 EKF融合odom和imu
   * @param {uint64_t} time_stemp
   * @param {Position&} last_pos 上一时刻的位姿（带时间戳）并根据预测进行更新
   * @param {Matrix3d&} covariance 上一时刻的协方差矩阵 并根据预测进行更新
   * @return {*} karto::Pose2 预测位姿信息
   */  
  karto::Pose2 _calCurrentOdomPos(uint64_t time_stemp, Position* last_pos,
                                  Eigen::Matrix3d* covariance);
  /**
   * @brief: 利用scan_match计算位姿
   * @param {Pose2&} predicted_pos 预测位姿
   * @param {Position*} scan_match_pos scan match获取的位姿带时间戳
   * @param {Matrix3d*} covariance scan match协方差矩阵
   * @return {*} 是否成功匹配
   */  
  bool _calScanMatchPos(const std::vector<std::string>& ladar_data,
      const karto::Pose2& predicted_pos, Eigen::Matrix3d* covariance);
  /**
   * @brief: 获取scanmatch数据的标准格式
   * @param ladar_data 文件中激光雷达数据
   * @param predicted_pos 预测位姿
   * @param range_scan 获取激光帧的标准格式
   * @return {*}
   */  
  void _calLocatizedRangeScan(const std::vector<std::string>& ladar_data,
                              const karto::Pose2& predicted_pos,
                              karto::LocalizedRangeScan* range_scan);

  /**
   * @brief: 开始创建地图
   * @param {string&} map_name 地图名字
   * @return {*}
   */  
  void _createMap(const std::string& map_name);
  /**
   * @brief: 计算地图的相关参数
   * @param rScans 全部激光数据
   * @param resolution 栅格地图分辨率
   * @param rWidth 栅格地图宽度
   * @param rHeight 栅格地图高度
   * @param rOffset 栅格地图的补偿
   * @return {*}
   */
  void _computeDimensions(const karto::LocalizedRangeScanVector& rScans,
                        const kt_double& resolution, kt_int32s* rWidth,
                        kt_int32s* rHeight, karto::Vector2<kt_double>* rOffset);


  // 打开odom和imu文件
  void _openOdomAndImuFile();
  // 关闭odom和imu文件
  void _closeOdomAndImuFile();

 private:
  std::ifstream odom_file_;
  std::ifstream imu_file_;
  karto::LaserRangeFinder* laser_range_finder_;  // 激光头的相关参数
  karto::Dataset* dataset_;
  karto::Mapper* karto_mapper_;
  SpaSolver* spa_solver_;

 private:
  bool is_record_last_odom_;      // 是否存在上一帧数据被忽略
  bool is_record_last_imu_;
  OdometerMessage last_odom_massage_;
  ImuSensoryMessage last_imu_massage_;
};







}  // namespace mapping_and_location
}  // namespace data_process
}  // namespace gomros


