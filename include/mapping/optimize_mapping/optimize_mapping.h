/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.0
 * @Author: renjy
 * @Date: 2023-09-24 14:45:01
 * @LastEditTime: 2023-11-09 10:29:59
 */
#pragma once
#include <sys/stat.h>
#include <memory>
#include <fstream>
#include <vector>
#include <string>
#include <map>
#include "Eigen/Dense"

#include "common_lib/node.h"
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

// 保存QR在航迹推算 和第一个推算的原始信息
struct QrAndPoseInfo {
  uint64_t time_stemp;   //  时间戳
  int tag_num;        // QR id
  bool get_pose_;    // 获取全局位姿
  // 在QR上的的局部为位姿
  float mx;
  float my;
  float theta;

  // 车体全局位姿
  float pos_x;
  float pos_y;
  float pos_theta;
};

// 当前移动类型
enum CurrentMoveType {
  STOP,     // 静止
  CircularMotion,  // 圆弧
  StraightMotion,  // 直行
  RotateMotion  // 原地旋转
};


class OptimizeMapping : public MappingInterface {
 public:
  explicit OptimizeMapping(const MappingConfig& config);
  virtual ~OptimizeMapping();

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
  void _startOptimizeMapping(const std::string& map_name);

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
   * @description: 获取qr位姿信息
   * @param {QrAndPoseInfo&} pre
   * @param {QrAndPoseInfo*} cur
   * @return {*}
   */
  void _calPoseFromQr(const QrAndPoseInfo& pre, QrAndPoseInfo* cur);
  /**
   * @brief: 利用scan_match计算位姿
   * @param {Pose2&} predicted_pos 预测位姿
   * @param {Position*} scan_match_pos scan match获取的位姿带时间戳
   * @param {Matrix3d*} covariance scan match协方差矩阵
   * @return {*} 是否成功匹配
   */  
  bool _calScanMatchPos(const std::vector<std::string>& ladar_data,
                        const karto::Pose2& predicted_pos,
                        Eigen::Matrix3d* covariance);
  /**
   * @description: 获取landmark约束 插值
   * @param {uint64_t} ladar_time 雷达对应时间戳
   * @param {Info*} qr_info 获取到的插值信息
   * @return {*}
   */
  bool _getLandmarkConstraints(uint64_t ladar_time,
                              karto::LandmarkConstraintsInfo* qr_info);
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

  /**
   * @brief: 开始创建地图
   * @param {string&} map_name 地图名字
   * @return {*}
   */  
  void _createMap(const std::string& map_name);


  // 打开odom和imu文件
  void _openOdomAndImuFile();
  // 关闭odom和imu文件
  void _closeOdomAndImuFile();

  // 判断机器初始状态
  karto::MoveType _checkMoveType(const OdometerMessage& message);

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
  std::map<int, QrAndPoseInfo> first_qr_pose_;
  std::map<uint64_t, QrAndPoseInfo> qrs_info_;
  karto::MoveType current_move_type_;
  bool is_change_type;
  bool is_first;
};







}  // namespace mapping_and_location
}  // namespace data_process
}  // namespace gomros


