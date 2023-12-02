/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.0
 * @Author: renjy
 * @Date: 2022-11-16 16:34:29
 * @LastEditTime: 2023-10-11 20:48:04
 */
#pragma once
#include <sys/stat.h>
#include <ceres/ceres.h>

#include <fstream>
#include <vector>
#include <string>
#include <memory>
#include <map>

#include "fusion_mapping/qr_map_cal.h"

#include "karto_mapping/Karto.h"
#include "include/config_struct.h"
#include "landmark_tool/landmark_center_calcuator.h"
#include "landmark_tool/landmark_dbscan_calcuator.h"
#include "include/mapping_and_location_math.h"
#include "display_result/display_result.h"

#include "message_lib/grid_map.h"
#include "message_lib/simple_grid_map.h"
#include "message_lib/odometer_message.h"
#include "message_lib/position_message.h"
#include "message_lib/imu_message.h"
#include "message_lib/radar_message.h"

#include "common/tool.h"

namespace gomros {
namespace data_process {
namespace mapping_and_location {


// 利用ceres库进行二维码offset调整
struct OffsetFittingCost {
  OffsetFittingCost(double gx, double gy, double gt,
                    double lx, double ly, double lt)
                    : gx_(gx), gy_(gy), gt_(gt),
                      lx_(lx), ly_(ly), lt_(lt) {}
  template <typename T>
  bool operator() (const T* center, T* residual) const {
    Eigen::Matrix3d global(3, 3);
    global << cos(gt_), -sin(gt_), gx_,
              sin(gt_), cos(gt_), gy_,
              0, 0, 1;
    Eigen::Matrix3d local(3, 3);
    local << cos(lt_), -sin(lt_), lx_,
              sin(lt_), cos(lt_), ly_,
              0, 0, 1;
    Eigen::Matrix3d offset(3, 3);
    offset = global * local.inverse();
    residual[0] = center[0] - offset(0, 2);
    residual[1] = center[1] - offset(1, 2);
    residual[2] = center[2] - atan2(offset(1, 0), offset(0, 0));
    return true;
  }
  // 全局坐标
  const double gx_, gy_, gt_;
  // 局部坐标
  const double lx_, ly_, lt_;
};


/**
 * @brief: 融合地图 地图中包括反光柱信息 二维码信息
 */
class FusionMap : public karto::Grid<kt_int8u> {
 public:
  using SimpleGridMap = gomros::message::SimpleGridMap;
  using MapInfo = gomros::message::MapInfo;
  using OdometerMessage = gomros::message::OdometerMessage;
  using RadarSensoryInfo = gomros::message::RadarSensoryInfo;
  using ImuSensoryMessage = gomros::message::ImuSensoryMessage;
  using Position = gomros::message::Position;

 public:
  FusionMap(const MappingConfig& config,
            kt_int32s width, kt_int32s height,
            const karto::Vector2<kt_double>& rOffset,
            bool is_qr_calibration = false);
  ~FusionMap() {}

  /**
   * @brief: 根据激光帧构建地图
   * @param scan_points 激光数据
   * @param map_name 地图保存名字
   * @return {*}
   */
  void CreateMapFromScans(const karto::LocalizedRangeScanVector& scan_points,
                          const std::string& map_name);

 private:
  /**
   * @brief: 初始化qr地图
   * @return {*}
   */ 
  void _initQrMapInfo();
  /**
   * @brief: 构建栅格地图信息及反光柱信息
   * @param scan 激光帧数据
   * @param land_mark_pos 反光柱的信息
   * @return {*}
   */  
  void _createScanMapAndLandMark(const karto::LocalizedRangeScan* scan,
                        std::map<int, LandMark::Center>* land_mark_pos);
  /**
   * @brief: 构建二维码地图
   * @param scan 激光帧数据
   * @param qrs_world 二维码地图
   * @return {*}
   */  
  void _createQrMap(const karto::LocalizedRangeScanVector& scan_points,
                    std::vector<QRCoordinate>* qrs_world);

  /**
   * @name: 雷达坐标转至车体坐标系
   * @param ladar_world 雷达在全局坐标系中的位置
   * @param robot_word 车体在全局坐标系中的位置
   * @return {*}
   */
  void _ladar2Robot(const Position& ladar_world, Position* robot_word);
  /**
   * @name: 计算qr和激光当前对应的位置信息（航迹推算出来的）
   * @param last_pos 上一时刻的姿态
   * @param now_pos 当前时刻姿态
   * @param last_odom_msg 上一时刻对应的里程计原始数据
   * @return {*}
   */
  bool _calQrScanMatch(const Position& last_pos, const Position& now_pos,
                      const OdometerMessage& last_odom_msg);

  /**
   * @name: 计算qr在全局地图的中位置
   * @return {*}
   */
  void _calQRCoordinateInMap(std::vector<QRCoordinate>* qrs_world);

  /**
   * @name: 计算指定时间坐标
   * @param last_pos 上时刻位姿
   * @param odom_msg 上时刻对应的odom信息
   * @param time 要预计到位置的时间戳
   * @return {*}
   */
  Position _calTimePos(const Position& last_pos,
                       const OdometerMessage& odom_msg, uint64_t time);


  /**
   * @brief: 激光数据追踪，记录栅格被打中次数
   * @return {*}
   */  
  bool _rayTrace(const karto::Vector2<kt_double>& rWorldFrom,
                 const karto::Vector2<kt_double>& rWorldTo,
                 bool isEndPointValid);

  /**
   * @brief: 更新激光扫描栅格地图
   * @return {*}
   */
  void _updateScanMap();
  /**
   * @brief: 更新每个栅格
   * @param {kt_int8u*} pCell 栅格
   * @param {kt_int32u} cellPassCnt 栅格通过的次数
   * @param {kt_int32u} cellHitCnt 栅格击中的次数
   * @return {*}
   */  
  void _updateCell(kt_int8u* pCell,
                   kt_int32u cellPassCnt, kt_int32u cellHitCnt);

  /**
   * @name: 记录栅格地图
   * @return {*}
   */
  void _recordGridMap(const std::string& map_name);
  /**
   * @description: 获取反光柱中心
   * @return {*}
   */  
  std::map<int, LandMark::Center> _getLandmarkCenters();
  /**
   * @brief: 将地图记录到文件中
   * @param land_mark_map 反光板地图
   * @param qr_map 二维码地图
   * @param map_name 保存文件名字
   * @return {*}
   */
  void _recordMapInfoToFile(
      const std::map<int, LandMark::Center>& land_mark_map,
      const std::vector<QRCoordinate>& qr_map,
      const std::string& map_name);

  /**
   * @name: 显示当前激光帧 仿真测试用
   * @param {LocalizedRangeScan*} scan
   * @return {*}
   */
  void _showMapLadar(const karto::LocalizedRangeScan* scan);

 private:
  MappingConfig config_;      // 融合地图相关的配置参数
  Grid<kt_int32u>* cell_pass_cnt_;  // 激光束通过一个cell的计算器
  Grid<kt_int32u>* cell_hits_cnt_;  // 激光束在一个cell的击中次数计算器
  kt_int32u min_pass_through_;  // 一个cell被定义为被占用/未被占用最小的通过光束
  // cell被定义为占用时，击中次数和穿过次数最小比例
  kt_double occupancy_threshold_;
  Json::Value map_json_;          // 保存到map的json格式

  std::vector<CreateQrMapInfo> create_qr_map_info_;
  bool is_qr_calibration_;
  std::vector<LandMark::DbscanPoint> landmark_points_;
  std::shared_ptr<DisPlayResult::SimulationDataPublisher> show_ladar_for_test_;
};





}  // namespace mapping_and_location
}  // namespace data_process
}  // namespace gomros




