/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.0
 * @Author: renjy
 * @Date: 2022-10-29 11:41:17
 * @LastEditTime: 2023-10-18 18:42:24
 */
#pragma once
#include <ceres/ceres.h>
#include <vector>
#include <map>
#include <boost/smart_ptr.hpp>

#include "Eigen/Dense"

#include "message_lib/radar_message.h"
#include "include/config_struct.h"


/**
 * @brief: 计算landmark中心点 反光板 相关聚类算法
 */

namespace LandMark {
using namespace gomros::data_process::mapping_and_location;  // NOLINT

std::vector<Eigen::Vector2d> CalLandMarkCentor(
  const gomros::message::RadarSensoryInfo& radar_info,
  const gomros::data_process::mapping_and_location::LandMarkConfig& config);
  
/**
 * @brief: 反光柱数据信息
 */
struct Point {
  float mx;       // 反光柱点全局坐标 单位：m
  float my;       // 反光柱点全局坐标 单位：m
  float intensitie;  // 反光柱光强
  bool involved_cirlce = false;   // 是否参与匹配反光柱
  Point() {
    mx = 0.0f;
    my = 0.0f;
    involved_cirlce = false;
  }
  Point(float x, float y) {
    mx = x;
    my = y;
    involved_cirlce = false;
  }
  Point(float x, float y, float i) {
    mx = x;
    my = y;
    intensitie = i;
    involved_cirlce = false;
  }
};


/**
 * @brief: 计算landmark box范围
 */
struct LandMarkBox {
  float min_mx;
  float min_my;
  float max_mx;
  float max_my;
  float max_intensity;
  Point max_intensity_point;
  LandMarkBox();
  void AddPoint(const Point& point);
  bool IsInBox(const Point& point, const float dis = 0.0) const;
  float GetHeigth() const {
    return max_my - min_my;
  }
  float GetWidth() const {
    return max_mx - min_mx;
  }
};


/**
 * @brief: 反光柱中心 及数据信息
 */
struct Center {
  Center() {
    is_cal_center = false;
  }
  Center(float x, float y) {
    mx = x;
    my = y;
    is_cal_center = false;
  }
  float mx;      // 反光柱的全局中心坐标 单位：m
  float my;      // 反光柱的全局中心坐标 单位：m
  bool is_cal_center;     // 是否已经计算了反光柱的信息
  LandMarkBox box;   // 反光柱点集范围
  std::vector<Point> land_mark_points;
};


// 利用ceres库进行曲线拟合
struct CicleFittingCost {
  CicleFittingCost(double x, double y) : x_(x), y_(y) {}
  template <typename T>
  bool operator() (const T* center, T* residual) const {
    residual[0]= center[2] - (T(x_) - center[0]) * (T(x_) - center[0]) -
                             (T(y_) - center[1]) * (T(y_) - center[1]);
    return true;
  }
  const double x_, y_;
};


// 用于已知位姿下的landMark中心计算
class Calcuator {
 public:
  explicit Calcuator(const LandMarkConfig& config);
  ~Calcuator() {}
  void Init();
  /**
   * @brief: 方式1：增加反光板信息 只用其聚类
   * @param candidate_point 激光全局候选点
   * @return {*}
   */
  void AddLandMarkCenter(const LandMark::Point& candidate_point);

  /**
   * @brief: 方式二：寻找雷达数据中的有效数据 添加至反光柱识别
   * @param radar_info 雷达数据帧 默认是逆时针传入
   * @return {*}
   */
  void AddValidPoints(const std::vector<LandMark::Point>& radar_info);

  /**
   * @brief: 获取landmark中心点
   * @return {*}
   */
  std::map<int, LandMark::Center> GetLandMarkCenter(bool use_expand = false);

 private:
   /**
   * @brief: 更新及计算反光柱的中心位置
   * @param land_mark_pos 激光柱点集
   * @return {*}
   */  
  void _calLandMarkCenter(std::map<int, LandMark::Center>* land_mark_pos);
  /**
   * @name: 判断是否为反光柱 避免误判
   * @param reflectors 超出光强的反光柱信息
   * @return {*}
   */ 
  bool _checkIsReflector(LandMark::Center* reflectors);
  /**
   * @name: 根据距离设置反光柱阈值范围
   * @param {float} dis
   * @return {*}
   */  
  int _intensityCurve(float dis);

 private:
  // pcl数据集
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> cloud_in_;
  LandMarkConfig config_;
};



}  // namespace LandMark



