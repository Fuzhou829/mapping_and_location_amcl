/*
 * @Descripttion: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @version: 1.0
 * @Author: renjy
 * @Date: 2023-05-14 09:55:13
 * @LastEditors: renjy
 * @LastEditTime: 2023-06-27 14:12:56
 */
#pragma once

#include <stdint.h>
#include <ceres/ceres.h>
#include <ceres/cubic_interpolation.h>

#include <vector>
#include <memory>
#include <opencv2/calib3d/calib3d.hpp>

#include "Eigen/Dense"

#include "include/qr_struct.h"
#include "karto_mapping/Karto.h"
#include "include/config_struct.h"

#include "display_result/display_result.h"

namespace gomros {
namespace data_process {
namespace mapping_and_location {

class OccupiedSpaceCostFunction2D {
 public:
  using RadarSensoryInfo = gomros::message::RadarSensoryInfo;

 public:
  OccupiedSpaceCostFunction2D(const double scaling_factor,
                              const SensorMount& config,
                              const RadarSensoryInfo& point_cloud,
                              const karto::Grid<kt_int8u>& grid)
      : scaling_factor_(scaling_factor),
        config_(config),
        point_cloud_(point_cloud),
        grid_(grid) {}

  template <typename T>
  bool operator()(const T* const pose, T* residual) const {
    // 将雷达数据转移至全局坐标系下
    // SLAM_DEBUG("to pos %f %f %f", pose[0], pose[1], pose[2]);
    Eigen::Matrix<T, 3, 3> in_robot(3, 3);
    in_robot <<
      T(cos(config_.radar_position_theta)),
      -T(sin(config_.radar_position_theta)), T(config_.radar_position_x),
      T(sin(config_.radar_position_theta)),
      T(cos(config_.radar_position_theta)), T(config_.radar_position_y),
      T(0), T(0), T(1);
    Eigen::Matrix<T, 3, 3> in_world(3, 3);
    in_world <<
      cos(pose[2]), -sin(pose[2]), pose[0],
      sin(pose[2]), cos(pose[2]), pose[1],
      T(0), T(0), T(1);
    Eigen::Matrix<T, 3, 3> transform2world = in_world * in_robot;
    const GridArrayAdapter adapter(grid_);
    ceres::BiCubicInterpolator<GridArrayAdapter> interpolator(adapter);
    double offset_x = grid_.GetCoordinateConverter()->GetOffset().GetX();
    double offset_y = grid_.GetCoordinateConverter()->GetOffset().GetY();
    double resolution = grid_.GetCoordinateConverter()->GetResolution();
    for (size_t i = 0;
        i < point_cloud_.mstruSingleLayerData.mvPoints.size(); ++i) {
      // Note that this is a 2D point. The third component is a scaling factor.
      Eigen::Matrix<T, 3, 3> point_mat(3, 3);
      point_mat <<
        T(1), T(0), T(point_cloud_.mstruSingleLayerData.mvPoints[i](0)),
        T(0), T(1), T(point_cloud_.mstruSingleLayerData.mvPoints[i](1)),
        T(0), T(0), T(1);

      // 根据预测位姿对单个点进行坐标变换
      Eigen::Matrix<T, 3, 3> result = transform2world * point_mat;

      // 获取三次插值之后的栅格free值, Evaluate函数内部调用了GetValue函数

      T grid_x = (result(0, 2) - offset_x) / resolution + 0.5;
      T grid_y = (result(1, 2) - offset_y) / resolution + 0.5;
      interpolator.Evaluate(grid_x, grid_y, &residual[i]);
      // free值越小, 表示占用的概率越大
      residual[i] = scaling_factor_ * residual[i];
    }
    return true;
  }

 private:
  static constexpr int kPadding = INT_MAX / 4;

  // 自定义网格
  class GridArrayAdapter {
   public:
    // 枚举 DATA_DIMENSION 表示被插值的向量或者函数的维度
    enum { DATA_DIMENSION = 1 };

    explicit GridArrayAdapter(const karto::Grid<kt_int8u>& grid) :
      grid_(grid) {}

    // 获取栅格free值
    void GetValue(const int row, const int column, double* const value) const {
      // 处于地图外部时, 赋予最大free值
      if (row < 0 || column < 0 || row >= grid_.GetWidth() ||
          column >= grid_.GetHeight()) {
        *value = 0.9f;
      } else {
        // 根据索引获取free值
        kt_int32s grid_val =
          grid_.GetValue(karto::Vector2<kt_int32s>(row, column));
        // SLAM_DEBUG("grid (%d %d) val %d", column, row, grid_val);
        switch (grid_val) {
          case karto::GridStates_Occupied:
            *value = 0.0f;
          break;
          default:
            *value = 0.9f;
          break;
        }
      }
    }

    // map上下左右各增加 kPaddingg
    int NumRows() const {
      return grid_.GetHeight();
    }

    int NumCols() const {
      return grid_.GetWidth();
    }

   private:
    const karto::Grid<kt_int8u>& grid_;
  };
  OccupiedSpaceCostFunction2D(const OccupiedSpaceCostFunction2D&) = delete;
  OccupiedSpaceCostFunction2D& operator=(const OccupiedSpaceCostFunction2D&) =
      delete;

  const double scaling_factor_;
  const RadarSensoryInfo& point_cloud_;
  const karto::Grid<kt_int8u>& grid_;
  const SensorMount config_;
};



class TranslationDeltaCostFunctor2D {
 public:
  // 静态成员函数, 返回CostFunction
  static ceres::CostFunction* CreateAutoDiffCostFunction(
      const double scaling_factor, const Eigen::Vector2d& target_translation) {
    return new ceres::AutoDiffCostFunction<TranslationDeltaCostFunctor2D,
                                           2 /* residuals */,
                                           3 /* pose variables */>(
        new TranslationDeltaCostFunctor2D(scaling_factor, target_translation));
  }

  // 平移量残差的计算, (pose[0] - x_)的平方ceres会自动加上
  template <typename T>
  bool operator()(const T* const pose, T* residual) const {
    residual[0] = scaling_factor_ * (pose[0] - x_);
    residual[1] = scaling_factor_ * (pose[1] - y_);
    return true;
  }

 private:
  // Constructs a new TranslationDeltaCostFunctor2D from the given
  // 'target_translation' (x, y).
  explicit TranslationDeltaCostFunctor2D(
      const double scaling_factor, const Eigen::Vector2d& target_translation)
      : scaling_factor_(scaling_factor),
        x_(target_translation.x()),
        y_(target_translation.y()) {}

  TranslationDeltaCostFunctor2D(const TranslationDeltaCostFunctor2D&) = delete;
  TranslationDeltaCostFunctor2D& operator=(
      const TranslationDeltaCostFunctor2D&) = delete;

  const double scaling_factor_;
  const double x_;
  const double y_;
};

class RotationDeltaCostFunctor2D {
 public:
  // 静态成员函数, 返回CostFunction
  static ceres::CostFunction* CreateAutoDiffCostFunction(
      const double scaling_factor, const double target_angle) {
    return new ceres::AutoDiffCostFunction<
        RotationDeltaCostFunctor2D, 1 /* residuals */, 3 /* pose variables */>(
        new RotationDeltaCostFunctor2D(scaling_factor, target_angle));
  }

  // 旋转量残差的计算
  template <typename T>
  bool operator()(const T* const pose, T* residual) const {
    residual[0] = scaling_factor_ * (pose[2] - angle_);
    return true;
  }

 private:
  explicit RotationDeltaCostFunctor2D(const double scaling_factor,
                                      const double target_angle)
      : scaling_factor_(scaling_factor), angle_(target_angle) {}

  RotationDeltaCostFunctor2D(const RotationDeltaCostFunctor2D&) = delete;
  RotationDeltaCostFunctor2D& operator=(const RotationDeltaCostFunctor2D&) =
      delete;

  const double scaling_factor_;
  const double angle_;
};

/**
 * @param: 用于构建二维码地图 
 */

struct ScanMatchOption {
  double occupied_space_cost_factor = 10.;
  double translation_delta_cost_factor = 1.;
  double rotation_delta_cost_factor = 1.;

  int max_num_iterations = 100;
  int num_threads = 4;
  bool use_nonmonotonic_steps = false;
};





class CreataQrMapAndCalibration {
 public:
  using RadarSensoryInfo = gomros::message::RadarSensoryInfo;
  using RadarSensoryMessage = gomros::message::RadarSensoryMessage;
  using Position = gomros::message::Position;

 public:
  CreataQrMapAndCalibration(const MappingConfig& config,
              const karto::Grid<kt_int8u>* map_info);
  virtual ~CreataQrMapAndCalibration();
  // 获取qr地图信息
  void GetQrMap(const CreateQrMapInfo& create_map_info,
                std::vector<QRCoordinate>* qrs_world);
  // 获取QR的标定信息
  void GetQrCalibration();

 private:
  void _scanMatch(const Position& init_pos,
                  const RadarSensoryInfo& ladar_info,
                  const karto::Grid<kt_int8u>& map_info,
                  Position* pose_estimate);
  /**
   * @name: 计算qr在全局地图中的坐标
   * @return {*}
   */
  void _calQrWorldPos(const Position& pose_estimate,
                      const CreateQrMapInfo& create_map_info,
                      std::vector<QRCoordinate>* qrs_world);
  /**
   * @name: 获取手眼标定所需数据
   * @return {*}
   */
  void _calHandEyeMat(const QRCoordinate& qr1_coor,
                      const QRCoordinate& qr2_coor,
                      const CalibrationQrInfo& qr1_info,
                      const CalibrationQrInfo& qr2_info,
                      cv::Mat* T_A, cv::Mat* R_A,
                      cv::Mat* T_B, cv::Mat* R_b);
  /**
   * @name: 齐次坐标转为旋转矩阵和平移矩阵
   * @param homo_mtr 齐次矩阵
   * @param t 平移矩阵
   * @return {*}
   * @param r 旋转矩阵
   */  
  void _homogeneousMtr2RT(const Eigen::Matrix3d& mat, cv::Mat* r, cv::Mat* t);

 private:
  ScanMatchOption option_;
  MappingConfig config_;
  const karto::Grid<kt_int8u>* map_info_;
  std::vector<CalibrationQrInfo> calibration_qrs_info_;
  std::shared_ptr<DisPlayResult::SimulationDataPublisher> show_ladar_;
};





}  // namespace mapping_and_location
}  // namespace data_process
}  // namespace gomros
