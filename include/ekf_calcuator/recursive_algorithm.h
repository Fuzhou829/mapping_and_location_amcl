/*
 * @Description: Copyright (C) 2023 山东亚历山大智能科技有限公司
 * @version: 1.2
 * @Author: renjy
 * @Data: 2023-09-01 10:44:54
 * @LastEditors: renjy
 * @LastEditTime: 2023-09-16 11:07:42
 */
#pragma once
#include <vector>

#include "Eigen/Dense"

namespace DataFusion {


class RecursiveAlgorithm {
 public:
  RecursiveAlgorithm(float esitimate_error, float measurement_error)
  : init_esitimate_error_(esitimate_error),
    measurement_error_(measurement_error) {
    is_first_data_ = true;
    count_ = 0;
  }
  virtual ~RecursiveAlgorithm() {}

  void Init() {
    is_first_data_ = true;
    esitimate_error_.clear();
    K_k_.clear();
    result_.clear();
    count_ = 0;
  }

  /**
   * @description: 增加一帧数据测量得到的obs
   * @param centers 一帧数据的结果
   * @return {*}a
   */
  void AddPoint(const std::vector<Eigen::Vector2d> centers) {
    if (is_first_data_) {
      is_first_data_ = false;
      result_ = centers;
      for (int i = 0; i < result_.size(); i++) {
        esitimate_error_.push_back(
          Eigen::Vector2d(init_esitimate_error_, init_esitimate_error_));
      }
      K_k_.resize(result_.size());
      count_++;
      return;
    }
    if (centers.size() != result_.size()) return;
    count_++;
    std::vector<Eigen::Vector2d> last_result = result_;
    for (int i = 0; i < centers.size(); i++) {
      K_k_[i](0) = esitimate_error_[i](0) /
                  (esitimate_error_[i](0) + measurement_error_);
      K_k_[i](1) = esitimate_error_[i](1) /
                  (esitimate_error_[i](1) + measurement_error_);
      if (SLAMMath::Dist(centers[i](0), centers[i](1),
                        result_[i](0), result_[i](1)) > 0.1) {
        result_ = last_result;
        break;
      }
      Eigen::Vector2d new_result;
      new_result(0) =
        result_[i](0) + K_k_[i](0) * (centers[i](0) - result_[i](0));
      new_result(1) =
        result_[i](1) + K_k_[i](1) * (centers[i](1) - result_[i](1));
      result_[i] = new_result;
      esitimate_error_[i](0) *= (1 - K_k_[i](0));
      esitimate_error_[i](1) *= (1 - K_k_[i](1));
    }
  }
  /**
   * @description: 返回多帧数据得到的反光柱子信息
   * @return {*}
   */
  bool GetResult(std::vector<Eigen::Vector2d>* result) const {
    *result = result_;
    return count_ > 10;
  }

 private:
  bool is_first_data_;
  int count_;
  std::vector<Eigen::Vector2d> esitimate_error_;
  float measurement_error_;
  float init_esitimate_error_;
  std::vector<Eigen::Vector2d> K_k_;
  std::vector<Eigen::Vector2d> result_;
};








}  // namespace DataFusion

