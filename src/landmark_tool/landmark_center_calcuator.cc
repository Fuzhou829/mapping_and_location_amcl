/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.0
 * @Author: renjy
 * @Date: 2022-10-29 11:56:20
 * @LastEditTime: 2023-10-20 16:34:05
 */

#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <map>
#include <algorithm>

#include "Eigen/Dense"
#include "include/config_struct.h"
#include "landmark_tool/landmark_center_calcuator.h"
#include "include/mapping_and_location_math.h"
#include "common/logger.h"


namespace LandMark {


std::vector<Eigen::Vector2d> CalLandMarkCentor(
  const gomros::message::RadarSensoryInfo& radar_info,
  const gomros::data_process::mapping_and_location::LandMarkConfig& config) {
  bool is_recording = false;
  bool is_judging_rise = false;
  bool is_falling = false;
  bool enter_top = false;
  bool exit_top = false;
  double distance;                             // 和上一个点之间的距离
  double delta_inten;                          // 反射率变化量
  double max_point_distance = 0.0;             // 最大点间距
  double estimate_point_num_of_cluster = 0.0;  // 预估反光柱拥有点的数量
  int judge_rise_count = 0;                    // 判断上升计数
  // 最大反射强度的点，用于计算反光柱中心
  int max_intensity_start_index = -1;
  int max_intensity_end_index = -1;
  int start_index = -1;
  int end_index = -1;
  int number_of_cluster = 0;
  int equal_count = 0;
  int large_change_count = 0;
  int down_count = 0;
  int big_number_count = 0;
  float big_inten_limit = config.landmark_intensities;
  double top_intensity_limit = 0.0;
  std::vector<Eigen::Vector2d> max_intensity_center;
  std::vector<double> set_of_max_intensity;
  std::vector<Eigen::Vector2d> centers;
  int last_index = 0;

  float angle_resolution = config.laser_resolution * M_PI / 180.0f;
  double landmark_diameter = config.landmark_radius * 2;
  float min_distance_for_add_new_center = 1.01;
  float radar_fluctuation_error = 0.06;
  std::vector<Eigen::Vector2d> mvPoints =
    radar_info.mstruSingleLayerData.mvPoints;
  std::vector<float> mvIntensities =
    radar_info.mstruSingleLayerData.mvIntensities;
  for (size_t i = 1; i < mvPoints.size(); i++) {
      // 去除异常值
    if (fabs(mvIntensities[i] - 1.76) < 0.000001) continue;
    if (fabs(mvIntensities[i - 1] - 1.76) < 0.000001) {
      last_index = i - 2;
    } else {
      last_index = i - 1;
    }
    // 计算和上一个点的变化
    distance = sqrt(pow(mvPoints[i](0) - mvPoints[last_index](0), 2) +
                    pow(mvPoints[i](1) - mvPoints[last_index](1), 2));
    delta_inten = mvIntensities[i] - mvIntensities[last_index];
    // 判断是否在记录
    if (!is_recording) {
      // 如果没在记录就判断是否在判断上升
      if (!is_judging_rise) {
       // 如果没在记录而且没在判断上升，那就判断这个点是否合格，合格就改为在判断上升，
       // 并计算点的最大间距，否则就遍历下一个点
        if (delta_inten > 0.0 /* && delta_inten < 0.15 */) {
          // 计算该点到雷达的距离
          double distance_to_origin =
            sqrt(pow(mvPoints[i](0), 2) + pow(mvPoints[i](1), 2));
            // 计算最大点间距
            max_point_distance = sqrt(pow(2 * std::sin(angle_resolution * 0.5) *
                                          distance_to_origin, 2) +
                                      pow(radar_fluctuation_error * 2.0, 2));
            if (distance < max_point_distance) {
              is_judging_rise = true;
              if (judge_rise_count != 0) SLAM_ERROR("judge_rise_count1!");
              judge_rise_count++;
            }
        }
      } else {
        // 如果没在记录但是在判断上升，那就判断这个点是否合格
        if (delta_inten > 0.0 && distance < max_point_distance) {
          judge_rise_count++;
          // 如果连续上升数量达到2,则转换成上升状态
          if (judge_rise_count == 2) {
            max_intensity_start_index = i;
            max_intensity_end_index = i;
            start_index = i - 2;
            number_of_cluster += 3;
            is_recording = true;                 // CHECK
            if (mvIntensities[start_index] >= 0.6) {
                  top_intensity_limit = mvIntensities[start_index] + 0.1;
            } else {
                  top_intensity_limit = 0.6;
            }
          }
        } else {
          // 如果点不合格，则判断上升失败，复原状态变量，并继续遍历下一点
          if (judge_rise_count != 1) SLAM_ERROR("judge_rise_count2!");
          judge_rise_count = 0;
          is_judging_rise = false;
          continue;
        }
       }
      } else {
        // 记录下降的点的个数
        if (delta_inten < 0.0) down_count++;
          // 如果正在记录且没在下降
          if (!is_falling) {
            // 如果正在记录，且没在下降，但点间距不符合
            if (distance >= max_point_distance) {
              // 如果是在顶部出现，则判断是否在下降
              if (enter_top && !exit_top) {
              is_falling = true;
              i--;
              } else {
                is_recording = false;
                is_judging_rise = false;
                is_falling = false;
                enter_top = false;
                exit_top = false;
                judge_rise_count = 0;  // 判断上升计数
                max_intensity_start_index = -1;
                max_intensity_end_index = -1;
                start_index = -1;
                end_index = -1;
                number_of_cluster = 0;
                equal_count = 0;
                large_change_count = 0;
                down_count = 0;
                big_number_count = 0;
                continue;
              }
            }
            // 如果没在下降且intensity大于0.6，则说明进入顶部，都记录下来
            if (mvIntensities[i] > top_intensity_limit) {
              if (delta_inten == 0) {
                equal_count++;
              } else if (fabs(delta_inten) > 0.10) {
                large_change_count++;
              }
              enter_top = true;
              if (mvIntensities[i] > mvIntensities[max_intensity_start_index]) {
                max_intensity_start_index = i;
                max_intensity_end_index = i;
              } else if (mvIntensities[i] ==
                        mvIntensities[max_intensity_start_index]) {
                max_intensity_end_index = i;
              }
              number_of_cluster += 1;
              if (mvIntensities[i] >= big_inten_limit) {
                big_number_count++;
              }
              // 如果在0.6以上出现相等较多的情况，则直接停止，并恢复状态变量
              if (equal_count / (1.0f * number_of_cluster) >= 0.4) {
                is_recording = false;
                is_judging_rise = false;
                is_falling = false;
                enter_top = false;
                exit_top = false;
                judge_rise_count = 0;  // 判断上升计数
                max_intensity_start_index = -1;
                max_intensity_end_index = -1;
                start_index = -1;
                end_index = -1;
                number_of_cluster = 0;
                equal_count = 0;
                large_change_count = 0;
                down_count = 0;
                big_number_count = 0;
              }
            } else {
                // 如果没在下降且intensity小于0.6，但进入过top
                if (enter_top) {
                  // 说明已经从top出来，且正在下降
                  if (exit_top) SLAM_ERROR("exit_top!");
                  // 说明刚从top出来
                  float max_intensities =
                    mvIntensities[max_intensity_start_index];
                  if (mvIntensities[i] > max_intensities) {
                    max_intensity_start_index = i;
                    max_intensity_end_index = i;
                  } else if (mvIntensities[i] ==max_intensities) {
                    max_intensity_end_index = i;
                  }
                  number_of_cluster += 1;
                  if (mvIntensities[i] >= big_inten_limit) {
                    big_number_count++;
                  }
                  is_falling = true;
                  exit_top = true;
                } else {
                  // 如果没在下降且intensity小于0.6，且没有进入过top,则正常比较
                  if (distance < max_point_distance) {
                    // 如果还在上升,则继续记录
                    float max_intensities =
                    mvIntensities[max_intensity_start_index];
                    if (delta_inten > 0.0) {
                      if (mvIntensities[i] > max_intensities) {
                        max_intensity_start_index = i;
                        max_intensity_end_index = i;
                      } else if (mvIntensities[i] == max_intensities) {
                        max_intensity_end_index = i;
                      }
                      number_of_cluster += 1;
                      if (mvIntensities[i] >= big_inten_limit) {
                        big_number_count++;
                      }
                    } else {
                      // 如果没在下降，且intensity小于0.6，但出现下降或者相等
                      if (mvIntensities[i] >max_intensities) {
                        max_intensity_start_index = i;
                        max_intensity_end_index = i;
                      } else if (mvIntensities[i] == max_intensities) {
                        max_intensity_end_index = i;
                      }
                      number_of_cluster += 1;
                      if (mvIntensities[i] >= big_inten_limit) {
                        big_number_count++;
                      }
                      is_falling = true;
                    }
                  }
                }
            }
          } else {
            // 如果正在记录，且在下降，但点间距不符合，那就停止记录，复原状态变量
            if (distance >= max_point_distance) {
              end_index = i - 1;
              // 首先检查反光强度的变化是否符合要求,不符合则直接放弃
              double delta_start = mvIntensities[max_intensity_start_index] -
                                   mvIntensities[start_index];
              double delta_end = mvIntensities[max_intensity_start_index] -
                                 mvIntensities[end_index];
              double delta_scale = delta_start > delta_end ?
                        (delta_end/delta_start) : (delta_start/delta_end);
              if (delta_start < 0.4 *  mvIntensities[max_intensity_start_index]
                  || delta_end < 0.4 *  mvIntensities[max_intensity_start_index]
                  ||  mvIntensities[max_intensity_start_index] < 1.0
                  || delta_scale < 0.5) {
                  is_recording = false;
                  is_judging_rise = false;
                  is_falling = false;
                  enter_top = false;
                  exit_top = false;
                  judge_rise_count = 0;
                  max_intensity_start_index = -1;
                  max_intensity_end_index = -1;
                  start_index = -1;
                  end_index = -1;
                  number_of_cluster = 0;
                  equal_count = 0;
                  large_change_count = 0;
                  down_count = 0;
                  big_number_count = 0;
                  continue;
              }
              Eigen::Vector2d max_intensity_point(0, 0);
              max_intensity_point.x() =
                (mvPoints[max_intensity_start_index].x() +
                 mvPoints[max_intensity_end_index].x()) * 0.5;
              max_intensity_point.y() =
                (mvPoints[max_intensity_start_index].y() +
                 mvPoints[max_intensity_end_index].y()) * 0.5;
              // 更新点的数量
              number_of_cluster = end_index - start_index + 1;
              // 计算后退点数的限制（纵向限制，基本应该用不上）
              // int back_num = floor(number_of_cluster * 0.15);
              double start_distance_to_origin =
                        sqrt(pow(mvPoints[start_index](0), 2) +
                              pow(mvPoints[start_index](1), 2));
              double end_distance_to_origin =
                        sqrt(pow(mvPoints[end_index](0), 2) +
                             pow(mvPoints[end_index](1), 2));
              double max_distance_to_origin =
                        sqrt(pow(max_intensity_point.x(), 2) +
                             pow(max_intensity_point.y(), 2));

              estimate_point_num_of_cluster = landmark_diameter /
                (2 * std::sin(angle_resolution * 0.5) * max_distance_to_origin);

              if (number_of_cluster > estimate_point_num_of_cluster &&
                  big_number_count/estimate_point_num_of_cluster > 0.6 &&
                  big_number_count/estimate_point_num_of_cluster < 2.0) {
                float proportion = equal_count / (1.0f * number_of_cluster);
                if (proportion < 0.3 && proportion > 0.125) {
                  max_intensity_center.push_back(max_intensity_point);
                  set_of_max_intensity.push_back(
                    mvIntensities[max_intensity_start_index]);
                }
              }
              is_recording = false;
              is_judging_rise = false;
              is_falling = false;
              enter_top = false;
              exit_top = false;
              judge_rise_count = 0;                    // 判断上升计数
              // 最大反射强度的点，用于计算反光柱中心
              max_intensity_start_index = -1;
              max_intensity_end_index = -1;
              start_index = -1;
              end_index = -1;
              number_of_cluster = 0;
              equal_count = 0;
              large_change_count = 0;
              down_count = 0;
              big_number_count = 0;
              continue;
            }
            // 如果正在记录，且在下降，但点间距符合,且反光率仍在下降，则正常记录
            if (delta_inten < 0.0) {
              // CHECK
              number_of_cluster += 1;
              if (mvIntensities[i] >= big_inten_limit) {
                big_number_count++;
              }
            } else {
              end_index = i - 1;
              // 首先检查反光强度的变化是否符合要求,不符合则直接放弃
              float max_intensities =
                    mvIntensities[max_intensity_start_index];
              double delta_start = max_intensities - mvIntensities[start_index];
              double delta_end = max_intensities - mvIntensities[end_index];
              double delta_scale = delta_start > delta_end ?
                        (delta_end/delta_start) : (delta_start/delta_end);
              if (delta_start < 0.4 *  max_intensities ||
                  delta_end < 0.4 * max_intensities ||
                  max_intensities < 1.0 || delta_scale < 0.5) {
                  is_recording = false;
                  is_judging_rise = false;
                  is_falling = false;
                  enter_top = false;
                  exit_top = false;
                  judge_rise_count = 0;
                  max_intensity_start_index = -1;
                  max_intensity_end_index = -1;
                  start_index = -1;
                  end_index = -1;
                  number_of_cluster = 0;
                  equal_count = 0;
                  large_change_count = 0;
                  down_count = 0;
                  big_number_count = 0;
                  continue;
              }

              Eigen::Vector2d max_intensity_point(0, 0);
              max_intensity_point.x() = (mvPoints[max_intensity_start_index].x()
                                + mvPoints[max_intensity_end_index].x()) * 0.5;
              max_intensity_point.y() = (mvPoints[max_intensity_start_index].y()
                                + mvPoints[max_intensity_end_index].y()) * 0.5;

              // 更新点的数量
              number_of_cluster = end_index - start_index + 1;
              // 计算后退点数的限制（纵向限制，基本应该用不上）
              // int back_num = floor(number_of_cluster * 0.15);
              double start_distance_to_origin =
                sqrt(pow(mvPoints[start_index](0), 2) +
                     pow(mvPoints[start_index](1), 2));
              double end_distance_to_origin =
                sqrt(pow(mvPoints[end_index](0), 2) +
                     pow(mvPoints[end_index](1), 2));
              double max_distance_to_origin =
                sqrt(pow(max_intensity_point.x(), 2) +
                     pow(max_intensity_point.y(), 2));

              estimate_point_num_of_cluster = landmark_diameter /
                (2 * std::sin(angle_resolution * 0.5) * max_distance_to_origin);

              if (number_of_cluster > estimate_point_num_of_cluster
                  && big_number_count/estimate_point_num_of_cluster > 0.6
                  && big_number_count/estimate_point_num_of_cluster < 2.0) {
                if (equal_count / (1.0f * number_of_cluster) < 0.3 &&
                    down_count / (1.0f * number_of_cluster) > 0.125) {
                    max_intensity_center.push_back(max_intensity_point);
                    set_of_max_intensity.push_back(
                      mvIntensities[max_intensity_start_index]);
                }
              }
              is_recording = false;
              is_judging_rise = false;
              is_falling = false;
              enter_top = false;
              exit_top = false;
              judge_rise_count = 0;                    // 判断上升计数
              // 最大反射强度的点，用于计算反光柱中心
              max_intensity_start_index = -1;
              max_intensity_end_index = -1;
              start_index = -1;
              end_index = -1;
              number_of_cluster = 0;
              equal_count = 0;
              large_change_count = 0;
              down_count = 0;
              big_number_count = 0;
            }
          }
      }
  }

  for (int i = 0; i < max_intensity_center.size(); i++) {
    Eigen::Vector2d center{0, 0};
    double distance_to_origin =
              sqrt(pow(max_intensity_center[i].x(), 2) +
                    pow(max_intensity_center[i].y(), 2));
    double theta = std::atan2(max_intensity_center[i].y(),
                              max_intensity_center[i].x());
    double dis = distance_to_origin + landmark_diameter * 0.5;
    center.x() = dis * std::cos(theta);
    center.y() = dis * std::sin(theta);
    // centers.push_back(center);
    // filter防止再同一帧激光数据中出现距离非常近的两个反光柱
    if (centers.empty()) {
      centers.push_back(center);
    } else {
      double distance_to_last_center =
                          sqrt(pow(center(0) - centers.back()(0), 2) +
                          pow(center(1) - centers.back()(1), 2));
      if (distance_to_last_center > min_distance_for_add_new_center) {
        centers.push_back(center);
      } else {
        if (set_of_max_intensity[i] > set_of_max_intensity[i - 1]) {
          centers.pop_back();
          centers.push_back(center);
        }
      }
    }
  }
  return centers;
}




LandMarkBox::LandMarkBox() {
  min_mx = 10000000.0f;
  min_my = 10000000.0f;
  max_mx = -1000000.0f;
  max_my = -1000000.0f;
  max_intensity = -1000000.0f;
}


void LandMarkBox::AddPoint(const Point& point) {
  if (point.mx < min_mx) min_mx = point.mx;
  if (point.my < min_my) min_my = point.my;
  if (point.mx > max_mx) max_mx = point.mx;
  if (point.my > max_my) max_my = point.my;
  if (point.intensitie > max_intensity) {
    max_intensity_point = point;
    max_intensity = point.intensitie;
  }
  return;
}

bool LandMarkBox::IsInBox(const Point& point, const float dis) const {
  if (point.mx < min_mx - dis) return false;
  if (point.my < min_my - dis) return false;
  if (point.mx > max_mx + dis) return false;
  if (point.my > max_my + dis) return false;
  return true;
}

Calcuator::Calcuator(const LandMarkConfig& config) {
  config_ = config;
  cloud_in_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
}

void Calcuator::Init() {
  cloud_in_->clear();
}


void Calcuator::AddLandMarkCenter(const LandMark::Point& candidate_point) {
  pcl::PointXYZI temp;
  temp.intensity = candidate_point.intensitie;
  temp.x = candidate_point.mx;
  temp.y = candidate_point.my;
  temp.z = 0.0;
  cloud_in_->push_back(temp);
}

void Calcuator::AddValidPoints(const std::vector<LandMark::Point>& radar_info) {
  // cloud_in_->clear();
  // auto trailingPointIter = radar_info.begin();
  // auto max_intensitie_iter = radar_info.begin();
  // auto last_iter = radar_info.begin();
  // for (auto iter = radar_info.begin(); iter != radar_info.end(); iter++) {
  //   if (trailingPointIter == iter) continue;
  //   float continue_dis =
  //     SLAMMath::Dist(iter->mx, iter->my, last_iter->mx, last_iter->my);
  //   float dis_thold = 2 * sin(config_.laser_resolution * M_PI / 360.0) *
  //                     SLAMMath::Dist(iter->mx, iter->my, 0.0f, 0.0f) + 0.05;
  //   if (trailingPointIter->intensitie > config_.landmark_intensities &&
  //      iter->intensitie > config_.landmark_intensities) {
  //     max_intensitie_iter =
  //       max_intensitie_iter->intensitie > iter->intensitie ?
  //       max_intensitie_iter : iter;
  //   } else if (iter->intensitie < config_.landmark_intensities ||
  //             continue_dis > dis_thold) {
  //     // 结束数据记录
  //     int up_count = max_intensitie_iter - trailingPointIter;
  //     int down_count = iter - max_intensitie_iter - 1;
  //     float dis = SLAMMath::Dist(0.0f, 0.0f,
  //                 max_intensitie_iter->mx, max_intensitie_iter->my);
  //     int landmark_size_threshold =
  //       std::atan2(config_.landmark_radius, dis) * 360 /
  //       (M_PI * config_.laser_resolution) - 1;
  //     if (landmark_size_threshold <= 0) landmark_size_threshold = 1;
  //     if (up_count > 0 && down_count > 0 &&
  //         up_count + down_count > landmark_size_threshold) {
  //     float check = up_count > down_count ? up_count / (down_count * 1.0f) :
  //                     down_count / (up_count * 1.0f);
  //       // if (check < 1.3) {
  //         for (auto temp_iter = trailingPointIter;
  //             temp_iter != iter; temp_iter++) {
  //           pcl::PointXYZI temp;
  //           temp.intensity = temp_iter->intensitie;
  //           temp.x = temp_iter->mx;
  //           temp.y = temp_iter->my;
  //           temp.z = 0.0;
  //           cloud_in_->push_back(temp);
  //         }
  //       // }
  //     }
  //     trailingPointIter = iter;
  //     max_intensitie_iter = iter;
  //   } else {
  //     trailingPointIter = iter;
  //     max_intensitie_iter = iter;
  //   }
  //   last_iter = iter;
  // }



  cloud_in_->clear();
  const double minSquareDistance = 0.01;  // in m^2

  auto trailingPointIter = radar_info.begin();

  LandMark::Point firstPoint;
  LandMark::Point rViewPoint(0, 0, 0);
  bool firstTime = true;
  for (auto iter = radar_info.begin(); iter != radar_info.end(); iter++) {
    LandMark::Point currentPoint = *iter;

    if (firstTime && !std::isnan(currentPoint.mx) &&
      !std::isnan(currentPoint.my)) {
      firstPoint = currentPoint;
      firstTime = false;
    }

    double delta = SLAMMath::Dist(firstPoint.mx, firstPoint.my,
                                  currentPoint.mx, currentPoint.my);
    if (delta > minSquareDistance) {
      double a = rViewPoint.my - firstPoint.my;
      double b = firstPoint.mx - rViewPoint.mx;
      double c = firstPoint.my * rViewPoint.mx - firstPoint.mx * rViewPoint.my;
      double ss = currentPoint.mx * a + currentPoint.my * b + c;

      // reset beginning point
      firstPoint = currentPoint;

      if (ss < 0.0)  {
        trailingPointIter = iter;
      } else {
        for (; trailingPointIter != iter; ++trailingPointIter) {
          pcl::PointXYZI temp;
          temp.intensity = trailingPointIter->intensitie;
          temp.x = trailingPointIter->mx;
          temp.y = trailingPointIter->my;
          temp.z = 0.0;
          if (temp.intensity > config_.landmark_intensities &&
              fabs(temp.intensity - 176) > 0.001)
            cloud_in_->push_back(temp);
        }
      }
    }
  }
  return;
}

void Calcuator::_calLandMarkCenter(
  std::map<int, LandMark::Center>* land_mark_pos) {
  for (auto iter = land_mark_pos->begin();
      iter != land_mark_pos->end(); iter++) {
    if (iter->second.land_mark_points.size() < config_.landmark_size_threshold)
      continue;
    // if (iter->second.box.GetHeigth() > 6 * config_.landmark_radius ||
    //     iter->second.box.GetWidth() > 6 *  config_.landmark_radius)
    //     continue;

    // 求解landmark中心
    ceres::Problem problem;
    double center[3] = {0.0, 0.0, config_.landmark_radius};

    for (int i = 0; i < iter->second.land_mark_points.size(); i++) {
      problem.AddResidualBlock(
        new ceres::AutoDiffCostFunction<CicleFittingCost, 1, 3> (
        new CicleFittingCost(iter->second.land_mark_points[i].mx,
                            iter->second.land_mark_points[i].my)),
                            nullptr, center);
    }
    // 配置求解器
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    // options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    // TODO(r) 需检查拟合中心是否异常
    iter->second.mx = center[0];
    iter->second.my = center[1];
    double r = std::sqrt(std::fabs(center[2]));
    // // SLAM_DEBUG("landmark CicleFitting %f %f %f", center[0], center[1], r);
    if (std::fabs(r - config_.landmark_radius) > config_.landmark_radius ||
      !iter->second.box.IsInBox(Point(iter->second.mx, iter->second.my, 0.0))) {
      iter->second.mx =
        0.5 * iter->second.box.GetWidth() + iter->second.box.min_mx;
      iter->second.my =
        0.5 * iter->second.box.GetHeigth() + iter->second.box.min_my;
    }
    iter->second.is_cal_center = true;
  }
}


std::map<int, LandMark::Center> Calcuator::GetLandMarkCenter(bool use_expand) {
  // 完成半径滤波
  pcl::PointCloud<pcl::PointXYZI>::Ptr
    cloud_out(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::RadiusOutlierRemoval<pcl::PointXYZI> cal;
  cal.setInputCloud(cloud_in_);
  cal.setRadiusSearch(config_.landmark_radius);
  cal.setMinNeighborsInRadius(config_.landmark_size_threshold);
  cal.filter(*cloud_out);

  std::map<int, LandMark::Center> land_mark_pos;
  for (auto iter = cloud_out->begin(); iter != cloud_out->end(); iter++) {
    bool is_find_same_landmark = false;
    for (auto land_mark = land_mark_pos.begin();
        land_mark != land_mark_pos.end(); land_mark++) {
      LandMark::Point max_point = land_mark->second.box.max_intensity_point;
      // 和拟合中心点保持距离 且符合激光距离长度
      if (SLAMMath::Dist(iter->x, iter->y,
          max_point.mx, max_point.my) < 4.0f * config_.landmark_radius) {
        is_find_same_landmark = true;
        land_mark->second.land_mark_points.push_back(
          Point(iter->x, iter->y, iter->intensity));
        land_mark->second.box.AddPoint(
          Point(iter->x, iter->y, iter->intensity));
        break;
      }
    }
    if (!is_find_same_landmark) {
      int landmark_id = land_mark_pos.size();
      LandMark::Center land_mark_center;
      land_mark_center.is_cal_center = false;
      land_mark_center.mx = iter->x;
      land_mark_center.my = iter->y;
      land_mark_center.land_mark_points.push_back(
        Point(iter->x, iter->y, iter->intensity));
      land_mark_pos.insert(std::make_pair(landmark_id, land_mark_center));
      land_mark_pos[landmark_id].box.AddPoint(
        Point(iter->x, iter->y, iter->intensity));
    }
  }
  if (use_expand) {
    for (auto iter = land_mark_pos.begin();
        iter != land_mark_pos.end(); iter++) {
      LandMark::Center new_reflectors;
      if (!_checkIsReflector(&iter->second)) continue;
      SLAMMath::ExPandLine(float(0.0), float(0.0),       // NOLINT
                          iter->second.box.max_intensity_point.mx,
                          iter->second.box.max_intensity_point.my,
                          config_.landmark_radius,
                          &iter->second.mx, &iter->second.my);
      float dis_min = FLT_MAX;
      float mx = iter->second.mx, my = iter->second.my;
      for (float delta_x = -0.01; delta_x <= 0.01; delta_x += 0.001) {
        for (float delta_y = -0.01; delta_y <= 0.01; delta_y += 0.001) {
          float dis_sum = 0.0;
          int involved_cirlce_count = 0;
          float mx_t = iter->second.mx + delta_x;
          float my_t = iter->second.my + delta_y;
          for (auto point : iter->second.land_mark_points) {
            if (!point.involved_cirlce) continue;
            involved_cirlce_count++;
            dis_sum += SLAMMath::Dist(mx_t, my_t, point.mx, point.my);
          }
          float dis_delta =
            fabs(involved_cirlce_count * config_.landmark_radius - dis_sum);
          if (dis_delta < dis_min) {
            dis_min = dis_delta;
            mx = mx_t;
            my = my_t;
          }
        }
      }
      iter->second.mx = mx;
      iter->second.my = my;

      // if (!iter->second.box.IsInBox(
      //   Point(iter->second.mx, iter->second.my), config_.landmark_radius)) {
      //   iter->second.mx =
      //     0.5 * iter->second.box.GetWidth() + iter->second.box.min_mx;
      //   iter->second.my =
      //     0.5 * iter->second.box.GetHeigth() + iter->second.box.min_my;
      //   SLAM_WARN("use in box!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
      // }
      iter->second.is_cal_center = true;
    }
  } else {
    _calLandMarkCenter(&land_mark_pos);
  }

  return land_mark_pos;
}

bool Calcuator::_checkIsReflector(LandMark::Center* reflectors) {
  float dis = SLAMMath::Dist(float(0.0), float(0.0),       // NOLINT
                  reflectors->box.max_intensity_point.mx,
                  reflectors->box.max_intensity_point.my);
  if (dis > config_.ladar_max_dis - 5 || dis < config_.ladar_min_dis)
    return false;
  int landmark_size_threshold =
        std::floor(std::atan2(config_.landmark_radius, dis) * 360 /
                  (M_PI * config_.laser_resolution) * 0.6);
  if (landmark_size_threshold <= 0) {
    // SLAM_WARN("watch out landmark_size_threshold size %d",
    //           landmark_size_threshold);
    landmark_size_threshold = 1;
  }
  if (reflectors->land_mark_points.size() < landmark_size_threshold) {
    // SLAM_INFO("reflector size not enough %d < %d",
    //           reflectors.land_mark_points.size(), landmark_size_threshold);
    return false;
  }
  int intensity_threshold = _intensityCurve(dis);
  int count = 0;
  for (int i = 0; i < reflectors->land_mark_points.size(); i++) {
    if (reflectors->land_mark_points[i].intensitie < intensity_threshold) {
      count++;
    } else {
      reflectors->land_mark_points[i].involved_cirlce = true;
    }
  }
  if (reflectors->land_mark_points.size() - count < landmark_size_threshold)
    return false;
  return true;
}

int Calcuator::_intensityCurve(float dis) {
  int result =
    1800 - 197.94 * dis + 10.4 * std::pow(dis, 2) - 0.2 * std::pow(dis, 3);
  result = config_.landmark_intensities;
  // result = 900 - 500 / (1 + 0.5 * std::exp(-0.5*(dis-5)));  // 600kg 适配
  if (dis < 8.0) {
    result = config_.landmark_intensities + 19;
  } else if (dis < 17.0) {
    result = config_.landmark_intensities + 11;
  } else  {
    result = config_.landmark_intensities;
  }
  return result;
}


}  //  namespace LandMark
