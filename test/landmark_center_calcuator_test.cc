/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.0
 * @Author: renjy
 * @Date: 2022-11-26 20:30:38
 * @LastEditTime: 2023-07-31 15:58:05
 */
#include <gtest/gtest.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <fstream>
#include <opencv2/opencv.hpp>

#include "common/run_time.h"
#include "landmark_tool/landmark_center_calcuator.h"

TEST(LandMarkTest, CircleCenter) {
  double x0 = -3.0, y0 = -4.0, r = 0.0375;         // 真实参数值
  int N = 100;                          // 数据点
  double w_sigma = 0.021;                 // 噪声Sigma值
  cv::RNG rng;                        // OpenCV随机数产生器
  double x0y0r[3] = {0, 0, 0};            // abc参数的估计值

  vector<double> x_data, y_data;      // 数据

  for (int i = 0; i < N; i++) {
    double noise_x = x0 + r * cos(i * M_PI / 50) + rng.gaussian(w_sigma);
    double noise_y = y0 + r * sin(i * M_PI / 50) + rng.gaussian(w_sigma);
    x_data.push_back(noise_x);
    y_data.push_back(noise_y);
    // SLAM_INFO("data = %f %f\n", x_data[i], y_data[i]);
  }
  ceres::Problem problem;
  for (int i = 0; i < N; i++) {
      problem.AddResidualBlock(
          new ceres::AutoDiffCostFunction<LandMark::CicleFittingCost, 1, 3>(
              new LandMark::CicleFittingCost(x_data[i], y_data[i])),
          nullptr, x0y0r);
  }
  // 配置求解器
  ceres::Solver::Options options;     // 这里有很多配置项可以填
  options.linear_solver_type = ceres::DENSE_QR;  // 增量方程如何求解
  options.minimizer_progress_to_stdout = true;   // 输出到cout

  ceres::Solver::Summary summary;                // 优化信息

  ceres::Solve(options, &problem, &summary);  // 开始优化
  // SLAM_INFO("circle %f %f %f", x0y0r[0], x0y0r[1], x0y0r[2]);
}




TEST(LandMarkTest, CenterCalcuator) {
  gomros::data_process::mapping_and_location::MappingConfig config;
  LandMark::Calcuator land_mark_cal(config.land_mark_config);
  std::ifstream infile;
  std::string file_name = "./landmark_data.txt";
  infile.open(file_name.c_str(), std::ifstream::in);
  if (!infile.is_open()) {
    // SLAM_ERROR("can not open %s", file_name.c_str());
    return;
  }
  LandMark::LandMarkBox landmark_map;
  LandMark::Point candidate_point;
  LandMark::Point sensor_pos;
  std::map<int, LandMark::Center> land_mark_pos;
  std::string line;
  while (std::getline(infile, line)) {
    std::istringstream iss(line);
    if (!(iss >> sensor_pos.intensitie >> sensor_pos.mx >> sensor_pos.my
              >> candidate_point.mx >> candidate_point.my)) {
      break;
    }
    candidate_point.intensitie = sensor_pos.intensitie;
    land_mark_cal.AddLandMarkCenter(candidate_point);
    landmark_map.AddPoint(candidate_point);
  }
  land_mark_pos = land_mark_cal.GetLandMarkCenter(true);

  float resolution = 0.02;
  int width = landmark_map.GetWidth() / resolution + 10;
  int heigth = landmark_map.GetHeigth() / resolution + 10;
  int map_size = std::max(width, heigth);
  cv::Mat show_map_test =
      cv::Mat(cv::Size(map_size, map_size), CV_8UC3, cv::Scalar(255, 255, 255));

  float offset = -0.5 * resolution * map_size;

  // 标记landmark信息
  for (auto iter = land_mark_pos.begin(); iter != land_mark_pos.end(); iter++) {
    float mx = iter->second.mx;
    float my = iter->second.my;
    int grid_x = (mx - offset) / resolution;
    int grid_y = (my - offset) / resolution;
    // SLAM_INFO("landmark %d is have center %d grid(%d, %d) pos(%f %f)",
          // iter->first, iter->second.is_cal_center, grid_x, grid_y, mx, my);
    if (!iter->second.is_cal_center)
      continue;
    for (auto data = iter->second.land_mark_points.begin();
         data != iter->second.land_mark_points.end(); data++) {
      grid_x = (data->mx - offset) / resolution;
      grid_y = (data->my - offset) / resolution;
      // SLAM_INFO("point grid (%d %d) pos(%f %f)",
                // grid_x, grid_y, data->mx, data->my);
      show_map_test.at<cv::Vec3b>(grid_y, grid_x)[0] = 0;
      show_map_test.at<cv::Vec3b>(grid_y, grid_x)[1] = 255;
      show_map_test.at<cv::Vec3b>(grid_y, grid_x)[2] = 0;
    }
    show_map_test.at<cv::Vec3b>(grid_y, grid_x)[0] = 0;
    show_map_test.at<cv::Vec3b>(grid_y, grid_x)[1] = 0;
    show_map_test.at<cv::Vec3b>(grid_y, grid_x)[2] = 255;
  }

  std::map<int, LandMark::Center> temp =
    land_mark_cal.GetLandMarkCenter(true);
}

TEST(LandMarkTest, RadiusOutlierRemoval) {
  std::ifstream infile;
  std::string file_name = "./landmark_data.txt";
  infile.open(file_name.c_str(), std::ifstream::in);
  if (!infile.is_open()) {
    // SLAM_ERROR("can not open %s", file_name.c_str());
    return;
  }
  LandMark::LandMarkBox landmark_map;
  LandMark::Point candidate_point;
  LandMark::Point sensor_pos;

  std::string line;
  float intensity;
  std::map<int, LandMark::Center> land_mark_pos;
  gomros::data_process::mapping_and_location::MappingConfig config;
  LandMark::Calcuator land_mark_cal(config.land_mark_config);
  while (std::getline(infile, line)) {
    std::istringstream iss(line);
    if (!(iss >> sensor_pos.intensitie >> sensor_pos.mx >> sensor_pos.my
              >> candidate_point.mx >> candidate_point.my)) {
      break;
    }
    candidate_point.intensitie = sensor_pos.intensitie;
    if (intensity < config.land_mark_config.landmark_intensities) continue;
    land_mark_cal.AddLandMarkCenter(candidate_point);
    landmark_map.AddPoint(candidate_point);
  }
  land_mark_pos = land_mark_cal.GetLandMarkCenter(true);

  float resolution = 0.02;
  int width = landmark_map.GetWidth() / resolution + 10;
  int height = landmark_map.GetHeigth() / resolution + 10;
  int map_size = std::max(width, height);
  cv::Mat show_map_test =
      cv::Mat(cv::Size(width, height), CV_8UC3, cv::Scalar(255, 255, 255));

  float offset_x = -0.5 * resolution * width;
  float offset_y = -0.5 * resolution * height;

  // 标记landmark信息
  for (auto iter = land_mark_pos.begin(); iter != land_mark_pos.end(); iter++) {
    float mx = iter->second.mx;
    float my = iter->second.my;
    int grid_x = (mx - offset_x) / resolution;
    int grid_y = (my - offset_y) / resolution;
    // SLAM_INFO("landmark %d is have center %d grid(%d, %d) pos(%f %f)",
        // iter->first, iter->second.is_cal_center, grid_x, grid_y, mx, my);
    if (!iter->second.is_cal_center)
      continue;
    for (auto data = iter->second.land_mark_points.begin();
         data != iter->second.land_mark_points.end(); data++) {
      grid_x = (data->mx - offset_x) / resolution;
      grid_y = (data->my - offset_y) / resolution;
      // SLAM_INFO("point grid (%d %d) pos(%f %f)",
          // grid_x, grid_y, data->mx, data->my);
      show_map_test.at<cv::Vec3b>(grid_y, grid_x)[0] = 0;
      show_map_test.at<cv::Vec3b>(grid_y, grid_x)[1] = 255;
      show_map_test.at<cv::Vec3b>(grid_y, grid_x)[2] = 0;
    }
    grid_x = (mx - offset_x) / resolution;
    grid_y = (my - offset_y) / resolution;
    show_map_test.at<cv::Vec3b>(grid_y, grid_x)[0] = 0;
    show_map_test.at<cv::Vec3b>(grid_y, grid_x)[1] = 0;
    show_map_test.at<cv::Vec3b>(grid_y, grid_x)[2] = 255;
  }
  cv::imwrite("./landmark.jpg", show_map_test);
  cv::imshow("landmark_points", show_map_test);
  cv::waitKey();
}
