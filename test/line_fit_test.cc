/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.0
 * @Author: renjy
 * @Date: 2023-02-06 18:00:05
 * @LastEditTime: 2023-09-01 11:42:57
 */
#include <gtest/gtest.h>
#include "mock/ransac_line_fit.h"
#include "Eigen/Dense"

#include "common/run_time.h"
#include "include/mapping_and_location_math.h"
#include "common/logger.h"

TEST(LineFit, save_smap) {
  std::string save_dir = "/media/sf_share/1111111111/";
  // std::string map_dir = "/media/sf_share/1111111111/0205map.smap";
  std::string map_dir = "/media/sf_share/1111111111/qr_map.smap";
  LineFit::MapFit map_fit(map_dir, save_dir);
}

TEST(math_test, math) {
  uint64_t time = 1689845224503549;
  SLAM_DEBUG("%lu", time);
  float theta = 18446744073616 * 1e-3;
  internal_common::RunTime run_time;
  SLAMMath::NormalizePITheta(theta);
  std::cout << run_time.ElapsedMillisecond() << std::endl;
}


TEST(clock_test, math) {
  double point_v_x = 2;
  double point_v_y = 0;

  double point_s_x = 1;
  double point_s_y = 1;

  double point_e_x = 1;
  double point_e_y = -1;

  double a = point_v_y - point_s_y;
  double b = point_s_x - point_v_x;
  double c = point_s_y * point_v_x -
             point_s_x * point_v_y;
  double ss = point_e_x * a + point_e_y * b + c;

  // ss < 0 顺时针 ss>= 0 逆时针
  std::cout << ss << std::endl;
}

TEST(Test_math, match) {
  Eigen::Matrix3d g_w(3, 3);
  g_w <<
    cos(-1.736216), -sin(-1.736216), -18.268753,
    sin(-1.736216), cos(-1.736216), -2.891556,
    0, 0, 1;

  Eigen::Matrix3d g_c(3, 3);
  g_c <<
    cos(-1.757655), -sin(-1.757655), -18.302299,
    sin(-1.757655), cos(-1.757655), -2.87787,
    0, 0, 1;


  Eigen::Matrix3d result(3, 3);
  result = g_c.inverse() * g_w;

  std::cout << result(0, 2) << " " << result(1, 2) << " " <<
    SLAMMath::NormalizePITheta(atan2(result(1, 0), result(0, 0)));
}
