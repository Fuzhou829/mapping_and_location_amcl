/*
 * @Description: Copyright (C) 2023 山东亚历山大智能科技有限公司
 * @version: 1.2
 * @Author: renjy
 * @Data: 2023-06-29 08:35:45
 * @LastEditors: renjy
 * @LastEditTime: 2023-09-04 19:34:46
 */


#include <gtest/gtest.h>
#include <memory>

#include "landmark_tool/trilateral_calcuator.h"

TEST(TrilateralCalcuator, SameTrilateral) {
  std::shared_ptr<LandMark::TrilateralCal> trilaterl_cal =
    std::make_shared<LandMark::TrilateralCal>();
  std::vector<Eigen::Vector2d> map_triangle, observation_triangle;
  map_triangle.push_back(Eigen::Vector2d(1.0, 2.0));
  map_triangle.push_back(Eigen::Vector2d(1.0, 4.0));
  map_triangle.push_back(Eigen::Vector2d(3.0, 4.0));

  observation_triangle.push_back(Eigen::Vector2d(1.0, 2.0));
  observation_triangle.push_back(Eigen::Vector2d(1.0, 4.0));
  observation_triangle.push_back(Eigen::Vector2d(3.0, 4.0));
  Eigen::Vector3d result;
  bool get_global_pos =
    trilaterl_cal->GetGlobalPos(map_triangle, observation_triangle, &result);
  EXPECT_TRUE(get_global_pos);
  EXPECT_FLOAT_EQ(result(0), 0.0);
  EXPECT_FLOAT_EQ(result(1), 0.0);
  EXPECT_FLOAT_EQ(result(2), 0.0);
}

TEST(TrilateralCalcuator, Translate) {
  std::shared_ptr<LandMark::TrilateralCal> trilaterl_cal =
    std::make_shared<LandMark::TrilateralCal>();
  std::vector<Eigen::Vector2d> map_triangle, observation_triangle;
  map_triangle.push_back(Eigen::Vector2d(1.0, 2.0));
  map_triangle.push_back(Eigen::Vector2d(1.0, 4.0));
  map_triangle.push_back(Eigen::Vector2d(3.0, 4.0));

  observation_triangle.push_back(Eigen::Vector2d(4.0, 4.0));
  observation_triangle.push_back(Eigen::Vector2d(4.0, 6.0));
  observation_triangle.push_back(Eigen::Vector2d(6.0, 6.0));
  Eigen::Vector3d result;
  bool get_global_pos =
    trilaterl_cal->GetGlobalPos(map_triangle, observation_triangle, &result);
  EXPECT_TRUE(get_global_pos);
  EXPECT_FLOAT_EQ(result(0), -3.0);
  EXPECT_FLOAT_EQ(result(1), -2.0);
  EXPECT_FLOAT_EQ(result(2), 0.0);
}


TEST(TrilateralCalcuator, Rotation) {
  std::shared_ptr<LandMark::TrilateralCal> trilaterl_cal =
    std::make_shared<LandMark::TrilateralCal>();
  std::vector<Eigen::Vector2d> map_triangle, observation_triangle;
  map_triangle.push_back(Eigen::Vector2d(0.0, 2.0));
  map_triangle.push_back(Eigen::Vector2d(0.0, 4.0));
  map_triangle.push_back(Eigen::Vector2d(3.0, 4.0));

  observation_triangle.push_back(Eigen::Vector2d(-2.0, 0.0));
  observation_triangle.push_back(Eigen::Vector2d(-4.0, 0.0));
  observation_triangle.push_back(Eigen::Vector2d(-4.0, 3.0));
  Eigen::Vector3d result;
  bool get_global_pos =
    trilaterl_cal->GetGlobalPos(map_triangle, observation_triangle, &result);
  EXPECT_TRUE(get_global_pos);
  EXPECT_FLOAT_EQ(result(0), 0.0);
  EXPECT_FLOAT_EQ(result(1), 0.0);
  EXPECT_FLOAT_EQ(result(2), -M_PI * 0.5);
}



