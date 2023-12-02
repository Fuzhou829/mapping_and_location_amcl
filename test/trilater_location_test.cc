/*
 * @Description: Copyright (C) 2023 山东亚历山大智能科技有限公司
 * @version: 1.2
 * @Author: renjy
 * @Data: 2023-11-07 14:46:41
 * @LastEditors: renjy
 * @LastEditTime: 2023-11-07 17:28:51
 */
#include <gtest/gtest.h>
#include <memory>
#include <string>

#include "mock/sensor_data_simulation.h"
#include "common/load_config.h"

#include "include/config_struct.h"
#include "mapping_and_location/mapping_and_location.h"
#include "mock/display_location_result.h"
#include "landmark_tool/trilateration.h"
#include "common/logger.h"


TEST(Reflector, location) {
  using namespace gomros::data_process::mapping_and_location;  // NOLINT
  LandMarkConfig land_mark_config;
  land_mark_config.error_threshold_for_loop = 0.07;
  land_mark_config.data_association_max_dis = 40;
  land_mark_config.reflector_max_distance = 30;
  LandMark::Trilateration trilateration_find_sim(land_mark_config);
  std::map<int, std::map<int, Eigen::Vector2d>> maps;
  std::map<int, Eigen::Vector2d> sub_map;
  sub_map.insert(std::make_pair(0, Eigen::Vector2d(-4.301277, -2.732909)));
  sub_map.insert(std::make_pair(1, Eigen::Vector2d(-4.298046, -8.725318)));
  sub_map.insert(std::make_pair(2, Eigen::Vector2d(-4.297701, -14.725192)));
  sub_map.insert(std::make_pair(3, Eigen::Vector2d(17.924196, -8.748401)));
  sub_map.insert(std::make_pair(4, Eigen::Vector2d(17.915564, -2.754477)));
  sub_map.insert(std::make_pair(5, Eigen::Vector2d(10.742188, 1.752130)));
  sub_map.insert(std::make_pair(6, Eigen::Vector2d(2.430039, 1.768712)));
  maps.insert(std::make_pair(-1, sub_map));

  trilateration_find_sim.SetLandMarkMap(maps);

  std::vector<Eigen::Vector2d> obs;
  obs.push_back(Eigen::Vector2d(14.555220, -6.274572));
  obs.push_back(Eigen::Vector2d(14.990592, 5.944454));
  obs.push_back(Eigen::Vector2d(15.202261, 11.922426));
  obs.push_back(Eigen::Vector2d(-7.196738, 6.730520));
  obs.push_back(Eigen::Vector2d(-7.419672, 0.736120));



  Eigen::Vector3d global_pos;
  if (!trilateration_find_sim.IsGetGlobalPos(obs, &global_pos)) {
    SLAM_ERROR("error!!!!!!!!!!");
    return;
  }
  SLAM_INFO("global_pos %f %f %f", global_pos[0], global_pos[1], global_pos[2]);
  // 转换至全局坐标系
  float c = cos(global_pos[2]);
  float s = sin(global_pos[2]);

  for (auto iter : obs) {
    float x_w = iter[0] * c - iter[1] * s + global_pos[0];
    float y_w = iter[0] * s + iter[1] * c + global_pos[1];
    SLAM_INFO("x1 = %f", x_w);
    SLAM_INFO("y1 = %f", y_w);
  }

  return;
}



