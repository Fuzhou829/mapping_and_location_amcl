/*
 * @Description: Copyright (C) 2023 山东亚历山大智能科技有限公司
 * @version: 1.2
 * @Author: renjy
 * @Data: 2023-09-21 11:36:36
 * @LastEditors: renjy
 * @LastEditTime: 2023-09-21 15:08:32
 */

#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>

#include "common/load_config.h"
#include "common/logger.h"
#include "reflector_mapping/trilateral_mapping.h"


#include "mock/load_data_for_test.h"



/**
 * @description: 多边建图
 * @return {*}
 */
TEST(Mapping, trilateral_mapping) {
  using namespace gomros::data_process::mapping_and_location;  // NOLINT
  std::string test_dir =
    "../../../../../Dataset/SLAM_Dataset/trilateral_mapping";

  MappingAndLocationConfig config;
  std::string config_dir =
    test_dir + "/mappingandlocation.json";
  if (!internal_common::ReadMappingAndLocationConfig(config_dir, &config)) {
    SLAM_ERROR("can not open config file, please check json!!");
    return;
  }
  int count = 0;
  MergeReflectors merge_reflector_map(config.mapping_config);

  while (1) {
    char buff[512];
    snprintf(buff, sizeof(buff), \
      "%s/ladar_data_%02d.txt", test_dir.c_str(), count++);
    LoadDataForTest::LadarInfo ladar_info_temp;
    if (!LoadDataForTest::ReadLadar(std::string(buff), &ladar_info_temp)) break;
    gomros::message::RadarSensoryInfo radar_info;
    radar_info.mstruSingleLayerData.mvIntensities =
      ladar_info_temp.mvIntensities;
    radar_info.mstruSingleLayerData.mvPoints = ladar_info_temp.mvPoints;
    DataFusion::Observation obs;
    obs.GetObsFromLadarData(config.mapping_config.sensor_mount,
                            config.mapping_config.land_mark_config,
                            radar_info);
    merge_reflector_map.SetReflectorMapInfo(obs.centers);
  }
  std::map<int, std::map<int, Eigen::Vector2d>> result;
  if (!merge_reflector_map.GetMapResult(&result)) {
    SLAM_ERROR("can not get reflector info");
    return;
  }

  float resolution = 0.01;
  float min_x = -50;
  float min_y = -50;
  float max_x = 50;
  float max_y = 50;
  int width = ceil((max_x - min_x) / resolution) + 1;
  int height = ceil((max_y - min_y) / resolution) + 1;

  auto getgrid = [&](float xx, float yy, int* grid_x, int* grid_y) {
    *grid_x = ceil((xx - min_x) / resolution);
    *grid_y = height - ceil((yy - min_y) / resolution);
    return;
  };
  cv::Mat show_map = cv::Mat(cv::Size(width, height), CV_8UC3,
                              cv::Scalar(255, 255, 255));

  SLAM_INFO("reflector size %d", result.at(-1).size());
  for (auto iter : result.at(-1)) {
    int grid_x, grid_y;
    getgrid(iter.second(0), iter.second(1), &grid_x, &grid_y);
    cv::circle(show_map, cv::Point2f(grid_x, grid_y), 5,
                  cv::Scalar(0, 0, 255), -1);
  }

  cv::imwrite("./map.png", show_map);
  return;
}
