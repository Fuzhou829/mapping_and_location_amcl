/*
 * @Description: Copyright (C) 2023 山东亚历山大智能科技有限公司
 * @version: 1.2
 * @Author: renjy
 * @Data: 2023-07-08 01:34:57
 * @LastEditors: renjy
 * @LastEditTime: 2023-10-20 16:50:24
 */


#include <gtest/gtest.h>
#include <string>
#include <fstream>

#include <opencv2/opencv.hpp>
#include "common/load_config.h"
#include "common/tool.h"
#include "common/transform.h"
#include "ekf_calcuator/reflector_ekf.h"

#include "message_lib/radar_message.h"
#include "include/mapping_and_location_math.h"
#include "mock/bresenham_line.h"
#include "ekf_calcuator/recursive_algorithm.h"

#include "landmark_tool/trilateral_calcuator.h"
#include "landmark_tool/trilateration.h"

TEST(ReflectorRecognition, ladar2ladar_WanJi) {
  int count = 0;
  struct LadarInfo {
    float mx;
    float my;
    float dis;
    int intensitie;
  };
  std::string test_dir = "../../../../../Dataset/SLAM_Dataset/";

  gomros::data_process::mapping_and_location::MappingAndLocationConfig config;
  std::string config_dir =
    test_dir + "reflector_mapping/data14/mappingandlocation.json";
  if (!internal_common::ReadMappingAndLocationConfig(config_dir, &config)) {
    SLAM_ERROR("can not open config file, please check json!!");
    return;
  }
  std::string out_file;
  if (config.mapping_config.land_mark_config.recognition_type ==
      gomros::data_process::mapping_and_location::RecognitionType::Cluster) {
    out_file = "cluster";
  } else {
    out_file = "find_peak";
  }
  std::map<float, LadarInfo> sort_intensitie;
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

  std::map<uint64_t, int> jump_time;
  std::string jump_time_dir =
    test_dir + "wanji_ladar_data/jump_time.txt";
  std::ifstream jump_infile;
  jump_infile.open(jump_time_dir.c_str(), std::ifstream::in);
  if (!jump_infile.is_open()) {
    SLAM_DEBUG("can not open %s", jump_time_dir.c_str());
  }

  std::string time_line;

  while (std::getline(jump_infile, time_line)) {
    std::istringstream iss(time_line);
    uint64_t time;
    if (!(iss >> time)) break;
    jump_time.insert(std::make_pair(time, 1));
  }

  std::string map_dir = test_dir + "reflector_mapping/data14/ale.smap";
  std::map<int, std::map<int, Eigen::Vector2d>> map_info;
  internal_common::ReadLandmarkInfoFromFile(map_dir, &map_info);
  LandMark::Trilateration global_trilateration(
    config.location_config.land_mark_config);
  global_trilateration.SetLandMarkMap(map_info);

  bool is_next = true;
  cv::Mat show_ladar_temp = cv::Mat(cv::Size(width, height), CV_8UC3,
                              cv::Scalar(255, 255, 255));
  while (1) {
    char buff[512];
    snprintf(buff, sizeof(buff), \
  "%sreflector_mapping/data14/ladar_data_%02d.txt", test_dir.c_str(), count++);
    std::ifstream infile;
    infile.open(buff, std::ifstream::in);
    if (!infile.is_open()) break;
    SLAM_INFO("file name %s", buff);
    cv::Mat show_ladar = show_ladar_temp.clone();
    std::vector<LadarInfo> ladars_data;
    std::string line;
    uint64_t time = 0;
    while (std::getline(infile, line)) {
      std::istringstream iss(line);
      if (0 == time) {
        if (!(iss >> time)) break;
        continue;
      }
      LadarInfo ladar_info;
      if (!(iss >> ladar_info.mx >> ladar_info.my >> ladar_info.intensitie))
        break;
      ladar_info.dis = SLAMMath::Dist(ladar_info.mx, ladar_info.my,
                  static_cast<float>(0.0), static_cast<float>(0.0));
      sort_intensitie.insert(std::make_pair(ladar_info.dis, ladar_info));
      // SLAM_INFO("ladar_info %f %f %d %f",
      //           ladar_info.mx, ladar_info.my, ladar_info.intensitie, dis);
      ladars_data.push_back(ladar_info);
    }
    // 只做位姿有跳变的数据
    // if (!jump_time.count(time) && is_next && count != 1) continue;
    is_next = false;
    if (!jump_time.count(time)) {
      is_next = true;
    }
    gomros::message::RadarSensoryInfo radar_info;

    for (auto iter : ladars_data) {
      int grid_x, grid_y;
      getgrid(iter.mx, iter.my, &grid_x, &grid_y);
      radar_info.mstruSingleLayerData.mvPoints.push_back(
        Eigen::Vector2d(iter.mx, iter.my));
      radar_info.mstruSingleLayerData.mvIntensities.push_back(iter.intensitie);
    }

    DataFusion::Observation obs;
    obs.GetObsFromLadarData(config.mapping_config.sensor_mount,
                            config.mapping_config.land_mark_config,
                            radar_info);
    SLAM_INFO("get obs size %d", obs.centers.size());
    Eigen::Vector3d global_pose(0.0, 0.0, 0.0);
    if (!global_trilateration.IsGetGlobalPos(obs.centers, &global_pose)) {
      SLAM_INFO("global pos fail");
      // continue;
    }
    std::map<int, int> result;
    global_trilateration.GetMatchId(&result);
    SLAM_INFO("match result size %d", result.size());
    for (auto iter : result) {
      SLAM_INFO("match result map id %d obs id %d", iter.first, iter.second);
    }
    Coordinate::Transform transform;
    transform.SetPose1InGlobal(global_pose);
    int current_grid_x, current_grid_y;
    getgrid(global_pose(0), global_pose(1), &current_grid_x, &current_grid_y);
    int ladar_count = 0;
    for (auto iter : ladars_data) {
      transform.SetPose2InPose1(Eigen::Vector3d(iter.mx, iter.my, 0.0));
      int grid_x, grid_y;
      Eigen::Vector3d pose;
      transform.GetPose2InGlobal(&pose);
      getgrid(pose(0), pose(1), &grid_x, &grid_y);
      LineMath::BresenhamLine line;
      line.SetStartAndEndGrid(grid_x, grid_y,
      current_grid_x, current_grid_y);
      if (ladar_count == 0) {
        SLAM_INFO("first grid %d %d", grid_x, grid_y);
      }
      int next_x, next_y;
      while (line.GetNextGrid(&next_x, &next_y)) {
        if (next_x < 0 || next_x >= width ||
            next_y < 0 || next_y >= height) continue;
        if (count == 1) {
          show_ladar.at<cv::Vec3b>(next_y, next_x)[0] = 0;
          show_ladar.at<cv::Vec3b>(next_y, next_x)[1] = 255;
          show_ladar.at<cv::Vec3b>(next_y, next_x)[2] = 0;
        } else {
          show_ladar.at<cv::Vec3b>(next_y, next_x)[0] = 192;
          show_ladar.at<cv::Vec3b>(next_y, next_x)[1] = 192;
          show_ladar.at<cv::Vec3b>(next_y, next_x)[2] = 192;
        }
      }
      show_ladar.at<cv::Vec3b>(grid_y, grid_x)[0] = iter.intensitie % 255;
      show_ladar.at<cv::Vec3b>(grid_y, grid_x)[1] = 255;
      show_ladar.at<cv::Vec3b>(grid_y, grid_x)[2] = 0;
      ladar_count++;
    }
    std::map<int, Eigen::Vector2d> maps =
      map_info.at(global_trilateration.GetMatchSubmapId());
    for (auto iter : maps) {
      int grid_x, grid_y;
      getgrid(iter.second(0), iter.second(1), &grid_x, &grid_y);
      SLAM_INFO("map index %d %d %d", iter.first, grid_x, grid_y);
      cv::circle(show_ladar, cv::Point2f(grid_x, grid_y), 3,
                  cv::Scalar(0, 0, 255), -1);
    }


    int line_color = 0;
    for (auto iter : obs.centers) {
      transform.SetPose2InPose1(Eigen::Vector3d(iter(0), iter(1), 0.0));
      int grid_x, grid_y;
      Eigen::Vector3d pose;
      transform.GetPose2InGlobal(&pose);
      getgrid(pose(0), pose(1), &grid_x, &grid_y);
      SLAM_INFO("obs index %d %d %d", line_color, grid_x, grid_y);
      // 画线 灰色
      LineMath::BresenhamLine line;
      line.SetStartAndEndGrid(grid_x, grid_y,
      current_grid_x, current_grid_y);
      int next_x, next_y;
      while (line.GetNextGrid(&next_x, &next_y)) {
        if (next_x < 0 || next_x >= width ||
            next_y < 0 || next_y >= height) continue;
        show_ladar.at<cv::Vec3b>(next_y, next_x)[0] = line_color;
        show_ladar.at<cv::Vec3b>(next_y, next_x)[1] = 0;
        show_ladar.at<cv::Vec3b>(next_y, next_x)[2] = 255;
      }
      line_color++;
      cv::circle(show_ladar, cv::Point2f(grid_x, grid_y), 4,
                  cv::Scalar(255, 0, 0), 1);
    }

    if (count == 1) {
      show_ladar_temp = show_ladar.clone();
    }
    char picture_buff[512];
    snprintf(picture_buff, sizeof(picture_buff), \
      "./out/%s_recognition_%02d.png", out_file.c_str(), count - 1);
    cv::imwrite(picture_buff, show_ladar);

    if (count % 20 == 0) {
      SLAM_INFO("watch out~~~~~~~~~~~~~~");
    }
  }

  for (auto iter = sort_intensitie.begin();
      iter != sort_intensitie.end(); iter++) {
    if (iter->second.intensitie <
      config.mapping_config.land_mark_config.landmark_intensities) continue;
    SLAM_INFO("ladar_info %d %f", iter->second.intensitie, iter->second.dis);
  }
  return;
}

TEST(ReflectorRecognition, WanJi) {
  int count = 0;
  struct LadarInfo {
    float mx;
    float my;
    float dis;
    int intensitie;
  };
  std::string test_dir = "../../../../../Dataset/SLAM_Dataset/";

  gomros::data_process::mapping_and_location::MappingAndLocationConfig config;
  std::string config_dir =
    test_dir + "wanji_ladar_data/mappingandlocation.json";
  if (!internal_common::ReadMappingAndLocationConfig(config_dir, &config)) {
    SLAM_ERROR("can not open config file, please check json!!");
    return;
  }
  std::string out_file;
  if (config.mapping_config.land_mark_config.recognition_type ==
      gomros::data_process::mapping_and_location::RecognitionType::Cluster) {
    out_file = "cluster";
  } else {
    out_file = "find_peak";
  }
  std::map<float, LadarInfo> sort_intensitie;

  float resolution = 0.01;
  DataFusion::RecursiveAlgorithm data_fusion(0.01, 0.01);
  while (1) {
    // 数据读取
    char buff[512];
    snprintf(buff, sizeof(buff), \
        "%swanji_ladar_data/ladar_data_%02d.txt", test_dir.c_str(), count++);
    std::ifstream infile;
    infile.open(buff, std::ifstream::in);
    if (!infile.is_open()) break;
    SLAM_INFO("file name %s", buff);

    uint64_t time = 0;
    float min_x = FLT_MAX;
    float min_y = FLT_MAX;
    float max_x = -FLT_MAX;
    float max_y = -FLT_MAX;
    std::vector<LadarInfo> ladars_data;
    std::string line;
    while (std::getline(infile, line)) {
      std::istringstream iss(line);
      if (0 == time) {
        if (!(iss >> time)) break;
        continue;
      }
      LadarInfo ladar_info;
      if (!(iss >> ladar_info.mx >> ladar_info.my >> ladar_info.intensitie))
        break;
      ladar_info.dis = SLAMMath::Dist(ladar_info.mx, ladar_info.my,
                  static_cast<float>(0.0), static_cast<float>(0.0));
      sort_intensitie.insert(std::make_pair(ladar_info.dis, ladar_info));
      ladars_data.push_back(ladar_info);
      if (ladar_info.mx < min_x) min_x = ladar_info.mx;
      if (ladar_info.my < min_y) min_y = ladar_info.my;
      if (ladar_info.mx > max_x) max_x = ladar_info.mx;
      if (ladar_info.my > max_y) max_y = ladar_info.my;
    }
    int width = ceil((max_x - min_x) / resolution) + 1;
    int height = ceil((max_y - min_y) / resolution) + 1;

    auto getgrid = [&](float xx, float yy, int* grid_x, int* grid_y) {
      *grid_x = ceil((xx - min_x) / resolution);
      *grid_y = height - ceil((yy - min_y) / resolution);
      return;
    };
    cv::Mat show_ladar_temp = cv::Mat(cv::Size(width, height), CV_8UC3,
                              cv::Scalar(255, 255, 255));
    cv::Mat show_ladar = show_ladar_temp.clone();


    // 放入雷达数据结构体中
    gomros::message::RadarSensoryInfo radar_info;
    for (auto iter : ladars_data) {
      int grid_x, grid_y;
      getgrid(iter.mx, iter.my, &grid_x, &grid_y);
      radar_info.mstruSingleLayerData.mvPoints.push_back(
        Eigen::Vector2d(iter.mx, iter.my));
      radar_info.mstruSingleLayerData.mvIntensities.push_back(iter.intensitie);
    }

    DataFusion::Observation obs;
    obs.GetObsFromLadarData(config.mapping_config.sensor_mount,
                            config.mapping_config.land_mark_config,
                            radar_info);
    SLAM_INFO("get obs size %d", obs.centers.size());
    data_fusion.AddPoint(obs.centers);

    int current_grid_x, current_grid_y;
    getgrid(0.0, 0.0, &current_grid_x, &current_grid_y);
    SLAM_INFO("current_grid %d %d", current_grid_x, current_grid_y);
    int ladar_count = 0;
    for (auto iter : ladars_data) {
      int grid_x, grid_y;
      getgrid(iter.mx, iter.my, &grid_x, &grid_y);
      LineMath::BresenhamLine line;
      line.SetStartAndEndGrid(grid_x, grid_y,
      current_grid_x, current_grid_y);
      if (ladar_count == 0) {
        SLAM_INFO("first grid %d %d", grid_x, grid_y);
      }
      int next_x, next_y;
      while (line.GetNextGrid(&next_x, &next_y)) {
        if (next_x < 0 || next_x >= width ||
            next_y < 0 || next_y >= height) continue;
        show_ladar.at<cv::Vec3b>(next_y, next_x)[0] = 192;
        show_ladar.at<cv::Vec3b>(next_y, next_x)[1] = 192;
        show_ladar.at<cv::Vec3b>(next_y, next_x)[2] = 192;
      }
      show_ladar.at<cv::Vec3b>(grid_y, grid_x)[0] = iter.intensitie % 255;
      show_ladar.at<cv::Vec3b>(grid_y, grid_x)[1] = 255;
      show_ladar.at<cv::Vec3b>(grid_y, grid_x)[2] = 0;
      ladar_count++;
    }
    for (auto iter : ladars_data) {
      int grid_x, grid_y;
      getgrid(iter.mx, iter.my, &grid_x, &grid_y);
      show_ladar.at<cv::Vec3b>(grid_y, grid_x)[0] = iter.intensitie % 255;
      show_ladar.at<cv::Vec3b>(grid_y, grid_x)[1] = 255;
      show_ladar.at<cv::Vec3b>(grid_y, grid_x)[2] = 0;
    }
    show_ladar_temp = show_ladar.clone();
    int line_color = 0;
    for (auto iter : obs.centers) {
      int grid_x, grid_y;
      getgrid(iter(0), iter(1), &grid_x, &grid_y);
      float dis = SLAMMath::Dist(iter(0), iter(1), 0.0, 0.0);
      SLAM_INFO("obs index %d %d %d %f %f dis %f",
                  line_color, grid_x, grid_y, iter(0), iter(1), dis);
      // 画线 灰色
      LineMath::BresenhamLine line;
      line.SetStartAndEndGrid(grid_x, grid_y,
      current_grid_x, current_grid_y);
      int next_x, next_y;
      while (line.GetNextGrid(&next_x, &next_y)) {
        if (next_x < 0 || next_x >= width ||
            next_y < 0 || next_y >= height) continue;
        show_ladar.at<cv::Vec3b>(next_y, next_x)[0] = line_color;
        show_ladar.at<cv::Vec3b>(next_y, next_x)[1] = 0;
        show_ladar.at<cv::Vec3b>(next_y, next_x)[2] = 255;
      }
      line_color++;
      cv::circle(show_ladar, cv::Point2f(grid_x, grid_y), 4,
                  cv::Scalar(255, 0, 0), 1);
    }

    char picture_buff[512];
    snprintf(picture_buff, sizeof(picture_buff), \
      "./out/%s_recognition_%02d.png", out_file.c_str(), count - 1);
    cv::imwrite(picture_buff, show_ladar);

    if (count % 10 == 0) {
      SLAM_INFO("watch out~~~~~~~~~~~~~~");
    }
  }
  std::vector<Eigen::Vector2d> obs_centers;
  data_fusion.GetResult(&obs_centers);
  for (auto iter : obs_centers) {
    SLAM_INFO("data_fusion obs %f %f", iter(0), iter(1));
  }

  for (auto iter = sort_intensitie.begin();
      iter != sort_intensitie.end(); iter++) {
    if (iter->second.intensitie <
      config.mapping_config.land_mark_config.landmark_intensities ||
      iter->second.intensitie == 176) continue;
    SLAM_INFO("ladar_info %d %f", iter->second.intensitie, iter->second.dis);
  }
  return;
}


TEST(ReflectorRecognition, Pepperl) {
  int count = 0;
  struct LadarInfo {
    float mx;
    float my;
    float dis;
    int intensitie;
  };
  std::string test_dir = "../../../../../Dataset/SLAM_Dataset/";

  gomros::data_process::mapping_and_location::MappingAndLocationConfig config;
  std::string config_dir =
    test_dir + "Pepperl_ladar_data/mappingandlocation.json";
  if (!internal_common::ReadMappingAndLocationConfig(config_dir, &config)) {
    SLAM_ERROR("can not open config file, please check json!!");
    return;
  }
  std::string out_file;
  if (config.mapping_config.land_mark_config.recognition_type ==
      gomros::data_process::mapping_and_location::RecognitionType::Cluster) {
    out_file = "cluster";
  } else {
    out_file = "find_peak";
  }
  std::map<float, LadarInfo> sort_intensitie;
  // float resolution = 0.01;
  // float min_x = -40;
  // float min_y = -40;
  // float max_x = 40;
  // float max_y = 40;
  // int width = ceil((max_x - min_x) / resolution) + 1;
  // int height = ceil((max_y - min_y) / resolution) + 1;
  // cv::Mat show_ladar = cv::Mat(cv::Size(width, height), CV_8UC3,
  //                           cv::Scalar(255, 255, 255));
  // cv::Mat show_ladar_temp = cv::Mat(cv::Size(width, height), CV_8UC3,
  //                           cv::Scalar(255, 255, 255));
  while (1) {
    char buff[512];
    snprintf(buff, sizeof(buff), \
      "%sPepperl_ladar_data/ladar_data_%02d.txt", test_dir.c_str(), count++);
    std::ifstream infile;
    infile.open(buff, std::ifstream::in);
    if (!infile.is_open()) break;
    SLAM_INFO("file name %s", buff);
    float min_x = FLT_MAX;
    float min_y = FLT_MAX;
    float max_x = -FLT_MAX;
    float max_y = -FLT_MAX;
    std::vector<LadarInfo> ladars_data;
    std::string line;
    int count = 0;
    while (std::getline(infile, line)) {
      if (count++ == 0) continue;

      std::istringstream iss(line);
      LadarInfo ladar_info;
      if (!(iss >> ladar_info.mx >> ladar_info.my >> ladar_info.intensitie))
        break;
      ladar_info.dis = SLAMMath::Dist(ladar_info.mx, ladar_info.my,
                  static_cast<float>(0.0), static_cast<float>(0.0));
      sort_intensitie.insert(std::make_pair(ladar_info.dis, ladar_info));
      // SLAM_INFO("ladar_info %f %f %d %f",
      //           ladar_info.mx, ladar_info.my, ladar_info.intensitie, dis);
      ladars_data.push_back(ladar_info);
      if (ladar_info.mx < min_x) min_x = ladar_info.mx;
      if (ladar_info.my < min_y) min_y = ladar_info.my;
      if (ladar_info.mx > max_x) max_x = ladar_info.mx;
      if (ladar_info.my > max_y) max_y = ladar_info.my;
    }

    float resolution = 0.01;
    int width = ceil((max_x - min_x) / resolution) + 1;
    int height = ceil((max_y - min_y) / resolution) + 1;
    cv::Mat show_ladar = cv::Mat(cv::Size(width, height), CV_8UC3,
                                cv::Scalar(255, 255, 255));
    cv::Mat show_ladar_temp = cv::Mat(cv::Size(width, height), CV_8UC3,
                                cv::Scalar(255, 255, 255));

    auto getgrid = [&](float xx, float yy, int* grid_x, int* grid_y) {
        *grid_x = ceil((xx - min_x) / resolution);
        *grid_y = ceil((yy - min_y) / resolution);
        return;
    };

    gomros::message::RadarSensoryInfo radar_info;
    int current_grid_x, current_grid_y;
    getgrid(0.0, 0.0, &current_grid_x, &current_grid_y);

    for (auto iter : ladars_data) {
      int grid_x, grid_y;
      getgrid(iter.mx, iter.my, &grid_x, &grid_y);
      radar_info.mstruSingleLayerData.mvPoints.push_back(
        Eigen::Vector2d(iter.mx, iter.my));
      radar_info.mstruSingleLayerData.mvIntensities.push_back(iter.intensitie);
      int data = iter.intensitie / 10 > 255 ? 255 : iter.intensitie / 10;
      LineMath::BresenhamLine line;
      line.SetStartAndEndGrid(grid_x, grid_y, current_grid_x, current_grid_y);
      int next_x, next_y;
      while (line.GetNextGrid(&next_x, &next_y)) {
        if (next_x < 0 || next_x >= width ||
            next_y < 0 || next_y >= height) continue;
        show_ladar.at<cv::Vec3b>(next_y, next_x)[0] = 192;
        show_ladar.at<cv::Vec3b>(next_y, next_x)[1] = 192;
        show_ladar.at<cv::Vec3b>(next_y, next_x)[2] = 192;
      }
      show_ladar.at<cv::Vec3b>(grid_y, grid_x)[0] = data;
      show_ladar.at<cv::Vec3b>(grid_y, grid_x)[1] = data;
      show_ladar.at<cv::Vec3b>(grid_y, grid_x)[2] = data;
      show_ladar_temp.at<cv::Vec3b>(grid_y, grid_x)[0] = data;
      show_ladar_temp.at<cv::Vec3b>(grid_y, grid_x)[1] = data;
      show_ladar_temp.at<cv::Vec3b>(grid_y, grid_x)[2] = data;
    }

    DataFusion::Observation obs;
    obs.GetObsFromLadarData(config.mapping_config.sensor_mount,
                            config.mapping_config.land_mark_config,
                            radar_info);
    SLAM_INFO("get obs size %d", obs.centers.size());
    int line_color = 1;
    int r =
      ceil(config.mapping_config.land_mark_config.landmark_radius / resolution);
    for (auto iter : obs.centers) {
      int grid_x, grid_y;
      getgrid(iter(0), iter(1), &grid_x, &grid_y);
      SLAM_INFO("reflector_mapping index %d %f %f", count, iter(0), iter(1));
      float dis = SLAMMath::Dist(iter(0), iter(1), 0.0, 0.0);
      line_color++;
      SLAM_INFO("slam->grid %f %f, %d %d, dis %f line_color %d",
                iter(0), iter(1), grid_x, grid_y, dis, line_color);
      // 画线 灰色
      LineMath::BresenhamLine line;
      line.SetStartAndEndGrid(grid_x, grid_y, current_grid_x, current_grid_y);
      int next_x, next_y;
      while (line.GetNextGrid(&next_x, &next_y)) {
        if (next_x < 0 || next_x >= width ||
            next_y < 0 || next_y >= height) continue;
        show_ladar.at<cv::Vec3b>(next_y, next_x)[0] = line_color;
        show_ladar.at<cv::Vec3b>(next_y, next_x)[1] = 0;
        show_ladar.at<cv::Vec3b>(next_y, next_x)[2] = 255;
      }
      cv::circle(show_ladar, cv::Point2f(grid_x, grid_y), r,
                  cv::Scalar(0, 0, 255), 1);
      // show_ladar.at<cv::Vec3b>(grid_y, grid_x)[0] = 0;
      // show_ladar.at<cv::Vec3b>(grid_y, grid_x)[1] = 0;
      // show_ladar.at<cv::Vec3b>(grid_y, grid_x)[2] = 255;
    }
    char picture_buff[512];
    snprintf(picture_buff, sizeof(picture_buff), \
      "./out/%s_recognition_%02d.png", out_file.c_str(), count - 1);
    cv::imwrite(picture_buff, show_ladar);

    if (count % 5 == 0) {
      SLAM_INFO("watch out~~~~~~~~~~~~~~");
    }
  }

  for (auto iter = sort_intensitie.begin();
      iter != sort_intensitie.end(); iter++) {
      if (iter->second.intensitie <
        config.mapping_config.land_mark_config.landmark_intensities) continue;
      SLAM_INFO("ladar_info %d %f", iter->second.intensitie, iter->second.dis);
  }
  return;
}



TEST(compare, map_obs) {
  // 读取地图
  std::string test_map_dir =
  "../../../../../Dataset/SLAM_Dataset/map/ale.smap";
    SLAM_DEBUG("load map dir is %s", test_map_dir.c_str());
  std::string map_data;
  if (!internal_common::Read(test_map_dir.c_str(), &map_data)) {
    SLAM_ERROR("read map file failed....");
    return;
  }
  Json::Reader reader;
  Json::Value map_json;
  if (!reader.parse(map_data, map_json)) {
    SLAM_ERROR("parse map failed.....");
    return;
  }
  std::map<int, std::map<int, Eigen::Vector2d>> map_info;
  internal_common::ReadLandmarkInfoFromJson(map_json, &map_info);
  if (!map_info.count(-1)) return;
  float min_x = -40;
  float min_y = -40;
  float max_x = 40;
  float max_y = 40;
  for (auto iter : map_info.at(-1)) {
    if (iter.second(0) < min_x) min_x = iter.second(0);
    if (iter.second(1) < min_y) min_y = iter.second(1);
    if (iter.second(0) > max_x) max_x = iter.second(0);
    if (iter.second(1) > max_y) max_y = iter.second(1);
  }

  float resolution = 0.01;
  int width = ceil((max_x - min_x) / resolution) + 1;
  int height = ceil((max_y - min_y) / resolution) + 1;
  cv::Mat show_ladar = cv::Mat(cv::Size(width, height), CV_8UC3,
                              cv::Scalar(255, 255, 255));
  auto getgrid = [&](float xx, float yy, int* grid_x, int* grid_y) {
      *grid_x = ceil((xx - min_x) / resolution);
      *grid_y = ceil((yy - min_y) / resolution);
      return;
  };

  for (auto iter :  map_info.at(-1)) {
    int grid_x, grid_y;
    getgrid(iter.second(0), iter.second(1), &grid_x, &grid_y);
    SLAM_INFO("map info %d %d", grid_x, grid_y);
    show_ladar.at<cv::Vec3b>(grid_y, grid_x)[0] = 0;
    show_ladar.at<cv::Vec3b>(grid_y, grid_x)[1] = 0;
    show_ladar.at<cv::Vec3b>(grid_y, grid_x)[2] = 255;
  }

  // 读取识别到的反光柱子

  char buff[512];
  snprintf(buff, sizeof(buff), \
      "../../../../../Dataset/SLAM_Dataset/map/obs.txt");
  std::ifstream infile;
  infile.open(buff, std::ifstream::in);
  if (!infile.is_open()) return;
  SLAM_INFO("file name %s", buff);
  std::string line;
  // -0.026294 -0.035509 3.076487
  float ladar_global_x = -0.026294;
  float ladar_global_y = -0.035509;
  float ladar_global_t = 3.076487;
  Eigen::Matrix3d ladar_global(3, 3);
  ladar_global <<
    cos(ladar_global_t), -sin(ladar_global_t), ladar_global_x,
    sin(ladar_global_t), cos(ladar_global_t), ladar_global_y,
    0, 0, 1;
  while (std::getline(infile, line)) {
    std::istringstream iss(line);
    float mx, my;
    if (!(iss >> mx >> my))
      break;
    // 转还至全局坐标
    Eigen::Matrix3d obs_inladar(3, 3);
    obs_inladar <<
      1, 0, mx,
      0, 1, my,
      0, 0, 1;
    Eigen::Matrix3d obs_global(3, 3);
    obs_global = ladar_global * obs_inladar;
    int grid_x, grid_y;
    getgrid(obs_global(0, 2), obs_global(1, 2), &grid_x, &grid_y);
    SLAM_INFO("obs info %d %d", grid_x, grid_y);
    show_ladar.at<cv::Vec3b>(grid_y, grid_x)[0] = 255;
    show_ladar.at<cv::Vec3b>(grid_y, grid_x)[1] = 0;
    show_ladar.at<cv::Vec3b>(grid_y, grid_x)[2] = 0;
  }

  cv::imwrite("./result.png", show_ladar);
}

TEST(check_data, wanji) {
  float resolution = 0.02;
  float min_x = -100;
  float min_y = -100;
  float max_x = 100;
  float max_y = 100;
  int width = ceil((max_x - min_x) / resolution) + 1;
  int height = ceil((max_y - min_y) / resolution) + 1;
  cv::Mat show_ladar = cv::Mat(cv::Size(width, height), CV_8UC3,
                            cv::Scalar(255, 255, 255));
  cv::Mat show_ladar_1 = cv::Mat(cv::Size(width, height), CV_8UC3,
                            cv::Scalar(255, 255, 255));
  cv::Mat show_ladar_2 = cv::Mat(cv::Size(width, height), CV_8UC3,
                            cv::Scalar(255, 255, 255));
  auto getgrid = [&](float xx, float yy, int* grid_x, int* grid_y) {
    *grid_x = ceil((xx - min_x) / resolution);
    *grid_y = ceil((yy - min_y) / resolution);
    return;
  };
  std::vector<Eigen::Vector2d> map_data;
  std::vector<Eigen::Vector2d> obs_data;
  int count = 0;
  char buff[512];
  snprintf(buff, sizeof(buff), \
      "../../../../../Dataset/SLAM_Dataset/wanji_ladar_data/compare.txt");
  std::ifstream infile;
  infile.open(buff, std::ifstream::in);
  if (!infile.is_open()) return;
  SLAM_INFO("file name %s", buff);
  std::string line;
  int current_grid_x, current_grid_y;
  getgrid(0.0, 0.0, &current_grid_x, &current_grid_y);
  float theta = -160 * M_PI / 180.0f;
  float delta_theta = 0.05 * M_PI / 180.0;
  while (std::getline(infile, line)) {
    std::istringstream iss(line);
    float mx, my;
    if (!(iss >> mx >> my))
      break;
    map_data.push_back(Eigen::Vector2d(mx, my));
    count++;
    int grid_x, grid_y;
    getgrid(mx, my, &grid_x, &grid_y);
    LineMath::BresenhamLine line;
    line.SetStartAndEndGrid(grid_x, grid_y, current_grid_x, current_grid_y);
    int next_x, next_y;
    while (line.GetNextGrid(&next_x, &next_y)) {
    if (next_x < 0 || next_x >= width ||
        next_y < 0 || next_y >= height) {
        continue;
      }
      show_ladar.at<cv::Vec3b>(next_y, next_x)[0] = count % 255;
      show_ladar.at<cv::Vec3b>(next_y, next_x)[1] = 0;
      show_ladar.at<cv::Vec3b>(next_y, next_x)[2] = 255;
    }
    // 旋转180
    float mx_1, my_1;
    float t = 160.f * M_PI / 180.f;
    mx_1 = mx * cos(t) - my * sin(t);
    my_1 = mx * sin(t) + my * cos(t);
    // float dis = SLAMMath::Dist(0.0f, 0.0f, mx, my);
    // mx_1 = dis * cos(theta);
    // my_1 = dis * sin(theta);
    // theta += delta_theta;
    getgrid(mx_1, my_1, &grid_x, &grid_y);
    line.SetStartAndEndGrid(grid_x, grid_y, current_grid_x, current_grid_y);
    while (line.GetNextGrid(&next_x, &next_y)) {
    if (next_x < 0 || next_x >= width ||
        next_y < 0 || next_y >= height) {
        continue;
      }
      show_ladar_2.at<cv::Vec3b>(next_y, next_x)[0] = 255;
      show_ladar_2.at<cv::Vec3b>(next_y, next_x)[1] = 0;
      show_ladar_2.at<cv::Vec3b>(next_y, next_x)[2] = count % 255;
    }
  }

  char buff_1[512];
  snprintf(buff_1, sizeof(buff_1), \
      "../../../../../Dataset/SLAM_Dataset/wanji_ladar_data/ladar_data_00.txt");
  std::ifstream infile_1;
  infile_1.open(buff_1, std::ifstream::in);
  if (!infile_1.is_open()) return;
  count = 0;
  while (std::getline(infile_1, line)) {
    std::istringstream iss(line);
    float mx, my, intensitie;
    if (!(iss >> mx >> my >> intensitie))
      break;
    obs_data.push_back(Eigen::Vector2d(mx, my));
    count++;
    int grid_x, grid_y;
    getgrid(mx, my, &grid_x, &grid_y);
    LineMath::BresenhamLine line;
    line.SetStartAndEndGrid(grid_x, grid_y, current_grid_x, current_grid_y);
    int next_x, next_y;
    while (line.GetNextGrid(&next_x, &next_y)) {
    if (next_x < 0 || next_x >= width ||
        next_y < 0 || next_y >= height) {
        continue;
      }
      show_ladar_1.at<cv::Vec3b>(next_y, next_x)[0] = count % 255;
      show_ladar_1.at<cv::Vec3b>(next_y, next_x)[1] = 255;
      show_ladar_1.at<cv::Vec3b>(next_y, next_x)[2] = 0;
    }
  }

  LandMark::TrilateralCal trilater_cal;
  Eigen::Vector3d result;
  trilater_cal.GetGlobalPos(obs_data, map_data, &result);
  SLAM_INFO("result %f %f %f %f",
    result(0), result(1), result(2), result(2) * 180.f / M_PI);


  cv::imwrite("show_ladar.png", show_ladar);
  cv::imwrite("show_ladar_1.png", show_ladar_1);
  cv::imwrite("show_ladar_change.png", show_ladar_2);
}


TEST(compare_static_data, wanji) {
  std::string test_dir = "../../../../../Dataset/SLAM_Dataset/";

  gomros::data_process::mapping_and_location::MappingAndLocationConfig config;
  std::string config_dir =
    test_dir + "static_data/mappingandlocation.json";
  if (!internal_common::ReadMappingAndLocationConfig(config_dir, &config)) {
    SLAM_ERROR("can not open config file, please check json!!");
    return;
  }
  std::string out_file;
  if (config.mapping_config.land_mark_config.recognition_type ==
      gomros::data_process::mapping_and_location::RecognitionType::Cluster) {
    out_file = "cluster";
  } else {
    out_file = "find_peak";
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


  std::string ladar_0 = test_dir + "static_data/ladar_data_00.txt";
  std::string ladar_1 = test_dir + "static_data/ladar_data_01.txt";
  std::string ladar_2 = test_dir + "static_data/ladar_data_02.txt";

  std::ifstream infile;
  infile.open(ladar_0.c_str(), std::ifstream::in);

  gomros::message::RadarSensoryInfo radar_info_0;
  gomros::message::RadarSensoryInfo radar_info_1;
  gomros::message::RadarSensoryInfo radar_info_2;

  std::string line;
  uint64_t time = 0;
  while (std::getline(infile, line)) {
    std::istringstream iss(line);
    if (0 == time) {
      if (!(iss >> time)) break;
      continue;
    }
    float mx, my, intensitie;
    if (!(iss >> mx >> my >> intensitie))
      break;
    radar_info_0.mstruSingleLayerData.mvPoints.push_back(
      Eigen::Vector2d(mx, my));
    radar_info_0.mstruSingleLayerData.mvIntensities.push_back(intensitie);
  }
  infile.close();
  infile.open(ladar_1.c_str(), std::ifstream::in);

  time = 0;
  while (std::getline(infile, line)) {
    std::istringstream iss(line);
    if (0 == time) {
      if (!(iss >> time)) break;
      continue;
    }
    float mx, my, intensitie;
    if (!(iss >> mx >> my >> intensitie))
      break;
    radar_info_1.mstruSingleLayerData.mvPoints.push_back(
      Eigen::Vector2d(mx, my));
    radar_info_1.mstruSingleLayerData.mvIntensities.push_back(intensitie);
  }

  infile.close();
  infile.open(ladar_2.c_str(), std::ifstream::in);

  time = 0;
  while (std::getline(infile, line)) {
    std::istringstream iss(line);
    if (0 == time) {
      if (!(iss >> time)) break;
      continue;
    }
    float mx, my, intensitie;
    if (!(iss >> mx >> my >> intensitie))
      break;
    radar_info_2.mstruSingleLayerData.mvPoints.push_back(
      Eigen::Vector2d(mx, my));
    radar_info_2.mstruSingleLayerData.mvIntensities.push_back(intensitie);
  }

  // 以激光数据1为标准
  DataFusion::Observation obs_0;
  obs_0.GetObsFromLadarData(config.mapping_config.sensor_mount,
                          config.mapping_config.land_mark_config,
                          radar_info_0);
  std::map<int, std::map<int, Eigen::Vector2d>> map_info;
  std::map<int, Eigen::Vector2d> temp_map_info;
  for (int i = 0; i < obs_0.centers.size(); i++) {
    temp_map_info.insert(std::make_pair(i, obs_0.centers[i]));
  }
  map_info.insert(std::make_pair(-1, temp_map_info));

  LandMark::Trilateration global_trilateration(
  config.location_config.land_mark_config);
  global_trilateration.SetLandMarkMap(map_info);

  DataFusion::Observation obs_1;
  obs_1.GetObsFromLadarData(config.mapping_config.sensor_mount,
                          config.mapping_config.land_mark_config,
                          radar_info_1);
  DataFusion::Observation obs_2;
  obs_2.GetObsFromLadarData(config.mapping_config.sensor_mount,
                          config.mapping_config.land_mark_config,
                          radar_info_2);



  bool is_next = true;
  cv::Mat show_ladar_temp = cv::Mat(cv::Size(width, height), CV_8UC3,
                              cv::Scalar(255, 255, 255));
  int current_grid_x, current_grid_y;
  getgrid(0.0, 0.0, &current_grid_x, &current_grid_y);
  for (auto iter : radar_info_0.mstruSingleLayerData.mvPoints) {
    int grid_x, grid_y;
    getgrid(iter(0), iter(1), &grid_x, &grid_y);
    LineMath::BresenhamLine line;
    line.SetStartAndEndGrid(grid_x, grid_y,
    current_grid_x, current_grid_y);
    int next_x, next_y;
    while (line.GetNextGrid(&next_x, &next_y)) {
      if (next_x < 0 || next_x >= width ||
          next_y < 0 || next_y >= height) continue;
      show_ladar_temp.at<cv::Vec3b>(next_y, next_x)[0] = 0;
      show_ladar_temp.at<cv::Vec3b>(next_y, next_x)[1] = 255;
      show_ladar_temp.at<cv::Vec3b>(next_y, next_x)[2] = 0;
    }
  }

  Eigen::Vector3d global_pose;
  if (!global_trilateration.IsGetGlobalPos(obs_1.centers, &global_pose)) {
    SLAM_INFO("global pos fail");
  }

  getgrid(global_pose(0), global_pose(1), &current_grid_x, &current_grid_y);
  cv::Mat show_ladar_1 = show_ladar_temp.clone();
  Coordinate::Transform transform;
  transform.SetPose1InGlobal(global_pose);
  for (auto iter : radar_info_1.mstruSingleLayerData.mvPoints) {
    transform.SetPose2InPose1(Eigen::Vector3d(iter(0), iter(1), 0.0));
    int grid_x, grid_y;
    Eigen::Vector3d pose;
    transform.GetPose2InGlobal(&pose);
    getgrid(pose(0), pose(1), &grid_x, &grid_y);
    LineMath::BresenhamLine line;
    line.SetStartAndEndGrid(grid_x, grid_y,
    current_grid_x, current_grid_y);
    int next_x, next_y;
    while (line.GetNextGrid(&next_x, &next_y)) {
      if (next_x < 0 || next_x >= width ||
          next_y < 0 || next_y >= height) continue;
      show_ladar_1.at<cv::Vec3b>(next_y, next_x)[0] = 192;
      show_ladar_1.at<cv::Vec3b>(next_y, next_x)[1] = 192;
      show_ladar_1.at<cv::Vec3b>(next_y, next_x)[2] = 192;
    }
  }
  cv::imwrite("./show_ladar_1.png", show_ladar_1);


  if (!global_trilateration.IsGetGlobalPos(obs_2.centers, &global_pose)) {
    SLAM_INFO("global pos fail");
  }

  getgrid(global_pose(0), global_pose(1), &current_grid_x, &current_grid_y);
  cv::Mat show_ladar_2 = show_ladar_temp.clone();
  transform.SetPose1InGlobal(global_pose);
  for (auto iter : radar_info_2.mstruSingleLayerData.mvPoints) {
    transform.SetPose2InPose1(Eigen::Vector3d(iter(0), iter(1), 0.0));
    int grid_x, grid_y;
    Eigen::Vector3d pose;
    transform.GetPose2InGlobal(&pose);
    getgrid(pose(0), pose(1), &grid_x, &grid_y);
    LineMath::BresenhamLine line;
    line.SetStartAndEndGrid(grid_x, grid_y,
    current_grid_x, current_grid_y);
    int next_x, next_y;
    while (line.GetNextGrid(&next_x, &next_y)) {
      if (next_x < 0 || next_x >= width ||
          next_y < 0 || next_y >= height) continue;
      show_ladar_2.at<cv::Vec3b>(next_y, next_x)[0] = 192;
      show_ladar_2.at<cv::Vec3b>(next_y, next_x)[1] = 192;
      show_ladar_2.at<cv::Vec3b>(next_y, next_x)[2] = 192;
    }
  }
  cv::imwrite("./show_ladar_2.png", show_ladar_2);
  return;
}
