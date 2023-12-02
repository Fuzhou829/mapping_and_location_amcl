/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.0
 * @Date: 2022-08-06 11:11:30
 * @LastEditTime: 2023-09-29 12:23:44
 * @Author: lcfc-desktop
 */

#include "karto_mapping/slam_kar.h"

#include <fstream>
#include <iostream>
#include <string>

#include "common/logger.h"

namespace gomros {
namespace data_process {
namespace mapping_and_location {

SlamKarto::SlamKarto(const MappingConfig& config)
  : laser_count_(0), marker_count_(0) {
  SetConfiguration(config);
  map_successed = false;
  mapper_ = new karto::Mapper();
  dataset_ = new karto::Dataset();

  // Setting General Parameters from the Parameter Server
  bool use_scan_matching = true;
  mapper_->setParamUseScanMatching(use_scan_matching);
  mapper_->setParamUselandmarkConstraints(false);
  bool use_scan_barycenter = true;
  mapper_->setParamUseScanBarycenter(use_scan_barycenter);
  double minimum_time_interval = 3600;
  mapper_->setParamMinimumTimeInterval(minimum_time_interval);

  double minimum_travel_distance = 0.2;
  mapper_->setParamMinimumTravelDistance(minimum_travel_distance);
  double minimum_travel_heading = 0.5;
  mapper_->setParamMinimumTravelHeading(minimum_travel_heading);

  int scan_buffer_size = 100;
  mapper_->setParamScanBufferSize(scan_buffer_size);

  double scan_buffer_maximum_scan_distance = 20.0;
  mapper_->setParamScanBufferMaximumScanDistance(
      scan_buffer_maximum_scan_distance);

  double link_match_minimum_response_fine = 0.8;
  mapper_->setParamLinkMatchMinimumResponseFine(
      link_match_minimum_response_fine);

  double link_scan_maximum_distance = 10.0;
  mapper_->setParamLinkScanMaximumDistance(link_scan_maximum_distance);

  double loop_search_maximum_distance = 7.0;
  mapper_->setParamLoopSearchMaximumDistance(loop_search_maximum_distance);

  bool do_loop_closing = true;
  mapper_->setParamDoLoopClosing(do_loop_closing);

  int loop_match_minimum_chain_size = 10;
  mapper_->setParamLoopMatchMinimumChainSize(loop_match_minimum_chain_size);

  double loop_match_maximum_variance_coarse = 0.9;
  mapper_->setParamLoopMatchMaximumVarianceCoarse(
      loop_match_maximum_variance_coarse);

  double loop_match_minimum_response_coarse = 0.5;
  mapper_->setParamLoopMatchMinimumResponseCoarse(
      loop_match_minimum_response_coarse);

  double loop_match_minimum_response_fine = 0.6;
  mapper_->setParamLoopMatchMinimumResponseFine(
      loop_match_minimum_response_fine);

  double correlation_search_space_dimension = 0.3;
  mapper_->setParamCorrelationSearchSpaceDimension(
      correlation_search_space_dimension);

  double correlation_search_space_resolution = 0.01;
  mapper_->setParamCorrelationSearchSpaceResolution(
      correlation_search_space_resolution);

  double correlation_search_space_smear_deviation = 0.03;
  mapper_->setParamCorrelationSearchSpaceSmearDeviation(
      correlation_search_space_smear_deviation);

  // Setting Correlation Parameters, Loop Closure Parameters from the Parameter
  // Server

  double loop_search_space_dimension = 9.0;
  mapper_->setParamLoopSearchSpaceDimension(loop_search_space_dimension);

  double loop_search_space_resolution = 0.05;
  mapper_->setParamLoopSearchSpaceResolution(loop_search_space_resolution);

  double loop_search_space_smear_deviation = 0.03;
  mapper_->setParamLoopSearchSpaceSmearDeviation(
      loop_search_space_smear_deviation);

  // Setting Scan Matcher Parameters from the Parameter Server
  double distance_variance_penalty = 0.3;
  mapper_->setParamDistanceVariancePenalty(distance_variance_penalty);

  double angle_variance_penalty = 0.174;
  mapper_->setParamAngleVariancePenalty(angle_variance_penalty);

  double fine_search_angle_offset = 0.00174;
  mapper_->setParamFineSearchAngleOffset(fine_search_angle_offset);

  double coarse_search_angle_offset = 0.174;
  mapper_->setParamCoarseSearchAngleOffset(coarse_search_angle_offset);

  double coarse_angle_resolution = 0.0174;
  mapper_->setParamCoarseAngleResolution(coarse_angle_resolution);

  double minimum_angle_penalty = 0.9;
  mapper_->setParamMinimumAnglePenalty(minimum_angle_penalty);

  double minimum_distance_penalty = 0.5;
  mapper_->setParamMinimumDistancePenalty(minimum_distance_penalty);

  bool use_response_expansion = false;
  mapper_->setParamUseResponseExpansion(use_response_expansion);

  // Set solver to be used in loop closure
  solver_ = new SpaSolver();
  mapper_->SetScanSolver(solver_);
  // getLaser();
}

SlamKarto::~SlamKarto() {
  if (solver_) delete solver_;
  if (mapper_) delete mapper_;
  if (dataset_) delete dataset_;
  // TODO: delete the pointers in the lasers_ map; not sure whether or not
  // I'm supposed to do that.
}

void SlamKarto::SetConfiguration(const MappingConfig& config) {
  m_MappingConfig = config;
}

std::vector<std::string> SlamKarto::SplitCString(std::string& str,
                                                 std::string delimit) {
  std::vector<std::string> result;
  size_t pos = str.find(delimit);
  str += delimit;  //将分隔符加入到最后一个位置，方便分割最后一位
  while (pos != std::string::npos) {
    result.push_back(str.substr(0, pos));
    str = str.substr(
        pos +
        1);  // substr的第一个参数为起始位置，第二个参数为复制长度，默认为string::npos到最后一个位置
    pos = str.find(delimit);
  }
  return result;
}
karto::LaserRangeFinder* SlamKarto::getLaser() {
  // Check whether we know about this laser yet

  // Create a laser range finder device and copy in data from the first
  // scan
  // karto::Name name("laser0");
  std::string name("laser0");
  karto::LaserRangeFinder* laser =
      karto::LaserRangeFinder::CreateLaserRangeFinder(
          karto::LaserRangeFinder_Custom, karto::Name(name));
  laser->SetOffsetPose(
      karto::Pose2(laser_x_offset, laser_y_offset, laser_th_offset));
  laser->SetMinimumRange(0.1);
  laser->SetMaximumRange(19);
  laser->SetMinimumAngle(karto::math::DegreesToRadians(-95));
  laser->SetMaximumAngle(karto::math::DegreesToRadians(95));
  laser->SetAngularResolution(karto::math::DegreesToRadians(0.5));
  // TODO: expose this, and many other parameters
  // laser_->SetRangeThreshold(12.0);

  // Store this laser device for later
  lasers_[name] = laser;

  // Add it to the dataset, which seems to be necessary
  dataset_->Add(laser);

  return lasers_[name];
}

void SlamKarto::begin_slam(std::vector<char>* output, std::string map_name) {
  /// <summary>
  ///初始化 laser
  resolution_ = m_MappingConfig.mapping_resolution;
  if (resolution_ < 0 || resolution_ > 1) resolution_ = 0.05;
  laser_x_offset = m_MappingConfig.sensor_mount.radar_position_x;
  laser_y_offset = m_MappingConfig.sensor_mount.radar_position_y;
  laser_th_offset = m_MappingConfig.sensor_mount.radar_position_theta;
  float laser_min_range = m_MappingConfig.mapping_laser_min_range;
  if (laser_min_range < 0) laser_min_range = 0.5;
  float laser_max_range = m_MappingConfig.mapping_laser_max_range;
  if (laser_max_range < 0) laser_max_range = 20;
  float laser_min_angle = m_MappingConfig.mapping_start_angle;
  float laser_max_angle = m_MappingConfig.mapping_end_angle;
  float laser_resolution = m_MappingConfig.laser_resolution;

  std::string name("laser0");
  karto::LaserRangeFinder* laser =
      karto::LaserRangeFinder::CreateLaserRangeFinder(
          karto::LaserRangeFinder_Custom, karto::Name(name));
  laser->SetOffsetPose(
      karto::Pose2(laser_x_offset, laser_y_offset, laser_th_offset));
  // SLAM_INFO("###slam karto modual laser_x=%f,laser_y=%f,laser_th=%f\n",
        //  laser_x_offset, laser_y_offset, laser_th_offset);
  laser->SetMinimumRange(laser_min_range);
  laser->SetMaximumRange(laser_max_range);
  laser->SetMinimumAngle(karto::math::DegreesToRadians(laser_min_angle));
  laser->SetMaximumAngle(karto::math::DegreesToRadians(laser_max_angle));
  laser->SetAngularResolution(karto::math::DegreesToRadians(laser_resolution));

  // Add it to the dataset, which seems to be necessary
  dataset_->Add(laser);
  /// </summary>

  karto::Pose2 odom_pose;
  map_successed = false;
  usleep(10000);
  std::vector<double> laserScan;
  double odom_x = 0.0;
  double odom_y = 0.0;
  double odom_th = 0.0;
  bool got_first_scan = false;

  int processedScancount = 0;
  std::string record_str;
  std::string raw_map_data_path =
      m_MappingConfig.map_data_file_path + "/map.rawmap";
  std::ifstream rawMapFileName(
      raw_map_data_path.c_str());  ////未处理地图数据文件yxh
  usleep(100000);
  assert(rawMapFileName);
  while (getline(rawMapFileName, record_str)) {
    laserScan.clear();
    std::vector<std::string> odom_str = SplitCString(record_str, "\t");
    odom_x = atof(odom_str[3].c_str());
    odom_y = atof(odom_str[4].c_str());
    odom_th = atof(odom_str[5].c_str());

    std::vector<std::string> laser_str = SplitCString(odom_str[7], ",");
    for (int i = 0; i < laser_str.size() - 1; i++) {
      laserScan.push_back(atof(laser_str[i].c_str()));
    }

    karto::Pose2 odom_poset(
        odom_x + laser_x_offset * cos(odom_th) - laser_y_offset * sin(odom_th),
        odom_y + laser_x_offset * sin(odom_th) + laser_y_offset * cos(odom_th),
        odom_th);
    if (addScan(laser, laserScan, odom_poset)) {
      processedScancount++;
    }
    usleep(50000);
  }
  updateMap(output, map_name);
  rawMapFileName.close();
}

bool SlamKarto::updateMap(std::vector<char>* output,
                          std::string given_map_name) {
  karto::OccupancyGrid* occ_grid = karto::OccupancyGrid::CreateFromScans(
      mapper_->GetAllProcessedScans(), resolution_);
  if (!occ_grid) return false;
  // Translate to ROS format
  kt_int32s width = occ_grid->GetWidth();
  kt_int32s height = occ_grid->GetHeight();
  karto::Vector2<kt_double> offset =
      occ_grid->GetCoordinateConverter()->GetOffset();
  MapInfo info;
  info.miMapHeight = height;
  info.miMapWidth = width;
  info.mdOriginXInWorld = offset.GetX();
  info.mdOriginYInWorld = offset.GetY();
  info.mdResolution = resolution_;
  given_map_name.erase(
      std::remove(given_map_name.begin(), given_map_name.end(), ' '),
      given_map_name.end());
  time_t now_time = time(NULL);
  tm* time = gmtime(&now_time);
  std::string create_time =
      std::to_string(time->tm_year + 1900) + "-" +
      std::to_string(time->tm_mon + 1) + "-" + std::to_string(time->tm_mday) +
      "-" + std::to_string(time->tm_hour + 8) + "-" +
      std::to_string(time->tm_min) + "-" + std::to_string(time->tm_sec);
  std::string map_name;
  if (given_map_name.empty())
    map_name = create_time + ".smap";
  else
    map_name = given_map_name + ".smap";
  // SLAM_INFO("save karto mapping name %s\n", map_name.c_str());

  std::cout << "width = " << width << ", height = " << height
            << ", scale = " << occ_grid->GetCoordinateConverter()->GetScale()
            << ", offset: " << offset.GetX() << ", " << offset.GetY()
            << std::endl;
  SimpleGridMap* grid_map = new SimpleGridMap(info);
  grid_map->datas.reserve(info.miMapWidth * info.miMapHeight);
  Json::Value map_json;
  Json::Value map_header;
  Json::Value point_append;
  Json::Value atring_add;
  map_header["mapType"] = "2DGrid-Map";
  map_header["mapName"] = map_name;
  atring_add["x"] = info.mdOriginXInWorld;
  atring_add["y"] = info.mdOriginYInWorld;
  map_header["minPos"] = atring_add;
  atring_add["x"] = info.mdOriginXInWorld + resolution_ * width;
  atring_add["y"] = info.mdOriginYInWorld + resolution_ * height;
  map_header["maxPos"] = atring_add;
  map_header["resolution"] = resolution_;
  map_header["version"] = "1.2";
  map_json["header"] = map_header;
  int index_cnt = 0;
  for (kt_int32s y = 0; y < height; y++) {
    for (kt_int32s x = 0; x < width; x++) {
      Json::Value point_xy;
      // Getting the value at position x,y
      kt_int8u value = occ_grid->GetValue(karto::Vector2<kt_int32s>(x, y));
      switch (value) {
        case karto::GridStates_Unknown:
          grid_map->datas.push_back(255);
          break;
        case karto::GridStates_Occupied:
          grid_map->datas.push_back(100);
          point_xy["x"] = info.mdOriginXInWorld + x * resolution_;
          point_xy["y"] = info.mdOriginYInWorld + y * resolution_;
          point_append[index_cnt] = point_xy;
          index_cnt++;
          break;
        case karto::GridStates_Free:
          grid_map->datas.push_back(0);
          break;
        default:
          // SLAM_INFO("Encountered unknown cell value at %d, %d", x, y);
          break;
      }
    }
  }
  map_json["normalPosList"] = point_append;
  grid_map->to_json_char_array(output);
  std::string full_map_data_path =
      m_MappingConfig.map_data_file_path + "/" + map_name;
  json_save(full_map_data_path.c_str(), map_json);
  std::string data_string;
  std::string full_map_config_path =
      m_MappingConfig.map_config_file_path + "/map_config.json";
  json_read(full_map_config_path.c_str(), &data_string);
  Json::Value data_json;
  Json::Reader reader;
  if (reader.parse(data_string, data_json)) {
    Json::Value data_json_copy = data_json;
    Json::Value json_map_info;
    json_map_info["width"] = width;
    json_map_info["height"] = height;
    json_map_info["resolution"] = resolution_;
    struct stat statbuf;
    stat(full_map_data_path.c_str(), &statbuf);
    int size = statbuf.st_size;
    json_map_info["size"] = size;
    json_map_info["createTime"] = create_time;
    json_map_info["mapName"] = map_name;
    json_map_info["isUseful"] = false;
    data_json_copy["AllMapList"].append(json_map_info);
    data_json["AllMapList"] = data_json_copy["AllMapList"];
    json_save(full_map_config_path.c_str(), data_json);
  }
  delete grid_map;
  delete occ_grid;
  usleep(200000);
  map_successed = true;
  return true;
}
void SlamKarto::json_save(const char* file_name, Json::Value v) {
  Json::StyledWriter style_writer;
  std::string str = style_writer.write(v);
  str.erase(std::remove(str.begin(), str.end(), '\n'), str.end());
  str.erase(std::remove(str.begin(), str.end(), '\r'), str.end());
  str.erase(std::remove(str.begin(), str.end(), ' '), str.end());
  std::ofstream ofs(file_name, std::ios::trunc);
  ofs << str;
  ofs.close();
}

void SlamKarto::json_read(const char* file_name, std::string* read_data) {
  std::ifstream read_file;
  std::ostringstream oss;
  read_file.open(file_name, std::ios::binary);
  if (read_file.is_open()) {
    oss.str("");
    oss << read_file.rdbuf();
    *read_data = oss.str();
    read_file.close();
  } else {
  }
}

bool SlamKarto::addScan(karto::LaserRangeFinder* laser,
                        std::vector<double> scan, karto::Pose2& karto_pose) {
  // Create a vector of doubles for karto
  std::vector<kt_double> readings;
  std::vector<kt_double> intensities;
  for (unsigned int i = 0; i < scan.size(); i++) {
    readings.push_back((double)scan[i]);
    intensities.push_back(0.0);
  }


  // create localized range scan
  karto::LocalizedRangeScan* range_scan =
      new karto::LocalizedRangeScan(laser->GetName(), readings, intensities);
  range_scan->SetIntensitiesReadings(intensities);
  range_scan->SetOdometricPose(karto_pose);
  range_scan->SetCorrectedPose(karto_pose);

  // SLAM_DEBUG("get odom pos %f %f %f\n", karto_pose.GetX(),
            // karto_pose.GetY(), karto_pose.GetHeading());
  // Add the localized range scan to the mapper
  bool processed;
  if ((processed = mapper_->Process(range_scan))) {
    // std::cout << "Pose: " << range_scan->GetOdometricPose() << " Corrected
    // Pose: " << range_scan->GetCorrectedPose() << std::endl;
    karto::Pose2 corrected_pose = range_scan->GetCorrectedPose();
    // SLAM_DEBUG("karto scan match = %f %f %f\n", corrected_pose.GetX(),
              //  corrected_pose.GetY(), corrected_pose.GetHeading());
    // Add the localized range scan to the dataset (for memory management)
    dataset_->Add(range_scan);
  } else
    delete range_scan;

  return processed;
}

}  // namespace mapping_and_location
}  // namespace data_process
}  // namespace gomros
