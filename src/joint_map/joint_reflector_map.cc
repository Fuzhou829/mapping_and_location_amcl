/*
 * @Description: Copyright (C) 2023 山东亚历山大智能科技有限公司
 * @version: 1.2
 * @Author: renjy
 * @Data: 2023-08-06 15:56:33
 * @LastEditors: renjy
 * @LastEditTime: 2023-08-22 17:04:06
 */

#include <float.h>
#include "joint_map/joint_reflector_map.h"
#include "common/logger.h"

namespace JointMap {

void JointReflectorMap::SetMajorMapInfo(const std::string& major_map) {
  SLAM_INFO("major_map %s", major_map.c_str());
  std::vector<Eigen::Vector2d> reflector = _getreflectorFromMap(major_map);
  if (reflector.size() <= 0) {
    SLAM_ERROR("there is no reflector in map, please check %s",
                major_map.c_str());
    return;
  }
  int map_id = candidate_map_.size();
  candidate_map_.insert(std::make_pair(map_id, reflector));
}

void JointReflectorMap::SetSecondMap(const std::string& second_map) {
  std::vector<Eigen::Vector2d> reflector = _getreflectorFromMap(second_map);
  SLAM_INFO("second_map %s", second_map.c_str());

  if (reflector.size() <= 0) {
    SLAM_ERROR("there is no reflector in map, please check %s",
                second_map.c_str());
    return;
  }
  int map_id = candidate_map_.size();
  candidate_map_.insert(std::make_pair(map_id, reflector));
}

bool JointReflectorMap::JointAndSaveMap(const std::string& final_map) {
  using namespace gomros::data_process::mapping_and_location;  // NOLINT
  if (candidate_map_.size() <= 1) {
    SLAM_ERROR("there is not enough info to major!!!");
    return false;
  }

  MergeReflectors merge_reflector_map(mapping_config_);
  for (auto iter : candidate_map_) {
    merge_reflector_map.SetReflectorMapInfo(iter.second);
  }
  std::map<int, std::map<int, Eigen::Vector2d>> result;
  if (!merge_reflector_map.GetMapResult(&result)) return false;
  float min_x = FLT_MAX;
  float min_y = FLT_MAX;
  float max_x = -FLT_MAX;
  float max_y = -FLT_MAX;

  Json::Value map_json;
  Json::Value map_header;
  Json::Value atring_add;
  map_header["mapType"] = "Joint-Map";
  map_header["mapName"] = final_map;

  map_header["resolution"] = mapping_config_.mapping_resolution;
  map_header["version"] = "1.2";

  internal_common::SaveLandmarkInfoToJson(result,
    mapping_config_.land_mark_config.landmark_radius,
    &map_json, &min_x, &min_y, &max_x, &max_y);

  atring_add["x"] = min_x - 1.0;
  atring_add["y"] = min_y - 1.0;
  map_header["minPos"] = atring_add;
  atring_add["x"] = max_x + 1.0;
  atring_add["y"] = max_y + 1.0;
  map_header["maxPos"] = atring_add;
  map_json["header"] = map_header;

  SLAM_INFO("save reflector map to %s", final_map.c_str());
  internal_common::Save(final_map.c_str(), map_json);

  return true;
}


std::vector<Eigen::Vector2d> JointReflectorMap::_getreflectorFromMap(
  const std::string& map_name) {
  std::string map_data;
  std::vector<Eigen::Vector2d> reflectors;

  if (!internal_common::Read(map_name.c_str(), &map_data)) {
    SLAM_ERROR("read map file failed....");
    return reflectors;
  }
  Json::Reader reader;
  Json::Value map_json;
  if (!reader.parse(map_data, map_json)) {
    SLAM_ERROR("parse map failed.....");
    return reflectors;
  }
  // TODO(r) 后续需要全部更改一下
  std::map<int, std::map<int, Eigen::Vector2d>> result;
  internal_common::ReadLandmarkInfoFromJson(map_json, &result);
  if (result.empty()) {
    SLAM_ERROR("reflector map empty!");
    return reflectors;
  } else if (!result.count(-1)) {
    SLAM_ERROR("there is no global map!");
    return reflectors;
  }
  for (auto iter : result.at(-1)) {
    reflectors.push_back(iter.second);
  }
  return reflectors;
}

}  // namespace JointMap

