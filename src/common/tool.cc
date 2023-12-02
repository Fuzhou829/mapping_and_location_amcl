/*
 * @Descripttion: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @version: 1.0
 * @Author: renjy
 * @Date: 2023-03-17 07:56:31
 * @LastEditors: renjy
 * @LastEditTime: 2023-09-07 14:38:32
 */

#include "common/tool.h"
#include "common/logger.h"


namespace internal_common {

void Save(const char *file_name, Json::Value v) {
  Json::StyledWriter style_writer;
  std::string str = style_writer.write(v);
  str.erase(std::remove(str.begin(), str.end(), '\n'), str.end());
  str.erase(std::remove(str.begin(), str.end(), '\r'), str.end());
  str.erase(std::remove(str.begin(), str.end(), ' '), str.end());
  std::ofstream ofs(file_name, std::ios::trunc);
  ofs << str;
  ofs.close();
}


bool Read(const char* file_name, std::string* read_data) {
  std::ifstream read_file;
  std::ostringstream oss;
  read_file.open(file_name, std::ios::binary);
  if (read_file.is_open()) {
    oss.str("");
    oss << read_file.rdbuf();
    *read_data = oss.str();
    read_file.close();
    return true;
  } else {
    return false;
  }
}

std::vector<std::string> SplitCString(const std::string& line,
  const std::string& delimit) {
  std::vector<std::string> result;
  std::string str = line;
  size_t pos = str.find(delimit);
  // 将分隔符加入到最后一个位置，方便分割最后一位
  str += delimit;
  while (pos != std::string::npos) {
    result.push_back(str.substr(0, pos));
    /* 
    * substr的第一个参数为起始位置，第二个参数为复制长度，
    * 默认为string::npos到最后一个位置
    */
    str = str.substr(pos + 1);
    pos = str.find(delimit);
  }
  return result;
}

bool GetOdomMessage(const std::string& odom_line,
  const gomros::data_process::mapping_and_location::OdomType& odom_type,
  gomros::message::OdometerMessage* message) {
  using namespace gomros::data_process::mapping_and_location;  // NOLINT
  std::vector<std::string> odom_data = SplitCString(odom_line, " ");
  return GetOdomMessage(odom_data, odom_type, message);
}

bool GetOdomMessage(const std::vector<std::string>& odom_data,
  const gomros::data_process::mapping_and_location::OdomType& odom_type,
  gomros::message::OdometerMessage* message) {
  using namespace gomros::data_process::mapping_and_location;  // NOLINT
  uint64_t odom_time_stemp = atof(odom_data[0].c_str());
  message->mlDeltaTime = 1;  // 不影响定位
  message->mclDeltaPosition.mlTimestamp = odom_time_stemp;
  if (odom_type == OdomType::DiffWheelModel) {
    if (odom_data.size() != 3) return false;
    message->mstruDiffSteerSts.mfLeftLinearVel = atof(odom_data[1].c_str());
    message->mstruDiffSteerSts.mfRightLinearVel = atof(odom_data[2].c_str());
  } else if (odom_type == OdomType::SingleSteerModel) {
    if (odom_data.size() != 4) return false;
    message->mstruSingleSteerSts.mdLinearVel = atof(odom_data[1].c_str());
    message->mstruSingleSteerSts.mdAngularVel = atof(odom_data[2].c_str());
    message->mstruSingleSteerSts.mdRotAngle = atof(odom_data[3].c_str());
  } else if (odom_type == OdomType::DoubleSteerModel) {
    if (odom_data.size() != 7) return false;
    message->mstruDualSteerSts.mdFrontLinearVel = atof(odom_data[1].c_str());
    message->mstruDualSteerSts.mdBackLinearVel = atof(odom_data[2].c_str());
    message->mstruDualSteerSts.mdFrontAngularVel = atof(odom_data[3].c_str());
    message->mstruDualSteerSts.mdBackAngularVel = atof(odom_data[4].c_str());
    message->mstruDualSteerSts.mdFrontRotAngle = atof(odom_data[5].c_str());
    message->mstruDualSteerSts.mdBackRotAngle = atof(odom_data[6].c_str());
  }
  return true;
}


std::string CreatedMapFileName(const std::string& map_name) {
  std::string temp_name = map_name;
  temp_name.erase(std::remove(temp_name.begin(), temp_name.end(), ' '),
                  temp_name.end());
  time_t now_time = time(NULL);
  tm* time = gmtime(&now_time);  // NOLINT
  std::string create_time =
      std::to_string(time->tm_year + 1900) + "-" +
      std::to_string(time->tm_mon + 1) + "-" + std::to_string(time->tm_mday) +
      "-" + std::to_string(time->tm_hour + 8) + "-" +
      std::to_string(time->tm_min) + "-" + std::to_string(time->tm_sec);
  std::string save_map_name;
  if (temp_name.empty())
    save_map_name = create_time + ".smap";
  else
    save_map_name = temp_name + ".smap";
  return save_map_name;
}


void ReadLandmarkInfoFromFile(const std::string& map_file,
  std::map<int, std::map<int, Eigen::Vector2d>>* result) {
  char filename[128];
  SLAM_DEBUG("load map dir is %s", map_file.c_str());
  std::string map_data;
  if (!internal_common::Read(map_file.c_str(), &map_data)) {
    SLAM_ERROR("read map file failed....");
    return;
  }
  Json::Reader reader;
  Json::Value map_json;
  if (!reader.parse(map_data, map_json)) {
    SLAM_ERROR("parse map failed.....");
    return;
  }
  ReadLandmarkInfoFromJson(map_json, result);
  return;
}


void ReadLandmarkInfoFromJson(const Json::Value &map_json,
  std::map<int, std::map<int, Eigen::Vector2d>>* result) {
  for (int i = 0; i < map_json["submap_info"].size(); i++) {
    int map_id = atoi(map_json["submap_info"][i]["map_id"].asString().c_str());
    int pos_info_size = map_json["submap_info"][i]["pos_info"].size();
    std::map<int, Eigen::Vector2d> sub_map;
    for (int j = 0; j < pos_info_size; j++) {
      int id =
        map_json["submap_info"][i]["pos_info"][j]["landmark_id"].asInt();
      if ((*result).count(-1) && !(*result).at(-1).count(id)) {
        SLAM_WARN("landmarkid %d in map_id %d does not exist", id, map_id);
        continue;
      }
      Eigen::Vector2d pos_info(
        map_json["submap_info"][i]["pos_info"][j]["x"].asDouble(),
        map_json["submap_info"][i]["pos_info"][j]["y"].asDouble());
      SLAM_INFO("submap_info id %d, golbal_id %d, pos(%f %f)",
                map_id, id, pos_info(0), pos_info(1));
      sub_map.insert(std::make_pair(id, pos_info));
    }
    result->insert(std::make_pair(map_id, sub_map));
  }
  if (result->empty()) {
    std::map<int, Eigen::Vector2d> sub_map;
    int i = 0;
    for (i = 0; i < map_json["landmarks_pos_list"].size(); i++) {
      sub_map.insert(std::make_pair(
        i,
        Eigen::Vector2d(map_json["landmarks_pos_list"][i]["x"].asDouble(),
                        map_json["landmarks_pos_list"][i]["y"].asDouble())));
    }
    result->insert(std::make_pair(-1, sub_map));
    SLAM_WARN("反光柱数量为%d   %d\n", i, sub_map.size());
  }
  return;
}


void SaveLandmarkInfoToJson(
  const std::map<int, std::map<int, Eigen::Vector2d>>& reflector_info,
  float radius, Json::Value* map_json,
  float* min_x, float* min_y, float* max_x, float* max_y) {
  int id = 0;
  Json::Value landmark_pos_list, submap_info;
  for (auto map_iter : reflector_info) {
    Json::Value maps_info, pos_info;
    int i = 0;
    for (auto reflector_iter : map_iter.second) {
      if (reflector_iter.second(0) < *min_x) *min_x = reflector_iter.second(0);
      if (reflector_iter.second(1) < *min_y) *min_y = reflector_iter.second(1);
      if (reflector_iter.second(0) > *max_x) *max_x = reflector_iter.second(0);
      if (reflector_iter.second(1) > *max_y) *max_y = reflector_iter.second(1);
      Json::Value landmark_info;
      landmark_info["x"] = reflector_iter.second(0);
      landmark_info["y"] = reflector_iter.second(1);
      landmark_info["radius"] = radius;
      landmark_info["landmark_id"] = reflector_iter.first;
      pos_info[i++] = landmark_info;
    }
    maps_info["map_id"] = map_iter.first;
    maps_info["pos_info"] = pos_info;
    if (map_iter.first == -1) {
      landmark_pos_list = pos_info;
    }
    submap_info[id++] = maps_info;
  }

  (*map_json)["landmarks_pos_list"] = landmark_pos_list;
  (*map_json)["submap_info"] = submap_info;

  if ((*map_json)["landmarks_pos_list"].size() <= 0) {
    *min_x = *min_y = *max_x = *max_y = 0;
  }

  SLAM_INFO("reflector box %f %f %f %f", *min_x, *min_y, *max_x, *max_y);
  return;
}




}  // namespace internal_common
