/*
 * @Description: Copyright (C) 2023 山东亚历山大智能科技有限公司
 * @version: 1.2
 * @Author: renjy
 * @Data: 2023-07-07 08:01:16
 * @LastEditors: renjy
 * @LastEditTime: 2023-08-06 10:39:58
 */

#include <jsoncpp/json/json.h>

#include <iostream>
#include <fstream>
#include "Eigen/Dense"

#include "joint_map/joint_qr_map.h"
#include "include/qr_struct.h"
#include "include/mapping_and_location_math.h"
#include "common/logger.h"

namespace JointMap {

void JointQRMap::SetMajorMapInfo(const std::string& major_map) {
  major_map_ = major_map;
  SLAM_INFO("joint map major_map %s", major_map_.c_str());
}

void JointQRMap::SetSecondMap(const std::string& second_map) {
  second_map_ = second_map;
  SLAM_INFO("joint map second_map %s", second_map_.c_str());
}

bool JointQRMap::JointAndSaveMap(const std::string& final_map) {
  if (!_jointMap()) return false;
  SLAM_INFO("joint map result is %s", final_map.c_str());
  _jsonSave(final_map.c_str(), major_map_json_);
  return true;
}


bool JointQRMap::_jointMap() {
  using namespace gomros::data_process::mapping_and_location;  // NOLINT
  if (!_getMapJson()) return false;
  bool is_find_same_qr = false;

  QRCoordinate major_qr, second_qr;
  std::vector<QRCoordinate> major_qrs, second_qrs;
  int count = 0;
  std::map<int, int> check_tag_nums;
  for (int i = 0; i < second_map_json_["qr_pos_list"].size(); i++) {
    for (int j = 0; j < major_map_json_["qr_pos_list"].size(); j++) {
      if (second_map_json_["qr_pos_list"][i]["tag_num"].asInt() ==
          major_map_json_["qr_pos_list"][j]["tag_num"].asInt()) {
        is_find_same_qr = true;
        major_qr.mx = major_map_json_["qr_pos_list"][j]["x"].asDouble();
        major_qr.my = major_map_json_["qr_pos_list"][j]["y"].asDouble();
        major_qr.theta = major_map_json_["qr_pos_list"][j]["theta"].asDouble();
        major_qr.tag_num = major_map_json_["qr_pos_list"][j]["tag_num"].asInt();
        major_qrs.push_back(major_qr);
        check_tag_nums.insert(std::make_pair(
          major_qr.tag_num, major_qr.tag_num));
        second_qr.mx = second_map_json_["qr_pos_list"][i]["x"].asDouble();
        second_qr.my = second_map_json_["qr_pos_list"][i]["y"].asDouble();
        second_qr.theta =
          second_map_json_["qr_pos_list"][i]["theta"].asDouble();
        second_qr.tag_num =
          second_map_json_["qr_pos_list"][i]["tag_num"].asInt();
        second_qrs.push_back(second_qr);
        count++;
        break;
      }
    }
  }
  if (!is_find_same_qr) {
    SLAM_ERROR(" thete is no same qr id, please check map info");
    return false;
  }
  Eigen::Matrix3d coordinate(3, 3);
  Eigen::Matrix3d second_qr_mat(3, 3);
  float offset_x, offset_y, offset_theta;
  offset_x = 0;
  offset_y = 0;
  offset_theta = 0;
  for (int i = 0; i < second_qrs.size(); i++) {
    second_qr_mat <<
      cos(second_qrs[i].theta), -sin(second_qrs[i].theta), second_qrs[i].mx,
      sin(second_qrs[i].theta), cos(second_qrs[i].theta), second_qrs[i].my,
      0, 0, 1;
    Eigen::Matrix3d major_qr_mat(3, 3);
    major_qr_mat <<
      cos(major_qrs[i].theta), -sin(major_qrs[i].theta), major_qrs[i].mx,
      sin(major_qrs[i].theta), cos(major_qrs[i].theta), major_qrs[i].my,
      0, 0, 1;
    coordinate = major_qr_mat * second_qr_mat.inverse();
    offset_x += coordinate(0, 2);
    offset_y += coordinate(1, 2);
    offset_theta += atan2(coordinate(1, 0), coordinate(0, 0));
    // offset_theta = SLAMMath::NormalizePITheta(offset_theta);
  }

  float size = 1.0f * second_qrs.size();
  offset_x = offset_x / size;
  offset_y = offset_y / size;
  offset_theta = offset_theta / size;
  offset_theta = SLAMMath::NormalizePITheta(offset_theta);


  coordinate <<
    cos(offset_theta), -sin(offset_theta), offset_x,
    sin(offset_theta), cos(offset_theta), offset_y,
    0, 0, 1;

  Json::Value qr_point;
  int qr_count = 0;
  for (int j = 0; j < major_map_json_["qr_pos_list"].size(); j++) {
    Json::Value qr_info;
    qr_info["x"] = major_map_json_["qr_pos_list"][j]["x"].asDouble();
    qr_info["y"] = major_map_json_["qr_pos_list"][j]["y"].asDouble();
    // qr_info["theta"] = qr_thetas_ave[iter->tag_num];
    qr_info["theta"] = major_map_json_["qr_pos_list"][j]["theta"].asDouble();
    qr_info["tag_num"] =
      major_map_json_["qr_pos_list"][j]["tag_num"].asDouble();
    qr_point[qr_count] = qr_info;
    qr_count++;
  }
  major_map_json_["qr_pos_list"].clear();
  for (int i = 0; i < second_map_json_["qr_pos_list"].size(); i++) {
    if (check_tag_nums.count(
      second_map_json_["qr_pos_list"][i]["tag_num"].asInt())) continue;
    // if (second_map_json_["qr_pos_list"][i]["tag_num"].asInt() ==
    //     major_qr.tag_num) continue;
    double mx = second_map_json_["qr_pos_list"][i]["x"].asDouble();
    double my = second_map_json_["qr_pos_list"][i]["y"].asDouble();
    double theta = second_map_json_["qr_pos_list"][i]["theta"].asDouble();
    Eigen::Matrix3d second_qr_temp(3, 3);
    second_qr_temp <<
      cos(theta), -sin(theta), mx,
      sin(theta), cos(theta), my,
      0, 0, 1;
    Eigen::Matrix3d major_qr_temp(3, 3);
    major_qr_temp = coordinate * second_qr_temp;

    Json::Value qr_info;
    qr_info["x"] = major_qr_temp(0, 2);
    qr_info["y"] = major_qr_temp(1, 2);
    // qr_info["theta"] = qr_thetas_ave[iter->tag_num];
    qr_info["theta"] = SLAMMath::NormalizePITheta(
                      atan2(major_qr_temp(1, 0), major_qr_temp(0, 0)));

    qr_info["tag_num"] =
      second_map_json_["qr_pos_list"][i]["tag_num"].asDouble();
    qr_point[qr_count] = qr_info;
    qr_count++;
  }

  major_map_json_["qr_pos_list"] = qr_point;
  return true;
}



bool JointQRMap::_getMapJson() {
  std::string major_char_string;
  std::ifstream major_infile(major_map_);
  if (!major_infile.is_open()) {
     SLAM_ERROR("open major_map_ %s failed", major_map_.c_str());
      return false;
  }
  std::getline(major_infile, major_char_string);
  const char* major_map_to_parse = major_char_string.c_str();
  Json::Reader major_reader;

  if (!major_reader.parse(major_map_to_parse, major_map_json_)) {
    SLAM_ERROR("load major slam grid map error!!");
    return false;
  }

  std::string second_char_string;
  std::ifstream second_infile(second_map_);
  if (!major_infile.is_open()) {
      SLAM_ERROR("open second_map_ %s failed", second_map_.c_str());
      return false;
  }
  std::getline(second_infile, second_char_string);
  const char* second_map_to_parse = second_char_string.c_str();
  Json::Reader second_reader;

  if (!second_reader.parse(second_map_to_parse, second_map_json_)) {
    SLAM_ERROR("load second slam grid map error!!");
    return false;
  }
  return true;
}

void JointQRMap::_jsonSave(const char* file_name, Json::Value v) {
  Json::StyledWriter style_writer;
  std::string str = style_writer.write(v);
  str.erase(std::remove(str.begin(), str.end(), '\n'), str.end());
  str.erase(std::remove(str.begin(), str.end(), '\r'), str.end());
  str.erase(std::remove(str.begin(), str.end(), ' '), str.end());
  std::ofstream ofs(file_name, std::ios::trunc);
  ofs << str;
  ofs.close();
}



}  // namespace JointMap
















