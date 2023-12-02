/*
 * @Descripttion: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @version: 1.2
 * @Author: renjy
 * @Date: 2023-03-17 07:56:31
 * @LastEditors: renjy
 * @LastEditTime: 2023-09-07 14:35:22
 */

#pragma once

#include <jsoncpp/json/json.h>
#include <fstream>
#include <string>
#include <algorithm>
#include <sstream>
#include <vector>
#include <map>

#include "Eigen/Dense"

#include "message_lib/odometer_message.h"
#include "include/config_struct.h"

/**
 * @name: json读取，字符串处理等
 * @return {*}
 */
namespace internal_common {


void Save(const char *file_name, Json::Value v);

bool Read(const char* file_name, std::string* read_data);

std::vector<std::string> SplitCString(const std::string& line,
  const std::string& delimit);

/**
 * @name: 根据不同的odom类型获取message
 * @param odom_line 文件读取行信息
 * @param odom_type 里程计类型
 * @param message 获取到的message
 * @return {*}
 */
bool GetOdomMessage(const std::string& odom_line,
  const gomros::data_process::mapping_and_location::OdomType& odom_type,
  gomros::message::OdometerMessage* message);

bool GetOdomMessage(const std::vector<std::string>& odom_data,
  const gomros::data_process::mapping_and_location::OdomType& odom_type,
  gomros::message::OdometerMessage* message);

std::string CreatedMapFileName(const std::string& map_name);

// 从文件中读取 landmark信息
void ReadLandmarkInfoFromFile(const std::string& map_file,
  std::map<int, std::map<int, Eigen::Vector2d>>* result);

// 从json中读取 landmark信息
void ReadLandmarkInfoFromJson(const Json::Value &map_json,
  std::map<int, std::map<int, Eigen::Vector2d>>* result);

// 将landmark信息转为json保存
void SaveLandmarkInfoToJson(
  const std::map<int, std::map<int, Eigen::Vector2d>>& reflector_info,
  float radius, Json::Value* map_json,
  float* min_x, float* min_y, float* max_x, float* max_y);


}  // namespace internal_common
