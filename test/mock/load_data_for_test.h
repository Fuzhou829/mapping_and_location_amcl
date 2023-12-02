/*
 * @Description: Copyright (C) 2023 山东亚历山大智能科技有限公司
 * @version: 1.2
 * @Author: renjy
 * @Data: 2023-09-21 14:30:39
 * @LastEditors: renjy
 * @LastEditTime: 2023-09-21 15:00:19
 */
#pragma once

#include <string>
#include <vector>
#include <fstream>

#include "Eigen/Dense"
#include "common/logger.h"

namespace LoadDataForTest {

struct LadarInfo {
  uint64_t time;
  std::vector<Eigen::Vector2d> mvPoints; /* 二维雷达数据X、Y坐标信息 */
  std::vector<float> mvIntensities;      /* 点云光强信息 */
};




/**
 * @description: 读取文件中的雷达点云数据
 * @param {string&} file_dir 文件地址 格式：时间戳 \n x y i
 * @param {LadarInfo*} ladar_info
 * @return {*}
 */
bool ReadLadar(const std::string& file_dir, LadarInfo* ladar_info);




}  // namespace LoadDataForTest


