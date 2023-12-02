/*
 * @Description: Copyright (C) 2023 山东亚历山大智能科技有限公司
 * @version: 1.2
 * @Author: oujs
 * @Data: 2023-06-27 13:46:50
 * @LastEditors: renjy
 * @LastEditTime: 2023-06-27 13:50:21
 */
#pragma once

#include <algorithm>
#include <vector>
#include <fstream>
#include <cmath>
#include <string>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include "location/amcl/amcl_sensor.h"
#include "json/json.h"

namespace gomros {
namespace data_process {
namespace mapping_and_location {
namespace amcl {

// 边界框类型
typedef enum {
    POLYGON,
    RECTANGLE,
    CIRCLE,
} box_type_t;

struct boundingBox {
  box_type_t boxtype;     // 所围当前区域的边界框类型
  double weight;          // 当前区域的权重系数，该值越小权重越大
  double (*corners)[2];   // 所围当前区域的角点坐标
  size_t vert_num;           // 所围当前区域的角点数
  // bool mode;           // 按膨胀区域true/绝对区域false修改权重
};

// 权重区域
class WeightedAreas {
 public:
  explicit WeightedAreas(const std::string file_name);
  ~WeightedAreas();

  void ReadAreasFromJson(const std::string file_path);
  double GetWeightOfGrid(const double point[2]);

  std::vector<boundingBox> m_boundingBoxs;    // 地图上所有权重区域
  std::string file_path_;     // 地图文件路径
  std::vector<double> all_weights_;   // 保存所有粒子集合的概率

 private:
  bool IsPointInBox(const double point[2], const boundingBox box);
  int pnpoly(size_t n, const double point[2], const double (*verts)[2]);
};

}  // namespace amcl
}  // namespace mapping_and_location
}  // namespace data_process
}  // namespace gomros

