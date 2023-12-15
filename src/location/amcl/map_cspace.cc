/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
 *                      gerkey@usc.edu    kaspers@robotics.usc.edu
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include <queue>

#include "location/amcl/map.h"

#include "location/amcl/weighted_areas.h"
#include "common/logger.h"

namespace gomros {
namespace data_process {
namespace mapping_and_location {
namespace amcl {

class CellData {
 public:
  map_t* map_;
  unsigned int i_, j_;
  unsigned int src_i_, src_j_;
};

class CompareOccDist {
  public:
    bool operator()(const CellData& a, const CellData& b) {
        return a.map_->cells[MAP_INDEX(a.map_, a.i_, a.j_)].occ_dist >
               b.map_->cells[MAP_INDEX(b.map_, b.i_, b.j_)].occ_dist;
    }
};

class CompareOccDistLand {
  public:
    bool operator()(const CellData& a, const CellData& b) {
        return a.map_->cells[MAP_INDEX(a.map_, a.i_, a.j_)].occ_dist_to_land_mark >
               b.map_->cells[MAP_INDEX(b.map_, b.i_, b.j_)].occ_dist_to_land_mark;
    }
};

bool compare_occ_dist(const CellData& a, const CellData& b) {
  return a.map_->cells[MAP_INDEX(a.map_, a.i_, a.j_)].occ_dist >
         a.map_->cells[MAP_INDEX(b.map_, b.i_, b.j_)].occ_dist;
}

bool compare_occ_dist_land(const CellData& a, const CellData& b) {
  return a.map_->cells[MAP_INDEX(a.map_, a.i_, a.j_)].occ_dist_to_land_mark >
         a.map_->cells[MAP_INDEX(b.map_, b.i_, b.j_)].occ_dist_to_land_mark;
}

class CachedDistanceMap {
 public:
  CachedDistanceMap(double scale, double max_dist, double max_dist_land)
      : distances_(NULL), scale_(scale), max_dist_(max_dist), max_dist_land_(max_dist_land) {

    cell_radius_ = max_dist / scale;
    cell_radius_land_ = max_dist_land_ / scale;
    distances_ = new double*[cell_radius_land_ + 2];
    for (int i = 0; i <= cell_radius_land_ + 1; i++) {
      distances_[i] = new double[cell_radius_land_ + 2];
      for (int j = 0; j <= cell_radius_land_ + 1; j++) {
        distances_[i][j] = sqrt(i * i + j * j);
      }
    }
  }
  ~CachedDistanceMap() {
    if (distances_) {
      for (int i = 0; i <= cell_radius_ + 1; i++) delete[] distances_[i];
      delete[] distances_;
    }
  }
  double** distances_;
  double scale_;
  double max_dist_;  
  double max_dist_land_;
  int cell_radius_;
  int cell_radius_land_;
};

// bool operator<(const CellData& a, const CellData& b) {
//   return a.map_->cells[MAP_INDEX(a.map_, a.i_, a.j_)].occ_dist >
//          a.map_->cells[MAP_INDEX(b.map_, b.i_, b.j_)].occ_dist;
// }

CachedDistanceMap* get_distance_map(double scale, double max_dist, double max_dist_land) {
  static CachedDistanceMap* cdm = NULL;

  if (!cdm || (cdm->scale_ != scale) || (cdm->max_dist_ != max_dist)) {
    if (cdm) delete cdm;
    cdm = new CachedDistanceMap(scale, max_dist, max_dist_land);
  }

  return cdm;
}

void enqueue(map_t* map, unsigned int i, unsigned int j, unsigned int src_i,
             unsigned int src_j, std::priority_queue< CellData, std::vector<CellData>, CompareOccDist >& Q,
             CachedDistanceMap* cdm, unsigned char* marked, WeightedAreas* weighted_areas) {
  if (marked[MAP_INDEX(map, i, j)]) return;

  unsigned int di = abs((int)i - (int)src_i);
  unsigned int dj = abs((int)j - (int)src_j);
  double distance = cdm->distances_[di][dj];

  if (distance > cdm->cell_radius_) return;

  // 获取weighted_areas系数
  map->cells[MAP_INDEX(map, i, j)].occ_dist = distance * map->scale;

  CellData cell;
  cell.map_ = map;
  cell.i_ = i;
  cell.j_ = j;
  cell.src_i_ = src_i;
  cell.src_j_ = src_j;

  Q.push(cell);

  marked[MAP_INDEX(map, i, j)] = 1;
}

void enqueue2(map_t* map, unsigned int i, unsigned int j, unsigned int src_i,
             unsigned int src_j, std::priority_queue< CellData, std::vector<CellData>, CompareOccDistLand >& Q,
             CachedDistanceMap* cdm, unsigned char* marked, WeightedAreas* weighted_areas) {
  if (marked[MAP_INDEX(map, i, j)]) return;

  unsigned int di = abs((int)i - (int)src_i);
  unsigned int dj = abs((int)j - (int)src_j);
  double distance = cdm->distances_[di][dj];

  if (distance > cdm->cell_radius_land_) return;

  map->cells[MAP_INDEX(map, i, j)].occ_dist_to_land_mark = distance * map->scale;

  CellData cell;
  cell.map_ = map;
  cell.i_ = i;
  cell.j_ = j;
  cell.src_i_ = src_i;
  cell.src_j_ = src_j;

  Q.push(cell);

  marked[MAP_INDEX(map, i, j)] = 1;

}

// Update the cspace distance values
void map_update_cspace(map_t* map, double max_occ_dist, double max_occ_dist_land, WeightedAreas* weighted_areas) {
  SLAM_WARN("进入map_update_cspace\n");
  unsigned char* marked;
  unsigned char*marked2;
  std::priority_queue< CellData, std::vector<CellData>, CompareOccDist > Q1;
  std::priority_queue< CellData, std::vector<CellData>, CompareOccDistLand > Q2;

  marked = new unsigned char[map->size_x * map->size_y];
  marked2 = new unsigned char[map->size_x * map->size_y];
  SLAM_WARN("地图高度为 %d, 宽度为 %d  max_occ_dist_land %f\n", map->size_x, map->size_y, max_occ_dist_land);
  memset(marked, 0, sizeof(unsigned char) * map->size_x * map->size_y);

  map->max_occ_dist = max_occ_dist;
  map->max_occ_dist_land = max_occ_dist_land;

  CachedDistanceMap* cdm = get_distance_map(map->scale, map->max_occ_dist, map->max_occ_dist_land);

  // Enqueue all the obstacle cells
  CellData cell;
  cell.map_ = map;
  SLAM_WARN("开始计算占用栅格\n");
  for (int i = 0; i < map->size_x; i++) {
    cell.src_i_ = cell.i_ = i;
    for (int j = 0; j < map->size_y; j++) {
      if (map->cells[MAP_INDEX(map, i, j)].occ_state == +1) {
        map->cells[MAP_INDEX(map, i, j)].occ_dist = 0.0;
        cell.src_j_ = cell.j_ = j;
        marked[MAP_INDEX(map, i, j)] = 1;
        Q1.push(cell);
      } else
        map->cells[MAP_INDEX(map, i, j)].occ_dist = max_occ_dist;
    }
  }

  // 此处输出初始化occ_dist的栅格地图图像
  cv::Size size(map->size_x, map->size_y);
  cv::Mat img_map_init(size, CV_8UC1, cv::Scalar(255));
  // for (int i = 0; i < map->size_x; i++) {
  //   for (int j = 0; j < map->size_y; j++) {
  //     img_map_init.at<uchar>(j, i) = floor(map->cells[MAP_INDEX(map, i, j)].occ_dist * 255) / max_occ_dist;
  //   }
  // }
  cv::imwrite("./img_map_init.png", img_map_init);
  SLAM_WARN("开始计算occ_dist\n");
  while (!Q1.empty()) {
    CellData current_cell = Q1.top();
    if (current_cell.i_ > 0)
      enqueue(map, current_cell.i_ - 1, current_cell.j_, current_cell.src_i_,
              current_cell.src_j_, Q1, cdm, marked, weighted_areas);
    if (current_cell.j_ > 0)
      enqueue(map, current_cell.i_, current_cell.j_ - 1, current_cell.src_i_,
              current_cell.src_j_, Q1, cdm, marked, weighted_areas);
    if ((int)current_cell.i_ < map->size_x - 1)
      enqueue(map, current_cell.i_ + 1, current_cell.j_, current_cell.src_i_,
              current_cell.src_j_, Q1, cdm, marked, weighted_areas);
    if ((int)current_cell.j_ < map->size_y - 1)
      enqueue(map, current_cell.i_, current_cell.j_ + 1, current_cell.src_i_,
              current_cell.src_j_, Q1, cdm, marked, weighted_areas);
    Q1.pop();
  }
  cv::Mat img_map_likelihood(size, CV_8UC1, cv::Scalar(255));
  double grayscale_temp, grayscale_temp1;
  for (int i = 0; i < map->size_x; i++) {
    for (int j = 0; j < map->size_y; j++) {
      grayscale_temp = floor(map->cells[MAP_INDEX(map, i, j)].occ_dist * 255) / max_occ_dist;
      grayscale_temp = grayscale_temp < 255 ? grayscale_temp : 255;
      img_map_likelihood.at<uchar>(j, i) = grayscale_temp;
    }
  }
  cv::imwrite("./img_map_likelihood.png", img_map_likelihood);
  // 创建反光柱似然与地图 
  memset(marked2, 0, sizeof(unsigned char) * map->size_x * map->size_y);
  SLAM_WARN("开始计算反光柱占用栅格\n");
  for (int i = 0; i < map->size_x; i++) {
    cell.src_i_ = cell.i_ = i;
    for (int j = 0; j < map->size_y; j++) {
      if (map->cells[MAP_INDEX(map, i, j)].occ_state == +2) {
        map->cells[MAP_INDEX(map, i, j)].occ_dist_to_land_mark = 0.0;
        cell.src_j_ = cell.j_ = j;
        marked[MAP_INDEX(map, i, j)] = 1;
        Q2.push(cell);
      } else
        map->cells[MAP_INDEX(map, i, j)].occ_dist_to_land_mark = max_occ_dist_land;
    }
  }
  // for (int i = 0; i < map->size_x; i++) {
  //   cell.src_i_ = cell.i_ = i;
  //   for (int j = 0; j < map->size_y; j++) {
  //       cell.src_j_ = cell.j_ = j;     
  //       map->cells[MAP_INDEX(map, i, j)].occ_dist_to_land_mark = max_occ_dist_land;
  //   }
  // }
  // for (auto landmark_grid : map->landmarks) {
  //   int i = landmark_grid.first;
  //   int j = landmark_grid.second;
  //   map->cells[MAP_INDEX(map, i, j)].occ_dist_to_land_mark = 0.0;
  //   marked[MAP_INDEX(map, i, j)] = 1;
  //   Q2.push(cell);
  // }
  
  SLAM_WARN("反光柱栅格数量%d\n", Q2.size());
  SLAM_WARN("开始计算occ_dist_to_landmark\n");
  while (!Q2.empty()) {
    CellData current_cell = Q2.top();
    if (current_cell.i_ > 0)
      enqueue2(map, current_cell.i_ - 1, current_cell.j_, current_cell.src_i_,
              current_cell.src_j_, Q2, cdm, marked2, weighted_areas);
    if (current_cell.j_ > 0)
      enqueue2(map, current_cell.i_, current_cell.j_ - 1, current_cell.src_i_,
              current_cell.src_j_, Q2, cdm, marked2, weighted_areas);
    if ((int)current_cell.i_ < map->size_x - 1)
      enqueue2(map, current_cell.i_ + 1, current_cell.j_, current_cell.src_i_,
              current_cell.src_j_, Q2, cdm, marked2, weighted_areas);
    if ((int)current_cell.j_ < map->size_y - 1)
      enqueue2(map, current_cell.i_, current_cell.j_ + 1, current_cell.src_i_,
              current_cell.src_j_, Q2, cdm, marked2, weighted_areas);
    Q2.pop();
  }
  

  // 此处输出处理后的似然域栅格地图图像（TODO）
	cv::Mat img_map_landmark_likelihood(size, CV_8UC1, cv::Scalar(255));
  for (int i = 0; i < map->size_x; i++) {
    for (int j = 0; j < map->size_y; j++) {
      grayscale_temp = floor(map->cells[MAP_INDEX(map, i, j)].occ_dist * 255) / max_occ_dist;
      grayscale_temp1 = floor(map->cells[MAP_INDEX(map, i, j)].occ_dist_to_land_mark * 255) / max_occ_dist_land;
      grayscale_temp = std::min(grayscale_temp, grayscale_temp1);
      grayscale_temp = grayscale_temp < 255 ? grayscale_temp : 255;
      img_map_landmark_likelihood.at<uchar>(j, i) = grayscale_temp;
    }
  }
  cv::imwrite("./img_map_landmrak_likelihood.png", img_map_landmark_likelihood);
  cv::Mat img_map_landmark_likelihood1(size, CV_8UC1, cv::Scalar(255));
  for (int i = 0; i < map->size_x; i++) {
    for (int j = 0; j < map->size_y; j++) {
      grayscale_temp = floor(map->cells[MAP_INDEX(map, i, j)].occ_dist_to_land_mark * 255) / max_occ_dist_land;
      grayscale_temp = grayscale_temp < 255 ? grayscale_temp : 255;
      img_map_landmark_likelihood1.at<uchar>(j, i) = grayscale_temp;
    }
  }
  cv::imwrite("./img_map_landmrak_likelihood1.png", img_map_landmark_likelihood1);
  SLAM_WARN("Likelihood map construction completed, starting localization ......\n");

  delete[] marked;
  delete[] marked2;
}



}  // namespace amcl
}  // namespace mapping_and_location
}  // namespace data_process
}  // namespace gomros
