/*
 * @Descripttion: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @version: 1.0
 * @Author: renjy
 * @Date: 2023-05-06 15:32:57
 * @LastEditors: renjy
 * @LastEditTime: 2023-09-21 15:46:10
 */
#pragma once
#include <map>
#include <vector>
#include <memory>
#include <tuple>


#include "common/octree.h"
#include "common/tool.h"
#include "include/config_struct.h"
#include "landmark_tool/landmark_center_calcuator.h"
#include "landmark_tool/trilateral_calcuator.h"
#include "message_lib/position_message.h"
#include "include/mapping_and_location_math.h"

namespace LandMark {

/**
 * @describes: 三边定位
 * @return {*}
 */
namespace MAL = gomros::data_process::mapping_and_location;

class Trilateration {
 public:
  using Position = gomros::message::Position;

 public:
  explicit Trilateration(const MAL::LandMarkConfig& config);
  virtual ~Trilateration() {}
  /**
   * @description: 将地图转换至该格式
   * @return {*}
   */  
  std::map<int, std::map<int, Eigen::Vector2d>> Transform(
    const std::vector<Eigen::Vector2d>& map_info);
  /**
   * @name: 传入全局地图信息
   * @return {*}
   */
  void SetLandMarkMap(
    const std::map<int, std::map<int, Eigen::Vector2d>>& map_info);

  /**
   * @name: 获取是否得到重定位位姿 (启用三边定位)
   * @param obs_info 观测反光柱信息
   * @param global_pos 获取到的全局定位信息
   * @return {*} 是否
   */  
  bool IsGetGlobalPos(const std::vector<Eigen::Vector2d>& obs_info,
        Eigen::Vector3d* global_pos);

  /**
   * @describes: 获取对应的id信息
   * @param result 对应全局地图id 和对应的obs id
   * @return {*}
   */
  void GetMatchId(std::map<int, int>* result);
  /**
   * @description: 获取定位的子图ID
   * @return {*}
   */
  int GetMatchSubmapId() const {
    return match_map_id_;
  }

 private:
   /**
   * @name: 构建八叉树地图
   * @return {*}
   */ 
  void _buildOctreeReflectorMap();

  /**
   * @description: 在地图中寻找全局坐标
   * @param map_id -1 代表全局地图
   * @param observation_info 观测信息
   * @param global_pos 车体全局坐标
   * @param global_id 与全局地图的匹配 （在子图中取全局坐标时不进行处理）
   * @return {*}
   */
  bool _getGlobalPosFromMap(int map_id,
    const std::vector<Eigen::Vector2d>& observation_info,
    Eigen::Vector3d* global_pos, std::vector<int>* global_id);

  /**
   * @description: 获取最匹配的子图信息
   * @return int 最佳匹配子图
   */
  int _matchMapId(const std::vector<int>& global_map_id);

  /**
   * @name: 判断三角形是否符合放入八叉树的条件
   * @param {float} side_1 边长1
   * @param {float} side_2 边长2
   * @param {float} side_3 边长3
   * @return {bool} true 满足 false 不满足
   */
  bool _checkTriangle(float side_1, float side_2, float side_3);

  /**
   * @name: 查找八叉树反光柱地图中的相似三角形
   * @param {float} side_1 边长1
   * @param {float} side_2 边长2
   * @param {float} side_3 边长3
   * @param {int} map_id 地图id
   * @param {std::vector<Octree::OctreePoint>*} similar_triangles 相似三角形
   * @return {*}
   */  
  void _findSimilarTriangles(float side_1, float side_2, float side_3,
          int map_id, std::vector<Octree::OctreePoint>* similar_triangles);

  /**
   * @description: 查找得到的匹配结果是否正确， 避免一个反光柱子匹配不同的
   * @param std::map<int, int> match_id 多边定位匹配结果 （地图 观测id）
   * @param std::vector<Eigen::Vector2d> observation_info 观测信息
   * @param int map_id 地图id
   * @param std::vector<int> 和地图id匹配的landmark id集合
   * @param triangle_points 地图点集合 -- 形成三角形
   * @param observation_triangle_points 观测点集合 -- 形成三角形
   * @return {*}
   */
  void _checkMatchResult(const std::map<int, int>& match_id,
                    const std::vector<Eigen::Vector2d>& observation_info,
                    int map_id, std::vector<int>* global_id,
                    std::vector<Eigen::Vector2d>* triangle_points,
                    std::vector<Eigen::Vector2d>* observation_triangle_points);
  /**
   * @name: 进行定位
   * @param map_triangle map中对应三角形顶点
   * @param observation_triangle 观测三角形顶点
   * @param global_pos 全局坐标
   * @return {*} bool 是否定位成功
   */
  bool _doLocation(const std::vector<Eigen::Vector2d>& map_triangle,
                   const std::vector<Eigen::Vector2d>& observation_triangle,
                    Eigen::Vector3d* global_pos);


 private:
  MAL::LandMarkConfig config_;
  std::map<int, std::map<int, Eigen::Vector2d>> land_mark_map_;
  std::map<int, std::shared_ptr<Octree::Map>> octree_reflector_maps_;
  // 匹配信息 地图id 观测id
  std::map<int, int> match_id_;
  int match_map_id_;
};




}  // namespace LandMark
