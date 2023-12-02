/*
 * @Description: Copyright (C) 2023 山东亚历山大智能科技有限公司
 * @version: 1.2
 * @Author: renjy
 * @Data: 2023-06-04 08:27:09
 * @LastEditors: renjy
 * @LastEditTime: 2023-08-06 14:55:38
 */

#pragma once

#include <map>
#include <utility>
#include <memory>
#include <vector>

#include "reflector_mapping/submap.h"
#include "include/mapping_and_location_math.h"
#include "include/config_struct.h"
#include "landmark_tool/trilateration.h"
#include "landmark_tool/trilateral_calcuator.h"


namespace gomros {
namespace data_process {
namespace mapping_and_location {


/**
 * @describes: 约束关系
 */
struct Constraint {
  int start_submap_id;
  int start_reflector_id;
  int end_submap_id;
  int end_reflector_id;
  float distance;
  // 新子图在关联子图中的位姿
  Eigen::Vector3d in_end_map_pos;
  Constraint() {
    start_submap_id = -1;
    start_reflector_id = -1;
    end_submap_id = -1;
    end_reflector_id = -1;
    distance = 0.0;
  }
  Constraint(int start_submap_id, int start_reflector_id,
             int end_submap_id, int end_reflector_id, float distance) {
    this->start_submap_id = start_submap_id;
    this->start_reflector_id = start_reflector_id;
    this->end_submap_id = end_submap_id;
    this->end_reflector_id = end_reflector_id;
    this->distance = distance;
  }
};


/**
 * @describes: 为子图计算约束
 * @return {*}
 */

class SubMapConstraint {
 public:
  explicit SubMapConstraint(const MappingConfig& config) : config_(config) {}
  ~SubMapConstraint() {}
  void Init() {
    constraints_.clear();
  }

  /**
   * @describes: 计算子图之间是否存在约束关系，在建图结束之后和加入新子图的时候调用
   * @param reflectors_in_submaps submaps 中的反光柱坐标信息
   * @param submap_global_pos submaps 在全局中的坐标系（也是第一帧数据带入的）
   * @param finished_submap 目前全部建图的子图
   * @return {*}
   */
  bool ComputeSubMapConstraint(
      const std::map<int, std::vector<Eigen::Vector2d>>& reflectors_in_submaps,
      const std::map<int, Eigen::Vector3d>& submap_global_pos,
      const std::vector<std::shared_ptr<Submap>>& finished_submap);
  /**
   * @describes: 获取求解得到的约束关系
   * @return {*}
   */  
  const std::vector<Constraint> GetConstraint() const {
    return constraints_;
  }
  void ClearConstraint() {
    constraints_.clear();
  }

 private:
  /**
   * @describes: 计算新子图与时刻接近的子图之间的约束关系
   * @param new_submap_id
   * @param current_submap_id
   * @param new_submap_pos
   * @param current_submap_pos
   * @param new_submap_reflectors
   * @param current_submap_reflectors
   * @return {*}
   */
  bool _calContraintForTimenearSubmap(int new_submap_id, int current_submap_id,
      const Eigen::Vector3d& new_submap_pos,
      const Eigen::Vector3d& current_submap_pos,
      const std::vector<Eigen::Vector2d>& new_submap_reflectors,
      const std::vector<Eigen::Vector2d>& current_submap_reflectors);

  /**
   * @describes: 计算子图间是否存在闭环关系
   * @param new_submap
   * @param current_submap
   * @return {*}
   */
  bool _isGetConstraintForLoopSubMap(const std::shared_ptr<Submap>& new_submap,
    const std::shared_ptr<Submap>& current_submap);

  /**
   * @describes:  将submap局部坐标下的反光柱转换至全局坐标
   * @param localcenter submap下反光柱的位置
   * @param submap_global_pos submap在全局中的坐标
   * @return {*}
   */
  Eigen::Vector2d _local2Global(const Eigen::Vector2d &localcenter,
    const Eigen::Vector3d &submap_global_pos);

  /**
   * @describes: 通过距离阈值进行匹配
   * @param new_submap_pos
   * @param current_submap_pos
   * @param new_submap_reflectors
   * @param current_submap_reflectors
   * @param initial_association_result 初匹配结果
   * @return {*}
   */
  void _getInitialAssociationResult(
      const Eigen::Vector3d& new_submap_pos,
      const Eigen::Vector3d& current_submap_pos,
      const std::vector<Eigen::Vector2d>& new_submap_reflectors,
      const std::vector<Eigen::Vector2d>& current_submap_reflectors,
      std::vector<std::pair<int, int>>* initial_association_result);

  /**
   * @describes: 获取约束信息
   * @param new_submap_id
   * @param current_submap_id
   * @param new_submap_pos
   * @param current_submap_pos
   * @param new_submap_reflectors
   * @param current_submap_reflectors
   * @return {*}
   */
  void _getContinstrint(int new_submap_id, int current_submap_id,
    const Eigen::Vector3d& new_submap_pos,
      const Eigen::Vector3d& current_submap_pos,
      const std::vector<Eigen::Vector2d>& new_submap_reflectors,
      const std::vector<Eigen::Vector2d>& current_submap_reflector);

  /**
   * @describes: 在八叉树内查找相似三角形
   * @param side_1 边长1
   * @param side_2 边长2
   * @param side_3 边长3
   * @param octree 八叉树
   * @param similar_triangles 相似三角形
   * @return {*}
   */
  void _findSimilarTriangles(double side_1, double side_2, double side_3,
                        std::shared_ptr<Octree::Map> octree,
                        std::vector<Octree::OctreePoint>* similar_triangles);
  /**
   * @describes: 判断是否出现回环 回环后位姿+子图中其他反光柱匹配
   * @param obs_id 观测到的信息id -- 对应new_submap中的点
   * @param new_submap new_finished_submap
   * @param similar_triangles 对应点 -- 对应current_submap中的点
   * @param current_submap 上一时刻子图
   * @return {*}
   */
  bool _isLoop(const std::vector<int>& obs_id,
               const std::shared_ptr<Submap>& new_submap,
               const std::vector<Octree::OctreePoint>& similar_triangles,
               const std::shared_ptr<Submap>& current_submap);

  /**
   * @describes: 判断是否为最优的约束关系
   * @param {Constraint&} constraint 和 已有的进行比较
   * @return {*}
   */
  bool _checkBestConstraints(const Constraint& constraint);

 private:
  MappingConfig config_;
  std::vector<Constraint> constraints_;
};








}  // namespace mapping_and_location
}  // namespace data_process
}  // namespace gomros
