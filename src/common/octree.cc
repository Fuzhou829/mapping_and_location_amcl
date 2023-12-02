/*
 * @Descripttion: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @version: 1.0
 * @Author: renjy
 * @Date: 2022-12-16 12:57:11
 * @LastEditors: renjy
 * @LastEditTime: 2023-07-08 03:56:06
 */
#include "common/octree.h"


namespace Octree {

Map::Map(const Sides3& origin, const Sides3& half_dimension)
: origin_(origin), half_dimension_(half_dimension), data(NULL) {
  resolution_ = 0.10;
  // Initially, there are no children
  for (int i = 0; i < 8; ++i)
    children[i] = NULL;
}



int Map::GetOctantContainingPoint(const Sides3& point) const {
  int oct = 0;
  if (point.x >= origin_.x) oct |= 4;
  if (point.y >= origin_.y) oct |= 2;
  if (point.z >= origin_.z) oct |= 1;
  return oct;
}

bool Map::IsLeafNode() const {
  // This is correct, but overkill. See below.
  /*
      for (int i=0; i<8; ++i)
      if (children[i] != NULL) 
      return false;
      return true;
    */

  // We are a leaf if we have no children. Since we either have none, or
  // all eight, it is sufficient to just check the first.
  return children[0] == NULL;
}

void Map::Insert(OctreePoint* point) {
  // 如果这个节点还没有指定数据点，并且它是一个叶子，那么我们就完成了！
  if (IsLeafNode()) {
    if (data == NULL) {
      data = point;
      return;
    } else {
      // 如果正方体的边长小于分辨率，则停止
      if (half_dimension_.x <= resolution_ / 2.0) {
        // SLAM_WARN("half_dimension_.x %f", half_dimension_.x);
      }
      // We're at a leaf, but there's already something here
      // We will split this node so that it has 8 child octants
      // and then insert the old data that was here, along with
      // this new data point

      // Save this data point that was here for a later re-insert
      OctreePoint *oldPoint = data;
      data = NULL;

      // Split the current node and create new empty trees for each
      // child octant.
      for (int i = 0; i < 8; ++i) {
        // Compute new bounding box for this child
        Sides3 newOrigin = origin_;
        newOrigin.x += half_dimension_.x * (i&4 ? .5f : -.5f);
        newOrigin.y += half_dimension_.y * (i&2 ? .5f : -.5f);
        newOrigin.z += half_dimension_.z * (i&1 ? .5f : -.5f);
        children[i] = new Map(newOrigin, half_dimension_*.5f);
      }

      // Re-insert the old point, and insert this new point
      // (We wouldn't need to insert from the root, because we already
      // know it's guaranteed to be in this section of the tree)
      children[GetOctantContainingPoint(oldPoint->getPosition())]
        ->Insert(oldPoint);
      children[GetOctantContainingPoint(point->getPosition())]
        ->Insert(point);
    }
  } else {
    // We are at an interior node. Insert recursively into the
    // appropriate child octant
    int octant = GetOctantContainingPoint(point->getPosition());
    children[octant]->Insert(point);
  }
}

// 用于查询树中由最小/最大点（bmin，bmax）定义的边界框内的点。所有结果都被推到“结果”中。
void Map::GetPointsInsideBox(const Sides3& bmin, const Sides3& bmax,
  std::vector<OctreePoint>* results) {
  // If we're at a leaf node
  // just see if the current data point is inside the query bounding box
    // 如果是叶子节点且不为空
  if (IsLeafNode()) {
    if (data != NULL) {
      const Sides3& p = data->getPosition();
      if (p.x > bmax.x || p.y > bmax.y || p.z > bmax.z) return;
      if (p.x < bmin.x || p.y < bmin.y || p.z < bmin.z) return;
      results->push_back(*data);
    }
  } else {
    // We're at an interior node of the tree.
    // We will check to see if the query bounding box lies
    // outside the octants of this node.
    // 我们在树的内部节点。我们将检查查询边界框是否位于该节点的八分之一之外。
    for (int i = 0; i < 8; ++i) {
      // Compute the min/max corners of this child octant
      Sides3 cmax = children[i]->origin_ + children[i]->half_dimension_;
      Sides3 cmin = children[i]->origin_ - children[i]->half_dimension_;

      // If the query rectangle is outside the child's bounding box,
      // then continue
      if (cmax.x < bmin.x || cmax.y < bmin.y || cmax.z < bmin.z) continue;
      if (cmin.x > bmax.x || cmin.y > bmax.y || cmin.z > bmax.z) continue;

      // At this point, we've determined that this child is intersecting
      // the query bounding box
      // 递归直到成为叶子节点
      children[i]->GetPointsInsideBox(bmin, bmax, results);
    }
  }
}



}  // namespace Octree

