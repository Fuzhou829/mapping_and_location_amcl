/*
 * @Descripttion: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @version: 1.0
 * @Author: renjy
 * @Date: 2022-12-16 12:46:34
 * @LastEditors: renjy
 * @LastEditTime: 2023-07-08 03:55:55
 */
#pragma once

#include <cmath>
#include <vector>

#include "common/logger.h"

namespace Octree {

struct Sides3;
Sides3 operator*(float r, const Sides3& v);

struct Sides3 {
  union {
    struct {
      float x, y, z;
    };
    float D[3];
  };

  Sides3() { }
  Sides3(float _x, float _y, float _z)
    :x(_x), y(_y), z(_z)
  { }

  float& operator[](unsigned int i) {
    return D[i];
  }

  const float& operator[](unsigned int i) const {
    return D[i];
  }

  float maxComponent() const {
    float r = x;
    if (y > r) r = y;
    if (z > r) r = z;
    return r;
  }

  float minComponent() const {
    float r = x;
    if (y < r) r = y;
    if (z < r) r = z;
    return r;
  }

  Sides3 operator+(const Sides3& r) const {
    return Sides3(x + r.x, y + r.y, z + r.z);
  }

  Sides3 operator-(const Sides3& r) const {
    return Sides3(x - r.x, y - r.y, z - r.z);
  }

  Sides3 cmul(const Sides3& r) const {
    return Sides3(x * r.x, y * r.y, z * r.z);
  }

  Sides3 cdiv(const Sides3& r) const {
    return Sides3(x / r.x, y / r.y, z / r.z);
  }

  Sides3 operator*(float r) const {
    return Sides3(x * r, y * r, z * r);
  }


  Sides3 operator/(float r) const {
    return Sides3(x / r, y / r, z / r);
  }

  Sides3& operator+=(const Sides3& r) {
    x += r.x;
    y += r.y;
    z += r.z;
    return *this;
  }

  Sides3& operator-=(const Sides3& r) {
    x -= r.x;
    y -= r.y;
    z -= r.z;
    return *this;
  }

  Sides3& operator*=(float r) {
    x *= r; y *= r; z *= r;
    return *this;
  }

  // Inner/dot product
  float operator*(const Sides3& r) const {
    return x * r.x + y * r.y + z * r.z;
  }

  float norm() const {
    return sqrtf(x * x + y * y + z * z);
  }

  float normSquared() const {
    return x * x + y * y + z * z;
  }

  // Cross product
  Sides3 operator^(const Sides3& r) const {
    return Sides3(
        y * r.z - z * r.y,
        z * r.x - x * r.z,
        x * r.y - y * r.x);
  }

  Sides3 normalized() const {
    return *this / norm();
  }
};

inline Sides3 operator*(float r, const Sides3& v) {
    return Sides3(v.x * r, v.y * r, v.z * r);
}

class OctreePoint {
  Sides3 position;
  std::vector<int> landmark_local_ids;  // 有序的（排列问题）
 public:
  OctreePoint() { }
  explicit OctreePoint(const Sides3& p) : position(p) {}
  OctreePoint(const Sides3& p, const std::vector<int>& ids)
    : position(p), landmark_local_ids(ids) {}
  inline const Sides3& getPosition() const {
            return position;
          }
  inline const std::vector<int>& getLandmarkids() const {
            return landmark_local_ids;
          }
  inline void setValue(const Sides3& p, const std::vector<int>& ids) {
            position = p; landmark_local_ids = ids;
          }
};



class Map {
 public:
  Map(const Sides3& origin, const Sides3& half_dimension);
  Map(const Map& copy)
    : origin_(copy.origin_), half_dimension_(copy.half_dimension_),
      data(copy.data) {}

  ~Map() {
    // Recursively destroy octants
    for (int i = 0; i < 8; ++i) {
      if (children[i]) {
        delete children[i];
      }
    }
  }

  // Determine which octant of the tree would contain 'point'
  // 确定树的哪个八分之一将包含“点”
  int GetOctantContainingPoint(const Sides3& point) const;
  bool IsLeafNode() const;
  void Insert(OctreePoint* point);
  void GetPointsInsideBox(const Sides3& bmin, const Sides3& bmax,
                          std::vector<OctreePoint>* results);

 private:
  Sides3 origin_;         //! The physical center of this node
  Sides3 half_dimension_;  //! Half the width/height/depth of this node
  float resolution_;
  Map *children[8];  //! Pointers to child octants,指针数组
  OctreePoint *data;   //! Data point to be stored at a node
};




}  // namespace Octree
