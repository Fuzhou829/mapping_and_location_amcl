/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.0
 * @Author: renjy
 * @Date: 2022-10-29 14:35:18
 * @LastEditTime: 2023-08-06 17:19:28
 */
#pragma once

#include <cmath>

// let define a small number
#ifndef inf
#define inf 1e-06
#endif  // inf
namespace SLAMMath {


template <typename T>
T Clamp(const T value, const T min, const T max) {
  if (value > max) {
    return max;
  }
  if (value < min) {
    return min;
  }
  return value;
}

template <typename T>
bool InRange(T n, T min, T max) {
  return (n >= min) && (n <= max);
}

template <typename T>
inline T Dist(T x1, T y1, T x2, T y2) {
  return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

template <typename T>
inline T ManHTDist(T x1, T y1, T x2, T y2) {
  return (std::fabs(x1-x2)+std::fabs(y1-y2));
}

inline int ManHTDist(int x, int y) { return (x + y); }  // 曼哈顿距离

inline int ManHTDist(int x1, int y1, int x2, int y2) {
    return (std::abs(x1-x2)+std::abs(y1-y2));
}

template <typename T>
inline T NormalizeTheta(T theta) {
  if (theta > 2 * M_PI) theta -= 2 * M_PI;
  if (theta < -2 * M_PI) theta += 2 *M_PI;
  return theta;
}


template <typename T>
inline T NormalizePITheta(T theta) {
  while (theta >= M_PI) theta -= 2 * M_PI;
  while (theta < -M_PI) theta += 2 * M_PI;
  return theta;
}



template <typename T>
inline void ExPandLine(T x1, T y1, T x2, T y2, T len, T* endx, T* endy) {
  if (x1 - x2 == 0) {
    *endx = x1;
    if (y1 - y2 > 0) {
      *endy = y2 - len;
    } else {
      *endy = y2 + len;
    }
  } else if (y1 - y2 == 0) {
    *endy = y1;
    if (x1 - x2 > 0) {
      *endx = x2 - len;
    } else {
      *endx = x2 + len;
    }
  } else {
    T k = T(0.0);
    T b = T(0.0);
    k = (y1 - y2) / (x1-x2);
    b = y1 - k * x1;
    T zoom = 0.0;
    zoom = len / Dist(x1, y1, x2, y2);

    if (k > 0) {
      if (x1-x2 > 0) {
        *endx = x2 - zoom * (x1-x2);
        *endy = k**endx + b;
      } else {
        *endx = x2 + zoom * (x2-x1);
        *endy = k**endx + b;
      }
    } else {
      if (x1-x2 > 0) {
        *endx = x2 - zoom * (x1-x2);
        *endy = k**endx + b;
      } else {
        *endx = x2 + zoom * (x2 - x1);
        *endy = k**endx + b;
      }
    }
  }
}

template <typename T>
inline bool isEqual(const T a, const T b) {
  const double eps_0 = 1e-6;
  bool isEqualFlag = false;
  if (std::fabs(a - b) <= eps_0) {
    isEqualFlag = true;
  }
  return isEqualFlag;
}


}  // namespace SLAMMath
