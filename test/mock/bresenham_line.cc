/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.0
 * @Author: renjy
 * @Date: 2022-07-12 16:29:37
 * @LastEditTime: 2023-08-07 17:24:30
 */

#include <math.h>

#include <utility>
#include <algorithm>

#include "mock/bresenham_line.h"


namespace LineMath {

void BresenhamLine::SetStartAndEndGrid(
  int start_x, int start_y, int end_x, int end_y) {
  _setLineGrid(start_x, start_y, end_x, end_y);
}

void BresenhamLine::SetStartAndTheta(
  int start_x, int start_y, int length, float theta) {
  int end_x = start_x;
  int end_y = start_y;
  if (fabs(theta - M_PI) < 1e-6) {
    end_x = start_x - length;
  } else if (fabs(theta) < 1e-6) {
    end_x = start_x + length;
  } else if (fabs(theta + M_PI * 0.5) < 1e-6) {
    end_y = start_y + length;
  } else if (fabs(theta - M_PI * 0.5) < 1e-6) {
    end_y = start_y - length;
  } else {
    end_x = std::ceil(start_x + length * std::cos(theta));
    end_y = std::ceil(start_y - length * std::sin(theta));
  }

  _setLineGrid(start_x, start_y, end_x, end_y);
}

bool BresenhamLine::GetNextGrid(int* next_grid_x, int* next_grid_y) {
  if (current_x_ > end_x_) return false;
  if (steep_) {
    *next_grid_x = current_y_;
    *next_grid_y = current_x_;
  } else {
    *next_grid_x = current_x_;
    *next_grid_y = current_y_;
  }
  error_ -= delta_y_;
  if (error_ < 0) {
    current_y_ += y_step_;
    error_ += delta_x_;
  }
  current_x_++;
  return true;
}

void BresenhamLine::_setLineGrid(
  int start_x, int start_y, int end_x, int end_y) {
  steep_ = abs(end_y - start_y) > abs(end_x - start_x);
  if (steep_) {
    std::swap(start_x, start_y);
    std::swap(end_x, end_y);
  }
  if (start_x > end_x) {
    std::swap(start_x, end_x);
    std::swap(start_y, end_y);
  }
  end_x_ = end_x;
  end_y_ = end_y;
  delta_x_ = end_x - start_x;
  delta_y_ = abs(end_y - start_y);
  error_ = delta_x_ / 2.0f;
  y_step_ = start_y < end_y ? 1 : -1;
  current_x_ = start_x;
  current_y_ = start_y;
}

}  // namespace LineMath
