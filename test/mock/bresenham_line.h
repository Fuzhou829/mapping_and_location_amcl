/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.0
 * @Author: renjy
 * @Date: 2022-07-12 14:16:07
 * @LastEditTime: 2023-03-24 05:58:55
 */

#pragma once

namespace LineMath {

/**
 * @brief: Bresenham直线
 * 坐标系：
 *  O------------>x(0)
 *  |
 *  |
 *  |
 *  |
 *  ↓
 *  y(-pi/2)
 */
class BresenhamLine {
 public:
  BresenhamLine() {}
  /**
   * @brief: 用直线起点终点对BresenhamLine初始化
   * @param {int} start_x 直线起点x坐标 单位：栅格
   * @param {int} start_y 直线起点y坐标 单位：栅格
   * @param {int} end_x 直线终点x坐标 单位：栅格
   * @param {int} end_y 直线终点y坐标 单位：栅格
   * @return {*}
   */  
  void SetStartAndEndGrid(int start_x, int start_y, int end_x, int end_y);
  /**
   * @brief: 给定直线起点、直线长度、直线角度，获取直线栅格信息
   * @param {int} start_x 直线起点x坐标 单位：栅格
   * @param {int} start_y 直线起点y坐标 单位：栅格
   * @param {int} length  直线长度 单位：栅格
   * @param {float} theta 栅格坐标系下弧度(-pi, pi]
   * @return {*}
   */  
  void SetStartAndTheta(int start_x, int start_y, int length, float theta);

  /**
   * @brief: 获取直线的下一个点
   * @param {int*} next_grid_x 直线的下一个栅格点
   * @param {int*} next_grid_y 直线的下一个栅格点
   * @return bool true 获取下一个目标点成功 false 获取下一目标点失败
   */
  bool GetNextGrid(int* next_grid_x, int* next_grid_y);

 private:
  /**
   * @brief: 用直线起点终点对BresenhamLine初始化
   * @param {int} start_x 直线起点x坐标 单位：栅格
   * @param {int} start_y 直线起点y坐标 单位：栅格
   * @param {int} end_x 直线终点x坐标 单位：栅格
   * @param {int} end_y 直线终点y坐标 单位：栅格
   * @return {*}
   */  
  void _setLineGrid(int start_x, int start_y, int end_x, int end_y);

 private:
  bool steep_;
  int end_x_;
  int end_y_;
  int delta_x_;
  int delta_y_;
  int error_;
  int y_step_;
  int current_x_;
  int current_y_;
};


}  // namespace LineMath
