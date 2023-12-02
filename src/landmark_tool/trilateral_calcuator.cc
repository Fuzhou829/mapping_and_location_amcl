/*
 * @Description: Copyright (C) 2023 山东亚历山大智能科技有限公司
 * @version: 1.2
 * @Author: renjy
 * @Data: 2023-06-07 03:02:15
 * @LastEditors: renjy
 * @LastEditTime: 2023-06-29 11:13:05
 */

#include "landmark_tool/trilateral_calcuator.h"
#include "include/mapping_and_location_math.h"

namespace LandMark {



bool TrilateralCal::GetGlobalPos(
  const std::vector<Eigen::Vector2d>& map_triangle,
  const std::vector<Eigen::Vector2d>& observation_triangle,
  Eigen::Vector3d* result) {
  assert(map_triangle.size() == observation_triangle.size());
  if (map_triangle.size() < 3) return false;
  int size = map_triangle.size();
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(size - 1, 2);
  Eigen::MatrixXd b = Eigen::MatrixXd::Zero(size - 1, 1);
  auto iter = map_triangle.begin();
  double Xn = map_triangle[0](0);
  double Yn = map_triangle[0](1);
  double dn = sqrt(pow(observation_triangle[0](0), 2) +
                   pow(observation_triangle[0](1), 2));
  int i = 0;
  for (int count = 1; count < size; count++) {
    double X = map_triangle[count](0);
    double Y = map_triangle[count](1);
    double d = sqrt(pow(observation_triangle[count](0), 2) +
                    pow(observation_triangle[count](1), 2));
    A(i, 0) = 2 * (X - Xn);
    A(i, 1) = 2 * (Y - Yn);
    b(i, 0) = X * X - Xn * Xn + Y * Y - Yn * Yn + dn * dn - d * d;
    i++;
  }

  Eigen::MatrixXd pose_xy = (A.transpose() * A).inverse() * A.transpose() * b;
  (*result)(0) = pose_xy(0, 0);
  (*result)(1) = pose_xy(1, 0);

  std::vector<double> angles;
  float sum_theta = 0.0;
  for (int count = 0; count < map_triangle.size(); count++) {
      double y1 = map_triangle[count](1);
      double x1 = map_triangle[count](0);
      double y2 = observation_triangle[count](1);
      double x2 = observation_triangle[count](0);
      double theta1 = atan2(y1 - (*result)(1), x1 - (*result)(0));
      double theta2 = atan2(y2, x2);
      double theta3 = SLAMMath::NormalizePITheta(theta1 - theta2);
      angles.push_back(theta3);
  }


  double diff = 0.0;
  double last = angles[0];  // 只有size大于等于3时才会进入这里
  double sum = angles[0];
  for (i = 1; i < angles.size(); i++) {
      diff = SLAMMath::NormalizePITheta(angles[i] - angles[i - 1]);
      last += diff;
      sum += last;
  }
  (*result)(2) = SLAMMath::NormalizePITheta(sum / size);

  return true;
}










}  // namespace LandMark











