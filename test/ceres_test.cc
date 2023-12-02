/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.0
 * @Author: renjy
 * @Date: 2022-11-22 14:51:22
 * @LastEditTime: 2023-11-08 20:32:20
 */

#include <gtest/gtest.h>
#include <ceres/ceres.h>

#include <vector>
#include "opencv2/opencv.hpp"

#include "include/mapping_and_location_math.h"


/*代价函数的计算模型*/

struct CURVE_FITTING_COST {
  CURVE_FITTING_COST(double x, double y): _x(x), _y(y) {}
  // 残差的计算
  template<typename T>
  bool operator()(
      const T *const abc,  // 模型参数，有3维
      T* residual) const {
          // y = 1800+ax + bxx+cxxx
          residual[0] = T(_y) - abc[0] -
                    abc[1] * T(_x) - abc[2] * T(_x) * T(_x);
          return true;
      }

  const double _x, _y;  // x，y数据
};

TEST(CERES, FitCurve) {
  // 利用cere拟合曲线
  using namespace std;  // NOLINT
  double ae = 100, be = 10.0, ce = 0.1;  //   估计参数值

  /*第二部分，定义残差块*/
  double abc[3] = {ae, be, ce};  // 定义待优化的参数，并赋初值
  ceres::Problem problem;  // 构建最小二乘问题
  // 向问题中添加误差项，每一个误差项是一个ResidualBlock
  // 每个ResidualBlock由三部分组成：误差项、核函数、待估计参数
  problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 3>(
        new CURVE_FITTING_COST(1, 1050)), nullptr, abc);
  problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 3>(
        new CURVE_FITTING_COST(2, 1010)), nullptr, abc);
  problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 3>(
        new CURVE_FITTING_COST(3, 1000)), nullptr, abc);
  problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 3>(
        new CURVE_FITTING_COST(4, 950)), nullptr, abc);
  problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 3>(
        new CURVE_FITTING_COST(5, 930)), nullptr, abc);
  problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 3>(
        new CURVE_FITTING_COST(6, 700)), nullptr, abc);
  problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 3>(
        new CURVE_FITTING_COST(7, 660)), nullptr, abc);
  problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 3>(
        new CURVE_FITTING_COST(8, 630)), nullptr, abc);
    problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 3>(
        new CURVE_FITTING_COST(9, 620)), nullptr, abc);
  problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 3>(
        new CURVE_FITTING_COST(10, 610)), nullptr, abc);
  problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 3>(
        new CURVE_FITTING_COST(11, 600)), nullptr, abc);
  problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 3>(
        new CURVE_FITTING_COST(12, 550)), nullptr, abc);
  problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 3>(
        new CURVE_FITTING_COST(13, 520)), nullptr, abc);
  problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 3>(
        new CURVE_FITTING_COST(14, 510)), nullptr, abc);
  problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 3>(
        new CURVE_FITTING_COST(15, 500)), nullptr, abc);
  problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 3>(
        new CURVE_FITTING_COST(16, 490)), nullptr, abc);
    problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 3>(
        new CURVE_FITTING_COST(17, 470)), nullptr, abc);
  problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 3>(
        new CURVE_FITTING_COST(18, 450)), nullptr, abc);
  problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 3>(
        new CURVE_FITTING_COST(19, 430)), nullptr, abc);
  problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 3>(
        new CURVE_FITTING_COST(20, 420)), nullptr, abc);
  problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 3>(
        new CURVE_FITTING_COST(21, 400)), nullptr, abc);
  problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 3>(
        new CURVE_FITTING_COST(22, 400)), nullptr, abc);
  problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 3>(
        new CURVE_FITTING_COST(23, 400)), nullptr, abc);
  problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 3>(
        new CURVE_FITTING_COST(24, 400)), nullptr, abc);
  /*第三部分，配置优化器*/
  ceres::Solver::Options options;
  // 增量方程如何求解
  options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
  options.minimizer_progress_to_stdout = true;  //   输出到cout
  ceres::Solver::Summary summary;  //   优化信息

  ceres::Solve(options, &problem, &summary);  // 开始优化

  /*第四部分  输出结果*/
  cout << summary.BriefReport() << endl;
  cout << "estimated a,b,c=";
  for (auto a : abc) cout << a <<" ";
  cout << endl;
}


class LocalCostFunction {
 public:
  explicit LocalCostFunction(
      const std::array<double, 3> &local_landmark_pose)
      : local_landmark_pose_(local_landmark_pose) {}

  template <typename T>
  bool operator()(const T *const start_pose, T *e) const {
    // 信任局部位置，增加权重
    e[0] = (start_pose[0] - local_landmark_pose_[0]) * 1.0;
    e[1] = (start_pose[1] - local_landmark_pose_[1]) * 1.0;
    e[2] = SLAMMath::NormalizePITheta(
            start_pose[2] - local_landmark_pose_[2]) * 1.0;
    return true;
  }

 private:
  // 约束, 图结构的边
  const std::array<double, 3> local_landmark_pose_;
};


class TransCostFunction {
 public:
  explicit TransCostFunction(
      const std::array<double, 2> &delta)
      : delta_(delta) {}

  template <typename T>
  bool operator()(const T *const start_pose,
                  const T *const end_pos, T *e) const {
    T dis =
      SLAMMath::Dist(start_pose[0], start_pose[1], end_pos[0], end_pos[1]);
    T lin_theta;
    if (ceres::abs(end_pos[0] - start_pose[0]) < 0.0000000001) {
      lin_theta = T(M_PI * 0.5);
    } else {
     lin_theta = ceres::atan(
      (end_pos[1] - start_pose[1]) / (end_pos[0] - start_pose[0]));
    }

    e[0] = ceres::abs(ceres::cos(lin_theta) * (delta_[0] - dis));
    e[1] = ceres::abs(ceres::sin(lin_theta) * (delta_[0] - dis));
    e[2] = ceres::abs(
      SLAMMath::NormalizePITheta(delta_[1] - start_pose[2] + end_pos[2]));
    return true;
  }

 private:
  const std::array<double, 2> delta_;
};


// 增加qr约束
TEST(CERES, add_qr_constraint) {
  struct Constraint {
    int last_id;
    int current_id;
    float delta_theta;  // 绝对值
    float delta_dis;   // 绝对值
    Constraint(int last, int current, float theta,
               float dis) {
      last_id = last;
      current_id = current;
      delta_theta = theta;
      delta_dis = dis;
    }
  };

  ceres::Problem problem;
  std::map<int, std::array<double, 3>> poses;
  poses.insert(std::make_pair(0, std::array<double, 3>{0, 0, 0}));
  poses.insert(std::make_pair(1, std::array<double, 3>{1.01, 0, 0}));
  poses.insert(std::make_pair(2, std::array<double, 3>{2, 1, 0.5 * M_PI}));
  poses.insert(std::make_pair(3, std::array<double, 3>
    {2, 2.01, 0.5 * M_PI - 0.01}));
  poses.insert(std::make_pair(4, std::array<double, 3>{2, 2, M_PI}));
  poses.insert(std::make_pair(5, std::array<double, 3>{1, 2, M_PI}));
  poses.insert(std::make_pair(6, std::array<double, 3>{0, 2, M_PI}));
  poses.insert(std::make_pair(7, std::array<double, 3>{0, 1, 1.5 * M_PI}));
  poses.insert(std::make_pair(8, std::array<double, 3>
    {0, 0, 1.5 * M_PI + 0.01}));

  std::map<int, std::array<double, 3>> poses_last = poses;


  std::vector<Constraint> constraints;
  constraints.push_back(Constraint(0, 1, 0.0, 1.0));
  constraints.push_back(Constraint(1, 2, -0.5 * M_PI, std::sqrt(2)));
  constraints.push_back(Constraint(2, 3, 0.0, 1.0));
  constraints.push_back(Constraint(3, 4, -0.5 * M_PI, 0.0));
  constraints.push_back(Constraint(4, 5, 0.0, 1.0));
  constraints.push_back(Constraint(5, 6, 0.0, 1.0));
  constraints.push_back(Constraint(6, 7, -0.5 * M_PI, 1.0));
  constraints.push_back(Constraint(7, 8, 0.0, 1.0));

  // constraints.push_back(Constraint(3, 4, 0.5 * M_PI, 0, Rotate));
  // constraints.push_back(Constraint(5, 6, 0, 1, Translate));
  // constraints.push_back(Constraint(7, 8, 0, 1, Translate));



  for (int i = 0; i < 9; i++) {
    problem.AddParameterBlock(poses.at(i).data(), 3);
    if (i == 0) {
      problem.SetParameterBlockConstant(poses.at(i).data());
    }
  }

  // 1. 建立建图时构建出的约束
  for (int i = 0; i < 9; i++) {
    ceres::CostFunction *cost_function =
        new ceres::AutoDiffCostFunction<LocalCostFunction, 3, 3>(
            new LocalCostFunction(poses.at(i)));
      // 设置残差项
      problem.AddResidualBlock(cost_function, NULL, poses.at(i).data());
  }


  for (auto iter : constraints) {
    // 2. 构建出距离及角度形成节点间的约束
    ceres::CostFunction *cost_function_translate =
        new ceres::AutoDiffCostFunction<TransCostFunction, 3, 3, 3>(
            new TransCostFunction(
              std::array<double, 2>{iter.delta_dis, iter.delta_theta}));
      problem.AddResidualBlock(cost_function_translate, NULL,
                                poses.at(iter.last_id).data(),
                                poses.at(iter.current_id).data());
  }
  // 处理问题
  ceres::Solver::Options options;
  options.max_num_iterations = 200;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  // 输出最后的位姿信息
  for (auto iter : poses) {
    std::cout << iter.first << " " << iter.second[0] << " "
              << iter.second[1] << " " << iter.second[2] << " delta "
              << (poses_last.at(iter.first)[0] - iter.second[0]) << " "
              << (poses_last.at(iter.first)[1] - iter.second[1]) << " "
              << (poses_last.at(iter.first)[2] - iter.second[2]) << std::endl;
  }
}

