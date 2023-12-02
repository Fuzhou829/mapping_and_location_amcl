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
/**************************************************************************
 * Desc: Simple particle filter for localization.
 * Author: Andrew Howard
 * Date: 10 Dec 2002
 * CVS: $Id: pf.h 3293 2005-11-19 08:37:45Z gerkey $
 *************************************************************************/

#pragma once

#include "location/amcl/pf_kdtree.h"
#include "location/amcl/pf_vector.h"
#include <vector>

namespace gomros {
namespace data_process {
namespace mapping_and_location {
namespace amcl {

#ifdef __cplusplus
extern "C" {
#endif

struct PFParam {
  // 一些关于粒子滤波的参数
  int m_MinParticles;  // 允许的粒子数量的最小值，默认100(500)
  int m_MaxParticles;  // 允许的例子数量的最大值，默认5000
  float m_KldErr;  // 真实分布和估计分布之间的最大误差，默认0.01(0.05)
  float m_KldZ;  // 上标准分位数（1-p），
                 // 其中p是估计分布上误差小于kld_err的概率，默认0.99
  float m_DThresh;            // 粒子滤波触发条件1(0.2)
  float m_AThresh;            // 粒子滤波触发条件2(0.5)
  int m_ResampleInterval;     // 重采样间隔(2)
  float m_RecoveryAlphaSlow;  // 慢速的平均权重滤波的指数衰减频率
                              // 用作决定什么时候通过增加随机位姿来recover
                              // 默认0（disable），可能0.001是一个不错的值
  float m_RecoveryAlphaFast;  // 快速的平均权重滤波的指数衰减频率
                              // 用作决定什么时候通过增加随机位姿来recover
                              // 默认0（disable），可能0.1是个不错的值
  float
      m_SavePoseRate;  // 存储上一次估计的位姿和协方差到参数服务器的最大速率。
                       // 被保存的位姿将会用在连续的运动上来初始化滤波器。-1.0失能。(0.5)

  // 激光模型参数
  float m_LaserMinRange;  // 被考虑的最小扫描范围；参数设置为-1.0时
                          // 将会使用激光上报的最小扫描范围
  float m_LaserMaxRange;  // 被考虑的最大扫描范围；参数设置为-1.0时
                          // 将会使用激光上报的最大扫描范围
  float
      m_LaserMaxBeams;  // 更新滤波器时，每次扫描中多少个等间距的光束被使用
                        // 减小计算量，测距扫描中相邻波束往往不是独立的可以减小噪声影响
  float
      m_LaserZHit;  // (0.5)模型的z_hit部分的混合权值，默认0.95
                    // (混合权重1.具有局部测量噪声的正确范围--以测量距离近似真实距离为均值
                    // 其后laser_sigma_hit为标准偏差的高斯分布的权重)
  float
      m_LaserZShort;  // (0.05)模型的z_short部分的混合权值，默认0.1
                      // （混合权重2.意外对象权重
                      // 类似于一元指数关于y轴对称0～测量距离（非最大距离）的部分：
  float m_LaserZMax;  // 0.05模型的z_max部分的混合权值，默认0.05
                      // （混合权重3.测量失败权重（最大距离时为1，其余为0）
  float
      m_LaserZRand;  // 0.5模型的z_rand部分的混合权值，默认0.05
                     // （混合权重4.随机测量权重--均匀分布（1平均分布到0～最大测量范围）
  float m_LaserSigmaHit;  // 0.2被用在模型的z_hit部分的高斯模型的标准差
                          // 默认0.2m
  float
      m_LaserLambdaShort;  // 0.1模型z_short部分的指数衰减参数，默认0.1
                           // （根据ηλe^(-λz)，λ越大随距离增大意外对象概率衰减越快）
  float m_LaserLikelihoodMaxDist;  // 2.0地图上做障碍物膨胀的最大距离
                                   // 用作likelihood_field模型
  int m_LaserModelType;  // 可以是beam, likehood_field, likehood_field_prob
  bool m_DoBeamskip;     // false
  float m_BeamSkipDistance;   // 0.5
  float m_BeamSkipThreshold;  // 0.3

  float m_OdomAlpha1;  // 0.2指定由机器人运动部分的旋转分量
                       // 估计的里程计旋转的期望噪声，默认0.2
  float m_OdomAlpha2;  // 0.2制定由机器人运动部分的平移分量
                       // 估计的里程计旋转的期望噪声，默认0.2
  float m_OdomAlpha3;  // 0.8指定由机器人运动部分的平移分量
                       // 估计的里程计平移的期望噪声，默认0.2
  float m_OdomAlpha4;  // 0.2指定由机器人运动部分的旋转分量
                       // 估计的里程计平移的期望噪声，默认0.2
  float m_OdomAlpha5;  // 0.1平移相关的噪声参数

  PFParam();
};



// Forward declarations
struct _pf_t;
struct _rtk_fig_t;
struct _pf_sample_set_t;

// Function prototype for the initialization model; generates a sample pose from
// an appropriate distribution.
typedef pf_vector_t (*pf_init_model_fn_t)(void *init_data);

// Function prototype for the action model; generates a sample pose from
// an appropriate distribution
typedef void (*pf_action_model_fn_t)(void *action_data,
                                     struct _pf_sample_set_t *set);

// Function prototype for the sensor model; determines the probability
// for the given set of sample poses.
typedef double (*pf_sensor_model_fn_t)(void *sensor_data,
                                       struct _pf_sample_set_t *set);

// Information for a single sample
typedef struct {
  // Pose represented by this sample
  pf_vector_t pose;

  // Weight for this pose
  double weight;
} pf_sample_t;

// Information for a cluster of samples
typedef struct {
  // Number of samples
  int count;

  // Total weight of samples in this cluster
  double weight;

  // Cluster statistics
  pf_vector_t mean;
  pf_matrix_t cov;

  // Workspace
  double m[4], c[2][2];
} pf_cluster_t;

// Information for a set of samples
typedef struct _pf_sample_set_t {
  // The samples
  int sample_count;
  pf_sample_t *samples;

  // A kdtree encoding the histogram
  pf_kdtree_t *kdtree;

  // Clusters
  int cluster_count, cluster_max_count;
  pf_cluster_t *clusters;

  // Filter statistics
  pf_vector_t mean;
  pf_matrix_t cov;
  int converged;
} pf_sample_set_t;

// Information for an entire filter
typedef struct _pf_t {
  // This min and max number of samples
  int min_samples, max_samples;

  // Population size parameters
  double pop_err, pop_z;

  // The sample sets.  We keep two sets and use [current_set]
  // to identify the active set.
  int current_set;
  pf_sample_set_t sets[2];

  // Running averages, slow and fast, of likelihood
  double w_slow, w_fast;

  // Decay rates for running averages
  double alpha_slow, alpha_fast;

  // Function used to draw random pose samples
  pf_init_model_fn_t random_pose_fn;
  void *random_pose_data;

  double dist_threshold;  // distance threshold in each axis over which the pf
                          // is considered to not be converged
  int converged;
} pf_t;

// Create a new filter
pf_t *pf_alloc(int min_samples, int max_samples, double alpha_slow,
               double alpha_fast, pf_init_model_fn_t random_pose_fn,
               void *random_pose_data);

// Free an existing filter
void pf_free(pf_t *pf);

// Initialize the filter using a guassian
void pf_init(pf_t *pf, pf_vector_t mean, pf_matrix_t cov);
void pf_init_landmark(pf_t *pf, double dis_mean, double dis_cov, double pose_mean, double pose_cov, std::vector<pf_vector_t> landmarks);

// Initialize the filter using some model
void pf_init_model(pf_t *pf, pf_init_model_fn_t init_fn, void *init_data);

// Update the filter with some new action
void pf_update_action(pf_t *pf, pf_action_model_fn_t action_fn,
                      void *action_data);

// Update the filter with some new sensor observation
void pf_update_sensor(pf_t *pf, pf_sensor_model_fn_t sensor_fn,
                      void *sensor_data);

// Resample the distribution
void pf_update_resample(pf_t *pf, char update);
// Resample the distribution///插入新粒子到原粒子集中
void pf_insert_sample(pf_t *pf, pf_vector_t meant);
// Compute the CEP statistics (mean and variance).
void pf_get_cep_stats(pf_t *pf, pf_vector_t *mean, double *var);

// Compute the statistics for a particular cluster.  Returns 0 if
// there is no such cluster.
int pf_get_cluster_stats(pf_t *pf, int cluster, double *weight,
                         pf_vector_t *mean, pf_matrix_t *cov);

// Display the sample set
void pf_draw_samples(pf_t *pf, struct _rtk_fig_t *fig, int max_samples);

// Draw the histogram (kdtree)
void pf_draw_hist(pf_t *pf, struct _rtk_fig_t *fig);

// Draw the CEP statistics
void pf_draw_cep_stats(pf_t *pf, struct _rtk_fig_t *fig);

// Draw the cluster statistics
void pf_draw_cluster_stats(pf_t *pf, struct _rtk_fig_t *fig);

// calculate if the particle filter has converged -
// and sets the converged flag in the current set and the pf
int pf_update_converged(pf_t *pf);

// sets the current set and pf converged values to zero
void pf_init_converged(pf_t *pf);
pf_sample_set_t* pf_get_current_set(pf_t *pf);

#ifdef __cplusplus
}
#endif

}  // namespace amcl
}  // namespace mapping_and_location
}  // namespace data_process
}  // namespace gomros

