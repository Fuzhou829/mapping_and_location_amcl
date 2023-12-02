/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.0
 * @Date: 2022-10-01 16:11:29
 * @LastEditTime: 2023-10-23 22:16:34
 */
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
 * CVS: $Id: pf.c 6345 2008-04-17 01:36:39Z gerkey $
 *************************************************************************/
#include "location/amcl/pf.h"

#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>

#include <stdio.h>
#include "common/logger.h"
#include "location/amcl/pf_kdtree.h"
#include "location/amcl/pf_pdf.h"

namespace gomros {
namespace data_process {
namespace mapping_and_location {
namespace amcl {

PFParam::PFParam() {
  m_MinParticles = 500;   // 允许的粒子数量的最小值，默认100(500)
  m_MaxParticles = 3000;  // 允许的例子数量的最大值，默认5000
  // m_MaxParticles = 2500;
  m_KldErr = 0.009;  // 真实分布和估计分布之间的最大误差，默认0.01(0.05)
  // 上标准分位数（1-p），其中p是估计分布上误差小于kld_err的概率，默认0.99
  m_KldZ = 0.99;
  m_DThresh = 0.1;         // 粒子滤波触发条件1(0.2)
  m_AThresh = 0.1;         // 粒子滤波触发条件2(0.5)
  m_ResampleInterval = 2;  // 重采样间隔(2)
  // 慢速的平均权重滤波的指数衰减频率，用作决定什么时候通过增加随机位姿来recover
  // 默认0（disable），可能0.001是一个不错的值
  m_RecoveryAlphaSlow = 0.001;
  // 快速的平均权重滤波的指数衰减频率，用作决定什么时候通过增加随机位姿来recover
  // 默认0（disable），可能0.1是个不错的值
  m_RecoveryAlphaFast = 0.1;
  // 存储上一次估计的位姿和协方差到参数服务器的最大速率
  // 被保存的位姿将会用在连续的运动上来初始化滤波器。-1.0失能。(0.5)
  m_SavePoseRate = 2;

  // 激光模型参数
  // 被考虑的最小扫描范围；参数设置为-1.0时，将会使用激光上报的最小扫描范围
  m_LaserMinRange = 0.5;
  // 被考虑的最大扫描范围；参数设置为-1.0时，将会使用激光上报的最大扫描范围
  m_LaserMaxRange = 30.0;
  // get_configs()->get_float("radar", "laser_max_range", nullptr);
  if (m_LaserMaxRange < 0) m_LaserMaxRange = 19.0;
  // 更新滤波器时，每次扫描中多少个等间距的光束被使用
  // 减小计算量，测距扫描中相邻波束往往不是独立的可以减小噪声影响，太小也会造成信息量少定位不准
  // m_LaserMaxBeams = 50;
  m_LaserMaxBeams = 90;
  // (0.5)模型的z_hit部分的混合权值，默认0.95
  // 混合权重1.具有局部测量噪声的正确范围--以测量距离近似真实距离为均值，
  // 其后laser_sigma_hit为标准偏差的高斯分布的权重
  m_LaserZHit = 0.95;
  // (0.05)模型的z_short部分的混合权值
  // 默认0.1（混合权重2.意外对象权重（类似于一元指数关于y轴对称0～测量距离（非最大距离）的部分
  m_LaserZShort = 0.02;
  // 0.05模型的z_max部分的混合权值，默认0.05（混合权重3.测量失败权重（最大距离时为1，其余为0）
  m_LaserZMax = 0.002;
  m_LaserZRand = 0.05;  // 0.5模型的z_rand部分的混合权值
                        // 默认0.05（混合权重4.随机测量权重
  // 均匀分布（1平均分布到0～最大测量范围）
  m_LaserSigmaHit = 0.05;  // 0.2被用在模型的z_hit部分的高斯模型的标准差
                           // 默认0.2m
  m_LaserLambdaShort = 0.5;  // 0.1模型z_short部分的指数衰减参数
                             // 默认0.1（根据ηλe^(-λz)，λ越大随距离增大意外对象概率衰减越快）
  m_LaserLikelihoodMaxDist = 0.3;  // 2.0地图上做障碍物膨胀的最大距离
                                   // 用作likelihood_field模型
  m_LaserModelType = 1;            // 可以是beam
                                   // likehood_field,
                                   // likehood_field_prob
  m_DoBeamskip = false;            // false
  m_BeamSkipDistance = 0.5;        // 0.5
  m_BeamSkipThreshold = 0.3;       // 0.3
  // 里程计模型参数
  // odom_model_t odom_model_type;
  // 可以是"diff", "omni", "diff-corrected",
  // "omni-corrected",后面两
  m_OdomAlpha1 = 0.2;  // 0.2指定由机器人运动部分的旋转分量
                       // 估计的里程计旋转的期望噪声,默认0.2
  m_OdomAlpha2 = 0.2;  // 默认0.2
  m_OdomAlpha3 = 0.1;  // 默认0.2
  m_OdomAlpha4 = 0.1;  // 默认0.2
  m_OdomAlpha5 = 0.1;  // 0.1平移相关的噪声参数
}

// Compute the required number of samples, given that there are k bins
// with samples in them.
static int pf_resample_limit(pf_t *pf, int k);

// Re-compute the cluster statistics for a sample set
static void pf_cluster_stats(pf_t *pf, pf_sample_set_t *set);

typedef struct {
  // Total weight (weights sum to 1)
  double weight;
  double score;
  // Mean of pose esimate
  pf_vector_t pf_pose_mean;

  // Covariance of pose estimate
  pf_matrix_t pf_pose_cov;

} amcl_hyp_t;
// Create a new filter
pf_t *pf_alloc(int min_samples, int max_samples, double alpha_slow,
               double alpha_fast, pf_init_model_fn_t random_pose_fn,
               void *random_pose_data) {
  int i, j;
  pf_t *pf;
  pf_sample_set_t *set;
  pf_sample_t *sample;

  srand48(time(NULL));

  pf = (pf_t *)calloc(1, sizeof(pf_t));

  pf->random_pose_fn = random_pose_fn;
  pf->random_pose_data = random_pose_data;

  pf->min_samples = min_samples;
  pf->max_samples = max_samples;

  // Control parameters for the population size calculation.  [err] is
  // the max error between the true distribution and the estimated
  // distribution.  [z] is the upper standard normal quantile for (1 -
  // p), where p is the probability that the error on the estimated
  // distrubition will be less than [err].
  pf->pop_err = 0.01;
  pf->pop_z = 3;
  pf->dist_threshold = 0.5;

  pf->current_set = 0;
  for (j = 0; j < 2; j++) {
    set = pf->sets + j;

    set->sample_count = max_samples;
    set->samples = (pf_sample_t *)calloc(max_samples, sizeof(pf_sample_t));

    for (i = 0; i < set->sample_count; i++) {
      sample = set->samples + i;
      sample->pose.v[0] = 0.0;
      sample->pose.v[1] = 0.0;
      sample->pose.v[2] = 0.0;
      sample->weight = 1.0 / max_samples;
    }

    // HACK: is 3 times max_samples enough?
    set->kdtree = pf_kdtree_alloc(3 * max_samples);

    set->cluster_count = 0;
    set->cluster_max_count = max_samples;
    set->clusters =
        (pf_cluster_t *)calloc(set->cluster_max_count, sizeof(pf_cluster_t));

    set->mean = pf_vector_zero();
    set->cov = pf_matrix_zero();
  }

  pf->w_slow = 0.0;
  pf->w_fast = 0.0;

  pf->alpha_slow = alpha_slow;
  pf->alpha_fast = alpha_fast;

  // set converged to 0
  pf_init_converged(pf);

  return pf;
}

// Free an existing filter
void pf_free(pf_t *pf) {
  int i;

  for (i = 0; i < 2; i++) {
    free(pf->sets[i].clusters);
    pf_kdtree_free(pf->sets[i].kdtree);
    free(pf->sets[i].samples);
  }
  free(pf);

  return;
}

// Initialize the filter using a guassian
void pf_init(pf_t *pf, pf_vector_t mean, pf_matrix_t cov) {
  int i;
  pf_sample_set_t *set;
  pf_sample_t *sample;
  pf_pdf_gaussian_t *pdf;

  set = pf->sets + pf->current_set;

  // Create the kd tree for adaptive sampling
  pf_kdtree_clear(set->kdtree);

  set->sample_count = pf->max_samples;

  pdf = pf_pdf_gaussian_alloc(mean, cov);

  // Compute the new sample poses
  for (i = 0; i < set->sample_count; i++) {
    sample = set->samples + i;
    sample->weight = 1.0 / pf->max_samples;
    sample->pose = pf_pdf_gaussian_sample(pdf);

    // Add sample to histogram
    pf_kdtree_insert(set->kdtree, sample->pose, sample->weight);
  }

  pf->w_slow = pf->w_fast = 0.0;

  pf_pdf_gaussian_free(pdf);

  // Re-compute cluster statistics
  pf_cluster_stats(pf, set);

  // set converged to 0
  pf_init_converged(pf);

  return;
}

void pf_init_landmark(pf_t *pf, double dis_mean, double dis_cov,
                      double pose_mean, double pose_cov,
                      std::vector<pf_vector_t> landmarks) {
  SLAM_WARN("进入pf_init_landmark!!!!!!!!!!!!!\n");
  // pf_ran_gaussian
  pf_sample_set_t *set;
  pf_sample_t *sample;
  set = pf->sets + pf->current_set;

  // Create the kd tree for adaptive sampling
  pf_kdtree_clear(set->kdtree);

  set->sample_count = pf->max_samples;
  int landmark_count = landmarks.size();

  for (int i = 0; i < set->sample_count; i++) {
    double distance = dis_mean + pf_ran_gaussian(dis_cov);
    double pose = pose_mean + pf_ran_gaussian(pose_cov);
    double dir = 2 * M_PI * drand48() - M_PI;
    sample = set->samples + i;
    int index = (int)(drand48() * landmark_count);
    if (index == landmark_count) {
      i--;
      continue;
    }
    pf_vector_t mean = landmarks[index];
    sample->weight = 1.0 / pf->max_samples;
    double x = mean.v[0] + distance * cos(dir);
    double y = mean.v[1] + distance * sin(dir);
    double theta = dir + M_PI - pose;
    sample->pose.v[0] = x;
    sample->pose.v[1] = y;
    sample->pose.v[2] = theta;
    // SLAM_DEBUG("粒子x %f      y %f      theta %f  pose_mean %f pose %f\n", x,
    // y, theta, pose_mean, pose);
    pf_kdtree_insert(set->kdtree, sample->pose, sample->weight);
  }

  pf->w_slow = pf->w_fast = 0.0;
  // pf_pdf_gaussian_free(pdf);

  // Re-compute cluster statistics
  pf_cluster_stats(pf, set);

  // set converged to 0
  pf_init_converged(pf);
  SLAM_WARN("退出pf_init_landmark    %d!!!!!!!!!!!!!\n", set->sample_count);
}

// Initialize the filter using some model
void pf_init_model(pf_t *pf, pf_init_model_fn_t init_fn, void *init_data) {
  int i;
  pf_sample_set_t *set;
  pf_sample_t *sample;

  set = pf->sets + pf->current_set;

  // Create the kd tree for adaptive sampling
  pf_kdtree_clear(set->kdtree);

  set->sample_count = pf->max_samples;

  // Compute the new sample poses
  for (i = 0; i < set->sample_count; i++) {
    sample = set->samples + i;
    sample->weight = 1.0 / pf->max_samples;
    sample->pose = (*init_fn)(init_data);

    // Add sample to histogram
    pf_kdtree_insert(set->kdtree, sample->pose, sample->weight);
  }

  pf->w_slow = pf->w_fast = 0.0;

  // Re-compute cluster statistics
  pf_cluster_stats(pf, set);

  // set converged to 0
  pf_init_converged(pf);

  return;
}

void pf_init_converged(pf_t *pf) {
  pf_sample_set_t *set;
  set = pf->sets + pf->current_set;
  set->converged = 0;
  pf->converged = 0;
}

int pf_update_converged(pf_t *pf) {
  int i;
  pf_sample_set_t *set;
  pf_sample_t *sample;
  double total;

  set = pf->sets + pf->current_set;
  double mean_x = 0, mean_y = 0;

  for (i = 0; i < set->sample_count; i++) {
    sample = set->samples + i;

    mean_x += sample->pose.v[0];
    mean_y += sample->pose.v[1];
  }
  mean_x /= set->sample_count;
  mean_y /= set->sample_count;

  for (i = 0; i < set->sample_count; i++) {
    sample = set->samples + i;
    if (fabs(sample->pose.v[0] - mean_x) > pf->dist_threshold ||
        fabs(sample->pose.v[1] - mean_y) > pf->dist_threshold) {
      set->converged = 0;
      pf->converged = 0;
      return 0;
    }
  }
  set->converged = 1;
  pf->converged = 1;
  return 1;
}

// Update the filter with some new action
void pf_update_action(pf_t *pf, pf_action_model_fn_t action_fn,
                      void *action_data) {
  pf_sample_set_t *set;

  set = pf->sets + pf->current_set;

  (*action_fn)(action_data, set);

  return;
}

#include <float.h>
// Update the filter with some new sensor observation
void pf_update_sensor(pf_t *pf, pf_sensor_model_fn_t sensor_fn,
                      void *sensor_data) {
  int i;
  pf_sample_set_t *set;
  pf_sample_t *sample;
  double total;

  set = pf->sets + pf->current_set;

  // Compute the sample weights
  total = (*sensor_fn)(sensor_data, set);
  //  my_real_time_data.score = total;
  // printf("当前粒子的置信度=%f",my_real_time_data.score);
  if (total > 0.0) {
    // Normalize weights
    double w_avg = 0.0;
    for (i = 0; i < set->sample_count; i++) {
      sample = set->samples + i;
      w_avg += sample->weight;
      sample->weight /= total;
    }
    // Update running averages of likelihood of samples (Prob Rob p258)
    w_avg /= set->sample_count;
    if (pf->w_slow == 0.0)
      pf->w_slow = w_avg;
    else
      pf->w_slow += pf->alpha_slow * (w_avg - pf->w_slow);
    if (pf->w_fast == 0.0)
      pf->w_fast = w_avg;
    else
      pf->w_fast += pf->alpha_fast * (w_avg - pf->w_fast);
    // printf("w_avg: %e slow: %e fast: %e------alpha_fast=%f--alpha_slow=%f\n",
    //      w_avg, pf->w_slow, pf->w_fast,pf->alpha_fast,pf->alpha_slow);
  } else {
    // Handle zero total
    for (i = 0; i < set->sample_count; i++) {
      sample = set->samples + i;
      sample->weight = 1.0 / set->sample_count;
    }
  }
  // printf("当前粒子total=%f的amcl_laser_prop=%f\n",total,my_real_time_data.amcl_laser_prop);
  // if(my_real_time_data.amcl_laser_prop>1.0) my_real_time_data.amcl_laser_prop
  // =0.99;
  // printf("当前粒子total=%f的amcl_laser_prop=%f\n",total,my_real_time_data.amcl_laser_prop);
  return;
}
void pf_insert_sample(pf_t *pf, pf_vector_t meant) {
  int i;
  double total;
  pf_sample_set_t *set_a, *set_b;
  pf_sample_t *sample_a, *sample_b;

  // double r,c,U;
  // int m;
  // double count_inv;
  double *c;

  double w_diff = 0.05;
  double max_weight = 0;
  int max_weight_index = 0;
  // double theta_robot = my_real_time_data.base_map.v[2];
  pf_pdf_gaussian_t *pdf;

  set_a = pf->sets + pf->current_set;
  set_b = pf->sets + (pf->current_set + 1) % 2;

  // Build up cumulative probability table for resampling.
  // TODO: Replace this with a more efficient procedure
  // (e.g.,
  // http://www.network-theory.co.uk/docs/gslref/GeneralDiscreteDistributions.html)
  c = (double *)malloc(sizeof(double) * (set_a->sample_count + 1));
  c[0] = 0.0;
  for (i = 0; i < set_a->sample_count; i++) {
    c[i + 1] = c[i] + set_a->samples[i].weight;
  }

  pf_matrix_t pf_init_pose_cov = pf_matrix_zero();
  pf_init_pose_cov.m[0][0] = 0.07;
  pf_init_pose_cov.m[1][1] = 0.07;
  pf_init_pose_cov.m[2][2] = 0.02;

  pf_vector_t mean = meant;
  pdf = pf_pdf_gaussian_alloc(mean, pf_init_pose_cov);
  // Create the kd tree for adaptive sampling
  pf_kdtree_clear(set_b->kdtree);

  // Draw samples from set a to create set b.
  total = 0;
  set_b->sample_count = 0;
  // if(set_a->sample_count<= (pf->min_samples*2)&&update)

  while (set_b->sample_count < pf->max_samples) {
    sample_b = set_b->samples + set_b->sample_count++;

    if (drand48() < w_diff) sample_b->pose = pf_pdf_gaussian_sample(pdf);
    // sample_b->pose = (pf->random_pose_fn)(pf->random_pose_data);
    else {
      double r;
      r = drand48();
      for (i = 0; i < set_a->sample_count; i++) {
        if ((c[i] <= r) && (r < c[i + 1])) break;
      }
      // assert(i<set_a->sample_count);
      if (i < set_a->sample_count) sample_a = set_a->samples + i;

      // assert(sample_a->weight > 0);
      if (sample_a->weight > 0)
        // Add sample to list
        sample_b->pose = sample_a->pose;
    }

    sample_b->weight = 1.0;
    total += sample_b->weight;

    // Add sample to histogram
    pf_kdtree_insert(set_b->kdtree, sample_b->pose, sample_b->weight);

    // See if we have enough samples yet
    if (set_b->sample_count > pf_resample_limit(pf, set_b->kdtree->leaf_count))
      break;
  }

  // Normalize weights
  for (i = 0; i < set_b->sample_count; i++) {
    sample_b = set_b->samples + i;
    sample_b->weight /= total;
  }

  // Re-compute cluster statistics
  pf_cluster_stats(pf, set_b);

  // Use the newly created sample set
  pf->current_set = (pf->current_set + 1) % 2;

  pf_update_converged(pf);

  free(c);
  return;
}

// Resample the distribution
void pf_update_resample(pf_t *pf, char update) {
  int i;
  double total;
  pf_sample_set_t *set_a, *set_b;
  pf_sample_t *sample_a, *sample_b;

  // double r,c,U;
  // int m;
  // double count_inv;
  double *c;

  double w_diff;
  double max_weight = 0;
  int max_weight_index = 0;
  // double theta_robot = my_real_time_data.base_map.v[2];
  pf_pdf_gaussian_t *pdf;

  set_a = pf->sets + pf->current_set;
  set_b = pf->sets + (pf->current_set + 1) % 2;

  // Build up cumulative probability table for resampling.
  // TODO: Replace this with a more efficient procedure
  // (e.g.,
  // http://www.network-theory.co.uk/docs/gslref/GeneralDiscreteDistributions.html)
  c = (double *)malloc(sizeof(double) * (set_a->sample_count + 1));
  c[0] = 0.0;
  for (i = 0; i < set_a->sample_count; i++) {
    c[i + 1] = c[i] + set_a->samples[i].weight;
    if (max_weight < set_a->samples[i].weight) {
      max_weight = set_a->samples[i].weight;
      max_weight_index = i;
    }
  }
  /*    pf_vector_t pose_mean;
      pf_matrix_t pose_cov;
      int hyp_count = 0;
      int max_weight_hyp = -1;
      double weight;
      std::vector <amcl_hyp_t> hyps;
          hyps.resize(pf->sets[pf->current_set].cluster_count);
          for(int hyp_count = 0;
          hyp_count < pf->sets[pf->current_set].cluster_count; hyp_count++)
          {

            pf_vector_t pose_mean;
            pf_matrix_t pose_cov;
            if (!pf_get_cluster_stats(pf, hyp_count, &weight, &pose_mean,
     &pose_cov))
            {
          break;
            }
            hyps[hyp_count].weight = weight;
            hyps[hyp_count].pf_pose_mean = pose_mean;
            hyps[hyp_count].pf_pose_cov = pose_cov;

            if(hyps[hyp_count].weight > max_weight)
            {
          max_weight = hyps[hyp_count].weight;
          max_weight_hyp = hyp_count;
            }
          }*/
  float delta_covx;  // fabs(my_real_time_data.robot_line_vel)*0.16;
  float delta_covy;  // fabs(my_real_time_data.robot_line_vel)*0.16;
                     // if (delta_cov<0.05) delta_cov = 0.05;
  // if(fabs(my_real_time_data.robot_line_vel)>0.85)
  //  delta_covx = 0.05;
  // else
  delta_covx = 0.1;
  delta_covy = 0.1;
  sample_a = set_a->samples + max_weight_index;
  // printf("权重%f粒子%f,%f,%f\n",max_weight,sample_a->pose.v[0],sample_a->pose.v[1],sample_a->pose.v[2]);
  pf_matrix_t pf_init_pose_cov = pf_matrix_zero();
  pf_init_pose_cov.m[0][0] = 0.15;
  pf_init_pose_cov.m[1][1] = 0.15;
  pf_init_pose_cov.m[2][2] = 0.03;

  if (update == 1) {
    pf_init_pose_cov.m[0][0] =
        delta_covx;  // delta_covx*cos(theta_robot)+delta_covy*sin(theta_robot);////0.15
    pf_init_pose_cov.m[1][1] =
        delta_covy;  // delta_covx*sin(theta_robot)+delta_covy*cos(theta_robot);////0.15
    pf_init_pose_cov.m[2][2] = 0.02;
  } else if (update == 2) {
    pf_init_pose_cov.m[0][0] = 0.05;  /// 0.03
    pf_init_pose_cov.m[1][1] = 0.05;
    pf_init_pose_cov.m[2][2] = 0.01;
  } else {
    pf_init_pose_cov.m[0][0] = 0.1;
    pf_init_pose_cov.m[1][1] = 0.1;
    pf_init_pose_cov.m[2][2] = 0.07;
    // pf_init_pose_cov.m[0][0] = 0.07;
    // pf_init_pose_cov.m[1][1] = 0.07;
    // pf_init_pose_cov.m[2][2] = 0.02;
  }

  // if(update == 1)
  //  w_diff = 0.3;
  // else if(update == 2)
  //  w_diff = 0.1;
  // else
  //  w_diff = 0.0;
  w_diff = 1.0 - pf->w_fast / pf->w_slow;
  // if (w_diff > 0 && w_diff < 0.1) {
  //   printf("!!!!!!w_diff=%f", w_diff);
  //   w_diff = 0.1;
  // }
  // if(update == 5)
  //    w_diff = 0.5;
  pf_vector_t mean = sample_a->pose;
  pdf = pf_pdf_gaussian_alloc(mean, pf_init_pose_cov);
  // Create the kd tree for adaptive sampling
  pf_kdtree_clear(set_b->kdtree);

  // Draw samples from set a to create set b.
  total = 0;
  set_b->sample_count = 0;
  // if(set_a->sample_count<= (pf->min_samples*2)&&update)

  if (w_diff < 0.0) w_diff = 0.00;
  // printf("w_diff: %9.6faaa-sample_count=%d,delta_covx=%f,delta_covy=%f\n",
  // w_diff,set_a->sample_count,delta_covx,delta_covy);

  // Can't (easily) combine low-variance sampler with KLD adaptive
  // sampling, so we'll take the more traditional route.
  /*
  // Low-variance resampler, taken from Probabilistic Robotics, p110
  count_inv = 1.0/set_a->sample_count;
  r = drand48() * count_inv;
  c = set_a->samples[0].weight;
  i = 0;
  m = 0;
  */
  //    printf("重采样过程函数运行...............\n");
  while (set_b->sample_count < pf->max_samples) {
    sample_b = set_b->samples + set_b->sample_count++;

    if (drand48() < w_diff) sample_b->pose = pf_pdf_gaussian_sample(pdf);
    // sample_b->pose = (pf->random_pose_fn)(pf->random_pose_data);
    else {
      // Can't (easily) combine low-variance sampler with KLD adaptive
      // sampling, so we'll take the more traditional route.
      /*
      // Low-variance resampler, taken from Probabilistic Robotics, p110
      U = r + m * count_inv;
      while(U>c)
      {
        i++;
        // Handle wrap-around by resetting counters and picking a new random
        // number
        if(i >= set_a->sample_count)
        {
          r = drand48() * count_inv;
          c = set_a->samples[0].weight;
          i = 0;
          m = 0;
          U = r + m * count_inv;
          continue;
        }
        c += set_a->samples[i].weight;
      }
      m++;
      */

      // Naive discrete event sampler
      double r;
      r = drand48();
      for (i = 0; i < set_a->sample_count; i++) {
        if ((c[i] <= r) && (r < c[i + 1])) break;
      }
      // assert(i<set_a->sample_count);
      if (i < set_a->sample_count) sample_a = set_a->samples + i;

      // assert(sample_a->weight > 0);
      if (sample_a->weight > 0)
        // Add sample to list
        sample_b->pose = sample_a->pose;
    }

    sample_b->weight = 1.0;
    total += sample_b->weight;

    // Add sample to histogram
    pf_kdtree_insert(set_b->kdtree, sample_b->pose, sample_b->weight);

    // See if we have enough samples yet
    if (set_b->sample_count > pf_resample_limit(pf, set_b->kdtree->leaf_count))
      break;
  }

  // Reset averages, to avoid spiraling off into complete randomness.
  if (w_diff > 0.0) {
    pf->w_slow = 0.0;
    pf->w_fast = 0.0;
    // printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
  }

  // fprintf(stderr, "\n\n");

  // Normalize weights
  for (i = 0; i < set_b->sample_count; i++) {
    sample_b = set_b->samples + i;
    sample_b->weight /= total;
  }

  // Re-compute cluster statistics
  pf_cluster_stats(pf, set_b);

  // Use the newly created sample set
  pf->current_set = (pf->current_set + 1) % 2;

  pf_update_converged(pf);

  free(c);
  return;
}

// Compute the required number of samples, given that there are k bins
// with samples in them.  This is taken directly from Fox et al.
int pf_resample_limit(pf_t *pf, int k) {
  double a, b, c, x;
  int n;

  if (k <= 1) return pf->max_samples;

  a = 1;
  b = 2 / (9 * ((double)k - 1));
  c = sqrt(2 / (9 * ((double)k - 1))) * pf->pop_z;
  x = a - b + c;

  n = (int)ceil((k - 1) / (2 * pf->pop_err) * x * x * x);

  if (n < pf->min_samples) return pf->min_samples;
  if (n > pf->max_samples) return pf->max_samples;

  return n;
}

// Re-compute the cluster statistics for a sample set
void pf_cluster_stats(pf_t *pf, pf_sample_set_t *set) {
  int i, j, k, cidx;
  pf_sample_t *sample;
  pf_cluster_t *cluster;

  // Workspace
  double m[4], c[2][2];
  size_t count;
  double weight;

  // Cluster the samples
  pf_kdtree_cluster(set->kdtree);

  // Initialize cluster stats
  set->cluster_count = 0;

  for (i = 0; i < set->cluster_max_count; i++) {
    cluster = set->clusters + i;
    cluster->count = 0;
    cluster->weight = 0;
    cluster->mean = pf_vector_zero();
    cluster->cov = pf_matrix_zero();

    for (j = 0; j < 4; j++) cluster->m[j] = 0.0;
    for (j = 0; j < 2; j++)
      for (k = 0; k < 2; k++) cluster->c[j][k] = 0.0;
  }

  // Initialize overall filter stats
  count = 0;
  weight = 0.0;
  set->mean = pf_vector_zero();
  set->cov = pf_matrix_zero();
  for (j = 0; j < 4; j++) m[j] = 0.0;
  for (j = 0; j < 2; j++)
    for (k = 0; k < 2; k++) c[j][k] = 0.0;

  // Compute cluster stats
  for (i = 0; i < set->sample_count; i++) {
    sample = set->samples + i;

    // printf("%d %f %f %f\n", i, sample->pose.v[0], sample->pose.v[1],
    // sample->pose.v[2]);

    // Get the cluster label for this sample
    cidx = pf_kdtree_get_cluster(set->kdtree, sample->pose);
    assert(cidx >= 0);
    if (cidx >= set->cluster_max_count) continue;
    if (cidx + 1 > set->cluster_count) set->cluster_count = cidx + 1;

    cluster = set->clusters + cidx;

    cluster->count += 1;
    cluster->weight += sample->weight;

    count += 1;
    weight += sample->weight;

    // Compute mean
    cluster->m[0] += sample->weight * sample->pose.v[0];
    cluster->m[1] += sample->weight * sample->pose.v[1];
    cluster->m[2] += sample->weight * cos(sample->pose.v[2]);
    cluster->m[3] += sample->weight * sin(sample->pose.v[2]);

    m[0] += sample->weight * sample->pose.v[0];
    m[1] += sample->weight * sample->pose.v[1];
    m[2] += sample->weight * cos(sample->pose.v[2]);
    m[3] += sample->weight * sin(sample->pose.v[2]);

    // Compute covariance in linear components
    for (j = 0; j < 2; j++)
      for (k = 0; k < 2; k++) {
        cluster->c[j][k] +=
            sample->weight * sample->pose.v[j] * sample->pose.v[k];
        c[j][k] += sample->weight * sample->pose.v[j] * sample->pose.v[k];
      }
  }

  // Normalize
  for (i = 0; i < set->cluster_count; i++) {
    cluster = set->clusters + i;

    cluster->mean.v[0] = cluster->m[0] / cluster->weight;
    cluster->mean.v[1] = cluster->m[1] / cluster->weight;
    cluster->mean.v[2] = atan2(cluster->m[3], cluster->m[2]);

    cluster->cov = pf_matrix_zero();

    // Covariance in linear components
    for (j = 0; j < 2; j++)
      for (k = 0; k < 2; k++)
        cluster->cov.m[j][k] = cluster->c[j][k] / cluster->weight -
                               cluster->mean.v[j] * cluster->mean.v[k];

    // Covariance in angular components; I think this is the correct
    // formula for circular statistics.
    cluster->cov.m[2][2] = -2 * log(sqrt(cluster->m[2] * cluster->m[2] +
                                         cluster->m[3] * cluster->m[3]));

    // printf("cluster %d %d %f (%f %f %f)\n", i, cluster->count,
    // cluster->weight, cluster->mean.v[0], cluster->mean.v[1],
    // cluster->mean.v[2]);
    // pf_matrix_fprintf(cluster->cov, stdout, "%e");
  }

  // Compute overall filter stats
  set->mean.v[0] = m[0] / weight;
  set->mean.v[1] = m[1] / weight;
  set->mean.v[2] = atan2(m[3], m[2]);

  // Covariance in linear components
  for (j = 0; j < 2; j++)
    for (k = 0; k < 2; k++)
      set->cov.m[j][k] = c[j][k] / weight - set->mean.v[j] * set->mean.v[k];

  // Covariance in angular components; I think this is the correct
  // formula for circular statistics.
  set->cov.m[2][2] = -2 * log(sqrt(m[2] * m[2] + m[3] * m[3]));

  return;
}

// Compute the CEP statistics (mean and variance).
void pf_get_cep_stats(pf_t *pf, pf_vector_t *mean, double *var) {
  int i;
  double mn, mx, my, mrr;
  pf_sample_set_t *set;
  pf_sample_t *sample;

  set = pf->sets + pf->current_set;

  mn = 0.0;
  mx = 0.0;
  my = 0.0;
  mrr = 0.0;

  for (i = 0; i < set->sample_count; i++) {
    sample = set->samples + i;

    mn += sample->weight;
    mx += sample->weight * sample->pose.v[0];
    my += sample->weight * sample->pose.v[1];
    mrr += sample->weight * sample->pose.v[0] * sample->pose.v[0];
    mrr += sample->weight * sample->pose.v[1] * sample->pose.v[1];
  }

  mean->v[0] = mx / mn;
  mean->v[1] = my / mn;
  mean->v[2] = 0.0;

  *var = mrr / mn - (mx * mx / (mn * mn) + my * my / (mn * mn));

  return;
}

// Get the statistics for a particular cluster.
int pf_get_cluster_stats(pf_t *pf, int clabel, double *weight,
                         pf_vector_t *mean, pf_matrix_t *cov) {
  pf_sample_set_t *set;
  pf_cluster_t *cluster;

  set = pf->sets + pf->current_set;

  if (clabel >= set->cluster_count) return 0;
  cluster = set->clusters + clabel;

  *weight = cluster->weight;
  *mean = cluster->mean;
  *cov = cluster->cov;

  return 1;
}

pf_sample_set_t *pf_get_current_set(pf_t *pf) {
  return pf->sets + pf->current_set;
}

}  // namespace amcl
}  // namespace mapping_and_location
}  // namespace data_process
}  // namespace gomros