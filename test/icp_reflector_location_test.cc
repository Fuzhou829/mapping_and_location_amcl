/*
 * @Description: Copyright (C) 2023 山东亚历山大智能科技有限公司
 * @version: 1.2
 * @Author: renjy
 * @Data: 2023-11-07 14:46:41
 * @LastEditors: renjy
 * @LastEditTime: 2023-11-09 10:35:27
 */
#include <gtest/gtest.h>
#include <memory>
#include <string>

#include <pcl/io/ply_io.h>  // NOLINT
#include <pcl/io/pcd_io.h>  // NOLINT
#include <pcl/point_types.h>  // NOLINT
#include <pcl/registration/icp.h>  // NOLINT
#include <pcl/visualization/pcl_visualizer.h>  // NOLINT

#include "mock/sensor_data_simulation.h"
#include "common/load_config.h"

#include "include/config_struct.h"
#include "mapping_and_location/mapping_and_location.h"
#include "mock/display_location_result.h"
#include "landmark_tool/trilateration.h"
#include "common/logger.h"


TEST(Reflector_Location, ICP) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr
    cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr
    cloud_tr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr
    cloud_icp(new pcl::PointCloud<pcl::PointXYZ>);

  cloud_tr->push_back(pcl::PointXYZ(-12.211207, -9.850181, 0.0));
  cloud_tr->push_back(pcl::PointXYZ(0.004008, -10.454499, 0.0));
  cloud_tr->push_back(pcl::PointXYZ(5.978224, -10.748704, 0.0));
  cloud_tr->push_back(pcl::PointXYZ(11.981507, -11.038376, 0.0));
  cloud_tr->push_back(pcl::PointXYZ(17.847918, -4.280269, 0.0));
  cloud_tr->push_back(pcl::PointXYZ(18.205065, 3.702944, 0.0));
  cloud_tr->push_back(pcl::PointXYZ(13.081898, 11.160574, 0.0));
  cloud_tr->push_back(pcl::PointXYZ(7.075381, 11.436640, 0.0));
  cloud_tr->push_back(pcl::PointXYZ(1.087586, 11.728990, 0.0));
  cloud_tr->push_back(pcl::PointXYZ(-4.910215, 12.022557, 0.0));


  cloud_in->push_back(pcl::PointXYZ(-3.917293, -1.173232, 0.0));
  cloud_in->push_back(pcl::PointXYZ(-4.360066, -7.155798, 0.0));
  cloud_in->push_back(pcl::PointXYZ(18.229647, -2.774398, 0.0));
  cloud_in->push_back(pcl::PointXYZ(18.658525, 3.190135, 0.0));
  cloud_in->push_back(pcl::PointXYZ(4.523357, 15.760834, 0.0));
  cloud_in->push_back(pcl::PointXYZ(-3.059587, 10.812913, 0.0));
  cloud_in->push_back(pcl::PointXYZ(-3.482694, 4.820919, 0.0));
  cloud_in->push_back(pcl::PointXYZ(-4.387100, 0.399619, 0.0));


  pcl::io::savePCDFileASCII<pcl::PointXYZ> ("cloud_in.pcd", *cloud_in);
  pcl::io::savePCDFileASCII<pcl::PointXYZ> ("cloud_tr.pcd", *cloud_tr);
  // cloud_in->push_back(pcl::PointXYZ(14.555220, -6.274572, 0.0));
  // cloud_in->push_back(pcl::PointXYZ(14.990592, 5.944454, 0.0));
  // cloud_in->push_back(pcl::PointXYZ(15.202261, 11.922426, 0.0));
  // cloud_in->push_back(pcl::PointXYZ(-7.196738, 6.730520, 0.0));
  // cloud_in->push_back(pcl::PointXYZ(-7.419672, 0.736120, 0.0));

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

  // Set the input source and target
  icp.setInputCloud(cloud_tr);
  icp.setInputTarget(cloud_in);

  //  Set the max correspondence distance to 5cm
  // (e.g., correspondences with higher distances will be ignored)
  // icp.setMaxCorrespondenceDistance(0.02);
  // Set the maximum number of iterations (criterion 1)
  icp.setMaximumIterations(200);
  // Set the transformation epsilon (criterion 2)
  icp.setTransformationEpsilon(1e-8);
  // Set the euclidean distance difference epsilon (criterion 3)
  icp.setEuclideanFitnessEpsilon(0.01);

  // Perform the alignment
  icp.align(*cloud_tr);
  // Obtain the transformation that aligned cloud_source to
  // cloud_source_registered
  Eigen::Matrix4f transformation = icp.getFinalTransformation();

  printf("Rotation transformation :\n");
  printf("    | %6.3f %6.3f %6.3f | \n",
    transformation(0, 0), transformation(0, 1), transformation(0, 2));
  printf("R = | %6.3f %6.3f %6.3f | \n",
    transformation(1, 0), transformation(1, 1), transformation(1, 2));
  printf("    | %6.3f %6.3f %6.3f | \n",
    transformation(2, 0), transformation(2, 1), transformation(2, 2));
  float theta = atan2(transformation(1, 0), transformation(0, 0)) * 180 / M_PI;
  printf(" theta %6.3f\n", theta);
  printf("Translation vector :\n");
  printf("t = < %6.3f, %6.3f, %6.3f >\n\n",
    transformation(0, 3), transformation(1, 3), transformation(2, 3));

  for (auto iter : *cloud_tr) {
    printf("%f %f\n", iter.x, iter.y);
  }
}
