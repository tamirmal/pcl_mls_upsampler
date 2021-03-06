/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: test_surface.cpp 6579 2012-07-27 18:57:32Z rusu $
 *
 */

//#include <gtest/gtest.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/impl/normal_3d.hpp>
#include <pcl/surface/mls.h>
#include <pcl/common/common.h>
#include <pcl/search/search.h>
#include <pcl/filters/crop_box.h>

#include <pcl/surface/impl/mls.hpp>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/search/impl/organized.hpp>
#include <pcl/search/impl/flann_search.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/filters/impl/crop_box.hpp>

#include <string>
using std::string;

using namespace pcl;
using namespace pcl::io;
using namespace std;


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/* ---[ */
int
main (int argc, char** argv)
{
  if (argc < 2)
  {
    std::cerr << "No test file given." << std::endl;
    return (-1);
  }

  if (argc < 3)
  {
    std::cerr << "No output file given" << std::endl;
    return (-1);
  }

  if (0 == (strcmp(argv[1], argv[2])))
  {
    std:cerr << "input==output ! give different file";
    return (-1);
  }

  // Load file
  PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);
  loadPCDFile (argv[1], *cloud);

  cloud->is_dense = true;

  printf("input cloud : %d points\n", cloud->size());

  // Create search tree
  search::KdTree<PointXYZ>::Ptr tree;
  tree.reset (new search::KdTree<PointXYZ> (false));
  tree->setInputCloud (cloud);

  // Testing upsampling
  MovingLeastSquares<PointXYZ, PointNormal> mls_upsampling;
  // Set parameters
  mls_upsampling.setInputCloud (cloud);
  mls_upsampling.setComputeNormals (true);
  mls_upsampling.setPolynomialFit (true);
  mls_upsampling.setSearchMethod (tree);
  mls_upsampling.setSearchRadius (0.03);
  //mls_upsampling.setUpsamplingMethod (MovingLeastSquares<PointXYZ, PointNormal>::SAMPLE_LOCAL_PLANE);
  mls_upsampling.setUpsamplingMethod (MovingLeastSquares<PointXYZ, PointNormal>::DISTINCT_CLOUD);
  mls_upsampling.setUpsamplingRadius (0.005);
  mls_upsampling.setUpsamplingStepSize (0.005);
  mls_upsampling.setPolynomialOrder(2);

  PointCloud<PointNormal>::Ptr cloud_out(new PointCloud<PointNormal>);
  cloud_out->clear();
  mls_upsampling.process (*cloud_out);

  printf("after mls : %d points\n", cloud_out->size());

  pcl::CropBox<pcl::PointNormal> boxFilter;
  boxFilter.setMin(Eigen::Vector4f(-1, -1, -1, 1.0));
  boxFilter.setMax(Eigen::Vector4f(1, 1, 1, 1.0));
  boxFilter.setInputCloud(cloud_out);
  //boxFilter.setNegative(false);

  PointCloud<PointNormal>::Ptr cloud_out_boxed(new PointCloud<PointNormal>);
  cloud_out_boxed->clear();
  boxFilter.filter(*cloud_out_boxed);

  printf("after box : %d points\n", cloud_out_boxed->size());

  pcl::io::savePCDFile (argv[2], *cloud_out_boxed);
}
/* ]--- */