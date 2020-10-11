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

#include <pcl/surface/impl/mls.hpp>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/search/impl/organized.hpp>
#include <pcl/search/impl/flann_search.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>

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

  PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);
  search::KdTree<PointXYZ>::Ptr tree;

  // Load file
  pcl::PCLPointCloud2 cloud_blob;
  loadPCDFile (argv[1], cloud_blob);
  fromPCLPointCloud2 (cloud_blob, *cloud);

  // Create search tree
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
  mls_upsampling.setUpsamplingMethod (MovingLeastSquares<PointXYZ, PointNormal>::SAMPLE_LOCAL_PLANE);
  mls_upsampling.setUpsamplingRadius (0.01);
  mls_upsampling.setUpsamplingStepSize (0.01);

  PointCloud<PointNormal>::Ptr cloud_out(new PointCloud<PointNormal>);
  cloud_out->clear();

  mls_upsampling.process (*cloud_out);
  pcl::io::savePCDFile (argv[2], *cloud_out);
}
/* ]--- */