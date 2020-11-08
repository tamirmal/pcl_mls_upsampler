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

#include <stdlib.h>

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
static char *input_file = NULL;
static char *output_file = NULL;
static double radius = 0.01;
static unsigned int poly = 2;

void parse_args(int argc, char **argv) {
    for (int i = 0; i < argc; i++) {
        switch (argv[i][0]) {
            case 'i':
                input_file = &argv[i][1];
                break;
            case 'o':
                output_file = &argv[i][1];
                break;
            case 'r':
                radius = (double)atof(&argv[i][1]);
                break;
            case 'p':
                poly = atoi(&argv[i][1]);
                break;
            default:
                cout << "unknown option " << argv[i] << endl;
                break;
        }
    }

    cout << "input_file=" << input_file << endl;
    cout << "output_file=" << output_file << endl;
    cout << "r=" << radius << endl;
    cout << "poly=" << poly << endl;

    if (!input_file || !output_file)
        exit(-1);
}

/* ---[ */
int
main (int argc, char** argv)
{
  parse_args(argc - 1, &argv[1]);

  if (0 == (input_file, output_file)) {
    std:cerr << "input==output ! give different file";
    return (-1);
  }

  // Load file
  PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);
  loadPCDFile (input_file, *cloud);

  cloud->is_dense = true;

  printf("input cloud : %d points\n", cloud->size());

  // Create search tree
  search::KdTree<PointXYZ>::Ptr tree;
  tree.reset (new search::KdTree<PointXYZ> (false));
  tree->setInputCloud (cloud);

  // Testing upsampling
  MovingLeastSquares<PointXYZ, PointNormal> mls;
  // Set parameters
  mls_upsampling.setInputCloud (cloud);
  mls_upsampling.setComputeNormals (true);
  mls_upsampling.setPolynomialFit (poly > 1);
  mls_upsampling.setPolynomialOrder(poly);
  mls_upsampling.setSearchMethod (tree);
  mls_upsampling.setSearchRadius (radius);

  PointCloud<PointNormal>::Ptr cloud_out(new PointCloud<PointNormal>);
  cloud_out->clear();
  mls.process (*cloud_out);

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

  pcl::io::savePCDFile (output_file, *cloud_out_boxed);
}
/* ]--- */