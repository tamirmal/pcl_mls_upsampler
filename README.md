# pcl_mls_upsampler
A small tool to upsample point cloud (.pcd file) with PCL library MLS implementation

usage:
	upsampler <input.pcd> <output.pcd>

build:
	mkdir build
	cd build
	cmake ..
	make

Requirements:
	PCL pointclouds.org, version tested is 1.7
	CMake
