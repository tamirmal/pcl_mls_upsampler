cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(MY_GRAND_PROJECT)
#set(PCL_DIR "/usr/lib/x86_64-linux-gnu/cmake/pcl/")
find_package(PCL 1.7 REQUIRED COMPONENTS common io)

list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(upsampler upsampler.cpp)

target_link_libraries(upsampler ${PCL_LIBRARIES})

## if it fails with lz4 linkage problems, locate the liblz4.so file locally
## and manually link it (need to comment the above 'target_link_libraries()' and use the one below

#add_library(testlib SHARED IMPORTED)
#set_property(TARGET testlib PROPERTY IMPORTED_LOCATION "/workspace/PointCloud-SR-PnP/extensions/pcl_mls_upsampler/build/liblz4.so")
#target_link_libraries(upsampler ${PCL_LIBRARIES} testlib)
