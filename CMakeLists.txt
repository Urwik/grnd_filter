cmake_minimum_required(VERSION 3.16.3)
project(codeUtilities VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

# find_package(PCL 1.3 REQUIRED COMPONENTS common io filters visualization)
find_package(PCL 1.3 REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(include  ${PCL_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS} ${Eigen3_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS} ${Eigen3_DEFINITIONS})

add_executable(arvc_ground_segmentation_and_filtering src/ground_segmentation_and_filtering.cpp)
target_link_libraries(arvc_ground_segmentation_and_filtering ${PCL_LIBRARIES})

