cmake_minimum_required(VERSION 3.5)
project(arm_pose_estimation_final_final)

SET(CMAKE_BUILD_TYPE Debug)

# 设置C++标准
# set(CMAKE_CXX_STANDARD 11)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找PCL库
find_package(PCL 1.10 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(Eigen3 REQUIRED)

# 查找OpenCV库
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

# find_package(jsoncpp REQUIRED)
# include_directories(/usr/include/jsoncpp/json)
# list(APPEND CMAKE_PREFIX_PATH "/usr/lib/x86_64-linux-gnu/cmake/jsoncpp")
# set(Jsoncpp_DIR "/usr/lib/x86_64-linux-gnu/cmake/jsoncpp")

# find_package(Jsoncpp REQUIRED)
# # 添加 Jsoncpp 头文件目录
# include_directories(${Jsoncpp_INCLUDE_DIRS})
# # 添加 Jsoncpp 库文件目录
# link_directories(${Jsoncpp_LIBRARY_DIRS})
# include_directories(/home/chongwenyue/arm/test/arm_pose_estimation_final/jsoncpp/include)
# link_directories(/home/chongwenyue/arm/test/arm_pose_estimation_final/jsoncpp/build/lib/libjsoncpp.a)

# 添加可执行文件
add_executable(arm_pose_estimation_final arm_pose_estimation_final.cpp)

target_link_libraries(arm_pose_estimation_final ${PCL_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES} ${OpenCV_LIBS} boost_thread Eigen3::Eigen)

