cmake_minimum_required(VERSION 3.5)
project(AutoAim)
#add_definitions(-D_GLIBCXX_USE_CXX14_ABI=0)
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O2")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")

set(CMAKE_CXX_STANDARD 17)
set(Sophus_DIR /path/to/source/code/Sophus)
set(MVS_DIR /opt/MVS/include)
# # 设置 MVS 的路径（根据实际情况修改）
# set(MVS_DIR /opt/MVS)
# # 查找 MVS 包
# find_package(MVS REQUIRED)

# # 如果 MVS 包未被找到，可以手动指定路径
# include_directories(${MVS_INCLUDE_DIRS})
# link_directories(${MVS_LIBRARIES})

# # 添加目标
# add_executable(driver driver.cpp)

# # 链接 MVS 库
# target_link_libraries(driver ${MVS_LIBRARIES})
list(APPEND CMAKE_PREFIX_PATH "opt/MVS/lib/64")
list(APPEND CMAKE_PREFIX_PATH "opt/MVS/lib/32")
list(APPEND CMAKE_PREFIX_PATH "opt/MVS/include")
list(APPEND CMAKE_PREFIX_PATH "opt/MVS/bin")
list(APPEND CMAKE_MODULE_PATH /path/to/source/code/g2o/cmake_modules/)
#find_package(Threads)

list(APPEND CMAKE_PREFIX_PATH "/usr/local/include/opencv4/")
set(OpenCV_DIR /usr/local/include/opencv4/)
find_package(OpenCV 4 REQUIRED)

# set(InferenceEngine_DIR /opt/intel/openvino_2022/runtime/cmake)
# set(ngraph_DIR /opt/intel/openvino_2022/runtime/cmake)
# find_package(InferenceEngine REQUIRED)
# find_package(ngraph REQUIRED)
# set(BOOST_ROOT "/usr/include/")
find_package(Boost REQUIRED COMPONENTS system filesystem)

add_subdirectory(src/armor_detector)
add_subdirectory(src/pose_estimate)
add_subdirectory(src/driver)
add_subdirectory(src/utils)


#include_directories(/opt/MVS/include)
#link_directories(/opt/MVS/lib)
include_directories(
        src/
        src/utils/include
        /usr/local/include/eigen3
        ${Boost_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
#         ${CMAKE_CURRENT_SOURCE_DIR}
#         ${CMAKE_CURRENT_BINARY_DIR}
)

add_executable(AutoAim
        src/AutoAim.cpp
)

add_executable(train
       src/trainSVM.cpp
)

add_executable(testArray
       src/utils/testArray.cpp
)


target_link_libraries(AutoAim
        utils
        driver
        armor_detector
        ${OpenCV_LIBS}
        -lboost_system
        -lboost_filesystem
        -lpthread
        #mvscamapi
)

target_link_libraries(train
       utils
       driver
       armor_detector
       ${Boost_LIBRARIES}
#        ${InferenceEngine_LIBRARIES}
       ${OpenCV_LIBS}
       #mvscamapi
)

target_link_libraries(testArray
       utils
       ${OpenCV_LIBS}
       
)
