#####################################
        # 动态库安装 / 单元测试#
#####################################
cmake_minimum_required(VERSION 3.15.0)
set(PACKAGE_NAME mapping_and_location)
project(${PACKAGE_NAME})

# 自动寻找库的头文件以及库，预置变量
find_package(PCL REQUIRED)
find_package(Ceres REQUIRED)

set(OPENCV_ROOT ${ROBOTIC_PROJECT_DIR}/3rdparty/opencv)
LIST(APPEND CMAKE_PREFIX_PATH ${OPENCV_ROOT}/cmake)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

#打印编译动态库的版本
execute_process(COMMAND sh print_version.sh)

if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64" OR CMAKE_CROSSCOMPILING)
    message("this is aarch64 cpu")
    set(3rdparty_include ${CMAKE_CURRENT_SOURCE_DIR}/../../3rdparty/aarch64)
    set(3rdparty_lib ${CMAKE_CURRENT_SOURCE_DIR}/../../3rdparty/aarch64/lib)
elseif(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
        message("this is x86_64 cpu")
  set(3rdparty_include ${CMAKE_CURRENT_SOURCE_DIR}/../../3rdparty/x86_64)
  set(3rdparty_lib ${CMAKE_CURRENT_SOURCE_DIR}/../../3rdparty/x86_64/lib)
endif()

set(3RDPARTIES_INCLUDE_DIR ${ROBOTIC_PROJECT_DIR}/3rdparty)

include_directories(${3RDPARTIES_INCLUDE_DIR})
# include_directories(${EIGEN3_INCLUDE_DIRS})
# include_directories(${3rdparty_include})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${CERES_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/location)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/mapping)
include_directories(${ROBOTIC_PROJECT_DIR}/prebuilt/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/test)



message("#####################################")
message("CMAKE_CURRENT_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}")
message("ROBOTIC_PROJECT_DIR=${ROBOTIC_PROJECT_DIR}")
message("EIGEN3_INCLUDE_DIRS=${EIGEN3_INCLUDE_DIRS}")
message("PCL_INCLUDE_DIRS = ${PCL_INCLUDE_DIRS}")
message("CERES_INCLUDE_DIRS = ${CERES_INCLUDE_DIRS}")
message("#####################################")

file(GLOB_RECURSE PLUGIN_PUBLIC_H # 捕获指定目录下的所有文件
  "include/${PACKAGE_NAME}/*"
)

message(STATUS "PLUGIN_PUBLIC_H")
foreach(file ${PLUGIN_PUBLIC_H})
  message(STATUS "${file}")
endforeach(file ${PLUGIN_PUBLIC_H})

file(GLOB_RECURSE PLUGIN_SRC
  "src/*.cc"
)

message(STATUS "PLUGIN_SRC")
foreach(file ${PLUGIN_SRC})
  message(STATUS "${file}")
endforeach(file ${PLUGIN_SRC})

file(GLOB_RECURSE ALL_TEST
  "test/*_test.cc"
)

message(STATUS "ALL_TEST")
foreach(file ${ALL_TEST})
  message(STATUS "${file}")
endforeach(file ${ALL_TEST})


file(GLOB_RECURSE MOCK_SRC
  "test/mock/*.cc"
)

message(STATUS "MOCK_SRC")
foreach(file ${MOCK_SRC})
  message(STATUS "${file}")
endforeach(file ${MOCK_SRC})


# 安装动态库
set(CMAKE_INSTALL_INCLUDEDIR ${ROBOTIC_PROJECT_DIR}/prebuilt/include)
set(CMAKE_INSTALL_LIBDIR ${ROBOTIC_PROJECT_DIR}/prebuilt/${PLATFORM})
message("CMAKE_INSTALL_LIBDIR=${CMAKE_INSTALL_LIBDIR}")

link_directories(${CMAKE_INSTALL_LIBDIR} ${OpenCV_LIBRARY_DIR} ${PCL_LIBRARY_DIR})
add_library(${PACKAGE_NAME} SHARED ${PLUGIN_SRC})
target_link_libraries(${PACKAGE_NAME} 
  -lgcov -lpthread -lz
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  ${CERES_LIBRARIES} 
  ${ROBOTIC_PROJECT_DIR}/prebuilt/lib_so/libcommon_lib.so 
  ${ROBOTIC_PROJECT_DIR}/prebuilt/lib_so/libagv_config_lib.so
)

set_target_properties(${PACKAGE_NAME} PROPERTIES PUBLIC_HEADER "${PLUGIN_PUBLIC_H}")
include(GNUInstallDirs)
install(TARGETS ${PACKAGE_NAME} DESTINATION ${CMAKE_INSTALL_LIBDIR}
  PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PACKAGE_NAME}
)


# set(CMAKE_CXX_FLAGS "-std=c++11 -g -O2 -fprofile-arcs -ftest-coverage")
set(CMAKE_CXX_FLAGS "-std=c++11 -g -O0 -fprofile-arcs -ftest-coverage")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
# # 增加单元测试
if (NOT CMAKE_CROSSCOMPILING)
#add_subdirectory(test)
endif()

file(GLOB_RECURSE LINT_PLUGIN_SRC
  "src/*.cc"
)

file(GLOB_RECURSE LINT_PLUGIN_H
  "include/*.h"
)

file(GLOB_RECURSE NO_CHECK_H
  "include/mapping/karto_mapping/*.h"
  "include/location/amcl/*.h"
  "include/sparse_bundle_adjustment/*.h"
)

file(GLOB_RECURSE NO_CHECK_SRC
  "src/mapping/karto_mapping/*.cc"
  "src/location/amcl/*.cc"
  "src/sparse_bundle_adjustment/*.cc"
)

list(REMOVE_ITEM LINT_PLUGIN_SRC ${NO_CHECK_SRC})
list(REMOVE_ITEM LINT_PLUGIN_H ${NO_CHECK_H})


# 代码规范化检查 cpplint
# file(GLOB_RECURSE LINT_SRC
#   ${LINT_PLUGIN_SRC}
#   ${LINT_PLUGIN_H}
#   "test/*.cc"
#   "test/*.h"
# )

# foreach(v ${LINT_SRC})
#   message(STATUS "${v}")
#   execute_process(COMMAND cpplint --quiet ${v} COMMAND_ERROR_IS_FATAL ANY)
# endforeach(v)
