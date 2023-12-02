# 动态库单元测试设置
set(LIB_TEST_NAME MappingAndLocation_test)

  # 配置单元测试
################################
# GFalgs
################################
find_package(GFlags REQUIRED)
message("GFLAGS_INCLUDE_DIRS=${GFLAGS_INCLUDE_DIRS}")
message("GFLAGS_LIBRARIES=${GFLAGS_LIBRARIES}")

################################
# GLog
################################
find_package(GLog REQUIRED)
message("GLOG_INCLUDE_DIRS=${GLOG_INCLUDE_DIRS}")
message("GLOG_LIBRARIES=${GLOG_LIBRARIES}")

################################
# GTest
################################
enable_testing()
include(GoogleTest)
find_package(GTest REQUIRED)
message("GTEST_INCLUDE_DIR=${GTEST_INCLUDE_DIR}")
message("GTEST_LIBRARIES=${GTEST_LIBRARIES}")


option(TEST_DEBUG "test" ON)
message("option TEST_DEBUG" ${TEST_DEBUG})
if(TEST_DEBUG)
  message("---------TEST_DEBUG is ${TEST_DEBUG}--------")
  add_definitions(-DTEST_DEBUG)
endif()

add_executable(${LIB_TEST_NAME}
  ${PLUGIN_SRC}
  ${ALL_TEST}
  ${MOCK_SRC}
  test_main.cc
)

link_directories(${CMAKE_INSTALL_LIBDIR} ${OpenCV_LIBRARY_DIR} ${PCL_LIBRARY_DIR})
target_link_libraries(${LIB_TEST_NAME}
  ${GTEST_LIBRARIES}
  ${GLOG_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  ${CERES_LIBRARIES} 
  ${ROBOTIC_PROJECT_DIR}/prebuilt/lib_so/libcommon_lib.so 
  ${ROBOTIC_PROJECT_DIR}/prebuilt/lib_so/libagv_config_lib.so
)

gtest_discover_tests(${LIB_TEST_NAME}
  WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
  -lpthread
  -lgcov -lz
)
