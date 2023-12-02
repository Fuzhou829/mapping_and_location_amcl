#!/bin/sh
###
 # @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 # @Version: 1.0
 # @Date: 2022-08-05 17:12:12
 # @LastEditTime: 2023-03-07 20:21:02
 # @Author: lcfc-desktop
### 
###
 # @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 # @Version: 1.0
 # @Author: renjy
 # @Date: 2022-06-21 15:25:19
 # @LastEditTime: 2022-07-13 20:05:57
### 
set -e

run_install="yes"

if [ $# != 0 ]; then
  for arg in "$@"
  do
    if [ "$arg" = "no_install" ]; then
      run_install="no"
    fi
  done
fi

platform=lib_so

build_dir=${PWD}/build
robotic_project_dir=$(dirname $(dirname "$PWD"))
cmake_dir=${robotic_project_dir}/prebuilt/cmake
output_dir=${robotic_project_dir}/prebuilt/${platform}

echo "########################################################"
echo ${build_dir}
echo ${robotic_project_dir}
echo ${output_dir}
echo "########################################################"

#rm -rf ${build_dir}

cmake -B${build_dir} \
      -DROBOTIC_PROJECT_DIR="${robotic_project_dir}" \
      -DCMAKE_TOOLCHAIN_FILE=/home/shinva/toolchain_x86.cmake \
      -DPLATFORM="${platform}"

cd ${build_dir}
make -j8

if [ "$run_install" = "yes" ]; then
  mkdir -p ${output_dir}
  make install
fi
