
###
 # @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 # @Version: 1.0
 # @Date: 2022-10-01 17:14:21
 # @LastEditTime: 2022-10-01 17:14:22
### 
aarch64-linux-gnu-g++ \
    -I ./../../prebuilt/include/** \
    -I /usr/include/pcl-1.7 \
    -I ./../../3rdparty \
    -I . ./src/*.cc \
    -fPIC -shared \
    -std=c++11 \
    -lpthread -o libmapping_and_location.so

#cp ./libdevice_lib.so  ~/tong/gomros_refactor/test_agv_sample/prebuilt/lib_so
