
###
 # @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 # @Version: 1.0
 # @Date: 2022-08-05 17:12:12
 # @LastEditTime: 2023-04-17 12:43:42
 # @Author: lcfc-desktop
### 
#!/bin/sh
set -e

cp ../../prebuilt/lib_so/libmapping_and_location.so ../../../project/600kg-slam-agv/prebuilt/lib_so/
cp -r ../../prebuilt/include/mapping_and_location ../../../project/600kg-slam-agv/prebuilt/include/
#cp ../../prebuilt/lib_so/libmapping_and_location.so ../../../project/DoubleSteer_project/prebuilt/lib_so/
#cp -r ../../prebuilt/include/mapping_and_location ../../../project/DoubleSteer_project/prebuilt/include/
#cp -r ../../prebuilt/include/mapping_and_location ../../../project/SF1516_project/prebuilt/include/
#cp ../../prebuilt/lib_so/libmapping_and_location.so ../../../project/SF1516_project/prebuilt/lib_so/
