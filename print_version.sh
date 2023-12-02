###
###
 # @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 # @Version: 1.0
 # @Author: tong
 # @Date: 2022-11-09 22:53:59
 # @LastEditTime: 2023-09-12 10:02:49
### 

#!/bin/bash
set -e

version_file="MappingAndLocation.h.in"
#set -x

echo "#####################print version#####################"
echo "#####################print version#####################"


echo "#ifndef __MAL_VERSION_H__" > $version_file
echo "#define __MAL_VERSION_H__" >> $version_file
echo "#include <string>" >> $version_file

svn log -l 1 -v | awk 'NR==2'

version=$(svn log -l 1 -v | awk 'NR==2')

echo "#####################print date#####################"
echo "#####################print date#####################"
date
compile_time=$(date)
# echo "$$version"
echo "std::string version = \" $version \";" >> $version_file
echo "std::string compile_time = \" $compile_time \";" >> $version_file

# svn log -l 1 -v | awk 'NR==2' -n >> MappingAndLocation.h.in

# echo "\"; " >> $version_file

echo "#endif" >> $version_file

mv $version_file ./include/mapping_and_location
