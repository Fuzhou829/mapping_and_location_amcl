/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.0
 * @Author: renjy
 * @Date: 2022-11-07 15:14:43
 * @LastEditTime: 2023-07-10 02:57:13
 */


#include <gtest/gtest.h>



GTEST_API_ int main(int argc, char** argv) {
  int ret = system("rm -rf ./out && mkdir -p out");

  testing::InitGoogleTest(&argc, argv);
  ret = RUN_ALL_TESTS();
  return ret;
}
