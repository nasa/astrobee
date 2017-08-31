/* Copyright (c) 2017, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 * 
 * All rights reserved.
 * 
 * The Astrobee platform is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

#include <common/thread.h>

#include <gtest/gtest.h>

#include <vector>

void Simple(int a, int b) {
  EXPECT_EQ(2, a);
  EXPECT_EQ(3, b);
}

void SimpleVecPass(std::vector<int> const& vec1,
                   std::vector<int> const& vec2) {
  EXPECT_EQ(&vec1, &vec2);
}

void SimpleVecFail(std::vector<int> const& vec1,
                   std::vector<int> const& vec2) {
  EXPECT_NE(&vec1, &vec2);
}

void AppendToVec(std::vector<int> * vec) {
  vec->push_back(3);
}

TEST(thread, thread_pool) {
  FLAGS_num_threads = 1;
  common::ThreadPool pool;

  // Verify we can add tasks contain lvalues and rvalues.
  pool.AddTask(Simple, 2, 3);
  pool.AddTask(Simple, 2, 3);
  int lvalue = 2;
  pool.AddTask(Simple, lvalue, 3);
  pool.Join();

  // Verify we can send complex types without copy
  std::vector<int> vec(3, 1);
  pool.AddTask(SimpleVecPass, std::ref(vec), std::ref(vec));
  pool.AddTask(SimpleVecPass, std::ref(vec), std::ref(vec));
  pool.AddTask(SimpleVecPass, std::ref(vec), std::ref(vec));
  pool.Join();

  // Verify/Show that not using std ref causes a copy to happen
  pool.AddTask(SimpleVecFail, std::ref(vec), vec);
  pool.AddTask(SimpleVecFail, vec, std::ref(vec));
  pool.AddTask(SimpleVecFail, vec, vec);
  pool.Join();

  // Verify we can send pointers
  pool.AddTask(AppendToVec, &vec);
  pool.Join();
  EXPECT_EQ(4u, vec.size());
}
