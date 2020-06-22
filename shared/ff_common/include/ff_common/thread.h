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

#ifndef FF_COMMON_THREAD_H_
#define FF_COMMON_THREAD_H_

#include <gflags/gflags.h>
#include <pthread.h>

#include <list>
#include <functional>

DECLARE_int32(num_threads);

#define GOOGLE_ALLOW_RVALUE_REFERENCES_PUSH
#define GOOGLE_ALLOW_RVALUE_REFERENCES_POP

GOOGLE_ALLOW_RVALUE_REFERENCES_PUSH

namespace ff_common {

  void* HolderFunction(void* ptr);

  class ThreadPool {
   public:
    typedef std::tuple<std::function<void(void)>, pthread_mutex_t*, pthread_cond_t*, bool> VarsTuple;

    ThreadPool();
    ~ThreadPool();
    // The following identifies this thread as non copyable and non
    // moveable. Our threads are holding pointers to this exact
    // object.
    ThreadPool(const ThreadPool&) = delete;
    ThreadPool& operator=(const ThreadPool&) = delete;

    // This pushes back a function and it's arguments to be
    // executed. This method will block if the number of threads is
    // already maxed. You can also push back mixed types of functions.
    //
    // Example:
    // void Monkey(std::vector const& input, int val, std::vector * output);
    // ThreadPool pool;
    // pool.AddTask(Monkey, std::ref(input), 3, &output);
    //
    // If you don't use a std::ref as in the above example, the input
    // will be copied. Other alternatives are to use a pointer.
    template <typename Function, typename... Args>
    void AddTask(Function&& f, Args&&... args) {
      WaitTillJobOpening();

      // Bind up the function the user has given us
      var_bindings_.emplace_back(std::bind(f, args...),
                                 &cond_mutex_,
                                 &cond_, false);

      // Create the thread and start doing work
      threads_.push_back(pthread_t());
      pthread_create(&threads_.back(), NULL, HolderFunction, &var_bindings_.back());
    }

    void Join();

   private:
    void WaitTillJobOpening();

    size_t max_concurrent_jobs_;
    std::list<pthread_t> threads_;
    std::list<VarsTuple> var_bindings_;
    pthread_mutex_t cond_mutex_;
    pthread_cond_t cond_;
  };

}  // namespace ff_common

GOOGLE_ALLOW_RVALUE_REFERENCES_POP

#endif  // FF_COMMON_THREAD_H_
