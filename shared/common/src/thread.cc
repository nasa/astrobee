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

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <sys/time.h>
#include <thread>

DEFINE_int32(num_threads, (std::thread::hardware_concurrency() == 0 ? 2 : std::thread::hardware_concurrency()),
             "Number of threads to use for processing.");

void* common::HolderFunction(void* ptr) {
  common::ThreadPool::VarsTuple* vars = reinterpret_cast<common::ThreadPool::VarsTuple*>(ptr);

  // Run the function
  std::get<0>(*vars)();

  // Set out boolean so someone can know they can delete this
  std::get<3>(*vars) = true;

  // Signal that we are finished
  pthread_mutex_lock(std::get<1>(*vars));
  pthread_cond_signal(std::get<2>(*vars));
  pthread_mutex_unlock(std::get<1>(*vars));

  return NULL;
}

common::ThreadPool::ThreadPool()
  : max_concurrent_jobs_(FLAGS_num_threads) {
  pthread_mutex_init(&cond_mutex_, NULL);
  pthread_cond_init(&cond_, NULL);
  if (max_concurrent_jobs_ <= 0) {
    LOG(ERROR) << "Thread pool without threads created...";
  }
}

common::ThreadPool::~ThreadPool() {
  Join();
  pthread_cond_destroy(&cond_);
  pthread_mutex_destroy(&cond_mutex_);
}

void common::ThreadPool::Join() {
  while (threads_.size()) {
    pthread_join(threads_.front(), NULL);
    threads_.pop_front();
    var_bindings_.pop_front();
  }
}

void common::ThreadPool::WaitTillJobOpening() {
  while (threads_.size() >= max_concurrent_jobs_) {
    // Wait for a condition event to signal a free job or time out
    // after one second. I do a time out because somehow I still have
    // a deadlock. I think it occurs if a task just finishes faster
    // than we can get back into a state to wait for condition.
    pthread_mutex_lock(&cond_mutex_);
    struct timeval tv;
    struct timespec ts;
    gettimeofday(&tv, NULL);
    ts.tv_sec = tv.tv_sec + 1;
    ts.tv_nsec = 0;
    pthread_cond_timedwait(&cond_, &cond_mutex_, &ts);
    pthread_mutex_unlock(&cond_mutex_);

    // Iterate through and clean out all the finished jobs
    std::list<pthread_t>::iterator thread_it = threads_.begin();
    std::list<VarsTuple>::iterator vars_it = var_bindings_.begin();
    while (thread_it != threads_.end()) {
      if (std::get<3>(*vars_it)) {
        // Thread says it has finished. Join.
        pthread_join(*thread_it, NULL);
        thread_it = threads_.erase(thread_it);
        vars_it = var_bindings_.erase(vars_it);
      } else {
        thread_it++;
        vars_it++;
      }
    }
  }
}
