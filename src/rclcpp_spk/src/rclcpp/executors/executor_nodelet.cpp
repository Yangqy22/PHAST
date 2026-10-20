// Copyright 2015 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rclcpp/executors/executor_nodelet.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <vector>
#include <sys/time.h>
#include <unistd.h>
#include <sys/syscall.h>
#include "rclcpp/utilities.hpp"
#include "rclcpp/scope_exit.hpp"

using rclcpp::executors::ExecutorNodelet;

ExecutorNodelet::ExecutorNodelet(
  const rclcpp::executor::ExecutorArgs & args,
  size_t number_of_threads,
  bool yield_before_execute)
: executor::Executor(args), yield_before_execute_(yield_before_execute)
{
  number_of_threads_ = number_of_threads ? number_of_threads : std::thread::hardware_concurrency();
  if (number_of_threads_ == 0) {
    number_of_threads_ = 1;
  }
}

ExecutorNodelet::~ExecutorNodelet() {
  printf("[~ExecutorNodelet]\n");
}

void
ExecutorNodelet::spin()
{
  if (spinning.exchange(true)) {
    throw std::runtime_error("spin() called while already spinning");
  }
  RCLCPP_SCOPE_EXIT(this->spinning.store(false); );
  size_t thread_id = 0;
  {
    std::lock_guard<std::mutex> wait_lock(wait_mutex_);
    for (; thread_id < number_of_threads_ - 1; ++thread_id) {
      auto func = std::bind(&ExecutorNodelet::run, this, thread_id);
      threads_.emplace_back(func);
    }
  }
  run(thread_id);
  for (auto & thread : threads_) {
    thread.join();
  }
}

void
ExecutorNodelet::co_spin()
{
  if (spinning.exchange(true)) {
    throw std::runtime_error("spin() called while already spinning");
  }
  RCLCPP_SCOPE_EXIT(this->spinning.store(false); );
  co_run(number_of_threads_);
}

size_t
ExecutorNodelet::get_number_of_threads()
{
  return number_of_threads_;
}

void
ExecutorNodelet::run(size_t)
{
  printf("[Hello RoboSpike] [Thread %d] Use ExecutorNodelet\n", gettid());
  
  while (rclcpp::ok(this->context_) && spinning.load()) {
    executor::AnyExecutable any_exec;
    {
      // Use lock to serialize wait_set visiting
      timeval ctime, ftime;
      gettimeofday(&ctime, NULL);
      std::lock_guard<std::mutex> wait_lock(wait_mutex_);
      gettimeofday(&ftime, NULL);
      int duration_us = (ftime.tv_sec - ctime.tv_sec) * 1000000 + (ftime.tv_usec - ctime.tv_usec);
      long tv_sec = duration_us / 1000000;
      long tv_usec = duration_us - tv_sec * 1000000;
      
      if (tv_sec > 0 || tv_usec > 10000)
      printf("[Blocking] [PID: %d] [s: %ld] [us: %ld]\n", gettid(), tv_sec, tv_usec);
      
      if (!rclcpp::ok(this->context_) || !spinning.load()) {
        return;
      }
      if (!get_next_executable(any_exec)) {
        continue;
      }
      if (any_exec.timer) {
        // Guard against multiple threads getting the same timer.
        if (scheduled_timers_.count(any_exec.timer) != 0) {
          // Make sure that any_exec's callback group is reset before
          // the lock is released.
          if (any_exec.callback_group) {
            any_exec.callback_group->can_be_taken_from().store(true);
          }
          continue;
        }
        scheduled_timers_.insert(any_exec.timer);
      }
    }
    if (yield_before_execute_) {
      std::this_thread::yield();
    }

    execute_any_executable(any_exec);

    if (any_exec.timer) {
      std::lock_guard<std::mutex> wait_lock(wait_mutex_);
      auto it = scheduled_timers_.find(any_exec.timer);
      if (it != scheduled_timers_.end()) {
        scheduled_timers_.erase(it);
      }
    }
    // Clear the callback_group to prevent the AnyExecutable destructor from
    // resetting the callback group `can_be_taken_from`
    any_exec.callback_group.reset();
  }
}

void
ExecutorNodelet::co_run(size_t number_of_threads)
{
  while (rclcpp::ok(this->context_) && spinning.load()) {
    executor::AnyExecutable any_exec;
    {
      std::lock_guard<std::mutex> wait_lock(wait_mutex_);
      if (!rclcpp::ok(this->context_) || !spinning.load()) {
        return;
      }
      if (!get_next_executable(any_exec)) {
        continue;
      }
      if (any_exec.timer) {
        // Guard against multiple threads getting the same timer.
        if (scheduled_timers_.count(any_exec.timer) != 0) {
          // Make sure that any_exec's callback group is reset before
          // the lock is released.
          if (any_exec.callback_group) {
            any_exec.callback_group->can_be_taken_from().store(true);
          }
          continue;
        }
        scheduled_timers_.insert(any_exec.timer);
      }
    }
    if (yield_before_execute_) {
      std::this_thread::yield();
    }

    execute_any_executable(any_exec);

    if (any_exec.timer) {
      std::lock_guard<std::mutex> wait_lock(wait_mutex_);
      auto it = scheduled_timers_.find(any_exec.timer);
      if (it != scheduled_timers_.end()) {
        scheduled_timers_.erase(it);
      }
    }
    // Clear the callback_group to prevent the AnyExecutable destructor from
    // resetting the callback group `can_be_taken_from`
    any_exec.callback_group.reset();
  }
}