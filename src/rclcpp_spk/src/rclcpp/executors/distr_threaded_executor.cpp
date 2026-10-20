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

#include "rclcpp/executors/distr_threaded_executor.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <vector>
#include <sys/time.h>
#include <unistd.h>
#include <sys/syscall.h>
#include "rclcpp/utilities.hpp"
#include "rclcpp/scope_exit.hpp"
#include "rclcpp/node.hpp"

using rclcpp::executors::DistrThreadedExecutor;

DistrThreadedExecutor::DistrThreadedExecutor(
  const rclcpp::executor::ExecutorArgs & args,
  size_t number_of_threads,
  size_t number_of_nodelet,
  bool yield_before_execute)
: yield_before_execute_(yield_before_execute)
{
  number_of_threads_ = number_of_threads ? number_of_threads : std::thread::hardware_concurrency();
  if (number_of_threads_ == 0) {
    number_of_threads_ = 1;
  }
  number_of_nodelet_  = (number_of_threads_ > number_of_nodelet) ? number_of_nodelet : 1;
  printf("[DistrThreadedExecutor()] %d, %d\n", number_of_threads_, number_of_nodelet_);
}

DistrThreadedExecutor::~DistrThreadedExecutor() {}

// Public -------------------------------------------

void
DistrThreadedExecutor::spin()
{
  // [1] Alloc the Executor & Threads
  alloc_exe();

  // [2] Run the Executors
  for (size_t i = 1; i < number_of_nodelet_; i++) {
    exe_threads_.push_back(
      std::thread(&DistrThreadedExecutor::run_exe, this, exe_nodelets_[i])
    );
  }
  run_exe(exe_nodelets_[0]);
  
  // [3] Join the threads
  for (size_t i = 0; i < exe_threads_.size(); i++) {
    exe_threads_[i].join(); 
  }
}

size_t
DistrThreadedExecutor::get_number_of_threads()
{
  return number_of_threads_;
}

size_t
DistrThreadedExecutor::get_number_of_executors()
{
  return number_of_nodelet_;
}

void
DistrThreadedExecutor::remove_node(std::shared_ptr<rclcpp::Node> node_ptr, bool notify)
{
  node_to_executor_map_[node_ptr]->remove_node(node_ptr);
}

void
DistrThreadedExecutor::add_node(std::shared_ptr<rclcpp::Node> node_ptr, bool notify) {
  node_list_.push_back(node_item(node_ptr, notify));
}

// Protected ----------------------------------------

void 
DistrThreadedExecutor::run_exe(rclcpp::executors::ExecutorNodelet* exe) {
  printf("[DistrThreadedExecutor] [%d] run_exe\n", gettid());
  exe->spin();
}

void
DistrThreadedExecutor::alloc_exe()
{
  // Initialize the Node Layer List
  for (auto & node_item_ : node_list_) {
    node_layer_list_.push_back(0);
    auto node_ptr = node_item_.node_ptr;
    if (node_ptr->get_sub_names().size() == 0) {
      node_layer_list_.push_back(0);
    } else {
      node_layer_list_.push_back(-1);
    }
  }
  bool run_continue = true;
  while (run_continue) {
    run_continue = false;
    // Build edge: ( node[i] ) -> node[j]
    for (size_t i = 0; i < node_list_.size(); i++) {
      if (node_layer_list_[i] >= 0) {
        int cur_layer_id = node_layer_list_[i];
        auto cur_pub_list = node_list_[i].node_ptr->get_pub_names();
        // Build edge: node[i] -> ( node[j] )
        for (size_t j = 0; j < node_list_.size(); j++) {
          auto next_sub_list = node_list_[j].node_ptr->get_sub_names();
          // Matching: next_sub_list & cur_pub_list -------------------
          for (auto & cur_pub : cur_pub_list) {
            for (auto & next_sub : next_sub_list) {
              if (cur_pub == next_sub && node_layer_list_[j] < cur_layer_id + 1) {
                node_layer_list_[j] = cur_layer_id + 1;
                run_continue = true;
        }}}}
    }}
  }
  std::vector<int> th_num_per_nodelet(number_of_nodelet_, 0);
  for (size_t k = 0; k < node_list_.size(); k++) {
    th_num_per_nodelet[node_layer_list_[k] % number_of_nodelet_] += 1;
  }
  int thread_num = 0;
  for (size_t i = 0; i < th_num_per_nodelet.size(); i++) {
    th_num_per_nodelet[i] = th_num_per_nodelet[i] * number_of_nodelet_ / number_of_threads_;
    thread_num += th_num_per_nodelet[i];
  }
  for (size_t i = 0; i < number_of_threads_ - thread_num; i++) {
    th_num_per_nodelet[i % number_of_nodelet_] += 1;
  }
  for (size_t i = 0; i < number_of_nodelet_; i++) {
    auto exe_ptr = new rclcpp::executors::ExecutorNodelet(rclcpp::executor::ExecutorArgs(), th_num_per_nodelet[i], true);
    exe_nodelets_.push_back(exe_ptr);
  }
  for (size_t i = 0; i < number_of_nodelet_; i++) {
    for (size_t k = 0; k < node_list_.size(); k++) {
      if (node_layer_list_[k] % number_of_nodelet_ == i) {
        printf( "Map Node [%s] to executor [%d] (%d threads)\n", 
                node_list_[k].node_ptr->get_name(), i, th_num_per_nodelet[i]);
        exe_nodelets_[i]->add_node(node_list_[k].node_ptr);
        node_to_executor_map_.insert(std::pair<std::shared_ptr<rclcpp::Node>, 
          rclcpp::executors::ExecutorNodelet*>(node_list_[k].node_ptr, exe_nodelets_[i]));
      }
    }
  }
}
