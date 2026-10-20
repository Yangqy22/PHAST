// Copyright 2014 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__EXECUTORS__DISTR_THREADED_EXECUTOR_HPP_
#define RCLCPP__EXECUTORS__DISTR_THREADED_EXECUTOR_HPP_

#include <memory>
#include <mutex>
#include <set>
#include <thread>
#include <unordered_map>

#include "rclcpp/executor.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/memory_strategies.hpp"
#include "rclcpp/visibility_control.hpp"
#include "cospike/coexecutor.hpp"
#include "rclcpp/executors/executor_nodelet.hpp"

struct node_item {
  std::shared_ptr<rclcpp::Node> node_ptr;
  bool notify;
  int executor_id;
  node_item(std::shared_ptr<rclcpp::Node> node_ptr, bool notify) {
    this->node_ptr = node_ptr;
    this->notify = notify;
    this->executor_id = -1;
  }
  bool operator==(const node_item& other)
	{
		if (  this->node_ptr == other.node_ptr && 
          this->notify   == other.notify  )
      return true;
		else return false;
	}
};

namespace rclcpp
{
namespace executors
{

class DistrThreadedExecutor
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(DistrThreadedExecutor)

  RCLCPP_PUBLIC
  DistrThreadedExecutor(
    const executor::ExecutorArgs & args = executor::ExecutorArgs(),
    size_t number_of_threads = 0,
    size_t number_of_nodelet = 0,
    bool yield_before_execute = false);

  RCLCPP_PUBLIC
  virtual ~DistrThreadedExecutor();

  RCLCPP_PUBLIC
  void
  spin();

  RCLCPP_PUBLIC
  size_t
  get_number_of_threads();

  RCLCPP_PUBLIC
  size_t
  get_number_of_executors();

  RCLCPP_PUBLIC
  void
  add_node(std::shared_ptr<rclcpp::Node> node_ptr, bool notify = true);

  RCLCPP_PUBLIC
  void
  remove_node(std::shared_ptr<rclcpp::Node> node_ptr, bool notify = true);

protected:

  RCLCPP_PUBLIC
  void run_exe(rclcpp::executors::ExecutorNodelet* exe);

  RCLCPP_PUBLIC
  void alloc_exe();

private:
  RCLCPP_DISABLE_COPY(DistrThreadedExecutor)

  size_t number_of_threads_;
  size_t number_of_nodelet_;
  bool yield_before_execute_;
  std::vector<std::thread> exe_threads_;
  std::vector<rclcpp::executors::ExecutorNodelet*> exe_nodelets_;
  std::vector<node_item> node_list_;
  std::vector<int> node_layer_list_;
  std::map<std::shared_ptr<rclcpp::Node>, rclcpp::executors::ExecutorNodelet*> node_to_executor_map_;
};

}  // namespace executors
}  // namespace rclcpp

#endif  // RCLCPP__EXECUTORS__MULTI_THREADED_EXECUTOR_HPP_
