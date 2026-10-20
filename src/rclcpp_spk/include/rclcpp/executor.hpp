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

#ifndef RCLCPP__EXECUTOR_HPP_
#define RCLCPP__EXECUTOR_HPP_

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <list>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <sys/time.h>
#include <utility>

#include "rcl/guard_condition.h"
#include "rcl/wait.h"
#include "rclcpp/contexts/default_context.hpp"
#include "rclcpp/memory_strategies.hpp"
#include "rclcpp/memory_strategy.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rclcpp/scope_exit.hpp"

#include "cospike/coroutine.hpp"

namespace rclcpp
{

// Forward declaration is used in convenience method signature.
class Node;

namespace executor
{

/// Return codes to be used with spin_until_future_complete.
/**
 * SUCCESS: The future is complete and can be accessed with "get" without blocking.
 * INTERRUPTED: The future is not complete, spinning was interrupted by Ctrl-C or another error.
 * TIMEOUT: Spinning timed out.
 */
enum class FutureReturnCode {SUCCESS, INTERRUPTED, TIMEOUT};

RCLCPP_PUBLIC
std::ostream &
operator<<(std::ostream & os, const FutureReturnCode & future_return_code);

RCLCPP_PUBLIC
std::string
to_string(const FutureReturnCode & future_return_code);

///
/**
 * Options to be passed to the executor constructor.
 */
struct ExecutorArgs
{
  ExecutorArgs()
  : memory_strategy(memory_strategies::create_default_strategy()),
    context(rclcpp::contexts::default_context::get_global_default_context()),
    max_conditions(0)
  {}

  memory_strategy::MemoryStrategy::SharedPtr memory_strategy;
  std::shared_ptr<rclcpp::Context> context;
  size_t max_conditions;
};

static inline ExecutorArgs create_default_executor_arguments()
{
  return ExecutorArgs();
}

// Shared with delayed coroutine dispatchers so shutdown can close their access
// before destroying the queue and guard condition.
struct CoroutineLifetime {
  std::mutex mutex;
  bool accepting_work = true;
};

/// Coordinate the order and timing of available communication tasks.
/**
 * Executor provides spin functions (including spin_node_once and spin_some).
 * It coordinates the nodes and callback groups by looking for available work and completing it,
 * based on the threading or concurrency scheme provided by the subclass implementation.
 * An example of available work is executing a subscription callback, or a timer callback.
 * The executor structure allows for a decoupling of the communication graph and the execution
 * model.
 * See SingleThreadedExecutor and MultiThreadedExecutor for examples of execution paradigms.
 */
class Executor
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS_NOT_COPYABLE(Executor)

  /// Default constructor.
  // \param[in] ms The memory strategy to be used with this executor.
  RCLCPP_PUBLIC
  explicit Executor(const ExecutorArgs & args = ExecutorArgs());

  /// Default destructor.
  RCLCPP_PUBLIC
  virtual ~Executor();

  /// Do work periodically as it becomes available to us. Blocking call, may block indefinitely.
  // It is up to the implementation of Executor to implement spin.
  virtual void
  spin() = 0;

  /// Add a node to the executor.
  /**
   * An executor can have zero or more nodes which provide work during `spin` functions.
   * \param[in] node_ptr Shared pointer to the node to be added.
   * \param[in] notify True to trigger the interrupt guard condition during this function. If
   * the executor is blocked at the rmw layer while waiting for work and it is notified that a new
   * node was added, it will wake up.
   */
  RCLCPP_PUBLIC
  virtual void
  add_node(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr, bool notify = true);

  /// Convenience function which takes Node and forwards NodeBaseInterface.
  RCLCPP_PUBLIC
  virtual void
  add_node(std::shared_ptr<rclcpp::Node> node_ptr, bool notify = true);

  /// Remove a node from the executor.
  /**
   * \param[in] node_ptr Shared pointer to the node to remove.
   * \param[in] notify True to trigger the interrupt guard condition and wake up the executor.
   * This is useful if the last node was removed from the executor while the executor was blocked
   * waiting for work in another thread, because otherwise the executor would never be notified.
   */
  RCLCPP_PUBLIC
  virtual void
  remove_node(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr, bool notify = true);

  /// Convenience function which takes Node and forwards NodeBaseInterface.
  RCLCPP_PUBLIC
  virtual void
  remove_node(std::shared_ptr<rclcpp::Node> node_ptr, bool notify = true);

  /// Add a node to executor, execute the next available unit of work, and remove the node.
  /**
   * \param[in] node Shared pointer to the node to add.
   * \param[in] timeout How long to wait for work to become available. Negative values cause
   * spin_node_once to block indefinitely (the default behavior). A timeout of 0 causes this
   * function to be non-blocking.
   */
  template<typename RepT = int64_t, typename T = std::milli>
  void
  spin_node_once(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node,
    std::chrono::duration<RepT, T> timeout = std::chrono::duration<RepT, T>(-1))
  {
    return spin_node_once_nanoseconds(
      node,
      std::chrono::duration_cast<std::chrono::nanoseconds>(timeout)
    );
  }

  /// Convenience function which takes Node and forwards NodeBaseInterface.
  template<typename NodeT = rclcpp::Node, typename RepT = int64_t, typename T = std::milli>
  void
  spin_node_once(
    std::shared_ptr<NodeT> node,
    std::chrono::duration<RepT, T> timeout = std::chrono::duration<RepT, T>(-1))
  {
    return spin_node_once_nanoseconds(
      node->get_node_base_interface(),
      std::chrono::duration_cast<std::chrono::nanoseconds>(timeout)
    );
  }

  /// Add a node, complete all immediately available work, and remove the node.
  /**
   * \param[in] node Shared pointer to the node to add.
   */
  RCLCPP_PUBLIC
  void
  spin_node_some(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node);

  /// Convenience function which takes Node and forwards NodeBaseInterface.
  RCLCPP_PUBLIC
  void
  spin_node_some(std::shared_ptr<rclcpp::Node> node);

  /// Complete all available queued work without blocking.
  /**
   * This function can be overridden. The default implementation is suitable for a
   * single-threaded model of execution.
   * Adding subscriptions, timers, services, etc. with blocking callbacks will cause this function
   * to block (which may have unintended consequences).
   *
   * \param[in] max_duration The maximum amount of time to spend executing work, or 0 for no limit.
   * Note that spin_some() may take longer than this time as it only returns once max_duration has
   * been exceeded.
   */
  RCLCPP_PUBLIC
  virtual void
  spin_some(std::chrono::nanoseconds max_duration = std::chrono::nanoseconds(0));

  RCLCPP_PUBLIC
  virtual void
  spin_once(std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));

  /// Spin (blocking) until the future is complete, it times out waiting, or rclcpp is interrupted.
  /**
   * \param[in] future The future to wait on. If SUCCESS, the future is safe to access after this
   *   function.
   * \param[in] timeout Optional timeout parameter, which gets passed to Executor::spin_node_once.
   *   `-1` is block forever, `0` is non-blocking.
   *   If the time spent inside the blocking loop exceeds this timeout, return a TIMEOUT return
   *   code.
   * \return The return code, one of `SUCCESS`, `INTERRUPTED`, or `TIMEOUT`.
   */
  template<typename ResponseT, typename TimeRepT = int64_t, typename TimeT = std::milli>
  FutureReturnCode
  spin_until_future_complete(
    std::shared_future<ResponseT> & future,
    std::chrono::duration<TimeRepT, TimeT> timeout = std::chrono::duration<TimeRepT, TimeT>(-1))
  {
    // TODO(wjwwood): does not work recursively; can't call spin_node_until_future_complete
    // inside a callback executed by an executor.

    // Check the future before entering the while loop.
    // If the future is already complete, don't try to spin.
    std::future_status status = future.wait_for(std::chrono::seconds(0));
    if (status == std::future_status::ready) {
      return FutureReturnCode::SUCCESS;
    }

    auto end_time = std::chrono::steady_clock::now();
    std::chrono::nanoseconds timeout_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      timeout);
    if (timeout_ns > std::chrono::nanoseconds::zero()) {
      end_time += timeout_ns;
    }
    std::chrono::nanoseconds timeout_left = timeout_ns;

    if (spinning.exchange(true)) {
      throw std::runtime_error("spin_until_future_complete() called while already spinning");
    }
    RCLCPP_SCOPE_EXIT(this->spinning.store(false); );
    while (rclcpp::ok(this->context_) && spinning.load()) {
      // Do one item of work.
      spin_once_impl(timeout_left);

      // Check if the future is set, return SUCCESS if it is.
      status = future.wait_for(std::chrono::seconds(0));
      if (status == std::future_status::ready) {
        return FutureReturnCode::SUCCESS;
      }
      // If the original timeout is < 0, then this is blocking, never TIMEOUT.
      if (timeout_ns < std::chrono::nanoseconds::zero()) {
        continue;
      }
      // Otherwise check if we still have time to wait, return TIMEOUT if not.
      auto now = std::chrono::steady_clock::now();
      if (now >= end_time) {
        return FutureReturnCode::TIMEOUT;
      }
      // Subtract the elapsed time from the original timeout.
      timeout_left = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - now);
    }

    // The future did not complete before ok() returned false, return INTERRUPTED.
    return FutureReturnCode::INTERRUPTED;
  }

  /// Cancel any running spin* function, causing it to return.
  /* This function can be called asynchonously from any thread. */
  RCLCPP_PUBLIC
  void
  cancel();

  /// Support dynamic switching of the memory strategy.
  /**
   * Switching the memory strategy while the executor is spinning in another threading could have
   * unintended consequences.
   * \param[in] memory_strategy Shared pointer to the memory strategy to set.
   */
  RCLCPP_PUBLIC
  void
  set_memory_strategy(memory_strategy::MemoryStrategy::SharedPtr memory_strategy);

  // spk-priority
  bool callback_priority_enabled = false;
  int executor_priority = 0;

  RCLCPP_PUBLIC
  void 
  enable_priority_scheduling()
  {
    callback_priority_enabled = true;
    if (memory_strategy_) memory_strategy_->callback_priority_enabled = true;
  }

  RCLCPP_PUBLIC
  void
  set_priority(rclcpp::TimerBase::SharedPtr ptr, int priority)
  {
    if (!ptr) return;
    ptr->callback_priority = priority;
  }

  RCLCPP_PUBLIC
  void 
  set_priority(rclcpp::SubscriptionBase::SharedPtr ptr, int priority)
  {
    if (!ptr) return;
    ptr->callback_priority = priority;
  }

  RCLCPP_PUBLIC
  void
  set_priority(rclcpp::ServiceBase::SharedPtr ptr, int priority)
  {
    if (ptr) ptr->callback_priority = priority;
  }

  RCLCPP_PUBLIC
  void
  set_priority(rclcpp::ClientBase::SharedPtr ptr, int priority)
  {
    if (ptr) ptr->callback_priority = priority;
  }

  RCLCPP_PUBLIC
  void
  set_priority(rclcpp::Waitable::SharedPtr ptr, int priority)
  {
    if (ptr) ptr->callback_priority = priority;
  }

  RCLCPP_PUBLIC
  void 
  enable_callback_priority()
  {
    callback_priority_enabled = true;
    if (memory_strategy_) memory_strategy_->callback_priority_enabled = true;
  }

  RCLCPP_PUBLIC
  void 
  disable_callback_priority()
  {
    callback_priority_enabled = false;
    if (memory_strategy_) memory_strategy_->callback_priority_enabled = false;
  }

  RCLCPP_PUBLIC
  void
  set_callback_priority(rclcpp::TimerBase::SharedPtr ptr, int priority)
  {
    if (!ptr) return;
    ptr->callback_priority = priority;
  }

  RCLCPP_PUBLIC
  void 
  set_callback_priority(rclcpp::SubscriptionBase::SharedPtr ptr, int priority)
  {
    if (!ptr) return;
    ptr->callback_priority = priority;
 
    // There might be other waitables associated with the subscription
    // (e.g., events, intra-process msgs; see NodeTopics::add_subscription() in node_topics.cpp)
    // auto intra_process_waitable = ptr->get_intra_process_waitable();
    // if (intra_process_waitable) {
    //   intra_process_waitable->callback_priority = priority;
    // }
    // for (auto & subscription_event : ptr->get_event_handlers()) {
    //   subscription_event->callback_priority = priority;
    // }
  }

  RCLCPP_PUBLIC
  void
  set_callback_priority(rclcpp::ServiceBase::SharedPtr ptr, int priority)
  {
    if (ptr) ptr->callback_priority = priority;
  }

  RCLCPP_PUBLIC
  void
  set_callback_priority(rclcpp::ClientBase::SharedPtr ptr, int priority)
  {
    if (ptr) ptr->callback_priority = priority;
  }

  RCLCPP_PUBLIC
  void
  set_callback_priority(rclcpp::Waitable::SharedPtr ptr, int priority)
  {
    if (ptr) ptr->callback_priority = priority;
  }


  /// Maintaining the life of TaskPromise
  RCLCPP_PUBLIC
  static std::queue<retTask> task_queue;
  std::shared_ptr<CoroutineLifetime> coroutine_lifetime = std::make_shared<CoroutineLifetime>();

  /// The resumed callback
  RCLCPP_PUBLIC
  // std::queue<std::function<void()> > coroutine_queue_;

  struct Compare {
    using Pair = std::pair<int, std::function<void()>>;
    bool operator()(const Pair& a, const Pair& b) const {
        return a.first < b.first;  // Compare only the priority
    }
  };
  RCLCPP_PUBLIC
  // std::priority_queue<std::pair<int, std::function<void()>>, 
  //                       std::vector<std::pair<int, std::function<void()>>>, 
  //                       Compare> coroutine_queue_;
  std::priority_queue<std::pair<int, std::function<void()>>, 
                        std::vector<std::pair<int, std::function<void()>>>, 
                        Compare> coroutine_queue_;

  /// Mutex for coroutine_queue_
  RCLCPP_PUBLIC
  std::mutex coroutine_queue_mutex_;

  /// Has suspending coroutine
  RCLCPP_PUBLIC
  std::vector<bool> has_ready_coroutine_vec_;

  /// Has record the suspend <thread_id, coroutine_id>
  /// thread_id: use to leave at leat one thread to get the re_execute of coroutine
  /// coroutine_id: use to match the coroutine and pop the queue
  RCLCPP_PUBLIC
  std::queue<std::pair<int, int>> free_thread_pid_queue_;

  /// Mutex for both has_ready_coroutine_vec_ & free_thread_pid_queue_
  RCLCPP_PUBLIC
  std::mutex co_vec_mutex_;

  /// Suspending Coroutine List
  RCLCPP_PUBLIC
  std::list<void*> suspend_coroutine_list_;

  /// Mutex for suspend_coroutine_list_
  RCLCPP_PUBLIC
  std::mutex suspend_coroutine_list_mutex_;

  // Wake a blocked ROS wait after coroutine state/queue changes.
  RCLCPP_PUBLIC
  void notify_coroutine_work() noexcept;

protected:
  // spk-priority
  RCLCPP_PUBLIC
  void
  print_list_ready_executable(AnyExecutable & any_executable);

  RCLCPP_PUBLIC
  void
  spin_node_once_nanoseconds(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node,
    std::chrono::nanoseconds timeout);

  /// Find the next available executable and do the work associated with it.
  /** \param[in] any_exec Union structure that can hold any executable type (timer, subscription,
   * service, client).
   */
  RCLCPP_PUBLIC
  void
  execute_any_executable(AnyExecutable & any_exec);

  RCLCPP_PUBLIC
  static void
  execute_subscription(
    void* executor_ptr,
    rclcpp::SubscriptionBase::SharedPtr subscription);

  RCLCPP_PUBLIC
  static void
  execute_intra_process_subscription(
    void* executor_ptr,
    rclcpp::SubscriptionBase::SharedPtr subscription);

  RCLCPP_PUBLIC
  static void
  execute_timer(rclcpp::TimerBase::SharedPtr timer);

  RCLCPP_PUBLIC
  static void
  execute_service(rclcpp::ServiceBase::SharedPtr service);

  RCLCPP_PUBLIC
  static void
  execute_client(rclcpp::ClientBase::SharedPtr client);

  RCLCPP_PUBLIC
  void
  wait_for_work(std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));

  RCLCPP_PUBLIC
  void
  wait_free_wfk(std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));

  RCLCPP_PUBLIC
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
  get_node_by_group(rclcpp::callback_group::CallbackGroup::SharedPtr group);

  RCLCPP_PUBLIC
  rclcpp::callback_group::CallbackGroup::SharedPtr
  get_group_by_timer(rclcpp::TimerBase::SharedPtr timer);

  RCLCPP_PUBLIC
  void
  get_rest_coroutine(AnyExecutable & any_exec, int & highest_priority);

  RCLCPP_PUBLIC
  void
  get_next_timer(AnyExecutable & any_exec);

  RCLCPP_PUBLIC
  bool
  get_next_ready_executable(AnyExecutable & any_executable);

  RCLCPP_PUBLIC
  bool
  get_next_executable(
    AnyExecutable & any_executable,
    std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));

  /// Spinning state, used to prevent multi threaded calls to spin and to cancel blocking spins.
  std::atomic_bool spinning;

  /// Guard condition for signaling the rmw layer to wake up for special events.
  rcl_guard_condition_t interrupt_guard_condition_ = rcl_get_zero_initialized_guard_condition();

  /// Wait set for managing entities that the rmw layer waits on.
  rcl_wait_set_t wait_set_ = rcl_get_zero_initialized_wait_set();

  // Mutex to protect the subsequent memory_strategy_.
  std::mutex memory_strategy_mutex_;

  /// The memory strategy: an interface for handling user-defined memory allocation strategies.
  memory_strategy::MemoryStrategy::SharedPtr memory_strategy_;

  /// The context associated with this executor.
  std::shared_ptr<rclcpp::Context> context_;

private:
  RCLCPP_DISABLE_COPY(Executor)

  RCLCPP_PUBLIC
  void
  spin_once_impl(std::chrono::nanoseconds timeout);

  std::list<rclcpp::node_interfaces::NodeBaseInterface::WeakPtr> weak_nodes_;
  std::list<const rcl_guard_condition_t *> guard_conditions_;
};

}  // namespace executor
}  // namespace rclcpp

template<typename ResultType, typename ExecutorType>
struct TaskPromise {
    // (MUST) initial_suspend & final_suspend & get_return_object
    // --> standard design
    DispatchAwaiter initial_suspend() { 
        return DispatchAwaiter{&executor}; 
    }
    std::suspend_always final_suspend() noexcept { return {}; }
    Task<ResultType, ExecutorType> get_return_object() {
        return Task{ std::coroutine_handle<TaskPromise>::from_promise(*this) };
    }

    // (MUST) use await_transform to obtain the await <expr>
    // --> specify "TaskAwaiter" to handle --> await <coroutine>
    template<typename _ResultType, typename _ExecutorType>
    TaskAwaiter<_ResultType, _ExecutorType> await_transform(Task<_ResultType, _ExecutorType> &&task) {
        return TaskAwaiter<_ResultType, _ExecutorType>(&executor, std::move(task));
    }
    // --> specify "SleepAwaiter" to handle --> await <time>
    // spk-priority
    template<typename TimedAwaiter>
    SleepAwaiter await_transform(TimedAwaiter awaiter) {
        return SleepAwaiter(&executor, awaiter.duration.count(), awaiter.priority);
    }
    // template<typename _Rep, typename _Period>
    // SleepAwaiter await_transform(std::chrono::duration<_Rep, _Period> &&duration, int priority=0) {
    //     return SleepAwaiter(&executor, std::chrono::duration_cast<std::chrono::milliseconds>(duration).count(), priority);
    // }
    // --> general awaiter to handle --> await <AwaiterImpl>
    template<typename AwaiterImpl>
    requires AwaiterImplRestriction<AwaiterImpl, typename AwaiterImpl::ResultType>
    AwaiterImpl await_transform(AwaiterImpl awaiter) {
        awaiter.install_executor(&executor);
        return awaiter;
    }

    // (MUST) process exception
    void unhandled_exception() {
        {
          std::lock_guard lock(completion_lock);
          result = Result<ResultType>(std::current_exception());
        }
        finish_execution();
        completion.notify_all();
        notify_callbacks();
    }

    void return_value(ResultType value) {
        {
          std::lock_guard lock(completion_lock);
          result = Result<ResultType>(std::move(value));
        }
        finish_execution();
        completion.notify_all();
        notify_callbacks();
    }

    // Pass the private result out
    ResultType get_result() {
        std::unique_lock lock(completion_lock);
        if (!result.has_value()) {
            // blocking for result
            completion.wait(lock);
        }
        // give result or throw on exception
        return result->get_or_throw();
    }

    // Execute the func if value is obtained
    void on_completed(std::function<void(Result<ResultType>)> &&func) {
        std::unique_lock lock(completion_lock);
        if (result.has_value()) {
            auto value = result.value();
            lock.unlock();
            func(value); // --> handle is resumed here, should not do any thing after that
        } else {
            completion_callbacks.push_back(func);
        }
    }

    // init the executor
    void promise_executor_init(void* ros_executer_ptr, void* sub_ptr) {
        std::unique_lock lock(executer_ptr_lock);
        executer_ptr = static_cast<rclcpp::executor::Executor*>(ros_executer_ptr);
        sub_node_ptr = sub_ptr;
        if (!executer_ptr) {
          lock.unlock();
          executor.executor_init(ros_executer_ptr);
          return;
        }
        if (has_returned) {
          // Eager execution may finish before the subscription gets its Task.
          std::lock_guard co_list_lock(executer_ptr->suspend_coroutine_list_mutex_);
          executer_ptr->suspend_coroutine_list_.remove(sub_node_ptr);
        } else {
          std::lock_guard vec_lock(executer_ptr->co_vec_mutex_);
          has_ready_coroutine_vec_id_ = executer_ptr->has_ready_coroutine_vec_.size();
          executer_ptr->has_ready_coroutine_vec_.push_back(true);
          executer_ptr->free_thread_pid_queue_.push(
            std::make_pair(static_cast<int>(gettid()), has_ready_coroutine_vec_id_));
        }
        auto * ros_executor = executer_ptr;
        lock.unlock();
        ros_executor->notify_coroutine_work();
        // Initial dispatch may run the coroutine inline. It must happen only
        // after all bookkeeping is initialized and the promise lock released.
        executor.executor_init(ros_executer_ptr);
    }

    // Indicate Constructor
    TaskPromise() { 
      has_returned = false;
      executer_ptr = NULL;
      has_ready_coroutine_vec_id_ = -1;
    }

    // Indicate Deconstructor
    ~TaskPromise() {}

private:
    std::optional<Result<ResultType>> result;
    std::mutex completion_lock;
    std::condition_variable completion;
    std::list<std::function<void(Result<ResultType>)>> completion_callbacks;
    ExecutorType executor;
    void* sub_node_ptr = nullptr;
    
    // Point to the ROS2 executor
    std::mutex executer_ptr_lock;
    rclcpp::executor::Executor* executer_ptr;
    bool has_returned;
    int has_ready_coroutine_vec_id_;

    void finish_execution() {
        std::unique_lock lock(executer_ptr_lock);
        has_returned = true;
        auto * ros_executor = executer_ptr;
        if (ros_executor) {
          {
            std::lock_guard co_list_lock(ros_executor->suspend_coroutine_list_mutex_);
            ros_executor->suspend_coroutine_list_.remove(sub_node_ptr);
          }
          {
            std::lock_guard vec_lock(ros_executor->co_vec_mutex_);
            if (has_ready_coroutine_vec_id_ >= 0) {
              ros_executor->has_ready_coroutine_vec_[has_ready_coroutine_vec_id_] = false;
            }
          }
        }
        lock.unlock();
        if (ros_executor) ros_executor->notify_coroutine_work();
    }

    void notify_callbacks() {
        std::unique_lock lock(completion_lock);
        auto value = result.value();
        decltype(completion_callbacks) callbacks;
        callbacks.swap(completion_callbacks);
        lock.unlock();
        for (auto & callback : callbacks) callback(value);
    }
};

class RosCoExecutor : public AbstractExecutor {
private:
    rclcpp::executor::Executor* executer_ptr = nullptr;
    std::mutex initialization_mutex_;
    std::shared_ptr<rclcpp::executor::CoroutineLifetime> lifetime_;
    std::function<void()> initial_dispatch_;
public:
    void executor_init(void* ros_executer_ptr) override {
        std::unique_lock lock(initialization_mutex_);
        executer_ptr = static_cast<rclcpp::executor::Executor*>(ros_executer_ptr);
        this->use_re_execute = executer_ptr != nullptr;
        if (executer_ptr) lifetime_ = executer_ptr->coroutine_lifetime;
        auto dispatch = std::move(initial_dispatch_);
        lock.unlock();
        if (dispatch) dispatch();
    }
    void execute(std::function<void()> &&func) override {
        std::unique_lock lock(initialization_mutex_);
        if (!executer_ptr) {
          initial_dispatch_ = std::move(func);
          return;
        }
        lock.unlock();
        func();
    }
    void re_execute(std::function<void()> &&func, int priority) override {
        if (!lifetime_) return;
        std::lock_guard lifetime_lock(lifetime_->mutex);
        if (!lifetime_->accepting_work) return;
        std::unique_lock<std::mutex> lock(this->executer_ptr->coroutine_queue_mutex_);
        // this->executer_ptr->coroutine_queue_.push(func);
        // this->executer_ptr->coroutine_queue_.emplace(priority, std::move(func));
        this->executer_ptr->coroutine_queue_.emplace(priority, std::move(func));

        timeval allftime;
        gettimeofday(&allftime, NULL);
        printf("[coroutine_queue] [PID: %ld] [priority: %ld] Push <%p> [s: %ld] [us: %ld]\n", gettid(), priority, 
                                                                     (void*)&executer_ptr->coroutine_queue_.top().second,  
                                                                     allftime.tv_sec, allftime.tv_usec);
        lock.unlock();
        executer_ptr->notify_coroutine_work();
    }
};
#endif  // RCLCPP__EXECUTOR_HPP_
