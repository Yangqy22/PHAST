#include <rclcpp/rclcpp.hpp>
#include <rclcpp/strategies/allocator_memory_strategy.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/empty.hpp>
#include <future>
#include <iostream>
#include <stdexcept>

using Any = rclcpp::executor::AnyExecutable;
using WeakNodes = rclcpp::memory_strategy::MemoryStrategy::WeakNodeList;
using StrategyBase = rclcpp::memory_strategies::allocator_memory_strategy::AllocatorMemoryStrategy<>;
struct TestExecutor : rclcpp::executor::Executor {
  void spin() override {}
  using Executor::get_next_ready_executable;
  using Executor::get_next_executable;
  using Executor::execute_any_executable;
  using Executor::wait_for_work;
  using Executor::wait_free_wfk;
  using Executor::spinning;
};
struct Candidates : StrategyBase {
  rclcpp::SubscriptionBase::SharedPtr subscription;
  rclcpp::ServiceBase::SharedPtr service;
  rclcpp::Node::SharedPtr sub_node, service_node;
  void get_next_subscription(std::list<void*> &, std::mutex *, Any & out,
                             const WeakNodes &) override {
    out.subscription = subscription;
    out.callback_group = sub_node->get_node_base_interface()->get_default_callback_group();
    out.node_base = sub_node->get_node_base_interface();
  }
  void get_next_service(Any & out, const WeakNodes &) override {
    out.service = service;
    out.callback_group = service_node->get_node_base_interface()->get_default_callback_group();
    out.node_base = service_node->get_node_base_interface();
  }
};
void require(bool value, const char * message) {
  if (!value) throw std::runtime_error(message);
}
Task<int, RosCoExecutor> immediate(const std_msgs::msg::String::SharedPtr) { co_return 1; }
Task<int, NoopExecutor> eager() { co_return 1; }

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  {
    TestExecutor executor;
    auto strategy = std::make_shared<Candidates>();
    strategy->sub_node = std::make_shared<rclcpp::Node>("regression_sub");
    strategy->service_node = std::make_shared<rclcpp::Node>("regression_service");
    strategy->subscription = strategy->sub_node->create_subscription<std_msgs::msg::String>(
      true, "empty_topic", 10, immediate);
    strategy->service = strategy->service_node->create_service<std_srvs::srv::Empty>(
      "unused_service", [](std::shared_ptr<std_srvs::srv::Empty::Request>,
                           std::shared_ptr<std_srvs::srv::Empty::Response>) {});
    strategy->subscription->callback_priority = 26;
    strategy->service->callback_priority = 0;
    executor.add_node(strategy->sub_node);
    executor.add_node(strategy->service_node);
    executor.set_memory_strategy(strategy);
    executor.enable_priority_scheduling();
    auto group = strategy->sub_node->get_node_base_interface()->get_default_callback_group();
    auto service_group = strategy->service_node->get_node_base_interface()->get_default_callback_group();
    {
      Any selected;
      require(executor.get_next_ready_executable(selected), "subscription not selected");
      require(selected.subscription == strategy->subscription && !selected.service,
              "wrong priority winner");
      require(selected.callback_group == group && selected.node_base == strategy->sub_node->get_node_base_interface(),
              "rejected service corrupted winner metadata");
      require(!executor.suspend_coroutine_list_.empty(), "reservation missing");
    }
    require(executor.suspend_coroutine_list_.empty(), "cancelled selection leaked reservation");
    executor.spinning = true;
    {
      Any selected;
      require(executor.get_next_ready_executable(selected), "second selection failed");
      executor.execute_any_executable(selected); // Real rcl_take on a topic with no publisher.
      require(executor.suspend_coroutine_list_.empty(), "failed take leaked reservation");
    }
    require(group->can_be_taken_from() && service_group->can_be_taken_from(), "callback group left busy");
    for (bool exception : {false, true}) {
      executor.suspend_coroutine_list_.push_back(strategy->subscription.get());
      TaskPromise<int, RosCoExecutor> promise;
      promise.promise_executor_init(&executor, strategy->subscription.get());
      if (exception) {
        try { throw std::runtime_error("expected callback failure"); }
        catch (...) { promise.unhandled_exception(); }
      } else promise.return_value(1);
      require(executor.suspend_coroutine_list_.empty(), "completed task leaked reservation");
      require(!executor.has_ready_coroutine_vec_.back(), "completed task stayed active");
    }
    // The coroutine can finish eagerly before promise_executor_init is called.
    executor.suspend_coroutine_list_.push_back(strategy->subscription.get());
    auto task = eager();
    task.task_executor_init(&executor, strategy->subscription.get());
    require(executor.suspend_coroutine_list_.empty(), "eager completion leaked reservation");
    executor.spinning = false;
  }
  {
    TestExecutor executor;
    auto node = std::make_shared<rclcpp::Node>("regression_readiness");
    auto subscription = node->create_subscription<std_msgs::msg::String>(
      true, "never_published", 10, immediate);
    subscription->callback_priority = 26;
    executor.add_node(node);
    executor.enable_priority_scheduling();
    executor.wait_free_wfk(std::chrono::nanoseconds(0));
    Any selected;
    require(!executor.get_next_ready_executable(selected),
            "populated wait-set handles were mistaken for ready data");
    require(executor.suspend_coroutine_list_.empty(), "readiness poll created false reservation");
  }
  {
    TestExecutor executor;
    executor.spinning = true;
    executor.wait_for_work(std::chrono::nanoseconds(0)); // Drain construction notifications.
    std::promise<void> entered;
    auto entered_wait = entered.get_future();
    auto waiting = std::async(std::launch::async, [&] {
      Any selected;
      entered.set_value();
      return executor.get_next_executable(selected, std::chrono::seconds(2));
    });
    entered_wait.wait();
    require(waiting.wait_for(std::chrono::milliseconds(100)) == std::future_status::timeout,
            "executor did not block in ROS wait");
    bool lock_free = executor.co_vec_mutex_.try_lock();
    if (lock_free) executor.co_vec_mutex_.unlock();
    RosCoExecutor dispatcher;
    dispatcher.executor_init(&executor);
    dispatcher.re_execute([] {}, 42);
    bool woke = waiting.wait_for(std::chrono::milliseconds(500)) == std::future_status::ready;
    bool found_coroutine = waiting.get();
    require(found_coroutine, "woken executor did not select queued coroutine");
    require(lock_free, "ROS wait holds coroutine state mutex");
    require(woke, "coroutine notification did not wake ROS wait");
    executor.spinning = false;
  }
  {
    std::promise<void> started;
    auto began = started.get_future();
    std::atomic<bool> finished{false};
    {
      Scheduler scheduler;
      scheduler.execute([&] {
        started.set_value();
        std::this_thread::sleep_for(std::chrono::milliseconds(60));
        finished = true;
      }, 0);
      require(began.wait_for(std::chrono::seconds(2)) == std::future_status::ready,
              "scheduler did not start callback");
    }
    require(finished, "scheduler destructor failed to join running callback");
  }
  {
    RosCoExecutor delayed_dispatcher;
    {
      TestExecutor executor;
      delayed_dispatcher.executor_init(&executor);
    }
    delayed_dispatcher.re_execute([] {}, 1); // Must not touch the destroyed executor.
  }
  rclcpp::shutdown();
  std::cout << "PASS: real readiness, candidate ownership, cancellation, failed take, completion, exception, eager completion, lock, wakeup and scheduler shutdown\n";
}
