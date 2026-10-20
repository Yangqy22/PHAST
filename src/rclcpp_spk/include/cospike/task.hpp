#ifndef __COSPIKE_TASK_HPP__
#define __COSPIKE_TASK_HPP__

#include <concepts> // c++20
#include <coroutine> // c++20
#include <exception>
#include <iostream>
#include <thread>
#include <future>
#include <list>
#include <optional>
#include <functional>
#include <mutex>
#include <unistd.h>
#include <sys/types.h>
#include <errno.h>
#include <sys/syscall.h>
#include "cospike/coexecutor.hpp"
#include "cospike/awaiter.hpp"
#include "cospike/io_utils.hpp"
#include "cospike/result.hpp"

template<typename AwaiterImpl, typename R>
concept AwaiterImplRestriction = std::is_base_of<Awaiter<R>, AwaiterImpl>::value;

/**
 * TaskPromise: take result into consideration
 */
template<typename ResultType, typename ExecutorType>
struct TaskPromise;

/**
 * Task: the general return value type
 */
template<typename ResultType, typename ExecutorType = NoopExecutor>
struct Task {
    using promise_type = TaskPromise<ResultType, ExecutorType>;

    // [sync] Used to obtain results
    ResultType get_result() {
        // --> TaskPromise.get_result();
        return handle.promise().get_result();
    }

    void task_executor_init(void* ros_executer_ptr, void* sub_ptr) {
        handle.promise().promise_executor_init(ros_executer_ptr, sub_ptr);
    }

    // [async] Used to obtain results
    Task &then(std::function<void(ResultType)> &&func) {
        handle.promise().on_completed([func](auto result) {
            try {
                func(result.get_or_throw());
            } catch (std::exception &e) {
                // ...
            }
        });
        return *this;
    }

    // Used to obtain exceptions
    Task &catching(std::function<void(std::exception &)> &&func) {
          handle.promise().on_completed([func](auto result) {
              try {
                  result.get_or_throw();
              } catch (std::exception &e) {
                  func(e);
              }
          });
        return *this;
    }

    // Execute anyway
    Task &finally(std::function<void()> &&func) {
        handle.promise().on_completed([func](auto result) { func(); });
        return *this;
    }
    explicit Task(std::coroutine_handle<promise_type> handle) noexcept: handle(handle) {}
    Task(Task &&task) noexcept: handle(std::exchange(task.handle, {})) {}
    Task(Task &) = delete;
    Task &operator=(Task &) = delete;
    ~Task() {
        if (handle) handle.destroy();
    }
private:
    std::coroutine_handle<promise_type> handle;
};
#endif