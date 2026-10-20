#ifndef __COSPIKE_COROUTINE_HPP__
#define __COSPIKE_COROUTINE_HPP__

#include <concepts>
#include <coroutine>
#include <exception>
#include <iostream>
#include <thread>
#include "cospike/awaiter.hpp"
#include "cospike/coexecutor.hpp"
#include "cospike/task_void.hpp"
#include "cospike/task.hpp"
#include "cospike/result.hpp"

// All message/allocator specializations retain Tasks in the same executor
// queue. Use one mutex across specializations, not a per-callback lock.
namespace cospike {
inline std::mutex retained_task_mutex;
}

#define retTask         Task<int, RosCoExecutor>
#define queueTaskRef    std::queue<retTask>&
#define queueTaskPtr    std::queue<retTask>*

/**
 * SimpleResult: an empty promise_type
 *         promise_type indicate the struct is for coroutine
 */
struct SimpleResult {
    struct promise_type {
        std::suspend_never initial_suspend() { return {}; }
        std::suspend_never final_suspend() noexcept { return {}; }
        SimpleResult get_return_object() { return {}; }
        void return_void() {}
        void unhandled_exception() {}
    };
};

#endif