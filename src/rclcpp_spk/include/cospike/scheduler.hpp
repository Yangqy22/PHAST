#ifndef __COSPIKE_SCHEDULER_HPP__
#define __COSPIKE_SCHEDULER_HPP__

#include <thread>
#include <mutex>
#include <functional>
#include <future>
#include <queue>
#include <condition_variable>
#include <chrono>

#include "cospike/io_utils.hpp"

class DelayedExecutable {
public:
    DelayedExecutable(std::function<void()> &&func, long long delay) : func(std::move(func)) {
        using namespace std;
        using namespace std::chrono;
        auto now = system_clock::now();
        auto current = duration_cast<milliseconds>(now.time_since_epoch()).count();

        scheduled_time = current + delay;
    }

    long long delay() const {
        using namespace std;
        using namespace std::chrono;

        auto now = system_clock::now();
        auto current = duration_cast<milliseconds>(now.time_since_epoch()).count();
        return scheduled_time - current;
    }

    long long get_scheduled_time() const {
        return scheduled_time;
    }

    void operator()() {
        func();
    }

private:
    long long scheduled_time;
    std::function<void()> func;
};

class DelayedExecutableCompare {
public:
    bool operator()(DelayedExecutable &left, DelayedExecutable &right) {
        return left.get_scheduled_time() > right.get_scheduled_time();
    }
};

class Scheduler {
private:
    std::condition_variable queue_condition;
    std::mutex queue_lock;
    std::priority_queue<DelayedExecutable, std::vector<DelayedExecutable>, DelayedExecutableCompare> executable_queue;
    std::atomic<bool> is_active;
    std::thread work_thread;

    void run_loop() {
        for (;;) {
            std::unique_lock lock(queue_lock);
            queue_condition.wait(lock, [this] {
                return !is_active.load(std::memory_order_relaxed) || !executable_queue.empty();
            });
            if (executable_queue.empty()) return;
            auto executable = executable_queue.top();
            long long delay = executable.delay();
            if (delay > 0) {
                // Re-evaluate after every notification, including shutdown or
                // insertion of an earlier deadline.
                queue_condition.wait_for(lock, std::chrono::milliseconds(delay));
                continue;
            }
            executable_queue.pop();
            lock.unlock();
            executable();
        }
    }

public:
    Scheduler() {
        is_active.store(true, std::memory_order_relaxed);
        std::atomic_thread_fence(std::memory_order_release);
        work_thread = std::thread(&Scheduler::run_loop, this);
    }
    ~Scheduler() {
        shutdown(false);
        if (work_thread.joinable()) work_thread.join();
    }
    void execute(std::function<void()> &&func, long long delay) {
        delay = delay < 0 ? 0 : delay;
        std::unique_lock lock(queue_lock);
        if (is_active.load(std::memory_order_relaxed)) {
            bool need_notify = executable_queue.empty() || executable_queue.top().delay() > delay;
            executable_queue.push(DelayedExecutable(std::move(func), delay));
            lock.unlock();
            if (need_notify) {
                queue_condition.notify_one();
            }
        }
    }
    void shutdown(bool wait_for_complete = true) {
        {
            std::lock_guard lock(queue_lock);
            is_active.store(false, std::memory_order_relaxed);
            if (!wait_for_complete) {
                decltype(executable_queue) empty_queue;
                std::swap(executable_queue, empty_queue);
            }
        }
        queue_condition.notify_all();
    }
};
#endif
