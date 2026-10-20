#ifndef __COSPIKE_EXECUTOR_HPP__
#define __COSPIKE_EXECUTOR_HPP__

#include <queue>
#include <thread>
#include <mutex>
#include <functional>
#include <future>
#include <random>
#include "cospike/io_utils.hpp"

#define NON_BLOCKING_THREAD
#ifndef NON_BLOCKING_THREAD
#define BLOCKING_THREAD
#endif

#define DEBUG if(false)

class AbstractExecutor {
public:
    bool use_re_execute;
    virtual void executor_init(void* ros_executer_ptr) = 0;
    virtual void execute(std::function<void()> &&func) = 0;
    virtual void re_execute(std::function<void()> &&func, int priority) = 0;
    AbstractExecutor(){use_re_execute = false;}
};

class NoopExecutor : public AbstractExecutor {
public:
    void re_execute(std::function<void()> &&func, int priority) override {}
    void executor_init(void* ros_executer_ptr) override {}
    void execute(std::function<void()> &&func) override {
        func();
    }
};

class NewThreadExecutor : public AbstractExecutor {
public:
    void re_execute(std::function<void()> &&func, int priority) override {}
    void executor_init(void* ros_executer_ptr) override {}
    void execute(std::function<void()> &&func) override {
        auto th = std::thread(func);
    #ifdef BLOCKING_THREAD
        th.join(); // blocking
    #else
        th.detach(); // non-blocking
    #endif
        // debug("[NewThreadExecutor] exeute end");
    }
};

class AsyncExecutor : public AbstractExecutor {
public:
    void re_execute(std::function<void()> &&func, int priority) override {}
    void executor_init(void* ros_executer_ptr) override {}
    void execute(std::function<void()> &&func) override {
        auto future = std::async(func);
        // [async] enforces blocking
    }
};

class LooperExecutor : public AbstractExecutor {
private:
    std::condition_variable queue_condition;
    std::mutex queue_lock;
    std::queue<std::function<void()>> executable_queue;
    std::atomic<bool> is_active;
    std::thread work_thread;

    void run_loop() {
        while (is_active.load(std::memory_order_relaxed) || !executable_queue.empty()) {
            std::unique_lock lock(queue_lock);
            if (executable_queue.empty()) {
                queue_condition.wait(lock); 
                if (executable_queue.empty()) {
                    continue;
                }
            }
            auto func = executable_queue.front();
            executable_queue.pop();
            lock.unlock();
            func();
        }
    }

public:
    LooperExecutor() {
        is_active.store(true, std::memory_order_relaxed);
        std::atomic_thread_fence(std::memory_order_release);
        work_thread = std::thread(&LooperExecutor::run_loop, this);
        work_thread.detach();
    }
    ~LooperExecutor() {
        shutdown(false);
        if (work_thread.joinable()) work_thread.join();
    }
    void re_execute(std::function<void()> &&func, int priority) override {}
    void executor_init(void* ros_executer_ptr) override {}
    void execute(std::function<void()> &&func) override {
        std::unique_lock lock(queue_lock);
        if (is_active.load(std::memory_order_relaxed)) {
            executable_queue.push(func);
            lock.unlock();
            queue_condition.notify_one();
        }
    }
    void shutdown(bool wait_for_complete = true) {
        is_active.store(false, std::memory_order_relaxed);
        if (!wait_for_complete) {
            // clear queue.
            std::unique_lock lock(queue_lock);
            decltype(executable_queue) empty_queue;
            std::swap(executable_queue, empty_queue);
            lock.unlock();
        }
        queue_condition.notify_all();
    }
};

class ThreadPoolExecutor : public AbstractExecutor {
private:
    // Global lock
    std::condition_variable queue_condition;
    std::mutex queue_lock;
    // Local queue
    std::vector<std::queue<std::function<void()> > > executable_queue_pool;
    std::vector<std::thread> work_thread_pool;
    std::atomic<bool> is_active;
    int _thread_num;
    void run_loop(int thread_id) {
        while (is_active.load(std::memory_order_relaxed) || !executable_queue_pool[thread_id].empty()) {
            DEBUG printf("[ThreadPoolExecutor] run_loop queue[%d] get task\n", thread_id);
            std::unique_lock lock(queue_lock);
            DEBUG printf("[ThreadPoolExecutor] run_loop thread[%d] lock obtain\n", thread_id);
            if (executable_queue_pool[thread_id].empty()) {
                DEBUG printf("[ThreadPoolExecutor] run_loop [%d] executable_queue.empty()\n", thread_id);
                queue_condition.wait(lock); 
                if (executable_queue_pool[thread_id].empty()) {
                    continue;
                }
            }
            DEBUG printf("[ThreadPoolExecutor] run_loop [%d] auto func = executable_queue.front();\n", thread_id);
            auto func = executable_queue_pool[thread_id].front();
            executable_queue_pool[thread_id].pop();
            lock.unlock();
            // debug("[ThreadPoolExecutor] run_loop run!", thread_id);
            func();
        }
    }
public:
    ThreadPoolExecutor(){}
    void executor_init(int thread_num) {
        // Exception handle
        if (thread_num < 1) {
            // debug("[ThreadPoolExecutor] The thread_num should be larger than zero.");
            throw "[ThreadPoolExecutor] The thread_num should be larger than zero.";
        }
        _thread_num = thread_num;

        // Set the is_active (atomic)
        is_active.store(true, std::memory_order_relaxed);
        std::atomic_thread_fence(std::memory_order_release);

        // Initialize the threadpool with run_loop
        for (int i = 0; i < thread_num; i++) {
            // Warning copy construction is forbidden in std::thread 
            executable_queue_pool.emplace_back();
            work_thread_pool.push_back(std::thread(&ThreadPoolExecutor::run_loop, this, i));
            work_thread_pool.back().detach();
        }
    }
    ~ThreadPoolExecutor() {
        shutdown_all(false);
    }
    int thread_select() {
        std::random_device rd;
        std::default_random_engine eng(rd());
        std::uniform_int_distribution<int> distr(0, _thread_num - 1);
        return distr(eng);
    }
    void re_execute(std::function<void()> &&func, int priority) override {}
    void executor_init(void* ros_executer_ptr) override {}
    void execute(std::function<void()> &&func) override {
        int id = thread_select();
        std::unique_lock lock(queue_lock);
        if (is_active.load(std::memory_order_relaxed)) {
            DEBUG printf("[CoSpike] ThreadPoolExecutor: push func to thread %d\n", id);
            executable_queue_pool[id].push(func);
            DEBUG printf("[CoSpike] ThreadPoolExecutor: executable_queue_pool[%d].size = %ld\n", id, executable_queue_pool[id].size());
            lock.unlock();
            DEBUG printf("[CoSpike] ThreadPoolExecutor: unlock\n");
            queue_condition.notify_all();
        }
    }
    void shutdown_all(bool wait_for_complete = true) {
        is_active.store(false, std::memory_order_relaxed);
        if (!wait_for_complete) {
            // clear queue.
            for (int i = 0; i < _thread_num; i++) {
                auto executable_queue = executable_queue_pool[i];
                std::unique_lock lock(queue_lock);
                decltype(executable_queue) empty_queue;
                std::swap(executable_queue, empty_queue);
                lock.unlock();
            }
        }
        queue_condition.notify_all();
    }
    void shutdown_one(int thread_id, bool wait_for_complete = true) {
        is_active.store(false, std::memory_order_relaxed);
        if (!wait_for_complete) {
            // clear queue.
            auto executable_queue = executable_queue_pool[thread_id];
            std::unique_lock lock(queue_lock);
            decltype(executable_queue) empty_queue;
            std::swap(executable_queue, empty_queue);
            lock.unlock();
        }
        queue_condition.notify_all();
    }
};

/**
 * [Hint] When using SharedThreadPoolExecutor, you should use executor_init
 *        to initialize the ThreadPoolExecutor before using the execute()
 * 
 * Example:
 *          SharedThreadPoolExecutor stpExecutor;           // initialize SharedThreadPoolExecutor
 *          stpExecutor.executor_thread_init(THREAD_SIZE);  // setup the thread pool
 *          auto simpleTask = simple_task();                // call coroutine
 */
class SharedThreadPoolExecutor : public AbstractExecutor {
public:
    static ThreadPoolExecutor sharedThreadPoolExecutor;
    void executor_thread_init(int thread_num) {
        DEBUG printf("[stpExecutor executor_init]: %p\n", (void*) &sharedThreadPoolExecutor);
        sharedThreadPoolExecutor.executor_init(thread_num);
    }
    void executor_init(void* ros_executer_ptr) override {}
    void execute(std::function<void()> &&func) override {
        DEBUG printf("[stpExecutor execute]: %p\n", (void*) &sharedThreadPoolExecutor);
        sharedThreadPoolExecutor.execute(std::move(func));
    }
    void re_execute(std::function<void()> &&func, int priority) override {
        DEBUG printf("[stpExecutor execute]: %p\n", (void*) &sharedThreadPoolExecutor);
        sharedThreadPoolExecutor.re_execute(std::move(func), priority);
    }
};

// Just Define
class RosCoExecutor;

#endif