#ifndef __COSPIKE_AWAITER_HPP__
#define __COSPIKE_AWAITER_HPP__

#include <concepts>
#include <coroutine>
#include <exception>
#include <iostream>
#include <thread>
#include <list>
#include <optional>
#include "cospike/coexecutor.hpp"
#include "cospike/io_utils.hpp"
#include "cospike/scheduler.hpp"
#include "cospike/result.hpp"

/**
 * General Awaiter
 */
template<typename R>
struct Awaiter {
public:
    using ResultType = R;
    bool await_ready() const { return false; }
    void await_suspend(std::coroutine_handle<> handle) {
        this->_handle = handle;
        after_suspend();
    }
    R await_resume() {
        before_resume();
        return _result->get_or_throw();
    }
    void resume(R value) {
        dispatch([this, value]() {
            _result = Result<R>(static_cast<R>(value));
            _handle.resume();
        });
    }
    void resume_unsafe() {
        dispatch([this]() { _handle.resume(); });
    }
    void resume_exception(std::exception_ptr &&e) {
        dispatch([this, e]() {
            _result = Result<R>(static_cast<std::exception_ptr>(e));
            _handle.resume();
        });
    }
    void install_executor(AbstractExecutor *executor) {
        _executor = executor;
    }
protected:
    std::optional<Result<R>> _result{};
    virtual void after_suspend() {}
    virtual void before_resume() {}
private:
    AbstractExecutor *_executor = nullptr;
    std::coroutine_handle<> _handle = nullptr;
    void dispatch(std::function<void()> &&f) {
        if (_executor) {
            _executor->execute(std::move(f));
        } else {
            f();
        }
    }
};

/**
 * General Awaiter (for void)
 */
template<>
struct Awaiter<void> {
public:
    using ResultType = void;
    bool await_ready() { return false; }
    void await_suspend(std::coroutine_handle<> handle) {
        this->_handle = handle;
        after_suspend();
    }
    void await_resume() {
        before_resume();
        _result->get_or_throw();
    }
    void resume() {
        dispatch([this]() {
            _result = Result<void>();
            _handle.resume();
        });
    }
    void resume_exception(std::exception_ptr &&e) {
        dispatch([this, e]() {
            _result = Result<void>(static_cast<std::exception_ptr>(e));
            _handle.resume();
        });
    }
    void install_executor(AbstractExecutor *executor) {
        _executor = executor;
    }
    // specific interfaces
    virtual void after_suspend() {}
    virtual void before_resume() {}
private:
    std::optional<Result<void>> _result{};
    AbstractExecutor *_executor = nullptr;
    std::coroutine_handle<> _handle = nullptr;
    void dispatch(std::function<void()> &&f) {
        if (_executor) {
            _executor->execute(std::move(f));
        } else {
            f();
        }
    }
};

/**
 * SimpleAwaiter: Need to define three functions
 * - [1] await_ready
 * - [2] await_suspend
 * - [3] await_resume
 */
struct SimpleAwaiter {
    int value;
    bool await_ready() {
        // suspend the coroutine
        // - true:  ready, no need to suspend 
        // - false: not ready, need to suspend
        return false;
        // [false] -> go await_suspend()
    }
    void await_suspend(std::coroutine_handle<> coroutine_handle) {
        // what to do when suspending
        using namespace std::literals;
        std::async([=](){
            using namespace std::chrono_literals;
            // sleep
            std::this_thread::sleep_for(1s); 
            // recovery
            coroutine_handle.resume();
            // -> go await_resume()
        });
    }
    int await_resume() {
        // triggered after coroutine resume
        return value;
    }
};

/**
 * DispatchAwaiter: Enable scheduling @initial_suspend()
 */
struct DispatchAwaiter {
    explicit DispatchAwaiter(AbstractExecutor *executor) noexcept
        : _executor(executor) {}
    bool await_ready() const { return false; }
    void await_suspend(std::coroutine_handle<> handle) const {
        _executor->execute([handle]() {
            handle.resume();
        });
    }
    void await_resume() {}
    private:
    AbstractExecutor *_executor;
};

template<typename ResultType, typename ExecutorType> struct Task;

/**
 * TaskPromise: regarded as a coroutine scheduler
 */
template<typename ResultType, typename ExecutorType>
struct TaskAwaiter {
    // Constructor Maintaining
    explicit TaskAwaiter(AbstractExecutor *executor, Task<ResultType, ExecutorType> &&task) noexcept
      : _executor(executor), task(std::move(task)) {}
    TaskAwaiter(TaskAwaiter &&completion) noexcept
        : _executor(completion._executor), task(std::exchange(completion.task, {})) {}
    TaskAwaiter(TaskAwaiter &) = delete;
    TaskAwaiter &operator=(TaskAwaiter &) = delete;
    constexpr bool await_ready() const noexcept {
        return false;
    }
    void await_suspend(std::coroutine_handle<> handle) noexcept {
        task.finally([handle, this]() {
            // Use the executor to process handler
            // instead of perform it by awaiter itself
            _executor->execute([handle]() {
                handle.resume();
            });
        });
    }
    ResultType await_resume() noexcept {
        return task.get_result();
    }
private:
    Task<ResultType, ExecutorType> task;
    AbstractExecutor *_executor;
};

/**
 * SleepAwaiter: unblocking awaiter for sleep
 */
struct SleepAwaiter {
    explicit SleepAwaiter(AbstractExecutor *executor, long long duration, int priority) noexcept
        : _executor(executor), _duration(duration) , _priority(priority) {}
    bool await_ready() const { return false; }
    void await_suspend(std::coroutine_handle<> handle) const {
        static Scheduler scheduler;
        scheduler.execute([this, handle]() {
            if (_executor->use_re_execute) {
                _executor->re_execute([handle]() { handle.resume(); }, _priority);
            } else {
                _executor->execute([handle]() { handle.resume(); });
            }
        }, _duration);
    }
    void await_resume() noexcept {
        // [Hint] executed by the Scheduler thread
    }
private:
    AbstractExecutor *_executor;
    long long _duration;
    int _priority;
};

template<typename ValueType> struct Channel;

/**
 * WriterAwaiter & ReaderAwaiter
 */
template<typename ValueType>
struct WriterAwaiter : public Awaiter<void> {
    Channel<ValueType> *channel;
    ValueType _value;
    WriterAwaiter(Channel<ValueType> *channel, ValueType value) : channel(channel), _value(value) {}
    WriterAwaiter(WriterAwaiter &&other) noexcept
        :   Awaiter(other),
            channel(std::exchange(other.channel, nullptr)),
            _value(other._value) {}
    
    void after_suspend() override {
        channel->try_push_writer(this);
    }
    void before_resume() override {
        channel->check_closed();
        channel = nullptr;
    }
    ~WriterAwaiter() {
        if (channel) channel->remove_writer(this);
    }
};

template<typename ValueType>
struct ReaderAwaiter : public Awaiter<ValueType> {
    Channel<ValueType> *channel;
    ValueType *p_value = nullptr;
    explicit ReaderAwaiter(Channel<ValueType> *channel) : Awaiter<ValueType>(), channel(channel) {}
    ReaderAwaiter(ReaderAwaiter &&other) noexcept
        : Awaiter<ValueType>(other),
            channel(std::exchange(other.channel, nullptr)),
            p_value(std::exchange(other.p_value, nullptr)) {}
    void after_suspend() override {
        channel->try_push_reader(this);
    }
    void before_resume() override {
        channel->check_closed();
        if (p_value) {
            *p_value = this->_result->get_or_throw();
        }
        channel = nullptr;
    }
    ~ReaderAwaiter() {
        if (channel) channel->remove_reader(this);
    }
};

#endif