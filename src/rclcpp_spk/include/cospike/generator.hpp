#ifndef __COSPIKE_GENERATOR_HPP__
#define __COSPIKE_GENERATOR_HPP__

#include <concepts>
#include <coroutine>
#include <exception>
#include <iostream>
#include <thread>
#include <future>

/**
 * Generator: more than an empty promise_type
 */
template<typename T>
struct Generator {
    // Exception Handling
    class ExhaustedException: std::exception { };
    // Claim the class as a promise_type
    struct promise_type {
        T value;
        bool is_ready = false; // use to mark whether the value is consumed
        std::suspend_always initial_suspend() { return {}; };
        std::suspend_always final_suspend() noexcept { return {}; }
        // For co_yield
        std::suspend_always yield_value(T value) {
            this->value = value;
            is_ready = true;
            return {};
        }
        // For co_await
        std::suspend_always await_transform(T value) {
            this->value = value;
            is_ready = true;
            return {};
        }
        void unhandled_exception() { }
        Generator get_return_object() {
            return Generator{ std::coroutine_handle<promise_type>::from_promise(*this) };
        }
        void return_void() { }
    };

    // Coroutine handler
    std::coroutine_handle<promise_type> handle;

    // Copy Constructor is forbidden
    explicit Generator(std::coroutine_handle<promise_type> handle) noexcept
        : handle(handle) {}
    Generator(Generator &) = delete;
    Generator &operator=(Generator &) = delete;

    // Move Constructor is allowed (only)
    Generator(Generator &&generator) noexcept
        : handle(std::exchange(generator.handle, {})) {}
    
    ~Generator() {
        if (handle) handle.destroy();
    }

    // Methods
    bool has_next();
    T next();

    // Stream: from list to stream (by fold expression in C++17) 
    template<typename ...TArgs>
    Generator static from(TArgs ...args) {
        (co_yield args, ...);
    }
    // Stream: select the first n numbers
    Generator take(int n) {
        int i = 0;
        while (i++ < n && has_next()) {
            co_yield next();
        }
    }
    // Stream: take + filter
    template<typename F>
    Generator take_while(F f) {
        while (has_next()) {
            T value = next();
            if (f(value)) co_yield value;
            else break;
        }
    }

    // Functor: map
    template<typename F>
    Generator<std::invoke_result_t<F,T>> map(F f) {
        while (has_next()) co_yield f(next());
    }
    // Functor: flat_map
    template<typename F>
    std::invoke_result_t<F, T> flat_map(F f) {
        while (has_next()) {
            auto generator = f(next());
            while (generator.has_next()) {
                co_yield generator.next();
            }
        }
    }
    // Functor: for_each
    template<typename F>
    void for_each(F f) {
        while (has_next()) {
            f(next());
        }
    }
    // Functor: fold
    template<typename R, typename F>
    R fold(R initial, F f) {
        R acc = initial;
        while (has_next()) {
            acc = f(acc, next());
        }
        return acc;
    }
};

// Validation request for the generator
template<typename T>
bool Generator<T>::has_next() {
    if (handle.done()) return false;
    // If the coroutine is still alive 
    // and is suspending, resume it!
    if (!handle.promise().is_ready) handle.resume();
    if (handle.done()) return false;
    else return true;
}

// The hand use to obtain the value
template<typename T>
T Generator<T>::next() {
    if (has_next()) {
        // Get the result & lock it
        handle.promise().is_ready = false;
        return handle.promise().value;
    }
    throw ExhaustedException();
}

#endif