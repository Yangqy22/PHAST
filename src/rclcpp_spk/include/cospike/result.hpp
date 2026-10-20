#ifndef __COSPIKE_RESULT_HPP__
#define __COSPIKE_RESULT_HPP__

#include <exception>

/**
 * Result: Value with Exception
 */
template<typename T>
struct Result {
    explicit Result() = default;
    explicit Result(T &&value) : _value(value) {}
    explicit Result(std::exception_ptr &&exception_ptr) : _exception_ptr(exception_ptr) {}
    T get_or_throw() {
        if (_exception_ptr) {
            std::rethrow_exception(_exception_ptr);
        }
        return _value;
    }
private:
    T _value{};
    std::exception_ptr _exception_ptr;
};

/**
 * Result: Value with Exception (for void)
 */

template<>
struct Result<void> {
    explicit Result() = default;
    explicit Result(std::exception_ptr &&exception_ptr) : _exception_ptr(exception_ptr) {}
    void get_or_throw() {
        if (_exception_ptr) {
            std::rethrow_exception(_exception_ptr);
        }
    }
private:
    std::exception_ptr _exception_ptr;
};

#endif // !__RESULT_HPP__