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

#ifndef RCLCPP__ANY_SUBSCRIPTION_CALLBACK_HPP_
#define RCLCPP__ANY_SUBSCRIPTION_CALLBACK_HPP_

#include <rmw/types.h>
#include <unistd.h>
#include <functional>
#include <memory>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <sys/syscall.h>
#include "rclcpp/allocator/allocator_common.hpp"
#include "rclcpp/function_traits.hpp"
#include "rclcpp/visibility_control.hpp"
#include "cospike/coroutine.hpp"

namespace rclcpp
{

template<typename MessageT, typename Alloc>
class AnySubscriptionCallback
{
  using MessageAllocTraits = allocator::AllocRebind<MessageT, Alloc>;
  using MessageAlloc = typename MessageAllocTraits::allocator_type;
  using MessageDeleter = allocator::Deleter<MessageAlloc, MessageT>;
  using ConstMessageSharedPtr = std::shared_ptr<const MessageT>;
  using MessageUniquePtr = std::unique_ptr<MessageT, MessageDeleter>;

  
  // Normal Callback
  using SharedPtrCallback = std::function<int (const std::shared_ptr<MessageT>)>;
  using SharedPtrWithInfoCallback =
    std::function<int (const std::shared_ptr<MessageT>, const rmw_message_info_t &)>;
  using ConstSharedPtrCallback = std::function<int (const std::shared_ptr<const MessageT>)>;
  using ConstSharedPtrWithInfoCallback =
    std::function<int (const std::shared_ptr<const MessageT>, const rmw_message_info_t &)>;
  using UniquePtrCallback = std::function<int (MessageUniquePtr)>;
  using UniquePtrWithInfoCallback =
    std::function<int (MessageUniquePtr, const rmw_message_info_t &)>;
  
  SharedPtrCallback shared_ptr_callback_;
  SharedPtrWithInfoCallback shared_ptr_with_info_callback_;
  ConstSharedPtrCallback const_shared_ptr_callback_;
  ConstSharedPtrWithInfoCallback const_shared_ptr_with_info_callback_;
  UniquePtrCallback unique_ptr_callback_;
  UniquePtrWithInfoCallback unique_ptr_with_info_callback_;

  
  // Coroutine Callback
  using CoSharedPtrCallback = std::function<retTask (const std::shared_ptr<MessageT>)>;
  using CoSharedPtrWithInfoCallback =
    std::function<retTask (const std::shared_ptr<MessageT>, const rmw_message_info_t &)>;
  using CoConstSharedPtrCallback = std::function<retTask (const std::shared_ptr<const MessageT>)>;
  using CoConstSharedPtrWithInfoCallback =
    std::function<retTask (const std::shared_ptr<const MessageT>, const rmw_message_info_t &)>;
  using CoUniquePtrCallback = std::function<retTask (MessageUniquePtr)>;
  using CoUniquePtrWithInfoCallback =
    std::function<retTask (MessageUniquePtr, const rmw_message_info_t &)>;
  
  CoSharedPtrCallback co_shared_ptr_callback_;
  CoSharedPtrWithInfoCallback co_shared_ptr_with_info_callback_;
  CoConstSharedPtrCallback co_const_shared_ptr_callback_;
  CoConstSharedPtrWithInfoCallback co_const_shared_ptr_with_info_callback_;
  CoUniquePtrCallback co_unique_ptr_callback_;
  CoUniquePtrWithInfoCallback co_unique_ptr_with_info_callback_;

public:
  explicit AnySubscriptionCallback(std::shared_ptr<Alloc> allocator)
  : shared_ptr_callback_(nullptr), shared_ptr_with_info_callback_(nullptr),
    const_shared_ptr_callback_(nullptr), const_shared_ptr_with_info_callback_(nullptr),
    unique_ptr_callback_(nullptr), unique_ptr_with_info_callback_(nullptr)
  {
    message_allocator_ = std::make_shared<MessageAlloc>(*allocator.get());
    allocator::set_allocator_for_deleter(&message_deleter_, message_allocator_.get());
  }

  AnySubscriptionCallback(const AnySubscriptionCallback &) = default;

  // Nomal Callback
  template<
    typename CallbackT,
    typename std::enable_if<
      rclcpp::function_traits::same_function<CallbackT, SharedPtrCallback>::value
    >::type * = nullptr
  >
  void set(CallbackT callback)
  {
    shared_ptr_callback_ = callback;
  }

  template<
    typename CallbackT,
    typename std::enable_if<
      rclcpp::function_traits::same_function<CallbackT, SharedPtrWithInfoCallback>::value
    >::type * = nullptr
  >
  void set(CallbackT callback)
  {
    shared_ptr_with_info_callback_ = callback;
  }

  template<
    typename CallbackT,
    typename std::enable_if<
      rclcpp::function_traits::same_function<CallbackT, ConstSharedPtrCallback>::value
    >::type * = nullptr
  >
  void set(CallbackT callback)
  {
    const_shared_ptr_callback_ = callback;
  }

  template<
    typename CallbackT,
    typename std::enable_if<
      rclcpp::function_traits::same_function<CallbackT, ConstSharedPtrWithInfoCallback>::value
    >::type * = nullptr
  >
  void set(CallbackT callback)
  {
    const_shared_ptr_with_info_callback_ = callback;
  }

  template<
    typename CallbackT,
    typename std::enable_if<
      rclcpp::function_traits::same_function<CallbackT, UniquePtrCallback>::value
    >::type * = nullptr
  >
  void set(CallbackT callback)
  {
    unique_ptr_callback_ = callback;
  }

  template<
    typename CallbackT,
    typename std::enable_if<
      rclcpp::function_traits::same_function<CallbackT, UniquePtrWithInfoCallback>::value
    >::type * = nullptr
  >
  void set(CallbackT callback)
  {
    unique_ptr_with_info_callback_ = callback;
  }

  // Coroutine Callback
  template<
    typename CallbackT,
    typename std::enable_if<
      rclcpp::function_traits::same_function<CallbackT, CoSharedPtrCallback>::value
    >::type * = nullptr
  >
  void set(CallbackT callback)
  {
    co_shared_ptr_callback_ = callback;
  }

  template<
    typename CallbackT,
    typename std::enable_if<
      rclcpp::function_traits::same_function<CallbackT, CoSharedPtrWithInfoCallback>::value
    >::type * = nullptr
  >
  void set(CallbackT callback)
  {
    co_shared_ptr_with_info_callback_ = callback;
  }

  template<
    typename CallbackT,
    typename std::enable_if<
      rclcpp::function_traits::same_function<CallbackT, CoConstSharedPtrCallback>::value
    >::type * = nullptr
  >
  void set(CallbackT callback)
  {
    co_const_shared_ptr_callback_ = callback;
  }

  template<
    typename CallbackT,
    typename std::enable_if<
      rclcpp::function_traits::same_function<CallbackT, CoConstSharedPtrWithInfoCallback>::value
    >::type * = nullptr
  >
  void set(CallbackT callback)
  {
    co_const_shared_ptr_with_info_callback_ = callback;
  }

  template<
    typename CallbackT,
    typename std::enable_if<
      rclcpp::function_traits::same_function<CallbackT, CoUniquePtrCallback>::value
    >::type * = nullptr
  >
  void set(CallbackT callback)
  {
    co_unique_ptr_callback_ = callback;
  }

  template<
    typename CallbackT,
    typename std::enable_if<
      rclcpp::function_traits::same_function<CallbackT, CoUniquePtrWithInfoCallback>::value
    >::type * = nullptr
  >
  void set(CallbackT callback)
  {
    co_unique_ptr_with_info_callback_ = callback;
  }

  // Nomal Callback
  void dispatch(
    std::shared_ptr<MessageT> message, const rmw_message_info_t & message_info)
  {
    if (shared_ptr_callback_) {
      shared_ptr_callback_(message);
    } else if (shared_ptr_with_info_callback_) {
      shared_ptr_with_info_callback_(message, message_info);
    } else if (const_shared_ptr_callback_) {
      const_shared_ptr_callback_(message);
    } else if (const_shared_ptr_with_info_callback_) {
      const_shared_ptr_with_info_callback_(message, message_info);
    } else if (unique_ptr_callback_) {
      auto ptr = MessageAllocTraits::allocate(*message_allocator_.get(), 1);
      MessageAllocTraits::construct(*message_allocator_.get(), ptr, *message);
      unique_ptr_callback_(MessageUniquePtr(ptr, message_deleter_));
    } else if (unique_ptr_with_info_callback_) {
      auto ptr = MessageAllocTraits::allocate(*message_allocator_.get(), 1);
      MessageAllocTraits::construct(*message_allocator_.get(), ptr, *message);
      unique_ptr_with_info_callback_(MessageUniquePtr(ptr, message_deleter_), message_info);
    } else {
      throw std::runtime_error("unexpected message without any callback set");
    }
  }

  void dispatch_intra_process(
    ConstMessageSharedPtr message, const rmw_message_info_t & message_info)
  {
    if (const_shared_ptr_callback_) {
      const_shared_ptr_callback_(message);
    } else if (const_shared_ptr_with_info_callback_) {
      const_shared_ptr_with_info_callback_(message, message_info);
    } else {
      if (unique_ptr_callback_ || unique_ptr_with_info_callback_ ||
        shared_ptr_callback_ || shared_ptr_with_info_callback_)
      {
        throw std::runtime_error("unexpected dispatch_intra_process const shared "
                "message call with no const shared_ptr callback");
      } else {
        throw std::runtime_error("unexpected message without any callback set");
      }
    }
  }

  void dispatch_intra_process(
    MessageUniquePtr message, const rmw_message_info_t & message_info)
  {
    if (shared_ptr_callback_) {
      typename std::shared_ptr<MessageT> shared_message = std::move(message);
      shared_ptr_callback_(shared_message);
    } else if (shared_ptr_with_info_callback_) {
      typename std::shared_ptr<MessageT> shared_message = std::move(message);
      shared_ptr_with_info_callback_(shared_message, message_info);
    } else if (unique_ptr_callback_) {
      unique_ptr_callback_(std::move(message));
    } else if (unique_ptr_with_info_callback_) {
      unique_ptr_with_info_callback_(std::move(message), message_info);
    } else if (const_shared_ptr_callback_ || const_shared_ptr_with_info_callback_) {
      throw std::runtime_error("unexpected dispatch_intra_process unique message call"
              " with const shared_ptr callback");
    } else {
      throw std::runtime_error("unexpected message without any callback set");
    }
  }

  // Coroutine Callback
  void co_dispatch(
    queueTaskPtr qPtr, void* executor_ptr, void* sub_ptr,
    std::shared_ptr<MessageT> message, const rmw_message_info_t & message_info) 
  {
    if (co_shared_ptr_callback_) {
      auto ret = co_shared_ptr_callback_(message);
      ret.task_executor_init(executor_ptr, sub_ptr);
      {
        std::lock_guard<std::mutex> lock(cospike::retained_task_mutex);
        (*qPtr).push(std::move(ret));
      }
    } else if (co_shared_ptr_with_info_callback_) {
      auto ret = co_shared_ptr_with_info_callback_(message, message_info);
      ret.task_executor_init(executor_ptr, sub_ptr);
      {
        std::lock_guard<std::mutex> lock(cospike::retained_task_mutex);
        (*qPtr).push(std::move(ret));
      }
    } else if (co_const_shared_ptr_callback_) {
      auto ret = co_const_shared_ptr_callback_(message);
      ret.task_executor_init(executor_ptr, sub_ptr);
      {
        std::lock_guard<std::mutex> lock(cospike::retained_task_mutex);
        (*qPtr).push(std::move(ret));
      }
    } else if (co_const_shared_ptr_with_info_callback_) {
      auto ret = co_const_shared_ptr_with_info_callback_(message, message_info);
      ret.task_executor_init(executor_ptr, sub_ptr);
      {
        std::lock_guard<std::mutex> lock(cospike::retained_task_mutex);
        (*qPtr).push(std::move(ret));
      }
    } else if (co_unique_ptr_callback_) {
      auto ptr = MessageAllocTraits::allocate(*message_allocator_.get(), 1);
      MessageAllocTraits::construct(*message_allocator_.get(), ptr, *message);
      auto ret = co_unique_ptr_callback_(MessageUniquePtr(ptr, message_deleter_));
      ret.task_executor_init(executor_ptr, sub_ptr);
      {
        std::lock_guard<std::mutex> lock(cospike::retained_task_mutex);
        (*qPtr).push(std::move(ret));
      }
    } else if (co_unique_ptr_with_info_callback_) {
      auto ptr = MessageAllocTraits::allocate(*message_allocator_.get(), 1);
      MessageAllocTraits::construct(*message_allocator_.get(), ptr, *message);
      auto ret = co_unique_ptr_with_info_callback_(MessageUniquePtr(ptr, message_deleter_), message_info);
      ret.task_executor_init(executor_ptr, sub_ptr);
      {
        std::lock_guard<std::mutex> lock(cospike::retained_task_mutex);
        (*qPtr).push(std::move(ret));
      }
    } else {
      throw std::runtime_error("unexpected message without any callback set");
    }
  }

  void co_dispatch_intra_process(
    queueTaskPtr qPtr, void* executor_ptr, void* sub_ptr,
    ConstMessageSharedPtr message, const rmw_message_info_t & message_info)
  {
    if (co_const_shared_ptr_callback_) {
      auto ret = co_const_shared_ptr_callback_(message);
      ret.task_executor_init(executor_ptr, sub_ptr);
      {
        std::lock_guard<std::mutex> lock(cospike::retained_task_mutex);
        (*qPtr).push(std::move(ret));
      }
    } else if (co_const_shared_ptr_with_info_callback_) {
      auto ret = co_const_shared_ptr_with_info_callback_(message, message_info);
      ret.task_executor_init(executor_ptr, sub_ptr);
      {
        std::lock_guard<std::mutex> lock(cospike::retained_task_mutex);
        (*qPtr).push(std::move(ret));
      }
    } else {
      if (co_unique_ptr_callback_ || co_unique_ptr_with_info_callback_ ||
        co_shared_ptr_callback_ || co_shared_ptr_with_info_callback_)
      {
        throw std::runtime_error("unexpected dispatch_intra_process const shared "
                "message call with no const shared_ptr callback");
      } else {
        throw std::runtime_error("unexpected message without any callback set");
      }
    }
  }

  void co_dispatch_intra_process(
    queueTaskPtr qPtr, void* executor_ptr, void* sub_ptr,
    MessageUniquePtr message, const rmw_message_info_t & message_info)
  {
    if (co_shared_ptr_callback_) {
      typename std::shared_ptr<MessageT> shared_message = std::move(message);
      auto ret = co_shared_ptr_callback_(shared_message);
      ret.task_executor_init(executor_ptr, sub_ptr);
      {
        std::lock_guard<std::mutex> lock(cospike::retained_task_mutex);
        (*qPtr).push(std::move(ret));
      }
    } else if (co_shared_ptr_with_info_callback_) {
      typename std::shared_ptr<MessageT> shared_message = std::move(message);
      auto ret = co_shared_ptr_with_info_callback_(shared_message, message_info);
      ret.task_executor_init(executor_ptr, sub_ptr);
      {
        std::lock_guard<std::mutex> lock(cospike::retained_task_mutex);
        (*qPtr).push(std::move(ret));
      }
    } else if (co_unique_ptr_callback_) {
      auto ret = co_unique_ptr_callback_(std::move(message));
      ret.task_executor_init(executor_ptr, sub_ptr);
      {
        std::lock_guard<std::mutex> lock(cospike::retained_task_mutex);
        (*qPtr).push(std::move(ret));
      }
    } else if (co_unique_ptr_with_info_callback_) {
      auto ret = co_unique_ptr_with_info_callback_(std::move(message), message_info);
      ret.task_executor_init(executor_ptr, sub_ptr);
      {
        std::lock_guard<std::mutex> lock(cospike::retained_task_mutex);
        (*qPtr).push(std::move(ret));
      }
    } else if (co_const_shared_ptr_callback_ || co_const_shared_ptr_with_info_callback_) {
      throw std::runtime_error("unexpected dispatch_intra_process unique message call"
              " with const shared_ptr callback");
    } else {
      throw std::runtime_error("unexpected message without any callback set");
    }
  }

  bool use_take_shared_method()
  {
    return   (   const_shared_ptr_callback_ ||    const_shared_ptr_with_info_callback_)
          || (co_const_shared_ptr_callback_ || co_const_shared_ptr_with_info_callback_);
  }

private:
  std::shared_ptr<MessageAlloc> message_allocator_;
  MessageDeleter message_deleter_;
};

}  // namespace rclcpp

#endif  // RCLCPP__ANY_SUBSCRIPTION_CALLBACK_HPP_
