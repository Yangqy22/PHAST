#include <chrono>
#include <memory>
#include <sys/time.h>
#include <unistd.h>
#include <sys/types.h>
#include <errno.h>
#include <sys/syscall.h>
#include <mutex>
#include <coroutine>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executor.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_msgs/msg/string.hpp"
#include "cospike/coroutine.hpp"

#ifndef PHAST_CPU_ONLY
#include "cuda_runtime.h"
#include "device_types.h"
#endif

using namespace std::chrono_literals;
using std::placeholders::_1;
#define gettid() syscall(__NR_gettid)
#define USE_INTRA_PROCESS_COMMS true 
#define DUMMY_LOAD_ITER	1000
#define THREAD_SIZE     3

timeval starting_time;
int dummy_load_calib = 1;

void dummy_load(int load_ms, const char * name_str) {
    int i, j;
    for (j = 0; j < dummy_load_calib * 100; j++)
        for (i = 0 ; i < DUMMY_LOAD_ITER; i++) 
            __asm__ volatile ("nop");
    rclcpp::sleep_for(std::chrono::milliseconds(load_ms));
}

namespace cb_chain_demo
{
class StartNode : public rclcpp::Node
{
public:
    StartNode(const std::string node_name, const std::string pub_topic, int exe_time, int period) 
        : Node(node_name, rclcpp::NodeOptions().use_intra_process_comms(USE_INTRA_PROCESS_COMMS)), count_(0), exe_time_(exe_time), period_(period)
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>(pub_topic, 1);
        name_ = node_name;
        timer_ = this->create_wall_timer(std::chrono::milliseconds(period_), std::bind(&StartNode::timer_callback, this));
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
private:
    std::string name_;
    size_t count_;
    int exe_time_;
    int period_;
    timeval ctime, ftime;

    void show_time(timeval ftime, timeval ctime) 
    {
        int duration_us = (ftime.tv_sec - starting_time.tv_sec) * 1000000 + (ftime.tv_usec - starting_time.tv_usec);
        long tv_sec = duration_us / 1000000;
        long tv_usec = duration_us - tv_sec * 1000000;
        RCLCPP_INFO(this->get_logger(), "[PID: %ld] [Bgn] [s: %ld] [us: %ld]", gettid(), tv_sec, tv_usec);

        duration_us = (ctime.tv_sec - starting_time.tv_sec) * 1000000 + (ctime.tv_usec - starting_time.tv_usec);
        tv_sec = duration_us / 1000000;
        tv_usec = duration_us - tv_sec * 1000000;
        RCLCPP_INFO(this->get_logger(), "[PID: %ld] [End] [s: %ld] [us: %ld]", gettid(), tv_sec, tv_usec);
    }

    void timer_callback()
    {
        gettimeofday(&ftime, NULL);

        dummy_load(exe_time_, this->name_.c_str());

        std::string name = this->get_name();
        auto message = std_msgs::msg::String();
        message.data = std::to_string(count_++);

        gettimeofday(&ctime, NULL);
        
        publisher_->publish(message);
        show_time(ftime, ctime);
    }
};

class FinalNodeCPU : public rclcpp::Node
{
public:
    FinalNodeCPU(const std::string node_name, const std::string sub_topic, const std::string pub_topic, int exe_time) 
        : Node(node_name), count_(0), exe_time_(exe_time), pub_topic_(pub_topic)
    {                        
        // create_subscription interface for sync callback
        // subscription_ = this->create_subscription<std_msgs::msg::String>(true, sub_topic, 10, std::bind(&FinalNodeCPU::co_callback, this, _1));
        subscription_ = this->create_subscription<std_msgs::msg::String>(false, sub_topic, 10, std::bind(&FinalNodeCPU::callback, this, _1));
        if (pub_topic != "") publisher_ = this->create_publisher<std_msgs::msg::String>(pub_topic, 10);
        this->name_ = node_name;
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
private:
    std::string name_;
    std::string pub_topic_;
    size_t count_;
    int exe_time_;
    timeval ctime, ftime;

    void show_time(timeval ftime, timeval ctime) 
    {
        int duration_us = (ftime.tv_sec - starting_time.tv_sec) * 1000000 + (ftime.tv_usec - starting_time.tv_usec);
        long tv_sec = duration_us / 1000000;
        long tv_usec = duration_us - tv_sec * 1000000;
        RCLCPP_INFO(this->get_logger(), "[PID: %ld] [Bgn] [s: %ld] [us: %ld]", gettid(), tv_sec, tv_usec);

        duration_us = (ctime.tv_sec - starting_time.tv_sec) * 1000000 + (ctime.tv_usec - starting_time.tv_usec);
        tv_sec = duration_us / 1000000;
        tv_usec = duration_us - tv_sec * 1000000;

        int fc_duration_us = (ctime.tv_sec - ftime.tv_sec) * 1000000 + (ctime.tv_usec - ftime.tv_usec);
        RCLCPP_INFO(this->get_logger(), "[PID: %ld] [End] [s: %ld] [us: %ld] [duration_us: %ld]", gettid(), tv_sec, tv_usec, fc_duration_us);
    }

    int callback(const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "[callback] Received message: %s", msg->data.c_str());
        gettimeofday(&ftime, NULL);

        /* Blocking Style */
        rclcpp::sleep_for(std::chrono::milliseconds(exe_time_));

        std::string name = this->get_name();
        auto message = std_msgs::msg::String();
        // message.data = msg->data;
        message.data = pub_topic_;

        gettimeofday(&ctime, NULL);
        if (publisher_) {
            publisher_->publish(message);
            // RCLCPP_INFO(this->get_logger(), "[publisher] [name: %ld] [PID: %ld]", name_.c_str(), gettid());
        }
        show_time(ftime, ctime);
        return 1;
    }

    Task<int, RosCoExecutor> co_callback(const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "[co_callback] Received message: %s", msg->data.c_str());
        gettimeofday(&ftime, NULL);

        /* Non-Blocking Style */
        int i, j;
        for (j = 0; j < 10000; j++)
            for (i = 0 ; i < DUMMY_LOAD_ITER; i++) 
                __asm__ volatile ("nop");
                // co_await TimedAwaiter(std::chrono::milliseconds(exe_time_), 0);
                co_await TimedAwaiter(std::chrono::milliseconds(exe_time_), subscription_->callback_priority);
        for (j = 0; j < 10000; j++)
            for (i = 0 ; i < DUMMY_LOAD_ITER; i++) 
                __asm__ volatile ("nop");

        std::string name = this->get_name();
        auto message = std_msgs::msg::String();
        // message.data = msg->data;
        message.data = pub_topic_;
        gettimeofday(&ctime, NULL);
        if (publisher_) {
            publisher_->publish(message);
            // RCLCPP_INFO(this->get_logger(), "[publisher] [name: %ld] [PID: %ld]", name_.c_str(), gettid());
        }
        show_time(ftime, ctime);
        co_return 1;
    }
};

class ItmidNodeCPU : public rclcpp::Node
{
public:
    ItmidNodeCPU(const std::string node_name, const std::string sub_topic, const std::string pub_topic, int exe_time) 
        : Node(node_name), count_(0), exe_time_(exe_time), pub_topic_(pub_topic)
    {                        
        // create_subscription interface for sync callback
        subscription_ = this->create_subscription<std_msgs::msg::String>(false, sub_topic, 10, std::bind(&ItmidNodeCPU::callback, this, _1));
        if (pub_topic != "") publisher_ = this->create_publisher<std_msgs::msg::String>(pub_topic, 10);
        this->name_ = node_name;
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
private:
    std::string name_;
    std::string pub_topic_;
    size_t count_;
    int exe_time_;
    timeval ctime, ftime;

    void show_time(timeval ftime, timeval ctime) 
    {
        int duration_us = (ftime.tv_sec - starting_time.tv_sec) * 1000000 + (ftime.tv_usec - starting_time.tv_usec);
        long tv_sec = duration_us / 1000000;
        long tv_usec = duration_us - tv_sec * 1000000;
        RCLCPP_INFO(this->get_logger(), "[PID: %ld] [Bgn] [s: %ld] [us: %ld]", gettid(), tv_sec, tv_usec);

        duration_us = (ctime.tv_sec - starting_time.tv_sec) * 1000000 + (ctime.tv_usec - starting_time.tv_usec);
        tv_sec = duration_us / 1000000;
        tv_usec = duration_us - tv_sec * 1000000;

        int fc_duration_us = (ctime.tv_sec - ftime.tv_sec) * 1000000 + (ctime.tv_usec - ftime.tv_usec);
        RCLCPP_INFO(this->get_logger(), "[PID: %ld] [End] [s: %ld] [us: %ld] [duration_us: %ld]", gettid(), tv_sec, tv_usec, fc_duration_us);
    }

    int callback(const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "[callback] Received message: %s", msg->data.c_str());
        gettimeofday(&ftime, NULL);

        /* Blocking Style */
        int i, j;
        for (j = 0; j < 10000; j++)
            for (i = 0 ; i < DUMMY_LOAD_ITER; i++) 
                __asm__ volatile ("nop");
        rclcpp::sleep_for(std::chrono::milliseconds(exe_time_));
        for (j = 0; j < 10000; j++)
            for (i = 0 ; i < DUMMY_LOAD_ITER; i++) 
                __asm__ volatile ("nop");

        std::string name = this->get_name();
        auto message = std_msgs::msg::String();
        // message.data = msg->data;
        message.data = pub_topic_;

        gettimeofday(&ctime, NULL);
        if (publisher_) {
            publisher_->publish(message);
            // RCLCPP_INFO(this->get_logger(), "[publisher] [name: %s] [PID: %ld]", name_.c_str(), gettid());
        }
        show_time(ftime, ctime);
        return 1;
    }
};

}

void run_exe(rclcpp::executors::ExecutorNodelet* exe) {
    exe->spin();
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    // Define graph: c1_timer -> c1_callback
    // std::make_shared --> return the ptr & allocate the memory space on heap  
    auto c1_timer = std::make_shared<cb_chain_demo::StartNode>("Timer_cb10", "c1", 10, 1000);
    auto c1_r_cb1 = std::make_shared<cb_chain_demo::ItmidNodeCPU>("LA_Itmid_cb1", "c1", "r1", 300);
    auto c1_r_cb2 = std::make_shared<cb_chain_demo::ItmidNodeCPU>("LA_Itmid_cb2", "c1", "r2", 300);
    auto c1_r_cb3 = std::make_shared<cb_chain_demo::ItmidNodeCPU>("LA_Itmid_cb3", "c1", "r3", 300);
    auto c1_r_cb4 = std::make_shared<cb_chain_demo::ItmidNodeCPU>("LA_Itmid_cb4", "c1", "r4", 150);
    auto c1_r_cb5 = std::make_shared<cb_chain_demo::ItmidNodeCPU>("key_LA_Itmid_cb5", "c1", "r5", 150);
    auto c1_r_cb6 = std::make_shared<cb_chain_demo::ItmidNodeCPU>("LA_Itmid_cb6", "c1", "r6", 150);

    auto c1_r_cb7 = std::make_shared<cb_chain_demo::FinalNodeCPU>("LB_Final_cb7", "r1", "r7", 150);
    auto c1_r_cb8 = std::make_shared<cb_chain_demo::FinalNodeCPU>("LB_Final_cb8", "r2", "r8", 150);
    auto c1_r_cb9 = std::make_shared<cb_chain_demo::FinalNodeCPU>("LB_Final_cb9", "r3", "r9", 150);
    auto c1_r_cb10 = std::make_shared<cb_chain_demo::FinalNodeCPU>("LB_Final_cb10", "r4", "r10", 300);
    auto c1_r_cb11 = std::make_shared<cb_chain_demo::ItmidNodeCPU>("critical_LB_Final_cb11", "r5", "r11", 300);
    auto c1_r_cb12 = std::make_shared<cb_chain_demo::FinalNodeCPU>("LB_Final_cb12", "r6", "r12", 300);

    auto c1_r_cb13 = std::make_shared<cb_chain_demo::FinalNodeCPU>("LB_Final_cb13", "r4", "r13", 300);
    auto c1_r_cb14 = std::make_shared<cb_chain_demo::FinalNodeCPU>("LB_Final_cb14", "r3", "r14", 150);
    
    // Create executors
    int number_of_threads = 4;
    rclcpp::executors::ExecutorNodelet exec1(rclcpp::executor::ExecutorArgs(), number_of_threads, true);
    std::queue<Task<int, RosCoExecutor> > task_queue;

    // spk-priority
    exec1.enable_priority_scheduling();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "priority-based callback scheduling: %s", exec1.callback_priority_enabled ? "Enabled" : "Disabled");
    
    // Allocate callbacks to executors
    exec1.add_node(c1_timer);
    exec1.add_node(c1_r_cb1);
    exec1.add_node(c1_r_cb2);
    exec1.add_node(c1_r_cb3);
    exec1.add_node(c1_r_cb4);
    exec1.add_node(c1_r_cb5);
    exec1.add_node(c1_r_cb6);
    exec1.add_node(c1_r_cb7);
    exec1.add_node(c1_r_cb8);
    exec1.add_node(c1_r_cb9);
    exec1.add_node(c1_r_cb10);
    exec1.add_node(c1_r_cb11);
    exec1.add_node(c1_r_cb12);
    exec1.add_node(c1_r_cb13);
    exec1.add_node(c1_r_cb14);

    exec1.set_priority(c1_timer->timer_, 40);
    exec1.set_priority(c1_r_cb1->subscription_, 15);
    exec1.set_priority(c1_r_cb2->subscription_, 16);
    exec1.set_priority(c1_r_cb3->subscription_, 17);
    exec1.set_priority(c1_r_cb4->subscription_, 18);
    // Fig. 9 / Section VI-A: Cb5 is the highest-priority subscription.
    exec1.set_priority(c1_r_cb5->subscription_, 39);
    exec1.set_priority(c1_r_cb6->subscription_, 19);
    exec1.set_priority(c1_r_cb7->subscription_, 25);
    exec1.set_priority(c1_r_cb8->subscription_, 26);
    exec1.set_priority(c1_r_cb9->subscription_, 27);
    exec1.set_priority(c1_r_cb10->subscription_, 28);
    exec1.set_priority(c1_r_cb11->subscription_, 38);
    exec1.set_priority(c1_r_cb12->subscription_, 29);
    exec1.set_priority(c1_r_cb13->subscription_, 31);
    exec1.set_priority(c1_r_cb14->subscription_, 30);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Timer_callback->priority: %d", c1_timer->timer_->callback_priority);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Regular_callback11->priority: %d", c1_r_cb11->subscription_->callback_priority);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Regular_callback5->priority: %d", c1_r_cb5->subscription_->callback_priority);

    // Record Starting Time:
    gettimeofday(&starting_time, NULL);

    //^^^^ Spin lock: Single-Executor Mode
    exec1.spin();

    // Remove Extra-node
    exec1.remove_node(c1_timer);
    exec1.remove_node(c1_r_cb1);
    exec1.remove_node(c1_r_cb2);
    exec1.remove_node(c1_r_cb3);
    exec1.remove_node(c1_r_cb4);
    exec1.remove_node(c1_r_cb5);
    exec1.remove_node(c1_r_cb6);
    exec1.remove_node(c1_r_cb7);
    exec1.remove_node(c1_r_cb8);
    exec1.remove_node(c1_r_cb9);
    exec1.remove_node(c1_r_cb10);
    exec1.remove_node(c1_r_cb11);
    exec1.remove_node(c1_r_cb12);
    exec1.remove_node(c1_r_cb13);
    exec1.remove_node(c1_r_cb14);

    // Shutdown
    rclcpp::shutdown();
    return 0;
}
