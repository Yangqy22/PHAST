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

#include "cuda_runtime.h"
#include "device_types.h"
#include "gpu_workload.cuh"

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
        subscription_ = this->create_subscription<std_msgs::msg::String>(true, sub_topic, 10, std::bind(&FinalNodeCPU::co_callback, this, _1));
        // subscription_ = this->create_subscription<std_msgs::msg::String>(false, sub_topic, 10, std::bind(&FinalNodeCPU::callback, this, _1));
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

class ItmidNodeGPU : public rclcpp::Node
{
public:
    ItmidNodeGPU(const std::string node_name, const std::string sub_topic, const std::string pub_topic, int exe_time) 
        : Node(node_name), count_(0), exe_time_(exe_time)
    {                        
        // create_subscription interface for async callback
        // subscription_ = this->create_subscription<std_msgs::msg::String>(true, sub_topic, 10, std::bind(&ItmidNodeGPU::co_callback, this, _1));
        subscription_ = this->create_subscription<std_msgs::msg::String>(false, sub_topic, 10, std::bind(&ItmidNodeGPU::callback, this, _1));
        
        if (pub_topic != "") publisher_ = this->create_publisher<std_msgs::msg::String>(pub_topic, 10);
        this->name_ = node_name;

        this->gpu_record_ = 0;
        this->gpu_await_time_us_ = 0;
        this->gpu_base_latency_us_ = 0;
        this->gpu_best_await_time_us_ = 0;
        this->gpu_best_eval_num_ = -100000;

        srand((unsigned)time(NULL));
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

private:
    timeval ctime, ftime;
    std::string name_;
    size_t count_;
    int exe_time_;
    double latency;

    int gpu_record_;
    long gpu_await_time_us_;
    long gpu_base_latency_us_;
    long gpu_best_await_time_us_;
    long gpu_best_eval_num_;

    void show_time(timeval ftime, timeval ctime, int mark = false) 
    {
        int duration_us = (ftime.tv_sec - starting_time.tv_sec) * 1000000 + (ftime.tv_usec - starting_time.tv_usec);
        long tv_sec = duration_us / 1000000;
        long tv_usec = duration_us - tv_sec * 1000000;
        if (mark) RCLCPP_INFO(this->get_logger(), "[*] [PID: %ld] [Bgn] [s: %ld] [us: %ld]", gettid(), tv_sec, tv_usec);
        else RCLCPP_INFO(this->get_logger(), "[PID: %ld] [Bgn] [s: %ld] [us: %ld]", gettid(), tv_sec, tv_usec);

        duration_us = (ctime.tv_sec - starting_time.tv_sec) * 1000000 + (ctime.tv_usec - starting_time.tv_usec);
        tv_sec = duration_us / 1000000;
        tv_usec = duration_us - tv_sec * 1000000;

        int fc_duration_us = (ctime.tv_sec - ftime.tv_sec) * 1000000 + (ctime.tv_usec - ftime.tv_usec);
        if (mark) RCLCPP_INFO(this->get_logger(), "[*] [PID: %ld] [End] [s: %ld] [us: %ld] [duration_us: %ld]", gettid(), tv_sec, tv_usec, fc_duration_us);
        else RCLCPP_INFO(this->get_logger(), "[PID: %ld] [End] [s: %ld] [us: %ld] [duration_us: %ld]", gettid(), tv_sec, tv_usec, fc_duration_us);
    }

    int callback(const std_msgs::msg::String::SharedPtr msg) {

        gettimeofday(&ftime, NULL);
        int N = 200000000;
        int duration_us;
        long tv_sec, tv_usec;

        //^^^^^ GPU Run It
        int* h_a = new int[N];
        for (int i = 0; i < N; i++) {
            h_a[i] = i + this->gpu_await_time_us_;
        }

        int* d_a; int device = 5;
        cudaSetDevice(device);
        cudaMalloc(&d_a, N * sizeof(int));
        cudaMemcpy(d_a, h_a, N * sizeof(int), cudaMemcpyHostToDevice);

        int threadsPerBlock = 256;
        int blocksPerGrid = (N + threadsPerBlock - 1) / threadsPerBlock;

        cudaStream_t stream1;
        cudaStreamCreate(&stream1);
        kernelCpp(h_a, N, threadsPerBlock, blocksPerGrid, d_a, stream1);
        gettimeofday(&ctime, NULL);
        show_time(ftime, ctime); // First Show Time

        cudaStreamSynchronize(stream1);

        gettimeofday(&ftime, NULL);
        cudaMemcpyAsync(h_a, d_a, N * sizeof(int), cudaMemcpyDeviceToHost, stream1);
        cudaStreamDestroy(stream1);
        delete[] h_a;
        cudaFree(d_a);

        this->gpu_record_ += 1;
        gettimeofday(&ctime, NULL);

        std::string name = this->get_name();
        auto message = std_msgs::msg::String();
        message.data = msg->data;
        if (publisher_) publisher_->publish(message);
        
        show_time(ftime, ctime, true);

        return 1;
    }

    long eval(long base_latency, long cur_latency, long sync_time){
        // The larger the better
        return (base_latency - cur_latency) - sync_time;
    }

    Task<int, RosCoExecutor> co_callback(const std_msgs::msg::String::SharedPtr msg) {
        
        gettimeofday(&ftime, NULL);
        int N = 200000000;
        int duration_us;
        long tv_sec, tv_usec;

        //^^^^^ GPU Run It
        int* h_a = new int[N];
        for (int i = 0; i < N; i++) {
            h_a[i] = i + this->gpu_await_time_us_;
        }

        int* d_a; int device = 5;
        cudaSetDevice(device);
        cudaMalloc(&d_a, N * sizeof(int));
        cudaMemcpy(d_a, h_a, N * sizeof(int), cudaMemcpyHostToDevice);

        int threadsPerBlock = 256;
        int blocksPerGrid = (N + threadsPerBlock - 1) / threadsPerBlock;

        cudaStream_t stream1;
        cudaStreamCreate(&stream1);

        timeval kerftime, kerctime;
        timeval allftime;
        gettimeofday(&allftime, NULL);
        kernelCpp(h_a, N, threadsPerBlock, blocksPerGrid, d_a, stream1);

        if (this->gpu_record_ < 4) {
            // Record Phase
            gettimeofday(&kerftime, NULL);
            gettimeofday(&ctime, NULL);
            RCLCPP_INFO(this->get_logger(), "[Round: %ld] [PID: %ld] Before First Show Time",this->gpu_record_, gettid());
            show_time(ftime, ctime);

            cudaStreamSynchronize(stream1);

            gettimeofday(&ftime, NULL);
            gettimeofday(&kerctime, NULL);
            duration_us = (kerctime.tv_sec  - kerftime.tv_sec ) * 1000000 + 
                          (kerctime.tv_usec - kerftime.tv_usec);
            long all_duration_us = (kerctime.tv_sec - allftime.tv_sec) * 1000000 + (kerctime.tv_usec - allftime.tv_usec);
            
            if (gpu_record_ == 3) {
                this->gpu_await_time_us_ = (this->gpu_await_time_us_ + duration_us) >> 2;
                this->gpu_base_latency_us_ = (this->gpu_base_latency_us_ + all_duration_us) >> 2;
            } else {
                this->gpu_await_time_us_ += duration_us;
                this->gpu_base_latency_us_ += all_duration_us;
            }
            
            // Time Report
            RCLCPP_INFO(this->get_logger(), "[Round: %d] All [us: %ld] Await [us: %ld] Sync [us: %ld]", 
                        this->gpu_record_, all_duration_us, this->gpu_await_time_us_, duration_us);

        } else {
            // Pending Phase
            gettimeofday(&ctime, NULL);
            RCLCPP_INFO(this->get_logger(), "[Round: %ld] [PID: %ld] Before First Show Time",this->gpu_record_, gettid());
            show_time(ftime, ctime);

            // co_await std::chrono::duration<int, std::chrono::microseconds::period>(this->gpu_await_time_us_);
            co_await TimedAwaiter(std::chrono::milliseconds(this->gpu_await_time_us_), this->subscription_->callback_priority);

            gettimeofday(&ftime, NULL);

            gettimeofday(&kerftime, NULL);
            cudaStreamSynchronize(stream1);
            gettimeofday(&kerctime, NULL);
            
            duration_us = (kerctime.tv_sec - kerftime.tv_sec) * 1000000 + (kerctime.tv_usec - kerftime.tv_usec);
            tv_sec = duration_us / 1000000;
            tv_usec = duration_us - tv_sec * 1000000;

            long all_duration_us = (kerctime.tv_sec - allftime.tv_sec) * 1000000 + (kerctime.tv_usec - allftime.tv_usec);

            // Time Report
            RCLCPP_INFO(this->get_logger(), "[Round: %ld] [Base: %ld] [Score: %ld] All [us: %ld] Await [us: %ld] Sync [us: %ld]",
                        this->gpu_record_, this->gpu_base_latency_us_, 
                        eval(this->gpu_base_latency_us_, all_duration_us, duration_us),
                        all_duration_us, this->gpu_await_time_us_, duration_us);
            
            RCLCPP_INFO(this->get_logger(), "[Round: %ld] [Best Score: %ld] Await [us: %ld]",
                        this->gpu_record_, this->gpu_best_eval_num_, this->gpu_best_await_time_us_);
            
            // Auto Modification
            long delta = 10;
            if (duration_us > (this->gpu_base_latency_us_ >> 4) && this->gpu_await_time_us_ + (duration_us >> 1) < this->gpu_base_latency_us_) 
                this->gpu_await_time_us_ += duration_us >> 1;
            else if (duration_us < 20) 
                this->gpu_await_time_us_  = this->gpu_await_time_us_ - (this->gpu_await_time_us_ >> 3);
            else {
                if (eval(this->gpu_base_latency_us_, all_duration_us, duration_us) > this->gpu_best_eval_num_) {
                    this->gpu_best_await_time_us_ = this->gpu_await_time_us_;
                    this->gpu_best_eval_num_ = eval(this->gpu_base_latency_us_, all_duration_us, duration_us);
                    if (duration_us > (this->gpu_base_latency_us_ >> 8)) this->gpu_await_time_us_ += (rand() % (delta));
                } else {
                    this->gpu_await_time_us_ = this->gpu_best_await_time_us_ + (rand() % (delta));
                }
            }
        }
        cudaMemcpyAsync(h_a, d_a, N * sizeof(int), cudaMemcpyDeviceToHost, stream1);
        cudaStreamDestroy(stream1);
        delete[] h_a;
        cudaFree(d_a);

        this->gpu_record_ += 1;
        gettimeofday(&ctime, NULL);

        std::string name = this->get_name();
        auto message = std_msgs::msg::String();
        message.data = msg->data;
        if (publisher_) publisher_->publish(message);
        
        show_time(ftime, ctime, true);
        co_return 1;
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
    auto c1_timer = std::make_shared<cb_chain_demo::StartNode>("Timer_cb1", "c1", 10, 1000);
    auto c1_r_cb1 = std::make_shared<cb_chain_demo::ItmidNodeCPU>("LA_Itmid_cb1", "c1", "r1", 100);
    auto c1_r_cb2 = std::make_shared<cb_chain_demo::FinalNodeCPU>("LA_Itmid_cb2", "c1", "r2", 200);
    auto c1_r_cb3 = std::make_shared<cb_chain_demo::FinalNodeCPU>("LA_Itmid_cb3", "c1", "r3", 300);
    auto c1_r_cb4 = std::make_shared<cb_chain_demo::FinalNodeCPU>("LA_Itmid_cb4", "c1", "r4", 350);
    auto c1_r_cb5 = std::make_shared<cb_chain_demo::ItmidNodeCPU>("key_LA_Itmid_cb5", "c1", "r5", 450);
    auto c1_r_cb6 = std::make_shared<cb_chain_demo::ItmidNodeCPU>("LA_Itmid_cb6", "c1", "r6", 400);

    auto c2_timer = std::make_shared<cb_chain_demo::StartNode>("Timer_cb2", "c2", 10, 30000);
    auto c1_r_cb7 = std::make_shared<cb_chain_demo::ItmidNodeCPU>("LA_Itmid_cb7", "c2", "r7", 100);
    auto c1_r_cb8 = std::make_shared<cb_chain_demo::ItmidNodeGPU>("LB_Final_cb8", "r7", "r8", 150);
    
    // Create executors
    int number_of_threads = 4;
    rclcpp::executors::ExecutorNodelet exec1(rclcpp::executor::ExecutorArgs(), number_of_threads, true);
    std::queue<Task<int, RosCoExecutor> > task_queue;

    // spk-priority
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "priority-based callback scheduling: %s", exec1.callback_priority_enabled ? "Enabled" : "Disabled");
    
    // Allocate callbacks to executors
    exec1.add_node(c1_timer);
    exec1.add_node(c1_r_cb1);
    exec1.add_node(c1_r_cb2);
    exec1.add_node(c1_r_cb3);
    exec1.add_node(c1_r_cb4);
    exec1.add_node(c1_r_cb5);
    exec1.add_node(c1_r_cb6);
    exec1.add_node(c2_timer);
    exec1.add_node(c1_r_cb7);
    exec1.add_node(c1_r_cb8);

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
    exec1.remove_node(c2_timer);
    exec1.remove_node(c1_r_cb7);
    exec1.remove_node(c1_r_cb8);

    // Shutdown
    rclcpp::shutdown();
    return 0;
}
