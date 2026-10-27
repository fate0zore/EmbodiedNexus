#include "lslidar_driver/lslidar_driver.h"

#include <chrono>
#include <thread>

using namespace lslidar_driver;
volatile sig_atomic_t flag = 1;

/**
 * @brief 记录进程退出信号并通知驱动停止运行。
 * @param sig 收到的 POSIX 信号编号。
 * @return 无返回值。
 */
static void my_handler([[maybe_unused]] int sig) {
    flag = 0;
}

/**
 * @brief 初始化 ROS 2 雷达驱动节点并运行其事件循环。
 * @param argc 命令行参数数量。
 * @param argv 命令行参数字符串数组。
 * @return 进程退出状态码；0 表示正常结束或初始化失败已处理。
 */
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    signal(SIGINT, my_handler);
    signal(SIGTERM, my_handler);
    auto node = std::make_shared<lslidar_driver::LslidarDriver>();
    if (!node->initialize()) {
        LS_ERROR << "cannot initialize lslidar driver." << LS_END;
        flag = 0;
        rclcpp::shutdown();
        return 0;
    }

    std::thread shutdown_monitor([] {
        while (rclcpp::ok() && flag != 0) std::this_thread::sleep_for(std::chrono::milliseconds(50));
        if (rclcpp::ok()) rclcpp::shutdown();
    });
    rclcpp::spin(node);
    flag = 0;
    rclcpp::shutdown();
    if (shutdown_monitor.joinable()) shutdown_monitor.join();
    node.reset();
    return 0;
}
