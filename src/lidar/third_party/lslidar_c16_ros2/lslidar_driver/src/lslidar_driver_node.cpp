#include "lslidar_driver/lslidar_driver.h"

#include <chrono>
#include <thread>

using namespace lslidar_driver;
volatile sig_atomic_t flag = 1;

static void my_handler([[maybe_unused]] int sig) {
    flag = 0;
}

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
