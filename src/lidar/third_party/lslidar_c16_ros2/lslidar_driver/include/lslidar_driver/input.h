/******************************************************************************
 * This file is part of lslidar_cx driver.
 *
 * Copyright 2022 LeiShen Intelligent Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#ifndef __LSLIDAR_INPUT_H_
#define __LSLIDAR_INPUT_H_

#include "lslidar_log.h"
#include "rclcpp/rclcpp.hpp"
#include <unistd.h>
#include <stdio.h>
#include <pcap.h>
#include <netinet/in.h>
#include <lslidar_msgs/msg/lslidar_packet.hpp>
#include <lslidar_msgs/msg/lslidar_point.hpp>
#include <lslidar_msgs/msg/lslidar_scan.hpp>
#include <string>
#include <sstream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <poll.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/file.h>
#include <signal.h>
#include <sensor_msgs/msg/time_reference.hpp>

namespace lslidar_driver {
    static uint16_t MSOP_DATA_PORT_NUMBER = 2368;   // lslidar default data port on PC
    //static uint16_t DIFOP_DATA_PORT_NUMBER = 2369;  // lslidar default difop data port on PC
/**
 *  从在线的网络数据或离线的网络抓包数据（pcap文件）中提取出lidar的原始数据，即packet数据包
 * @brief The Input class,
     *
     * @param private_nh  一个NodeHandled,用于通过节点传递参数
     * @param port
     * @returns 0 if successful,
     *          -1 if end of file
     *          >0 if incomplete packet (is this possible?)
 */
    class Input {
    public:
        /**
         * @brief 初始化网络或抓包输入基类并读取 ROS 配置。
         * @param private_nh 用于读取驱动参数的 ROS 节点。
         * @param port 接收雷达数据的 UDP 端口。
         * @param packet_size 单个数据包的预期字节数。
         * @return 无返回值。
         */
        Input(rclcpp::Node* private_nh, uint16_t port, int packet_size);

        /**
         * @brief 销毁输入基类并完成基础资源清理。
         * @return 无返回值。
         */
        virtual ~Input() {
        }

        /**
         * @brief 从具体输入源读取一个雷达数据包；派生类负责实现读取方式。
         * @param pkt 用于接收数据包的输出参数。
         * @return 返回 0 表示读取成功，-1 表示输入结束，正数表示数据包不完整。
         */
        virtual int getPacket(lslidar_msgs::msg::LslidarPacket::UniquePtr &pkt) = 0;


    protected:
        rclcpp::Node*  private_nh_;
        uint16_t port_;
        std::string devip_str_;
        int return_mode_;
        bool npkt_update_flag_;
        bool add_multicast;
        std::string group_ip;
        int packet_size_;
    };

/** @brief Live lslidar input from socket. */
    class InputSocket : public Input {
    public:
        /**
         * @brief 创建 UDP 套接字并准备接收在线雷达数据。
         * @param private_nh 用于读取网络配置的 ROS 节点。
         * @param port 接收数据的 UDP 端口。
         * @param packet_size 数据包的预期字节数。
         * @return 无返回值。
         */
        InputSocket(rclcpp::Node*  private_nh, uint16_t port = MSOP_DATA_PORT_NUMBER,int packet_size = 1212);

        /**
         * @brief 关闭 UDP 套接字并释放网络资源。
         * @return 无返回值。
         */
        virtual ~InputSocket();

        /**
         * @brief 从 UDP 套接字读取一个雷达数据包。
         * @param pkt 用于接收数据包的输出参数。
         * @return 返回 0 表示读取成功，-1 表示输入结束，正数表示数据包不完整。
         */
        virtual int getPacket(lslidar_msgs::msg::LslidarPacket::UniquePtr &pkt);

    private:
        int sockfd_;
        in_addr devip_;

    };

/** @brief lslidar input from PCAP dump file.
   *
   * Dump files can be grabbed by libpcap
   */
    class InputPCAP : public Input {
    public:
        /**
         * @brief 打开 PCAP 文件并设置离线数据包读取节奏。
         * @param private_nh 用于读取离线回放配置的 ROS 节点。
         * @param port 数据流对应的端口号。
         * @param packet_size 数据包的预期字节数。
         * @param packet_rate 离线回放的预期数据包频率，单位为 Hz。
         * @param filename 待读取的 PCAP 文件路径。
         * @return 无返回值。
         */
        InputPCAP(rclcpp::Node* private_nh, uint16_t port = MSOP_DATA_PORT_NUMBER, int packet_size = 1212, double packet_rate = 0.0,
                  std::string filename = "");

//        InputPCAP(rclcpp::Node* private_nh, uint16_t port = MSOP_DATA_PORT_NUMBER, double packet_rate = 0.0,
//                  std::string filename = "", bool read_once = false, bool read_fast = false, double repeat_delay = 0.0);

        /**
         * @brief 关闭 PCAP 文件并释放离线抓包资源。
         * @return 无返回值。
         */
        virtual ~InputPCAP();

        /**
         * @brief 从 PCAP 文件读取下一个雷达数据包。
         * @param pkt 用于接收数据包的输出参数。
         * @return 返回 0 表示读取成功，-1 表示文件结束，正数表示数据包不完整。
         */
        virtual int getPacket(lslidar_msgs::msg::LslidarPacket::UniquePtr &pkt);

    private:
        rclcpp::Rate packet_rate_;
        std::string filename_;
        pcap_t *pcap_;
        bpf_program pcap_packet_filter_;
        char errbuf_[PCAP_ERRBUF_SIZE];
        bool empty_;
        bool read_once_;
        bool read_fast_;
        double repeat_delay_;
    };
}

#endif  // __LSLIDAR_INPUT_H
