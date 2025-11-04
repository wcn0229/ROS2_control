#include <memory>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string>
#include <vector>
#include <thread>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "ds_node/msg/binary_data.hpp"

using std::placeholders::_1;

class DsRtcmForwarder : public rclcpp::Node
{
private:
    bool is_tcp_;
    bool is_socket_valid_;
    //
    rclcpp::Subscription<ds_node::msg::BinaryData>::SharedPtr subscriber_;
    //
    int client_fd_ = -1;
    sockaddr_in serv_addr_;

    void callback(const ds_node::msg::BinaryData &msg)
    {
        const uint8_t *data_ptr = msg.data.data();
        size_t data_size = msg.data.size();
        RCLCPP_INFO(this->get_logger(), "Receiving %zu bytes from topic.", data_size);

        // send
        int r = 0;
        if (is_tcp_)
        { 
            r = send(client_fd_, data_ptr, data_size, 0);
        }
        else
        {
            r = sendto(client_fd_, data_ptr, data_size, 0,
                (struct sockaddr *)&serv_addr_, sizeof(struct sockaddr));
        }
        if (r <= 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to send data via socket.");
            is_socket_valid_ = false;
            while (!is_socket_valid_)
            {
                RCLCPP_INFO(this->get_logger(), "Re-initializing socket...");
                initClientSocket();
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Sent %zu bytes to socket.", data_size);
        }
    }

public:
    DsRtcmForwarder() : Node("ds_rtcm_forwarder"), is_tcp_(true), is_socket_valid_(false)
    {
        // declare parameters
        this->declare_parameter<bool>("use_tcp", true);
        // this->declare_parameter<std::string>("dsnav_ip", "192.168.230.97");
        // this->declare_parameter<std::string>("dsnav_port", "9999");
        this->declare_parameter<std::string>("rtcm_data_topic", "rtcm");

        // 
        auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this, "ds_node_talker");
        while (!parameters_client->wait_for_service()) {}
        auto dsnav_ip = parameters_client->get_parameter<std::string>("eth_server", "");
        auto dsnav_port = parameters_client->get_parameter<std::string>("eth_port", "");

        // get parameters
        if (this->get_parameter("use_tcp", is_tcp_)) {}
        else { is_tcp_ = true; }
        RCLCPP_INFO(this->get_logger(), "Using %s.", is_tcp_ ? "TCP" : "UDP");
        /*
        std::string dsnav_ip;
        if (this->get_parameter("dsnav_ip", dsnav_ip)) {}
        else { dsnav_ip = "192.168.230.97"; }
        //
        std::string dsnav_port;
        if (this->get_parameter("dsnav_port", dsnav_port)) {}
        else { dsnav_port = "9999"; }
        */
        RCLCPP_INFO(this->get_logger(), "dsnav_ip = %s", dsnav_ip.c_str());
        RCLCPP_INFO(this->get_logger(), "dsnav_port = %s", dsnav_port.c_str());
        //
        std::string rtcm_topic;
        if (this->get_parameter("rtcm_data_topic", rtcm_topic)) {}
        else { rtcm_topic = "rtcm"; }
        RCLCPP_INFO(this->get_logger(), "rtcm_data_topic = %s", rtcm_topic.c_str());

        // set addresss
        serv_addr_.sin_family = AF_INET;
        serv_addr_.sin_addr.s_addr = inet_addr(dsnav_ip.c_str());
        uint32_t port = static_cast<uint32_t>(std::stoul(dsnav_port));
        serv_addr_.sin_port = htons(port);

        // initialize client socket
        initClientSocket();

        // Create ROS2 subscriber
        subscriber_ = this->create_subscription<ds_node::msg::BinaryData>(
            rtcm_topic, 10, std::bind(&DsRtcmForwarder::callback, this, _1));
    }

    ~DsRtcmForwarder()
    {
        if (is_socket_valid_)
        {
            close(client_fd_);
        }
    }

private:
    void initClientSocket()
    {
        // new socket
        client_fd_ = socket(AF_INET, is_tcp_ ? SOCK_STREAM : SOCK_DGRAM, 0);
        if (client_fd_ == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "Cannot open socket.");
            return;
        }        

        // connect if TCP
        if (is_tcp_ && ::connect(
            client_fd_, reinterpret_cast<sockaddr *>(&serv_addr_), sizeof(serv_addr_)) == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "Cannot connect to server.");
            close(client_fd_);
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Connected to server.");
        is_socket_valid_ = true;
    } 
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DsRtcmForwarder>());
    rclcpp::shutdown();
    return 0;
}
