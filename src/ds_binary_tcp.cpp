#include <memory>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string>
#include <vector>
#include <thread>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "ds_node/msg/binary_data.hpp"

using std::placeholders::_1;

class PolyxBinaryTCP : public rclcpp::Node {
public:
    PolyxBinaryTCP() : Node("polyx_binary_tcp") {
        // Init TCP Server Socket
        init_server_socket();
        
        // Create ROS2 subscriber
        subscription_ = this->create_subscription<ds_node::msg::BinaryData>(
            "binary_topic", 10, 
            std::bind(&PolyxBinaryTCP::topic_callback, this, _1));
    }

    ~PolyxBinaryTCP() {
        running_ = false;
        if (accept_thread_.joinable()) {
            accept_thread_.join();
        }
        close(server_fd);
    }

private:
    void init_server_socket() {
        // get parameters
        this->declare_parameter<std::string>("local_ip", "127.0.0.1");
        this->declare_parameter<std::string>("local_port", "7777");
        std::string local_ip;
        if (this->get_parameter("local_ip", local_ip))
        {
            RCLCPP_INFO(this->get_logger(), "local_ip = %s", local_ip.c_str());
        }
        else
            local_ip = "127.0.0.1";
        std::string local_port;
        if (this->get_parameter("local_port", local_port))
        {
            RCLCPP_INFO(this->get_logger(), "local_port = %s", local_port.c_str());
        }
        else
            local_port = "7777";
        uint32_t port = static_cast<uint32_t>(std::stoul(local_port));

        server_fd = socket(AF_INET, SOCK_STREAM, 0);
        if (server_fd == -1) {
            RCLCPP_ERROR(this->get_logger(), "Socket creation failed");
            return;
        }

        int opt = 1;
        setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

        sockaddr_in serv_addr;
        serv_addr.sin_family = AF_INET;
        serv_addr.sin_addr.s_addr = INADDR_ANY;
        serv_addr.sin_port = htons(port);

        if (bind(server_fd, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Socket bind failed");
            close(server_fd);
            return;
        }

        if (listen(server_fd, 1000) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Socket listen failed");
            close(server_fd);
            return;
        }

        RCLCPP_INFO(this->get_logger(), "TCP server listening on port %d", port);
        accept_thread_ = std::thread(&PolyxBinaryTCP::accept_connections, this);
    }

    void accept_connections() {
        while (running_) {
            sockaddr_in client_addr;
            socklen_t client_len = sizeof(client_addr);
            int client_fd = accept(server_fd, (struct sockaddr*)&client_addr, &client_len);
            
            if (client_fd < 0) {
                RCLCPP_ERROR(this->get_logger(), "Accept failed");
                continue;
            }

            {
                std::lock_guard<std::mutex> lock(clients_mutex_);
                clients_.push_back(client_fd);
            }
            
            RCLCPP_INFO(this->get_logger(), "New client connected: %s:%d", 
                        inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port));
        }
    }

    void topic_callback(const ds_node::msg::BinaryData & msg) {
        std::lock_guard<std::mutex> lock(clients_mutex_);
        const uint8_t* data_ptr = msg.data.data();
        size_t data_size = msg.data.size();
        RCLCPP_INFO(this->get_logger(), "recv %zu bytes from topic", data_size);
        for (auto it = clients_.begin(); it != clients_.end(); ) {
            if (send(*it, data_ptr, data_size, 0) <= 0) {
                close(*it);
                it = clients_.erase(it);
                RCLCPP_ERROR(this->get_logger(), "Failed to send data via socket");
            } else {
                RCLCPP_INFO(this->get_logger(), "Sent %zu bytes to socket", data_size);
            }
            ++it;
        }
    }

    rclcpp::Subscription<ds_node::msg::BinaryData>::SharedPtr subscription_;
    int server_fd = -1;
    std::vector<int> clients_;
    mutable std::mutex clients_mutex_;
    std::thread accept_thread_;
    bool running_ = true;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PolyxBinaryTCP>());
    rclcpp::shutdown();
    return 0;
}
