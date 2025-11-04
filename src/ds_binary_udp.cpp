#include <memory>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <chrono>
#include <atomic>
#include <unordered_map>
#include <cstring>
#include <deque>

#include "rclcpp/rclcpp.hpp"
#include "ds_node/msg/binary_data.hpp"

using std::placeholders::_1;

class PolyxBinaryUDP : public rclcpp::Node {
public:
    PolyxBinaryUDP() : Node("polyx_binary_udp") {
        // Init UDP Server Socket
        init_server_socket();
        
        // Create ROS2 subscriber
        subscription_ = this->create_subscription<ds_node::msg::BinaryData>(
            "binary_topic", 10, 
            std::bind(&PolyxBinaryUDP::topic_callback, this, _1));
        
        // Start timeout checker thread
        timeout_thread_ = std::thread(&PolyxBinaryUDP::check_timeouts, this);
    }

    ~PolyxBinaryUDP() {
        running_ = false;
        if (recv_thread_.joinable()) {
            recv_thread_.join();
        }
        if (timeout_thread_.joinable()) {
            timeout_thread_.join();
        }
        if (flush_thread_.joinable()) {
            flush_thread_.join();
        }
        close(server_fd);
    }

private:
    struct ClientInfo {
        sockaddr_in address;
        std::chrono::steady_clock::time_point last_received;
        std::vector<uint8_t> buffer;  // Buffer for each client
    };

    void init_server_socket() {
        // Get parameters
        this->declare_parameter<std::string>("local_ip", "127.0.0.1");
        this->declare_parameter<std::string>("local_port", "6666");
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
            local_port = "6666";
        uint32_t port = static_cast<uint32_t>(std::stoul(local_port));

        // Create UDP socket
        server_fd = socket(AF_INET, SOCK_DGRAM, 0);
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

        RCLCPP_INFO(this->get_logger(), "UDP server listening on port %d", port);
        recv_thread_ = std::thread(&PolyxBinaryUDP::receive_messages, this);
    }

    void receive_messages() {
        char buffer[1024];
        sockaddr_in client_addr;
        socklen_t client_len = sizeof(client_addr);

        while (running_) {
            ssize_t n = recvfrom(server_fd, buffer, sizeof(buffer), 0,
                            (struct sockaddr*)&client_addr, &client_len);
        
            if (n < 0) {
                if (running_) {
                    RCLCPP_ERROR(this->get_logger(), "recvfrom error");
                }
                continue;
            }

            // Get client key
            uint64_t key = get_client_key(client_addr);
            auto now = std::chrono::steady_clock::now();

            {
                std::lock_guard<std::mutex> lock(clients_mutex_);
                
                // Find or create client
                auto it = clients_.find(key);
                if (it == clients_.end()) {
                    // New client
                    ClientInfo new_client;
                    new_client.address = client_addr;
                    new_client.last_received = now; // Initialize last refresh time
                    clients_[key] = new_client;
                    
                    RCLCPP_INFO(this->get_logger(), "New client: %s:%d", 
                            inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port));
                } else {
                    // Update existing client's last received time
                    it->second.last_received = now;
                }
            }
        }
    }

    void topic_callback(const ds_node::msg::BinaryData & msg) {
        const std::vector<uint8_t>& new_data = msg.data;
        RCLCPP_INFO(this->get_logger(), "Received %zu bytes from topic", new_data.size());

        std::lock_guard<std::mutex> lock(clients_mutex_);
        
        for (auto& [key, client] : clients_) {
            // Append new data to buffer
            client.buffer.insert(client.buffer.end(), new_data.begin(), new_data.end());
            
            // Check if buffer reaches 512 bytes
            if (client.buffer.size() >= 512) {
                send_buffer(client);
            }
        }
    }

    void send_buffer(ClientInfo& client) {
        if (client.buffer.empty()) return;

        ssize_t sent = sendto(server_fd, client.buffer.data(), client.buffer.size(), 0,
                             (const struct sockaddr*)&client.address, sizeof(client.address));
        
        if (sent < 0) {
            RCLCPP_ERROR(this->get_logger(), "UDP sendto failed for %s:%d", 
                       inet_ntoa(client.address.sin_addr), ntohs(client.address.sin_port));
        } else if (static_cast<size_t>(sent) != client.buffer.size()) {
            RCLCPP_WARN(this->get_logger(), "Partial UDP send: %zd/%zu bytes to %s:%d",
                      sent, client.buffer.size(), inet_ntoa(client.address.sin_addr), ntohs(client.address.sin_port));
        } else {
            RCLCPP_INFO(this->get_logger(), "Sent %zu bytes to %s:%d", 
                      client.buffer.size(), inet_ntoa(client.address.sin_addr), ntohs(client.address.sin_port));
        }
        
        // Clear buffer
        client.buffer.clear();
    }

    void check_timeouts() {
        constexpr std::chrono::seconds TIMEOUT(10);
        
        while (running_) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            auto now = std::chrono::steady_clock::now();
            
            std::lock_guard<std::mutex> lock(clients_mutex_);
            auto it = clients_.begin();
            while (it != clients_.end()) {
                if (now - it->second.last_received > TIMEOUT) {
                    RCLCPP_INFO(this->get_logger(), "Client timeout: %s:%d", 
                               inet_ntoa(it->second.address.sin_addr), 
                               ntohs(it->second.address.sin_port));
                    it = clients_.erase(it);
                } else {
                    ++it;
                }
            }
        }
    }

    // Generate unique key for client
    uint64_t get_client_key(const sockaddr_in& addr) const {
        return (static_cast<uint64_t>(addr.sin_addr.s_addr) << 32 | addr.sin_port);
    }

    rclcpp::Subscription<ds_node::msg::BinaryData>::SharedPtr subscription_;
    int server_fd = -1;
    mutable std::mutex clients_mutex_;
    
    // Client storage with buffers
    std::unordered_map<uint64_t, ClientInfo> clients_;
    
    std::thread recv_thread_;
    std::thread timeout_thread_;
    std::thread flush_thread_;  // New buffer checking thread
    std::atomic<bool> running_{true};
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PolyxBinaryUDP>());
    rclcpp::shutdown();
    return 0;
}