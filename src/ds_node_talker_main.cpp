#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "ds_node_talker.hpp"

using namespace std::chrono_literals;

void intHandler(int)
{
    printf("ds_node_talker: Ctrl-C hit, will stop!\r\n");
}

void abortHandler(int)
{
    printf("ds_node_talker: Asked to Abort, will stop!\r\n");
}

void segHandler(int)
{
    printf("ds_node_talker: Segment Fault\r\n");
}

void ioHandler(int)
{
    printf("ds_node_talker: Received SIGIO\r\n");
}

int main(int argc, char *argv[])
{

    struct sigaction int_act;
    int_act.sa_handler = intHandler;
    sigaction(SIGINT, &int_act, NULL);

    struct sigaction abort_act;
    abort_act.sa_handler = abortHandler;
    sigaction(SIGABRT, &abort_act, NULL);

    struct sigaction seg_act;
    seg_act.sa_handler = segHandler;
    sigaction(SIGILL, &seg_act, NULL);

    struct sigaction io_act;
    io_act.sa_handler = ioHandler;
    sigaction(SIGIO, &io_act, NULL);

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DsnodeTalker>());
    rclcpp::shutdown();
    return 0;
}