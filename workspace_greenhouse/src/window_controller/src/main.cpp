// ros stuff
#include "rclcpp/rclcpp.hpp"

// WindowController
#include "window_controller/window_controller.hpp"

int main(
    int argc,
    char * argv[]
)
{
    rclcpp::init(argc, argv);
    // Spins the node, making the topics, services available.
    rclcpp::spin(std::make_shared<WindowController>());
    rclcpp::shutdown();
    return 0;
}