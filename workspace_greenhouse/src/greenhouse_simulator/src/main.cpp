// ros stuff
#include "rclcpp/rclcpp.hpp"

// Simulator
#include "greenhouse_simulator/simulator.hpp"

int main(
    int argc,
    char * argv[]
)
{
    rclcpp::init(argc, argv);
    // Spins the node, making the topics, services available.
    rclcpp::spin(std::make_shared<Simulator>());
    rclcpp::shutdown();
    return 0;
}