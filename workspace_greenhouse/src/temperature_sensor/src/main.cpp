// ros stuff
#include "rclcpp/rclcpp.hpp"

// TemperatureSensor
#include "temperature_sensor/temperature_sensor.hpp"

int main(
    int argc,
    char * argv[]
)
{
    rclcpp::init(argc, argv);
    // Spins the node, making the topics, services available.
    rclcpp::spin(std::make_shared<TemperatureSensor>());
    rclcpp::shutdown();
    return 0;
}