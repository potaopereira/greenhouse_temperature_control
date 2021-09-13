//TemperatureSensor
#include "temperature_sensor/temperature_sensor.hpp"

#include <chrono>
using namespace std::chrono_literals;

TemperatureSensor::TemperatureSensor(

):
Node("TemperatureSensor") // node
,
mTemperature(0)
// ,
// temperaturePublisherTimer(
//     create_wall_timer(1000ms, std::bind(&TemperatureSensor::publishTemperature, this))
// )
,
temperaturePublisher(
    create_publisher<temperature_t>("temperature", 10)
)
,
temperatureSub(
    create_subscription<temperature_t>(
        "read_temperature",
        10,
        std::bind(&TemperatureSensor::readTemperature, this, std::placeholders::_1)
    )
)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "TemperatureSensor ready");
}

// void
// TemperatureSensor::publishTemperature(){
//     auto message = temperature_t();
//     message.temperature = mTemperature;
//     temperaturePublisher->publish(message);
// }

void
TemperatureSensor::readTemperature(
    const temperature_t::SharedPtr temperature
){
    auto message = temperature_t();
    message.temperature = temperature->temperature;
    temperaturePublisher->publish(message);
}