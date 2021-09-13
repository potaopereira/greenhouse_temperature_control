#pragma once
/**
 * @file temperature_sensor.hpp
 * @author Pedro Pereira (pedro.m.otao.pereira@gmail.com)
 * @brief Reads a temperature sensor and it publishes a temperature topic
 * 
 * @details \image html images/controller_and_sensors_diagram.png
 * 
 * @version 0.1
 * @date 2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */

// std::shared_ptr
#include <memory>
// ros client
#include "rclcpp/rclcpp.hpp"
// temperature message
#include "greenhouse_msg_srv/msg/temperature.hpp"

/**
 * @brief Node that reads temperature sensor and publishes temperature topic
 * 
 * @details \image html images/controller_and_sensors_diagram.png
 * 
 */
class TemperatureSensor:
public rclcpp::Node
{
public:
    /**
     * @brief Construct a new temperature sensor node
     * 
     */
    TemperatureSensor();
private:
    /**
     * @brief Type representing ouput, i.e., the temperature readout
     * 
     */
    typedef greenhouse_msg_srv::msg::Temperature temperature_t;
    /**
     * @brief Float to store readout of temperature
     * 
     */
    float mTemperature;
    // rclcpp::TimerBase::SharedPtr temperaturePublisherTimer;
    /**
     * @brief Temperature publisher
     * 
     */
    rclcpp::Publisher<temperature_t>::SharedPtr temperaturePublisher;
    /**
     * @brief Subscribe to temperature coming from a simulator
     * 
     */
    rclcpp::Subscription<temperature_t>::SharedPtr temperatureSub;
    // void publishTemperature();
    /**
     * @brief Read temperature message coming from simulators
     * 
     * @param temperature Temperature message
     */
    void
    readTemperature(
        const temperature_t::SharedPtr temperature
    );
};
