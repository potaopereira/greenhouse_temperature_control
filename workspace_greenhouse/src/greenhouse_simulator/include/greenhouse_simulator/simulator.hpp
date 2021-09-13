#pragma once
/**
 * @file simulator.hpp
 * @author Pedro Pereira (pedro.m.otao.pereira@gmail.com)
 * @brief Simulates the temperature inside a greenhouse given a requested window position
 * @details \image html images/simulator.png
 * @version 0.1
 * @date 2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */

// std::shared_ptr
#include <memory>
// std::vector
#include <vector>
// std::pair
#include <utility>
// rclcpp::Node
#include "rclcpp/rclcpp.hpp"
// greenhouse_msg_srv::msg::Temperature
#include "greenhouse_msg_srv/msg/temperature.hpp"
// greenhouse_msg_srv::msg::WindowCommand
#include "greenhouse_msg_srv/msg/window_command.hpp"
// SimulatorImpl
#include "greenhouse_simulator/simulator_impl.hpp"

/**
 * @brief Class node that publishes the temperatures that sensors would read, and it reads a requested window position
 * 
 * @details \image html images/simulator.png
 * 
 */
class Simulator:
public rclcpp::Node
{
public:
    /**
     * @brief Construct a new simulator object
     * 
     */
    Simulator();
private:
    /**
     * @brief Type that represents the outputs, i.e., temperatures
     * 
     */
    typedef greenhouse_msg_srv::msg::Temperature temperature_t;
    /**
     * @brief Type that represents the input, i.e., the window position
     * 
     */
    typedef greenhouse_msg_srv::msg::WindowCommand window_command_t;

    /**
     * @brief Vector of temperatures for each temperature sensor
     * 
     */
    std::vector<float> mTemperature;
    /**
     * @brief Timer period to be used by timer that triggers the publishing of the temperature messages
     * 
     */
    int mTimerMS;
    /**
     * @brief Timer whose callback triggers the publishing of the temperature messages
     * 
     */
    rclcpp::TimerBase::SharedPtr temperaturePublishersTimer;
    /**
     * @brief Type consisting of a temperature publishes and a delay (temperature published is delayed)
     * 
     */
    typedef std::pair<rclcpp::Publisher<temperature_t>::SharedPtr, int> temperature_sensor_t;
    /**
     * @brief Vectors of temperature publishers
     * 
     */
    std::vector<temperature_sensor_t> temperaturePublishers;
    /**
     * @brief Simulator that solves ODE
     * 
     */
    SimulatorImpl mSimulatorImpl;
    /**
     * @brief Subscriber to window position
     * 
     */
    rclcpp::Subscription<window_command_t>::SharedPtr mWindowSub;
    /**
     * @brief Requested window position
     * 
     */
    double mWindowCommand;
    /**
     * @brief Callback of timer where temperatures are published
     * 
     */
    void publishTemperatures(
    );
    /**
     * @brief Callback for new message from \p mWindowSub
     * 
     * @param window_command Window position message
     */
    void
    readWindowPosition(
        const window_command_t::SharedPtr window_command
    );

};
