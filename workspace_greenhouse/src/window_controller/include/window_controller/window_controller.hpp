#pragma once

/**
 * @file window_controller.hpp
 * @author Pedro Pereira (pedro.m.otao.pereira@gmail.com)
 * @brief It provides class node that will publish the window position
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
// std::vector
#include <vector>
// rclcpp::Node
#include "rclcpp/rclcpp.hpp"
// greenhouse_msg_srv::msg::WindowCommand
#include "greenhouse_msg_srv/msg/window_command.hpp"
// greenhouse_msg_srv::msg::Temperature
#include "greenhouse_msg_srv/msg/temperature.hpp"
// ControllerParam
#include "greenhouse_msg_srv/srv/controller_param.hpp"

/**
 * @brief This node-class will publish the requested window position
 * @details The node listens to the temperature sensors and it attempts to regulate the measured temperature to the requested temperature.
 * If no temperature readings are available, this node will not publish any window position command
 * 
 * \image html images/controller_and_sensors_diagram.png
 * 
 */
class WindowController:
public rclcpp::Node
{
public:
    /**
     * @brief Construct a new window controller object
     * 
     */
    WindowController();
private:

    /**
     * @brief Type representing the output, i.e., the window command
     * 
     */
    typedef greenhouse_msg_srv::msg::WindowCommand command_t;
    /**
     * @brief Type representing the inputs, i.e., the temperatures
     * 
     */
    typedef greenhouse_msg_srv::msg::Temperature temperature_t;

    /**
     * @brief Period at which window position command is published
     * 
     */
    int mTimerMS;
    /**
     * @brief Timer whose callback is used to publish window position command
     * 
     */
    rclcpp::TimerBase::SharedPtr mPubTimer;
    /**
     * @brief Window position publisher
     * 
     */
    rclcpp::Publisher<command_t>::SharedPtr mPub;

    /**
     * @brief Pair with temperature reading and boolean indicating whether reading has already been used by the controller
     * 
     */
    std::vector<std::pair<double, bool>> mTemperatures;
    /**
     * @brief Vector of subscribers to temperature sensors
     * 
     */
    std::vector<rclcpp::Subscription<temperature_t>::SharedPtr> mSubs;
    /**
     * @brief Service to set controller parameters
     * 
     */
    rclcpp::Service<greenhouse_msg_srv::srv::ControllerParam>::SharedPtr setParamsSrv;

    /**
     * @brief Controller proportional gain
     * 
     */
    float mPgain;
    /**
     * @brief Controller integral gain
     * 
     */
    float mIGain;
    /**
     * @brief Maximum value that integral component is allowed to have
     * 
     */
    float mIMax;
    /**
     * @brief If abs(tracking error) > mIErrorMax, then integrator is not updated
     * 
     */
    float mIErrorMax;
    /**
     * @brief Target temperature
     * 
     */
    float mTarget;
    /**
     * @brief Target temperature
     * 
     */
    int mPWMPeriod;

    /**
     * @brief Callback to be used by timer associated to window position publisher
     * 
     */
    void
    publish();

    /**
     * @brief Service provider
     * 
     * @param request Request message
     * @param response Reply message
     */
    void setParams(
        const std::shared_ptr<greenhouse_msg_srv::srv::ControllerParam::Request> request
        ,
        std::shared_ptr<greenhouse_msg_srv::srv::ControllerParam::Response> response
    );
};
