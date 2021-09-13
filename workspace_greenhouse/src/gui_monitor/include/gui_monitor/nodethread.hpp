/**
 * @file nodethread.hpp
 * @author Pedro Pereira (pedro.m.otao.pereira@gmail.com)
 * @brief QThread that handles ros spin
 * @version 0.1
 * @date 2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#pragma once

// rclcpp::Node
#include "rclcpp/rclcpp.hpp"

// QThread
#include <QThread>

// greenhouse_msg_srv::msg::WindowCommand
#include "greenhouse_msg_srv/msg/window_command.hpp"

// greenhouse_msg_srv::msg::Temperature
#include "greenhouse_msg_srv/msg/temperature.hpp"

// greenhouse_msg_srv::srv::ControllerParam
#include "greenhouse_msg_srv/srv/controller_param.hpp"

/**
 * @brief Node that subscribes to topics whose data we wish to plot and service to set useful controller parameters
 * 
 */
class GuiNode:
public rclcpp::Node
{
public:
    /**
     * @brief Constructor
     * 
     * @param f Function to be called when a new message from \p mWindowCommandSub is received
     */
    GuiNode(
        std::function<void(double, double, double)> f
    );
    /**
     * @brief Method that should be called if one wishes to set the controller parameters
     * 
     * @param pgain Proportional gain
     * @param igain Integral gain
     * @param imax Maximum integral value
     * @param ierrormax Maximum error for integrator to work
     * @param target Target temperature
     * @param pwmperiod PWM period
     */
    void setParams(
        float pgain,
        float igain,
        float imax,
        float ierrormax,
        float target,
        int pwmperiod
    );
private:
    /**
     * @brief Type represeting the input to this node
     * 
     */
    typedef greenhouse_msg_srv::msg::WindowCommand command_t;
    /**
     * @brief Subscriber to windown position message
     * 
     */
    rclcpp::Subscription<command_t>::SharedPtr mWindowCommandSub;
    /**
     * @brief Client to service used to set important controller parameters
     * 
     */
    rclcpp::Client<greenhouse_msg_srv::srv::ControllerParam>::SharedPtr mControllerParamsClient;
};

/**
 * @brief QThread used to call ros init (in constructor) and ros spin (in run)
 * 
 */
class NodeThread:
public QThread
{
    Q_OBJECT
protected:
    /**
     * @brief Method called after thread is started
     * 
     */
    void run() override;
public:
    /**
     * @brief Constructor
     * 
     * @param argc Argument counter
     * @param argv Argument vector
     * @param parent Parent qobject
     */
    NodeThread(
        int argc,
        char** argv,
        QObject *parent = nullptr
    );
    /**
     * @brief This is a called of the signal setMsg
     * 
     * @param temperature Temperature measurement
     * @param target_temperature Target temperature measurement
     * @param percentage Windown opennes measurement
     */
    void emitSetMsg(
        double temperature,
        double target_temperature,
        double percentage
    );
public slots:
    /**
     * @brief Method that should be called if one wishes to set the controller parameters
     * 
     * @param pgain Proportional gain
     * @param igain Integral gain
     * @param imax Maximum integral value
     * @param ierrormax Maximum error for integrator to work
     * @param target Target temperature
     * @param pwmperiod PWM period
     */
    void setControllerParams(
        float pgain,
        float igain,
        float imax,
        float ierrormax,
        float target,
        int pwmperiod
    );
signals:
    /**
     * @brief Signal to be used by other qobjects: node reads message and will trigger this signal
     * 
     * @param temperature Temperature measurement
     * @param target_temperature Target temperature measurement
     * @param percentage Windown opennes measurement
     */
    void setMsg(
        double temperature,
        double target_temperature,
        double percentage
    );
private:
    /**
     * @brief Shared pointer to node
     * 
     */
    std::shared_ptr<GuiNode> mGuiNode;
};