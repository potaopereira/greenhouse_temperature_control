// Simulator
#include "greenhouse_simulator/simulator.hpp"

#include <chrono>
using namespace std::chrono_literals;

Simulator::Simulator(

):
Node("Simulator") // Creates a node named add_two_ints_server
,
mTemperature({})
,
mTimerMS(7) // 10 / (24*60) * 1000 correponds to one minute in real life
,
temperaturePublishersTimer(
    // std::chrono::milliseconds duration</*signed integer type of at least 35 bits*/, std::milli>
    create_wall_timer(std::chrono::milliseconds(mTimerMS), std::bind(&Simulator::publishTemperatures, this))
)
,
temperaturePublishers(
    {
        std::make_pair(create_publisher<temperature_t>("temperature_sensor1_sim", 10), 1) // delay of 1 sample // @todo
        ,
        std::make_pair(create_publisher<temperature_t>("temperature_sensor2_sim", 10), 2) // delay of 2 samples // @todo
    }
)
,
mSimulatorImpl(mTimerMS)
,
mWindowSub(
    create_subscription<window_command_t>(
        "window_position",
        10,
        std::bind(&Simulator::readWindowPosition, this, std::placeholders::_1)
    )
)
,
mWindowCommand(0)
{

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Simulator ready");

    // @todo change this
    // chosen maximum delay is of 2 samples
    mTemperature.push_back(mSimulatorImpl.evolve(mWindowCommand));
    mTemperature.push_back(mSimulatorImpl.evolve(mWindowCommand));
    mTemperature.push_back(mSimulatorImpl.evolve(mWindowCommand));

    declare_parameter<float>("time_constant", 1);
}

void
Simulator::publishTemperatures(
    //
){
    for(std::size_t i = 0; i < mTemperature.size() - 1; ++i){
        mTemperature[i+1] = mTemperature[i];
    }
    mTemperature[0] = mSimulatorImpl.evolve(mWindowCommand);
    
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Temperature %.2f", mTemperature[0]);
    for(std::size_t i = 0; i < temperaturePublishers.size(); ++i){
        auto pub = std::get<0>(temperaturePublishers[i]);
        int delay = std::get<1>(temperaturePublishers[i]);

        auto msg = temperature_t();
        msg.temperature = mTemperature[delay];
        pub->publish(msg);
    }
}

void
Simulator::readWindowPosition(
    const window_command_t::SharedPtr window_command
){
    mWindowCommand = window_command->percentage/100;
}