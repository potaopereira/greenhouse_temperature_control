// WindowController
#include "window_controller/window_controller.hpp"
// for period specification
#include <chrono>
using namespace std::chrono_literals;

WindowController::WindowController(

):
Node("WindowController") // node
,
mTimerMS(7) // 10 / (24*60) * 1000 correponds to one minute in real life [10 seconds in simulation corresponds to 24 hours in real life]
,
mPubTimer(
    create_wall_timer(std::chrono::milliseconds(mTimerMS), std::bind(&WindowController::publish, this))
)
,
mPub(
    create_publisher<command_t>("window_position", 10)
)
,
mTemperatures(
    {
        std::make_pair(0, false)
        ,
        std::make_pair(0, false)
    }
)
,
mSubs(
    {
        create_subscription<temperature_t>(
            "temperature_sensor1",
            10,
            [this](const temperature_t::SharedPtr msg) {
                std::get<0>(mTemperatures[0]) = msg->temperature;
                std::get<1>(mTemperatures[0]) = true;
            }
        )
        ,
        create_subscription<temperature_t>(
            "temperature_sensor2",
            10,
            [this](const temperature_t::SharedPtr msg) {
                std::get<0>(mTemperatures[1]) = msg->temperature;
                std::get<1>(mTemperatures[1]) = true;
            }
        )
    }
)
,
setParamsSrv(
    create_service<greenhouse_msg_srv::srv::ControllerParam>(
        "input_params", // service name 
        std::bind(&WindowController::setParams, this, std::placeholders::_1, std::placeholders::_2) // callback
    )
)
,
mPgain(0)
,
mIGain(0)
,
mIMax(50)
,
mIErrorMax(10)
,
mTarget(0)
,
mPWMPeriod(65536)
{

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "WindowController ready");

    declare_parameter<float>("pgain", 0);
    declare_parameter<float>("igain", 0);
    declare_parameter<float>("imax", 50);
    declare_parameter<float>("ierrormax", 10);
    declare_parameter<float>("target", 25);

}

void
WindowController::publish(
    //
){
    // if we are not reading any temperature readings, then we publish no position command
    if(mSubs.size()==0){
        return;
    }

    // get_parameter<float>("pgain", mPgain);
    // get_parameter<float>("igain", mIGain);
    // get_parameter<float>("imax", mIMax);
    // get_parameter<float>("ierrormax", mIErrorMax);
    // get_parameter<float>("target", mTarget);

    /* calculate average temperature */
    double average = 0;
    int counter = 0;
    for(std::size_t i = 0; i < mTemperatures.size(); ++ i){
        bool* new_measurement = &std::get<1>(mTemperatures[i]);
        if(*new_measurement){
            average+=std::get<0>(mTemperatures[i]);
            ++counter;
            *new_measurement = false;
        }
    }
    // no new measurements, so no need to publish anything new
    if(counter == 0)
        return;
    average /= counter;

    double error = (average - mTarget);
    static double integral = 0;
    if(error > -mIErrorMax && error < +mIErrorMax){
        integral += mIGain*error;
        // prevent unwiding but limiting integral
        if(integral < -mIMax)
            integral = -mIMax;
        if(integral > +mIMax)
            integral = +mIMax;
    }

    double percentage = mPgain*error + integral;
    percentage = percentage > 100 ? 100 : percentage < 0 ? 0 : percentage;

    auto msg = command_t();
    msg.percentage = int(percentage/100*mPWMPeriod)*100./mPWMPeriod;
    msg.temperature = average;
    msg.target_temperature = mTarget;
    mPub->publish(msg);
}

void
WindowController::setParams(
    const std::shared_ptr<greenhouse_msg_srv::srv::ControllerParam::Request> request
    ,
    std::shared_ptr<greenhouse_msg_srv::srv::ControllerParam::Response> response
){

    // get_parameter<float>("pgain", mPgain);
    // get_parameter<float>("igain", mIGain);
    // get_parameter<float>("imax", mIMax);
    // get_parameter<float>("ierrormax", mIErrorMax);
    // get_parameter<float>("target", mTarget);

    /*
    check parameters are within bounds
    */
    if(request->pgain < 0){
        response->accepted = false;
        response->msg = std::string("Proportional gain cannot be negative.");
        return;
    }
    if(request->igain < 0){
        response->accepted = false;
        response->msg = std::string("Integral gain cannot be negative.");
        return;
    }
    if(request->imax < 0){
        response->accepted = false;
        response->msg = std::string("Maximum integral value cannot be negative.");
        return;
    }
    if(request->ierrormax < 0){
        response->accepted = false;
        response->msg = std::string("Maximum error use to trigger integration cannot be negative.");
        return;
    }
    if(request->pwmperiod <= 0){
        response->accepted = false;
        response->msg = std::string("PWM period cannot be non-positive.");
        return;
    }

    mPgain = request->pgain;
    mIGain = request->igain;
    mIMax = request->imax;
    mIErrorMax = request->ierrormax;
    mTarget = request->target;
    mPWMPeriod = request->pwmperiod;
    response->accepted = false;
    response->msg = std::string("Parameters set as requested.");
}