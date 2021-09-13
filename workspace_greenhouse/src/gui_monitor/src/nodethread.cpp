// GuiNode, NodeThread
#include "gui_monitor/nodethread.hpp"

// 1s
#include <chrono>
using namespace std::chrono_literals;


GuiNode::GuiNode(
    std::function<void(double, double, double)> f
):
Node("GuiMonitor") // node
,
mWindowCommandSub(
    create_subscription<command_t>(
        "window_position",
        10,
        [this, f](const command_t::SharedPtr msg) {
            f(
                msg->temperature,
                msg->target_temperature,
                msg->percentage
            );
        }
    )
)
,
mControllerParamsClient(
    create_client<greenhouse_msg_srv::srv::ControllerParam>("input_params")
)
{

    while(!mControllerParamsClient->wait_for_service(1s)) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting for input_params service broadcast");
        if (!rclcpp::ok()) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ROS down");
        }
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "input_params service found");
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "GuiMonitor started");

}

void
GuiNode::setParams(
    float pgain,
    float igain,
    float imax,
    float ierrormax,
    float target,
    int pwmperiod
){
    auto request = std::make_shared<greenhouse_msg_srv::srv::ControllerParam::Request>();

    request->pgain = pgain;
    request->igain = igain;
    request->imax = imax;
    request->ierrormax = ierrormax;
    request->target = target;
    request->pwmperiod = pwmperiod;

    using ServiceResponseFuture =
    rclcpp::Client<greenhouse_msg_srv::srv::ControllerParam>::SharedFuture;
    auto response_received_callback = [this](ServiceResponseFuture future) {
        auto result = future.get();
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%s", result.get()->msg.c_str());
    };
    auto future_result = mControllerParamsClient->async_send_request(
        request,
        response_received_callback
    );
    return;
}


void
NodeThread::run() {
    rclcpp::spin(mGuiNode);
    rclcpp::shutdown();
}

NodeThread::NodeThread(
    int argc,
    char** argv,
    QObject* parent
):
QThread(parent)
{
    rclcpp::init(argc, argv);
    mGuiNode = std::make_shared<GuiNode>(
        std::bind(
            &NodeThread::emitSetMsg,
            this,
            std::placeholders::_1,
            std::placeholders::_2,
            std::placeholders::_3
        )
    );
}

void NodeThread::emitSetMsg(
    double temperature,
    double target_temperature,
    double percentage
){
    emit setMsg(
        temperature,
        target_temperature,
        percentage
    );
}

void
NodeThread::setControllerParams(
    float pgain,
    float igain,
    float imax,
    float ierrormax,
    float target,
    int pwmperiod
){
    mGuiNode->setParams(
        pgain,
        igain,
        imax,
        ierrormax,
        target,
        pwmperiod
    );
}