// ros stuff
#include "rclcpp/rclcpp.hpp"

// QApplication
#include <QApplication>

// GuiMonitor
#include "gui_monitor/gui_monitor.hpp"

int main(
    int argc,
    char ** argv
)
{

    QApplication a(argc, argv);
    GuiMonitor gui(argc, argv);
    gui.show();

    return a.exec();
}

