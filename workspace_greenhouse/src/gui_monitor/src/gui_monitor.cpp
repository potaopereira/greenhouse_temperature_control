// GuiMonitor
#include "gui_monitor/gui_monitor.hpp"

#include <QMenu>
#include <QAction>

GuiMonitor::GuiMonitor(
    int argc,
    char** argv,
    QWidget *parent
):
QMainWindow(parent)
,
mNode(argc, argv, parent)
,
mGui()
{
    connect(
        &mNode, &NodeThread::setMsg,
        &mGui, &Gui::plot
    );
    connect(
        &mGui, &Gui::setParams,
        &mNode, &NodeThread::setControllerParams
    );

    QMenu* fileMenu = menuBar()->addMenu(tr("&Help"));
    QAction* helpAction = new QAction("Help");
    fileMenu->addAction(helpAction);
    // connect(
    //     helpAction, &QActionGroup::triggered,
    //     this, &GuiMonitor::openHelpDialog
    // );

    mNode.start();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "GuiMonitor ready");
    setCentralWidget(&mGui);
}
