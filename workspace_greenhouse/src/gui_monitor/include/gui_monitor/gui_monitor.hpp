/**
 * @file gui_monitor.hpp
 * @author Pedro Pereira (pedro.m.otao.pereira@gmail.com)
 * @brief Gui used to plot the relevant data and to allow user to change important controller parameters
 * @details
 * \image html images/target_position_changed.gif
 * \image html images/change_pwm_period.gif
 * 
 * @version 0.1
 * @date 2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#pragma once

// QMainWindow
#include <QMainWindow>

// Gui
#include "gui_monitor/gui.hpp"

// NodeThread
#include "gui_monitor/nodethread.hpp"

/**
 * @brief Gui used to plot the relevant data and to allow user to change important controller parameters
 * \image html images/target_position_changed.gif
 * \image html images/change_pwm_period.gif
 */
class GuiMonitor:
public QMainWindow
{
    Q_OBJECT
public:
    /**
     * @brief Constructor
     * 
     * @param argc Argument counter
     * @param argv Argument vector
     * @param parent Pointer to parent widget
     */
    GuiMonitor(
        int argc,
        char** argv,
        QWidget *parent = 0
    );
private:
    /**
     * @brief Object that handles ros init and ros spin
     * 
     */
    NodeThread mNode;
    /**
     * @brief Gui to show data and allow user to change parameters
     * 
     */
    Gui mGui;
};

