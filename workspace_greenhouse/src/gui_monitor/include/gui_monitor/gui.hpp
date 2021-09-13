/**
 * @file gui.hpp
 * @author Pedro Pereira (pedro.m.otao.pereira@gmail.com)
 * @brief Widget to combine plot widget with widget that sets the parameters
 * @version 0.1
 * @date 2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#pragma once

// Plot
#include "gui_monitor/plot.hpp"

// Parameters
#include "gui_monitor/parameters.hpp"

// QWidget
#include <QWidget>

// QHBoxLayout
#include <QHBoxLayout>

/**
 * @brief Widget that combines the plot widget with the widget to set the parameters
 * 
 */
class Gui:
public QWidget
{
    Q_OBJECT
public:
    /**
     * @brief Constructor
     * 
     * @param parent Parent widget
     */
    Gui(
        QWidget *parent = 0
    );

public slots:
    /**
     * @brief Redirect plot slot from this class to member mPlot plot slot
     * 
     * @param temperature Temperature measurement
     * @param target_temperature Target temperature measurement
     * @param percentage Window position openess in percentage
     */
    void plot(
        double temperature,
        double target_temperature,
        double percentage
    );

    /**
     * @brief Method called to change controller parameters
     * 
     * @param pgain Proportional gain
     * @param igain Integral gain
     * @param imax Maximum integral value
     * @param ierrormax Maximum error for integrator to work
     * @param target Target temperature
     * @param pwmperiod PWM period
     */
    void changeParams(
        float pgain,
        float igain,
        float imax,
        float ierrormax,
        float target,
        int pwmperiod
    );

signals:
    /**
     * @brief Signal emitted to change the controller parameters
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
     * @brief Plot widget that plots important data
     * 
     */
    Plot mPlot;
    /**
     * @brief Parameters widget that allows user to change important controller parameters
     * 
     */
    Parameters mParam;
    /**
     * @brief Layout
     * 
     */
    QHBoxLayout mLayout;
};
