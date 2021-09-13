/**
 * @file plot.hpp
 * @author Pedro Pereira (pedro.m.otao.pereira@gmail.com)
 * @brief Plot-widget with important signals
 * @version 0.1
 * @date 2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#pragma once

// QCustomPlot
#include "gui_monitor/qcustomplot.h"
// QVector
#include <QVector>

/**
 * @brief Plot widget
 * 
 */
class Plot:
public QCustomPlot
{
    Q_OBJECT
public:
    /**
     * @brief Construct a new plot widget
     * 
     * @param parent 
     */
    Plot(
        QWidget *parent = 0
    );
public slots:
    /**
     * @brief Plot important signals
     * 
     * @param temperature New temperature measurement
     * @param target_temperature New target temperature measurement
     * @param percentage New target windown position command
     */
    void plot(
        double temperature,
        double target_temperature,
        double percentage
    );
private:
    /**
     * @brief Current measurement index
     * 
     */
    int mIndex;
    /**
     * @brief Index vector
     * 
     */
    QVector<double> mTime;
    /**
     * @brief Temperature measurements vector
     * 
     */
    QVector<double> mTemperature;
    /**
     * @brief Target temperature measurements vector
     * 
     */
    QVector<double> mTemperatureTarget;
    /**
     * @brief Window requested positions vector
     * 
     */
    QVector<double> mWindowCommand;
    /**
     * @brief Current index
     * 
     */
    QVector<double> mTimeNow;
    /**
     * @brief Current temperature measurement
     * 
     */
    QVector<double> mTemperatureNow;
    /**
     * @brief Current target temperature measurement
     * 
     */
    QVector<double> mTemperatureTargetNow;
    /**
     * @brief Current window position command
     * 
     */
    QVector<double> mWindowCommandNow;
    /**
     * @brief Layout
     * 
     */
    QGridLayout mLayout;

    /**
     * @brief Update the internal data
     * 
     * @param temperature New temperature measurement
     * @param target_temperature New target temperature measurement
     * @param percentage New target windown position command
     */
    void
    setData(
        double temperature,
        double target_temperature,
        double percentage
    );

    /**
     * @brief Plot the current data
     * 
     */
    void
    plotData(
        //
    );

};