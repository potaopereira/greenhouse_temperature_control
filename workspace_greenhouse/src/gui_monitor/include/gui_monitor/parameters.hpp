/**
 * @file parameters.hpp
 * @author Pedro Pereira (pedro.m.otao.pereira@gmail.com)
 * @brief Widget to set important parameters
 * @version 0.1
 * @date 2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#pragma once

#include <QScrollArea>
#include <QLabel>
#include <QLineEdit>
#include <QGridLayout>
#include <QGroupBox>

/**
 * @brief Widget to set important parameters
 * 
 */
class Parameters:
public QScrollArea
{
    Q_OBJECT
public:
    /**
     * @brief Construct a new Parameters widget
     * 
     * @param parent pointer to parent widget
     */
    Parameters(
        QWidget *parent = 0
    );
public slots:
    /**
     * @brief slot triggered when any of the QLineEdit's are editted
     * 
     */
    void changeParams();
signals:
    /**
     * @brief Signal emitted when any of the QLineEdit's are editted
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
     * @brief Label for target temperature
     * 
     */
    QLabel* mTargetLabel;
    /**
     * @brief Editor for target temperature
     * 
     */
    QLineEdit* mTargetEditor;
    /**
     * @brief Label for proportional gain
     * 
     */
    QLabel* mPGainLabel;
    /**
     * @brief Editor for proportional gain
     * 
     */
    QLineEdit* mPGainEditor;
    /**
     * @brief Label for integral gain
     * 
     */
    QLabel* mIGainLabel;
    /**
     * @brief Editor for integral gain
     * 
     */
    QLineEdit* mIGainEditor;
    /**
     * @brief Label for maximum integral value
     * 
     */
    QLabel* mIMaxLabel;
    /**
     * @brief Editor for maximum integral value
     * 
     */
    QLineEdit* mIMaxEditor;
    /**
     * @brief Label for maximum error for integrator to work
     * 
     */
    QLabel* mIErrorMaxLabel;
    /**
     * @brief Editor for maximum error for integrator to work
     * 
     */
    QLineEdit* mIErrorMaxEditor;
    /**
     * @brief Label for PWM period
     * 
     */
    QLabel* mPWMPeriodLabel;
    /**
     * @brief Editor for PWM period
     * 
     */
    QLineEdit* mPWMPeriodEditor;
    /**
     * @brief Group box used to bundle all labels and editors
     * 
     */
    QGroupBox mGroup;
    /**
     * @brief Layout of widget
     * 
     */
    QGridLayout mLayout;
};
