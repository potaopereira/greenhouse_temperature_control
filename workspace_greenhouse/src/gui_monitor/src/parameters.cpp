// Parameters
#include "gui_monitor/parameters.hpp"

#include <iostream>

Parameters::Parameters(
    QWidget *parent
):
QScrollArea(parent)
,
mTargetLabel(new QLabel("Target temperature"))
,
mTargetEditor(new QLineEdit(QString("25")))
,
mPGainLabel(new QLabel("P gain"))
,
mPGainEditor(new QLineEdit(QString("0")))
,
mIGainLabel(new QLabel("I gain"))
,
mIGainEditor(new QLineEdit(QString("0")))
,
mIMaxLabel(new QLabel("I max"))
,
mIMaxEditor(new QLineEdit(QString("50")))
,
mIErrorMaxLabel(new QLabel("I error max"))
,
mIErrorMaxEditor(new QLineEdit(QString("10")))
,
mPWMPeriodLabel(new QLabel("PWM period"))
,
mPWMPeriodEditor(new QLineEdit(QString("65536")))
,
mGroup(QString("Parameters"))
,
mLayout()
{
    mLayout.addWidget(mTargetLabel, 0, 0);
    mLayout.addWidget(mTargetEditor, 0, 1);
    mTargetLabel->setToolTip("Select target temperature in Celcius");
    mTargetEditor->setToolTip("Any temperature is acceptable");
    connect(
        mTargetEditor, &QLineEdit::returnPressed,
        this, &Parameters::changeParams
    );

    mLayout.addWidget(mPGainLabel, 1, 0);
    mLayout.addWidget(mPGainEditor, 1, 1);
    mPGainLabel->setToolTip("Select proportional gain of controller");
    mPGainEditor->setToolTip("Positive number");
    connect(
        mPGainEditor, &QLineEdit::returnPressed,
        this, &Parameters::changeParams
    );

    mLayout.addWidget(mIGainLabel, 2, 0);
    mLayout.addWidget(mIGainEditor, 2, 1);
    mIGainLabel->setToolTip("Select integral gain of controller");
    mIGainEditor->setToolTip("Positive number");
    connect(
        mIGainEditor, &QLineEdit::returnPressed,
        this, &Parameters::changeParams
    );

    mLayout.addWidget(mIMaxLabel, 3, 0);
    mLayout.addWidget(mIMaxEditor, 3, 1);
    mIMaxLabel->setToolTip("Select maximum value of integral of controller");
    mIMaxEditor->setToolTip("Value between 0 and 100");
    connect(
        mIMaxEditor, &QLineEdit::returnPressed,
        this, &Parameters::changeParams
    );

    mLayout.addWidget(mIErrorMaxLabel, 4, 0);
    mLayout.addWidget(mIErrorMaxEditor, 4, 1);
    mIErrorMaxLabel->setToolTip("Select maximum value of error: if exceeded, integral of controller does not work");
    mIErrorMaxEditor->setToolTip("Value between 0 and 50");
    connect(
        mIErrorMaxEditor, &QLineEdit::returnPressed,
        this, &Parameters::changeParams
    );

    mLayout.addWidget(mPWMPeriodLabel, 5, 0);
    mLayout.addWidget(mPWMPeriodEditor, 5, 1);
    mPWMPeriodLabel->setToolTip("Select PWM period");
    mPWMPeriodEditor->setToolTip("Integer value between 1 and 65536");
    connect(
        mPWMPeriodEditor, &QLineEdit::returnPressed,
        this, &Parameters::changeParams
    );


    mGroup.setLayout(&mLayout);
    setWidget(&mGroup);
}

void
Parameters::changeParams(){
    try{
        emit setParams(
            mPGainEditor->text().toFloat(), // pgain
            mIGainEditor->text().toFloat(), // igain
            mIMaxEditor->text().toFloat(), // imax
            mIErrorMaxEditor->text().toFloat(), // ierrormax
            mTargetEditor->text().toFloat(), // target
            mPWMPeriodEditor->text().toInt()
        );
    }
    catch(std::exception const & e){
        std::cout << e.what() << std::endl;
        std::cout << "Invalid input number" << std::endl;
    }
}