// Gui
#include "gui_monitor/gui.hpp"

Gui::Gui(
    QWidget *parent
):
QWidget(parent)
,
mPlot(this)
,
mParam(this)
{
    mLayout.addWidget(&mPlot);
    mLayout.addWidget(&mParam);
    setLayout(&mLayout);
    mLayout.setStretch(0, 2);

    connect(
        &mParam, &Parameters::setParams,
        this, &Gui::changeParams
    );
}

void
Gui::plot(
    double temperature,
    double target_temperature,
    double percentage
){
    mPlot.plot(
        temperature,
        target_temperature,
        percentage
    );
}

void
Gui::changeParams(
    float pgain,
    float igain,
    float imax,
    float ierrormax,
    float target,
    int pwmperiod
){
    emit setParams(
        pgain,
        igain,
        imax,
        ierrormax,
        target,
        pwmperiod
    );
}
