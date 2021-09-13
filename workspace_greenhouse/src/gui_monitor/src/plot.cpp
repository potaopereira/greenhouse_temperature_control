// Plot
#include "gui_monitor/plot.hpp"

Plot::Plot(
    QWidget *parent
):
QCustomPlot(parent)
,
mIndex(0)
,
mTime(24*60, 0) // 10 seconds simulation <-> 1 day real
,
mTemperature(24*60, 0) // 10 seconds simulation <-> 1 day real
,
mTemperatureTarget(24*60, 0) // 10 seconds simulation <-> 1 day real
,
mWindowCommand(24*60, 0) // 10 seconds simulation <-> 1 day real
,
mTimeNow(1, 0)
,
mTemperatureNow(1, 0)
,
mTemperatureTargetNow(1, 0)
,
mWindowCommandNow(1, 0)
{

    
    for (int i = 0; i < 24*60; ++i)
        mTime[i] = i;

    // create graphs
    addGraph();
    graph(0)->setData(mTime, mTemperature);
    graph(0)->setPen(QPen(QColor(139, 0, 0)));
    addGraph();
    graph(1)->setData(mTime, mTemperatureTarget);
    graph(1)->setPen(QPen(QColor(0, 100, 0)));
    addGraph();
    graph(2)->setData(mTime, mWindowCommand);
    graph(2)->setPen(QPen(QColor(0, 0, 139)));

    addGraph();
    graph(3)->setData(mTimeNow, mTemperatureNow);
    graph(3)->setPen(QPen(QColor(139, 0, 0)));
    graph(3)->setLineStyle(QCPGraph::lsNone);
    graph(3)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle, 4));
    addGraph();
    graph(4)->setData(mTimeNow, mTemperatureTargetNow);
    graph(4)->setPen(QPen(QColor(0, 100, 0)));
    graph(4)->setLineStyle(QCPGraph::lsNone);
    graph(4)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle, 4));
    addGraph();
    graph(5)->setData(mTimeNow, mWindowCommandNow);
    graph(5)->setPen(QPen(QColor(0, 0, 139)));
    graph(5)->setLineStyle(QCPGraph::lsNone);
    graph(5)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle, 4));

    // give the axes some labels:
    xAxis->setLabel("time");
    yAxis->setLabel("Celcius or Percentage");

    // prepare x axis with country labels:
    QVector<double> ticks;
    QVector<QString> labels;
    for(int i = 0; i < 25; ++i){ 
        ticks << 60*i; // show a tick for every hour (1 hour is 60 minutes, and we get data correponding to 1 minute)
        labels << QString(QString::number(i));
    }
    QSharedPointer<QCPAxisTickerText> textTicker(new QCPAxisTickerText);
    textTicker->addTicks(ticks, labels);
    xAxis->setTicker(textTicker);
    // xAxis->setTickLabelRotation();
    // xAxis->setSubTicks(false);
    xAxis->setTickLength(0, 4);
    // xAxis->setBasePen(QPen(Qt::white));
    // xAxis->setTickPen(QPen(Qt::white));
    xAxis->grid()->setVisible(true);
    xAxis->grid()->setPen(QPen(QColor(130, 130, 130), 0, Qt::DotLine));
    // xAxis->setTickLabelColor(Qt::white);
    // xAxis->setLabelColor(Qt::white);

    // xAxis2->setLabel("time");
    // yAxis2->setLabel("Percentage");
    // yAxis2->setVisible(true);
    // set axes ranges, so we see all data:
    xAxis->setRange(0, 24*60);
    yAxis->setRange(-0.1, 40);

    setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
    axisRect()->setRangeDrag(Qt::Vertical);
    axisRect()->setRangeZoom(Qt::Vertical);

    setToolTip("Plot the results of three days: Temperature in red, Target temperature in green and window openess in blue");

    setMinimumWidth(500);
    setMinimumHeight(500);
}

void
Plot::plot(
    double temperature,
    double target_temperature,
    double percentage
){
    setData(
        temperature,
        target_temperature,
        percentage
    );
    plotData();
}

void
Plot::setData(
    double temperature,
    double target_temperature,
    double percentage
){
    mTemperature[mIndex] = temperature;
    mTemperatureTarget[mIndex] = target_temperature;
    mWindowCommand[mIndex] = percentage;

    mTimeNow[0] = mIndex;
    mTemperatureNow[0] = temperature;
    mTemperatureTargetNow[0] = target_temperature;
    mWindowCommandNow[0] = percentage;

    ++mIndex;
    mIndex = mIndex % (24*60);
}

void
Plot::plotData(

){
    // update plot only once every 10 calls
    static int c = -1;
    ++c;
    c = c%10;
    if(c==0){
        graph(0)->setData(mTime, mTemperature);
        graph(1)->setData(mTime, mTemperatureTarget);
        graph(2)->setData(mTime, mWindowCommand);
        graph(3)->setData(mTimeNow, mTemperatureNow);
        graph(4)->setData(mTimeNow, mTemperatureTargetNow);
        graph(5)->setData(mTimeNow, mWindowCommandNow);
        replot();
    }
}