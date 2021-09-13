// Simulator
#include "greenhouse_simulator/simulator_impl.hpp"

// M_PI
#include <math.h>

// sin, cos
#include <cmath>

SimulatorImpl::SimulatorImpl(
    int periodMS
):
mPeriodMS(periodMS)
,
mDayPeriodS(10) // one day corresponds to 10 seconds in the simulation
,
mFactorDissipation(0.1)
,
mFactorSunExposureConstant(10)
,
mFactorSunExposure(0.1)
,
mFactorWindow(30)
,
mFactorWindowTimeConstantInv(1/0.1)
{

    //   gsl_odeiv2_system sys = {function, jacobian, dimension, pointer to parameters};
    mSys = {SimulatorImpl::ddtState, NULL, 2, this};

    /*initial temperature*/
    mState[0] = 20;
    /*between 0 and 1*/
    mState[1] = 0.5;
    mTime = 0;

    /*Explicit embedded Runge-Kutta Prince-Dormand (8, 9) method*/
    mDriver = gsl_odeiv2_driver_alloc_y_new(
        &mSys,
        gsl_odeiv2_step_rk8pd, // gsl_odeiv2_step_rkf45
        double(mPeriodMS/2.), // initial step size
        1e-6, // epsabs
        0.0 // epsrel
    );

}

SimulatorImpl::~SimulatorImpl(
){
    gsl_odeiv2_driver_free(mDriver);
}


int
SimulatorImpl::ddtState(
    double t,
    const double y[],
    double f[],
    void* params
)
{
    // (void)(t); /* avoid unused parameter warning */
    SimulatorImpl *p = (SimulatorImpl*) params;

    double temperature = y[0];
    double window = y[1];

    double dissipation = -p->mFactorDissipation*(temperature - p->getOutsideTemperature(t));
    double sun_exposure = p->getSunExposureEffect(t);
    double window_effect = -p->mFactorWindow*window;
    double temperature_dot = dissipation + sun_exposure + window_effect;
    double window_dot = p->mFactorWindowTimeConstantInv*(p->mRequestedWindowPosition - window);

    f[0] = temperature_dot;
    f[1] = window_dot;

    return GSL_SUCCESS;
}

double
SimulatorImpl::getOutsideTemperature(
    double t
){
    return 28 + 5*sin(2*M_PI*t/mDayPeriodS - M_PI/2);
}

double
SimulatorImpl::getSunExposureEffect(
    double t
){
    return mFactorSunExposureConstant + mFactorSunExposure*(1 - cos(2*M_PI*t/mDayPeriodS - M_PI/2));
}

double
SimulatorImpl::evolve(
    double window_position
){
    mRequestedWindowPosition = window_position > 1 ? 1 : (window_position < 0 ? 0 : window_position);
    float time  = mTime + mPeriodMS/1000.;
    int status = gsl_odeiv2_driver_apply(mDriver, &mTime, time, mState);

    if (status != GSL_SUCCESS)
    {
        printf("error, return value=%d\n", status);
        // @todo throw exception here?
    }
    return mState[0];
}