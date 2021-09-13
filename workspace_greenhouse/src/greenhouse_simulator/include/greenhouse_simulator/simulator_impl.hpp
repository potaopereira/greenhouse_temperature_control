/**
 * @file simulator_impl.hpp
 * @author Pedro Pereira (pedro.m.otao.pereira@gmail.com)
 * @brief Simulating the temperature time evolution inside greenhouse
 * 
 * @details We are simulating the system such that 10 seconds-sim correspond to 1 day-real.
 * In a real scenario, we would update the window position every once a minute:
 * which in the simulation it corresponds to
 * 60 seconds-real * 10 seconds-sim / 1 day-real = 60 * 10 / (24*60*60) = 10 / (24 * 60) * 1000 approx 7 milliseconds-sim
 * 
 * \image html images/greenhouse_temperature_control.png
 * 
 * @version 0.1
 * @date 2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#pragma once

// gsl for solving differential equations
#include <gsl/gsl_errno.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_odeiv2.h>

/**
 * @brief Simulator implementation using gsl ode solver
 * 
 */
class SimulatorImpl
{
public:
    /**
     * @brief Construct a simulator object
     * 
     * @param periodMS Period between calls of \p evolve
     */
    SimulatorImpl(
        int periodMS
    );
    /**
     * @brief Destroy a simulator object
     * 
     */
    ~SimulatorImpl();
    /**
     * @brief Get the next temperature, given the provided window position
     * 
     * @param window_position Requested window position
     * @return double Temperature found solving ode
     */
    double evolve(
        double window_position
    );

private:
    /**
     * @brief Period used in solve ODE
     * 
     */
    int mPeriodMS;
    /**
     * @brief How many seconds in the simulation corresonds to one day in real life
     * 
     */
    double mDayPeriodS;
    /**
     * @brief System struc
     * 
     */
    gsl_odeiv2_system mSys;
    /**
     * @brief Driver that solves ODE
     * 
     */
    gsl_odeiv2_driver* mDriver;
    /**
     * @brief State of the system
     * 
     */
    double mState[2];
    /**
     * @brief Current time
     * 
     */
    double mTime;

    /**
     * @brief Factor in dissipation due to temperature gradient
     * 
     */
    double mFactorDissipation;
    /**
     * @brief Constant factor from sun exposure
     * 
     */
    double mFactorSunExposureConstant;
    /**
     * @brief Factor from time varying component in sun exposure
     * 
     */
    double mFactorSunExposure;
    /**
     * @brief Factor from window openess
     * 
     */
    double mFactorWindow;
    /**
     * @brief Inverse of time constant in requested position
     * 
     */
    double mFactorWindowTimeConstantInv;
    /**
     * @brief Requested window position
     * 
     */
    double mRequestedWindowPosition;

    /**
     * @brief ODE
     * 
     * @param t Time instant t
     * @param y Array t solution
     * @param f Array to d/dt solution
     * @param params pointer to object of type SimulatorImpl
     * @return int status integer from gsl
     */
    static
    int ddtState(
        double t,
        const double y[],
        double f[],
        void *params
    );
    /**
     * @brief Get the outside temperature at a given time instant
     * 
     * @param t Time instant
     * @return double Temperature
     */
    double
    getOutsideTemperature(
        double t
    );
    /**
     * @brief Get the sun exposure effect at a given time instant
     * 
     * @param t Time instant
     * @return double Temperature
     */
    double
    getSunExposureEffect(
        double t
    );

};
