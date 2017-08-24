/*
 * AttitudeFeedForward.cpp
 *
 *  Created on: Jun 26, 2017
 *      Author: matija
 */

#include "AttitudeFeedForward.h"


AttitudeFeedForward::AttitudeFeedForward()
{
    //Set initial params in constructor
    u_ff_ = 0.0;
    u_ff_old_ = 0.0;
    ud_old_ = 0.0;

    Kff_ = 0.002;

    //disturbace is modeled as descrete dynamic element:
    // yd = a1 * yd(k-1) + b1 * ud_(k-1)
    // coeficient a1_ represemts motor time constant
    a1_ = 0.0125;
    b1_ = 1 - a1_;
}

AttitudeFeedForward::~AttitudeFeedForward()
{
    //nothing to do in destructor
}

float AttitudeFeedForward::feedForwardCompute(float ud)
{
    float uff;

    uff = a1_ * u_ff_old_ + Kff_ * b1_ * ud_old_;

    //save the state of controller
    u_ff_ = uff;
    u_ff_old_ = u_ff_;
    ud_old_ = ud;

    return uff;
}

void AttitudeFeedForward::setFeedForwardGain(float kff)
{
    Kff_ = kff;
}

void AttitudeFeedForward::setFeedForwardModel(float timeConstant, float sampleTime, float modelFasterFactor)
{
    Tm_ = timeConstant/(float)modelFasterFactor;
    Ts_ = sampleTime;

    a1_ = Tm_ / (float)(Tm_+Ts_);
    b1_ = 1- a1_;
}
