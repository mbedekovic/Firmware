/*
 * AttitudeControlSS.cpp
 *
 *  Created on: Jun 12, 2017
 *      Author: matija
 */

#include "AttitudeControlSS.h"


AttitudeControlSS::AttitudeControlSS() {
    PX4_INFO("Initializing matrices");
    //set initial values of signals
    ui_old_ = 0.0;
    dead_zone_ = 0.0;
    saturation_ = FLT_MAX; //float max
    Td_ = 0.01;
}

AttitudeControlSS::~AttitudeControlSS() {
    //Empty destructor because we dont take up any memory
}

void AttitudeControlSS::setAMatrix(std::vector<double> double_list)
{
    for(int i = 0; i < n_; i++)
    {
        for(int j = 0; j < n_; j++)
        {
            A_(i,j) = double_list[i*n_ + j];
        }
    }
}

void AttitudeControlSS::setBMatrix(std::vector<double> double_list)
{
    {
        for(int i = 0; i < n_; i++)
        {
            for(int j = 0; j < m_; j++)
            {
                B_(i,j) = double_list[i*m_ + j];
            }
        }
    }
}

void AttitudeControlSS::setCMatrix(std::vector<double> double_list)
{
    {
        for(int i = 0; i < l_; i++)
        {
            for(int j = 0; j < n_; j++)
            {
                C_(i,j) = double_list[i*n_ + j];
            }
        }
    }
}

void AttitudeControlSS::setDMatrix(std::vector<double> double_list)
{
    {
        for(int i = 0; i < l_; i++)
        {
            for(int j = 0; j < m_; j++)
            {
                B_(i,j) = double_list[i*m_ + j];
            }
        }
    }
}

void AttitudeControlSS::setKctlMatrix(std::vector<double> double_list)
{
    {
        for(int i = 0; i < m_; i++)
        {
            for(int j = 0; j < n_; j++)
            {
                Kctl_(i,j) = double_list[i*n_ + j];
            }
        }
    }
}

void AttitudeControlSS::setKobsMatrix(std::vector<double> double_list)
{
    {
        for(int i = 0; i < n_; i++)
        {
            for(int j = 0; j < l_; j++)
            {
                Kobs_(i,j) = double_list[i*l_ + j];
            }
        }
    }
}

void AttitudeControlSS::setKiGain(float Ki)
{
    Ki_ = Ki;
}

void AttitudeControlSS::setDeadZone(double dz)
{
    dead_zone_ = dz;
}

void AttitudeControlSS::setSaturation(double sat)
{
    saturation_ = sat;
}

void AttitudeControlSS::observerCompute(float u, float* y)
{
    Matrix<2,1> y_meas;
    y_meas(0,0) = y[0];
    y_meas(0,1) = y[0];
    y_mv_[0] = y[0];
    y_mv_[1] = y[1];

    //Compute state estimate from measurement, control output and last estimate
    y_est_ = C_*states_;
    states_ = A_ * states_ + B_*u + Kobs_*(y_meas - y_est_);
}

float AttitudeControlSS::stateSpaceCtlCompute(float y_sp, float y_mv, float u_ff)
{
    float ui, kui, ux, u, error;
    Matrix<1,1> u_st;

    //calculate integral prat of control input
    error = y_sp - y_mv;
    if (error > 0)
    {
        if(error < dead_zone_)
            error = 0.0;
        else
            error = error - dead_zone_;
    }
    else
    {
        if(error > -dead_zone_)
            error = 0.0;
        else
            error = error + dead_zone_;
    }
    ui = ui_old_ + error;
    kui = -1*Ki_*ui;

    //calculate state spece output
    u_st = -Kctl_*states_;
    ux = u_st(0,0);

    //combined output
    u = kui + ux + u_ff;    //u_ff is feedforward loop output

    //saturation
    if(u > saturation_)
    {
     u = saturation_;
     ui = ui_old_;      //anti-wind up clamping
    }
    else if(u < -saturation_)
    {
        u = -saturation_;
        ui = ui_old_;      //anti-wind up clamping
    }

    //saving "new" old values
    ui_old_ = ui;
    u_ = u;
    ui_ = kui;
    us_ = ux;
    y_sp_ = y_sp;
    return u;
}




