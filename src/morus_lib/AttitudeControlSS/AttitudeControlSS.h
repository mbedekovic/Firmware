/*
 * AttitudeControlSS.h
 *
 *  Created on: Jun 12, 2017
 *      Author: matija
 */

#ifndef ATTITUDECONTROLSS_H_
#define ATTITUDECONTROLSS_H_

/*#include <px4_eigen.h>*/
//include math headers
#include <mathlib/mathlib.h>
#include <math.h>
#include <float.h>
#include <vector>
#include <px4.h>

using math::Matrix;
using math::Vector;
class AttitudeControlSS {
public:
    static const unsigned int n_ = 4;      //number of states
    static const unsigned int m_ = 1;      //number of inputs
    static const unsigned int l_ = 2;      //number of mesurements (outputs)
    //system matrices
    Matrix<n_,n_> A_;
    Matrix<n_,m_> B_;
    Matrix<l_,n_> C_;
    Matrix<l_,m_> D_;

    //controller and observer matrices
    Matrix<m_,n_> Kctl_;
    Matrix<n_,l_> Kobs_;

    //states and estimation vector
    Matrix<n_,1> states_;
    Matrix<l_,1> y_est_;

    float Ki_;          //integral gain
    float y_sp_;        //angle setpoint
    float y_mv_[2];     //measurement of angle and angle rate
    float ui_old_;       //previous control input
    float u_, ui_, us_; //control output, integral control output, state control output
    float Td_;          //discretization time
    float dead_zone_;   //error deadzone
    float saturation_;  //control value saturation


    //functions for loading the matrix
    void setAMatrix(std::vector<double> double_list);
    void setBMatrix(std::vector<double> double_list);
    void setCMatrix(std::vector<double> double_list);
    void setDMatrix(std::vector<double> double_list);
    void setKobsMatrix(std::vector<double> double_list);
    void setKctlMatrix(std::vector<double> double_list);
    void setKiGain(float Ki);
    void setDeadZone(double dz);
    void setSaturation(double sat);

    //functions for computing controller and observer output
    void observerCompute(float u, float* y);
    float stateSpaceCtlCompute(float y_sp, float y_mv, float u_ff);
    AttitudeControlSS();
    ~AttitudeControlSS();
};


#endif /* ATTITUDECONTROLSS_H_ */
