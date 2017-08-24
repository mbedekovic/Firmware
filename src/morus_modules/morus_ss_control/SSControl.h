/*
 * SSControl.h
 *
 *  Created on: Jun 27, 2017
 *      Author: matija
 */

#ifndef SSCONTROL_H_
#define SSCONTROL_H_

//Pixhawk
#include <px4.h>

//Drivers for time
#include <drivers/drv_hrt.h>

//State space and feedforward library
#include <morus_lib/AttitudeControlSS/AttitudeControlSS.h>
#include <morus_lib/AttitudeControlSS/AttitudeFeedForward.h>

//Topics
#include <platforms/nuttx/px4_messages/px4_attitude_controller_reference.h>
#include <platforms/nuttx/px4_messages/px4_moving_mass_setpoint_array.h>
#include <platforms/nuttx/px4_messages/px4_rc_channels.h>

// Standard
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>

// Math library
#include <lib/mathlib/mathlib.h>
#include <matrix/math.hpp>
#include <matrix/filter.hpp>
#include <matrix/integration.hpp>

class SSControl;

namespace ss_control
{
    extern SSControl *instance;
}

class SSControl {
public:

    int start();            //Starts task in the background (deamon task)

    SSControl();
    ~SSControl();

protected:
    // Node handles for subscribers and publishers
    px4::NodeHandle ss_control_node_handle_;
    px4::AppState ss_control_app_state_;

private:
       //loop rate
       px4::Rate ss_control_loop_rate_;

       //publishers
       px4::Publisher<px4::px4_moving_mass_setpoint_array> *moving_mass_setpoint_array_pub_;

       // ss controller and ff controller instances
       AttitudeControlSS *pitch_ss_;
       AttitudeControlSS *roll_ss_;
       AttitudeFeedForward *pitch_ff_;
       AttitudeFeedForward *roll_ff_;

       //angle and angular velocity measurements
       math::Vector<3> attitude_measured_;
       math::Vector<3> rates_measured_, rates_old_;
       math::Vector<3> attitude_error_, attitude_error_previous_;

       //controller flags
       bool armed_flag_;

       //ss controller ID and runnging check flag
       int ss_control_task_;
       bool ss_control_task_should_exit_;

       //class members
       bool first_meas_;       //indicates that the first measurement has arrieved
       float u_roll_;
       float u_pitch_;
       float dwx_, dwy_;       //BLDC motor speed delta (right stick or output from vx or vy control)
       float w0_;
       float gyro_pt1_const_;      //filter constant


       //task trampoline and task main loop
       static void ssControlTaskTrampoline(int argc, char *argv[]);
       void ssControlTaskMain();

       // Topics callbacks
       void vehicleAttitudeCallback(const px4::px4_vehicle_attitude &msg);
       void attitudeReferenceCallback(const px4::px4_rc_channels &msg);   //for collecting flags
       //Publishing function
       void publish();
};

#endif /* SSCONTROL_H_ */
