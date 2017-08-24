/*
 * SSControl.cpp
 *
 *  Created on: Jun 27, 2017
 *      Author: matija
 */

#include "SSControl.h"

namespace ss_control
{
    SSControl *instance;
}

SSControl::SSControl():
    ss_control_node_handle_(ss_control_app_state_),
    ss_control_loop_rate_(100),
    armed_flag_(false),
    ss_control_task_(-1),
    ss_control_task_should_exit_(false),
    first_meas_(false),
    u_roll_(0.0),
    u_pitch_(0.0),
    dwx_(0.0),
    dwy_(0.0),
    w0_(923.7384),
    gyro_pt1_const_(0.9)
{
    //setting everinthing to zero in constructor
    attitude_measured_.zero();
    rates_measured_.zero();
    rates_old_.zero();
    attitude_error_.zero();
    attitude_error_previous_.zero();


    //allocate memory for controllers

    pitch_ss_ = new AttitudeControlSS();
    roll_ss_ = new AttitudeControlSS();

    pitch_ff_ = new AttitudeFeedForward();
    roll_ff_ = new AttitudeFeedForward();

    //set matrices here
    std::vector<double> A {0.9922085930872637, 0.008864042317612512, 0.0, 0.0, -1.4961799981731592, 0.7791595631749836,
        0.0, 0.0, 0.002330367307996241, 7.326471989738716E-6, 1.0, 0.01, 0.4654625913871158,
        0.0021542740884284727, 0.0, 1.0};
    std::vector<double> B {0.007791406912736254, 1.4961799981731592, 3.128900584268966E-6, 0.0012366503289861848};
    std::vector<double> C {0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    std::vector<double> D {0.0, 0.0};
    std::vector<double> Kctl {0.6815358682116195, 0.022915395925945936, 0.31479401226319786, 0.15995957603890573};
    double Ki = -0.0026564179826072077;
    std::vector<double> Kobs {0.25789980094063686, 0.5597500633241494, -1.4238982707692978, -3.7884615746715253,
        0.5483407354414322, 0.01588841186484249, 0.2229925816651322, 1.073671143421576};

    //set system matrix
    pitch_ss_->setAMatrix(A);
    pitch_ss_->setBMatrix(B);
    pitch_ss_->setCMatrix(C);
    pitch_ss_->setDMatrix(D);
    pitch_ss_->setDeadZone(0.0);
    pitch_ss_->setSaturation(0.08);

    roll_ss_->setAMatrix(A);
    roll_ss_->setBMatrix(B);
    roll_ss_->setCMatrix(C);
    roll_ss_->setDMatrix(D);
    roll_ss_->setDeadZone(0.0);
    roll_ss_->setSaturation(0.08);

    // setting controller and observer matrix
    pitch_ss_->setKctlMatrix(Kctl);
    pitch_ss_->setKiGain(Ki);
    pitch_ss_->setKobsMatrix(Kobs);

    roll_ss_->setKctlMatrix(Kctl);
    roll_ss_->setKiGain(Ki);
    roll_ss_->setKobsMatrix(Kobs);

    PX4_INFO("Done setting controller matrx");
}

SSControl::~SSControl()
{
    if(ss_control_task_ != -1)
    {
        ss_control_task_should_exit_ = true;
        int i = 0;
        // Sleep for 0.5 s to make sure that task exits
        while(ss_control_task_ != -1)
        {
            usleep(10000);
            if(++i > 50)
            {
                px4_task_delete(ss_control_task_);
                break;
            }
        }
    }

    if(pitch_ss_)
        delete pitch_ss_;
    if(pitch_ff_)
        delete pitch_ff_;
    if(roll_ff_)
        delete roll_ff_;
    if(roll_ss_)
        delete roll_ss_;

    ss_control::instance = nullptr;
}

int SSControl::start()
{
    ASSERT(ss_control_task_ == -1);     //Check if task got the ID

    //Start the control task
    ss_control_task_ = px4_task_spawn_cmd("ss_control",
            SCHED_DEFAULT,
            SCHED_PRIORITY_MAX -5,
            1500,
            (px4_main_t)&SSControl::ssControlTaskTrampoline,
            nullptr);

    if(ss_control_task_ < 0)
    {
        PX4_WARN("Failed to start stte space control task");
        return -errno;
    }

    return OK;
}

void SSControl::ssControlTaskTrampoline(int argc, char *argv[])
{
    ss_control::instance -> ssControlTaskMain();
    PX4_INFO("Returning from state space control task trampoline!");
    return;
}

void SSControl::ssControlTaskMain()
{
    //Subsciebers
    ss_control_node_handle_.subscribe<px4::px4_vehicle_attitude>(
            &SSControl::vehicleAttitudeCallback, this, 1);
    ss_control_node_handle_.subscribe<px4::px4_rc_channels>(
            &SSControl::attitudeReferenceCallback,this,1);

    //Publishers
    moving_mass_setpoint_array_pub_ =
            ss_control_node_handle_.advertise<px4::px4_moving_mass_setpoint_array>();

    while(first_meas_ == false)
    {
        PX4_INFO("Waiting for the first measurement");
        sleep(1);
        ss_control_node_handle_.spinOnce(); //Check for updates on topics
    }

    // set initial states
    pitch_ss_->states_.set_col(0, math::Vector<4>(0.0, 0.0, attitude_measured_(1),rates_measured_(1)));
    roll_ss_->states_.set_col(0,math::Vector<4>(0.0, 0.0, attitude_measured_(0), rates_measured_(0)));

    //local variables for control outputs
    float u_roll_ff;
    float u_pitch_ff;
    float y_obs[2] = {0.0};

    //begin control loop
    PX4_INFO("Starting Control");
    while(!ss_control_task_should_exit_)
    {
        ss_control_loop_rate_.sleep();
        ss_control_node_handle_.spinOnce();

        if(armed_flag_ == true)
        {
          //roll control
            //compute feedforward
            u_roll_ff = roll_ff_->feedForwardCompute(dwy_);
            y_obs[0] = attitude_measured_(0);
            y_obs[1] = rates_measured_(0);
            roll_ss_->observerCompute(u_roll_,y_obs);
            u_roll_ = roll_ss_->stateSpaceCtlCompute(0.0,attitude_measured_(0),u_roll_ff);

          //pitch control
            u_pitch_ff = -pitch_ff_->feedForwardCompute(dwx_);
            y_obs[0] = attitude_measured_(1);
            y_obs[1] = rates_measured_(1);
            pitch_ss_->observerCompute(u_pitch_, y_obs);
            u_pitch_ = pitch_ss_->stateSpaceCtlCompute(0.0,attitude_measured_(1),u_pitch_ff);
        }
        else
        {
            //If not armed set outputs and integral output to zero
            u_pitch_ = 0;
            u_roll_ = 0;
            pitch_ss_->ui_ = 0.0; pitch_ss_->ui_old_ = 0.0;
            roll_ss_->ui_ = 0.0;  roll_ss_->ui_old_ = 0.0;
        }
        publish();  //Publish moving mass setpoint
    }

    PX4_INFO("Returning from state space controller task main");
    return;
}

void SSControl::vehicleAttitudeCallback(const px4::px4_vehicle_attitude &msg)
{
    float p, q, r, sx, cx, cy, ty;

    // Set message quaternion to matrix quaternion
    matrix::Quatf qt(msg.data().q[0], msg.data().q[1], msg.data().q[2], msg.data().q[3]);

    // Quaternion to Euler
    matrix::Eulerf e(qt);

    p = msg.data().rollspeed;
    q = msg.data().pitchspeed;
    r = msg.data().yawspeed;

    sx = sinf(e(0));        //sin(roll)
    cx = cosf(e(0));        //cos(roll)
    cy = cosf(e(1));        //sin(pitch)
    ty = tanf(e(1));        //tan(pitch)
    attitude_measured_(0) = e(0);
    attitude_measured_(1) = e(1);
    attitude_measured_(2) = e(2);

    // gyro mesurements to rollrate, pitchrate, yawrate
    rates_measured_(0) = p + sx * ty * q + cx * ty * r;
    rates_measured_(1) = cx * q - sx * r;
    rates_measured_(2) = sx / cy * q + cx / cy * r;

    //filter rates
    rates_measured_(0) = gyro_pt1_const_*rates_old_(0)+(1-gyro_pt1_const_)*rates_measured_(0);
    rates_measured_(1) = gyro_pt1_const_*rates_old_(1)+(1-gyro_pt1_const_)*rates_measured_(1);
    rates_measured_(2) = gyro_pt1_const_*rates_old_(2)+(1-gyro_pt1_const_)*rates_measured_(2);

    //Save old rates
    rates_old_ = rates_measured_;

    if(first_meas_ == false)
        first_meas_ = true;
}

void SSControl::attitudeReferenceCallback(const px4::px4_rc_channels &msg)
{
    if (msg.data().channels[6] > 0.5f)
    	armed_flag_ = true;
    else if(msg.data().channels[6] < 0.5f)
    	armed_flag_ = false;
    return;
}

void SSControl::publish()
{
    // form a message
    px4::px4_moving_mass_setpoint_array moving_mass_array;

    moving_mass_array.data().position[moving_mass_setpoint_array_s::FRONT] = u_pitch_;
    moving_mass_array.data().position[moving_mass_setpoint_array_s::BACK] = -u_pitch_;

    moving_mass_array.data().position[moving_mass_setpoint_array_s::LEFT] = -u_roll_;
    moving_mass_array.data().position[moving_mass_setpoint_array_s::RIGHT] = u_roll_;

    moving_mass_array.data().timestamp = hrt_absolute_time();
    // Publish
    moving_mass_setpoint_array_pub_->publish(moving_mass_array);
}

