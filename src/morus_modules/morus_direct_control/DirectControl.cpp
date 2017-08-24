/*
 * DirectControl.cpp
 *
 *  Created on: Jul 7, 2017
 *      Author: matija
 */

#include "DirectControl.h"

namespace direct_control
{
    DirectControl *instance;
}

DirectControl::DirectControl():
    direct_control_node_handle_(direct_control_app_state_),
    direct_control_loop_rate_(100),
    throttle_(0.0),
    pitch_(0.0),
    roll_(0.0),
    yaw_(0.0),
    armed_(false),
    yaw_control_(false),
    direct_control_task_(-1),
    direct_control_task_should_exit_(false)
{
    //No need for anything else
}

DirectControl::~DirectControl()
{
    if(direct_control_task_ != -1)
    {
        direct_control_task_should_exit_ = true;
        int i = 0;
        //Sleep for 0.5 s to ensure task exits
        while(direct_control_task_ != -1)
        {
            usleep(10000);
            if(++i > 50)
                px4_task_delete(direct_control_task_);
                break;
        }
    }

    direct_control::instance = nullptr;
}


int DirectControl::start()
{
    ASSERT(direct_control_task_ == -1);

    //Start task
    direct_control_task_ = px4_task_spawn_cmd("direct_actuator_control",
            SCHED_DEFAULT,
            SCHED_PRIORITY_MAX-5,
            1600,
            (px4_main_t)&DirectControl::directControlTaskTrampoline,
            nullptr);

    if(direct_control_task_ < 0)
    {
        PX4_WARN("Failed to start task!");
        return -errno;
    }

    return OK;
}

void DirectControl::directControlTaskTrampoline(int argc, char *argv[])
{
    direct_control::instance->directControlTaskMain();
    PX4_INFO("Returning from direct control task trampoline!");
    return;
}

void DirectControl::directControlTaskMain()
{
    //Subscriber
    direct_control_node_handle_.subscribe<px4::px4_actuator_armed>(
            &DirectControl::attitudeReferenceCallback,this,1);
    direct_control_node_handle_.subscribe<px4::px4_actuator_controls_3>(
    		&DirectControl::controlImputsCallback,this,1);


    //Publisher
    actuator_setpoint_pub_ = direct_control_node_handle_.advertise<px4::px4_actuator_controls_1>();

    //Starting main loop at 100 Hz
    PX4_INFO("Starting control!");
    while(!direct_control_task_should_exit_)
    {
        direct_control_loop_rate_.sleep();
        direct_control_node_handle_.spinOnce();

        //publish control values
        if(armed_)
        {
            /*PX4_INFO("ROLL %f  PITCH %f THROTTLE: %f",
                    static_cast<double>(roll_), static_cast<double>(pitch_), static_cast<double>(throttle_)); */
			float mot1, mot2, mot3, mot4;
			
            mot1 =(2.0f*throttle_ - (0.08f*pitch_)) -1.0f;
            mot3 =(2.0f*throttle_ + (0.08f*pitch_)) -1.0f;
            mot2 =(2.0f*throttle_ + (0.08f*roll_)) -1.0f;
            mot4 =(2.0f*throttle_ - (0.08f*roll_)) -1.0f;
           

            //Saturation
            saturation(mot1);
            saturation(mot2);
            saturation(mot3);
            saturation(mot4);
			
			px4::px4_actuator_controls_1 actuator_setpoint;
			
            actuator_setpoint.data().control[0] = mot1;
            actuator_setpoint.data().control[1] = mot2;
            actuator_setpoint.data().control[2] = mot3;
            actuator_setpoint.data().control[3] = mot4;
            
            //PX4_INFO("throttle %d, roll %d, pitch %d", (int)(1000*throttle_), (int)(1000*roll_), (int)(1000*pitch_));


            actuator_setpoint.data().timestamp = hrt_absolute_time();

            actuator_setpoint_pub_->publish(actuator_setpoint);


        }
        else
        {	
        	px4::px4_actuator_controls_1 actuator_setpoint;
            //Send minimum pwm when dissarmed
            actuator_setpoint.data().control[0] = -1.0f;
            actuator_setpoint.data().control[1] = -1.0f;
            actuator_setpoint.data().control[2] = -1.0f;
            actuator_setpoint.data().control[3] = -1.0f;
            
            actuator_setpoint.data().timestamp = hrt_absolute_time();

            actuator_setpoint_pub_->publish(actuator_setpoint);
        }
        
    }
    PX4_WARN("Exiting main task!");
}

void DirectControl::attitudeReferenceCallback(const px4::px4_actuator_armed &msg)
{
    armed_ = msg.data().armed;
}

void DirectControl::saturation(float &value)
{
    if(value > 1.0f)
        value = 1.0f;
    if(value < -1.0f)
        value = -1.0f;
    return;
}


void DirectControl::controlImputsCallback(const px4::px4_actuator_controls_3 &msg)
{
	pitch_ 		= msg.data().control[1];
	roll_ 		= msg.data().control[0];
	throttle_ 	= msg.data().control[3];
	yaw_		= msg.data().control[2];
	yaw_control_= false;
}
