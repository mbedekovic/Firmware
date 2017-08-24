/*
 * StepperControllerCommander.cpp
 *
 *  Created on: Mar 30, 2017
 *      Author: matija
 */

#include "StepperControllerCommander.h"

namespace stepper_commander
{
    StepperControllerCommander *instance;
}

StepperControllerCommander::StepperControllerCommander() :
node_handle_(app_state_),
params_(),
loop_rate_(50),
stepper_commander_task_(-1),
stepper_commander_task_should_exit_(false)
{
    //All data members are initialised in initialization list
    port_ = new SerialPort();
}

StepperControllerCommander::~StepperControllerCommander()
{
    if(stepper_commander_task_ != -1)
    {
        stepper_commander_task_should_exit_ = true;
        //free alocated memory for SerialPort class object
        delete port_;
        int i = 0;
        //Sleep for 0.5s to ensure task is properly shut down
        while(stepper_commander_task_ != -1)
        {
            usleep(10000);
            if(++i > 50)
                px4_task_delete(stepper_commander_task_);
                break;
        }
    }
    stepper_commander::instance = nullptr;
}

void StepperControllerCommander::taskMainTrampoline(int argc, char *argv[])
{
    stepper_commander::instance->taskMain();
    PX4_INFO("Returning from stepper controller commander task trampoline! \n");
    return;
}


int StepperControllerCommander::start()
{
    ASSERT(stepper_commander_task_ == -1);

    // Start stepper controller commander in task scheduler
    stepper_commander_task_ = px4_task_spawn_cmd("stepper_commander",
            SCHED_DEFAULT,
            SCHED_PRIORITY_MAX-5,
            1500,
            (px4_main_t)&StepperControllerCommander::taskMainTrampoline,
            nullptr);

    if(stepper_commander_task_ < 0)
    {
        PX4_WARN("Failed to start attitude control task! \n");
        return -errno;
    }

   return OK;
}


void StepperControllerCommander::taskMain()
{
    // Subscribe to moving mass setpoint topic
    node_handle_.subscribe<px4::px4_moving_mass_setpoint_array>(
            &StepperControllerCommander::movingMassSetpointCallback,this,1);
    // Subscribe to parameter server
    node_handle_.subscribe<px4::px4_parameter_update>(
            &StepperControllerCommander::parametersUpdateCallback, this, 1);
    //Open serial port
    port_->openSerial();

    while(!stepper_commander_task_should_exit_)
    {
        //In this loop it waits for topic update and then calls callback function
        loop_rate_.sleep();
        node_handle_.spinOnce();
    }
    //Close serial port
    port_->closeSerial();

    PX4_INFO("Returning from stepper commander task main! \n");
    return;
}

void StepperControllerCommander::movingMassSetpointCallback(const px4::px4_moving_mass_setpoint_array &msg)
{
    //static int flag = 0;
    float pozicija[4];
    int i = 0;

    pozicija[0] = msg.data().position[moving_mass_setpoint_array_s::BACK];
    pozicija[1] = msg.data().position[moving_mass_setpoint_array_s::LEFT];
    pozicija[2] = msg.data().position[moving_mass_setpoint_array_s::FRONT];
    pozicija[3] = msg.data().position[moving_mass_setpoint_array_s::RIGHT];

    for (i=0;i<4;i++)
    {
        pozicija[i] = 5000*pozicija[i]; //5000 pulseva motora za 1 m pomaka mase
    }

    //Print out debugging info every 0.1 s
   /* if((flag++)%200 == 0)
    {
        printf("\033[2J\n"); //clear screen
        //Debugging print to console
        PX4_INFO("FRONT: %f", (double)(pozicija[0]));
        PX4_INFO("LEFT: %f", (double)(pozicija[1]));
        PX4_INFO("BACK: %f", (double)(pozicija[2]));
        PX4_INFO("RIGHT: %f", (double)(pozicija[3]));
        flag = 0;
    } */


    //Forming a serial message frame
    motor_control_t tx_msg;
    tx_msg.cmd = 'C';           //Motor setpoint message identifier
    tx_msg.motor1 = (int)(pozicija[0]);
    tx_msg.motor2 = (int)(pozicija[1]);
    tx_msg.motor3 = (int)(pozicija[2]);
    tx_msg.motor4 = (int)(pozicija[3]);

    //send message over serial
    port_->sendMotorControl(&tx_msg);
}

void StepperControllerCommander::parametersUpdateCallback(const px4::px4_parameter_update &msg)
{
    // Call the parameters class update method
    PX4_INFO("Parameters updated!");
    params_.update();
}

void StepperControllerCommander::sendParameters(void)
{
    motor_control_t msg;
    msg.cmd = 'S'; //Setup message identifier
    msg.motor1 = params_.gain;
    msg.motor2 = params_.omega;
    msg.motor3 = (int)params_.acceleration/100;
    msg.motor4 = (int)params_.deacceleration/100;

    //Send over serial
    port_->sendMotorControl(&msg);
}


