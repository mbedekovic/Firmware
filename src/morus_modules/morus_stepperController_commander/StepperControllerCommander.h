/*
 * StepperControllerCommander.h
 *
 *  Created on: Mar 30, 2017
 *      Author: matija
 */

#ifndef STEPPERCONTROLLERCOMMANDER_H_
#define STEPPERCONTROLLERCOMMANDER_H_

#include <px4.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>

//#include <build_nuttx_px4fmu-v2_default_morus/src/platforms/nuttx/px4_messages/px4_moving_mass_setpoint_array.h>
#include <platforms/nuttx/px4_messages/px4_moving_mass_setpoint_array.h>

#include <serial_lib.h>

#include "stepper_controller_params.h"
#include "StepperControllerParams.h"

class StepperControllerCommander;
namespace stepper_commander
{
    extern StepperControllerCommander *instance;
}

class StepperControllerCommander {
public:
    // Constructor
    StepperControllerCommander();
    //Destructor
    ~StepperControllerCommander();
    // Starts the app
    int start();
private:
    //Node handle for uORB msg framework
    px4::NodeHandle node_handle_;

    //Variable used to track the state of the task (running or stopped)
    px4::AppState app_state_;

    //Parameters structure
    StepperControllerParams params_;

    //Commander loop rate
    px4::Rate loop_rate_;

    //Task process ID (PID or file descriptor FD); Stores returned value from builtin
    //  function px4_task_spawn_cmd.
    int stepper_commander_task_;
    //Flag for killing the task
    bool stepper_commander_task_should_exit_;

    //Serial port member
    SerialPort *port_;

    /*---------------------------------------------*/

    //Function given by reference to px4_task_spawn_cmd. Within it the real main task
    //  function is called. This is used so the main task function doesn't receive
    //  any arguments in it's call.
    static void taskMainTrampoline(int argc, char *argv[]);
    //Real task main function
    void taskMain();

    //Callback function for moving mass position reference
    void movingMassSetpointCallback(const px4::px4_moving_mass_setpoint_array &msg);
    //Parameters update callback function
    void parametersUpdateCallback(const px4::px4_parameter_update &msg);

    //Sends parameters to stepper controller
    void sendParameters(void);
};

#endif /* STEPPERCONTROLLERCOMMANDER_H_ */
