/*
 * DirectControl.h
 *
 *  Created on: Jul 7, 2017
 *      Author: matija
 */

#ifndef DIRECTCONTROL_H_
#define DIRECTCONTROL_H_

#include <px4.h>

// Standard
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>

// Drivers for time stamp - required for logging
#include <drivers/drv_hrt.h>

//#include <platforms/nuttx/px4_messages/px4_attitude_controller_reference.h>
//#include <build_px4fmu-v2_default/src/platforms/nuttx/px4_messages/px4_actuator_controls.h>
#include <platforms/nuttx/px4_messages/px4_actuator_armed.h>
#include <platforms/nuttx/px4_messages/px4_actuator_controls.h>


class DirectControl;

namespace direct_control
{
    extern DirectControl *instance;
}

class DirectControl {
public:
    DirectControl();
    ~DirectControl();

    int start();

protected:
    px4::NodeHandle direct_control_node_handle_;
    px4::AppState direct_control_app_state_;
private:

    px4::Rate direct_control_loop_rate_;

    //Publishers
    px4::Publisher<px4::px4_actuator_controls_1> *actuator_setpoint_pub_;

    //data memebers to store control imput values
    float throttle_;
    float pitch_;
    float roll_;
    float yaw_;

    //flags
    bool armed_;
    bool yaw_control_;

    //task state signaling flags
    int direct_control_task_;
    bool direct_control_task_should_exit_;

    //Callback for transmitter inputs
    void attitudeReferenceCallback(const px4::px4_actuator_armed &msg);
    void controlImputsCallback(const px4::px4_actuator_controls_3 &msg);
    void rcCallback(const px4::px4_rc_channels &msg);

    //standard functions
    static void directControlTaskTrampoline(int argc, char *argv[]);
    void directControlTaskMain();
    void saturation(float &value);

};

#endif /* DIRECTCONTROL_H_ */
