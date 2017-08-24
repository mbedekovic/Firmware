/*
 * StepperControllerParams.h
 *
 *  Created on: Apr 20, 2017
 *      Author: matija
 */

#ifndef STEPPERCONTROLLERPARAMS_H_
#define STEPPERCONTROLLERPARAMS_H_

#include <systemlib/param/param.h>
#include <px4_defines.h>
#include <px4.h>

#include "stepper_controller_params.h"

class StepperControllerParams
{
public:
    //Constructor
    StepperControllerParams();
    //Destructor
    ~StepperControllerParams();

    void update();

    //Parameter values
    int acceleration;
    int deacceleration;
    int gain;
    int omega;

private:
    //Parameter handles

    px4::ParameterInt acceleration_handle_;
    px4::ParameterInt deacceleration_handle;
    px4::ParameterInt gain_handle_;
    px4::ParameterInt omega_handle_;
};

#endif /* STEPPERCONTROLLERPARAMS_H_ */
