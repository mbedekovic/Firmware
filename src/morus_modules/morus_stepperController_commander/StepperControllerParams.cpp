/*
 * StepperControllerParams.cpp
 *
 *  Created on: Apr 20, 2017
 *      Author: matija
 */

#include "StepperControllerParams.h"

StepperControllerParams::StepperControllerParams():
acceleration(STEPPER_ACC_RAMP),
deacceleration(STEPPER_DCC_RAMP),
gain(STEPPER_P),
omega(STEPPER_W_MAX),
acceleration_handle_("STP_ACC",STEPPER_ACC_RAMP),
deacceleration_handle("STP_DEC",STEPPER_DCC_RAMP),
gain_handle_("STP_P",STEPPER_P),
omega_handle_("STP_W_MAX",STEPPER_W_MAX)
{
    // parameters and handles init finished, call update function once
   update();
}

StepperControllerParams::~StepperControllerParams()
{
    //Empty destructor because object doesn't allocate aditional memory
}

void StepperControllerParams::update()
{
    //Update parameter values
    acceleration = acceleration_handle_.update();
    deacceleration = deacceleration_handle.update();
    gain = gain_handle_.update();
    omega = omega_handle_.update();
}

