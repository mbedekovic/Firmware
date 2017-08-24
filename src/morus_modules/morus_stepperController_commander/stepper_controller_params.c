/*
 * stepper_controller_params.c
 *
 *  Created on: Apr 20, 2017
 *      Author: Matija Bedekovic
 */


#include <px4_defines.h>
#include "stepper_controller_params.h"

/*
 * Acceleration of motor
 * @min 1
 * @max 10000
 * @increment 1
 * @group Stepper Controller
 */
PARAM_DEFINE_INT32(STP_ACC,100);

/*
 * Deacceleration of motor
 * @min 1
 * @max 10000
 * @increment 1
 * @group Stepper Controller
 */
PARAM_DEFINE_INT32(STP_DEC,1800);

/*
 * Regulator P gain
 * @min 1
 * @max 100
 * @increment 1
 * @group Stepper Controller
 */

PARAM_DEFINE_INT32(STP_P,5);

/*
 * Maximum motor speed
 * @min 1
 * @max 2000
 * @increment 1
 * @group Stepper Controller
 */

PARAM_DEFINE_INT32(STP_W_MAX, 1200);
