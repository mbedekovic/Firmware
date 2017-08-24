/* morus_stepper_controller_main.cpp */

#include "StepperControllerCommander.h"

extern "C" __EXPORT int morus_stepperController_commander_main(int argc, char *argv[]);

int morus_stepperController_commander_main(int argc, char *argv[])
{
    //Display usage
    if(argc < 2)
    {
        PX4_INFO("usage: stepper_commander {start|stop|satus}");
        return 1;
    }

    //Try to start the task
    if(!strcmp(argv[1],"start"))
    {
        if (stepper_commander::instance != nullptr)
        {
            //Task is already running if memory was allocated
            PX4_WARN("allready running");
            return 1;
        }

        //Allocate memory for the object
        stepper_commander::instance = new StepperControllerCommander;
        //If there is not enough memory (failed allocation) print info and exit
        if(stepper_commander::instance == nullptr)
        {
            PX4_WARN("failed allocation, exiting...");
            return 1;
        }

        //Try to start the task (register it with task scheduler)
        //  if it fails, deallocate memory
        if(OK != stepper_commander::instance->start())
        {
            delete stepper_commander::instance;
            stepper_commander::instance = nullptr;
            PX4_WARN("start failed");
            return 1;
        }

        //If everything passed return 0
        return 0;
    }

    //Stopping the task by deleting it
    if(!strcmp(argv[1],"stop"))
    {
        if(stepper_commander::instance == nullptr)
        {
            PX4_WARN("not running");
            return 1;
        }

        delete stepper_commander::instance;
        stepper_commander::instance = nullptr;
        PX4_INFO("task stopped");
        return 0;
    }

    if(!strcmp(argv[1],"status"))
    {
        if(stepper_commander::instance)
        {
            PX4_INFO("running");
            return 0;
        }
        else
        {
            PX4_INFO("not yet started. usage: stepper_commander {start|stop|status}");
            return 1;
        }
    }

    PX4_WARN("Unrecognized command");
    return 1;
}
