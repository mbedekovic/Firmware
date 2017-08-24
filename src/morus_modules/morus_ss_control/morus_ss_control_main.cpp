#include "SSControl.h"

extern "C" __EXPORT int morus_ss_control_main(int argc, char *argv[]);

int morus_ss_control_main(int argc, char *argv[])
{
    //Show usage to user
    if(argc < 2)
    {
        PX4_WARN("usage: ss_control {start|stop|status}");
        return 1;
    }

    //Try to start the task
    if(!strcmp(argv[1],"start"))
    {
        if (ss_control::instance != nullptr)
        {
            PX4_WARN("allready running");
            return 1;
        }

        ss_control::instance = new SSControl;

        //If there is not enough memory send info and exit
        if (ss_control::instance == nullptr)
        {
            PX4_WARN("Alloc failed");
            return 1;
        }

        //If task couldn't be spawned
        if (OK != ss_control::instance->start())
        {
            delete ss_control::instance;
            ss_control::instance = nullptr;
            PX4_WARN("Start failed");
            return 1;
        }

      return 0;
    }

    //Stopping the task means deleting it
    if (!strcmp(argv[1], "stop"))
    {
        if(ss_control::instance == nullptr)
        {
            PX4_WARN("Not running");
            return 1;
        }

        //If task exist, delte it and set instance to nullptr
        delete ss_control::instance;
        ss_control::instance = nullptr;
        return 0;
    }

    //Status query
    if(!strcmp(argv[1],"status"))
    {
        if(ss_control::instance != nullptr)
        {
            PX4_INFO("running...");
            return 0;
        }
        else
        {
            PX4_INFO("Not running, type morus_ss_contro start");
            return 0;
        }
    }

    //If nothing worked tell user that he is wrong
    PX4_WARN("Unrecognized command");
    return 1;
}
