#ifndef __PID_SET_H__
#define __PID_SET_H__

#include <stdio.h>
#include <stdlib.h>

namespace CtrlType{

    typedef struct
    {
        float kp,ki,kd;
        float error;
        float error_last;
        float integral;
        float vel_lim, err_lim;
        float v;
    }pid_T;

    typedef struct
    {
        pid_T vel_loop;
        pid_T pos_loop;
    }dpid_T;

    typedef struct
    {
        float x, y, z;
    }Vector3;
}

#endif
