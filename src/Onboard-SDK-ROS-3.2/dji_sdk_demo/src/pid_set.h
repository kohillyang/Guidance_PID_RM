#ifndef __PID_SET_H__
#define __PID_SET_H__

#include <stdio.h>
#include <stdlib.h>

#define DEST_X_NLIM -2
#define DEST_X_PLIM 4
#define DEST_Y_NLIM -4
#define DEST_Y_PLIM 8

#define INC_XY_LIM  3
#define INC_Z_LIM   2

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
