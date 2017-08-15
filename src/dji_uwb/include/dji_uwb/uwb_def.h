
#ifndef __UWB_DEF_H__
#define __UWB_DEF_H__

#include <dji_uwb/uwb_msg.h>

#define BIT_TEST(x) &(0x01 << (x-1))

namespace dji_uwb {

    typedef struct
    {
        int16   x;
        int16   y;
        uint16  yaw;
        int16   distance[6];
        uint16  error:14;
        uint16  health:2;
        uint16  _reserved_;
    }__attribute__((packed)) UWBInfo;//20-byte

}

#endif
