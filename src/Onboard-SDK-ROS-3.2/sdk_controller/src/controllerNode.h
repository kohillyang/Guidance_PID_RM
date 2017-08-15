
#ifndef __CONTROLLER_NODE_H__
#define __CONTROLLER_NODE_H__

const static std::string SCRIPT_PREFIX = "/home/ubuntu/logFile/script/";
const static std::string BASIC_SCRIPT_PREFIX = "/home/ubuntu/logFile/basic/";
const static std::string CALLBACK_SCRIPT_PREFIX = "/home/ubuntu/logFile/callback/";

typedef struct
{
    bool flag;
    uint16_t line_num;
    uint16_t code;

}fail_info;

enum LogicState
{
    INIT		=	0x00,
    GRAB_MODE	=	0x01,
    DROP_MODE	=	0x02,
    CIRCLE_LAND	=	0x03,
    BASE_LAND	=	0x04,
    PATROL_MODE	=	0x05,
    MANUAL_MODE	=	0xFF
};

enum interrupt_flag
{
    CIRCLE_DETECTED         = 0x01,
    CAR_DETECTED            = 0x02,
    CIRCLE_OR_CAR_DETECTED  = 0x03,
    CIRCLE_AND_CAR_DETECTED = 0x04
};

enum execution_flag
{
    NO_INPUT_NO_PUB =   0x0, //represent for no input, no publish;
    INPUT_NO_PUB    =   0x1, //represent for with input, no publish;
    NO_INPUT_PUB    =   0x2, //represent for without input, publish;
    INPUT_PUB       =   0x3, //epresent for with input, publish
    SCRIPT          =   0x4,
	FAIL_SAFE		=	0x5
};

#endif
