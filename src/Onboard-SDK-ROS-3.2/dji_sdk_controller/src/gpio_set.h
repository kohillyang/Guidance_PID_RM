
#ifndef __GPIO_SET_H__
#define __GPIO_SET_H__

#define		GPIO157		"/sys/class/gpio/gpio157/value"
#define		GPIO158		"/sys/class/gpio/gpio158/value"
#define		GPIO_HIGH	"1"
#define		GPIO_LOW	"0"

int setGPIO(std::string gpio_pin, std::string val);

#endif
