
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string>
#include "gpio_set.h"

int setGPIO(std::string gpio_pin, std::string val)
{
	int fd;
	fd = open(gpio_pin.c_str(), O_RDWR);
	
	if(fd == -1)
	{
		return EXIT_FAILURE;
	}
	
	write(fd, val.c_str(), sizeof(val.c_str() ));
    usleep(1E6);
	close(fd);

	return 0;
}

