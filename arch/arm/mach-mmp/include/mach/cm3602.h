#ifndef _CM3602_H
#define _CM3602_H

struct cm3602_platform_data {
	char *name;
	void (*power)(int on);
	int (*get_aout)();
	int (*get_pout)();
};

#endif /*_SENSOR_INPUT_H*/

