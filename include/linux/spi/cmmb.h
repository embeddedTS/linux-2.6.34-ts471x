#ifndef LINUX_SPI_CMMB_H
#define LINUX_SPI_CMMB_H

struct cmmb_platform_data {
	int (*power_on)(void);
	int (*power_off)(void);
};

#endif
