#ifndef _POWER_BUTTON_H
#define _POWER_BUTTON_H

struct power_button_platform_data {
	int (*init)(irq_handler_t pwrdwn_handler, irq_handler_t standby_handler);
	void (*send_standby_ack)(void);
	void (*send_powerdwn_ack)(void);
};

#endif
