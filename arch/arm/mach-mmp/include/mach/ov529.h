#ifndef __ASM_ARCH_OV529_H__
#define __ASM_ARCH_OV529_H__

#define PCD_CMD		1
#define PCD_DATA	0

struct ov529_platform_data {
	char name[128];
	int (*init)(void);
	void (*release)(void);
	int (*power_on)(int on);
	void (*turnon_sensor)(void);
	void (*set_ptype)(int on);
	int (*get_sel_uart)(void);
	int (*get_pwait)(void);
	void (*set_pcd)(int on);
	void (*set_rts)(int on);
};

#endif

