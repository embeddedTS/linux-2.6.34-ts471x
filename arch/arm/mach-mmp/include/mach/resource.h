#ifndef __MACH_RESOURCE
#define __MACH_RESOURCE

struct vibrator_data {
	unsigned char max_v;
	unsigned char min_v;
	int (*set_vibrator)(unsigned char);
};
void res_add_sanremo_vibrator(void);





#endif /*__MACH_RESOURCE*/
