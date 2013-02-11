#ifndef __RTC_H__
#define __RTC_H__

struct pxa168_rtc_platform_data {
	void (*rtc_write)(u32 rtc);
	void (*rtc_read)(u32 *rtc);
	void (*alarm_write)(u32 rtc);
};

#endif

