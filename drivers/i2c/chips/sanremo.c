/*
 * * Monahans Sanremo PMIC Management Routines
 * *
 * *
 * * Copyright (C) 2009, Marvell Corporation
 * * Author: Bin Yang <bin.yang@marvell.com> 
 * *				 Yael Sheli Chemla<yael.s.shemla@marvell.com> 
 * *
 * * This software program is licensed subject to the GNU General Public License
 * * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html
 * */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/kthread.h>
#include <linux/wait.h>
#include <asm/uaccess.h>
#include <plat/pxa3xx_pmic.h>
#include <mach/sanremo.h>
#include <mach/portofino.h>
#include <linux/input.h>
#include <mach/regs-mpmu.h>
#include <asm/io.h>

#define SANREMO_REG_NUM		0xef
static struct input_dev *onkey_input_dev;

extern int get_pm_state(void);
static struct power_supply_module *sanremo_power_module;
static unsigned int event;
static struct i2c_client *g_client;
static int charging_status;
static int usb_connect;
DECLARE_WAIT_QUEUE_HEAD(charging_display_wait);

static int sanremo_version;
struct timer_list	on_key_timer;
static int sanremo_on_key(int enable);
#define ONKEY_TIMER_TIMEOUT (8*HZ)

int sanremo_get_headset_state(void)
{
	u8 tmp;

	if (sanremo_version < 0x48) {
		sanremo_read(SANREMO_STATUS, &tmp);
		return (tmp & SANREMO_MICIN_STATUS) ? 1 : 0;
	} else {
		sanremo_read(SANREMO_STATUS_B0, &tmp);
		return (tmp & SANREMO_HEADSET_STATUS) ? 0 : 1;
	}
}

int sanremo_read(u8 reg, u8 *pval)
{
	struct sanremo_platform_data *pdata = g_client->dev.platform_data;
	int ret;
	int status;

	if (g_client == NULL)	/* No global client pointer? */
		return -ENODEV;

	spin_lock(&pdata->lock);
	disable_irq(g_client->irq);
	ret = i2c_smbus_read_byte_data(g_client, reg);
	enable_irq(g_client->irq);
	if (ret >= 0) {
		*pval = ret;
		status = 0;
	} else
		status = -EIO;
	spin_unlock(&pdata->lock);

	return status;
}
EXPORT_SYMBOL(sanremo_read);

int sanremo_write(u8 reg, u8 val)
{
	struct sanremo_platform_data *pdata = g_client->dev.platform_data;
	int ret;
	int status;

	if (g_client == NULL)	/* No global client pointer? */
		return -ENODEV;

	spin_lock(&pdata->lock);
	disable_irq(g_client->irq);
	ret = i2c_smbus_write_byte_data(g_client, reg, val);
	enable_irq(g_client->irq);
	if (ret == 0)
		status = 0;
	else
		status = -EIO;
	spin_unlock(&pdata->lock);

	return status;
}
EXPORT_SYMBOL(sanremo_write);

int sanremo_enable_pen_down_irq(int enable)
{
	u8 tmp;
	if(enable) {
		sanremo_read(SANREMO_INTERRUPT_ENABLE3, &tmp); 
		if(!(tmp & SANREMO_PEN_MASK)) {
			tmp |= SANREMO_PEN_MASK;
			sanremo_write(SANREMO_INTERRUPT_ENABLE3, tmp); 
		}
	} else {
		sanremo_read(SANREMO_INTERRUPT_ENABLE3, &tmp); 
		if(tmp & SANREMO_PEN_MASK) {
			tmp &= ~SANREMO_PEN_MASK;
			sanremo_write(SANREMO_INTERRUPT_ENABLE3, tmp); 
		}
	}
	return 0;
}

int sanremo_enable_mic_irq(int enable)
{
	u8 tmp;
	if (enable) {
		sanremo_read(SANREMO_INTERRUPT_ENABLE3, &tmp);
		if (!(tmp & SANREMO_MICIN_MASK)) {
			tmp |= SANREMO_MICIN_MASK;
			sanremo_write(SANREMO_INTERRUPT_ENABLE3, tmp);
		}
	} else {
		sanremo_read(SANREMO_INTERRUPT_ENABLE3, &tmp);
		if (tmp & SANREMO_MICIN_MASK) {
			tmp &= ~SANREMO_MICIN_MASK;
			sanremo_write(SANREMO_INTERRUPT_ENABLE3, tmp);
		}
	}
	return 0;
}

int sanremo_enable_headset_irq(int enable)
{
	u8 tmp;
	if (enable) {
		sanremo_read(SANREMO_INTERRUPT_ENABLE3, &tmp);
		if (!(tmp & SANREMO_HEADSET_MASK)) {
			tmp |= SANREMO_HEADSET_MASK;
			sanremo_write(SANREMO_INTERRUPT_ENABLE3, tmp);
		}
	} else {
		sanremo_read(SANREMO_INTERRUPT_ENABLE3, &tmp);
		if (tmp & SANREMO_HEADSET_MASK) {
			tmp &= ~SANREMO_HEADSET_MASK;
			sanremo_write(SANREMO_INTERRUPT_ENABLE3, tmp);
		}
	}
	return 0;
}

int sanremo_tsi_poweron(void)
{
	/*done in platform init*/
	return 0;
}

int sanremo_tsi_poweroff(void)
{
	/*don't disable GPADC because CP will use it*/
	return 0;
}

int sanremo_tsi_readxy(u16 *x, u16 *y, int *pen_state)
{
	u8 meas1, meas2;

	sanremo_read(SANREMO_TSIX_MEAS1, &meas1); 
	sanremo_read(SANREMO_TSIX_MEAS2, &meas2); 
	*pen_state = (meas2 & (1<<6))? 1 : 0;
	if(*pen_state) {
		*x = (meas1 << 4) + (meas2 & 0xf);
		sanremo_read(SANREMO_TSIY_MEAS1, &meas1); 
		sanremo_read(SANREMO_TSIY_MEAS2, &meas2); 
		*y = (meas1 << 4) + (meas2 & 0xf);
	}
	return 0;
}

int sanremo_tsi_enable_pen(int pen_en)
{
	u8 tmp;
	if(pen_en) {
		sanremo_read(SANREMO_MEAS_ENABLE3, &tmp); 
		tmp |= 0x08;
		sanremo_write(SANREMO_MEAS_ENABLE3, tmp); 
	} else {
		sanremo_read(SANREMO_MEAS_ENABLE3, &tmp); 
		tmp &= ~0x08;
		sanremo_write(SANREMO_MEAS_ENABLE3, tmp); 
	}
	return 0;
}

int sanremo_tsi_enable_mic(int mic_en)
{
	u8 tmp;
	if (sanremo_version < 0x48) {
		if (mic_en) {
			sanremo_read(SANREMO_AUDIO_REG_BASE + SANREMO_AUDIO_MIC_BUTTON_DETECTION, &tmp);
			tmp |= 0x27;
			sanremo_write(SANREMO_AUDIO_REG_BASE + SANREMO_AUDIO_MIC_BUTTON_DETECTION, tmp);
		} else {
			sanremo_read(SANREMO_AUDIO_REG_BASE + SANREMO_AUDIO_MIC_BUTTON_DETECTION, &tmp);
			tmp &= ~0x27;
			sanremo_write(SANREMO_AUDIO_REG_BASE + SANREMO_AUDIO_MIC_BUTTON_DETECTION, tmp);
		}
	} else {
		if (mic_en) {
			sanremo_read(SANREMO_AUDIO_MIC_BUTTON_DETECTION_B0, &tmp);
			tmp |= 0x27;
			sanremo_write(SANREMO_AUDIO_MIC_BUTTON_DETECTION_B0, tmp);
		} else {
			sanremo_read(SANREMO_AUDIO_MIC_BUTTON_DETECTION_B0, &tmp);
			tmp &= ~0x27;
			sanremo_write(SANREMO_AUDIO_MIC_BUTTON_DETECTION_B0, tmp);
		}
	}
	return 0;
}

int sanremo_tsi_enable_headset(int headset_en)
{
	u8 tmp;
	if (sanremo_version < 0x48) {
		if (headset_en) {
			sanremo_read(SANREMO_AUDIO_REG_BASE + SANREMO_AUDIO_HEADSET_DETECTION, &tmp);
			tmp |= 0x01;
			sanremo_write(SANREMO_AUDIO_REG_BASE + SANREMO_AUDIO_HEADSET_DETECTION, tmp);
		} else {
			sanremo_read(SANREMO_AUDIO_REG_BASE + SANREMO_AUDIO_HEADSET_DETECTION, &tmp);
			tmp &= ~0x01;
			sanremo_write(SANREMO_AUDIO_REG_BASE + SANREMO_AUDIO_HEADSET_DETECTION, tmp);
		}
	} else {
		if (headset_en) {
			sanremo_read(SANREMO_AUDIO_HEADSET_DETECTION_BO, &tmp);
			tmp |= 0x01;
			sanremo_write(SANREMO_AUDIO_HEADSET_DETECTION_BO, tmp);
		} else {
			sanremo_read(SANREMO_AUDIO_HEADSET_DETECTION_BO, &tmp);
			tmp &= ~0x01;
			sanremo_write(SANREMO_AUDIO_HEADSET_DETECTION_BO, tmp);
		}
	}
	return 0;
}

int sanremo_tsi_enable_tsi(int tsi_en)
{
	u8 tmp;
	if(tsi_en) {
		sanremo_read(SANREMO_MEAS_ENABLE3, &tmp); 
		tmp |= 0x30;
		sanremo_write(SANREMO_MEAS_ENABLE3, tmp); 
	} else {
		sanremo_read(SANREMO_MEAS_ENABLE3, &tmp); 
		tmp &= ~0x30;
		sanremo_write(SANREMO_MEAS_ENABLE3, tmp); 
	}
	return 0;
}

#define VIBRA_OFF_VALUE	0
#define VIBRA_ON_VALUE	0x30	/* you can choose the value of bit6 ~ bit1 */
															/*	 to change the freq of the vibrator */
int sanremo_set_vibrator(unsigned char value)
{
	if(value == 0) {
		sanremo_write(SANREMO_VIBRA_SET, 0x00);
		sanremo_write(SANREMO_VIBRA_PWM, 0x00);
	} else {
		sanremo_write(SANREMO_VIBRA_PWM, 0xc0);
		sanremo_write(SANREMO_VIBRA_SET, 0x01|((value&0x7)<<1));
	}
	return 0;
}
EXPORT_SYMBOL(sanremo_set_vibrator);

int sanremo_usb_connect(void)
{
	return usb_connect;
}
EXPORT_SYMBOL(sanremo_usb_connect);

int sanremo_get_vbat(void)
{
	u8 tmp;
	unsigned int val;

	sanremo_read(SANREMO_VBAT_MEAS2, &tmp);
	val = tmp&0xf;
	sanremo_read(SANREMO_VBAT_MEAS1, &tmp);
	val += tmp<<4;

	return val*131836/100000;
}

static int sanremo_charging_display(void *data)
{
	u8 tmp;
	u8 save;

	while(1) {
		wait_event(charging_display_wait, charging_status);
		tmp = 0;
		portofino_read(0xd, &save);
		while(charging_status) {
			portofino_write(0xd, tmp?save:0);
			tmp = !tmp;
			msleep(500);
		}
		portofino_write(0xd, save);
	}
	return 0;
}

static void sanremo_start_charging(void)
{
	u8 tmp;
	unsigned int val;

	/*check for battery*/
	sanremo_read(SANREMO_STATUS, &tmp);
	if(!(tmp & SANREMO_BAT_STATUS)) {
		printk(KERN_INFO "Battery is NOT detected, ignore charging\n");
		return;
	}else {
		printk(KERN_INFO "Battery is detected\n");
	}

	/*check charging*/
	sanremo_read(SANREMO_STATUS, &tmp);
	if(!(tmp & SANREMO_CHG_STATUS)) {
		printk(KERN_INFO "Charger cable is NOT detected\n");
		return;
	}else {

		printk(KERN_INFO "Charger cable is detected\n");
	}

	val = sanremo_get_vbat();

	if(val < 3850) {
		printk(KERN_INFO "Voltage is about %dmv, start charging\n", val);

		/*enable TIT and IBAT measurement*/
		sanremo_read(SANREMO_MEAS_ENABLE1, &tmp);
		sanremo_write(SANREMO_MEAS_ENABLE1, tmp|SANREMO_MEAS_EN1_TINT);
		sanremo_read(SANREMO_MEAS_ENABLE3, &tmp);
		sanremo_write(SANREMO_MEAS_ENABLE3, tmp|SANREMO_MEAS_EN3_IBAT);
		/*set a off time*/
		if (sanremo_version < 0x48) {
			sanremo_write(SANREMO_MEAS_OFF_TIME1, 0x41);
			sanremo_read(SANREMO_MEAS_OFF_TIME2, &tmp);
			sanremo_write(SANREMO_MEAS_OFF_TIME2, (tmp&0x3f)|0x6c);
		} else {
			sanremo_read(SANREMO_MEAS_ENABLE3, &tmp);
			sanremo_write(SANREMO_MEAS_ENABLE3, tmp|SANREMO_MEASOFFTIME1_BAT_DET_EN_B0);
			sanremo_write(SANREMO_MEAS_OFF_TIME1, 0x80);
			sanremo_write(SANREMO_MEAS_OFF_TIME2, (tmp&0x7f)|0xd8);
		}
		/* enable GPADC FSM */
		sanremo_read(SANREMO_GPADC_MISC1, &tmp);
		sanremo_write(SANREMO_GPADC_MISC1, tmp|SANREMO_GPADC_MISC1_GPFSM_EN);
		/*enable interrupts*/
		sanremo_read(SANREMO_INTERRUPT_ENABLE3, &tmp);
		sanremo_write(SANREMO_INTERRUPT_ENABLE3, 
				tmp|SANREMO_CHG_FAIL_MASK|SANREMO_CHG_DONE_MASK|
				SANREMO_CHG_IOVER_MASK);

		/* set pre-regulator to 1000mA & Vsys to 4.5v*/
		portofino_write(PORTOFINO_PREREG1,0x09);
		/*set the end of charge value and enable s ILIM FSM*/
		sanremo_write(SANREMO_CHG_CTRL1, 0x30); 
		/*set current limit at 550 mA*/
		sanremo_write(SANREMO_CHG_CTRL2, 0x2a); 
		/*set the max charging time to 90 min*/
		sanremo_write(SANREMO_CHG_CTRL3, 0x30); 
		/*enable monitoring*/
		sanremo_write(SANREMO_CHG_CTRL4, 0x40); 

		sanremo_write(SANREMO_GPADC0_MEAS1, 0x82); 

		charging_status = 1;
		wake_up(&charging_display_wait);

		/*enable fast charing*/
		sanremo_write(SANREMO_CHG_CTRL1, 0x32); 
		
		/*set the temperature threshold */
		sanremo_write(SANREMO_TINT_MEAS1, 0xc0); 
		sanremo_write(SANREMO_TINT_MEAS2, 0x82); 
		sanremo_write(SANREMO_GPADC0_MEAS1, 0x7d); 
	}
}

static void sanremo_stop_charging(void)
{
	u8 tmp;
	
	printk("sanremo stop charging\n");
	/*disable TIT and IBAT measurement*/
	sanremo_read(SANREMO_MEAS_ENABLE1, &tmp);
	sanremo_write(SANREMO_MEAS_ENABLE1, tmp&(~SANREMO_MEAS_EN1_TINT));
	sanremo_read(SANREMO_MEAS_ENABLE3, &tmp);
	sanremo_write(SANREMO_MEAS_ENABLE3, tmp&(~SANREMO_MEAS_EN3_IBAT));
	/*disable GPADC FSM */
	sanremo_read(SANREMO_GPADC_MISC1, &tmp);
	sanremo_write(SANREMO_GPADC_MISC1, tmp&(~SANREMO_GPADC_MISC1_GPFSM_EN));
	/*enable interrupts*/
	sanremo_read(SANREMO_INTERRUPT_ENABLE3, &tmp);
	sanremo_write(SANREMO_INTERRUPT_ENABLE3, 
			tmp&(~(SANREMO_CHG_FAIL_MASK|SANREMO_CHG_DONE_MASK|
			SANREMO_CHG_IOVER_MASK)));
	charging_status = 0;
}

static void sanremo_charger_init(void)
{
	u8 tmp;
	int val;

	/*enable interrupt*/
	sanremo_read(SANREMO_INTERRUPT_ENABLE1, &tmp);
	sanremo_write(SANREMO_INTERRUPT_ENABLE1, 
			tmp|SANREMO_CHG_MASK);


	sanremo_read(SANREMO_INTERRUPT_ENABLE1, &tmp);

	/*detection*/
	sanremo_read(SANREMO_MEAS_OFF_TIME1, &tmp);
	sanremo_write(SANREMO_MEAS_OFF_TIME1, tmp|0x1);
	if (sanremo_version < 0x48) {
		sanremo_read(SANREMO_GPADC_MISC2, &tmp);
		sanremo_write(SANREMO_GPADC_MISC2, tmp|0x40);
	} else {
		sanremo_read(SANREMO_CHG_CTRL6, &tmp);
		sanremo_write(SANREMO_CHG_CTRL6, tmp|0x01);
	}

	sanremo_write(SANREMO_VBAT_LOW_TH, 0xaa);
	sanremo_read(SANREMO_MEAS_ENABLE1, &tmp);
	sanremo_write(SANREMO_MEAS_ENABLE1, tmp|SANREMO_MEAS_EN1_VBAT);
/*
	sanremo_read(SANREMO_INTERRUPT_ENABLE2, &tmp);
	sanremo_write(SANREMO_INTERRUPT_ENABLE2, 
			tmp|SANREMO_VBAT_MASK);
*/

	kthread_run(sanremo_charging_display,	NULL, "cdisplay");

//	sanremo_start_charging();
}

static void sanremo_turn_off_booster(void)
{
	/* turn off booster by writing 0 to duty cycle string1/2/3 */
	portofino_write(0x03,0x0);

	return;
}

void sanremo_turn_off_power(void)
{
	u8 tmp;

	printk("turning off power....\n");
	sanremo_read(SANREMO_RESET_OUT, &tmp);
	sanremo_write(SANREMO_RESET_OUT, tmp | 0x80);

	return;
}

static int onkey_report(struct input_dev *idev)
{
	printk("------report key--------\n");
	input_report_key(idev, KEY_POWER, 1);
	input_report_key(idev, KEY_POWER, 0);
	input_sync(idev);
	return 0;
}

static void sanremo_on_key_event(void)
{
	u8 tmp;
	unsigned int val;

	/*check for battery*/
	sanremo_read(SANREMO_STATUS, &tmp);
	if(tmp & SANREMO_ONKEY_STATUS) {
		printk(KERN_INFO "On key pressed\n");
		sanremo_turn_off_booster();
		sanremo_turn_off_power();
		return;
	}else {
		printk(KERN_INFO "On key released\n");
	}
}

static unsigned int sanremo_event_change(void)
{
	unsigned int ret = 0;
	u8 status1, status2, status3;
	u8 e1, e2, e3;
	u8 tmp;

	sanremo_read(SANREMO_INTERRUPT_STATUS1, &status1);
	sanremo_read(SANREMO_INTERRUPT_STATUS2, &status2);
	sanremo_read(SANREMO_INTERRUPT_STATUS3, &status3);
	sanremo_write(SANREMO_INTERRUPT_STATUS1, status1);
	sanremo_write(SANREMO_INTERRUPT_STATUS2, status2);
	sanremo_write(SANREMO_INTERRUPT_STATUS3, status3);
	sanremo_read(SANREMO_INTERRUPT_ENABLE1, &e1);
	sanremo_read(SANREMO_INTERRUPT_ENABLE2, &e2);
	sanremo_read(SANREMO_INTERRUPT_ENABLE3, &e3);

	if (status1 & SANREMO_EXTON_INT)
		ret |= PMIC_EVENT_EXTON;

	if (status1 & SANREMO_CHG_INT) {
		ret |= PMIC_EVENT_CHARGER | PMIC_EVENT_VBUS;
		sanremo_read(SANREMO_STATUS, &tmp);
		usb_connect = !!(tmp & SANREMO_CHG_STATUS);
		sanremo_start_charging();
	}

	if (status1 & SANREMO_ONKEY_INT) {
		ret |= PMIC_EVENT_ON_KEY;
		if (mod_timer(&on_key_timer, jiffies + ONKEY_TIMER_TIMEOUT)) {
			/* timer is active already */
		}
	}

	/*
	if (status2 & SANREMO_VBAT_MASK) {
		printk(KERN_INFO "Battery is low.\n");
		sanremo_start_charging();
	}
	*/

	if (status2 & SANREMO_VCHG_INT) {
		ret |= PMIC_EVENT_VBUS;
	}

	if (status2 & SANREMO_GPADC1_INT)
		ret |= PMIC_EVENT_TBAT;

	if(status3 & SANREMO_PEN_INT)
		ret |= PMIC_EVENT_TOUCH;

	if (status3 & SANREMO_HEADSET_INT)
		ret |= PMIC_EVENT_HSDETECT;

	if (status3 & SANREMO_MICIN_INT)
		ret |= PMIC_EVENT_MICDETECT;

	if (status3 & SANREMO_HOOK_INT)
		ret |= PMIC_EVENT_HOOKSWITCH;

	if (status3 & SANREMO_CHG_FAIL_INT) {
		ret |= PMIC_EVENT_CHARGER;
		if(charging_status) {
			printk("charging failed.\n");
			sanremo_stop_charging();
		}
	}
	if (status3 & SANREMO_CHG_DONE_INT) {
		ret |= PMIC_EVENT_CHARGER;
		if(charging_status) {
			printk("charging done\n");
			sanremo_stop_charging();
		}
	}
	if (status3 & SANREMO_CHG_IOVER_INT) {
		ret |= PMIC_EVENT_REV_IOVER;
		if(charging_status) {
			printk("over charging.\n");
			sanremo_stop_charging();
		}
	}

	return ret;
}

static void sanremo_worker(struct work_struct *work)
{
	unsigned long flags;
	struct sanremo_platform_data *pdata = g_client->dev.platform_data;

	/* clear the irq */
	pdata->ack_irq();
	event |= sanremo_event_change();
	pmic_event_handle(event);
	event = 0;
	enable_irq(g_client->irq);
}

static irqreturn_t sanremo_irq_handler(int irq, void *dev_id)
{
	struct i2c_client *client = dev_id;
	struct sanremo_platform_data *pdata = client->dev.platform_data;

	disable_irq(client->irq);
	schedule_work(&pdata->work);

	return IRQ_HANDLED;
}

#ifdef	CONFIG_PM
static int sanremo_suspend(struct i2c_client *client, pm_message_t state)
{
	return 0;
}

static int sanremo_resume(struct i2c_client *client)
{
	return 0;
}
#else
#define	sanremo_suspend		NULL
#define	sanremo_resume		NULL
#endif

/* Get voltage */
static int get_power_supply_module(int cmd)
{
	int power_module, i;

	if (cmd < VCC_CORE || cmd >= MAX_CMD) {
		printk(KERN_WARNING "input wrong command: %d\n", cmd);
		return -EINVAL;
	}

	for (i = 0; sanremo_power_module[i].command != 0; i++) {
		if (sanremo_power_module[i].command == cmd) {
			power_module = sanremo_power_module[i].power_module;
			break;
		}
	}

	return power_module;
}

static int get_sanremo_voltage(int cmd, int *pmv)
{
	int power_module, status = 0, data;
	u8 val;

	*pmv = -1;

	power_module = get_power_supply_module(cmd);

	start_calc_time();
	switch (power_module) {
	case SAN_BUCK1:
	case SAN_BUCK1_SLEEP:
		status = sanremo_read(SANREMO_REG(power_module), &val);
		if (!status) {
			data = val & 0x3f;
			*pmv = SANREMO_VBUCK1_MV(data);
		}
		break;
	case SAN_BUCK2:
	case SAN_BUCK2_SLEEP:
		status = sanremo_read(SANREMO_REG(power_module), &val);
		if (!status) {
			data = val & 0x3f;
			*pmv = SANREMO_VBUCK2_MV(data);
		}
		break;
	case SAN_BUCK3:
	case SAN_BUCK3_SLEEP:
		status = sanremo_read(SANREMO_REG(power_module), &val);
		if (!status) {
			data = val & 0x3f;
			*pmv = SANREMO_VBUCK3_MV(data);
		}
		break;
	case SAN_LDO1:
		status = sanremo_read(SANREMO_LDO1, &val);
		if (!status) {
			data = val & 0x3f;
			*pmv = SANREMO_LDO1_MV(data);
		}
		break;
	case SAN_LDO1_SLEEP:
		status = sanremo_read(SANREMO_LDO1, &val);
		if (!status) {
			data = (val >> 2) & 0x03;
			*pmv = SANREMO_LDO1_SLEEP_MV(data);
		}
		break;
	case SAN_LDO2:
	case SAN_LDO3:
	case SAN_LDO4:
	case SAN_LDO7:
	case SAN_LDO8:
	case SAN_LDO9:
	case SAN_LDO14:
		status = sanremo_read(SANREMO_REG(power_module), &val);
		if (!status) {
			data = val & 0x07;
			*pmv = SANREMO_LDO2_MV(data);
		}
		break;
	case SAN_LDO2_SLEEP:
	case SAN_LDO3_SLEEP:
	case SAN_LDO4_SLEEP:
	case SAN_LDO7_SLEEP:
	case SAN_LDO8_SLEEP:
	case SAN_LDO9_SLEEP:
	case SAN_LDO14_SLEEP:
		status = sanremo_read(SANREMO_REG(power_module), &val);
		if (!status) {
			data = (val >> 3) & 0x07;
			*pmv = SANREMO_LDO2_MV(data);
		}
		break;
	case SAN_LDO5:
		status = sanremo_read(SANREMO_REG(power_module), &val);
		if (!status) {
			data = val & 0x03;
			*pmv = SANREMO_LDO5_MV(data);
		}
		break;
	case SAN_LDO5_SLEEP:
		status = sanremo_read(SANREMO_REG(power_module), &val);
		if (!status) {
			data = (val >> 2) & 0x03;
			*pmv = SANREMO_LDO5_SLEEP_MV(data);
		}
		break;
	case SAN_LDO6:
		status = sanremo_read(SANREMO_REG(power_module), &val);
		if (!status) {
			data = val & 0x03;
			*pmv = SANREMO_LDO6_MV(data);
		}
		break;
	case SAN_LDO6_SLEEP:
		status = sanremo_read(SANREMO_REG(power_module), &val);
		if (!status) {
			data = (val >> 2) & 0x03;
			*pmv = SANREMO_LDO6_MV(data);
		}
		break;
	case SAN_LDO10:
		status = sanremo_read(SANREMO_REG(power_module), &val);
		if (!status) {
			data = val & 0x0f;
			*pmv = SANREMO_LDO10_MV(data);
		}
		break;
	case SAN_LDO10_SLEEP:
		status = sanremo_read(SANREMO_REG(power_module), &val);
		if (!status) {
			data = (val >> 4) & 0x0f;
			*pmv = SANREMO_LDO10_MV(data);
		}
		break;
	case SAN_LDO12:
		status = sanremo_read(SANREMO_REG(power_module), &val);
		if (!status) {
			data = val & 0x0f;
			*pmv = SANREMO_LDO12_MV(data);
		}
		break;
	case SAN_LDO12_SLEEP:
		status = sanremo_read(SANREMO_REG(power_module), &val);
		if (!status) {
			data = (val >> 4) & 0x0f;
			*pmv = SANREMO_LDO10_MV(data);
		}
		break;
	default:
		printk(KERN_WARNING "input wrong command: %d\n", cmd);
		return -EINVAL;
	}
	end_calc_time();
	return status;
}

static int set_sanremo_voltage(int cmd, int mv)
{
	int power_module, status = 0, cnt;
	u8 val;

	power_module = get_power_supply_module(cmd);

	start_calc_time();
	switch (power_module) {
	case SAN_BUCK1:
		status = sanremo_read(SANREMO_MISC1, &val);
		if (status)
			break;
		/* General I2C port can control V_BUCK1 domain */
		val &= ~0x01;
		status = sanremo_write(SANREMO_MISC1, val);
		if (status)
			break;
		status = sanremo_write(SANREMO_REG(power_module),
					SANREMO_VBUCK1_CNT(mv));
		if (status)
			break;
		status = sanremo_read(SANREMO_GO, &val);
		if (status)
			break;
		/* enable V_BUCK1 change */
		val |= 0x01;
		status = sanremo_write(SANREMO_GO, val);
		break;
	case SAN_BUCK1_SLEEP:
		status = sanremo_write(SANREMO_REG(power_module),
					SANREMO_VBUCK1_CNT(mv));
		break;
	case SAN_BUCK2:
	case SAN_BUCK2_SLEEP:
		status = sanremo_write(SANREMO_REG(power_module),
					SANREMO_VBUCK2_CNT(mv));
		break;
	case SAN_BUCK3:
	case SAN_BUCK3_SLEEP:
		status = sanremo_write(SANREMO_REG(power_module),
					SANREMO_VBUCK3_CNT(mv));
		break;
	case SAN_LDO1:
		status = sanremo_read(SANREMO_REG(power_module), &val);
		if (!status) {
			val &= ~0x03;
			cnt = SANREMO_LDO1_CNT(mv);
			if (cnt == -1) {
				status = -EINVAL;
				break;
			}
			val |= cnt;
			status = sanremo_write(SANREMO_REG(power_module), val);
		}
		break;
	case SAN_LDO1_SLEEP:
		status = sanremo_read(SANREMO_REG(power_module), &val);
		if (!status) {
			val = (val >> 2) & ~0x03;
			cnt = SANREMO_LDO1_SLEEP_CNT(mv);
			if (cnt == -1) {
				status = -EINVAL;
				break;
			}
			val |= cnt;
			status = sanremo_write(SANREMO_REG(power_module), val);
		}
		break;
	case SAN_LDO2:
	case SAN_LDO3:
	case SAN_LDO4:
	case SAN_LDO7:
	case SAN_LDO8:
	case SAN_LDO9:
	case SAN_LDO14:
		status = sanremo_read(SANREMO_REG(power_module), &val);
		if (!status) {
			val &= ~0x07;
			cnt = SANREMO_LDO2_CNT(mv);
			if (cnt == -1) {
				status = -EINVAL;
				break;
			}
			val |= cnt;
			status = sanremo_write(SANREMO_REG(power_module), val);
		}
		break;
	case SAN_LDO2_SLEEP:
	case SAN_LDO3_SLEEP:
	case SAN_LDO4_SLEEP:
	case SAN_LDO7_SLEEP:
	case SAN_LDO8_SLEEP:
	case SAN_LDO9_SLEEP:
	case SAN_LDO14_SLEEP:
		status = sanremo_read(SANREMO_REG(power_module), &val);
		if (!status) {
			val = (val >> 3) & ~0x07;
			cnt = SANREMO_LDO2_CNT(mv);
			if (cnt == -1) {
				status = -EINVAL;
				break;
			}
			val |= cnt;
			status = sanremo_write(SANREMO_REG(power_module), val);
		}
		break;
	case SAN_LDO5:
		if (mv == 0) {
			/* Disable LDO5 output */
			status = sanremo_read(SANREMO_SUPPLIES_EN11, &val);
			if (!status)
				status = sanremo_write(SANREMO_SUPPLIES_EN11,
						val & ~0x80);
			return status;
		}

		status = sanremo_read(SANREMO_REG(power_module), &val);
		if (!status) {
			val = val & ~0x03;
			cnt = SANREMO_LDO5_CNT(mv);
			if (cnt == -1) {
				status = -EINVAL;
				break;
			}
			val |= cnt;
			status = sanremo_write(SANREMO_REG(power_module), val);
		}

		/* enable LDO5 output */
		status = sanremo_read(SANREMO_SUPPLIES_EN11, &val);
		if (!status)
			status = sanremo_write(SANREMO_SUPPLIES_EN11,
				val | 0x80);
		break;
	case SAN_LDO5_SLEEP:
		status = sanremo_read(SANREMO_REG(power_module), &val);
		if (!status) {
			val = (val >> 2) & ~0x03;
			cnt = SANREMO_LDO5_SLEEP_CNT(mv);
			if (cnt == -1) {
				status = -EINVAL;
				break;
			}
			val |= cnt;
			status = sanremo_write(SANREMO_REG(power_module), val);
		}
		break;
	case SAN_LDO6:
		status = sanremo_read(SANREMO_REG(power_module), &val);
		if (!status) {
			val = val & ~0x07;
			cnt = SANREMO_LDO6_CNT(mv);
			if (cnt == -1) {
				status = -EINVAL;
				break;
			}
			val |= cnt;
			status = sanremo_write(SANREMO_REG(power_module), val);
		}
		break;
	case SAN_LDO6_SLEEP:
		status = sanremo_read(SANREMO_REG(power_module), &val);
		if (!status) {
			val = (val >> 3) & ~0x07;
			cnt = SANREMO_LDO6_CNT(mv);
			if (cnt == -1) {
				status = -EINVAL;
				break;
			}
			val |= cnt;
			status = sanremo_write(SANREMO_REG(power_module), val);
		}
		break;
	case SAN_LDO10:
		status = sanremo_read(SANREMO_REG(power_module), &val);
		if (!status) {
			val = val & ~0x0f;
			cnt = SANREMO_LDO10_CNT(mv);
			if (cnt == -1) {
				status = -EINVAL;
				break;
			}
			val |= cnt;
			status = sanremo_write(SANREMO_REG(power_module), val);
		}
		break;
	case SAN_LDO10_SLEEP:
		status = sanremo_read(SANREMO_REG(power_module), &val);
		if (!status) {
			val = (val >> 4) & ~0x0f;
			cnt = SANREMO_LDO10_CNT(mv);
			if (cnt == -1) {
				status = -EINVAL;
				break;
			}
			val |= cnt;
			status = sanremo_write(SANREMO_REG(power_module), val);
		}
		break;
	case SAN_LDO12:
		status = sanremo_read(SANREMO_REG(power_module), &val);
		if (!status) {
			val = val & ~0x0f;
			cnt = SANREMO_LDO12_CNT(mv);
			if (cnt == -1) {
				status = -EINVAL;
				break;
			}
			val |= cnt;
			status = sanremo_write(SANREMO_REG(power_module), val);
		}
		break;
	case SAN_LDO12_SLEEP:
		status = sanremo_read(SANREMO_REG(power_module), &val);
		if (!status) {
			val = (val >> 4) & ~0x0f;
			cnt = SANREMO_LDO10_CNT(mv);
			if (cnt == -1) {
				status = -EINVAL;
				break;
			}
			val |= cnt;
			status = sanremo_write(SANREMO_REG(power_module), val);
		}
		break;
	default:
		printk(KERN_WARNING "input wrong command: %d\n", cmd);
		return -EINVAL;
	}
	end_calc_time();
	return status;
}

static int is_sanremo_vbus_assert(void)
{
	u8 tmp;

	sanremo_read(SANREMO_INTERRUPT_ENABLE1, &tmp); 
	sanremo_write(SANREMO_INTERRUPT_ENABLE1, tmp | SANREMO_CHG_MASK);
	sanremo_read(SANREMO_INTERRUPT_ENABLE1, &tmp); 

	sanremo_read(SANREMO_STATUS, &tmp); 
	if (tmp & SANREMO_CHG_STATUS)
		return 1;

	return 0;
}
static int is_sanremo_avbusvld(void)
{
	return 0;
}

static int is_sanremo_asessvld(void)
{
	return 0;
}

static int is_sanremo_bsessvld(void)
{
	return 0;
}

static int is_sanremo_srp_ready(void)
{
	return 0;
}

static int sanremo_set_pump(int enable)
{
	u8 tmp;

	if(enable) {
		sanremo_read(SANREMO_INTERRUPT_ENABLE1, &tmp); 
		tmp |= SANREMO_CHG_MASK;
		sanremo_write(SANREMO_INTERRUPT_ENABLE1, tmp); 
	} else {
		sanremo_read(SANREMO_INTERRUPT_ENABLE1, &tmp); 
		tmp &= ~SANREMO_CHG_MASK;
		sanremo_write(SANREMO_INTERRUPT_ENABLE1, tmp); 
	}
	return 0;
}

static int sanremo_on_key(int enable)
{
	u8 tmp;
	if(enable) {
		init_timer(&on_key_timer);
		on_key_timer.function = sanremo_on_key_event;
		on_key_timer.data = (long)NULL;
		sanremo_read(SANREMO_INTERRUPT_ENABLE1, &tmp); 
		tmp |= SANREMO_ONKEY_MASK;
		sanremo_write(SANREMO_INTERRUPT_ENABLE1, tmp); 
	} else {
		sanremo_read(SANREMO_INTERRUPT_ENABLE1, &tmp); 
		tmp &= ~SANREMO_ONKEY_MASK;
		sanremo_write(SANREMO_INTERRUPT_ENABLE1, tmp); 
	}
	return 0;
}

static int sanremo_set_vbus_supply(int enable, int srp)
{
	u8 tmp;
	sanremo_read(SANREMO_MISC1, &tmp);
	if (enable) {
		tmp |= SANREMO_MISC1_GPIO2_VAL;
	} else {
		tmp &= ~SANREMO_MISC1_GPIO2_VAL;
	}
	sanremo_write(SANREMO_MISC1, tmp);
	tmp |= SANREMO_MISC1_GPIO2_DIR;
	sanremo_write(SANREMO_MISC1, tmp);

	sanremo_read(SANREMO_MISC1, &tmp);

	pr_debug("%s enable %d misc1 %x\n\n", __func__, enable, tmp);

	return 0;
}
static int sanremo_set_usbotg_a_mask(void)
{
	return 0;
}
static int sanremo_set_usbotg_b_mask(void)
{
	return 0;
}

static struct pmic_ops sanremo_pmic_ops = {
	.get_voltage		= get_sanremo_voltage,
	.set_voltage		= set_sanremo_voltage,

	.is_vbus_assert		= is_sanremo_vbus_assert,
	.is_avbusvld		= is_sanremo_avbusvld,
	.is_asessvld		= is_sanremo_asessvld,
	.is_bsessvld		= is_sanremo_bsessvld,
	.is_srp_ready		= is_sanremo_srp_ready,

	.set_pump		= sanremo_set_pump,
	.set_vbus_supply	= sanremo_set_vbus_supply,
	.set_usbotg_a_mask	= sanremo_set_usbotg_a_mask,
	.set_usbotg_b_mask	= sanremo_set_usbotg_b_mask,
};

#ifdef	CONFIG_PROC_FS
#define	SANREMO_PROC_FILE	"driver/sanremo"
static struct proc_dir_entry *sanremo_proc_file;
static int index;

static ssize_t sanremo_proc_read(struct file *filp,
		char *buffer, size_t length, loff_t *offset)
{
	u8 reg_val;

	if (index == 0xffff) {
		int i;
		for(i=0; i<SANREMO_REG_NUM; i++) {
			sanremo_read(i, &reg_val);
			printk(KERN_INFO "[0x%02x]=0x%02x\n", i, reg_val);
		}
		return 0;
	}
	if ((index < 0) || (index > SANREMO_REG_NUM))
		return 0;

	sanremo_read(index, &reg_val);
	printk(KERN_INFO "register 0x%x: 0x%x\n", index, reg_val);
	return 0;
}

static ssize_t sanremo_proc_write(struct file *filp,
		const char *buff, size_t len, loff_t *off)
{
	u8 reg_val;
	char messages[256], vol[256];

	if (len > 256)
		len = 256;

	if (copy_from_user(messages, buff, len))
		return -EFAULT;

	if ('-' == messages[0]) {
		/* set the register index */
		memcpy(vol, messages+1, len-1);
		index = (int) simple_strtoul(vol, NULL, 16);
	}else if ('v' == messages[0]) {
		/* set the register index */
		memcpy(vol, messages+1, len-1);
		index = (int) simple_strtoul(vol, NULL, 16);
		if(index > 5) 
			index = 5;
		sanremo_set_vibrator((unsigned char) index);
	}else if ('b' == messages[0]) {
		printk("vbat is about %dmv\n", sanremo_get_vbat());
	} else {
		/* set the register value */
		reg_val = (int)simple_strtoul(messages, NULL, 16);
		sanremo_write(index, reg_val & 0xFF);
	}

	return len;
}

static struct file_operations sanremo_proc_ops = {
	.read = sanremo_proc_read,
	.write = sanremo_proc_write,
};

static void create_sanremo_proc_file(void)
{
	sanremo_proc_file = create_proc_entry(SANREMO_PROC_FILE, 0644, NULL);
	if (sanremo_proc_file) {
		sanremo_proc_file->owner = THIS_MODULE;
		sanremo_proc_file->proc_fops = &sanremo_proc_ops;
	} else
		printk(KERN_INFO "proc file create failed!\n");
}

static void remove_sanremo_proc_file(void)
{
	extern struct proc_dir_entry proc_root;
	remove_proc_entry(SANREMO_PROC_FILE, &proc_root);
}

#endif

static int sanremo_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct sanremo_platform_data *pdata;
	int ret;
	int chip_id;
	int i;
	u8 tmp;

	chip_id = i2c_smbus_read_byte_data(client, SANREMO_ID);
	if (chip_id < 0) {
		printk(KERN_WARNING "sanremo unavailable!\n");
		return -ENXIO;
	} else {
		printk(KERN_INFO "sanremo(chip id:0x%02x) detected.\n", chip_id);
	}

	sanremo_version = chip_id;
	g_client = client;

	pdata = client->dev.platform_data;
	/* init spinlock */
	spin_lock_init(&pdata->lock);

#if 1
	/* hw fix*/
	spin_lock(&pdata->lock);
	disable_irq(g_client->irq);
	i2c_smbus_write_byte(client, 0xfa);
	i2c_smbus_write_byte(client, 0xfb);
	i2c_smbus_write_byte(client, 0xff);
	i2c_smbus_write_byte_data(client, 0xd6, 0x0a);
	i2c_smbus_write_byte_data(client, 0xd7, 0x02);
	i2c_smbus_write_byte(client, 0xfe);
	enable_irq(g_client->irq);
	spin_unlock(&pdata->lock);
#endif

	/*init RTC clock*/
	sanremo_write(SANREMO_RTC1, 0x40);
	sanremo_read(SANREMO_RTC_MISC2, &tmp);
	tmp |= 0x2;
	sanremo_write(SANREMO_RTC_MISC2, tmp);

	/* init workqueue */
	INIT_WORK(&pdata->work, sanremo_worker);


	/* init irq */
	pdata->init_irq();
	/* FIXME: define trigger type in platform data later */
	ret = request_irq(client->irq, sanremo_irq_handler, IRQF_DISABLED,
			"Sanremo", client);
	if (ret) {
		printk(KERN_WARNING "Request IRQ for Sanremo failed, return:%d\n",
				ret);
		return ret;
	}

	pdata->platform_init();

	for (i = 0; pdata->power_chips[i].power_supply_modules != NULL; i++) {
		if (pdata->power_chips[i].chip_id == chip_id) {
			sanremo_power_module = \
				pdata->power_chips[i].power_supply_modules;
			break;
		}
	}
	if (pdata->power_chips[i].power_supply_modules == NULL)
		panic("This Sanremo chip is not supported");

#ifdef	CONFIG_PROC_FS
	create_sanremo_proc_file();
#endif

	pmic_set_ops(&sanremo_pmic_ops);

#ifdef CONFIG_TOUCHSCREEN_SANREMO
	{
	struct platform_device *pdev;
	if(pdata->tsi) {
		pdev = platform_device_alloc("sanremo_touch", 
				(int)id->driver_data);
		pdev->dev.platform_data = pdata->tsi;
		platform_device_add(pdev);
	}
	}
#endif

	/* enable on key interrupts */
	sanremo_on_key(1);

	sanremo_read(SANREMO_STATUS, &tmp);
	usb_connect = !!(tmp & SANREMO_CHG_STATUS);

	sanremo_charger_init();

	onkey_input_dev = input_allocate_device();
	if (onkey_input_dev == NULL) {
		printk(KERN_ERR "%s: failed to allocate input dev\n",
				__FUNCTION__);
		return -ENOMEM;
	}
	onkey_input_dev->name = "on-key";
	__set_bit(EV_KEY, onkey_input_dev->evbit);
	__set_bit(KEY_POWER, onkey_input_dev->keybit);
	__set_bit(EV_REP, onkey_input_dev->evbit);
	__set_bit(EV_SYN, onkey_input_dev->evbit);

	ret = input_register_device(onkey_input_dev);
	if (ret) {
		printk(KERN_ERR
				"%s: unabled to register input device, ret = %d\n",
				__FUNCTION__, ret);
		return ret;
	}

	return 0;
}

int sanremo_enable_headset_detect(void (*func)(unsigned long ), int enable)
{
	int ret = 0;
	printk(KERN_INFO "enable headset detect!\n");
	if (sanremo_version < 0x48) {
		sanremo_enable_mic_irq(enable);
		sanremo_tsi_enable_mic(enable);
		if (enable)
			ret = pmic_callback_register(PMIC_EVENT_MICDETECT, func);
		else
			pmic_callback_unregister(PMIC_EVENT_MICDETECT, func);
		if (enable && ret < 0)
			printk(KERN_INFO "pmic_callback_register PMIC_EVENT_MICDETECT failed\n");
	} else {
		sanremo_enable_headset_irq(enable);
		sanremo_tsi_enable_headset(enable);
		if (enable)
			ret = pmic_callback_register(PMIC_EVENT_HSDETECT, func);
		else
			pmic_callback_unregister(PMIC_EVENT_HSDETECT, func);
		if (enable && ret < 0)
			printk(KERN_INFO "pmic_callback_register PMIC_EVENT_HSDETECT failed\n");
	}
	func(PMIC_EVENT_HSDETECT);
}

static int sanremo_remove(struct i2c_client *client)
{
	pmic_set_ops(NULL);

#ifdef	CONFIG_PROC_FS
	remove_sanremo_proc_file();
#endif

	free_irq(client->irq, NULL);

	return 0;
}

static const struct i2c_device_id sanremo_id[] = {
	{ "sanremo", 0 },
	{ }
};

static struct i2c_driver sanremo_driver = {
	.driver = {
		.name	= "sanremo",
	},
	.id_table 	= sanremo_id,
	.probe		= sanremo_probe,
	.remove		= sanremo_remove,
	.suspend	= sanremo_suspend,
	.resume		= sanremo_resume,
};

static int __init sanremo_init(void)
{
	return i2c_add_driver(&sanremo_driver);
}

static void __exit sanremo_exit(void)
{
	flush_scheduled_work();
	i2c_del_driver(&sanremo_driver);
}

subsys_initcall(sanremo_init);
module_exit(sanremo_exit);

MODULE_DESCRIPTION("Sanremo Driver");
MODULE_LICENSE("GPL");

