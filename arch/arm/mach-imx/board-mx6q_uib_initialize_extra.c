/*
 * Copyright (C) 2012-2014 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/types.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/nodemask.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/i2c.h>
#include <linux/ata.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <linux/regulator/consumer.h>
#include <linux/pmic_status.h>
#include <linux/ipu.h>
#include <linux/mxcfb.h>
#include <linux/pwm_backlight.h>
#include <linux/pwm_generic.h>
#include <linux/fec.h>
#include <linux/memblock.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/suspend.h>
#include <linux/ktime.h>
#include <linux/wakelock.h>
#include <linux/power/sabresd_battery.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/alarmtimer.h>

#include <asm/irq.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>


#include <../drivers/leds/leds.h>

#define IMX_GPIO_NR(bank, nr)           (((bank) - 1) * 32 + (nr))

#define UIB_LCD_LED_EN			IMX_GPIO_NR(7, 12)
#ifdef CONFIG_MX6DL_UIB_REV_1
 #define UIB_LCD_CNTRL_VGH	IMX_GPIO_NR(2, 10)
#else
 #define UIB_LCD_PWR_INH IMX_GPIO_NR(3, 20)
 #define UIB_LCD_STBYB IMX_GPIO_NR(3, 25)
 #define UIB_LCD_RESET IMX_GPIO_NR(3, 27)

 #define UIB_USB_HUB_RESET    IMX_GPIO_NR(5, 30)

 // #define TESTS3 // allow S3 signal test on uSD boot jumper
 #ifdef TESTS3
  #define UIB_S3_PWR_MODE IMX_GPIO_NR(3, 5)		// boot_cfg jumper
  #define get_S3_PWR_MODE() !gpio_get_value(UIB_S3_PWR_MODE)
 #else
  #define UIB_S3_PWR_MODE IMX_GPIO_NR(4, 14)
  #define get_S3_PWR_MODE() gpio_get_value(UIB_S3_PWR_MODE)
 #endif
 #define UIB_SERVER_S5 IMX_GPIO_NR(1, 0)
 #define UIB_FIERY_ON_EN IMX_GPIO_NR(4, 9)
#endif // CONFIG_MX6DL_UIB_REV_1

#define UIB_LED0	IMX_GPIO_NR(1, 2)

#define UIB_TOUCH_IRQ    IMX_GPIO_NR(3, 23)
#define UIB_TOUCH_RESET  IMX_GPIO_NR(3, 24)

#define UIB_RFID_CS0    IMX_GPIO_NR(5, 17)
#define UIB_RFID_EN     IMX_GPIO_NR(3, 30)
#define UIB_RFID_EN2    -1
#define UIB_RFID_IRQ    IMX_GPIO_NR(3, 31)
#define UIB_RFID_MOD    IMX_GPIO_NR(4, 29)
#define UIB_RFID_ASKOOK IMX_GPIO_NR(4, 30)

#define SABRESD_POWER_OFF	IMX_GPIO_NR(3, 8)
#define SABRESD_INFO		IMX_GPIO_NR(3, 9)

#define SABRESD_BT_RESET	IMX_GPIO_NR(1, 2)
#define SABRESD_MICROPHONE_DET	IMX_GPIO_NR(1, 9)
#define SABRESD_CSI0_PWN	IMX_GPIO_NR(1, 16)
#define SABRESD_CSI0_RST	IMX_GPIO_NR(1, 17)
#define SABRESD_ACCL_INT	IMX_GPIO_NR(1, 18)
#define SABRESD_MIPICSI_PWN	IMX_GPIO_NR(1, 19)
#define SABRESD_MIPICSI_RST	IMX_GPIO_NR(1, 20)
#define SABRESD_RGMII_RST	IMX_GPIO_NR(1, 25)
#define SABRESD_RGMII_INT	IMX_GPIO_NR(1, 26)
#define SABRESD_CHARGE_UOK_B	IMX_GPIO_NR(1, 27)
#define SABRESD_USBH1_PWR_EN	IMX_GPIO_NR(1, 29)
#define SABRESD_DISP0_PWR_EN	IMX_GPIO_NR(1, 30)

#define SABRESD_CHARGE_DOK_B	IMX_GPIO_NR(2, 24)
#define SABRESD_GPS_RESET	IMX_GPIO_NR(2, 28)
#define SABRESD_SENSOR_EN	IMX_GPIO_NR(2, 31)

#define SABRESD_PCIE_PWR_EN	IMX_GPIO_NR(3, 19)
#define SABRESD_USB_OTG_PWR_N	IMX_GPIO_NR(3, 22)
#define SABRESD_USB_H1_PWR	IMX_GPIO_NR(1, 29)

#define SABRESD_CAN1_STBY	IMX_GPIO_NR(4, 5)
#define SABRESD_CODEC_PWR_EN	IMX_GPIO_NR(4, 10)
#define SABRESD_HDMI_CEC_IN	IMX_GPIO_NR(4, 11)
#define SABRESD_PCIE_DIS_B	IMX_GPIO_NR(4, 14)

#define SABRESD_DI0_D0_CS	IMX_GPIO_NR(5, 0)
#define SABRESD_CHARGE_FLT_1_B	IMX_GPIO_NR(5, 2)
#define SABRESD_PCIE_WAKE_B	IMX_GPIO_NR(5, 20)

#define SABRESD_DISP_RST_B	IMX_GPIO_NR(6, 11)
#define SABRESD_DISP_PWR_EN	IMX_GPIO_NR(6, 14)
#define SABRESD_CABC_EN0	IMX_GPIO_NR(6, 15)
#define SABRESD_CABC_EN1	IMX_GPIO_NR(6, 16)
#define SABRESD_AUX_3V15_EN	IMX_GPIO_NR(6, 9)
#define SABRESD_DISP0_WR_REVB	IMX_GPIO_NR(6, 9)
#define SABRESD_DI1_D0_CS	IMX_GPIO_NR(6, 31)

#define SABRESD_HEADPHONE_DET	IMX_GPIO_NR(7, 8)
#define SABRESD_PCIE_RST_B_REVB	IMX_GPIO_NR(7, 12)
#define SABRESD_PMIC_INT_B	IMX_GPIO_NR(7, 13)
#define SABRESD_PFUZE_INT	IMX_GPIO_NR(7, 13)

#define MX6_ENET_IRQ		IMX_GPIO_NR(1, 6)
#define IOMUX_OBSRV_MUX1_OFFSET	0x3c
#define OBSRV_MUX1_MASK			0x3f
#define OBSRV_MUX1_ENET_IRQ		0x9

static int board_version = 2;
//#define UOG_PORTSC1         USBOTG_REG32(0x184)

static void sabresd_suspend_enter(void)
{
	/* suspend preparation */
	/* disable RFID */
	gpio_set_value(UIB_RFID_EN, 0);
}

static void sabresd_suspend_exit(void)
{
	/* resume restore */
	/* enable RFID */
	gpio_set_value(UIB_RFID_EN, 1);
}


#ifdef CONFIG_MX6DL_UIB_REV_2

static int __init board_version_setup(char *version)
{
	char *endp;

	board_version = version ? simple_strtoul(version, &endp, 0) : 1;
	printk(KERN_ERR "board_version: %d\n", board_version);

	return 0;
}
early_param("board_version", board_version_setup);

void request_host_on(void)
{
	int state = 1;

#ifdef UIB_S3_PWR_MODE
	state = get_S3_PWR_MODE();
#endif
	printk(KERN_ERR "request_host_on: %d\n", state);
	if (state) {
#ifdef USE_FIERY_ON_EN
		// wake up the Fiery if it's sleeping (S3_PWR_MODE is high)
		// this should be driven low once S3_PWR_MODE goes low
		gpio_set_value(UIB_FIERY_ON_EN, 1);
#else
		/*u32 tmp = UOG_PORTSC1;
		printk(KERN_ERR "%s: UOG_PORTSC1 = %x\n", __func__, tmp);
		tmp |= PORTSC_PORT_FORCE_RESUME;
		UOG_PORTSC1 = tmp;*/
#endif
	}
}

static struct input_dev *wakeup_input;

void wakeup_android(void)
{
	input_report_key(wakeup_input, KEY_INFO, 1);
	input_report_key(wakeup_input, KEY_INFO, 0);
	input_sync(wakeup_input);
}

# ifdef UIB_S3_PWR_MODE
// very basic driver for S3 signal changes
// make suspend state change requests:
//   sleep system when S3 transitions to 1
//   wake up system when S3 transitions to 0
//   and send INFO key events on wakeup

static struct wake_lock s3_wake_lock;
static struct alarm s3_timer;
static struct alarm s3_suspend;

static int s3irq;

static uint s3_timeout = 5000;
module_param(s3_timeout, uint, S_IRUGO | S_IWUSR);

struct led_classdev *led0_cdev;

static void s3_work(struct work_struct *dummy)
{
	// turn up LCD panel's backlight to drain power supply in S3
	/*if (platform_backlight_device) {
		struct backlight_device *bd = platform_get_drvdata(platform_backlight_device);
		if (bd && bd->ops && bd->ops->update_status) {
			bd->props.brightness = bd->props.max_brightness;
			bd->ops->update_status(bd);
		}
	}*/
}

static DECLARE_WORK(s3_work_struct, s3_work);

static void s3_suspend_callback(struct alarm *alarm)
{
//	extern void request_suspend_state(suspend_state_t state);

	printk("s3_suspend_callback called\n");

	wake_unlock(&s3_wake_lock);
//	request_suspend_state(PM_SUSPEND_MEM);
	pm_suspend(PM_SUSPEND_MEM);
}

static void s3_timer_callback(struct alarm *alarm)
{
	printk("s3_timer_callback called at %llu\n", ktime_to_ns(ktime_get()));
	enable_irq(s3irq);
	// power down LED backlight
	if(led0_cdev)
		led_set_brightness(led0_cdev, LED_OFF);
	gpio_set_value(UIB_LCD_LED_EN, 0);
}

// IRQ handler
static irqreturn_t s3_irq(int irq, void *handle)
{
//	extern void request_suspend_state(suspend_state_t state);
	int state;
	ktime_t alarmtime;

	printk("Ambika: S3_irq called \n");

#ifdef USE_FIERY_ON_EN
	// this is turned on if touch panel wakes up the board
	// so make sure it's off now
	gpio_set_value(UIB_FIERY_ON_EN, 0);
#endif

	state = get_S3_PWR_MODE();

	printk("%s: state = %d at = %llu\n", __func__, state, ktime_to_ns(ktime_get()));

	if (!state) {
		wake_lock(&s3_wake_lock);
//		request_suspend_state(PM_SUSPEND_ON);
		pm_suspend(PM_SUSPEND_ON);
		wakeup_android();
	}
	else {
		/*
		S3 sleep sequence:
			1. Schedule s3_work to ramp up the LED backlight brightness to 255
			   (this gets trimmed to zero when the screen times out).  Must be
			   run in a work queue, as PWM settings require access to the timers,
			   which can't be referenced from inside an IRQ handler.
			2. Disable S3 interrupts.  The S3 signal can fluctuate wildly until
			   the power supply's caps are discharged.
			3. Drive LED0 high.  This GPIO is connected to the S3 logic and will
			   force S3 high until both it and the signal from the fiery go low.
			4. Put the USB hub into reset to conserve power.
			5. Set up a callback to s3_suspend_callback. When called, it releases
			   the S3 wakelock and requests a power state change to memory sleep.
			   By this time, any wakeups generated by previous S3 interrupts with
			   state == 0 (can be several due to the fluctuation mentioned in #2)
			   have been processed and nothing will interfere with the subsequent
			   transition to memory sleep mode.
			6. Set up a callback to s3_timer_callback. When called it drives LED0
			   low and re-enables S3 interrupts.  By this time, the transition to
			   memory sleep mode should be well on it way, so any transition of S3
			   back to 0 (it happens) will be processed in the correct sequence,
			   allowing the screen and USB hub to be awakened properly at this point.
		*/

#ifdef CONFIG_MX6DL_UIB_REV_2
		schedule_work(&s3_work_struct);

		// re-enabled in the timer callback
		disable_irq_nosync(s3irq);

		// this drives S3 high to prevent fluctuations while the power
		// supply voltage is dropping; it's driven low when the timer triggers
		if(led0_cdev)
			led_set_brightness(led0_cdev, LED_FULL);

		// reset hub to put into lower power mode
		gpio_set_value(UIB_USB_HUB_RESET, 0);
		mdelay(5);
		gpio_set_value(UIB_USB_HUB_RESET, 1);
		mdelay(5);
#endif
		alarmtime = ktime_add_ns(ktime_get_real(), (u64) s3_timeout * 1000 * 1000 / 2);
		printk("setting s3_suspend to fire in %ums\n", s3_timeout / 2);
		alarm_start(&s3_suspend, alarmtime);

		alarmtime = ktime_add_ns(ktime_get_real(), (u64) s3_timeout * 1000 * 1000);
		printk("setting s3_timeout to fire in %ums\n", s3_timeout);
		alarm_start(&s3_timer, alarmtime);
	}

	return IRQ_HANDLED;
}
# endif /* UIB_S3_PWR_MODE */

static int leddev_check(struct device *dev, void *name)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);

	if (!cdev)
		return 0;

	printk("led0 classdev name = %s\n", cdev->name);
	return !strcmp(name, cdev->name);
}

// initialize device driver
static int __init s3_irq_init(void)
{
	int err;
	int irq;
	struct device *dev;
	spinlock_t lock;
	
	printk("Ambika: s3_irq_init called\n");

	alarm_init(&s3_suspend, ALARM_REALTIME, s3_suspend_callback);
	alarm_init(&s3_timer, ALARM_REALTIME, s3_timer_callback);

	wakeup_input = input_allocate_device();
	if (!wakeup_input) {
		printk("%s: error allocating input device\n", __func__);
		return ENOMEM;
	}

	wakeup_input->name = "gpio-keys";
	wakeup_input->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_SYN);
	wakeup_input->keybit[BIT_WORD(KEY_INFO)] = BIT_MASK(KEY_INFO);

	err = input_register_device(wakeup_input);
	if (err < 0) {
		printk("%s: error registering irq %d\n", __func__, err);
		input_free_device(wakeup_input);
		return err;
	}

# ifdef UIB_S3_PWR_MODE
	dev = bus_find_device_by_name(&platform_bus_type, NULL, "leds-gpio");

	if(!dev)
	{
		printk(KERN_ERR "failed to find device for cell leds-gpio\n");
	}
	dev = device_find_child(dev, "led0", leddev_check);
	if(dev) {
		led0_cdev = dev_get_drvdata(dev);
		if (led0_cdev)
			printk("led0 cdev name = %s\n", led0_cdev->name);
	}

	irq = gpio_to_irq(UIB_S3_PWR_MODE);
	err = request_irq(irq, s3_irq,
		IRQF_TRIGGER_RISING |
		IRQF_TRIGGER_FALLING |
		IRQF_NO_SUSPEND |
		IRQF_EARLY_RESUME,
		"S3_PWR_MODE", (void *) wakeup_input);
	if (err < 0) {
		printk("%s: error requesting irq %d\n", __func__, irq);
		input_free_device(wakeup_input);
		return err;
	}
	printk("%s: s3_irq set on %d\n", __func__, irq);

	spin_lock_init(&lock);
	spin_lock_irq(&lock);
	wake_lock_init(&s3_wake_lock, WAKE_LOCK_SUSPEND, "S3_PWR_MODE");

	// S3 shouldn't normally be asserted when the board is booting, but
	// this could happen under rare conditions (e.g., glance crashes and
	// the watchdog timer forces a reset while the Fiery is sleeping).
	// In this case, don't make the initial call to the S3 interrupt handler;
	// call request_host_on() instead to wake up the Fiery and continue on
	// with booting.  When the Fiery wakes up, s3_irq() will be called with
	// S3_PWR_MODE deasserted, and power management can progress normally
	// from that point on.
	if (get_S3_PWR_MODE())
		request_host_on();
	else
		s3_irq(irq, (void *) wakeup_input);
	enable_irq_wake(irq);
	spin_unlock_irq(&lock);
# endif /* UIB_S3_PWR_MODE */

	return 0;
}

static void __init s3_irq_exit(void)
{
# ifdef UIB_S3_PWR_MODE
	free_irq(gpio_to_irq(UIB_S3_PWR_MODE), s3_irq);
	input_unregister_device(wakeup_input);
	wake_lock_destroy(&s3_wake_lock);
# endif /* UIB_S3_PWR_MODE */
}

late_initcall(s3_irq_init);
module_exit(s3_irq_exit);
#endif /* CONFIG_MX6DL_UIB_REV_2 */

/*!
 * Board specific initialization.
 */
int __init uib_board_init(void)
{
	int i;
	struct clk *clko, *clko2;
	struct clk *new_parent;
	int rate;
	struct platform_device *voutdev;

	printk("Ambika uib_board_init\n");

	
#ifdef CONFIG_MX6DL_UIB_REV_1
	// drive PMIC standby pad low
	// @@@ note: this is changing to the iMX6 PMIC_STBY_REQ pin in Rev 2
#define PMIC_STBY IMX_GPIO_NR(3, 16)
	gpio_request(PMIC_STBY, "");
	gpio_direction_output(PMIC_STBY, 0);
	// allow sysfs to modify this gpio for testing
	gpio_export(PMIC_STBY, false);

	// @@@ power down Micrel RGMII phy
#define MICREL_STBY IMX_GPIO_NR(6, 29)
	
	gpio_request(MICREL_STBY, "");
	gpio_direction_output(MICREL_STBY, 0);

	gpio_request(UIB_LCD_CNTRL_VGH, "LCD_CNTRL_VGH");
	gpio_direction_output(UIB_LCD_CNTRL_VGH, 1);
#else
	printk("Ambika board_version = %d\n", board_version);
	gpio_request(UIB_LCD_PWR_INH, "LCD_PWR_EN");
	gpio_direction_output(UIB_LCD_PWR_INH, board_version == 1 ? 1 : 0);
	gpio_request(UIB_LCD_STBYB, "LCD_STBYB");
	gpio_direction_output(UIB_LCD_STBYB, 1);
	gpio_request(UIB_LCD_RESET, "LCD_RESET");
	gpio_direction_output(UIB_LCD_RESET, 1);
#endif

	gpio_request(UIB_LCD_LED_EN, "LCD_LED_EN");
	gpio_direction_output(UIB_LCD_LED_EN, 1);


	//imx6q_add_device_gpio_leds();

	//gpio_request(UIB_TOUCH_RESET, "SSD-RESET");
	//gpio_request(UIB_TOUCH_IRQ, "SSD-IRQ");

#ifdef CONFIG_MX6DL_UIB_REV_2
	gpio_request(UIB_USB_HUB_RESET, "HUB-RESET");
	gpio_direction_output(UIB_USB_HUB_RESET, 1);
#endif

	//imx6q_add_pm_imx(0, &mx6q_sabresd_pm_data);

	/*
	   Mfgtools wants emmc as mmcblk0 and other sd card as mmcblk1.
	*/
	
	/*clko2 = clk_get(NULL, "clko2_clk");
	if (IS_ERR(clko2))
		pr_err("can't get CLKO2 clock.\n");

	new_parent = clk_get(NULL, "osc_clk");
	if (!IS_ERR(new_parent)) {
		clk_set_parent(clko2, new_parent);
		clk_put(new_parent);
	}
	rate = clk_round_rate(clko2, 24000000);
	clk_set_rate(clko2, rate);
	clk_enable(clko2);

	/* Camera and audio use osc clock */
	/*clko = clk_get(NULL, "clko_clk");
	if (!IS_ERR(clko))
		clk_set_parent(clko, clko2);*/

	//pm_power_off = mx6_snvs_poweroff;
	//imx6q_add_busfreq();

#ifdef CONFIG_MX6DL_UIB_REV_2
	int ret = gpio_request(UIB_FIERY_ON_EN, "fiery-on-enable");
	if(ret)
                printk("Ambika : UIB_FIERY_ON_EN gpio req failed %d ", ret);
	gpio_direction_output(UIB_FIERY_ON_EN, 0);
	gpio_export(UIB_FIERY_ON_EN, false);

	ret = gpio_request(UIB_SERVER_S5, "server-s5");
	if(ret)
                printk("Ambika : UIB_SERVER_S5 gpio req failed %d ", ret);
	gpio_direction_input(UIB_SERVER_S5);
	gpio_export(UIB_SERVER_S5, false);

# ifdef UIB_S3_PWR_MODE
	int ret1 = gpio_request(UIB_S3_PWR_MODE, "s3-pwr-mode") ;
	if(ret1)
		printk("Ambika : UIB_S3_PWR_MODE gpio req failed %d ", ret1);
	gpio_direction_input(UIB_S3_PWR_MODE);
	gpio_export(UIB_S3_PWR_MODE, false);
# endif /* UIB_S3_PWR_MODE */
#endif

	return 0;
}

arch_initcall_sync(uib_board_init);
