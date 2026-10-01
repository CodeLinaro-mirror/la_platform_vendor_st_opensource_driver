// SPDX-License-Identifier: GPL-2.0-only
/*
 * NFC Controller Driver
 * Copyright (C) 2020 ST Microelectronics S.A.
 * Copyright (C) 2010 Stollmann E+V GmbH
 * Copyright (C) 2010 Trusted Logic S.A.
 */

#define DEBUG
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/version.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include <linux/of_gpio.h>
#ifndef LEGACY
#include <linux/workqueue.h>
#include <linux/acpi.h>
#include <linux/gpio/consumer.h>
#include <net/nfc/nci.h>
#include <linux/clk.h>
#else
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_address.h>
#endif
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/pinctrl/qcom-pinctrl.h>
#include "st21nfc_i3c.h"
#include "st_uapi.h"
#include <linux/version.h>
#include <linux/nvmem-consumer.h>

#include <linux/i3c/device.h>
#include <linux/i3c/master.h>

#define MAX_BUFFER_SIZE 260
#define ST21NFC_POWER_STATE_MAX 3
// wake up for the duration of a typical transaction
#define WAKEUP_SRC_TIMEOUT (500)

#define DRIVER_VERSION "1.0_ST54M"
// stay in IRQ_Out mode #define ST54M_IBI

#define PROP_PWR_MON_RW_ON_NTF nci_opcode_pack(NCI_GID_PROPRIETARY, 5)
#define PROP_PWR_MON_RW_OFF_NTF nci_opcode_pack(NCI_GID_PROPRIETARY, 6)

#define gpiod_set_value_t(x,y) if(!st21nfc_dev->secure_zone)\
									gpiod_set_value(x,y)
#define I3C_ID_NAME "st21nfc_i3c"


static bool enable_debug_log = true;

struct completion i3c_ibi_done;

/* for i3c hack to reset NFCC before DAA */
int saved_gpio_num = 0;
int saved_irq_num = 0;

/*The enum is used to index a pw_states array, the values matter here*/
enum st21nfc_power_state {
	ST21NFC_IDLE = 0,
	ST21NFC_ACTIVE = 1,
	ST21NFC_ACTIVE_RW = 2
};

static const char *const st21nfc_power_state_name[] = {

	"IDLE", "ACTIVE", "ACTIVE_RW"
};

enum st21nfc_read_state { ST21NFC_HEADER, ST21NFC_PAYLOAD };

struct nfc_sub_power_stats {
	uint64_t count;
	uint64_t duration;
	uint64_t last_entry;
	uint64_t last_exit;
};

struct nfc_sub_power_stats_error {
	/* error transition header --> payload state machine */
	uint64_t header_payload;
	/* error transition from an active state when not in idle state */
	uint64_t active_not_idle;
	/* error transition from idle state to idle state */
	uint64_t idle_to_idle;
	/* warning transition from active_rw state to idle state */
	uint64_t active_rw_to_idle;
	/* error transition from active state to active state */
	uint64_t active_to_active;
	/* error transition from idle state to active state with notification */
	uint64_t idle_to_active_ntf;
	/* error transition from active_rw state to active_rw state */
	uint64_t act_rw_to_act_rw;
	/* error transition from idle state to */
	/* active_rw state with notification   */
	uint64_t idle_to_active_rw_ntf;
};

/*
 * The member 'polarity_mode' defines
 * how the wakeup pin is configured and handled.
 * it can take the following values :
 * IRQF_TRIGGER_RISING
 * IRQF_TRIGGER_HIGH
 */

struct st21nfc_ctrl_dev {
	struct device *dev;

	struct gpio_desc *gpiod_reset;
	struct gpio_desc *gpiod_clkreq; // needed ?
	struct gpio_desc *gpiod_irq;
	struct gpio_desc *gpiod_pidle; // needed ?


	struct miscdevice misc_gpio;
	struct mutex lock;
};

struct st21nfc_device {
	wait_queue_head_t read_wq;
	struct mutex read_mutex;
	struct mutex pidle_mutex;
	struct mutex irq_dir_mutex;

    struct i3c_device *i3c_dev;

	struct miscdevice st21nfc_device;
	uint8_t buffer[MAX_BUFFER_SIZE];
	bool irq_enabled;
	bool irq_wake_up;
	struct wakeup_source * irq_wakeup_source;
	bool irq_is_attached;
	bool device_open; /* Is device open? */
	spinlock_t irq_enabled_lock;
	enum st21nfc_power_state pw_current;
	enum st21nfc_read_state r_state_current;
	int irq_pw_stats_idle;
	int p_idle_last;
	struct nfc_sub_power_stats pw_states[ST21NFC_POWER_STATE_MAX];
	struct nfc_sub_power_stats_error pw_states_err;
	struct workqueue_struct *st_p_wq;
	struct work_struct st_p_work;
	/*Power state shadow copies for reading*/
	enum st21nfc_power_state c_pw_current;
	struct nfc_sub_power_stats c_pw_states[ST21NFC_POWER_STATE_MAX];
	struct nfc_sub_power_stats_error c_pw_states_err;

    /* Gpio Control platform driver */
    struct st21nfc_ctrl_dev *ctrl;
	/* CLK control */
	bool clk_run;
	struct clk *s_clk;
	uint8_t pinctrl_en;

	bool pidle_active_low;

	/* irq_gpio polarity to be used */
	unsigned int polarity_mode;
    int    irq;
	/*secure zone state*/
	bool secure_zone;

	/* ibi I3C */
	u8 *ibi_buf;
    size_t ibi_len;
    bool ibi_received;
};

// Definitions
static int st21nfc_master_send(struct st21nfc_device *dev, const char *buf, int count);
static int st21nfc_master_recv(struct st21nfc_device *dev, char *buf, int count);
static void st21nfc_disable_irq(struct st21nfc_device *st21nfc_dev);
static irqreturn_t st21nfc_dev_irq_handler(int irq, void *dev_id);



/*
 *  Routine for I3C master receive
*/
static int st21nfc_master_recv (struct st21nfc_device *dev, char *buf, int count)
{
    struct i3c_priv_xfer xfer;
    int ret = 0;
	if (enable_debug_log)
        pr_info("%s: start: read data, count:%d\n", __func__, count);
    // Set up the transfer structure for a read operation
    xfer.data.in = buf;
    xfer.len = count;
    xfer.rnw = 1; // Set to 1 for read operation

    // Perform the private transfer
    ret = i3c_device_do_priv_xfers(dev->i3c_dev, &xfer, 1);
    if (ret != 0) {
        pr_err("Failed to read data from I3C device: %d\n", ret);
        return ret;
	}
	// read success return the number of bytes read
	if (enable_debug_log)
        pr_info("%s: end: read data, len:%d, ret:0x%x\n", __func__, xfer.len, ret);
	return xfer.len;
}

/*
 *  Routine for I3C master send
*/
static int st21nfc_master_send(struct st21nfc_device *dev, const char *buf, int count)
{
    struct i3c_priv_xfer xfers;
    u8 *scratchbuf;
    int ret = 0;
	if (enable_debug_log)
        pr_info("%s: start: write data, count:%d\n", __func__, count);
    scratchbuf = kmalloc(sizeof(*scratchbuf) * count, GFP_KERNEL | GFP_DMA);
    if (!scratchbuf)
        return -ENOMEM;
    // copy buffer to scratchbuf
    memcpy(scratchbuf, buf, count);

    xfers.data.out = scratchbuf;
    xfers.len = count;
    xfers.rnw = 0; // Write operation

    ret = i3c_device_do_priv_xfers(dev->i3c_dev, &xfers, 1);
    if (ret != 0) {
	    pr_err("Failed to Write data to I3C device try again: %d\n", ret);
        ret = i3c_device_do_priv_xfers(dev->i3c_dev, &xfers, 1);
        if (ret != 0) {
            pr_err("Failed to Write data to I3C device retry failure: %d\n", ret);
            return ret;
		}
	}
	if (enable_debug_log)
        pr_info("%s: end: wrote data, len:%d, ret:0x%x\n", __func__, count, ret);

    kfree(scratchbuf);

    return xfers.len;
}

/*
 * Routine to disable clocks
 */
static int st21nfc_clock_deselect(struct st21nfc_device *st21nfc_dev)
{
	/* if NULL we assume external crystal and dont fail */
	if (IS_ERR_OR_NULL(st21nfc_dev->s_clk))
		return 0;

	if (st21nfc_dev->clk_run == true) {
		clk_disable_unprepare(st21nfc_dev->s_clk);
		st21nfc_dev->clk_run = false;
	}
	return 0;
}

static void st21nfc_disable_irq(struct st21nfc_device *st21nfc_dev)
{
	unsigned long flags;

	spin_lock_irqsave(&st21nfc_dev->irq_enabled_lock, flags);
	if (st21nfc_dev->irq_enabled) {
		disable_irq_nosync(st21nfc_dev->irq);
		st21nfc_dev->irq_enabled = false;
	}
	spin_unlock_irqrestore(&st21nfc_dev->irq_enabled_lock, flags);
}

static void st21nfc_enable_irq(struct st21nfc_device *st21nfc_dev)
{
	unsigned long flags;

	spin_lock_irqsave(&st21nfc_dev->irq_enabled_lock, flags);
	if (!st21nfc_dev->irq_enabled) {
		st21nfc_dev->irq_enabled = true;
		enable_irq(st21nfc_dev->irq);
	}
	spin_unlock_irqrestore(&st21nfc_dev->irq_enabled_lock, flags);
}

static irqreturn_t st21nfc_dev_irq_handler(int irq, void *dev_id)
{
	struct st21nfc_device *st21nfc_dev = dev_id;

	if (st21nfc_dev->irq_wakeup_source != NULL)
		__pm_wakeup_event(st21nfc_dev->irq_wakeup_source, WAKEUP_SRC_TIMEOUT);
	st21nfc_disable_irq(st21nfc_dev);

	/* Wake up waiting readers */
	wake_up(&st21nfc_dev->read_wq);

	return IRQ_HANDLED;
}

static int st21nfc_loc_set_polaritymode(struct st21nfc_device *st21nfc_dev,
					int mode)
{

    struct device *dev = &st21nfc_dev->i3c_dev->dev;

	unsigned int irq_type;
	int ret;

	if (enable_debug_log)
		pr_info("%s:%d mode %d", __FILE__, __LINE__, mode);

	st21nfc_dev->polarity_mode = mode;
	/* setup irq_flags */
	switch (mode) {
	case IRQF_TRIGGER_RISING:
		irq_type = IRQ_TYPE_EDGE_RISING;
		break;
	case IRQF_TRIGGER_HIGH:
		irq_type = IRQ_TYPE_LEVEL_HIGH;
		break;
	default:
		irq_type = IRQ_TYPE_EDGE_RISING;
		break;
	}
	if (st21nfc_dev->irq_is_attached) {
		devm_free_irq(dev, st21nfc_dev->irq, st21nfc_dev);
		st21nfc_dev->irq_is_attached = false;
	}
	ret = irq_set_irq_type(st21nfc_dev->irq, irq_type);
	if (ret) {
		pr_err("%s : set_irq_type failed\n", __func__);
		return -ENODEV;
	}
	/* request irq.  the irq is set whenever the chip has data available
	 * for reading.  it is cleared when all data has been read.
	 */
	if (enable_debug_log)
		pr_debug("%s : requesting IRQ %d\n", __func__, st21nfc_dev->irq);
	st21nfc_dev->irq_enabled = true;

	ret = devm_request_irq(dev, st21nfc_dev->irq, st21nfc_dev_irq_handler,
			       st21nfc_dev->polarity_mode,
			       st21nfc_dev->st21nfc_device.name, st21nfc_dev);
	if (ret) {
		pr_err("%s : devm_request_irq failed\n", __func__);
		return -ENODEV;
	}
	st21nfc_dev->irq_is_attached = true;
	st21nfc_disable_irq(st21nfc_dev);

	if (enable_debug_log)
		pr_info("%s:%d ret %d", __FILE__, __LINE__, ret);
	return ret;
}



static void st21nfc_power_stats_switch(struct st21nfc_device *st21nfc_dev,
				       uint64_t current_time_ms,
				       enum st21nfc_power_state old_state,
				       enum st21nfc_power_state new_state,
				       bool is_ntf)
{
	mutex_lock(&st21nfc_dev->pidle_mutex);

	if (new_state == old_state) {
		if ((st21nfc_dev->pw_states[ST21NFC_IDLE].last_entry != 0) ||
		    (old_state != ST21NFC_IDLE)) {
			pr_err("%s Error: Switched from %s to %s!: %llx, ntf=%d\n",
			       __func__, st21nfc_power_state_name[old_state],
			       st21nfc_power_state_name[new_state],
			       current_time_ms, is_ntf);

			if (new_state == ST21NFC_IDLE)
				st21nfc_dev->pw_states_err.idle_to_idle++;
			else if (new_state == ST21NFC_ACTIVE)
				st21nfc_dev->pw_states_err.active_to_active++;
			else if (new_state == ST21NFC_ACTIVE_RW)
				st21nfc_dev->pw_states_err.act_rw_to_act_rw++;

			mutex_unlock(&st21nfc_dev->pidle_mutex);
			return;
		}
	} else if (!is_ntf && new_state == ST21NFC_ACTIVE &&
		   old_state != ST21NFC_IDLE) {
		st21nfc_dev->pw_states_err.active_not_idle++;
	} else if (!is_ntf && new_state == ST21NFC_IDLE &&
		   old_state == ST21NFC_ACTIVE_RW) {
		st21nfc_dev->pw_states_err.active_rw_to_idle++;
	} else if (is_ntf && new_state == ST21NFC_ACTIVE &&
		   old_state == ST21NFC_IDLE) {
		st21nfc_dev->pw_states_err.idle_to_active_ntf++;
	} else if (is_ntf && new_state == ST21NFC_ACTIVE_RW &&
		   old_state == ST21NFC_IDLE) {
		st21nfc_dev->pw_states_err.idle_to_active_rw_ntf++;
	}

	pr_debug("%s Switching from %s to %s: %llx, ntf=%d\n", __func__,
		 st21nfc_power_state_name[old_state],
		 st21nfc_power_state_name[new_state], current_time_ms, is_ntf);
	st21nfc_dev->pw_states[old_state].last_exit = current_time_ms;
	st21nfc_dev->pw_states[old_state].duration +=
		st21nfc_dev->pw_states[old_state].last_exit -
		st21nfc_dev->pw_states[old_state].last_entry;
	st21nfc_dev->pw_states[new_state].count++;
	st21nfc_dev->pw_current = new_state;
	st21nfc_dev->pw_states[new_state].last_entry = current_time_ms;

	mutex_unlock(&st21nfc_dev->pidle_mutex);
}

static void st21nfc_power_stats_idle_signal(struct st21nfc_device *st21nfc_dev)
{
	uint64_t current_time_ms = ktime_to_ms(ktime_get_boottime());
	int value = gpiod_get_value(st21nfc_dev->ctrl->gpiod_pidle);

	if (st21nfc_dev->pidle_active_low)
		value = !value;

	if (value != 0) {
		st21nfc_power_stats_switch(st21nfc_dev, current_time_ms,
					   st21nfc_dev->pw_current,
					   ST21NFC_ACTIVE, false);
	} else {
		st21nfc_power_stats_switch(st21nfc_dev, current_time_ms,
					   st21nfc_dev->pw_current,
					   ST21NFC_IDLE, false);
	}
}

static void st21nfc_pstate_wq(struct work_struct *work)
{
	struct st21nfc_device *st21nfc_dev =
		container_of(work, struct st21nfc_device, st_p_work);

	st21nfc_power_stats_idle_signal(st21nfc_dev);
}

static irqreturn_t st21nfc_dev_power_stats_handler(int irq, void *dev_id)
{
	struct st21nfc_device *st21nfc_dev = dev_id;

	queue_work(st21nfc_dev->st_p_wq, &(st21nfc_dev->st_p_work));

	return IRQ_HANDLED;
}



static ssize_t st21nfc_dev_read(struct file *filp, char __user *buf,
				size_t count, loff_t *offset)
{
	struct st21nfc_device *st21nfc_dev = container_of(
		filp->private_data, struct st21nfc_device, st21nfc_device);
	int ret;

	if (count == 0)
		return 0;

	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

	if (enable_debug_log)
		pr_debug("%s : reading %zu bytes.\n", __func__, count);

	if (gpiod_get_value(st21nfc_dev->ctrl->gpiod_irq) == 0) {
		pr_info("%s : read called but no IRQ.\n", __func__);
		memset(st21nfc_dev->buffer, 0x7E, count);
		if (copy_to_user(buf, st21nfc_dev->buffer, count)) {
			pr_warn("%s : failed to copy to user space\n",
				__func__);
			return -EFAULT;
		}
		return count;
	}

	mutex_lock(&st21nfc_dev->read_mutex);

	/* Read data */
	ret = st21nfc_master_recv(st21nfc_dev, st21nfc_dev->buffer, count);
	mutex_unlock(&st21nfc_dev->read_mutex);

	if (ret < 0) {
		pr_err("%s: st21nfc_master_recv returned %d\n", __func__, ret);
		return ret;
	}
	if (ret > count) {
		pr_err("%s: received too many bytes from i3c (%d)\n", __func__,
		       ret);
		return -EIO;
	}

	if (copy_to_user(buf, st21nfc_dev->buffer, ret)) {
		pr_warn("%s : failed to copy to user space\n", __func__);
		return -EFAULT;
	}

	return ret;
}

static ssize_t st21nfc_dev_write(struct file *filp, const char __user *buf,
				 size_t count, loff_t *offset)
{
	struct st21nfc_device *st21nfc_dev = container_of(
		filp->private_data, struct st21nfc_device, st21nfc_device);
	char *tmp = NULL;
	int ret = count;

	if (enable_debug_log) {
		pr_debug("%s : writing %zu bytes.\n", __func__, count);
	}

	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

	tmp = memdup_user(buf, count);
	if (IS_ERR_OR_NULL(tmp)) {
		pr_err("%s : memdup_user failed\n", __func__);
		return -EFAULT;
	}

	/* Write data */
	ret = st21nfc_master_send(st21nfc_dev, tmp, count);
	if (ret != count) {
		pr_err("%s : st21nfc_master_send returned %d\n", __func__, ret);
		ret = -EIO;
	}
	kfree(tmp);

	return ret;
}

static int st21nfc_dev_open(struct inode *inode, struct file *filp)
{
	int ret = 0;
	struct st21nfc_device *st21nfc_dev = container_of(
		filp->private_data, struct st21nfc_device, st21nfc_device);

	if (enable_debug_log)
		pr_info("%s: dev_open", __func__);

	if (st21nfc_dev->device_open) {
		ret = -EBUSY;
		pr_err("%s : device already opened ret= %d\n", __func__, ret);
	} else {
		st21nfc_dev->device_open = true;
	}
	return ret;
}

static int st21nfc_release(struct inode *inode, struct file *file)
{

	struct st21nfc_device *st21nfc_dev = container_of(
		file->private_data, struct st21nfc_device, st21nfc_device);

    struct device *dev = &st21nfc_dev->i3c_dev->dev;

	if (st21nfc_dev->irq_is_attached) {
		st21nfc_disable_irq(st21nfc_dev);
		devm_free_irq(dev,
					st21nfc_dev->irq,
					st21nfc_dev);
		st21nfc_dev->irq_is_attached = false;
	}

	st21nfc_dev->device_open = false;
	if (enable_debug_log)
		pr_debug("%s : device_open  = false\n", __func__);

	return 0;
}

static long st21nfc_dev_ioctl(struct file *filp, unsigned int cmd,
			      unsigned long arg)
{

	struct st21nfc_device *st21nfc_dev = container_of(
		filp->private_data, struct st21nfc_device, st21nfc_device);
    struct device *dev = &st21nfc_dev->i3c_dev->dev;

	int ret = 0;
    u32 tmp = 0;

	// Debug trace
    if (enable_debug_log)
	   pr_debug("%s : cmd:%d  arg:%ld \n", __func__, cmd, arg);
	/* Check type and command number */
	if (_IOC_TYPE(cmd) != ST21NFC_MAGIC)
		return -ENOTTY;

	/* Check access direction once here; don't repeat below.
	 * IOC_DIR is from the user perspective, while access_ok is
	 * from the kernel perspective; so they look reversed.
	 */
	if (_IOC_DIR(cmd) & _IOC_READ)
		ret = !ACCESS_OK(VERIFY_WRITE, (void __user *)arg,
				 _IOC_SIZE(cmd));
	if (ret == 0 && _IOC_DIR(cmd) & _IOC_WRITE)
		ret = !ACCESS_OK(VERIFY_READ, (void __user *)arg,
				 _IOC_SIZE(cmd));
	if (ret)
		return -EFAULT;

	switch (cmd) {
	case ST21NFC_SET_POLARITY_RISING:
	case ST21NFC_LEGACY_SET_POLARITY_RISING:
		pr_info(" ### ST21NFC_SET_POLARITY_RISING ###\n");
		st21nfc_loc_set_polaritymode(st21nfc_dev, IRQF_TRIGGER_RISING);
		break;

	case ST21NFC_SET_POLARITY_HIGH:
	case ST21NFC_LEGACY_SET_POLARITY_HIGH:
		pr_info(" ### ST21NFC_SET_POLARITY_HIGH ###\n");
		st21nfc_loc_set_polaritymode(st21nfc_dev, IRQF_TRIGGER_HIGH);
		break;
	case ST21NFC_PULSE_RESET:
	case ST21NFC_LEGACY_PULSE_RESET:
		pr_info("%s Double Pulse Request\n", __func__);
		if (!IS_ERR_OR_NULL(st21nfc_dev->ctrl->gpiod_reset)) {

			/* pulse low for 20 millisecs */
			gpiod_set_value_t(st21nfc_dev->ctrl->gpiod_reset, 0);
			msleep(20);
			gpiod_set_value_t(st21nfc_dev->ctrl->gpiod_reset, 1);
			usleep_range(20000, 21000);
			if (gpiod_get_value(st21nfc_dev->ctrl->gpiod_irq) == 0) {
				/* another pulse low for 20 millisecs */
				gpiod_set_value_t(st21nfc_dev->ctrl->gpiod_reset, 0);
				msleep(20);
				gpiod_set_value_t(st21nfc_dev->ctrl->gpiod_reset, 1);
				pr_info("%s done Double Pulse Request\n", __func__);
			} else {
				pr_info("%s done Double Pulse Request (simple ok)\n", __func__);
			}
		}
		st21nfc_dev->r_state_current = ST21NFC_HEADER;
		break;

	case ST21NFC_GET_WAKEUP:
	case ST21NFC_LEGACY_GET_WAKEUP:
		/* deliver state of Wake_up_pin as return value of ioctl */
		ret = gpiod_get_value(st21nfc_dev->ctrl->gpiod_irq);

		/*
		 * Warning: depending on gpiod_get_value implementation,
		 * it can returns a value different than 1 in case of high level
		 */
		if (ret != 0)
			ret = 1;

		if (enable_debug_log)
			pr_debug("%s get gpio result %d\n", __func__, ret);
		break;
	case ST21NFC_RECOVERY:
	case ST21NFC_LEGACY_RECOVERY:
		/* For ST21NFCD usage only */
		pr_info("%s Recovery Request\n", __func__);
		mutex_lock(&st21nfc_dev->irq_dir_mutex);
		if (!IS_ERR_OR_NULL(st21nfc_dev->ctrl->gpiod_reset)) {
			if (st21nfc_dev->irq_is_attached) {
				devm_free_irq(dev,
					      st21nfc_dev->irq,
					      st21nfc_dev);
				st21nfc_dev->irq_is_attached = false;
			}
			/* pulse low for 20 millisecs */
			gpiod_set_value_t(st21nfc_dev->ctrl->gpiod_reset, 0);
			usleep_range(10000, 11000);
			/* During the reset, force IRQ OUT as */
			/* DH output instead of input in normal usage */
			ret = gpiod_direction_output(st21nfc_dev->ctrl->gpiod_irq, 1);
			if (ret) {
				pr_err("%s : gpiod_direction_output failed\n",
				       __func__);
				ret = -ENODEV;
				mutex_unlock(&st21nfc_dev->irq_dir_mutex);
				break;
			}

			gpiod_set_value(st21nfc_dev->ctrl->gpiod_irq, 1);
			usleep_range(10000, 11000);
			gpiod_set_value_t(st21nfc_dev->ctrl->gpiod_reset, 1);

			pr_info("%s done Pulse Request\n", __func__);
		}

		msleep(25);
		gpiod_set_value(st21nfc_dev->ctrl->gpiod_irq, 0);
		msleep(25);
		gpiod_set_value(st21nfc_dev->ctrl->gpiod_irq, 1);
		msleep(25);
		gpiod_set_value(st21nfc_dev->ctrl->gpiod_irq, 0);
		msleep(25);
		// Add one more sequence for ST54M I3C Revovery
		gpiod_set_value(st21nfc_dev->ctrl->gpiod_irq, 1);
		msleep(25);
		gpiod_set_value(st21nfc_dev->ctrl->gpiod_irq, 0);
		msleep(25);
		// ensure to wait 200ms before reset
		msleep(100);
		pr_info("%s Recovery procedure finished\n", __func__);
		ret = gpiod_direction_input(st21nfc_dev->ctrl->gpiod_irq);
		if (ret) {
			pr_err("%s : gpiod_direction_input failed\n", __func__);
			ret = -ENODEV;
		}

		st21nfc_dev->irq_enabled = true;

		ret = devm_request_irq(dev,
				       st21nfc_dev->irq,
				       st21nfc_dev_irq_handler,
				       st21nfc_dev->polarity_mode,
				       st21nfc_dev->st21nfc_device.name, st21nfc_dev);
		if (ret) {
			pr_err("%s : devm_request_irq failed\n", __func__);
			mutex_unlock(&st21nfc_dev->irq_dir_mutex);
			return -ENODEV;
		}
		st21nfc_dev->irq_is_attached = true;
		st21nfc_disable_irq(st21nfc_dev);

		mutex_unlock(&st21nfc_dev->irq_dir_mutex);

		break;
	case ST21NFC_ON_OFF:
	        ret = __get_user(tmp, (u32 __user *)arg);
		if (ret == 0) {
		}
        break;
	default:
		pr_err("%s bad ioctl %u\n", __func__, cmd);
		ret = -EINVAL;
		break;
	}
	return ret;
}

static unsigned int st21nfc_poll(struct file *file, poll_table *wait)
{

	struct st21nfc_device *st21nfc_dev = container_of(
		file->private_data, struct st21nfc_device, st21nfc_device);
	unsigned int mask = 0;
	int pinlev = 0;

    if (enable_debug_log)
        pr_debug("%s : before poll_wait \n", __func__);

	/* wait for Wake_up_pin == high  */
	poll_wait(file, &st21nfc_dev->read_wq, wait);

    if (enable_debug_log)
        pr_debug("%s : after poll_wait \n", __func__);

	pinlev = gpiod_get_value(st21nfc_dev->ctrl->gpiod_irq);
	mutex_lock(&st21nfc_dev->irq_dir_mutex);
	if (pinlev != 0) {
		if (enable_debug_log)
			pr_debug("%s return ready\n", __func__);

		mask = POLLIN | POLLRDNORM; /* signal data avail */
		st21nfc_disable_irq(st21nfc_dev);
	} else {
		/* Wake_up_pin is low. Activate ISR  */
		if (enable_debug_log)
			pr_debug("%s enable irq\n", __func__);

		st21nfc_enable_irq(st21nfc_dev);
	}

	mutex_unlock(&st21nfc_dev->irq_dir_mutex);
	return mask;
}


static const struct file_operations st21nfc_dev_fops = {
	.owner = THIS_MODULE,
	.read = st21nfc_dev_read,
	.write = st21nfc_dev_write,
	.open = st21nfc_dev_open,
	.poll = st21nfc_poll,
	.release = st21nfc_release,
	.unlocked_ioctl = st21nfc_dev_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = st21nfc_dev_ioctl
#endif
};


static ssize_t st21nfc_addr_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
    // Do Nothing
    return 0;

} /* st21nfc_addr_show() */

static ssize_t st21nfc_addr_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
    // Do Nothing
    return 0;

} /* st21nfc_addr_store() */

static ssize_t version_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%s\n", DRIVER_VERSION);
} /* version_show */

static uint64_t st21nfc_power_duration(struct st21nfc_device *data,
				       enum st21nfc_power_state pstate,
				       uint64_t current_time_ms)
{
	return data->c_pw_current != pstate ?
		       data->c_pw_states[pstate].duration :
		       data->c_pw_states[pstate].duration +
			       (current_time_ms -
				data->c_pw_states[pstate].last_entry);
}

static ssize_t power_stats_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct st21nfc_device *data = dev_get_drvdata(dev);
	uint64_t current_time_ms;
	uint64_t idle_duration;
	uint64_t active_ce_duration;
	uint64_t active_rw_duration;

	mutex_lock(&data->pidle_mutex);

	data->c_pw_current = data->pw_current;
	data->c_pw_states_err = data->pw_states_err;
	memcpy(data->c_pw_states, data->pw_states,
	       ST21NFC_POWER_STATE_MAX * sizeof(struct nfc_sub_power_stats));

	mutex_unlock(&data->pidle_mutex);

	current_time_ms = ktime_to_ms(ktime_get_boottime());
	idle_duration =
		st21nfc_power_duration(data, ST21NFC_IDLE, current_time_ms);
	active_ce_duration =
		st21nfc_power_duration(data, ST21NFC_ACTIVE, current_time_ms);
	active_rw_duration = st21nfc_power_duration(data, ST21NFC_ACTIVE_RW,
						    current_time_ms);

	return scnprintf(
		buf, PAGE_SIZE,
		"NFC subsystem\n"
		"Idle mode:\n"
		"\tCumulative count: 0x%llx\n"
		"\tCumulative duration msec: 0x%llx\n"
		"\tLast entry timestamp msec: 0x%llx\n"
		"\tLast exit timestamp msec: 0x%llx\n"
		"Active mode:\n"
		"\tCumulative count: 0x%llx\n"
		"\tCumulative duration msec: 0x%llx\n"
		"\tLast entry timestamp msec: 0x%llx\n"
		"\tLast exit timestamp msec: 0x%llx\n"
		"Active Reader/Writer mode:\n"
		"\tCumulative count: 0x%llx\n"
		"\tCumulative duration msec: 0x%llx\n"
		"\tLast entry timestamp msec: 0x%llx\n"
		"\tLast exit timestamp msec: 0x%llx\n"
		"\nError transition header --> payload state machine: 0x%llx\n"
		"Error transition from an Active state when not in Idle state: 0x%llx\n"
		"Error transition from Idle state to Idle state: 0x%llx\n"
		"Warning transition from Active Reader/Writer state to Idle state: 0x%llx\n"
		"Error transition from Active state to Active state: 0x%llx\n"
		"Error transition from Idle state to Active state with notification: 0x%llx\n"
		"Error transition from Active Reader/Writer state to Active Reader/Writer state: 0x%llx\n"
		"Error transition from Idle state to Active Reader/Writer state with notification: 0x%llx\n"
		"\nTotal uptime: 0x%llx Cumulative modes time: 0x%llx\n",
		data->c_pw_states[ST21NFC_IDLE].count, idle_duration,
		data->c_pw_states[ST21NFC_IDLE].last_entry,
		data->c_pw_states[ST21NFC_IDLE].last_exit,
		data->c_pw_states[ST21NFC_ACTIVE].count, active_ce_duration,
		data->c_pw_states[ST21NFC_ACTIVE].last_entry,
		data->c_pw_states[ST21NFC_ACTIVE].last_exit,
		data->c_pw_states[ST21NFC_ACTIVE_RW].count, active_rw_duration,
		data->c_pw_states[ST21NFC_ACTIVE_RW].last_entry,
		data->c_pw_states[ST21NFC_ACTIVE_RW].last_exit,
		data->c_pw_states_err.header_payload,
		data->c_pw_states_err.active_not_idle,
		data->c_pw_states_err.idle_to_idle,
		data->c_pw_states_err.active_rw_to_idle,
		data->c_pw_states_err.active_to_active,
		data->c_pw_states_err.idle_to_active_ntf,
		data->c_pw_states_err.act_rw_to_act_rw,
		data->c_pw_states_err.idle_to_active_rw_ntf, current_time_ms,
		idle_duration + active_ce_duration + active_rw_duration);
}

static DEVICE_ATTR_RW(st21nfc_addr);

static DEVICE_ATTR_RO(version);

static DEVICE_ATTR_RO(power_stats);

static struct attribute *st21nfc_attrs[] = {
	&dev_attr_st21nfc_addr.attr,
	&dev_attr_version.attr,
	&dev_attr_power_stats.attr,
	NULL,
};

static struct attribute_group st21nfc_attr_grp = {
	.attrs = st21nfc_attrs,
};

/* -------------------------------------------------------------------------- */
/* DT phandle lookup: I3C side finds control platform device                   */
/* -------------------------------------------------------------------------- */

static struct st21nfc_ctrl_dev *st21nfc_ctrl_from_phandle(struct device *dev)
{
	struct device_node *np;
	struct platform_device *pdev;
	struct st21nfc_ctrl_dev *ctrl;

	np = of_parse_phandle(dev->of_node, "st,nfc-gpio", 0);
	if (!np) {
		dev_err(dev, "missing nfc-ctrl phandle\n");
		return ERR_PTR(-EINVAL);
	}

	pdev = of_find_device_by_node(np);
	of_node_put(np);

	if (!pdev) {
		dev_dbg(dev, "nfc-ctrl device not ready, defer probe\n");
		return ERR_PTR(-EPROBE_DEFER);
	}

	ctrl = platform_get_drvdata(pdev);
	if (!ctrl) {
		put_device(&pdev->dev);
		dev_dbg(dev, "nfc-ctrl drvdata not ready, defer probe\n");
		return ERR_PTR(-EPROBE_DEFER);
	}

	put_device(&pdev->dev);
	return ctrl;
}

/* -------------------------------------------------------------------------- */
/* I3C driver                                                                 */
/* -------------------------------------------------------------------------- */

static int st21nfc_probe(struct i3c_device *client)
{
	int ret;
	struct st21nfc_device *st21nfc_dev;
	struct device *dev = &client->dev;
	struct st21nfc_ctrl_dev *ctrl;
	unsigned int clkreq_gpio = 0;
	pr_info("%s: enter\n",__func__);

	st21nfc_dev = devm_kzalloc(dev, sizeof(*st21nfc_dev), GFP_KERNEL);
	if (st21nfc_dev == NULL)
		return -ENOMEM;

	/* store for later use */
	st21nfc_dev->i3c_dev = client;
	st21nfc_dev->r_state_current = ST21NFC_HEADER;


    /* Get GPIOs from st21nfc_gpio */
	ctrl = st21nfc_ctrl_from_phandle(dev);
	if (IS_ERR(ctrl)) {
		pr_err("%s : st21nfc_ctrl_from_phandle failed\n", __func__);
		return PTR_ERR(ctrl);
	}
	st21nfc_dev->ctrl = ctrl;

    st21nfc_dev->secure_zone = false;
    pr_info("%s:st21nfc_dev->secure_zone = %s", __func__, st21nfc_dev->secure_zone ? "true" : "false");

    // GPIOs management
	// gpiod_pidle management
	if (IS_ERR_OR_NULL(st21nfc_dev->ctrl->gpiod_pidle)) {
		pr_warn("[OPTIONAL] %s: Unable to request pidle-gpio\n",
			__func__);
		ret = 0;
	} else {
		if (!device_property_read_bool(dev, "st,pidle_active_low")) {
			pr_info("%s:[OPTIONAL] pidle_active_low not set\n", __func__);
			st21nfc_dev->pidle_active_low = false;
		} else {
			pr_info("%s:[OPTIONAL] pidle_active_low set\n", __func__);
			st21nfc_dev->pidle_active_low = true;
		}
		/* Prepare a workqueue for st21nfc_dev_power_stats_handler */
		st21nfc_dev->st_p_wq = create_workqueue("st_pstate_work");
		if(!st21nfc_dev->st_p_wq) {
            /* gpio to be released by platform driver */
			return -ENODEV;
		}
		mutex_init(&st21nfc_dev->pidle_mutex);
		INIT_WORK(&(st21nfc_dev->st_p_work), st21nfc_pstate_wq);
		/* Start the power stat in power mode idle */
		st21nfc_dev->irq_pw_stats_idle =
			gpiod_to_irq(st21nfc_dev->ctrl->gpiod_pidle);

		ret = irq_set_irq_type(st21nfc_dev->irq_pw_stats_idle,
				       IRQ_TYPE_EDGE_BOTH);
		if (ret) {
			pr_err("%s : set_irq_type failed\n", __func__);
			goto err_pidle_workqueue;
		}

		/* This next call requests an interrupt line */
		ret = devm_request_irq(
			dev, st21nfc_dev->irq_pw_stats_idle,
			(irq_handler_t)st21nfc_dev_power_stats_handler,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			/* Interrupt on both edges */
			"st21nfc_pw_stats_idle_handle", st21nfc_dev);
		if (ret) {
			pr_err("%s : devm_request_irq for power stats idle failed\n",
			       __func__);
			goto err_pidle_workqueue;
		}
	}

	// gpiod_clkreq management
	if (IS_ERR_OR_NULL(st21nfc_dev->ctrl->gpiod_clkreq)) {
		pr_warn("[OPTIONAL] %s : Unable to request clkreq-gpios\n",
			__func__);
		ret = 0;
		st21nfc_dev->clk_run = false;
	} else {
		/* clkreq GPIO number from device tree*/
		ret = of_property_read_u32_index(st21nfc_dev->ctrl->dev->of_node,
				 "clkreq-gpios", 1, &clkreq_gpio);
		if (ret < 0) {
			pr_err("%s Failed to read clkreq gpio number (clkreq-gpios), ret: %d\n",
				__func__, ret);
			goto err_pidle_workqueue;
		}
		/* configure clkreq GPIO as wakeup capable */
		ret = msm_gpio_mpm_wake_set(clkreq_gpio, true);
		if (ret < 0) {
			pr_err("%s Failed to setup clkreq gpio %d as wakeup capable, ret: %d\n", __func__, clkreq_gpio , ret);
			goto err_pidle_workqueue;
		} else
			pr_info("%s clkreq gpio %d successfully setup for wakeup capable\n", __func__, clkreq_gpio);
        // configure clkreq gpio if needed
		if (!device_property_read_bool(dev, "st,clk_pinctrl")) {
			pr_debug("%s:[OPTIONAL] clk_pinctrl not set\n",
				 __func__);
			st21nfc_dev->pinctrl_en = 0;
		} else {
			pr_debug("%s:[OPTIONAL] clk_pinctrl set\n",
				 __func__);
			st21nfc_dev->pinctrl_en = 1;
		}

		/* Set clk_run when clock pinctrl already enabled */
		if (st21nfc_dev->pinctrl_en != 0)
			st21nfc_dev->clk_run = true;

	}

    st21nfc_dev->irq = gpiod_to_irq(st21nfc_dev->ctrl->gpiod_irq);

	/* init mutex and queues */
	init_waitqueue_head(&st21nfc_dev->read_wq);
	mutex_init(&st21nfc_dev->read_mutex);
	mutex_init(&st21nfc_dev->irq_dir_mutex);
	spin_lock_init(&st21nfc_dev->irq_enabled_lock);
	pr_debug(
		"%s : debug irq_gpio = %d, client-irq =  %d, pidle_gpio = %d\n",
		__func__,
		IS_ERR_OR_NULL(st21nfc_dev->ctrl->gpiod_irq) ?
			-1 :
			desc_to_gpio(st21nfc_dev->ctrl->gpiod_irq),
		st21nfc_dev->irq,
		IS_ERR_OR_NULL(st21nfc_dev->ctrl->gpiod_pidle) ?
			-1 :
			desc_to_gpio(st21nfc_dev->ctrl->gpiod_pidle));

	st21nfc_dev->st21nfc_device.minor = MISC_DYNAMIC_MINOR;
	st21nfc_dev->st21nfc_device.name = "st21nfc_i3c";
	st21nfc_dev->st21nfc_device.fops = &st21nfc_dev_fops;
	st21nfc_dev->st21nfc_device.parent = dev;

    i3cdev_set_drvdata(client, st21nfc_dev);

	ret = misc_register(&st21nfc_dev->st21nfc_device);
	if (ret) {
		pr_err("%s : misc_register failed\n", __func__);
		goto err_misc_register;
	}

	ret = sysfs_create_group(&dev->kobj, &st21nfc_attr_grp);
	if (ret) {
		pr_err("%s : sysfs_create_group failed\n", __func__);
		goto err_sysfs_create_group_failed;
	}
	st21nfc_dev->irq_wakeup_source = wakeup_source_register(NULL, "st21nfc");
	st21nfc_dev->irq_wake_up = false;

#ifdef ST54M_IBI
    st21_enable_ibi(st21nfc_dev->i3c_dev);
#endif

	pr_info("%s: probing nfc i3c success\n",__func__);

	return 0;

err_sysfs_create_group_failed:
	misc_deregister(&st21nfc_dev->st21nfc_device);
err_misc_register:
	mutex_destroy(&st21nfc_dev->read_mutex);
	mutex_destroy(&st21nfc_dev->irq_dir_mutex);
	if (!IS_ERR_OR_NULL(st21nfc_dev->ctrl->gpiod_pidle)) {
		sysfs_remove_file(&client->dev.kobj,
				  &dev_attr_power_stats.attr);
	}
err_pidle_workqueue:
	if (!IS_ERR_OR_NULL(st21nfc_dev->ctrl->gpiod_pidle)) {
		mutex_destroy(&st21nfc_dev->pidle_mutex);
		destroy_workqueue(st21nfc_dev->st_p_wq);
		//devm_gpiod_put(dev,st21nfc_dev->gpiod_pidle);
	}
	if (!IS_ERR_OR_NULL(st21nfc_dev->ctrl->gpiod_reset)) {
		//devm_gpiod_put(dev,st21nfc_dev->ctrl->gpiod_reset);
	}
	if (!IS_ERR_OR_NULL(st21nfc_dev->ctrl->gpiod_irq)) {
		//devm_gpiod_put(dev,st21nfc_dev->gpiod_irq);
	}
	if (!IS_ERR_OR_NULL(st21nfc_dev->ctrl->gpiod_clkreq)) {
		//devm_gpiod_put(dev,st21nfc_dev->gpiod_clkreq);
	}
	return ret;
}


static void st21nfc_remove(struct i3c_device *client)
{

    struct st21nfc_device *st21nfc_dev = i3cdev_get_drvdata(client);

	st21nfc_clock_deselect(st21nfc_dev);
	misc_deregister(&st21nfc_dev->st21nfc_device);
	if (!IS_ERR_OR_NULL(st21nfc_dev->ctrl->gpiod_pidle)) {
		sysfs_remove_file(&client->dev.kobj,
				  &dev_attr_power_stats.attr);
		mutex_destroy(&st21nfc_dev->pidle_mutex);
		devm_gpiod_put(&client->dev,st21nfc_dev->ctrl->gpiod_pidle);
	}
	sysfs_remove_group(&client->dev.kobj, &st21nfc_attr_grp);
	if (st21nfc_dev->irq_wakeup_source) {
		wakeup_source_unregister(st21nfc_dev->irq_wakeup_source);
		st21nfc_dev->irq_wakeup_source = NULL;
	}
	mutex_destroy(&st21nfc_dev->read_mutex);
	mutex_destroy(&st21nfc_dev->irq_dir_mutex);
	acpi_dev_remove_driver_gpios(ACPI_COMPANION(&client->dev));
	if (!IS_ERR_OR_NULL(st21nfc_dev->ctrl->gpiod_reset)) {
		devm_gpiod_put(&client->dev,st21nfc_dev->ctrl->gpiod_reset);
	}
	if (!IS_ERR_OR_NULL(st21nfc_dev->ctrl->gpiod_irq)) {
		devm_gpiod_put(&client->dev,st21nfc_dev->ctrl->gpiod_irq);
	}
	if (!IS_ERR_OR_NULL(st21nfc_dev->ctrl->gpiod_clkreq)) {
		devm_gpiod_put(&client->dev,st21nfc_dev->ctrl->gpiod_clkreq);
	}

}

static int st21nfc_suspend(struct device *device)
{
	// Do Nothing
	return 0;
}

static int st21nfc_resume(struct device *device)
{
	// Do Nothing
	return 0;
}

static const struct i3c_device_id st21nfc_id[] = {
	/* ST54L NFC i3c slave: MID:0x208 >> 1 = 0x104 - ST54 PID must be 0x0208 11005000 */
	//I3C_DEVICE(0x104, 0x1100, (void *)5000 ),
	/* ST54M NFC i3c slave: MID:0x208 >> 1 = 0x104 - ST54 PID must be 0x0208 11008000 */
	I3C_DEVICE(0x104, 0x1100, (void *)8000 ),
	{ /* sentinel */ },
};

static const struct of_device_id st21nfc_of_match[] = {
	{
		.compatible = "st,st21nfc_i3c",
	},
	{ /* same name declaration in device tree */ }
};

MODULE_DEVICE_TABLE(i3c, st21nfc_id);

static const struct dev_pm_ops st21nfc_pm_ops = { SET_SYSTEM_SLEEP_PM_OPS(
	st21nfc_suspend, st21nfc_resume) };

static struct i3c_driver st21nfc_i3c_driver = {
	.id_table = st21nfc_id,
	.probe = st21nfc_probe,
	.remove = st21nfc_remove,
	.driver =
		{
			.owner = THIS_MODULE,
			.name = I3C_ID_NAME,
			.of_match_table = st21nfc_of_match,
			.probe_type = PROBE_PREFER_ASYNCHRONOUS,
			.pm = &st21nfc_pm_ops,
		},
};

/* -------------------------------------------------------------------------- */
/* Platform char device: /dev/st21nfc_gpio                                    */
/* -------------------------------------------------------------------------- */

static int st21nfc_gpio_open(struct inode *inode, struct file *file)
{
	struct miscdevice *mdev = file->private_data;
	struct st21nfc_ctrl_dev *ctrl = container_of(mdev, struct st21nfc_ctrl_dev, misc_gpio);

	file->private_data = ctrl;
	return 0;
}

static long st21nfc_gpio_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{

	struct st21nfc_ctrl_dev *ctrl = file->private_data;
	int ret;

	switch (cmd) {
	case ST21NFC_RECOVERY:
	case ST21NFC_LEGACY_RECOVERY:
		pr_info("%s Recovery Request from platform driver\n", __func__);
		/* pulse low for 20 millisecs */
		gpiod_set_value(ctrl->gpiod_reset, 0);
		usleep_range(10000, 11000);
		/* During the reset, force IRQ OUT as DH output instead of input in normal usage */
		ret = gpiod_direction_output(ctrl->gpiod_irq, 1);
		if (ret) {
			pr_err("%s : gpiod_direction_output failed\n",
					__func__);
			ret = -ENODEV;
			break;
		}
		gpiod_set_value(ctrl->gpiod_irq, 1);
		usleep_range(10000, 11000);
		gpiod_set_value(ctrl->gpiod_reset, 1);
        pr_info("%s done Pulse Request\n", __func__);
		msleep(25);
		gpiod_set_value(ctrl->gpiod_irq, 0);
		msleep(25);
		gpiod_set_value(ctrl->gpiod_irq, 1);
		msleep(25);
		gpiod_set_value(ctrl->gpiod_irq, 0);
		msleep(25);
		// Add one more sequence for ST54M I3C Revovery
		gpiod_set_value(ctrl->gpiod_irq, 1);
		msleep(25);
		gpiod_set_value(ctrl->gpiod_irq, 0);
		msleep(25);
		// ensure to wait 200ms before reset
		msleep(100);
		pr_info("%s Recovery procedure finished\n", __func__);
		ret = gpiod_direction_input(ctrl->gpiod_irq);
		if (ret) {
			pr_err("%s : gpiod_direction_input failed\n", __func__);
			ret = -ENODEV;
		}
        // Reset NFCC
		gpiod_set_value(ctrl->gpiod_reset, 0);
		usleep_range(20000, 21000);
		gpiod_set_value(ctrl->gpiod_reset, 1);
		return 0;
    break;
	default:
	    pr_err("%s bad ioctl %u\n", __func__, cmd);
		return -ENOTTY;
	}
	return 0;
}

static const struct file_operations st21nfc_gpio_fops = {
	.owner          = THIS_MODULE,
	.open           = st21nfc_gpio_open,
	.unlocked_ioctl = st21nfc_gpio_ioctl,
	.llseek         = noop_llseek,
};

/* -------------------------------------------------------------------------- */
/* Platform driver                                                             */
/* -------------------------------------------------------------------------- */

static int st21nfc_ctrl_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct st21nfc_ctrl_dev *ctrl;
	int ret;
	pr_info("%s: enter\n",__func__);

	ctrl = devm_kzalloc(dev, sizeof(*ctrl), GFP_KERNEL);
	if (!ctrl)
		return -ENOMEM;

	ctrl->dev = dev;
	mutex_init(&ctrl->lock);

    // Reset GPIO - NFC_CHIP_EN
	ctrl->gpiod_reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctrl->gpiod_reset))
	{
	    pr_err("st21nfc_ctrl_probe error for gpiod_reset \n");
		return dev_err_probe(dev, PTR_ERR(ctrl->gpiod_reset),
				     "failed to get reset gpio\n");
	}
	/*
	 * IRQ line is owned by control driver as an input GPIO descriptor.
	 * The I3C driver will later map it to an IRQ number via gpiod_to_irq().
	 */
	// Irq GPIO - NFC_IRQ_OUT
	ctrl->gpiod_irq = devm_gpiod_get(dev, "irq", GPIOD_IN);
	if (IS_ERR(ctrl->gpiod_irq))
	{
	    pr_err("st21nfc_ctrl_probe error for gpiod_irq \n");
		return dev_err_probe(dev, PTR_ERR(ctrl->gpiod_irq),
				     "failed to get irq gpio\n");
	}

    // Pidle GPIO - PWR state ?
    ctrl->gpiod_pidle = devm_gpiod_get_optional(dev, "pidle", GPIOD_IN);
	if (IS_ERR(ctrl->gpiod_pidle)) {
		dev_warn(dev, "failed to get pidle gpio: %ld, continuing without it\n",
			PTR_ERR(ctrl->gpiod_pidle));
		ctrl->gpiod_pidle = NULL;
	}
    // ClkReq GPIO - NFC_CLK_REQ
    ctrl->gpiod_clkreq = devm_gpiod_get_optional(dev, "clkreq", GPIOD_IN);
	if (IS_ERR(ctrl->gpiod_clkreq)) {
		dev_warn(dev, "failed to get clkreq gpio: %ld, continuing without it\n",
			PTR_ERR(ctrl->gpiod_clkreq));
		ctrl->gpiod_clkreq = NULL;
	}

	platform_set_drvdata(pdev, ctrl);

	ctrl->misc_gpio.minor  = MISC_DYNAMIC_MINOR;
	ctrl->misc_gpio.name   = "st21nfc_gpio";
	ctrl->misc_gpio.fops   = &st21nfc_gpio_fops;
	ctrl->misc_gpio.parent = dev;

	ret = misc_register(&ctrl->misc_gpio);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register /dev/nfc_gpio\n");

	pr_info("%s: NFCC Done - /dev/st21nfc_gpio ready\n",__func__);

    /* Reset ST54M during GPIO probe to trig I3C Hot Join*/
    // Drive the GPIO (set it high low high)
    gpiod_set_value(ctrl->gpiod_reset, 1);
    usleep_range(10000, 11000);
    gpiod_set_value(ctrl->gpiod_reset, 0);
    usleep_range(10000, 11000);
    gpiod_set_value(ctrl->gpiod_reset, 1);
	/*
    usleep_range(10000, 11000);
	gpiod_set_value(ctrl->gpiod_reset, 0);
    usleep_range(10000, 11000);
    gpiod_set_value(ctrl->gpiod_reset, 1);
    usleep_range(10000, 11000); */
	pr_info("%s: NFCC reset Done\n",__func__);
    usleep_range(10000, 11000);

	return 0;
}

static void st21nfc_ctrl_remove(struct platform_device *pdev)
{
	struct st21nfc_ctrl_dev *ctrl = platform_get_drvdata(pdev);

	misc_deregister(&ctrl->misc_gpio);
	dev_info(&pdev->dev, "nfc ctrl removed\n");

}

static const struct of_device_id st21nfc_ctrl_of_match[] = {
	{ .compatible = "st,st21nfc_gpio" },
	{ }
};

MODULE_DEVICE_TABLE(of, st21nfc_ctrl_of_match);

static struct platform_driver st21nfc_ctrl_driver = {
	.probe  = st21nfc_ctrl_probe,
	.remove = st21nfc_ctrl_remove,
	.driver = {
		.name = "st21nfc_gpio",
		.of_match_table = st21nfc_ctrl_of_match,
	},
};

/* -------------------------------------------------------------------------- */
/* Module init/exit                                                            */
/* -------------------------------------------------------------------------- */

static int __init st21nfc_dual_init(void)
{
	int ret;

	if (enable_debug_log)
	    pr_info("%s :Loading st21nfc driver %s\n", __func__, DRIVER_VERSION);

	ret = platform_driver_register(&st21nfc_ctrl_driver);
	if (ret)
		return ret;

	ret = i3c_driver_register_with_owner(&st21nfc_i3c_driver, THIS_MODULE);
	if (ret) {
		platform_driver_unregister(&st21nfc_ctrl_driver);
		return ret;
	}

    if (enable_debug_log)
	    pr_info("%s :nfc dual-driver module loaded\n", __func__);

	return 0;
}

static void __exit st21nfc_dual_exit(void)
{
	i3c_driver_unregister(&st21nfc_i3c_driver);
	platform_driver_unregister(&st21nfc_ctrl_driver);
	pr_info("nfc dual-driver module unloaded\n");
}

module_init(st21nfc_dual_init);
module_exit(st21nfc_dual_exit);

MODULE_AUTHOR("STMicroelectronics");
MODULE_DESCRIPTION("NFC ST21NFC dual driver gpio i3c");
MODULE_VERSION(DRIVER_VERSION);
MODULE_LICENSE("GPL");
