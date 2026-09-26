// SPDX-License-Identifier: GPL-2.0-only
/*
 * RGB LED bar of the Crestron TSX-xx60 touch panels.
 *
 * The bar is driven by an STM32 microcontroller that is connected to the SoC
 * over USB (VID 0x14be, PID 0x001b: interface 0 = debug console, interface 1 =
 * "Cresnet IO"). The host sends Cresnet packets as single transfers on the
 * OUT endpoint of interface 1 and the STM32 answers with packets on the IN
 * endpoint of the same interface. A Cresnet packet is
 *
 *	dest(1) len(1) payload(len)	len = number of bytes after the len byte
 *
 * The LED bar channels are Cresnet analog joins (payload type 0x14):
 *
 *	00 05 14 JH JL VH VL		join J (big endian), value V (big endian)
 *
 * with join 3 = red, 4 = green, 5 = blue and values 0..100. Digital joins
 * (payload type 0x00) 0/1/2 switch red/green/blue on (00) or off (80):
 *
 *	00 03 00 JL (JH | 0x80 * off)
 *
 * The bar firmware lights a color only while its digital join is on: the
 * analog join sets the level, the digital join is the "control" switch. Both
 * are off after the STM32 starts. This driver registers one multicolor LED
 * ("tsx:rgb:bar", max_brightness 100). On every change it sends, per color,
 * the analog join and then the digital join (on for a level above 0, off for
 * 0), one join per packet as the vendor userland does. The STM32 sends an
 * update request (02 02 03 00) after it starts. The driver answers it with
 * the current state. The raw attribute sends any Cresnet packet (same checks
 * as the vendor stm32_io attribute), rx_last shows the last packet received
 * from the STM32 (hex), for bring-up.
 */

#include <linux/kernel.h>
#include <linux/led-class-multicolor.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/usb.h>
#include <linux/workqueue.h>

#define CSTM_VID		0x14be
#define CSTM_IO_IFNUM		1
#define CSTM_TX_TIMEOUT_MS	1000
#define CSTM_MAX_PACKET		257	/* dest + len + 255 */
#define CSTM_RX_LAST_MAX	64
#define CSTM_MAX_LEVEL		100

#define CSTM_JOIN_RED		3	/* analog join of the red channel */
#define CSTM_DJOIN_RED		0	/* digital join of the red channel */
#define CSTM_NUM_COLORS		3

struct cstm_led {
	struct usb_device *udev;
	struct usb_interface *intf;
	struct led_classdev_mc mc_cdev;
	struct mc_subled subled[CSTM_NUM_COLORS];
	char name[32];
	struct mutex state_lock;	/* serializes cstm_apply(), protects ctl */
	s8 ctl[CSTM_NUM_COLORS];	/* last digital join sent, -1 = unknown */
	struct work_struct resync_work;

	struct mutex tx_lock;		/* serializes OUT transfers, protects gone */
	bool gone;
	unsigned int out_pipe;
	u8 *tx_buf;

	struct urb *rx_urb;
	u8 *rx_buf;
	unsigned int rx_size;
	spinlock_t rx_lock;		/* protects rx_last, rx_last_len, rx_count */
	u8 rx_last[CSTM_RX_LAST_MAX];
	unsigned int rx_last_len;
	unsigned long rx_count;

	char fw[64];
};

static int cstm_send(struct cstm_led *led, const u8 *pkt, size_t len)
{
	int actual = 0, ret;

	if (len < 3 || len > CSTM_MAX_PACKET || pkt[1] != len - 2)
		return -EINVAL;

	mutex_lock(&led->tx_lock);
	if (led->gone) {
		ret = -ENODEV;
		goto out;
	}
	memcpy(led->tx_buf, pkt, len);
	/* usb_bulk_msg() also handles an interrupt endpoint */
	ret = usb_bulk_msg(led->udev, led->out_pipe, led->tx_buf, len, &actual,
			   CSTM_TX_TIMEOUT_MS);
	if (!ret && actual != len)
		ret = -EIO;
out:
	mutex_unlock(&led->tx_lock);
	if (ret && ret != -ENODEV)
		dev_warn_ratelimited(&led->intf->dev, "send %*ph: %d\n",
				     (int)len, pkt, ret);
	return ret;
}

static int cstm_send_analog(struct cstm_led *led, unsigned int join,
			    unsigned int value)
{
	u8 pkt[7] = { 0x00, 0x05, 0x14, join >> 8, join & 0xff,
		      value >> 8, value & 0xff };

	return cstm_send(led, pkt, sizeof(pkt));
}

/* digital join J: 00 03 00 JL JH, bit 7 of JH set = off */
static int cstm_send_digital(struct cstm_led *led, unsigned int join, bool on)
{
	u8 pkt[5] = { 0x00, 0x03, 0x00, join & 0xff,
		      ((join >> 8) & 0x7f) | (on ? 0 : 0x80) };

	return cstm_send(led, pkt, sizeof(pkt));
}

static int cstm_apply(struct cstm_led *led, enum led_brightness brightness,
		      bool force)
{
	struct led_classdev_mc *mc = &led->mc_cdev;
	unsigned int level;
	int i, ret = 0;
	bool on;

	mutex_lock(&led->state_lock);
	led_mc_calc_color_components(mc, brightness);
	for (i = 0; i < mc->num_colors; i++) {
		level = mc->subled_info[i].brightness;
		on = level > 0;
		ret = cstm_send_analog(led, mc->subled_info[i].channel, level);
		if (ret)
			break;
		if (!force && led->ctl[i] == on)
			continue;
		ret = cstm_send_digital(led, CSTM_DJOIN_RED + i, on);
		if (ret) {
			led->ctl[i] = -1;
			break;
		}
		led->ctl[i] = on;
	}
	mutex_unlock(&led->state_lock);
	return ret;
}

static int cstm_brightness_set(struct led_classdev *cdev,
			       enum led_brightness brightness)
{
	struct led_classdev_mc *mc = lcdev_to_mccdev(cdev);

	return cstm_apply(container_of(mc, struct cstm_led, mc_cdev),
			  brightness, false);
}

/* the STM32 asked for all join states: send the current color again */
static void cstm_resync_work(struct work_struct *work)
{
	struct cstm_led *led = container_of(work, struct cstm_led, resync_work);

	cstm_apply(led, READ_ONCE(led->mc_cdev.led_cdev.brightness), true);
}

static struct cstm_led *cstm_from_dev(struct device *dev)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);

	return container_of(lcdev_to_mccdev(cdev), struct cstm_led, mc_cdev);
}

/* raw: write one binary Cresnet packet, e.g. printf '\x00\x05\x14\x00\x03\x00\x64' */
static ssize_t raw_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	int ret = cstm_send(cstm_from_dev(dev), (const u8 *)buf, count);

	return ret ? ret : count;
}
static DEVICE_ATTR_WO(raw);

static ssize_t rx_last_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct cstm_led *led = cstm_from_dev(dev);
	u8 last[CSTM_RX_LAST_MAX];
	unsigned long count;
	unsigned int len;

	spin_lock_irq(&led->rx_lock);
	len = led->rx_last_len;
	count = led->rx_count;
	memcpy(last, led->rx_last, len);
	spin_unlock_irq(&led->rx_lock);

	return sysfs_emit(buf, "%lu %*phN\n", count, len, last);
}
static DEVICE_ATTR_RO(rx_last);

static ssize_t firmware_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	return sysfs_emit(buf, "%s\n", cstm_from_dev(dev)->fw);
}
static DEVICE_ATTR_RO(firmware);

static struct attribute *cstm_attrs[] = {
	&dev_attr_raw.attr,
	&dev_attr_rx_last.attr,
	&dev_attr_firmware.attr,
	NULL
};

/*
 * Added to the LED device after registration: the multicolor class sets
 * led_cdev.groups to its own groups, so a group set there would be lost.
 */
static const struct attribute_group cstm_group = {
	.attrs = cstm_attrs,
};

/*
 * Keep one IN transfer queued all the time: the vendor userland reads the IO
 * endpoint continuously, so the STM32 firmware may expect its answers to be
 * drained.
 */
static void cstm_rx_complete(struct urb *urb)
{
	struct cstm_led *led = urb->context;
	unsigned long flags;
	unsigned int len;
	int ret;

	switch (urb->status) {
	case 0:
		len = min_t(unsigned int, urb->actual_length, CSTM_RX_LAST_MAX);
		spin_lock_irqsave(&led->rx_lock, flags);
		memcpy(led->rx_last, led->rx_buf, len);
		led->rx_last_len = len;
		led->rx_count++;
		spin_unlock_irqrestore(&led->rx_lock, flags);
		dev_dbg(&led->intf->dev, "rx %*ph\n", (int)len, led->rx_buf);
		/* Cresnet update request: xx 02 03 00 */
		if (len == 4 && led->rx_buf[1] == 0x02 &&
		    led->rx_buf[2] == 0x03 && led->rx_buf[3] == 0x00)
			schedule_work(&led->resync_work);
		break;
	case -ENOENT:
	case -ECONNRESET:
	case -ESHUTDOWN:
	case -ENODEV:
		return;
	case -EPROTO:
	case -EILSEQ:
	case -ETIME:
		dev_dbg_ratelimited(&led->intf->dev, "rx error %d, stopped\n",
				    urb->status);
		return;
	default:
		dev_dbg_ratelimited(&led->intf->dev, "rx status %d\n",
				    urb->status);
		break;
	}

	ret = usb_submit_urb(urb, GFP_ATOMIC);
	if (ret && ret != -ENODEV && ret != -EPERM)
		dev_warn(&led->intf->dev, "rx resubmit: %d\n", ret);
}

static int cstm_probe(struct usb_interface *intf, const struct usb_device_id *id)
{
	struct usb_host_interface *alt = intf->cur_altsetting;
	struct usb_endpoint_descriptor *ep, *ep_in = NULL, *ep_out = NULL;
	struct device *dev = &intf->dev;
	struct cstm_led *led;
	unsigned int i;
	int ret;

	for (i = 0; i < alt->desc.bNumEndpoints; i++) {
		ep = &alt->endpoint[i].desc;
		if (usb_endpoint_xfer_bulk(ep) || usb_endpoint_xfer_int(ep)) {
			if (usb_endpoint_dir_in(ep) && !ep_in)
				ep_in = ep;
			else if (usb_endpoint_dir_out(ep) && !ep_out)
				ep_out = ep;
		}
	}
	if (!ep_out)
		return -ENODEV;

	led = devm_kzalloc(dev, sizeof(*led), GFP_KERNEL);
	if (!led)
		return -ENOMEM;
	led->tx_buf = devm_kmalloc(dev, CSTM_MAX_PACKET, GFP_KERNEL);
	if (!led->tx_buf)
		return -ENOMEM;

	led->intf = intf;
	led->udev = usb_get_dev(interface_to_usbdev(intf));
	mutex_init(&led->tx_lock);
	spin_lock_init(&led->rx_lock);
	mutex_init(&led->state_lock);
	INIT_WORK(&led->resync_work, cstm_resync_work);
	memset(led->ctl, -1, sizeof(led->ctl));

	if (usb_endpoint_xfer_int(ep_out))
		led->out_pipe = usb_sndintpipe(led->udev, ep_out->bEndpointAddress);
	else
		led->out_pipe = usb_sndbulkpipe(led->udev, ep_out->bEndpointAddress);

	/* string 4 = firmware name, as read by the vendor driver */
	if (usb_string(led->udev, 4, led->fw, sizeof(led->fw)) < 0)
		led->fw[0] = '\0';

	if (ep_in) {
		led->rx_size = max_t(unsigned int, usb_endpoint_maxp(ep_in), 64);
		led->rx_buf = devm_kmalloc(dev, led->rx_size, GFP_KERNEL);
		led->rx_urb = usb_alloc_urb(0, GFP_KERNEL);
		if (!led->rx_buf || !led->rx_urb) {
			ret = -ENOMEM;
			goto err_put;
		}
		if (usb_endpoint_xfer_int(ep_in))
			usb_fill_int_urb(led->rx_urb, led->udev,
					 usb_rcvintpipe(led->udev, ep_in->bEndpointAddress),
					 led->rx_buf, led->rx_size, cstm_rx_complete,
					 led, ep_in->bInterval);
		else
			usb_fill_bulk_urb(led->rx_urb, led->udev,
					  usb_rcvbulkpipe(led->udev, ep_in->bEndpointAddress),
					  led->rx_buf, led->rx_size, cstm_rx_complete,
					  led);
	}

	for (i = 0; i < ARRAY_SIZE(led->subled); i++) {
		led->subled[i].color_index = LED_COLOR_ID_RED + i;
		led->subled[i].channel = CSTM_JOIN_RED + i;
		led->subled[i].intensity = CSTM_MAX_LEVEL;
	}
	led->mc_cdev.subled_info = led->subled;
	led->mc_cdev.num_colors = ARRAY_SIZE(led->subled);
	led->mc_cdev.led_cdev.max_brightness = CSTM_MAX_LEVEL;
	led->mc_cdev.led_cdev.brightness_set_blocking = cstm_brightness_set;
	/* fixed name for the userland (a second bar would get a "_1" suffix) */
	strscpy(led->name, "tsx:rgb:bar", sizeof(led->name));
	led->mc_cdev.led_cdev.name = led->name;

	usb_set_intfdata(intf, led);

	ret = led_classdev_multicolor_register_ext(dev, &led->mc_cdev, NULL);
	if (ret) {
		dev_err(dev, "LED register failed: %d\n", ret);
		goto err_free_urb;
	}

	ret = device_add_group(led->mc_cdev.led_cdev.dev, &cstm_group);
	if (ret)
		dev_warn(dev, "raw/rx_last/firmware attributes: %d\n", ret);

	if (led->rx_urb) {
		ret = usb_submit_urb(led->rx_urb, GFP_KERNEL);
		if (ret)
			dev_warn(dev, "rx submit: %d (continuing without rx)\n", ret);
	}

	dev_info(dev, "Crestron STM32 LED bar, fw \"%s\", out ep %02x (%s), in ep %02x\n",
		 led->fw, ep_out->bEndpointAddress,
		 usb_endpoint_xfer_int(ep_out) ? "int" : "bulk",
		 ep_in ? ep_in->bEndpointAddress : 0);
	return 0;

err_free_urb:
	usb_set_intfdata(intf, NULL);
err_put:
	usb_free_urb(led->rx_urb);
	usb_put_dev(led->udev);
	return ret;
}

static void cstm_disconnect(struct usb_interface *intf)
{
	struct cstm_led *led = usb_get_intfdata(intf);

	usb_kill_urb(led->rx_urb);
	cancel_work_sync(&led->resync_work);
	mutex_lock(&led->tx_lock);
	led->gone = true;
	mutex_unlock(&led->tx_lock);
	device_remove_group(led->mc_cdev.led_cdev.dev, &cstm_group);
	led_classdev_multicolor_unregister(&led->mc_cdev);
	usb_free_urb(led->rx_urb);
	usb_set_intfdata(intf, NULL);
	usb_put_dev(led->udev);
}

static int cstm_suspend(struct usb_interface *intf, pm_message_t message)
{
	struct cstm_led *led = usb_get_intfdata(intf);

	usb_kill_urb(led->rx_urb);
	cancel_work_sync(&led->resync_work);
	return 0;
}

static int cstm_resume(struct usb_interface *intf)
{
	struct cstm_led *led = usb_get_intfdata(intf);

	return led->rx_urb ? usb_submit_urb(led->rx_urb, GFP_NOIO) : 0;
}

static const struct usb_device_id cstm_ids[] = {
	/* Cresnet (interface 1) + console (interface 0) */
	{ USB_DEVICE_INTERFACE_NUMBER(CSTM_VID, 0x001b, CSTM_IO_IFNUM) },
	{ }
};
MODULE_DEVICE_TABLE(usb, cstm_ids);

static struct usb_driver cstm_driver = {
	.name		= "leds-crestron-stm32",
	.probe		= cstm_probe,
	.disconnect	= cstm_disconnect,
	.suspend	= cstm_suspend,
	.resume		= cstm_resume,
	.id_table	= cstm_ids,
};
module_usb_driver(cstm_driver);

MODULE_DESCRIPTION("Crestron TSX-xx60 STM32 RGB LED bar");
MODULE_LICENSE("GPL");
