/*
 * Copyright 2016, Stefan Wahren <stefan.wahren@i2se.com>
 *
 * Based on net/can/led.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/netdevice.h>

#include "qca_spi.h"

static unsigned long led_delay = 50;
module_param(led_delay, ulong, 0644);
MODULE_PARM_DESC(led_delay,
		 "blink delay time for activity leds (msecs, default: 50).");

void qcaled_event(struct net_device *netdev, enum qcaled_event event)
{
	struct qcaspi *qca = netdev_priv(netdev);

	switch (event) {
	case QCALED_EVENT_OPEN:
		led_trigger_event(qca->tx_led_trig, LED_FULL);
		led_trigger_event(qca->rx_led_trig, LED_FULL);
		led_trigger_event(qca->rxtx_led_trig, LED_FULL);
		break;
	case QCALED_EVENT_CLOSE:
		led_trigger_event(qca->tx_led_trig, LED_OFF);
		led_trigger_event(qca->rx_led_trig, LED_OFF);
		led_trigger_event(qca->rxtx_led_trig, LED_OFF);
		break;
	case QCALED_EVENT_TX:
		if (led_delay) {
			led_trigger_blink_oneshot(qca->tx_led_trig,
						  &led_delay, &led_delay, 1);
			led_trigger_blink_oneshot(qca->rxtx_led_trig,
						  &led_delay, &led_delay, 1);
		}
		break;
	case QCALED_EVENT_RX:
		if (led_delay) {
			led_trigger_blink_oneshot(qca->rx_led_trig,
						  &led_delay, &led_delay, 1);
			led_trigger_blink_oneshot(qca->rxtx_led_trig,
						  &led_delay, &led_delay, 1);
		}
		break;
	}
}
EXPORT_SYMBOL_GPL(qcaled_event);

static void qcaled_release(struct device *gendev, void *res)
{
	struct qcaspi *qca = netdev_priv(to_net_dev(gendev));

	led_trigger_unregister_simple(qca->tx_led_trig);
	led_trigger_unregister_simple(qca->rx_led_trig);
	led_trigger_unregister_simple(qca->rxtx_led_trig);
}

void devm_qcaled_init(struct net_device *netdev)
{
	struct qcaspi *qca = netdev_priv(netdev);
	void *res;

	res = devres_alloc(qcaled_release, 0, GFP_KERNEL);
	if (!res) {
		netdev_err(netdev, "cannot register LED triggers\n");
		return;
	}

	snprintf(qca->tx_led_trig_name, sizeof(qca->tx_led_trig_name),
		 "%s-tx", netdev->name);
	snprintf(qca->rx_led_trig_name, sizeof(qca->rx_led_trig_name),
		 "%s-rx", netdev->name);
	snprintf(qca->rxtx_led_trig_name, sizeof(qca->rxtx_led_trig_name),
		 "%s-rxtx", netdev->name);

	led_trigger_register_simple(qca->tx_led_trig_name,
				    &qca->tx_led_trig);
	led_trigger_register_simple(qca->rx_led_trig_name,
				    &qca->rx_led_trig);
	led_trigger_register_simple(qca->rxtx_led_trig_name,
				    &qca->rxtx_led_trig);

	devres_add(&netdev->dev, res);
}
EXPORT_SYMBOL_GPL(devm_qcaled_init);

