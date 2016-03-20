/*
 * Copyright 2016, Stefan Wahren <stefan.wahren@i2se.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _QCA_LED_H
#define _QCA_LED_H

#include <linux/if.h>
#include <linux/leds.h>
#include <linux/netdevice.h>

enum qcaled_event {
	QCALED_EVENT_OPEN,
	QCALED_EVENT_CLOSE,
	QCALED_EVENT_TX,
	QCALED_EVENT_RX,
};

#ifdef CONFIG_QCA7000_LEDS

/* keep space for interface name + "-tx"/"-rx"/"-rxtx"
 * suffix and null terminator
 */
#define QCALED_NAME_SZ (IFNAMSIZ + 6)

void qcaled_event(struct net_device *netdev, enum qcaled_event event);
void devm_qcaled_init(struct net_device *netdev);

#else

static inline void qcaled_event(struct net_device *netdev,
				enum qcaled_event event)
{
}

static inline void devm_qcaled_init(struct net_device *netdev)
{
}

#endif

#endif /* _QCA_LED_H */
