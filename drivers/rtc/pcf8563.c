/*
 * Copyright (C) 2016 by NXP Semiconductors Inc.
 * Date & Time support for pcf8563 RTC
 */

/*	#define	DEBUG	*/

#include <common.h>
#include <command.h>
#include <dm.h>
#include <i2c.h>
#include <rtc.h>

static int pcf8563_rtc_set(struct udevice *dev, const struct rtc_time *tmp)
{
	uchar buf[1], century;
	int  ret;
	
	debug ( "Set DATE: %4d-%02d-%02d (wday=%d)  TIME: %2d:%02d:%02d\n",
		tmp->tm_year, tmp->tm_mon, tmp->tm_mday, tmp->tm_wday,
		tmp->tm_hour, tmp->tm_min, tmp->tm_sec);
	/* start register address */
	buf[0] = bin2bcd(tmp->tm_year % 100);
	ret = dm_i2c_write(dev, 0x08, buf, 1);
	if (ret < 0)
		return ret;
	
	century = (tmp->tm_year >= 2000) ? 0 : 0x80;
	buf[0] = bin2bcd(tmp->tm_mon) | century;
	ret = dm_i2c_write(dev, 0x07, buf, 1);
	if (ret < 0)
		return ret;

	buf[0] = bin2bcd(tmp->tm_wday);
	ret = dm_i2c_write(dev, 0x06, buf, 1);
	if (ret < 0)
		return ret;

	buf[0] = bin2bcd(tmp->tm_mday);
	ret = dm_i2c_write(dev, 0x5, buf, 1);
	if (ret < 0)
		return ret;

	buf[0] = bin2bcd(tmp->tm_hour);
	ret = dm_i2c_write(dev, 0x4, buf, 1);
	if (ret < 0)
		return ret;

	buf[0] = bin2bcd(tmp->tm_min);
	ret = dm_i2c_write(dev, 0x3, buf, 1);
	if (ret < 0)
		return ret;

	buf[0] = bin2bcd(tmp->tm_sec);
	ret = dm_i2c_write(dev, 0x2, buf, 1);
	if (ret < 0)
		return ret;

	return ret;
}

static int pcf8563_rtc_get(struct udevice *dev, struct rtc_time *tmp)
{
	int i, ret = 0;
	uchar buf[12];
	uchar *p = buf;
	for(i=2;i<0x0d;i++){
		ret = dm_i2c_read(dev, i, p, 1);
		if (ret < 0)
			return ret;
		p++;
	}
	
	debug ( "Get RTC year: %02x mon/cent: %02x mday: %02x wday: %02x "
		"hr: %02x min: %02x sec: %02x\n",
		buf[6], buf[5], buf[3], buf[4],buf[2], buf[1], buf[0] );
	debug ( "Alarms: wday: %02x day: %02x hour: %02x min: %02x\n",
		buf[10],buf[9],buf[8],buf[7]);
	if (buf[0] & 0x80) {
		puts ("### Warning: RTC Low Voltage - date/time not reliable\n");
		ret = -1;
	}
	
	tmp->tm_sec  = bcd2bin (buf[0]  & 0x7F);
	tmp->tm_min  = bcd2bin (buf[1]  & 0x7F);
	tmp->tm_hour = bcd2bin (buf[2] & 0x3F);
	tmp->tm_mday = bcd2bin (buf[3] & 0x3F);
	tmp->tm_mon  = bcd2bin (buf[5] & 0x1F);
	tmp->tm_year = bcd2bin (buf[6]) + ((buf[5] & 0x80) ? 1900 : 2000);
	tmp->tm_wday = bcd2bin (buf[4] & 0x07);
	tmp->tm_yday = 0;
	tmp->tm_isdst= 0;

	debug("Get DATE: %4d-%02d-%02d (wday=%d)  TIME: %2d:%02d:%02d\n",
	      tmp->tm_year, tmp->tm_mon, tmp->tm_mday, tmp->tm_wday,
	      tmp->tm_hour, tmp->tm_min, tmp->tm_sec);

	return ret;
}

static int pcf8563_rtc_reset(struct udevice *dev)
{
	uchar buf[1];
	int  i, ret;
	/* clear all control & status registers */
	buf[0] = 0x00;
	ret = dm_i2c_write(dev, 0, buf, 1);
	if (ret < 0)
		return ret;
	ret = dm_i2c_write(dev, 1, buf, 1);
	if (ret < 0)
		return ret;
	ret = dm_i2c_write(dev, 0x0D, buf, 1);
	if (ret < 0)
		return ret;
	
	/* clear Voltage Low bit */
	ret = dm_i2c_read(dev, 0x02, buf, 1);
	if (ret < 0)
		return ret;
	buf[0] &= 0x7F; 
	ret = dm_i2c_write(dev, 0x02, buf, 1);
	if (ret < 0)
		return ret;
	
	/* reset all alarms */
	buf[0] = 0x00;
	for(i=0x09;i<0x0d;i++){
		ret = dm_i2c_write(dev, i, buf, 1);
		if (ret < 0)
			return ret;
	}
	
	return 0;
}

static const struct rtc_ops pcf8563_rtc_ops = {
	.get = pcf8563_rtc_get,
	.set = pcf8563_rtc_set,
	.reset = pcf8563_rtc_reset,
};

static const struct udevice_id pcf8563_rtc_ids[] = {
	{ .compatible = "nxp,pcf8563" },
	{ }
};

U_BOOT_DRIVER(rtc_pcf8563) = {
	.name	= "rtc-pcf8563",
	.id	= UCLASS_RTC,
	.of_match = pcf8563_rtc_ids,
	.ops	= &pcf8563_rtc_ops,
};
