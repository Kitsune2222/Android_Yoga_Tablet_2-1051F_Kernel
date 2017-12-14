/******************** E-Compass STMicroelectronics LSM303D NEW driver ********************
 *
 * Description:	For Lenovo Yoga Tablet 2. Accelerometer & Magnetometer driver.
 * Author:		Kitsune2222
 * Date:		19.11.2017
 * Version:		0.5
******************************************************************************************
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
******************************************************************************************
*/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/acpi.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/bitops.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/kfifo_buf.h>

// Address registers
#define LSM303D_WHO_AM_I_REG			(0x0F)
#define LSM303D_CNTRL0_REG				(0x1F)
#define LSM303D_CNTRL1_REG				(0x20)
#define LSM303D_CNTRL2_REG				(0x21)
#define LSM303D_CNTRL3_REG				(0x22)
#define LSM303D_CNTRL4_REG				(0x23)
#define LSM303D_CNTRL5_REG				(0x24)
#define LSM303D_CNTRL6_REG				(0x25)
#define LSM303D_CNTRL7_REG				(0x26)

// A-Accelerometer; M-Magnetometer; T-Temperature
#define LSM303D_OUT_X_L_A_REG			(0x28)
#define LSM303D_OUT_X_H_A_REG			(0x29)
#define LSM303D_OUT_Y_L_A_REG			(0x2A)
#define LSM303D_OUT_Y_H_A_REG			(0x2B)
#define LSM303D_OUT_Z_L_A_REG			(0x2C)
#define LSM303D_OUT_Z_H_A_REG			(0x2D)
#define LSM303D_OUT_X_L_M_REG			(0x08)
#define LSM303D_OUT_X_H_M_REG			(0x09)
#define LSM303D_OUT_Y_L_M_REG			(0x0A)
#define LSM303D_OUT_Y_H_M_REG			(0x0B)
#define LSM303D_OUT_Z_L_M_REG			(0x0C)
#define LSM303D_OUT_Z_H_M_REG			(0x0D)
#define LSM303D_OUT_L_T_REG				(0x05)
#define LSM303D_OUT_H_T_REG				(0x06)

#define LSM303D_INT_CTRL_M_REG			(0x12)
#define LSM303D_INT_SRC_M_REG			(0x13)
#define LSM303D_INT_THS_L_M_REG			(0x14)
#define LSM303D_INT_THS_H_M_REG			(0x15)

#define LSM303D_INT_GEN_1_AXIS_REG		(0x30)
#define LSM303D_INT_GEN_1_SRC_REG		(0x31)
#define LSM303D_INT_GEN_1_THS_REG		(0x32)
#define LSM303D_INT_GEN_1_DUR_REG		(0x33)
#define LSM303D_INT_GEN_2_AXIS_REG		(0x34)
#define LSM303D_INT_GEN_2_SRC_REG		(0x35)
#define LSM303D_INT_GEN_2_THS_REG		(0x36)
#define LSM303D_INT_GEN_2_DUR_REG		(0x37)

// ODR (TEST)
//#define ODR_ACC_MASK			(0XF0)	/* Mask for odr change on acc */
//#define LSM303D_ACC_ODR_OFF		(0x00)  /* Power down */
//#define LSM303D_ACC_ODR3_125	(0x10)  /* 3.25Hz output data rate */
//#define LSM303D_ACC_ODR6_25		(0x20)  /* 6.25Hz output data rate */
//#define LSM303D_ACC_ODR12_5		(0x30)  /* 12.5Hz output data rate */
//#define LSM303D_ACC_ODR25		(0x40)  /* 25Hz output data rate */
//#define LSM303D_ACC_ODR50		(0x50)  /* 50Hz output data rate */
//#define LSM303D_ACC_ODR100		(0x60)  /* 100Hz output data rate */
//#define LSM303D_ACC_ODR200		(0x70)  /* 200Hz output data rate */
//#define LSM303D_ACC_ODR400		(0x80)  /* 400Hz output data rate */
//#define LSM303D_ACC_ODR800		(0x90)  /* 800Hz output data rate */
//#define LSM303D_ACC_ODR1600		(0xA0)  /* 1600Hz output data rate */

//#define ODR_MAG_MASK			(0X1C)	/* Mask for odr change on mag */
//#define LSM303D_MAG_ODR3_125	(0x00)  /* 3.25Hz output data rate */
//#define LSM303D_MAG_ODR6_25		(0x04)  /* 6.25Hz output data rate */
//#define LSM303D_MAG_ODR12_5		(0x08)  /* 12.5Hz output data rate */
//#define LSM303D_MAG_ODR25		(0x0C)  /* 25Hz output data rate */
//#define LSM303D_MAG_ODR50		(0x10)  /* 50Hz output data rate */
//#define LSM303D_MAG_ODR100		(0x14)  /* 100Hz output data rate */

#define LSM9DS0_ACCEL_POWER_DOWN        (0x00 << 4)
#define LSM9DS0_ACCEL_ODR_3_125HZ_VAL   (0x01 << 4)
#define LSM9DS0_ACCEL_ODR_6_25HZ_VAL    (0x02 << 4)
#define LSM9DS0_ACCEL_ODR_12_5HZ_VAL    (0x03 << 4)
#define LSM9DS0_ACCEL_ODR_25HZ_VAL      (0x04 << 4)
#define LSM9DS0_ACCEL_ODR_50HZ_VAL      (0x05 << 4)
#define LSM9DS0_ACCEL_ODR_100HZ_VAL     (0x06 << 4)
#define LSM9DS0_ACCEL_ODR_200HZ_VAL     (0x07 << 4)
#define LSM9DS0_ACCEL_ODR_400HZ_VAL     (0x08 << 4)
#define LSM9DS0_ACCEL_ODR_800HZ_VAL     (0x09 << 4)
#define LSM9DS0_ACCEL_ODR_1600HZ_VAL    (0x0A << 4)

#define LSM9DS0_ACCEL_FS_MASK           (0x03 << 3)
#define LSM9DS0_ACCEL_FS_2G_VAL         (0x00 << 3)
#define LSM9DS0_ACCEL_FS_4G_VAL         (0x01 << 3)
#define LSM9DS0_ACCEL_FS_6G_VAL         (0x02 << 3)
#define LSM9DS0_ACCEL_FS_8G_VAL         (0x03 << 3)
#define LSM9DS0_ACCEL_FS_16G_VAL        (0x04 << 3)
#define LSM9DS0_ACCEL_FS_2G_GAIN        61     /* ug/LSB  */
#define LSM9DS0_ACCEL_FS_4G_GAIN        122    /* ug/LSB  */
#define LSM9DS0_ACCEL_FS_6G_GAIN        183    /* ug/LSB  */
#define LSM9DS0_ACCEL_FS_8G_GAIN        244    /* ug/LSB  */
#define LSM9DS0_ACCEL_FS_16G_GAIN       732    /* ug/LSB  */

#define LSM9DS0_MAGN_ODR_3_125HZ_VAL    (0x00 << 2)
#define LSM9DS0_MAGN_ODR_6_25HZ_VAL     (0x01 << 2)
#define LSM9DS0_MAGN_ODR_12_5HZ_VAL     (0x02 << 2)
#define LSM9DS0_MAGN_ODR_25HZ_VAL       (0x03 << 2)
#define LSM9DS0_MAGN_ODR_50HZ_VAL       (0x04 << 2)
#define LSM9DS0_MAGN_ODR_100HZ_VAL      (0x05 << 2)

#define LSM9DS0_MAGN_FS_MASK            (0x03 << 5)
#define LSM9DS0_MAGN_FS_2GAUSS_VAL      (0x00 << 5)
#define LSM9DS0_MAGN_FS_4GAUSS_VAL      (0x01 << 5)
#define LSM9DS0_MAGN_FS_8GAUSS_VAL      (0x02 << 5)
#define LSM9DS0_MAGN_FS_12GAUSS_VAL     (0x03 << 5)
#define LSM9DS0_MAGN_FS_2GAUSS_GAIN     80     /* ugauss/LSB  */
#define LSM9DS0_MAGN_FS_4GAUSS_GAIN     160    /* ugauss/LSB  */
#define LSM9DS0_MAGN_FS_8GAUSS_GAIN     320    /* ugauss/LSB  */
#define LSM9DS0_MAGN_FS_12GAUSS_GAIN    480    /* ugauss/LSB  */

#define LSM9DS0_ACCEL_X_EN              BIT(0) 
#define LSM9DS0_ACCEL_Y_EN              BIT(1) 
#define LSM9DS0_ACCEL_Z_EN              BIT(2) 
#define LSM9DS0_TEMP_EN                 BIT(7)
#define LSM9DS0_MAGN_LOW_RES_VAL        (0x00 << 5)
#define LSM9DS0_MAGN_HIGH_RES_VAL       (0x03 << 5)
#define LSM9DS0_MAGN_POWER_DOWN         (0x02)
#define LSM9DS0_MAGN_CONT_CONV_MODE     (0x00)
#define LSM9DS0_MAGN_SINGLE_CONV_MODE   (0x01)

#define LSM303D_ACCEL_MAGN_ID	(0x49)	// ???

enum { ACCEL_MAGN };
enum { SCAN_INDEX_X, SCAN_INDEX_Y, SCAN_INDEX_Z };
enum {
	SCAN_INDEX_ACCEL_X, SCAN_INDEX_ACCEL_Y, SCAN_INDEX_ACCEL_Z,
	SCAN_INDEX_MAGN_X, SCAN_INDEX_MAGN_Y, SCAN_INDEX_MAGN_Z
};
	

struct lsm303d_data {
	struct i2c_client *client;
	struct mutex lock;
	int sensor_type;
	int accel_scale;
	int magn_scale;
};

struct sensor_fs_avl {
	unsigned int num;
	u8 value;
	unsigned int gain;
};

static const struct sensor_fs_avl lsm303d_accel_fs_avl[5] = {
	{2,  LSM9DS0_ACCEL_FS_2G_VAL,  LSM9DS0_ACCEL_FS_2G_GAIN},
	{4,  LSM9DS0_ACCEL_FS_4G_VAL,  LSM9DS0_ACCEL_FS_4G_GAIN},
	{6,  LSM9DS0_ACCEL_FS_6G_VAL,  LSM9DS0_ACCEL_FS_6G_GAIN},
	{8,  LSM9DS0_ACCEL_FS_8G_VAL,  LSM9DS0_ACCEL_FS_8G_GAIN},
	{16, LSM9DS0_ACCEL_FS_16G_VAL, LSM9DS0_ACCEL_FS_16G_GAIN},
};

static const struct sensor_fs_avl lsm303d_magn_fs_avl[4] = {
	{2,  LSM9DS0_MAGN_FS_2GAUSS_VAL,  LSM9DS0_MAGN_FS_2GAUSS_GAIN},
	{4,  LSM9DS0_MAGN_FS_4GAUSS_VAL,  LSM9DS0_MAGN_FS_4GAUSS_GAIN},
	{8,  LSM9DS0_MAGN_FS_8GAUSS_VAL,  LSM9DS0_MAGN_FS_8GAUSS_GAIN},
	{12, LSM9DS0_MAGN_FS_12GAUSS_VAL, LSM9DS0_MAGN_FS_12GAUSS_GAIN},
};

static ssize_t lsm303d_show_scale_avail(struct device *dev,
										struct device_attribute *attr, char *buf)
{
	size_t len = 0;
	int n;
	const struct sensor_fs_avl (*avl)[];
	
	if (strcmp(attr->attr.name, "in_accel_scale_available") == 0) {
		avl = &lsm303d_accel_fs_avl;
		n = ARRAY_SIZE(lsm303d_accel_fs_avl);
	} else if (strcmp(attr->attr.name, "in_magn_scale_available") == 0) {
		avl = &lsm303d_magn_fs_avl;
		n = ARRAY_SIZE(lsm303d_magn_fs_avl);
	} else {
		return -EINVAL;
	}
	
	while (n-- > 0)
		len += scnprintf(buf + len, PAGE_SIZE - len,
						 "0.%06u ", (*avl)[n].gain);
	buf[len - 1] = '\n';
	
	return len;
}

static IIO_DEVICE_ATTR(in_accel_scale_available, S_IRUGO,
					   lsm303d_show_scale_avail, NULL, 0);
static IIO_DEVICE_ATTR(in_magn_scale_available, S_IRUGO,
					   lsm303d_show_scale_avail, NULL, 0);

static struct attribute *lsm303d_acc_magn_attributes[] = {
	&iio_dev_attr_in_accel_scale_available.dev_attr.attr,
	&iio_dev_attr_in_magn_scale_available.dev_attr.attr,
	NULL
};

static const struct attribute_group lsm303d_acc_magn_group = {
	.attrs = lsm303d_acc_magn_attributes,
};

static const struct iio_buffer_setup_ops lsm303d_buffer_setup_ops = {
	.postenable = &iio_triggered_buffer_postenable,
	.predisable = &iio_triggered_buffer_predisable,
};
	
static const struct iio_chan_spec lsm303d_acc_magn_channels[] = {
	{
		.type = IIO_ACCEL,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),
		.modified = 1,
		.channel2 = IIO_MOD_X,
		.scan_index = SCAN_INDEX_ACCEL_X,
		.scan_type = {
			.sign = 's',
			.realbits = 16,
			.storagebits = 16,
			.shift = 0,
			.endianness = IIO_LE,
		},
	}, {
		.type = IIO_ACCEL,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),
		.modified = 1,
		.channel2 = IIO_MOD_Y,
		.scan_index = SCAN_INDEX_ACCEL_Y,
		.scan_type = {
			.sign = 's',
			.realbits = 16,
			.storagebits = 16,
			.shift = 0,
			.endianness = IIO_LE,
		},
	}, {
		.type = IIO_ACCEL,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),
		.modified = 1,
		.channel2 = IIO_MOD_Z,
		.scan_index = SCAN_INDEX_ACCEL_Z,
		.scan_type = {
			.sign = 's',
			.realbits = 16,
			.storagebits = 16,
			.shift = 0,
			.endianness = IIO_LE,
		},
	}, {
		.type = IIO_MAGN,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),
		.modified = 1,
		.channel2 = IIO_MOD_X,
		.scan_index = SCAN_INDEX_ACCEL_X,
		.scan_type = {
			.sign = 's',
			.realbits = 16,
			.storagebits = 16,
			.shift = 0,
			.endianness = IIO_LE,
		},
	}, {		
		.type = IIO_MAGN,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),
		.modified = 1,
		.channel2 = IIO_MOD_Y,
		.scan_index = SCAN_INDEX_ACCEL_Y,
		.scan_type = {
			.sign = 's',
			.realbits = 16,
			.storagebits = 16,
			.shift = 0,
			.endianness = IIO_LE,
		},
	}, {
		.type = IIO_MAGN,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),
		.modified = 1,
		.channel2 = IIO_MOD_Z,
		.scan_index = SCAN_INDEX_ACCEL_Z,
		.scan_type = {
			.sign = 's',
			.realbits = 16,
			.storagebits = 16,
			.shift = 0,
			.endianness = IIO_LE,
		},
	},
	IIO_CHAN_SOFT_TIMESTAMP(6),
};

static int lsm303d_read_measurements(struct i2c_client *client,
									 u8 reg_address, s16 *x, s16 *y, s16 *z)
{
	int ret;
	u8 buf[6] = {0};
	
	buf[0] = 0x80 | reg_address;
	ret = i2c_master_send(client, buf, 1);
	if (ret < 0)
		return ret;
	
	ret = i2c_master_recv(client, buf, 6);
	if (ret < 0)
		return ret;
	
	*x = (buf[1] << 8) | buf[0];
	*y = (buf[3] << 8) | buf[2];
	*z = (buf[5] << 8) | buf[4];
	return ret;
}

static int lsm303d_read_raw(struct iio_dev *iio_dev,
							struct iio_chan_spec const *channel,
							int *val, int *val2, long mask)
{
	struct lsm303d_data *data = iio_priv(iio_dev);
	int err = 0;
	s16 x = 0, y = 0, z = 0;
	int scale = 0;
	
	switch (mask) {
		case IIO_CHAN_INFO_RAW:
			mutex_lock(&data->lock);
			switch (channel->type) {
				case IIO_ACCEL:
					err = lsm303d_read_measurements(data->client,
						LSM303D_OUT_X_L_A_REG, &x, &y, &z);
					scale = data->accel_scale;
					break;
				case IIO_MAGN:
					err = lsm303d_read_measurements(data->client,
						LSM303D_OUT_X_L_M_REG, &x, &y, &z);
					scale = data->magn_scale;
					break;
				default:
					return -EINVAL;
			}
			mutex_unlock(&data->lock);
			if (err < 0)
				goto read_error;
			
			switch (channel->channel2) {
				case IIO_MOD_X:
					*val = x;
					break;
				case IIO_MOD_Y:
					*val = y;
					break;
				case IIO_MOD_Z:
					*val = z;
					break;
			}
			return IIO_VAL_INT;
		case IIO_CHAN_INFO_SCALE:
			*val = 0;
			switch (channel->type) {
				case IIO_ACCEL:
					*val2 = data->accel_scale;
					break;
				case IIO_MAGN:
					*val2 = data->magn_scale;
					break;
				default:
					return -EINVAL;
			}
			return IIO_VAL_INT_PLUS_MICRO;
		default:
			return -EINVAL;
	}
	
read_error:
	return err;
}
													
static int lsm303d_write_config(struct i2c_client *client,
								u8 reg_address, u8 mask, u8 value)
{
	u8 reg;
	s32 ret;
	ret = i2c_smbus_read_byte_data(client, reg_address);
	if (ret < 0)
		return -EINVAL;
	
	ret = (u8)ret;
	reg &= ~mask;
	reg |= value;
	
	ret = i2c_smbus_write_byte_data(client, reg_address, reg);
	
	return ret;
}
	
static int lsm303d_write_raw(struct iio_dev *indio_dev,
							 struct iio_chan_spec const *channel,
							 int val, int val2, long mask)
{
	struct lsm303d_data *data = iio_priv(indio_dev);
	struct i2c_client *client = data->client;
	const struct sensor_fs_avl (*avl)[];
	int n, i, ret;
	u8 reg_address, reg_mask, new_value;
	int *scale_in_data;
	
	mutex_lock(&data->lock);
	switch (mask) {
		case IIO_CHAN_INFO_SCALE:
			dev_info(&client->dev, "Vals %d %d\n", val, val2);
			switch (channel->type) {
				case IIO_ACCEL:
					avl = &lsm303d_accel_fs_avl;
					n = ARRAY_SIZE(lsm303d_accel_fs_avl);
					reg_address = LSM303D_CNTRL2_REG;
					reg_mask = LSM9DS0_ACCEL_FS_MASK;
					scale_in_data = &(data->accel_scale);
					break;
				case IIO_MAGN:
					avl = &lsm303d_magn_fs_avl;
					n = ARRAY_SIZE(lsm303d_magn_fs_avl);
					reg_address = LSM303D_CNTRL6_REG;
					reg_mask = LSM9DS0_MAGN_FS_MASK;
					scale_in_data = &(data->magn_scale);
					break;
				default:
					ret = -EINVAL;
					goto done;
			}
			ret = -EINVAL;
			for (i = 0; i < n; i++) {
				if ((*avl)[i].gain == val2) {
					ret = 0;
					new_value = (*avl)[i].value;
					break;
				}
			}
			if (ret < 0)
				goto done;
			
			*scale_in_data = (*avl)[i].gain;
			break;
		default:
			ret = -EINVAL;
	}
	
done:
	mutex_unlock(&data->lock);
	return ret;
}
	
static irqreturn_t lsm303d_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct lsm303d_data *data = iio_priv(indio_dev);
	u32 *buf_data;
	int i, j;
	s16 x1, y1, z1, x2, y2, z2;
	int err;
	
	buf_data = kmalloc(indio_dev->scan_bytes, GFP_KERNEL);
	if (!buf_data)
		goto done;
	
	mutex_lock (&data->lock);
	if (!bitmap_empty(indio_dev->active_scan_mask, indio_dev->masklength)) {
		
		err = lsm303d_read_measurements(data->client,
									  LSM303D_OUT_X_L_A_REG, &x1, &y1, &z1);
		if (err < 0)
			goto free_buf;
		err = lsm303d_read_measurements(data->client,
									  LSM303D_OUT_X_L_M_REG, &x2, &y2, &z2);
		if (err < 0)
			goto free_buf;
		
		for (i = 0, j = 0;
			 i < bitmap_weight(indio_dev->active_scan_mask, indio_dev->masklength);
			 i++, j++) {
			j = find_next_bit(indio_dev->active_scan_mask, indio_dev->masklength, j);
			
			switch (j) {
				case SCAN_INDEX_ACCEL_X:
					buf_data[i] = x1;
					break;
				case SCAN_INDEX_ACCEL_Y:
					buf_data[i] = y1;
					break;
				case SCAN_INDEX_ACCEL_Z:
					buf_data[i] = z1;
					break;
				case SCAN_INDEX_MAGN_X:
					buf_data[i] = x2;
					break;
                case SCAN_INDEX_MAGN_Y:
					buf_data[i] = y2;
					break;
                case SCAN_INDEX_MAGN_Z:
					buf_data[i] = z2;
					break;
				default:
					break;
			}
		}
	}
	
	iio_push_to_buffers_with_timestamp(indio_dev, buf_data, iio_get_time_ns(indio_dev));
	
free_buf:
	kfree(buf_data);
	mutex_unlock(&data->lock);
	
done:
	iio_trigger_notify_done(indio_dev->trig);
	
	return IRQ_HANDLED;
}

static const struct iio_info lsm303d_acc_magn_info = {
	.attrs = &lsm303d_acc_magn_group,
	.read_raw = lsm303d_read_raw,
	.write_raw = lsm303d_write_raw,
	.driver_module = THIS_MODULE,
};

static int lsm303d_acc_magn_init(struct i2c_client *client)
{
	int ret;
	struct iio_dev *indio_dev;
	struct lsm303d_data *data;
	
	ret = i2c_smbus_write_byte_data(client, LSM303D_CNTRL1_REG,
		LSM9DS0_ACCEL_ODR_100HZ_VAL | LSM9DS0_ACCEL_X_EN |
		LSM9DS0_ACCEL_Y_EN | LSM9DS0_ACCEL_Z_EN);
	if (ret < 0) {
		dev_err(&client->dev, "Failed to write control register 1.\n");
		return ret;
	}
	dev_info(&client->dev, "Write control register 1. OK.\n");
	ret = i2c_smbus_write_byte_data(client, LSM303D_CNTRL5_REG,
		LSM9DS0_TEMP_EN | LSM9DS0_MAGN_HIGH_RES_VAL | LSM9DS0_MAGN_ODR_50HZ_VAL);
	if (ret < 0) {
		dev_err(&client->dev, "Failed to write control register 5.\n");
		return ret;
	}
	dev_info(&client->dev, "Write control register 5. OK.\n");
	ret = i2c_smbus_write_byte_data(client, LSM303D_CNTRL7_REG,
		LSM9DS0_MAGN_CONT_CONV_MODE);
	if (ret < 0) {
		dev_err(&client->dev, "Failed to write control register 7.\n");
		return ret;
	}
	dev_info(&client->dev, "Write control register 7. OK.\n");
	ret = i2c_smbus_write_byte_data(client, LSM303D_CNTRL2_REG,
		LSM9DS0_ACCEL_FS_2G_VAL);
	if (ret < 0) {
		dev_err(&client->dev, "Failed to write control register 2.\n");
		return ret;
	}
	dev_info(&client->dev, "Write control register 2. OK.\n");
	ret = i2c_smbus_write_byte_data(client, LSM303D_CNTRL6_REG,
		LSM9DS0_MAGN_FS_2GAUSS_VAL);
	if (ret < 0) {
		dev_err(&client->dev, "Failed to write control register 6.\n");
		return ret;
	}
	dev_info(&client->dev, "Write control register 6. OK.\n");
	
	indio_dev = i2c_get_clientdata (client);
	data = iio_priv(indio_dev);
	
	data->accel_scale = LSM9DS0_ACCEL_FS_2G_GAIN;
	data->magn_scale = LSM9DS0_MAGN_FS_2GAUSS_GAIN;
	
	return 0;
}
	
static int lsm303d_probe(struct i2c_client *client,
						 const struct i2c_device_id *id)
{
	struct iio_dev *indio_dev;
	struct lsm303d_data *data;
	struct iio_buffer *buffer;
	int sensor_type;
	int ret;
	
	
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_WORD_DATA)) {
		ret = -ENODEV;
		goto error_ret;
	}
	
	ret = i2c_smbus_read_byte_data(client, LSM303D_WHO_AM_I_REG);
	if (ret < 0) {
		ret = -EINVAL;
		goto error_ret;
	}
	if (ret == LSM303D_ACCEL_MAGN_ID) {
		dev_info(&client->dev, "LSM303D e-compass found.\n");
		sensor_type = ACCEL_MAGN;
	} else {
		dev_err(&client->dev, "No LSM303D sensor found.\n");
		ret = -ENODEV;
		goto error_ret;
	}
	
	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*data));
	if (!indio_dev) {
		ret = -ENOMEM;
		goto error_ret;
	}
	
	data = iio_priv(indio_dev);
	mutex_init(&data->lock);
	i2c_set_clientdata(client, indio_dev);
	data->client = client;
	data->sensor_type = sensor_type;
	
	indio_dev->dev.parent = &client->dev;
	indio_dev->name = dev_name(&client->dev);
	indio_dev->modes = INDIO_DIRECT_MODE | INDIO_BUFFER_TRIGGERED;
	
	
	ret = lsm303d_acc_magn_init(client);
	indio_dev->info = &lsm303d_acc_magn_info;
	indio_dev->channels = lsm303d_acc_magn_channels;
	indio_dev->num_channels = ARRAY_SIZE(lsm303d_acc_magn_channels);
	if (ret < 0) {
		dev_err(&client->dev, "Accelerometer, magnetometer init error.\n");
		goto error_free_device;
	}
	dev_info(&client->dev, "Accelerometer, magnetometer init successfull.\n");
	
	buffer = iio_kfifo_allocate();
	if (!buffer) {
		ret = -ENOMEM;
		goto error_free_device;
	}
	iio_device_attach_buffer(indio_dev, buffer);

	indio_dev->setup_ops = &lsm303d_buffer_setup_ops;
	indio_dev->pollfunc = iio_alloc_pollfunc(NULL,
											 &lsm303d_trigger_handler,
											 IRQF_ONESHOT,
											 indio_dev,
											 "lsm303d_consumer%d",
											 indio_dev->id);
	if (!indio_dev->pollfunc) {
		ret = -ENOMEM;
		goto error_free_buffer;
	}
	ret = iio_device_register(indio_dev);
	if (ret < 0)
		dev_err(&client->dev, "Unable to register LSM303D.\n");
		goto error_unconfigure_buffer;
	
	return 0;
	
error_unconfigure_buffer:
	iio_dealloc_pollfunc(indio_dev->pollfunc);
error_free_buffer:
	iio_kfifo_free(indio_dev->buffer);
error_free_device:
	iio_device_free(indio_dev);
error_ret:
	return ret;
}

static int lsm303d_remove(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	iio_device_unregister(indio_dev);
	iio_device_free(indio_dev);
	dev_info(&client->dev, "Driver removed.");
	return 0;
}

static const struct i2c_device_id lsm303d_id[] = {
	{ "lsm303d_acc_mag", 0 },
	{ "ACCL0001", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, lsm303d_id);

#ifdef CONFIG_ACPI
static const struct acpi_device_id lsm303d_acpi_match[] = {
	{ "ACCL0001", 0 },
	{ },
};
MODULE_DEVICE_TABLE(acpi,lsm303d_acpi_match);
#endif

static struct i2c_driver lsm303d_driver = {
	.driver = {
		.name = "lsm303d",
		.owner = THIS_MODULE,
		.acpi_match_table = ACPI_PTR(lsm303d_acpi_match),
	},
	.probe = lsm303d_probe,
	.remove = lsm303d_remove,
	.id_table = lsm303d_id,
};
module_i2c_driver(lsm303d_driver);

MODULE_AUTHOR("Kitsune");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("LSM303D accelerometer and magnetometer driver");
	
