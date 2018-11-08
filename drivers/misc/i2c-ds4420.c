#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
#include <linux/workqueue.h>

#define _VERSION       "v2"
#define MMA7660FC_DATE          "11.01.2013"

#define CTRL_REGISTER 	0xF8

#define DS4420_VALUE_MASK	0x1F
#define DS4420_MODE_MASK	0x80
#define DS4420_MUTE_MASK	0x20
#define MIN_VOLUME		0
#define MAX_VOLUME 		20

struct ds4420_data {
	bool mode;
	bool mute;
	int value;
	struct i2c_client* client;
};

static int ds4420_set_mode(struct i2c_client* client, bool mode)
{
	u8 data = i2c_smbus_read_byte_data(client, CTRL_REGISTER);

	if (mode)
		data |= DS4420_MODE_MASK;
	else
		data &= ~DS4420_MODE_MASK;

	return i2c_smbus_write_byte_data(client, CTRL_REGISTER, data);
}

static int ds4420_set_mute(struct i2c_client* client,bool mute)
{
	u8 data = i2c_smbus_read_byte_data(client, CTRL_REGISTER);

	if (mute)
		data |= DS4420_MUTE_MASK;
	else
		data &= ~DS4420_MUTE_MASK;

	return i2c_smbus_write_byte_data(client, CTRL_REGISTER, data);
}

static int ds4420_set_volume(struct i2c_client* client,int value)
{
	u8 data = i2c_smbus_read_byte_data(client, CTRL_REGISTER);
	data &= ~DS4420_VALUE_MASK;
	return i2c_smbus_write_byte_data(client, CTRL_REGISTER, data | value);
}

static ssize_t ds4420_mode_read(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ds4420_data* info = dev_get_drvdata(dev);

	return snprintf(buf,PAGE_SIZE,
			"Mode: %s\n",(info->mode)?"Standby":"Normal");
}

static ssize_t ds4420_mute_read(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ds4420_data* info = dev_get_drvdata(dev);
	return snprintf(buf,PAGE_SIZE,
                        "Device is %s\n",(info->mute)?"muted":"unmuted");;
}

static ssize_t ds4420_volume_read(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ds4420_data* info = dev_get_drvdata(dev);
	return snprintf(buf,PAGE_SIZE,
			"%i\n"
                        "Volume: 0..20\n",info->value);
}

static ssize_t ds4420_mute_store(struct device *dev, struct device_attribute *attr,
                    const char *buf, size_t count){
	struct ds4420_data* info = dev_get_drvdata(dev);

	if (!strncmp(buf, "unmute", 6)) {
		info->mute = 0;
	}
	else if (!strncmp(buf,"mute",4)){
		info->mute = 1;
	}
	else
		return -EINVAL;
	ds4420_set_mute(info->client, info->mute);
	return count;
}

static ssize_t ds4420_volume_store(struct device *dev, struct device_attribute *attr,
                    const char *buf, size_t count)
{
	int ret;
	struct ds4420_data* info = dev_get_drvdata(dev);
	s32 value;
	ret = kstrtos32(buf, 0, &value);
	if ((value < MIN_VOLUME)||(value > MAX_VOLUME))
		return -EINVAL;
	ds4420_set_volume(info->client,(info->value=value));
	return count;
}

static DEVICE_ATTR(mode, (S_IRUGO|S_IWUSR), ds4420_mode_read ,NULL);
static DEVICE_ATTR(mute, (S_IRUGO|S_IWUSR), ds4420_mute_read ,ds4420_mute_store);
static DEVICE_ATTR(volume, (S_IRUGO|S_IWUSR), ds4420_volume_read ,ds4420_volume_store);


static const struct attribute *ds4420_manage_attrs[] = {
	&dev_attr_mode.attr,
	&dev_attr_mute.attr,
	&dev_attr_volume.attr,
	NULL,
};
static const struct attribute_group ds4420_manage_attrs_group = {
	.attrs = (struct attribute **) ds4420_manage_attrs,
};


static int ds4420_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct ds4420_data *i2c_data;
	int ret;

	if (!i2c_check_functionality(client->adapter,I2C_FUNC_SMBUS_BYTE_DATA))
		return -ENODEV;

	i2c_data = kzalloc(sizeof(struct ds4420_data), GFP_KERNEL);
	if (!i2c_data)
		return -ENOMEM;

	ds4420_set_mode(client,(i2c_data->mode=0));
	ds4420_set_mute(client,(i2c_data->mute=0));
	ds4420_set_volume(client,(i2c_data->value=MAX_VOLUME));

	i2c_set_clientdata(client, i2c_data);
	i2c_data->client = client;
	ret = sysfs_create_group(&client->dev.kobj, &ds4420_manage_attrs_group);
	if (ret) {
		pr_err("mma7660fc_probe: Unable to create sysfs \n");
		goto exit_sysfs_device_failed;
	}

	return 0;

exit_sysfs_device_failed:
	kfree(i2c_data);
	return ret;
}

static int ds4420_remove(struct i2c_client *client)
{
	struct ds4420_data *data = i2c_get_clientdata(client);
	sysfs_remove_group(&client->dev.kobj, &ds4420_manage_attrs_group);
	kfree(data);
	return 0;
}

static const struct i2c_device_id ds4420_id[] = {
	{"ds4420", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, ds4420_id);

#ifdef CONFIG_OF
static const struct of_device_id ds4420_of_id[] = {
	{ .compatible = "dallas,ds4420" },
	{},
};
MODULE_DEVICE_TABLE(of, ds4420_of_id);
#endif

static struct i2c_driver ds4420_driver = {
	.driver = {
		.name	= "ds4420",
		.of_match_table = of_match_ptr(ds4420_of_id),
	},
	.probe		= ds4420_probe,
	.remove		= ds4420_remove,
	.id_table	= ds4420_id,
};

module_i2c_driver(ds4420_driver);

MODULE_DESCRIPTION("I2C Amplifier DS4420");
MODULE_LICENSE("GPL");
