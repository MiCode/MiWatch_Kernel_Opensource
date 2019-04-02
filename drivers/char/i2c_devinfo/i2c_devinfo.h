/*************************************************************************
* File: i2c-devinfo.h
* Author: Quanyu.Lee
* Mail: liquanyu@longcheer.com
* Create time: 2016.10.20 10:22AM
*************************************************************************/

#ifndef _I2C_DEVINFO_H_
#define _I2C_DEVINFO_H_

#include <linux/cdev.h>
#include <linux/mutex.h>
#include <linux/string.h>

#define I2C_DEVINFO_DEVICE_NODE_NAME "i2c_devinfo"
#define I2C_DEVINFO_DEVICE_FILE_NAME "i2c_devinfo"
#define I2C_DEVINFO_DEVICE_PROC_NAME "i2c_devinfo"
#define I2C_DEVINFO_DEVICE_CLASS_NAME "i2c_devinfo"

struct i2c_devinfo_reg_dev{
    char val[255];
    struct cdev dev;
    struct mutex i2c_devinfo_mutex;
};

//int i2c_devinfo_device_write(char *buf);

#endif
