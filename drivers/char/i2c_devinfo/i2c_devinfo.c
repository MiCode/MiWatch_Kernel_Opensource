#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/slab.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/printk.h>
#include <linux/err.h>
#include <linux/miscdevice.h>
#include <linux/sysfs.h>
#include "i2c_devinfo.h"
//#include <linux/string.h>

//major dev number & minor dev number
static int i2c_devinfo_major = 0;
static int i2c_devinfo_minor = 0;

//dev class and dev variable
static struct i2c_devinfo_reg_dev*  i2c_devinfo_dev = NULL;

//the traditional operation method of device files
static int i2c_devinfo_open(struct inode* inode, struct file* filp);
static int i2c_devinfo_release(struct inode* inode, struct file* filp);
static ssize_t i2c_devinfo_read(struct file * filp, char __user * buf, size_t size, loff_t * ppos);
static ssize_t i2c_devinfo_write(struct file * filp, const char __user * buf, size_t size, loff_t * ppos);


static int i2c_devinfo_open(struct inode* inode, struct file* filp)
{
	filp->private_data = i2c_devinfo_dev;
	pr_info("i2c_devinfo %s\n",__func__);
	return 0;
}

static int i2c_devinfo_release(struct inode* inode, struct file* filp)
{
	pr_info("i2c_devinfo %s\n",__func__);

	return 0;
}

static  ssize_t i2c_devinfo_read(struct file * filp, char __user * buf, size_t size, loff_t * ppos)
{
	int cnt =0;
	char val[255];

	pr_info("i2c_devinfo %s bf lock\n",__func__);

	mutex_lock(&i2c_devinfo_dev->i2c_devinfo_mutex);
	cnt = sprintf(val,"%s\n",i2c_devinfo_dev->val);
	cnt = simple_read_from_buffer(buf, size, ppos, val, cnt);
	mutex_unlock(&i2c_devinfo_dev->i2c_devinfo_mutex);

	pr_info("i2c_devinfo %s af lock\n",__func__);

	return cnt;
}

static  ssize_t i2c_devinfo_write(struct file * filp, const char __user * buf, size_t size, loff_t * ppos)
{
	//char *mem[100];
	int cnt=1;

	pr_info("i2c_devinfo %s\n",__func__);

	//mutex_lock(&i2c_devinfo_dev->i2c_devinfo_mutex);
	//cnt =  simple_write_to_buffer(mem, size, ppos, buf, sizeof(i2c_devinfo_dev->val));
	strcat(i2c_devinfo_dev->val, buf);

	pr_info("i2c_devinfo write in with:%s\n", i2c_devinfo_dev->val);
	//i2c_devinfo_dev->val = (int)simple_strtoul(mem, NULL, 10);
	//mutex_unlock(&i2c_devinfo_dev->i2c_devinfo_mutex);

	return cnt;
	}

// the traditional dev file operation method table
static struct file_operations i2c_devinfo_fops =
{
	.owner = THIS_MODULE,
	.open = i2c_devinfo_open,
	.release = i2c_devinfo_release,
	.read    = i2c_devinfo_read,
	.write   = i2c_devinfo_write,
};


static struct kobject* i2c_devinfo_device; //sysfs kobject

static ssize_t  i2c_devinfo_sys_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	pr_info("i2c_devinfo %s\n",__func__);

	return  sprintf(buf,"%s\n", i2c_devinfo_dev->val);
}

ssize_t i2c_devinfo_sys_store (struct device *dev, struct device_attribute *attr,const char *buf, size_t count)
{
	pr_info("i2c_devinfo %s\n",__func__);

	strcat(i2c_devinfo_dev->val, buf);

	return  count;
}

static DEVICE_ATTR(i2c_devinfo_info, 0444, i2c_devinfo_sys_show, i2c_devinfo_sys_store);

static int  i2c_devinfo_creat_sysfs(void)
{
	int ret = 0;
	i2c_devinfo_device = kobject_create_and_add("i2c_devinfo_sys",NULL);
	if (!i2c_devinfo_device)
	{
		pr_info("i2c_devinfo subsystem_register failed\n");
		ret = -ENOMEM;
		return ret;
	}
	ret = sysfs_create_file(i2c_devinfo_device, &dev_attr_i2c_devinfo_info.attr);
	if (ret)
	{
		pr_info("i2c_devinfo sysfs_create_file fail\n");
		kobject_del(i2c_devinfo_device);
	}
	return 0;

}

//for devices to write buf add by liquanyu at 1026
int i2c_devinfo_device_write(char *buf)
{
	pr_info("i2c_devinfo write %s\n",buf);

	strcat(i2c_devinfo_dev->val, buf);

	return 1;
}
EXPORT_SYMBOL(i2c_devinfo_device_write);

static int __init i2c_devinfo_init(void)
{
	int ret;
	dev_t devno;
	struct class *myclass;
	//create the char class dev

	devno = MKDEV(i2c_devinfo_major, i2c_devinfo_minor);

	if (i2c_devinfo_major)
	{
		ret = register_chrdev_region(devno, 1, "i2c_devinfo_cdev");
		if (ret)
		{
			printk(KERN_INFO "i2c_devinfo register_chrdev_region fail\n");
			return ret;
		}
	}
	else
	{
		ret  = alloc_chrdev_region(&devno, 0, 1, "i2c_devinfo_cdev");
		if (ret)
		{
			printk(KERN_INFO "i2c_devinfo alloc_chrdev_region fail\n");
			return ret;
		}
		i2c_devinfo_major = MAJOR(devno);
		i2c_devinfo_minor = MINOR(devno);
	}
	i2c_devinfo_dev = kzalloc(sizeof(struct i2c_devinfo_reg_dev), GFP_KERNEL);
	if (IS_ERR_OR_NULL(i2c_devinfo_dev))
	{
		printk(KERN_INFO "i2c_devinfo kzalloc fail\n");
		ret = -ENOMEM;
		goto fail_malloc;
	}

	cdev_init(&i2c_devinfo_dev->dev,&i2c_devinfo_fops);
	i2c_devinfo_dev->dev.owner = THIS_MODULE;

	mutex_init(&i2c_devinfo_dev->i2c_devinfo_mutex);

	ret = cdev_add(&i2c_devinfo_dev->dev, devno, 1);
	if (ret)
	{
		printk(KERN_INFO "i2c_devinfo register_chrdev_region fail\n");
		goto cdev_del;
	}

	myclass = class_create(THIS_MODULE, "i2c_devinfo_cdev_driver");
	//add a character device
	device_create(myclass, NULL, devno, NULL,"i2c_devinfo_cdev");

	//create the node
	i2c_devinfo_creat_sysfs();
	pr_info("i2c_devinfo i2c_devinfo_init sucessfully\n");
#ifdef TESTING
	i2c_devinfo_device_write("testing:1;");
#endif
	return ret;

cdev_del:
	cdev_del(&i2c_devinfo_dev->dev);
fail_malloc:
	unregister_chrdev_region(devno, 1);
	return ret;
}

static void __exit i2c_devinfo_exit(void)
{
	//the cancellation of equipment
	cdev_del(&i2c_devinfo_dev->dev);
	kfree(i2c_devinfo_dev);
	unregister_chrdev_region(i2c_devinfo_dev->dev.dev, 1);
	//remove the nodes
	sysfs_remove_file(i2c_devinfo_device, &dev_attr_i2c_devinfo_info.attr);
	kobject_del(i2c_devinfo_device);
}


module_init(i2c_devinfo_init);
module_exit(i2c_devinfo_exit);

MODULE_AUTHOR("LEE");
MODULE_DESCRIPTION("I2C_DEVINFO_TESTING");
MODULE_LICENSE("GPL v2");
