#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/spi/spi.h>
#include <asm/uaccess.h>
#include <linux/gpio.h>
#include <linux/string.h>

#define BOARDID_STR_LEN 64

int board_id;

int board_id_get_hw_version(void)
{
	printk("lrs enter board_id_get_hw_version()\n");
	return board_id;
}

static int __init setup_boardid(char *str)
{
	board_id = (int)simple_strtol(str, NULL, 10);
	printk("lrs board_id: %d\n", board_id);
	return 1;
}

__setup("androidboot.build_stage=", setup_boardid);
