/*************************************************************
*                                                            *
*  Driver for NanJingTianYiHeXin HX9031A & HX9023E Cap Sense *
*                                                            *
*************************************************************/
/*
                       客制化配置说明

1. dts参考配置:
    sar_hx9031a@28 {
            compatible = "tyhx,hx9031a";
            i2c_num = <3>;
            i2c_addr = <0x28 0 0 0>;
            interrupt-parent = <&pio>;
            interrupts = <5 IRQ_TYPE_EDGE_FALLING 5 0>;
            tyhx,irq-gpio = <&pio 5 0x0>;
            tyhx,irq-flags = <2>;           // 1:rising, 2:falling;
            tyhx,channel-flag = <0x7>;      //必要，每个channel对应一个input设备，每个bit对应一个channel（channel）。如0xF代表开启0，1，2，3通道
            //tyhx,power-supply-type =<1>;  //可选，如何供电，如果有的话，客户自行移植
            status = "okay";
    };

2. 填充上下电函数：static void hx9031a_power_on(uint8_t on) 如果是常供电，可不用配置此项

3. 根据实际硬件电路形式，FAE协助确认通道配置函数中的cs和channel映射关系：static void hx9031a_ch_cfg(uint8_t chip_select)

4. 如果需要做hx9023E和hx9031A的芯片兼容，可在函数static uint8_t hx9031a_get_board_info(void)中根据board id的不同，
   返回不同的芯片信息。如果没有该兼容需求，则令该函数直接返回实际的芯片型号信息

5. 寄存器默认值
   如果贴片为hx9023E，请在头文件的数组 hx9031a_reg_init_list_hx9023e 中配置默认的寄存器值。
   如果贴片为hx9031A，请在头文件的数组 hx9031a_reg_init_list_hx9031a 中配置默认的寄存器值。

6. 阈值配置默认值
   如果贴片为hx9023E，请在数组 hx9031a_ch_thres_default_hx9023e 中根据实测结果修改默认阈值设置值。
   如果贴片为hx9031A，请在数组 hx9031a_ch_thres_default_hx9031a 中根据实测结果修改默认阈值设置值。

7. 根据实际需要，在dtsi文件的对应节点中和dts解析函数static int hx9031a_parse_dt(struct device *dev)中添加你需要的其他属性信息，
   如其他gpio和regulator等，参考dts配置中的中断gpio号"tyhx,irq-gpio"和通道数配置"tyhx,channel-flag"是必要的。
   更多的gpio配置，请放在 static int hx9031a_gpio_init(void) 和 static void hx9031a_gpio_deinit(void) 中
8. version change:
  v0.48 change based on vnc server version.
*/

#define HX9031A_DRIVER_VER "Change-Id 048"

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/syscalls.h>
#include <linux/version.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/notifier.h>
#include <linux/power_supply.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/irqdomain.h>
#include <linux/irq.h>
#include "sensors.h"
/*
#ifdef CONFIG_PM_WAKELOCKS
#include <linux/pm_wakeup.h>
#else
#include <linux/wakelock.h> //4.19的内核上必须走这个，所以需要将代码中的所有宏判断 #ifdef CONFIG_PM_WAKELOCKS 全部替换成 #if 0
#endif
*/
#include "hx9031a.h"

static struct i2c_client *hx9031a_i2c_client = NULL;
static struct hx9031a_platform_data hx9031a_pdata;
static uint8_t hx9031a_monitor_addr = RA_PRF_CFG_0x02;
static uint8_t hx9031a_monitor_val = 0;
static uint8_t ch_enable_status = 0x00;
static uint8_t hx9031a_polling_enable = 0;
static int hx9031a_polling_period_ms = 0;
struct regulator *hx9031a_vdd = NULL;

static int32_t data_raw[HX9031A_CH_NUM] = {0};
static int32_t data_diff[HX9031A_CH_NUM] = {0};
static int32_t data_lp[HX9031A_CH_NUM] = {0};
static int32_t data_bl[HX9031A_CH_NUM] = {0};
static uint32_t data_offset_dac[HX9031A_CH_NUM] = {0};

//hx9023E默认阈值设置值，请客户根据实测修改
static struct hx9031a_near_far_threshold hx9031a_ch_thres_default_hx9023e[HX9031A_CH_NUM] = {
    {.thr_near = 375, .thr_far = 310}, //ch0//gg1208
    {.thr_near = 160, .thr_far = 128},
    {.thr_near = 128, .thr_far = 128},
    {.thr_near = 32767, .thr_far = 32767},
};
//hx9031A默认阈值设置值，请客户根据实测修改
static struct hx9031a_near_far_threshold hx9031a_ch_thres_default_hx9031a[HX9031A_CH_NUM] = {
    {.thr_near = 256, .thr_far = 224}, //ch0
    {.thr_near = 160, .thr_far = 128},
    {.thr_near = 128, .thr_far = 128},
    {.thr_near = 32767, .thr_far = 32767},
};
static struct hx9031a_near_far_threshold *hx9031a_ch_thres = NULL;

static struct hx9031a_addr_val_pair *hx9031a_reg_init_list = NULL;
static uint16_t hx9031a_reg_init_list_size = 0;

static uint8_t hx9031a_ch_near_state[HX9031A_CH_NUM] = {0};  //只有远近
static volatile uint8_t hx9031a_irq_from_suspend_flag = 0;
static volatile uint8_t hx9031a_irq_en_flag = 1;
static volatile uint8_t hx9031a_hw_monitor_en_flag = HX9031A_HW_MONITOR_EN;
static volatile uint8_t hx9031a_output_switch = HX9031A_OUTPUT_LP_BL;
static volatile uint8_t hx9031a_data_accuracy = 16;

static DEFINE_MUTEX(hx9031a_i2c_rw_mutex);
static DEFINE_MUTEX(hx9031a_ch_en_mutex);
static DEFINE_MUTEX(hx9031a_cali_mutex);

/*
#ifdef CONFIG_PM_WAKELOCKS
static struct wakeup_source hx9031a_wake_lock;
#else
static struct wake_lock hx9031a_wake_lock;
#endif
*/

#if HX9031A_ALG_COMPILE_EN
static volatile uint8_t hx9031a_alg_ref_ch_drdy_int_en_flag = 0;
static volatile uint8_t hx9031a_alg_dynamic_threshold_en = 0;//默认关闭ALG，通过节点使能该功能前必须保证所有通道是关闭状态！！！
static int16_t hx9031a_alg_ch_thres_ini[HX9031A_CH_NUM] = {
    0, 0, 0, 0
};
static int16_t hx9031a_alg_ch_thres_drift[HX9031A_CH_NUM] = {
    0, 0, 0, 0
};
static struct hx9031a_near_far_threshold hx9031a_alg_ch_thres_backup[HX9031A_CH_NUM] = {
    {.thr_near = 0, .thr_far = 0}, //ch0
    {.thr_near = 0, .thr_far = 0},
    {.thr_near = 0, .thr_far = 0},
    {.thr_near = 0, .thr_far = 0},
};

//#define INT16_MAX = 32767           //#define INT16_MIN = -32768
static int16_t hx9031a_alg_ch_thres_drift_min[4] = {2000, -8000, -32768, 2000};
static int16_t hx9031a_alg_ch_thres_drift_max[4] = {32767, -2000, -8000, 32767};
static int16_t hx9031a_alg_ch_thres_drift_offset[4] = {700, 600, 4000, 500};

static int hx9031a_alg_ref_ch_drdy_int_en(uint8_t ch_id, uint8_t en);
static int hx9031a_alg_dynamic_threshold_init(uint8_t ref_ch_id);
static int hx9031a_alg_dynamic_threshold_adjust(uint8_t ref_ch_id);
#endif

//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^通用读写 START
//从指定起始寄存器开始，连续写入count个值
static int linux_common_i2c_write(struct i2c_client *client, uint8_t *reg_addr, uint8_t *txbuf, int count)
{
    int ret = -1;
    int ii = 0;
    uint8_t buf[HX9031A_MAX_XFER_SIZE + 1] = {0};
    struct i2c_msg msg[1];

    if(count > HX9031A_MAX_XFER_SIZE) {
        count = HX9031A_MAX_XFER_SIZE;
        PRINT_ERR("block write over size!!!\n");
    }
    buf[0] = *reg_addr;
    memcpy(buf + 1, txbuf, count);

    msg[0].addr = client->addr;
    msg[0].flags = 0;//write
    msg[0].len = count + 1;
    msg[0].buf = buf;

    ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
    if (ARRAY_SIZE(msg) != ret) {
        PRINT_ERR("linux_common_i2c_write failed. ret=%d\n", ret);
        ret = -1;
        for(ii = 0; ii < msg[0].len; ii++) {
            PRINT_ERR("msg[0].addr=0x%04X, msg[0].flags=0x%04X, msg[0].len=0x%04X, msg[0].buf[%02d]=0x%02X\n",
                      msg[0].addr,
                      msg[0].flags,
                      msg[0].len,
                      ii,
                      msg[0].buf[ii]);
        }
    } else {
        ret = 0;
    }

    return ret;
}

//从指定起始寄存器开始，连续读取count个值
static int linux_common_i2c_read(struct i2c_client *client, uint8_t *reg_addr, uint8_t *rxbuf, int count)
{
    int ret = -1;
    int ii = 0;
    struct i2c_msg msg[2];

    if(count > HX9031A_MAX_XFER_SIZE) {
        count = HX9031A_MAX_XFER_SIZE;
        PRINT_ERR("block read over size!!!\n");
    }

    msg[0].addr = client->addr;
    msg[0].flags = 0;//write
    msg[0].len = 1;
    msg[0].buf = reg_addr;

    msg[1].addr = client->addr;
    msg[1].flags = I2C_M_RD;//read
    msg[1].len = count;
    msg[1].buf = rxbuf;

    ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
    if (ARRAY_SIZE(msg) != ret) {
        PRINT_ERR("linux_common_i2c_read failed. ret=%d\n", ret);
        ret = -1;
        PRINT_ERR("msg[0].addr=0x%04X, msg[0].flags=0x%04X, msg[0].len=0x%04X, msg[0].buf[0]=0x%02X\n",
                  msg[0].addr,
                  msg[0].flags,
                  msg[0].len,
                  msg[0].buf[0]);
        if(msg[1].len >= 1) {
            for(ii = 0; ii < msg[1].len; ii++) {
                PRINT_ERR("msg[1].addr=0x%04X, msg[1].flags=0x%04X, msg[1].len=0x%04X, msg[1].buf[%02d]=0x%02X\n",
                          msg[1].addr,
                          msg[1].flags,
                          msg[1].len,
                          ii,
                          msg[1].buf[ii]);
            }
        }
    } else {
        ret = 0;
    }

    return ret;
}

//return 0 for success, return -1 for errors.
static int hx9031a_read(uint8_t addr, uint8_t *rxbuf, int count)
{
    int ret = -1;

    mutex_lock(&hx9031a_i2c_rw_mutex);
    ret = linux_common_i2c_read(hx9031a_i2c_client, &addr, rxbuf, count);
    if(0 != ret) {
        PRINT_ERR("linux_common_i2c_read failed\n");
        goto exit;
    }

exit:
    mutex_unlock(&hx9031a_i2c_rw_mutex);
    return ret;
}

//return 0 for success, return -1 for errors.
static int hx9031a_write(uint8_t addr, uint8_t *txbuf, int count)
{
    int ret = -1;

    mutex_lock(&hx9031a_i2c_rw_mutex);
    ret = linux_common_i2c_write(hx9031a_i2c_client, &addr, txbuf, count);
    if(0 != ret) {
        PRINT_ERR("linux_common_i2c_write failed\n");
        goto exit;
    }

exit:
    mutex_unlock(&hx9031a_i2c_rw_mutex);
    return ret;
}
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^通用读写 END

static void hx9031a_disable_irq(unsigned int irq)
{
    if(0 == irq) {
        PRINT_ERR("wrong irq number!\n");
        return;
    }

    if(1 == hx9031a_irq_en_flag) {
        disable_irq_nosync(hx9031a_pdata.irq);
        hx9031a_irq_en_flag = 0;
        PRINT_DBG("irq_%d is disabled!\n", irq);
    } else {
        PRINT_ERR("irq_%d is disabled already!\n", irq);
    }
}

static void hx9031a_enable_irq(unsigned int irq)
{
    if(0 == irq) {
        PRINT_ERR("wrong irq number!\n");
        return;
    }

    if(0 == hx9031a_irq_en_flag) {
        enable_irq(hx9031a_pdata.irq);
        hx9031a_irq_en_flag = 1;
        PRINT_DBG("irq_%d is enabled!\n", irq);
    } else {
        PRINT_ERR("irq_%d is enabled already!\n", irq);
    }
}

static void hx9031a_data_lock(uint8_t lock_flag)
{
    int ret = -1;
    uint8_t rx_buf[1] = {0};

    if(HX9031A_DATA_LOCK == lock_flag) {
        ret = hx9031a_read(RA_DSP_CONFIG_CTRL1_0xc8, rx_buf, 1);
        if(0 != ret) {
            PRINT_ERR("hx9031a_read failed\n");
        }

        //rx_buf[0] = rx_buf[0] | 0x10;
        rx_buf[0] = 0x10;

        ret = hx9031a_write(RA_DSP_CONFIG_CTRL1_0xc8, rx_buf, 1);
        if(0 != ret) {
            PRINT_ERR("hx9031a_write failed\n");
        }
    } else if(HX9031A_DATA_UNLOCK == lock_flag) {
        ret = hx9031a_read(RA_DSP_CONFIG_CTRL1_0xc8, rx_buf, 1);
        if(0 != ret) {
            PRINT_ERR("hx9031a_read failed\n");
        }

        //rx_buf[0] = rx_buf[0] & 0xEF;
        rx_buf[0] = 0x00;

        ret = hx9031a_write(RA_DSP_CONFIG_CTRL1_0xc8, rx_buf, 1);
        if(0 != ret) {
            PRINT_ERR("hx9031a_write failed\n");
        }
    } else {
        PRINT_ERR("ERROR!!! hx9031a_data_lock wrong para. now do data unlock!\n");
        ret = hx9031a_read(RA_DSP_CONFIG_CTRL1_0xc8, rx_buf, 1);
        if(0 != ret) {
            PRINT_ERR("hx9031a_read failed\n");
        }

        //rx_buf[0] = rx_buf[0] & 0xEF;
        rx_buf[0] = 0x00;

        ret = hx9031a_write(RA_DSP_CONFIG_CTRL1_0xc8, rx_buf, 1);
        if(0 != ret) {
            PRINT_ERR("hx9031a_write failed\n");
        }
    }
}

static int hx9031a_id_check(void)
{
    int ret = -1;
    int ii = 0;
    uint8_t device_id = 0;
    uint8_t rxbuf[1] = {0};

    for(ii = 0; ii < HX9031A_ID_CHECK_COUNT; ii++) {
        ret = hx9031a_read(RA_ID_REG_0x60, rxbuf, 1);
        if(ret < 0) {
            PRINT_ERR("i2c read error\n");
            continue;
        }
        device_id = rxbuf[0];
        if((HX90XX_CHIP_ID_9031A == device_id))
            break;
        else
            continue;
    }

    if(HX90XX_CHIP_ID_9031A == device_id) {
        PRINT_INF("success! device_id=0x%02X(HX9023E or HX9031A)\n", device_id);
        return 0;
    } else {
        PRINT_ERR("failed! device_id=0x%02X(UNKNOW_CHIP_ID)\n", device_id);
        return -1;
    }
}

//TODO: 功能：芯片和通道兼容性配置。客户必须自行填充此函数！
//return HX9031A_ON_BOARD：表示贴片为9031A
//return HX9023E_ON_BOARD：表示贴片为9023E
static uint8_t hx9031a_get_board_info(void)
{
    ENTER;

    //TODO: 获取board id
    return HX9023E_ON_BOARD;
}

static void hx9031a_ch_cfg(uint8_t chip_select)
{
    int ret = -1;
    int ii = 0;
    uint16_t ch_cfg = 0;
    uint8_t cfg[HX9031A_CH_NUM * 2] = {0};

    uint8_t CS0 = 0;
    uint8_t CS1 = 0;
    uint8_t CS2 = 0;
    uint8_t CS3 = 0;
    uint8_t CS4 = 0;
    uint8_t NA = 0;
    uint8_t CH0_POS = 0;
    uint8_t CH0_NEG = 0;
    uint8_t CH1_POS = 0;
    uint8_t CH1_NEG = 0;
    uint8_t CH2_POS = 0;
    uint8_t CH2_NEG = 0;
    uint8_t CH3_POS = 0;
    uint8_t CH3_NEG = 0;

    ENTER;

    //CS引脚配置在寄存器列表中的映射关系：这个映射表存在的目的是CS号在寄存器列表中并没有顺序排列 ┓(´∀`)┏
    if(HX9023E_ON_BOARD == chip_select) {
        CS0 = 0;
        CS1 = 1;
        CS2 = 2;
        CS3 = 3;
        CS4 = 4;
        NA = 10;
        PRINT_INF("HX9023E_ON_BOARD\n");
    } else if(HX9031A_ON_BOARD == chip_select) {
        CS0 = 2;
        CS1 = 1;
        CS2 = 3;
        CS3 = 0;
        CS4 = 4;
        NA = 10;
        PRINT_INF("HX9031A_ON_BOARD\n");
    } else {
        CS0 = 2;
        CS1 = 1;
        CS2 = 3;
        CS3 = 0;
        CS4 = 4;
        NA = 10;
        PRINT_ERR("unknow chip on board. set hx9031a by default\n");
    }

    //TODO:通道客制化配置开始 =================================================
    //每个通道的正负极都可以连接到任何一个CS。按需配置。
    if(HX9023E_ON_BOARD == chip_select) {
        CH0_POS = CS0;
        CH0_NEG = CS2;
        CH1_POS = CS2;
        CH1_NEG = NA;
        CH2_POS = NA;
        CH2_NEG = NA;
        CH3_POS = NA;
        CH3_NEG = NA;
    } else if(HX9031A_ON_BOARD == chip_select) {
        CH0_POS = CS0;
        CH0_NEG = CS1;
        CH1_POS = CS2;
        CH1_NEG = NA;
        CH2_POS = CS3;
        CH2_NEG = CS4;
        CH3_POS = NA;
        CH3_NEG = NA;
    } else {
        CH0_POS = CS1;
        CH0_NEG = CS0;
        CH1_POS = CS2;
        CH1_NEG = CS4;
        CH2_POS = CS3;
        CH2_NEG = CS4;
        CH3_POS = CS4;
        CH3_NEG = NA;
        PRINT_ERR("unknow chip on board. set hx9031a by default\n");
    }
    //TODO:通道客制化配置结束 ===============================================

    ch_cfg = (uint16_t)((0x03 << (CH0_POS * 2)) + (0x02 << (CH0_NEG * 2)));
    cfg[ii++] = (uint8_t)(ch_cfg);
    cfg[ii++] = (uint8_t)(ch_cfg >> 8);

    ch_cfg = (uint16_t)((0x03 << (CH1_POS * 2)) + (0x02 << (CH1_NEG * 2)));
    cfg[ii++] = (uint8_t)(ch_cfg);
    cfg[ii++] = (uint8_t)(ch_cfg >> 8);

    ch_cfg = (uint16_t)((0x03 << (CH2_POS * 2)) + (0x02 << (CH2_NEG * 2)));
    cfg[ii++] = (uint8_t)(ch_cfg);
    cfg[ii++] = (uint8_t)(ch_cfg >> 8);

    ch_cfg = (uint16_t)((0x03 << (CH3_POS * 2)) + (0x02 << (CH3_NEG * 2)));
    cfg[ii++] = (uint8_t)(ch_cfg);
    cfg[ii++] = (uint8_t)(ch_cfg >> 8);

    ret = hx9031a_write(RA_CH0_CFG_7_0_0x03, cfg, HX9031A_CH_NUM * 2);
    if(0 != ret) {
        PRINT_ERR("hx9031a_write failed\n");
    }
}

static void hx9031a_ch_thres_array_redirect(uint8_t chip_select)
{
    ENTER;
    if(HX9023E_ON_BOARD == chip_select) {
        hx9031a_ch_thres = hx9031a_ch_thres_default_hx9023e;
    } else if(HX9031A_ON_BOARD == chip_select) {
        hx9031a_ch_thres = hx9031a_ch_thres_default_hx9031a;
    } else {
        hx9031a_ch_thres = hx9031a_ch_thres_default_hx9031a;
        PRINT_ERR("unknow chip on board. set hx9031a by default\n");
    }
}

static void hx9031a_reg_list_redirect(uint8_t chip_select)
{
    ENTER;
    if(HX9023E_ON_BOARD == chip_select) {
        hx9031a_reg_init_list_size = ARRAY_SIZE(hx9031a_reg_init_list_hx9023e);
        hx9031a_reg_init_list = hx9031a_reg_init_list_hx9023e;
    } else if(HX9031A_ON_BOARD == chip_select) {
        hx9031a_reg_init_list_size = ARRAY_SIZE(hx9031a_reg_init_list_hx9031a);
        hx9031a_reg_init_list = hx9031a_reg_init_list_hx9031a;
    } else {
        hx9031a_reg_init_list_size = ARRAY_SIZE(hx9031a_reg_init_list_hx9031a);
        hx9031a_reg_init_list = hx9031a_reg_init_list_hx9031a;
        PRINT_ERR("unknow chip on board. set hx9031a by default\n");
    }
}

static void hx9031a_register_init(void)
{
    int ii = 0;
    int ret = -1;

    while(ii < hx9031a_reg_init_list_size) {
        //PRINT_DBG("WRITE:Reg0x%02X=0x%02X\n", hx9031a_reg_init_list[ii].addr, hx9031a_reg_init_list[ii].val);
        ret = hx9031a_write(hx9031a_reg_init_list[ii].addr, &hx9031a_reg_init_list[ii].val, 1);
        if(0 != ret) {
            PRINT_ERR("hx9031a_write failed\n");
        }

        if(hx9031a_monitor_addr == hx9031a_reg_init_list[ii].addr)
            hx9031a_monitor_val = hx9031a_reg_init_list[ii].val;

        ii++;
    }
}

static void hx9031a_read_offset_dac(void)
{
    int ret = -1;
    int ii = 0;
    uint8_t bytes_per_channel = 0;
    uint8_t bytes_all_channels = 0;
    uint8_t rx_buf[HX9031A_CH_NUM * CH_DATA_BYTES_MAX] = {0};
    uint32_t data[HX9031A_CH_NUM] = {0};

    hx9031a_data_lock(HX9031A_DATA_LOCK);
    bytes_per_channel = CH_DATA_2BYTES;
    bytes_all_channels = HX9031A_CH_NUM * bytes_per_channel;
    ret = hx9031a_read(RA_OFFSET_DAC0_7_0_0x15, rx_buf, bytes_all_channels);
    if(0 == ret) {
        for(ii = 0; ii < HX9031A_CH_NUM; ii++) {
            data[ii] = ((rx_buf[ii * bytes_per_channel + 1] << 8) | (rx_buf[ii * bytes_per_channel]));
            data[ii] = data[ii] & 0xFFF;//12位
            data_offset_dac[ii] = data[ii];
        }
    }
    hx9031a_data_lock(HX9031A_DATA_UNLOCK);

    PRINT_DBG("OFFSET_DAC, %-8d, %-8d, %-8d, %-8d\n", data_offset_dac[0], data_offset_dac[1], data_offset_dac[2], data_offset_dac[3]);
}

static void hx9031a_manual_offset_calibration(uint8_t ch_id)//手动校准指定的某个channel一次
{
    int ret = 0;
    uint8_t buf[1] = {0};

    mutex_lock(&hx9031a_cali_mutex);
    ret = hx9031a_read(RA_OFFSET_CALI_CTRL_0xc2, buf, 1);
    if(0 != ret) {
        PRINT_ERR("hx9031a_read failed\n");
    }

    buf[0] |= (1 << (ch_id + 4));
    ret = hx9031a_write(RA_OFFSET_CALI_CTRL_0xc2, buf, 1);
    if(0 != ret) {
        PRINT_ERR("hx9031a_write failed\n");
    }
    PRINT_INF("ch_%d will calibrate in next convert cycle (ODR=%dms)\n", ch_id, HX9031A_ODR_MS);//因此不能在一个ord周期内连续两次调用本函数！
    msleep(HX9031A_ODR_MS);
    mutex_unlock(&hx9031a_cali_mutex);
}

#if 0
static void hx9031a_manual_offset_calibration_all_chs()//手动校准所有channel
{
    int ret = 0;
    uint8_t buf[1] = {0};

    mutex_lock(&hx9031a_cali_mutex);
    ret = hx9031a_read(RA_OFFSET_CALI_CTRL_0xc2, buf, 1);
    if(0 != ret) {
        PRINT_ERR("hx9031a_read failed\n");
    }

    buf[0] |= 0xF0;
    ret = hx9031a_write(RA_OFFSET_CALI_CTRL_0xc2, buf, 1);
    if(0 != ret) {
        PRINT_ERR("hx9031a_write failed\n");
    }
    PRINT_INF("all channels will calibrate in next convert cycle (ODR=%dms)\n", HX9031A_ODR_MS);//因此不能在一个ord周期内连续两次调用本函数！
    msleep(HX9031A_ODR_MS);
    mutex_unlock(&hx9031a_cali_mutex);
}
#endif

#if 0 //不需要清除，因为是脉冲式中断（可以配置为电平）
static void hx9031a_int_clear(void)
{
    int ret = -1;
    uint8_t buf[1] = {0};

    ENTER;
    ret = hx9031a_read(RA_GLOBAL_CTRL0_0x00, buf, 1);
    if(0 != ret) {
        PRINT_ERR("hx9031a_read failed\n");
    }

    buf[0] |= 0x10;
    ret = hx9031a_write(RA_GLOBAL_CTRL0_0x00, buf, 1);
    if(0 != ret) {
        PRINT_ERR("hx9031a_write failed\n");
    }
}
#endif

static int16_t hx9031a_get_thres_near(uint8_t ch)
{
    int ret = 0;
    uint8_t buf[2] = {0};

    ret = hx9031a_read(RA_PROX_HIGH_DIFF_CFG_CH0_0_0x80 + (ch * CH_DATA_2BYTES), buf, 2);
    if(0 != ret) {
        PRINT_ERR("hx9031a_read failed\n");
    }

    hx9031a_ch_thres[ch].thr_near = (buf[0] + ((buf[1] & 0x03) << 8)) * 32;
    return hx9031a_ch_thres[ch].thr_near;
}
static int16_t hx9031a_get_thres_far(uint8_t ch)
{
    int ret = 0;
    uint8_t buf[2] = {0};

    ret = hx9031a_read(RA_PROX_LOW_DIFF_CFG_CH0_0_0x88 + (ch * CH_DATA_2BYTES), buf, 2);
    if(0 != ret) {
        PRINT_ERR("hx9031a_read failed\n");
    }

    hx9031a_ch_thres[ch].thr_far = (buf[0] + ((buf[1] & 0x03) << 8)) * 32;
    return hx9031a_ch_thres[ch].thr_far;
}

static int16_t hx9031a_set_thres_near(uint8_t ch, int16_t val)
{
    int ret = -1;
    uint8_t buf[2];

    val /= 32;
    buf[0] = val & 0xFF;
    buf[1] = (val >> 8) & 0x03;
    hx9031a_ch_thres[ch].thr_near = (val & 0x03FF) * 32;
    ret = hx9031a_write(RA_PROX_HIGH_DIFF_CFG_CH0_0_0x80 + (ch * CH_DATA_2BYTES), buf, 2);//10位！！！
    if(0 != ret) {
        PRINT_ERR("hx9031a_write failed\n");
    }

    PRINT_INF("hx9031a_ch_thres[%d].thr_near=%d\n", ch, hx9031a_ch_thres[ch].thr_near);
    return hx9031a_ch_thres[ch].thr_near;
}

static int16_t hx9031a_set_thres_far(uint8_t ch, int16_t val)
{
    int ret = -1;
    uint8_t buf[2];

    val /= 32;
    buf[0] = val & 0xFF;
    buf[1] = (val >> 8) & 0x03;
    hx9031a_ch_thres[ch].thr_far = (val & 0x03FF) * 32;
    ret = hx9031a_write(RA_PROX_LOW_DIFF_CFG_CH0_0_0x88 + (ch * CH_DATA_2BYTES), buf, 2);//10位！！！
    if(0 != ret) {
        PRINT_ERR("hx9031a_write failed\n");
    }

    PRINT_INF("hx9031a_ch_thres[%d].thr_far=%d\n", ch, hx9031a_ch_thres[ch].thr_far);
    return hx9031a_ch_thres[ch].thr_far;
}

static void hx9031a_get_prox_state(int32_t *data_compare, uint8_t ch_num)
{
    int ret = -1;
    int ii = 0;
    int16_t near = 0;
    int16_t far = 0;
    uint8_t buf[1] = {0};

    hx9031a_pdata.prox_state_reg = 0;
    hx9031a_pdata.prox_state_cmp = 0;
    for(ii = 0; ii < ch_num; ii++) {
        hx9031a_ch_near_state[ii] = 0;
    }

    ret = hx9031a_read(RA_PROX_STATUS_0x6b, buf, 1);
    if(0 != ret) {
        PRINT_ERR("hx9031a_read failed\n");
    }
    hx9031a_pdata.prox_state_reg = buf[0];

    for(ii = 0; ii < ch_num; ii++) {
        near = hx9031a_get_thres_near(ii);
        far = hx9031a_get_thres_far(ii);
        if(data_compare[ii] >= near) {
            hx9031a_ch_near_state[ii] = 1;//near
            hx9031a_pdata.prox_state_cmp |= (1 << ii);
        } else if(data_compare[ii] <= far) {
            hx9031a_ch_near_state[ii] = 0;//far
        }
    }

    //====================================================================
    for(ii = 0; ii < HX9031A_CH_NUM; ii++) { //标记每个通道由远离变为接近
        if((0 == ((hx9031a_pdata.prox_state_reg_pre >> ii) & 0x1)) && (1 == ((hx9031a_pdata.prox_state_reg >> ii) & 0x1))) {
            hx9031a_pdata.prox_state_far_to_near_flag |= (1 << ii);
        } else {
            hx9031a_pdata.prox_state_far_to_near_flag &= ~(1 << ii);
        }
    }
    hx9031a_pdata.prox_state_reg_pre = hx9031a_pdata.prox_state_reg;//保存上次的接近状态
    hx9031a_pdata.prox_state_cmp_pre = hx9031a_pdata.prox_state_cmp;//保存上次的接近状态
    //====================================================================

    PRINT_INF("prox_state_reg=0x%02X, prox_state_cmp=0x%02X, hx9031a_ch_near_state:%d-%d-%d-%d, far_to_near_flag=0x%02X\n",
              hx9031a_pdata.prox_state_reg,
              hx9031a_pdata.prox_state_cmp,
              hx9031a_ch_near_state[3],
              hx9031a_ch_near_state[2],
              hx9031a_ch_near_state[1],
              hx9031a_ch_near_state[0],
              hx9031a_pdata.prox_state_far_to_near_flag);
}

//因为raw data 和bl data共用0xE8寄存器，lp data和diff data共用0xF4寄存器，该函数用来切换输出数据。
static void hx9031a_data_switch(uint8_t output_switch)
{
    int ret = -1;
    uint8_t buf[1] = {0};

    //7:4 raw(1:bl 0:raw) 3:0 diff(1:diff 0:lp)
    if(HX9031A_OUTPUT_RAW_DIFF == output_switch) { //raw & diff
        hx9031a_output_switch = HX9031A_OUTPUT_RAW_DIFF;
        buf[0] = 0x0F;
        PRINT_INF("output data: raw & diff\n");
    } else { //lp & bl
        hx9031a_output_switch = HX9031A_OUTPUT_LP_BL;
        buf[0] = 0xF0;
        PRINT_INF("output data: lp & bl\n");
    }

    ret = hx9031a_write(RA_RAW_BL_RD_CFG_0x38, buf, 1);
    if(0 != ret) {
        PRINT_ERR("hx9031a_write failed\n");
    }
}

static void hx9031a_sample_16(void)//输出raw bl diff lp offse五组数据，默认diff和raw (diff=lp-bl)
{
    int ret = -1;
    int ii = 0;
    uint8_t bytes_per_channel = 0;
    uint8_t bytes_all_channels = 0;
    uint8_t rx_buf[HX9031A_CH_NUM * CH_DATA_BYTES_MAX] = {0};//设置为可能的最大buf
    int32_t data[HX9031A_CH_NUM] = {0};

    hx9031a_data_lock(HX9031A_DATA_LOCK);
    //====================================================================================================
    bytes_per_channel = CH_DATA_3BYTES;
    bytes_all_channels = HX9031A_CH_NUM * bytes_per_channel;
    ret = hx9031a_read(RA_RAW_BL_CH0_0_0xe8, rx_buf, bytes_all_channels);
    if(0 == ret) {
        for(ii = 0; ii < HX9031A_CH_NUM; ii++) {
            data[ii] = ((rx_buf[ii * bytes_per_channel + 2] << 8) | (rx_buf[ii * bytes_per_channel + 1]));//24位数据舍去低8位，保留高16位，下同
            if(HX9031A_OUTPUT_RAW_DIFF == hx9031a_output_switch) {
                data_raw[ii] = (data[ii] > 0x7FFF) ? (data[ii] - (0xFFFF + 1)) : data[ii]; //补码转换为原码
                data_bl[ii] = 0;
            } else {
                data_raw[ii] = 0;
                data_bl[ii] = (data[ii] > 0x7FFF) ? (data[ii] - (0xFFFF + 1)) : data[ii]; //补码转换为原码
            }
        }
    }
    //====================================================================================================
    bytes_per_channel = CH_DATA_3BYTES;
    bytes_all_channels = HX9031A_CH_NUM * bytes_per_channel;
    ret = hx9031a_read(RA_LP_DIFF_CH0_0_0xf4, rx_buf, bytes_all_channels);
    if(0 == ret) {
        for(ii = 0; ii < HX9031A_CH_NUM; ii++) {
            data[ii] = ((rx_buf[ii * bytes_per_channel + 2] << 8) | (rx_buf[ii * bytes_per_channel + 1]));
            if(HX9031A_OUTPUT_RAW_DIFF == hx9031a_output_switch) {
                data_diff[ii] = (data[ii] > 0x7FFF) ? (data[ii] - (0xFFFF + 1)) : data[ii]; //补码转换为原码
                data_lp[ii] = 0;
            } else {
                data_lp[ii] = (data[ii] > 0x7FFF) ? (data[ii] - (0xFFFF + 1)) : data[ii]; //补码转换为原码
                data_diff[ii] = data_lp[ii] - data_bl[ii];
            }
        }
    }

    if(HX9031A_OUTPUT_LP_BL == hx9031a_output_switch) {
        for(ii = 0; ii < HX9031A_CH_NUM; ii++) {
            data_diff[ii] = data_lp[ii] - data_bl[ii];
        }
    }
    //====================================================================================================
    bytes_per_channel = CH_DATA_2BYTES;
    bytes_all_channels = HX9031A_CH_NUM * bytes_per_channel;
    ret = hx9031a_read(RA_OFFSET_DAC0_7_0_0x15, rx_buf, bytes_all_channels);
    if(0 == ret) {
        for(ii = 0; ii < HX9031A_CH_NUM; ii++) {
            data[ii] = ((rx_buf[ii * bytes_per_channel + 1] << 8) | (rx_buf[ii * bytes_per_channel]));
            data[ii] = data[ii] & 0xFFF;//12位
            data_offset_dac[ii] = data[ii];
        }
    }
    //====================================================================================================
    hx9031a_data_lock(HX9031A_DATA_UNLOCK);

    PRINT_DBG("DIFF  , %-8d, %-8d, %-8d, %-8d\n", data_diff[0], data_diff[1], data_diff[2], data_diff[3]);
    PRINT_DBG("RAW   , %-8d, %-8d, %-8d, %-8d\n", data_raw[0], data_raw[1], data_raw[2], data_raw[3]);
    PRINT_DBG("OFFSET, %-8d, %-8d, %-8d, %-8d\n", data_offset_dac[0], data_offset_dac[1], data_offset_dac[2], data_offset_dac[3]);
    PRINT_DBG("BL    , %-8d, %-8d, %-8d, %-8d\n", data_bl[0], data_bl[1], data_bl[2], data_bl[3]);
    PRINT_DBG("LP    , %-8d, %-8d, %-8d, %-8d\n", data_lp[0], data_lp[1], data_lp[2], data_lp[3]);
}

static void hx9031a_sample_24(void)//输出raw bl diff lp offse五组数据，默认diff和raw (diff=lp-bl)
{
    int ret = -1;
    int ii = 0;
    uint8_t bytes_per_channel = 0;
    uint8_t bytes_all_channels = 0;
    uint8_t rx_buf[HX9031A_CH_NUM * CH_DATA_BYTES_MAX] = {0};//设置为可能的最大buf
    int32_t data[HX9031A_CH_NUM] = {0};

    hx9031a_data_lock(HX9031A_DATA_LOCK);
    //====================================================================================================
    bytes_per_channel = CH_DATA_3BYTES;
    bytes_all_channels = HX9031A_CH_NUM * bytes_per_channel;
    ret = hx9031a_read(RA_RAW_BL_CH0_0_0xe8, rx_buf, bytes_all_channels);
    if(0 == ret) {
        for(ii = 0; ii < HX9031A_CH_NUM; ii++) {
            data[ii] = ((rx_buf[ii * bytes_per_channel + 2] << 16) | (rx_buf[ii * bytes_per_channel + 1] << 8) | (rx_buf[ii * bytes_per_channel]));
            if(HX9031A_OUTPUT_RAW_DIFF == hx9031a_output_switch) {
                data_raw[ii] = (data[ii] > 0x7FFFFF) ? (data[ii] - (0xFFFFFF + 1)) : data[ii]; //补码转换为原码
                data_bl[ii] = 0;
            } else {
                data_raw[ii] = 0;
                data_bl[ii] = (data[ii] > 0x7FFFFF) ? (data[ii] - (0xFFFFFF + 1)) : data[ii]; //补码转换为原码
            }
        }
    }
    //====================================================================================================
    bytes_per_channel = CH_DATA_3BYTES;
    bytes_all_channels = HX9031A_CH_NUM * bytes_per_channel;
    ret = hx9031a_read(RA_LP_DIFF_CH0_0_0xf4, rx_buf, bytes_all_channels);
    if(0 == ret) {
        for(ii = 0; ii < HX9031A_CH_NUM; ii++) {
            data[ii] = ((rx_buf[ii * bytes_per_channel + 2] << 16) | (rx_buf[ii * bytes_per_channel + 1] << 8) | (rx_buf[ii * bytes_per_channel]));
            if(HX9031A_OUTPUT_RAW_DIFF == hx9031a_output_switch) {
                data_diff[ii] = (data[ii] > 0x7FFFFF) ? (data[ii] - (0xFFFFFF + 1)) : data[ii]; //补码转换为原码
                data_lp[ii] = 0;
            } else {
                data_diff[ii] = 0;
                data_lp[ii] = (data[ii] > 0x7FFFFF) ? (data[ii] - (0xFFFFFF + 1)) : data[ii]; //补码转换为原码
            }
        }
    }

    if(HX9031A_OUTPUT_LP_BL == hx9031a_output_switch) {
        for(ii = 0; ii < HX9031A_CH_NUM; ii++) {
            data_diff[ii] = data_lp[ii] - data_bl[ii];
        }
    }
    //====================================================================================================
    bytes_per_channel = CH_DATA_2BYTES;
    bytes_all_channels = HX9031A_CH_NUM * bytes_per_channel;
    ret = hx9031a_read(RA_OFFSET_DAC0_7_0_0x15, rx_buf, bytes_all_channels);
    if(0 == ret) {
        for(ii = 0; ii < HX9031A_CH_NUM; ii++) {
            data[ii] = ((rx_buf[ii * bytes_per_channel + 1] << 8) | (rx_buf[ii * bytes_per_channel]));
            data[ii] = data[ii] & 0xFFF;//12位
            data_offset_dac[ii] = data[ii];
        }
    }
    //====================================================================================================
    hx9031a_data_lock(HX9031A_DATA_UNLOCK);

    PRINT_DBG("DIFF  , %-8d, %-8d, %-8d, %-8d\n", data_diff[0], data_diff[1], data_diff[2], data_diff[3]);
    PRINT_DBG("RAW   , %-8d, %-8d, %-8d, %-8d\n", data_raw[0], data_raw[1], data_raw[2], data_raw[3]);
    PRINT_DBG("OFFSET, %-8d, %-8d, %-8d, %-8d\n", data_offset_dac[0], data_offset_dac[1], data_offset_dac[2], data_offset_dac[3]);
    PRINT_DBG("BL    , %-8d, %-8d, %-8d, %-8d\n", data_bl[0], data_bl[1], data_bl[2], data_bl[3]);
    PRINT_DBG("LP    , %-8d, %-8d, %-8d, %-8d\n", data_lp[0], data_lp[1], data_lp[2], data_lp[3]);
}

static void hx9031a_sample(void)
{
    if(24 == hx9031a_data_accuracy) {
        hx9031a_sample_24();
    } else {
        hx9031a_sample_16();
    }
}

static void hx9031a_input_report(void)
{
    int ii = 0;
    uint8_t touch_state = 0;
    int num_channels = 0;
    struct input_dev *input = NULL;
    struct hx9031a_channel_info *channel_p  = NULL;

    num_channels = ARRAY_SIZE(hx9031a_channels);

    if (unlikely(NULL == hx9031a_channels)) {
        PRINT_ERR("hx9031a_channels==NULL!!!\n");
        return;
    }

    for (ii = 0; ii < num_channels; ii++) {
        if (false == hx9031a_channels[ii].enabled) { //enabled表示该通道已经被开启。
            PRINT_DBG("ch_%d(name:%s) is disabled, nothing report\n", ii, hx9031a_channels[ii].name);
            continue;
        }
        if (false == hx9031a_channels[ii].used) { //used表示通道有效，但此刻该通道并不一定enable
            PRINT_DBG("ch_%d(name:%s) is unused, nothing report\n", ii, hx9031a_channels[ii].name);
            continue;
        }

        channel_p = &hx9031a_channels[ii];
        if (NULL == channel_p) {
            PRINT_ERR("ch_%d is NULL!!!\n", ii);
            return;
        }

        input = channel_p->hx9031a_input_dev;
        if (NULL == input) {
            PRINT_ERR("ch_%d input device is NULL!!!\n", ii);
            return;
        }

        //上报数据时采用何种状态判断依据，以下3选1
        //touch_state = hx9031a_ch_near_state[ii];
        touch_state = (hx9031a_pdata.prox_state_reg >> ii) & 0x1;
        //touch_state = (hx9031a_pdata.prox_state_cmp >> ii) & 0x1;

        if (BODYACTIVE == touch_state) {
            if (channel_p->state == BODYACTIVE)
                PRINT_DBG("%s already BODYACTIVE, nothing report\n", channel_p->name);
            else {
/*
#ifdef CONFIG_PM_WAKELOCKS
                __pm_wakeup_event(&hx9031a_wake_lock, 1000);
#else
                wake_lock_timeout(&hx9031a_wake_lock, HZ * 1);
#endif
*/
                input_report_abs(input, ABS_DISTANCE, 2);
                input_sync(input);
                channel_p->state = BODYACTIVE;
                PRINT_DBG("%s report BODYACTIVE(5mm)\n", channel_p->name);
            }
        } else if (PROXACTIVE == touch_state) {
            if (channel_p->state == PROXACTIVE)
                PRINT_DBG("%s already PROXACTIVE, nothing report\n", channel_p->name);
            else {
/*
#ifdef CONFIG_PM_WAKELOCKS
                __pm_wakeup_event(&hx9031a_wake_lock, 1000);
#else
                wake_lock_timeout(&hx9031a_wake_lock, HZ * 1);
#endif
*/
                input_report_abs(input, ABS_DISTANCE, 1);
                input_sync(input);
                channel_p->state = PROXACTIVE;
                PRINT_DBG("%s report PROXACTIVE(15mm)\n", channel_p->name);
            }
        } else if (IDLE == touch_state) {
            if (channel_p->state == IDLE)
                PRINT_DBG("%s already released, nothing report\n", channel_p->name);
            else {
/*
#ifdef CONFIG_PM_WAKELOCKS
                __pm_wakeup_event(&hx9031a_wake_lock, 1000);
#else
                wake_lock_timeout(&hx9031a_wake_lock, HZ * 1);
#endif
*/
                input_report_abs(input, ABS_DISTANCE, 0);
                input_sync(input);
                channel_p->state = IDLE;
                PRINT_DBG("%s report released\n", channel_p->name);
            }
        } else {
            PRINT_ERR("unknow touch state! touch_state=%d\n", touch_state);
        }
    }
}

// 必要DTS属性：
// "tyhx,irq-gpio"     必要！中断对应的gpio number
// "tyhx,channel-flag"  必要！每个channel对应一个input设备，每个bit对应一个channel（channel）。如0xF代表开启0，1，2，3通道
static int hx9031a_parse_dt(struct device *dev)
{
    int ret = -1;
    struct device_node *dt_node = dev->of_node;

    if (NULL == dt_node) {
        PRINT_ERR("No DTS node\n");
        return -ENODEV;
    }

#if HX9031A_TEST_ON_MTK_DEMO_XY6761
    ret = of_property_read_u32(dt_node, "xy6761-fake-irq-gpio", &hx9031a_pdata.irq_gpio);
    if(ret < 0) {
        PRINT_ERR("failed to get irq_gpio from DT\n");
        return -1;
    }
#else
    hx9031a_pdata.irq_gpio = of_get_named_gpio_flags(dt_node, "tyhx,irq-gpio", 0, NULL);
    if(hx9031a_pdata.irq_gpio < 0) {
        PRINT_ERR("failed to get irq_gpio from DT\n");
        return -1;
    }
#endif

    PRINT_INF("hx9031a_pdata.irq_gpio=%d\n", hx9031a_pdata.irq_gpio);

    hx9031a_pdata.channel_used_flag = 0x0F;
    ret = of_property_read_u32(dt_node, "tyhx,channel-flag", &hx9031a_pdata.channel_used_flag);//客户配置：有效通道标志位：9031A最大传入0x0F，9023E最大传入0x07
    if(ret < 0) {
        PRINT_ERR("\"tyhx,channel-flag\" is not set in DT\n");
        return -1;
    }
    if(hx9031a_pdata.channel_used_flag > ((1 << HX9031A_CH_NUM) - 1)) {
        PRINT_ERR("the max value of channel_used_flag is 0x%X\n", ((1 << HX9031A_CH_NUM) - 1));
        return -1;
    }
    PRINT_INF("hx9031a_pdata.channel_used_flag=0x%X\n", hx9031a_pdata.channel_used_flag);

    return 0;
}

static int hx9031a_gpio_init(void)
{
    int ret = -1;
    if (gpio_is_valid(hx9031a_pdata.irq_gpio)) {
        ret = gpio_request(hx9031a_pdata.irq_gpio, "hx9031a_irq_gpio");
        if (ret < 0) {
            PRINT_ERR("gpio_request failed. ret=%d\n", ret);
            return ret;
        }
        ret = gpio_direction_input(hx9031a_pdata.irq_gpio);
        if (ret < 0) {
            PRINT_ERR("gpio_direction_input failed. ret=%d\n", ret);
            gpio_free(hx9031a_pdata.irq_gpio);
            return ret;
        }
    } else {
        PRINT_ERR("Invalid gpio num\n");
        return -1;
    }

    hx9031a_pdata.irq = gpio_to_irq(hx9031a_pdata.irq_gpio);
    PRINT_INF("hx9031a_pdata.irq_gpio=%d hx9031a_pdata.irq=%d\n", hx9031a_pdata.irq_gpio, hx9031a_pdata.irq);
    return 0;
}

static void hx9031a_gpio_deinit(void)
{
    ENTER;
    gpio_free(hx9031a_pdata.irq_gpio);
}

static void hx9031a_power_on(struct regulator *power_vdd, bool on)
{
	int error;
	if (power_vdd != NULL) {
		if(on) {
			error = regulator_enable(power_vdd);
			if (error)
				PRINT_ERR("hx9031a power on failed\n");
		} else {
			error = regulator_disable(power_vdd);
			if (error)
				PRINT_ERR("hx9031a power off failed\n");	
		}
	}
}

#if 0
static int hx9031a_threshold_int_en(uint8_t ch_id, uint8_t en)
{
    int ret = -1;
    uint8_t buf[1] = {0};

    ret = hx9031a_read(RA_INTERRUPT_CFG_0x39, buf, 1);
    if(0 != ret) {
        PRINT_ERR("hx9031a_read failed\n");
        return -1;
    }

    if(ch_id >= HX9031A_CH_NUM) {
        PRINT_ERR("channel index over range !!!(ch_id=%d, en=%d)\n", ch_id, en);
        return -1;
    }

    if(0 == en) {
        buf[0] &= ~(1 << (ch_id + 0));
        buf[0] &= ~(1 << (ch_id + 4));
        ret = hx9031a_write(RA_INTERRUPT_CFG_0x39, buf, 1);
        if(0 != ret) {
            PRINT_ERR("hx9031a_write failed\n");
            return -1;
        }
        PRINT_INF("ch_%d threshold int disabled\n", ch_id);
    } else {
        buf[0] |= (1 << (ch_id + 0));
        buf[0] |= (1 << (ch_id + 4));
        ret = hx9031a_write(RA_INTERRUPT_CFG_0x39, buf, 1);
        if(0 != ret) {
            PRINT_ERR("hx9031a_write failed\n");
            return -1;
        }
        PRINT_INF("ch_%d threshold int enabled\n", ch_id);
    }

    return 0;
}
#endif

#if HX9031A_TEST_CHS_EN //通道测试时用全开全关策略
static int hx9031a_ch_en(uint8_t ch_id, uint8_t en)
{
    int ret = 0;
    uint8_t tx_buf[1] = {0};

    if(ch_enable_status > 0) {
        if(1 == en) {
            ch_enable_status |= (1 << ch_id);
            //hx9031a_threshold_int_en(ch_id, en);
            PRINT_INF("ch_%d enabled\n", ch_id);
        } else {
            ch_enable_status &= ~(1 << ch_id);
            //hx9031a_threshold_int_en(ch_id, en);
            if(0 == ch_enable_status) {
                tx_buf[0] = 0x00;
                ret = hx9031a_write(RA_CH_NUM_CFG_0x24, tx_buf, 1);
                if(0 != ret) {
                    PRINT_ERR("hx9031a_write failed\n");
                    return -1;
                }
#if HX9031A_ALG_COMPILE_EN
                if(1 == hx9031a_alg_dynamic_threshold_en) {
                    hx9031a_alg_dynamic_threshold_init(3);
                }
#endif
                if(1 == hx9031a_hw_monitor_en_flag) {
                    cancel_delayed_work_sync(&(hx9031a_pdata.hw_monitor_work));
                }
                PRINT_INF("ch_%d disabled, all channels disabled\n", ch_id);
            } else {
                PRINT_INF("ch_%d disabled\n", ch_id);
            }
        }
    } else {
        if(1 == en) {
            hx9031a_pdata.prox_state_reg = 0;
            hx9031a_pdata.prox_state_cmp = 0;
            hx9031a_pdata.prox_state_reg_pre = 0;
            hx9031a_pdata.prox_state_cmp_pre = 0;
            hx9031a_pdata.prox_state_far_to_near_flag = 0;

#if HX9031A_ALG_COMPILE_EN
            if(1 == hx9031a_alg_dynamic_threshold_en) {
                hx9031a_alg_dynamic_threshold_init(3);
                tx_buf[0] = hx9031a_pdata.channel_used_flag;
                tx_buf[0] |= (0x1 << 3); //ch3 enabled
            } else {
                tx_buf[0] = hx9031a_pdata.channel_used_flag;
            }
#else
            tx_buf[0] = hx9031a_pdata.channel_used_flag;
#endif
            ret = hx9031a_write(RA_CH_NUM_CFG_0x24, tx_buf, 1);
            if(0 != ret) {
                PRINT_ERR("hx9031a_write failed\n");
                return -1;
            }
            ch_enable_status |= (1 << ch_id);
            //hx9031a_threshold_int_en(ch_id, en);

            if(1 == hx9031a_hw_monitor_en_flag) {
                schedule_delayed_work(&hx9031a_pdata.hw_monitor_work, msecs_to_jiffies(10000));
            }

            PRINT_INF("ch_%d enabled\n", ch_id);
        } else {
            PRINT_INF("all channels disabled already\n");
        }
    }

    return ret;
}

#else

static int hx9031a_ch_en(uint8_t ch_id, uint8_t en)
{
    int ret = -1;
    uint8_t rx_buf[1] = {0};
    uint8_t tx_buf[1] = {0};

    ret = hx9031a_read(RA_CH_NUM_CFG_0x24, rx_buf, 1);
    if(0 != ret) {
        PRINT_ERR("hx9031a_read failed\n");
        return -1;
    }
    ch_enable_status = rx_buf[0];

    if(ch_id >= HX9031A_CH_NUM) {
        PRINT_ERR("channel index over range !!!ch_enable_status=0x%02X (ch_id=%d, en=%d)\n", ch_enable_status, ch_id, en);
        return -1;
    }

    if(1 == en) {
        if(0 == ch_enable_status) { //开启第一个ch
            hx9031a_pdata.prox_state_reg = 0;
            hx9031a_pdata.prox_state_cmp = 0;
            hx9031a_pdata.prox_state_reg_pre = 0;
            hx9031a_pdata.prox_state_cmp_pre = 0;
            hx9031a_pdata.prox_state_far_to_near_flag = 0;
#if HX9031A_ALG_COMPILE_EN
            if(1 == hx9031a_alg_dynamic_threshold_en) {
                hx9031a_alg_dynamic_threshold_init(3);
                ch_enable_status |= (1 << 3);//ch3 enabled
            }
#endif
            if(1 == hx9031a_hw_monitor_en_flag) {
                schedule_delayed_work(&hx9031a_pdata.hw_monitor_work, msecs_to_jiffies(10000));
            }
        }

        ch_enable_status |= (1 << ch_id);
        tx_buf[0] = ch_enable_status;
        ret = hx9031a_write(RA_CH_NUM_CFG_0x24, tx_buf, 1);
        if(0 != ret) {
            PRINT_ERR("hx9031a_write failed\n");
            return -1;
        }
        PRINT_INF("ch_enable_status=0x%02X (ch_%d=%d)\n", ch_enable_status, ch_id, en);
        msleep(10);
    } else {
        en = 0;
        ch_enable_status &= ~(1 << ch_id);
        if(0 == (ch_enable_status & 0x07)) { //关闭最后一个ch
#if HX9031A_ALG_COMPILE_EN
            if(1 == hx9031a_alg_dynamic_threshold_en) {
                hx9031a_alg_dynamic_threshold_init(3);
                ch_enable_status &= ~(1 << 3);//ch3 disabled
            }
#endif
            if(1 == hx9031a_hw_monitor_en_flag) {
                cancel_delayed_work_sync(&(hx9031a_pdata.hw_monitor_work));
            }
        }

        tx_buf[0] = ch_enable_status;
        ret = hx9031a_write(RA_CH_NUM_CFG_0x24, tx_buf, 1);
        if(0 != ret) {
            PRINT_ERR("hx9031a_write failed\n");
            return -1;
        }
        PRINT_INF("ch_enable_status=0x%02X (ch_%d=%d)\n", ch_enable_status, ch_id, en);
    }
    return 0;
}
#endif

static int hx9031a_set_enable(struct sensors_classdev *sensors_cdev, unsigned int enable)
{
    int ret = -1;
    int ii = 0;

    mutex_lock(&hx9031a_ch_en_mutex);
    for (ii = 0; ii < ARRAY_SIZE(hx9031a_channels); ii++) {
        if (strcmp(sensors_cdev->name, hx9031a_channels[ii].name) == 0) {
            if (1 == enable) {
                PRINT_INF("enable ch_%d(name:%s)\n", ii, sensors_cdev->name);
                ret = hx9031a_ch_en(ii, 1);
                if(0 != ret) {
                    PRINT_ERR("hx9031a_ch_en failed\n");
                    mutex_unlock(&hx9031a_ch_en_mutex);
                    return -1;
                }
                hx9031a_channels[ii].state = IDLE;
                hx9031a_channels[ii].enabled = true;
/*
#ifdef CONFIG_PM_WAKELOCKS
                __pm_wakeup_event(&hx9031a_wake_lock, 1000);
#else
                wake_lock_timeout(&hx9031a_wake_lock, HZ * 1);
#endif
*/
                input_report_abs(hx9031a_channels[ii].hx9031a_input_dev, ABS_DISTANCE, 0);
                input_sync(hx9031a_channels[ii].hx9031a_input_dev);
                //hx9031a_manual_offset_calibration(ii);//enable(en:0x24)的时候会自动校准，故此处不需要。
            } else if (0 == enable) {
                PRINT_INF("disable ch_%d(name:%s)\n", ii, sensors_cdev->name);
                ret = hx9031a_ch_en(ii, 0);
                if(0 != ret) {
                    PRINT_ERR("hx9031a_ch_en failed\n");
                    mutex_unlock(&hx9031a_ch_en_mutex);
                    return -1;
                }
                hx9031a_channels[ii].state = IDLE;
                hx9031a_channels[ii].enabled = false;
/*
#ifdef CONFIG_PM_WAKELOCKS
                __pm_wakeup_event(&hx9031a_wake_lock, 1000);
#else
                wake_lock_timeout(&hx9031a_wake_lock, HZ * 1);
#endif
*/
                input_report_abs(hx9031a_channels[ii].hx9031a_input_dev, ABS_DISTANCE, -1);
                input_sync(hx9031a_channels[ii].hx9031a_input_dev);
            } else {
                PRINT_ERR("unknown enable symbol\n");
            }
        }
    }
    mutex_unlock(&hx9031a_ch_en_mutex);

    return 0;
}

//alg start===========================================================================
#if HX9031A_ALG_COMPILE_EN
static int hx9031a_alg_ref_ch_drdy_int_en(uint8_t ch_id, uint8_t en)
{
    int ret = -1;
    uint8_t buf[1] = {0};

    ret = hx9031a_read(RA_CALI_DIFF_CFG_0x3b, buf, 1);
    if(0 != ret) {
        PRINT_ERR("hx9031a_read failed\n");
        return -1;
    }

    if(ch_id >= HX9031A_CH_NUM) {
        PRINT_ERR("channel index over range !!!(ch_id=%d, en=%d)\n", ch_id, en);
        return -1;
    }

    if(0 == en) {
        buf[0] &= ~(1 << (ch_id + 4));
        ret = hx9031a_write(RA_CALI_DIFF_CFG_0x3b, buf, 1);
        if(0 != ret) {
            PRINT_ERR("hx9031a_write failed\n");
            return -1;
        }
        PRINT_INF("ch_%d drdy int disabled by alg\n", ch_id);
    } else {
        buf[0] |= (1 << (ch_id + 4));
        ret = hx9031a_write(RA_CALI_DIFF_CFG_0x3b, buf, 1);
        if(0 != ret) {
            PRINT_ERR("hx9031a_write failed\n");
            return -1;
        }
        PRINT_INF("ch_%d drdy int enabled by alg\n", ch_id);
    }

    return 0;
}

static int hx9031a_alg_dynamic_threshold_init(uint8_t ref_ch_id)
{
    int ii = 0;

    ENTER;
    hx9031a_alg_ref_ch_drdy_int_en(ref_ch_id, 0);
    hx9031a_alg_ref_ch_drdy_int_en_flag = 0;
    for(ii = 0; ii < HX9031A_CH_NUM; ii++) {
        hx9031a_alg_ch_thres_ini[ii] = 0;
        hx9031a_alg_ch_thres_drift[ii] = 0;
        hx9031a_set_thres_near(ii, hx9031a_alg_ch_thres_backup[ii].thr_near);//恢复接近阈值
        hx9031a_set_thres_far(ii, hx9031a_alg_ch_thres_backup[ii].thr_far);//恢复远离阈值
        PRINT_INF("ch_%d threshold: near=%d, far=%d\n", ii, hx9031a_ch_thres[ii].thr_near, hx9031a_ch_thres[ii].thr_far);
    }
    return 0;
}

static int hx9031a_alg_dynamic_threshold_adjust(uint8_t ref_ch_id)//ch3 == ref_ch_id
{
    int ii = 0;

    ENTER;
    if((hx9031a_pdata.prox_state_far_to_near_flag & hx9031a_pdata.channel_used_flag) > 0) { //有某个通道由远离变为接近了！同时过滤掉未使用的通道
        if(0 == hx9031a_alg_ref_ch_drdy_int_en_flag) {//如果是第一次进来，则打开drdy中断，并标记
            hx9031a_alg_ref_ch_drdy_int_en(ref_ch_id, 1);
            hx9031a_alg_ref_ch_drdy_int_en_flag = 1;
        }

        for(ii = 0; ii < HX9031A_CH_NUM; ii++) {//由远离状态变为接近状态的通道要更新其ini值为当前diff3值
            if ((hx9031a_pdata.prox_state_far_to_near_flag >> ii) & 0x1) {
                hx9031a_alg_ch_thres_ini[ii] = data_diff[ref_ch_id];
            }
        }
    }

    if(1 == hx9031a_alg_ref_ch_drdy_int_en_flag) {
        for(ii = 0; ii < HX9031A_CH_NUM; ii++) {
            hx9031a_alg_ch_thres_drift[ii] = data_diff[ref_ch_id] - hx9031a_alg_ch_thres_ini[ii];
        }

        //更新阈值参数
        if((hx9031a_alg_ch_thres_drift[2] >= hx9031a_alg_ch_thres_drift_min[0])
           && (hx9031a_alg_ch_thres_drift[2] <= hx9031a_alg_ch_thres_drift_max[0])) {
            hx9031a_ch_thres[2].thr_near = hx9031a_alg_ch_thres_backup[2].thr_near + hx9031a_alg_ch_thres_drift_offset[0];
            hx9031a_ch_thres[2].thr_far = hx9031a_alg_ch_thres_backup[2].thr_far + hx9031a_alg_ch_thres_drift_offset[0];
        }

        if((hx9031a_alg_ch_thres_drift[1] >= hx9031a_alg_ch_thres_drift_min[1])
           && (hx9031a_alg_ch_thres_drift[1] <= hx9031a_alg_ch_thres_drift_max[1])) {
            hx9031a_ch_thres[1].thr_near = hx9031a_alg_ch_thres_backup[1].thr_near - hx9031a_alg_ch_thres_drift[1] / 2;
            hx9031a_ch_thres[1].thr_far = hx9031a_alg_ch_thres_backup[1].thr_far - hx9031a_alg_ch_thres_drift[1] / 2;
        }

        if((hx9031a_alg_ch_thres_drift[1] >= hx9031a_alg_ch_thres_drift_min[2])
           && (hx9031a_alg_ch_thres_drift[1] < hx9031a_alg_ch_thres_drift_max[2])) {
            hx9031a_ch_thres[1].thr_near = hx9031a_alg_ch_thres_backup[1].thr_near + hx9031a_alg_ch_thres_drift_offset[2];
            hx9031a_ch_thres[1].thr_far = hx9031a_alg_ch_thres_backup[1].thr_far + hx9031a_alg_ch_thres_drift_offset[2];
        }

        if((hx9031a_alg_ch_thres_drift[0] >= hx9031a_alg_ch_thres_drift_min[3])
           && (hx9031a_alg_ch_thres_drift[0] <= hx9031a_alg_ch_thres_drift_max[3])) {
            hx9031a_ch_thres[0].thr_near = hx9031a_alg_ch_thres_backup[0].thr_near + hx9031a_alg_ch_thres_drift_offset[3];
            hx9031a_ch_thres[0].thr_far = hx9031a_alg_ch_thres_backup[0].thr_far + hx9031a_alg_ch_thres_drift_offset[3];
        }

        for(ii = 0; ii < HX9031A_CH_NUM; ii++) {
            if ((hx9031a_pdata.prox_state_reg >> ii) & 0x1) {//如果该通道当前处于接近态，则更新该通道的阈值参数
                hx9031a_set_thres_near(ii, hx9031a_ch_thres[ii].thr_near);//更新接近阈值
                hx9031a_set_thres_far(ii, hx9031a_ch_thres[ii].thr_far);//更新远离阈值
            } else {//如果该通道当前处于远离态，则恢复该通道的阈值参数为预设值
                hx9031a_set_thres_near(ii, hx9031a_alg_ch_thres_backup[ii].thr_near);//恢复接近阈值
                hx9031a_set_thres_far(ii, hx9031a_alg_ch_thres_backup[ii].thr_far);//恢复远离阈值
                if(1 == ii) {
                    hx9031a_manual_offset_calibration(ii);
                }
            }
            PRINT_INF("ch_%d threshold: near=%d, far=%d\n", ii, hx9031a_ch_thres[ii].thr_near, hx9031a_ch_thres[ii].thr_far);
        }

        if (0 == (hx9031a_pdata.prox_state_reg & hx9031a_pdata.channel_used_flag)) {//所有使用的通道均处于远离态
            hx9031a_alg_dynamic_threshold_init(3);
        }
    }
    return 0;
}
#endif
//alg end===========================================================================

static int hx9031a_reg_reinitialize(void)
{
    int ret = -1;
    int ii = 0;
    uint8_t chip_select = 0;
    uint8_t tx_buf[1] = {0};

    ENTER;
    hx9031a_register_init();//寄存器初始化,如果需要，也可以使用chip_select做芯片区分
    chip_select = hx9031a_get_board_info();
    hx9031a_ch_cfg(chip_select);//通道配置
    hx9031a_data_switch(hx9031a_output_switch);//数据输出默认选择raw 和 diff

    for(ii = 0; ii < HX9031A_CH_NUM; ii++) {
        hx9031a_set_thres_near(ii, hx9031a_ch_thres[ii].thr_near);
        hx9031a_set_thres_far(ii, hx9031a_ch_thres[ii].thr_far);
    }

    if(ch_enable_status > 0) {
        tx_buf[0] = hx9031a_pdata.channel_used_flag;
        ret = hx9031a_write(RA_CH_NUM_CFG_0x24, tx_buf, 1);
        if(0 != ret) {
            PRINT_ERR("hx9031a_write failed\n");
            return -1;
        }
    }
    return 0;
}

static int hx9031a_state_monitoring(uint8_t addr, uint8_t val)
{
    int ret = -1;
    uint8_t rx_buf[1] = {0};

    if(0 == hx9031a_hw_monitor_en_flag) {
        return 0;
    }

    ret = hx9031a_read(addr, rx_buf, 1);
    if(0 != ret) {
        PRINT_ERR("hx9031a_read failed\n");
        return -1;
    }

    if(val != rx_buf[0]) {
        PRINT_ERR("chip restarted abnormally! do reg reinitialize!\n");
        hx9031a_reg_reinitialize();
    } else {
        PRINT_INF("OK\n");
    }

    return 0 ;
}

static void hx9031a_polling_work_func(struct work_struct *work)
{
    ENTER;
    mutex_lock(&hx9031a_ch_en_mutex);
    hx9031a_state_monitoring(hx9031a_monitor_addr, hx9031a_monitor_val);
    hx9031a_sample();
    hx9031a_get_prox_state(data_diff, HX9031A_CH_NUM);
    hx9031a_input_report();
    if(1 == hx9031a_polling_enable)
        schedule_delayed_work(&hx9031a_pdata.polling_work, msecs_to_jiffies(hx9031a_polling_period_ms));
    mutex_unlock(&hx9031a_ch_en_mutex);
    return;
}

static void hx9031a_hw_monitor_work_func(struct work_struct *work)
{
    ENTER;
    mutex_lock(&hx9031a_ch_en_mutex);
    hx9031a_state_monitoring(hx9031a_monitor_addr, hx9031a_monitor_val);
    schedule_delayed_work(&hx9031a_pdata.hw_monitor_work, msecs_to_jiffies(10000));
    mutex_unlock(&hx9031a_ch_en_mutex);
    return;
}

static irqreturn_t hx9031a_irq_handler(int irq, void *pvoid)
{
    ENTER;
    mutex_lock(&hx9031a_ch_en_mutex);
    if(1 == hx9031a_irq_from_suspend_flag) {
        hx9031a_irq_from_suspend_flag = 0;
        PRINT_INF("delay 50ms for waiting the i2c controller enter working mode\n");
        msleep(50);//如果从suspend被中断唤醒，该延时确保i2c控制器也从休眠唤醒并进入工作状态
    }
    hx9031a_state_monitoring(hx9031a_monitor_addr, hx9031a_monitor_val);
    hx9031a_sample();
    hx9031a_get_prox_state(data_diff, HX9031A_CH_NUM);
#if HX9031A_ALG_COMPILE_EN
    if(1 == hx9031a_alg_dynamic_threshold_en) {
        hx9031a_alg_dynamic_threshold_adjust(3);
    }
#endif
    hx9031a_input_report();
    mutex_unlock(&hx9031a_ch_en_mutex);
    return IRQ_HANDLED;
}

//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^sysfs for test begin
static ssize_t hx9031a_raw_data_show(struct class *class, struct class_attribute *attr, char *buf)
{
    char *p = buf;
    int ii = 0;

    ENTER;
    hx9031a_sample();
    for(ii = 0; ii < HX9031A_CH_NUM; ii++) {
        p += snprintf(p, PAGE_SIZE, "ch[%d]: DIFF=%-8d, RAW=%-8d, OFFSET=%-8d, BL=%-8d, LP=%-8d\n",
                      ii, data_diff[ii], data_raw[ii], data_offset_dac[ii], data_bl[ii], data_lp[ii]);
    }
    return (p - buf);//返回实际放到buf中的实际字符个数
}

static ssize_t hx9031a_register_write_store(struct class *class, struct class_attribute *attr, const char *buf, size_t count)
{
    int ret = -1;
    unsigned int reg_address = 0;
    unsigned int val = 0;
    uint8_t addr = 0;
    uint8_t tx_buf[1] = {0};

    ENTER;
    if (sscanf(buf, "%x,%x", &reg_address, &val) != 2) {
        PRINT_ERR("please input two HEX numbers: aa,bb (aa: reg_address, bb: value_to_be_set)\n");
        return -EINVAL;
    }

    addr = (uint8_t)reg_address;
    tx_buf[0] = (uint8_t)val;

    ret = hx9031a_write(addr, tx_buf, 1);
    if(0 != ret) {
        PRINT_ERR("hx9031a_write failed\n");
    }

    PRINT_INF("WRITE:Reg0x%02X=0x%02X\n", addr, tx_buf[0]);
    return count;
}

static ssize_t hx9031a_register_read_store(struct class *class, struct class_attribute *attr, const char *buf, size_t count)
{
    int ret = -1;
    unsigned int reg_address = 0;
    uint8_t addr = 0;
    uint8_t rx_buf[1] = {0};

    ENTER;
    if (sscanf(buf, "%x", &reg_address) != 1) {
        PRINT_ERR("please input a HEX number\n");
        return -EINVAL;
    }
    addr = (uint8_t)reg_address;

    ret = hx9031a_read(addr, rx_buf, 1);
    if(0 != ret) {
        PRINT_ERR("hx9031a_read failed\n");
    }

    PRINT_INF("READ:Reg0x%02X=0x%02X\n", addr, rx_buf[0]);
    return count;
}

static ssize_t hx9031a_channel_en_store(struct class *class, struct class_attribute *attr, const char *buf, size_t count)
{
    int ch_id = 0;
    int en = 0;

    ENTER;
    if (sscanf(buf, "%d,%d", &ch_id, &en) != 2) {
        PRINT_ERR("please input two DEC numbers: ch_id,en (ch_id: channel number, en: 1=enable, 0=disable)\n");
        return -EINVAL;
    }

    if((ch_id >= HX9031A_CH_NUM) || (ch_id < 0)) {
        PRINT_ERR("channel number out of range, the effective number is 0~%d\n", HX9031A_CH_NUM - 1);
        return -EINVAL;
    }

    if ((hx9031a_pdata.channel_used_flag >> ch_id) & 0x01) {
        hx9031a_set_enable(&hx9031a_channels[ch_id].hx9031a_sensors_classdev, (en > 0) ? 1 : 0);
    } else {
        PRINT_ERR("ch_%d is unused, you can not enable or disable an unused channel\n", ch_id);
    }

    return count;
}

static ssize_t hx9031a_channel_en_show(struct class *class, struct class_attribute *attr, char *buf)
{
    int ii = 0;
    char *p = buf;

    ENTER;
    for(ii = 0; ii < HX9031A_CH_NUM; ii++) {
        if ((hx9031a_pdata.channel_used_flag >> ii) & 0x1) {
            PRINT_INF("hx9031a_channels[%d].enabled=%d\n", ii, hx9031a_channels[ii].enabled);
            p += snprintf(p, PAGE_SIZE, "hx9031a_channels[%d].enabled=%d\n", ii, hx9031a_channels[ii].enabled);
        }
    }

    return (p - buf);
}

static ssize_t hx9031a_manual_offset_calibration_show(struct class *class, struct class_attribute *attr, char *buf)
{
    hx9031a_read_offset_dac();
    return sprintf(buf, "OFFSET_DAC, %-8d, %-8d, %-8d, %-8d\n", data_offset_dac[0], data_offset_dac[1], data_offset_dac[2], data_offset_dac[3]);
}

static ssize_t hx9031a_manual_offset_calibration_store(struct class *class, struct class_attribute *attr, const char *buf, size_t count)
{
    int ii = 0;
    unsigned long val;
    uint8_t ch_id = 0;

    ENTER;
    if (kstrtoul(buf, 10, &val)) {
        PRINT_ERR("Invalid Argument\n");
        return -EINVAL;
    }
    ch_id = (uint8_t)val;

    if (99 == ch_id) {
        PRINT_INF("you are enter the calibration test mode, all channels will be calibrated\n");
        for(ii = 0; ii < HX9031A_CH_NUM; ii++) {
            if ((hx9031a_pdata.channel_used_flag >> ii) & 0x1) {
                hx9031a_manual_offset_calibration(ii);
            }
        }
        return count;
    }

    if (ch_id < HX9031A_CH_NUM)
        hx9031a_manual_offset_calibration(ch_id);
    else
        PRINT_ERR(" \"echo ch_id > calibrate\" to do a manual calibrate(ch_id is a channel num (0~%d)\n", HX9031A_CH_NUM);
    return count;
}

static ssize_t hx9031a_prox_state_show(struct class *class, struct class_attribute *attr, char *buf)
{
    PRINT_DBG("prox_state_reg=0x%02X, prox_state_cmp=0x%02X, hx9031a_ch_near_state:%d-%d-%d-%d, far_to_near_flag=0x%02X\n",
              hx9031a_pdata.prox_state_reg,
              hx9031a_pdata.prox_state_cmp,
              hx9031a_ch_near_state[3],
              hx9031a_ch_near_state[2],
              hx9031a_ch_near_state[1],
              hx9031a_ch_near_state[0],
              hx9031a_pdata.prox_state_far_to_near_flag);

    return sprintf(buf, "prox_state_reg=0x%02X, prox_state_cmp=0x%02X, hx9031a_ch_near_state:%d-%d-%d-%d, far_to_near_flag=0x%02X\n",
                   hx9031a_pdata.prox_state_reg,
                   hx9031a_pdata.prox_state_cmp,
                   hx9031a_ch_near_state[3],
                   hx9031a_ch_near_state[2],
                   hx9031a_ch_near_state[1],
                   hx9031a_ch_near_state[0],
                   hx9031a_pdata.prox_state_far_to_near_flag);
}

static ssize_t hx9031a_polling_store(struct class *class, struct class_attribute *attr, const char *buf, size_t count)
{
    int value = 0;
    int ret = -1;

    ENTER;
    ret = kstrtoint(buf, 10, &value);
    if (0 != ret) {
        PRINT_ERR("kstrtoint failed\n");
        goto exit;
    }

    if (value >= 10) {
        hx9031a_polling_period_ms = value;
        if(1 == hx9031a_polling_enable) {
            PRINT_INF("polling is already enabled!, no need to do enable again!, just update the polling period\n");
            goto exit;
        }

        hx9031a_polling_enable = 1;
        hx9031a_disable_irq(hx9031a_pdata.irq);//关闭中断，并停止中断底半部对应的工作队列

        PRINT_INF("polling started! period=%dms\n", hx9031a_polling_period_ms);
        schedule_delayed_work(&hx9031a_pdata.polling_work, msecs_to_jiffies(hx9031a_polling_period_ms));
    } else {
        if(0 == hx9031a_polling_enable) {
            PRINT_INF("polling is already disabled!, no need to do again!\n");
            goto exit;
        }
        hx9031a_polling_period_ms = 0;
        hx9031a_polling_enable = 0;
        PRINT_INF("polling stoped!\n");

        cancel_delayed_work(&hx9031a_pdata.polling_work);//停止polling对应的工作队列，并重新开启中断模式
        hx9031a_enable_irq(hx9031a_pdata.irq);
    }

exit:
    return count;
}

static ssize_t hx9031a_polling_show(struct class *class, struct class_attribute *attr, char *buf)
{
    PRINT_INF("hx9031a_polling_enable=%d hx9031a_polling_period_ms=%d\n", hx9031a_polling_enable, hx9031a_polling_period_ms);
    return sprintf(buf, "hx9031a_polling_enable=%d hx9031a_polling_period_ms=%d\n", hx9031a_polling_enable, hx9031a_polling_period_ms);
}

static ssize_t hx9031a_loglevel_show(struct class *class, struct class_attribute *attr, char *buf)
{
    PRINT_INF("tyhx_log_level=%d\n", tyhx_log_level);
    return sprintf(buf, "tyhx_log_level=%d\n", tyhx_log_level);
}

static ssize_t hx9031a_loglevel_store(struct class *class, struct class_attribute *attr, const char *buf, size_t count)
{
    int ret = -1;
    int value = 0;

    ret = kstrtoint(buf, 10, &value);
    if (0 != ret) {
        PRINT_ERR("kstrtoint failed\n");
        return count;
    }

    tyhx_log_level = value;
    PRINT_INF("set tyhx_log_level=%d\n", tyhx_log_level);
    return count;
}

static ssize_t hx9031a_accuracy_show(struct class *class, struct class_attribute *attr, char *buf)
{
    PRINT_INF("hx9031a_data_accuracy=%d\n", hx9031a_data_accuracy);
    return sprintf(buf, "hx9031a_data_accuracy=%d\n", hx9031a_data_accuracy);
}

static ssize_t hx9031a_accuracy_store(struct class *class, struct class_attribute *attr, const char *buf, size_t count)
{
    int ret = -1;
    int value = 0;

    ret = kstrtoint(buf, 10, &value);
    if (0 != ret) {
        PRINT_ERR("kstrtoint failed\n");
        return count;
    }

    hx9031a_data_accuracy = (24 == value) ? 24 : 16;
    PRINT_INF("set hx9031a_data_accuracy=%d\n", hx9031a_data_accuracy);
    return count;
}

static ssize_t hx9031a_output_switch_show(struct class *class, struct class_attribute *attr, char *buf)
{
    PRINT_INF("hx9031a_output_switch=%d\n", hx9031a_output_switch);
    return sprintf(buf, "hx9031a_output_switch=%d\n", hx9031a_output_switch);
}

static ssize_t hx9031a_output_switch_store(struct class *class, struct class_attribute *attr, const char *buf, size_t count)
{
    int ret = -1;
    int value = 0;

    ret = kstrtoint(buf, 10, &value);
    if (0 != ret) {
        PRINT_ERR("kstrtoint failed\n");
        return count;
    }

    hx9031a_data_switch((HX9031A_OUTPUT_RAW_DIFF == value) ? HX9031A_OUTPUT_RAW_DIFF : HX9031A_OUTPUT_LP_BL);
    PRINT_INF("set hx9031a_output_switch=%d\n", hx9031a_output_switch);
    return count;
}

static ssize_t hx9031a_threshold_store(struct class *class, struct class_attribute *attr, const char *buf, size_t count)
{
    unsigned int ch = 0;
    unsigned int thr_near = 0;
    unsigned int thr_far = 0;

    ENTER;
    if (sscanf(buf, "%d,%d,%d", &ch, &thr_near, &thr_far) != 3) {
        PRINT_ERR("please input 3 numbers in DEC: ch,thr_near,thr_far (eg: 0,500,300)\n");
        return -EINVAL;
    }

    if(ch >= HX9031A_CH_NUM || thr_near > (0x03FF * 32) || thr_far > thr_near) {
        PRINT_ERR("input value over range! (valid value: ch=%d, thr_near=%d, thr_far=%d)\n", ch, thr_near, thr_far);
        return -EINVAL;
    }

    thr_near = (thr_near / 32) * 32;
    thr_far = (thr_far / 32) * 32;

    PRINT_INF("set threshold: ch=%d, thr_near=%d, thr_far=%d\n", ch, thr_near, thr_far);
    hx9031a_set_thres_near(ch, thr_near);
    hx9031a_set_thres_far(ch, thr_far);

    return count;
}

static ssize_t hx9031a_threshold_show(struct class *class, struct class_attribute *attr, char *buf)
{
    int ii = 0;
    char *p = buf;

    for(ii = 0; ii < HX9031A_CH_NUM; ii++) {
        hx9031a_get_thres_near(ii);
        hx9031a_get_thres_far(ii);
        PRINT_INF("ch_%d threshold: near=%-8d, far=%-8d\n", ii, hx9031a_ch_thres[ii].thr_near, hx9031a_ch_thres[ii].thr_far);
        p += snprintf(p, PAGE_SIZE, "ch_%d threshold: near=%-8d, far=%-8d\n", ii, hx9031a_ch_thres[ii].thr_near, hx9031a_ch_thres[ii].thr_far);
    }

    return (p - buf);
}

static ssize_t hx9031a_dump_show(struct class *class, struct class_attribute *attr, char *buf)
{
    int ret = -1;
    int ii = 0;
    char *p = buf;
    uint8_t rx_buf[1] = {0};

    for(ii = 0; ii < hx9031a_reg_init_list_size; ii++) {
        ret = hx9031a_read(hx9031a_reg_init_list[ii].addr, rx_buf, 1);
        if(0 != ret) {
            PRINT_ERR("hx9031a_read failed\n");
        }
        PRINT_INF("0x%02X=0x%02X\n", hx9031a_reg_init_list[ii].addr, rx_buf[0]);
        p += snprintf(p, PAGE_SIZE, "0x%02X=0x%02X\n", hx9031a_reg_init_list[ii].addr, rx_buf[0]);
    }

    p += snprintf(p, PAGE_SIZE, "driver version:%s\n", HX9031A_DRIVER_VER);
    return (p - buf);
}

static ssize_t hx9031a_offset_dac_show(struct class *class, struct class_attribute *attr, char *buf)
{
    int ii = 0;
    char *p = buf;

    hx9031a_read_offset_dac();

    for(ii = 0; ii < HX9031A_CH_NUM; ii++) {
        PRINT_INF("data_offset_dac[%d]=%dpF\n", ii, data_offset_dac[ii] * 58 / 1000);
        p += snprintf(p, PAGE_SIZE, "ch[%d]=%dpF ", ii, data_offset_dac[ii] * 58 / 1000);
    }
    p += snprintf(p, PAGE_SIZE, "\n");

    return (p - buf);
}

static ssize_t hx9031a_reinitialize_store(struct class *class, struct class_attribute *attr, const char *buf, size_t count)
{
    int ret = -1;
    int value = 0;

    ret = kstrtoint(buf, 10, &value);
    if (0 != ret) {
        PRINT_ERR("kstrtoint failed\n");
        return count;
    }

    if(1 == value) {
        PRINT_INF("start reg reinitialize\n");
        hx9031a_reg_reinitialize();
    }
    return count;
}

static ssize_t hx9031a_hw_monitor_en_show(struct class *class, struct class_attribute *attr, char *buf)
{
    PRINT_INF("hx9031a_hw_monitor_en_flag=%d\n", hx9031a_hw_monitor_en_flag);
    return sprintf(buf, "hx9031a_hw_monitor_en_flag=%d\n", hx9031a_hw_monitor_en_flag);
}

static ssize_t hx9031a_hw_monitor_en_store(struct class *class, struct class_attribute *attr, const char *buf, size_t count)
{
    int ret = -1;
    int value = 0;

    ret = kstrtoint(buf, 10, &value);
    if (0 != ret) {
        PRINT_ERR("kstrtoint failed\n");
        return count;
    }

    hx9031a_hw_monitor_en_flag = (0 == value) ? 0 : 1;

    if(1 == hx9031a_hw_monitor_en_flag) {
        cancel_delayed_work_sync(&(hx9031a_pdata.hw_monitor_work));
        schedule_delayed_work(&hx9031a_pdata.hw_monitor_work, msecs_to_jiffies(10000));
    } else {
        cancel_delayed_work_sync(&(hx9031a_pdata.hw_monitor_work));
    }

    PRINT_INF("set hx9031a_hw_monitor_en_flag=%d\n", hx9031a_hw_monitor_en_flag);
    return count;
}

#if HX9031A_ALG_COMPILE_EN
static ssize_t hx9031a_drift_min_store(struct class *class, struct class_attribute *attr, const char *buf, size_t count)
{
    int temp[4] = {0};
    int ii = 0;

    ENTER;
    if (sscanf(buf, "%d,%d,%d,%d", &temp[0],
               &temp[1],
               &temp[2],
               &temp[3]) != 4) {
        PRINT_ERR("please input 4 numbers in DEC:(eg: -300,500,300,800)\n");
        return -EINVAL;
    }

    for(ii = 0; ii < 4; ii++) {
        hx9031a_alg_ch_thres_drift_min[ii] = temp[ii];
        PRINT_INF("hx9031a_alg_ch_thres_drift_min[%d]=%d\n", ii, hx9031a_alg_ch_thres_drift_min[ii]);
    }

    return count;
}

static ssize_t hx9031a_drift_min_show(struct class *class, struct class_attribute *attr, char *buf)
{
    int ii = 0;
    char *p = buf;

    for(ii = 0; ii < 4; ii++) {
        PRINT_INF("hx9031a_alg_ch_thres_drift_min[%d]=%d\n", ii, hx9031a_alg_ch_thres_drift_min[ii]);
        p += snprintf(p, PAGE_SIZE, "hx9031a_alg_ch_thres_drift_min[%d]=%d\n", ii, hx9031a_alg_ch_thres_drift_min[ii]);
    }

    return (p - buf);
}

static ssize_t hx9031a_drift_max_store(struct class *class, struct class_attribute *attr, const char *buf, size_t count)
{
    int temp[4] = {0};
    int ii = 0;

    ENTER;
    if (sscanf(buf, "%d,%d,%d,%d", &temp[0],
               &temp[1],
               &temp[2],
               &temp[3]) != 4) {
        PRINT_ERR("please input 4 numbers in DEC:(eg: -300,500,300,800)\n");
        return -EINVAL;
    }

    for(ii = 0; ii < 4; ii++) {
        hx9031a_alg_ch_thres_drift_max[ii] = temp[ii];
        PRINT_INF("hx9031a_alg_ch_thres_drift_max[%d]=%d\n", ii, hx9031a_alg_ch_thres_drift_max[ii]);
    }

    return count;
}

static ssize_t hx9031a_drift_max_show(struct class *class, struct class_attribute *attr, char *buf)
{
    int ii = 0;
    char *p = buf;

    for(ii = 0; ii < 4; ii++) {
        PRINT_INF("hx9031a_alg_ch_thres_drift_max[%d]=%d\n", ii, hx9031a_alg_ch_thres_drift_max[ii]);
        p += snprintf(p, PAGE_SIZE, "hx9031a_alg_ch_thres_drift_max[%d]=%d\n", ii, hx9031a_alg_ch_thres_drift_max[ii]);
    }

    return (p - buf);
}

static ssize_t hx9031a_drift_offset_store(struct class *class, struct class_attribute *attr, const char *buf, size_t count)
{
    int temp[4] = {0};
    int ii = 0;

    ENTER;
    if (sscanf(buf, "%d,%d,%d,%d", &temp[0],
               &temp[1],
               &temp[2],
               &temp[3]) != 4) {
        PRINT_ERR("please input 4 numbers in DEC:(eg: -300,500,300,800)\n");
        return -EINVAL;
    }

    for(ii = 0; ii < 4; ii++) {
        hx9031a_alg_ch_thres_drift_offset[ii] = temp[ii];
        PRINT_INF("hx9031a_alg_ch_thres_drift_offset[%d]=%d\n", ii, hx9031a_alg_ch_thres_drift_offset[ii]);
    }

    return count;
}

static ssize_t hx9031a_drift_offset_show(struct class *class, struct class_attribute *attr, char *buf)
{
    int ii = 0;
    char *p = buf;

    for(ii = 0; ii < 4; ii++) {
        PRINT_INF("hx9031a_alg_ch_thres_drift_offset[%d]=%d\n", ii, hx9031a_alg_ch_thres_drift_offset[ii]);
        p += snprintf(p, PAGE_SIZE, "hx9031a_alg_ch_thres_drift_offset[%d]=%d\n", ii, hx9031a_alg_ch_thres_drift_offset[ii]);
    }

    return (p - buf);
}

static ssize_t hx9031a_threshold_backup_store(struct class *class, struct class_attribute *attr, const char *buf, size_t count)
{
    unsigned int ch = 0;
    unsigned int thr_near = 0;
    unsigned int thr_far = 0;

    ENTER;
    if (sscanf(buf, "%d,%d,%d", &ch, &thr_near, &thr_far) != 3) {
        PRINT_ERR("please input 3 numbers in DEC: ch,thr_near,thr_far (eg: 0,500,300)\n");
        return -EINVAL;
    }

    if(ch >= HX9031A_CH_NUM || thr_near > (0x03FF * 32) || thr_far > thr_near) {
        PRINT_ERR("input value over range! (valid value: ch=%d, thr_near=%d, thr_far=%d)\n", ch, thr_near, thr_far);
        return -EINVAL;
    }

    thr_near = (thr_near / 32) * 32;
    thr_far = (thr_far / 32) * 32;

    PRINT_INF("set threshold of backup: ch=%d, thr_near=%d, thr_far=%d\n", ch, thr_near, thr_far);
    hx9031a_alg_ch_thres_backup[ch].thr_far = thr_far;
    hx9031a_alg_ch_thres_backup[ch].thr_near = thr_near;

    return count;
}

static ssize_t hx9031a_threshold_backup_show(struct class *class, struct class_attribute *attr, char *buf)
{
    int ii = 0;
    char *p = buf;

    for(ii = 0; ii < HX9031A_CH_NUM; ii++) {
        PRINT_INF("ch_%d threshold of backup: near=%-8d, far=%-8d\n",
                  ii, hx9031a_alg_ch_thres_backup[ii].thr_near, hx9031a_alg_ch_thres_backup[ii].thr_far);
        p += snprintf(p, PAGE_SIZE, "ch_%d threshold of backup: near=%-8d, far=%-8d\n",
                      ii, hx9031a_alg_ch_thres_backup[ii].thr_near, hx9031a_alg_ch_thres_backup[ii].thr_far);
    }

    return (p - buf);
}

static ssize_t hx9031a_alg_dynamic_threshold_en_show(struct class *class, struct class_attribute *attr, char *buf)
{
    PRINT_INF("hx9031a_alg_dynamic_threshold_en=%d\n", hx9031a_alg_dynamic_threshold_en);
    return sprintf(buf, "hx9031a_alg_dynamic_threshold_en=%d\n", hx9031a_alg_dynamic_threshold_en);
}

static ssize_t hx9031a_alg_dynamic_threshold_en_store(struct class *class, struct class_attribute *attr, const char *buf, size_t count)
{
    int ret = -1;
    int value = 0;

    ret = kstrtoint(buf, 10, &value);
    if (0 != ret) {
        PRINT_ERR("kstrtoint failed\n");
        return count;
    }

    hx9031a_alg_dynamic_threshold_en = (0 == value) ? 0 : 1;
    hx9031a_alg_dynamic_threshold_init(3);

    PRINT_INF("set hx9031a_alg_dynamic_threshold_en=%d\n", hx9031a_alg_dynamic_threshold_en);
    PRINT_INF("Warning!!! all channels must be disabled before you set the hx9031a_alg_dynamic_threshold_en flag!\n");
    return count;
}
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,14,0)
static struct class_attribute class_attr_raw_data = __ATTR(raw_data, 0664, hx9031a_raw_data_show, NULL);
static struct class_attribute class_attr_reg_write = __ATTR(reg_write,  0664, NULL, hx9031a_register_write_store);
static struct class_attribute class_attr_reg_read = __ATTR(reg_read, 0664, NULL, hx9031a_register_read_store);
static struct class_attribute class_attr_channel_en = __ATTR(channel_en, 0664, hx9031a_channel_en_show, hx9031a_channel_en_store);
static struct class_attribute class_attr_calibrate = __ATTR(calibrate, 0664, hx9031a_manual_offset_calibration_show, hx9031a_manual_offset_calibration_store);
static struct class_attribute class_attr_prox_state = __ATTR(prox_state, 0664, hx9031a_prox_state_show, NULL);
static struct class_attribute class_attr_polling_period = __ATTR(polling_period, 0664, hx9031a_polling_show, hx9031a_polling_store);
static struct class_attribute class_attr_threshold = __ATTR(threshold, 0664, hx9031a_threshold_show, hx9031a_threshold_store);
static struct class_attribute class_attr_loglevel = __ATTR(loglevel, 0664, hx9031a_loglevel_show, hx9031a_loglevel_store);
static struct class_attribute class_attr_accuracy = __ATTR(accuracy, 0664, hx9031a_accuracy_show, hx9031a_accuracy_store);
static struct class_attribute class_attr_output_switch = __ATTR(output_switch, 0664, hx9031a_output_switch_show, hx9031a_output_switch_store);
static struct class_attribute class_attr_dump = __ATTR(dump, 0664, hx9031a_dump_show, NULL);
static struct class_attribute class_attr_offset_dac = __ATTR(offset_dac, 0664, hx9031a_offset_dac_show, NULL);
static struct class_attribute class_attr_reinitialize = __ATTR(reinitialize,  0664, NULL, hx9031a_reinitialize_store);
static struct class_attribute class_attr_hw_monitor_en = __ATTR(hw_monitor_en, 0664, hx9031a_hw_monitor_en_show, hx9031a_hw_monitor_en_store);
#if HX9031A_ALG_COMPILE_EN
static struct class_attribute class_attr_drift_min = __ATTR(drift_min, 0664, hx9031a_drift_min_show, hx9031a_drift_min_store);
static struct class_attribute class_attr_drift_max = __ATTR(drift_max, 0664, hx9031a_drift_max_show, hx9031a_drift_max_store);
static struct class_attribute class_attr_drift_offset = __ATTR(drift_offset, 0664, hx9031a_drift_offset_show, hx9031a_drift_offset_store);
static struct class_attribute class_attr_threshold_backup = __ATTR(threshold_backup, 0664, hx9031a_threshold_backup_show, hx9031a_threshold_backup_store);
static struct class_attribute class_attr_alg_dynamic_threshold_en = __ATTR(alg_dynamic_threshold_en, 0664, hx9031a_alg_dynamic_threshold_en_show, hx9031a_alg_dynamic_threshold_en_store);
#endif

static struct attribute *hx9031a_class_attrs[] = {
    &class_attr_raw_data.attr,
    &class_attr_reg_write.attr,
    &class_attr_reg_read.attr,
    &class_attr_channel_en.attr,
    &class_attr_calibrate.attr,
    &class_attr_prox_state.attr,
    &class_attr_polling_period.attr,
    &class_attr_threshold.attr,
    &class_attr_loglevel.attr,
    &class_attr_accuracy.attr,
    &class_attr_output_switch.attr,
    &class_attr_dump.attr,
    &class_attr_offset_dac.attr,
    &class_attr_reinitialize.attr,
    &class_attr_hw_monitor_en.attr,
#if HX9031A_ALG_COMPILE_EN
    &class_attr_drift_min.attr,
    &class_attr_drift_max.attr,
    &class_attr_drift_offset.attr,
    &class_attr_threshold_backup.attr,
    &class_attr_alg_dynamic_threshold_en.attr,
#endif
    NULL,
};
ATTRIBUTE_GROUPS(hx9031a_class);
#else
static struct class_attribute hx9031a_class_attributes[] = {
    __ATTR(raw_data, 0664, hx9031a_raw_data_show, NULL),
    __ATTR(reg_write,  0664, NULL, hx9031a_register_write_store),
    __ATTR(reg_read, 0664, NULL, hx9031a_register_read_store),
    __ATTR(channel_en, 0664, hx9031a_channel_en_show, hx9031a_channel_en_store),
    __ATTR(calibrate, 0664, hx9031a_manual_offset_calibration_show, hx9031a_manual_offset_calibration_store),
    __ATTR(prox_state, 0664, hx9031a_prox_state_show, NULL),
    __ATTR(polling_period, 0664, hx9031a_polling_show, hx9031a_polling_store),
    __ATTR(threshold, 0664, hx9031a_threshold_show, hx9031a_threshold_store),
    __ATTR(loglevel, 0664, hx9031a_loglevel_show, hx9031a_loglevel_store),
    __ATTR(accuracy, 0664, hx9031a_accuracy_show, hx9031a_accuracy_store),
    __ATTR(output_switch, 0664, hx9031a_output_switch_show, hx9031a_output_switch_store),
    __ATTR(dump, 0664, hx9031a_dump_show, NULL),
    __ATTR(offset_dac, 0664, hx9031a_offset_dac_show, NULL),
    __ATTR(reinitialize,  0664, NULL, hx9031a_reinitialize_store),
    __ATTR(hw_monitor_en, 0664, hx9031a_hw_monitor_en_show, hx9031a_hw_monitor_en_store),
#if HX9031A_ALG_COMPILE_EN
    __ATTR(drift_min, 0664, hx9031a_drift_min_show, hx9031a_drift_min_store),
    __ATTR(drift_max, 0664, hx9031a_drift_max_show, hx9031a_drift_max_store),
    __ATTR(drift_offset, 0664, hx9031a_drift_offset_show, hx9031a_drift_offset_store),
    __ATTR(threshold_backup, 0664, hx9031a_threshold_backup_show, hx9031a_threshold_backup_store),
    __ATTR(alg_dynamic_threshold_en, 0664, hx9031a_alg_dynamic_threshold_en_show, hx9031a_alg_dynamic_threshold_en_store),
#endif
    __ATTR_NULL,
};
#endif

struct class hx9031a_class = {
        .name = "hx9031a",
        .owner = THIS_MODULE,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,14,0)
        .class_groups = hx9031a_class_groups,
#else
        .class_attrs = hx9031a_class_attributes,
#endif
    };
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^sysfs for test end

static int hx9031a_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int ii = 0;
    int ret = 0;
	int error = 0;
    uint8_t chip_select = 0;
    struct hx9031a_channel_info *channel_p = NULL;

    ENTER;
    //Note:由于我们的i2c地址和友商相同，为了和友商的芯片在驱动层面共存，我们在dts中将i2c地址配置为0x29，此处需要改回正确的地址0x28
    PRINT_INF("i2c address from DTS is 0x%02X\n", client->addr);
    client->addr = 0x28;
    PRINT_INF("i2c address from driver is 0x%02X\n", client->addr);

    if (!i2c_check_functionality(to_i2c_adapter(client->dev.parent), I2C_FUNC_SMBUS_READ_WORD_DATA)) {
        PRINT_ERR("i2c_check_functionality failed\n");
        ret = -EIO;
        goto failed_i2c_check_functionality;
    }

    i2c_set_clientdata(client, (&hx9031a_pdata));
    hx9031a_i2c_client = client;
    hx9031a_pdata.pdev = &client->dev;
    client->dev.platform_data = (&hx9031a_pdata);

//{begin =============================================需要客户自行配置dts属性和实现上电相关内容
    ret = hx9031a_parse_dt(&client->dev);//yasin: power, irq, regs
    if (ret) {
        PRINT_ERR("hx9031a_parse_dt failed\n");
        ret = -ENODEV;
        goto failed_parse_dt;
    }
/*	
	hx9031a_vdd = devm_regulator_get(&client->dev, "tyhx,power");
	if (IS_ERR(hx9031a_vdd)) {
		error = PTR_ERR(hx9031a_vdd);
		if (error != -EPROBE_DEFER)
			dev_err(&client->dev,
						"Failed to get 'vdd' regulator: %d\n", error);
		return error;
	}
*/
    ret = hx9031a_gpio_init();
    if (ret) {
        PRINT_ERR("hx9031a_gpio_init failed\n");
        ret = -1;
        goto failed_gpio_init;
    }

    client->irq = hx9031a_pdata.irq;
    if (hx9031a_vdd != NULL)
    	hx9031a_power_on(hx9031a_vdd, true);
//}end =============================================================

    ret = hx9031a_id_check();
    if(0 != ret) {
        PRINT_ERR("hx9031_id_check failed\n");
        goto failed_id_check;
    }

    chip_select = hx9031a_get_board_info();
    hx9031a_reg_list_redirect(chip_select);
    hx9031a_register_init();//寄存器初始化,如果需要，也可以使用chip_select做芯片区分
    hx9031a_ch_cfg(chip_select);//通道配置
    hx9031a_ch_thres_array_redirect(chip_select);
    hx9031a_data_switch(hx9031a_output_switch);

#if HX9031A_ALG_COMPILE_EN
    for(ii = 0; ii < HX9031A_CH_NUM; ii++) {
        hx9031a_alg_ch_thres_backup[ii].thr_far = hx9031a_ch_thres[ii].thr_far;
        hx9031a_alg_ch_thres_backup[ii].thr_near = hx9031a_ch_thres[ii].thr_near;
    }
    hx9031a_alg_dynamic_threshold_init(3);
#else
    for(ii = 0; ii < HX9031A_CH_NUM; ii++) {
        hx9031a_set_thres_near(ii, hx9031a_ch_thres[ii].thr_near);
        hx9031a_set_thres_far(ii, hx9031a_ch_thres[ii].thr_far);
    }
#endif

    for (ii = 0; ii < ARRAY_SIZE(hx9031a_channels); ii++) {
        if ((hx9031a_pdata.channel_used_flag >> ii) & 0x1) { //used flag的值来自dts，表示通道有效性，每一位表示一个通道。
            hx9031a_channels[ii].used = true;
            hx9031a_channels[ii].state = IDLE;

            hx9031a_channels[ii].hx9031a_input_dev = input_allocate_device();
            if (!hx9031a_channels[ii].hx9031a_input_dev) {
                PRINT_ERR("input_allocate_device failed\n");
                ret = -ENOMEM;
                goto failed_allocate_input_dev;
            }

            hx9031a_channels[ii].hx9031a_input_dev->name = hx9031a_channels[ii].name;
            __set_bit(EV_ABS, hx9031a_channels[ii].hx9031a_input_dev->evbit);
            input_set_abs_params(hx9031a_channels[ii].hx9031a_input_dev, ABS_DISTANCE, -1, 100, 0, 0);

            ret = input_register_device(hx9031a_channels[ii].hx9031a_input_dev);

            input_report_abs(hx9031a_channels[ii].hx9031a_input_dev, ABS_DISTANCE, -1);
            input_sync(hx9031a_channels[ii].hx9031a_input_dev);

            hx9031a_channels[ii].hx9031a_sensors_classdev.sensors_enable = hx9031a_set_enable;
            hx9031a_channels[ii].hx9031a_sensors_classdev.sensors_poll_delay = NULL;
            hx9031a_channels[ii].hx9031a_sensors_classdev.name = hx9031a_channels[ii].name;
            hx9031a_channels[ii].hx9031a_sensors_classdev.vendor = "HX9031A";
            hx9031a_channels[ii].hx9031a_sensors_classdev.version = 1;
            hx9031a_channels[ii].hx9031a_sensors_classdev.type = SENSOR_TYPE_CAPSENSE;
            hx9031a_channels[ii].hx9031a_sensors_classdev.max_range = "5";
            hx9031a_channels[ii].hx9031a_sensors_classdev.resolution = "5.0";
            hx9031a_channels[ii].hx9031a_sensors_classdev.sensor_power = "3";
            hx9031a_channels[ii].hx9031a_sensors_classdev.min_delay = 0;
            hx9031a_channels[ii].hx9031a_sensors_classdev.fifo_reserved_event_count = 0;
            hx9031a_channels[ii].hx9031a_sensors_classdev.fifo_max_event_count = 0;
            hx9031a_channels[ii].hx9031a_sensors_classdev.delay_msec = 100;
            hx9031a_channels[ii].hx9031a_sensors_classdev.enabled = 0;
            hx9031a_channels[ii].enabled = false;

            ret = sensors_classdev_register(&hx9031a_channels[ii].hx9031a_input_dev->dev, &hx9031a_channels[ii].hx9031a_sensors_classdev);
            if (ret < 0) {
                PRINT_ERR("create %d cap sensor_class  file failed (%d)\n", ii, ret);
            }
        }
    }

    spin_lock_init(&hx9031a_pdata.lock);
    INIT_DELAYED_WORK(&hx9031a_pdata.polling_work, hx9031a_polling_work_func);
    INIT_DELAYED_WORK(&hx9031a_pdata.hw_monitor_work, hx9031a_hw_monitor_work_func);
/*
#ifdef CONFIG_PM_WAKELOCKS
    wakeup_source_init(&hx9031a_wake_lock, "hx9031a_wakelock");
#else
    wake_lock_init(&hx9031a_wake_lock, WAKE_LOCK_SUSPEND, "hx9031a_wakelock");
#endif
*/
    ret = request_threaded_irq(hx9031a_pdata.irq, NULL, hx9031a_irq_handler,
                               IRQF_TRIGGER_FALLING | IRQF_ONESHOT | IRQF_NO_SUSPEND,
                               hx9031a_pdata.pdev->driver->name, (&hx9031a_pdata));
    if(ret < 0) {
        PRINT_ERR("request_irq failed irq=%d ret=%d\n", hx9031a_pdata.irq, ret);
        goto failed_request_irq;
    }
    enable_irq_wake(hx9031a_pdata.irq);//enable irq wakeup PM
    hx9031a_irq_en_flag = 1;//irq is enabled after request by default

    //debug sys fs
    ret = class_register(&hx9031a_class);
    if (ret < 0) {
        PRINT_ERR("class_register failed\n");
        goto failed_class_register;
    }

#if HX9031A_TEST_CHS_EN //enable channels for test
    PRINT_INF("enable all chs for test\n");
    for(ii = 0; ii < HX9031A_CH_NUM; ii++) {
        if ((hx9031a_pdata.channel_used_flag >> ii) & 0x1) {
            hx9031a_set_enable(&hx9031a_channels[ii].hx9031a_sensors_classdev, 1);
            PRINT_INF("hx9031a_channels[%d].enabled=%d\n", ii, hx9031a_channels[ii].enabled);
        }
    }
#endif

    PRINT_INF("probe success\n");
    return 0;

failed_class_register:
    free_irq(hx9031a_pdata.irq, (&hx9031a_pdata));
failed_request_irq:
/*
#ifdef CONFIG_PM_WAKELOCKS
    wakeup_source_trash(&hx9031a_wake_lock);
#else
    wake_lock_destroy(&hx9031a_wake_lock);
#endif
*/
    cancel_delayed_work_sync(&(hx9031a_pdata.polling_work));
    for (ii = 0; ii < ARRAY_SIZE(hx9031a_channels); ii++) {
        channel_p = &(hx9031a_channels[ii]);
        if (channel_p->used == true) {
            sensors_classdev_unregister(&(channel_p->hx9031a_sensors_classdev));
            input_unregister_device(channel_p->hx9031a_input_dev);
        }
    }
failed_allocate_input_dev:
failed_id_check:
    if (hx9031a_vdd != NULL)
   	 hx9031a_power_on(hx9031a_vdd, false);
    hx9031a_gpio_deinit();
failed_gpio_init:
failed_parse_dt:
failed_i2c_check_functionality:
    PRINT_ERR("probe failed\n");
    return ret;
}

static int hx9031a_remove(struct i2c_client *client)
{
    struct hx9031a_channel_info *channel_p = NULL;
    int ii = 0;

    ENTER;
    cancel_delayed_work_sync(&(hx9031a_pdata.hw_monitor_work));
    class_unregister(&hx9031a_class);
    free_irq(hx9031a_pdata.irq, (&hx9031a_pdata));
/*
#ifdef CONFIG_PM_WAKELOCKS
    wakeup_source_trash(&hx9031a_wake_lock);
#else
    wake_lock_destroy(&hx9031a_wake_lock);
#endif
*/
    cancel_delayed_work_sync(&(hx9031a_pdata.polling_work));
    for (ii = 0; ii < ARRAY_SIZE(hx9031a_channels); ii++) {
        channel_p = &(hx9031a_channels[ii]);
        if (channel_p->used == true) {
            sensors_classdev_unregister(&(channel_p->hx9031a_sensors_classdev));
            input_unregister_device(channel_p->hx9031a_input_dev);
        }
    }
    if (hx9031a_vdd != NULL)
    	hx9031a_power_on(hx9031a_vdd, false);
    hx9031a_gpio_deinit();
    return 0;
}

static int hx9031a_suspend(struct device *dev)
{
    ENTER;
    hx9031a_irq_from_suspend_flag = 1;
    return 0;
}

static int hx9031a_resume(struct device *dev)
{
    ENTER;
    hx9031a_irq_from_suspend_flag = 0;
    hx9031a_state_monitoring(hx9031a_monitor_addr, hx9031a_monitor_val);
    return 0;
}

static struct i2c_device_id hx9031a_i2c_id_table[] = {
    { HX9031A_DRIVER_NAME, 0 },
    { }
};

MODULE_DEVICE_TABLE(i2c, hx9031a_i2c_id_table);
#ifdef CONFIG_OF
static struct of_device_id hx9031a_of_match_table[] = {
#if HX9031A_TEST_ON_MTK_DEMO_XY6761
    {.compatible = "mediatek,sar_hx9031a"},
#else
    {.compatible = "tyhx,hx9031a"},
#endif
    { },
};
#else
#define hx9031a_of_match_table NULL
#endif

static const struct dev_pm_ops hx9031a_pm_ops = {
    .suspend = hx9031a_suspend,
    .resume = hx9031a_resume,
};

static struct i2c_driver hx9031a_i2c_driver = {
    .driver = {
        .owner = THIS_MODULE,
        .name = HX9031A_DRIVER_NAME,
        .of_match_table = hx9031a_of_match_table,
        .pm = &hx9031a_pm_ops,
    },
    .id_table = hx9031a_i2c_id_table,
    .probe = hx9031a_probe,
    .remove = hx9031a_remove,
};

static int __init hx9031a_module_init(void)
{
    ENTER;
    printk(KERN_ERR "lyon driver version:%s\n", HX9031A_DRIVER_VER);
    return i2c_add_driver(&hx9031a_i2c_driver);
}

static void __exit hx9031a_module_exit(void)
{
    ENTER;
    i2c_del_driver(&hx9031a_i2c_driver);
}

module_init(hx9031a_module_init);
module_exit(hx9031a_module_exit);

MODULE_AUTHOR("Yasin Lee <yasin.lee.x@gmail.com><yasin.lee@tianyihexin.com>");
MODULE_DESCRIPTION("Driver for NanJingTianYiHeXin HX9031A & HX9023E Cap Sense");
MODULE_ALIAS("sar driver");
MODULE_LICENSE("GPL");
