#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/pm_runtime.h>
#include <linux/types.h>
#include <linux/unaligned/le_struct.h>
#include <linux/unistd.h>
#include <linux/kernel.h>
#include <linux/delay.h>

#define MFI_I2C_NAME "mfi343s00176"
#define MFI_REG_READ_NUM 10
#define MFI_I2C_RETRIES 2
#define MFI_I2C_RETRY_DELAY 2
#define MFI_DEVICE_ID 0x0300

#define MFI_REG_DEVICE_VERSION				0x00	//1byte ro default 0x07
#define MFI_REG_AUTH_REVISION				0x01	//1byte ro default 0x01
#define MFI_REG_AUTH_PROTO_MAJOR_VERSION		0x02	//1byte ro default 0x03
#define MFI_REG_AUTH_PROTO_MINOR_VERSION		0x03	//1byte ro default 0x00
#define MFI_REG_DEVICE_ID				0x04	//4bytes ro default 0x00 00 03 00
#define MFI_REG_ERROR_CODE				0x05	//1byte ro default 0x00
#define MFI_REG_AUTH_CONTROL_AND_STATUS			0x10	//1byte r/w
#define MFI_REG_CHALLENGE_RESPONSE_DATA_LEN		0x11	//2bytes ro
#define MFI_REG_CHALLENGE_RESPONSE_DATA			0x12	//64bytes ro
#define MFI_REG_CHALLENGE_DATA_LEN			0x20	//2bytes ro	
#define MFI_REG_CHALLENGE_DATA				0x21	//32bytes r/w
#define MFI_REG_CERTIFICATE_RESPONSE_DATA_LEN		0x30	//2bytes ro
#define MFI_REG_CERTIFICATE_DATA1			0x31	//128 bytes ro
#define MFI_REG_CERTIFICATE_DATA2			0x32	//128 bytes ro
#define MFI_REG_CERTIFICATE_DATA3			0x33	//128 bytes ro
#define MFI_REG_CERTIFICATE_DATA4			0x34	//128 bytes ro
#define MFI_REG_CERTIFICATE_DATA5			0x35	//128 bytes ro
#define MFI_REG_SELFTEST_STATUS				0x40	//1byte ro
#define MFI_REG_DEVICE_CERT_SERIAL_NUMBER		0x4E	//32bytes do test
#define MFI_REG_SLEEP					0x60	//1byte wo

#define MFI_REG_LEN_1				1
#define MFI_REG_LEN_2				2
#define MFI_REG_LEN_4				4
#define MFI_REG_LEN_32				32
#define MFI_REG_LEN_64				64
#define MFI_REG_LEN_128				128

static int mfi343s00176_general_reg[] = {
	MFI_REG_DEVICE_VERSION,
	MFI_REG_AUTH_REVISION,
	MFI_REG_AUTH_PROTO_MAJOR_VERSION,
	MFI_REG_AUTH_PROTO_MINOR_VERSION,
	MFI_REG_ERROR_CODE,
	MFI_REG_AUTH_CONTROL_AND_STATUS,
	MFI_REG_SELFTEST_STATUS,
	MFI_REG_CHALLENGE_RESPONSE_DATA_LEN,
	MFI_REG_CHALLENGE_DATA_LEN,
	MFI_REG_CERTIFICATE_RESPONSE_DATA_LEN,
};

static int mfi343s00176_data_reg[] = {
	MFI_REG_DEVICE_ID,
	MFI_REG_AUTH_CONTROL_AND_STATUS,
	MFI_REG_CHALLENGE_RESPONSE_DATA_LEN,
	MFI_REG_CHALLENGE_RESPONSE_DATA,
	MFI_REG_CHALLENGE_DATA_LEN,
	MFI_REG_CHALLENGE_DATA,
	MFI_REG_CERTIFICATE_RESPONSE_DATA_LEN,
	MFI_REG_CERTIFICATE_DATA1,
	MFI_REG_CERTIFICATE_DATA2,
	MFI_REG_CERTIFICATE_DATA3,
	MFI_REG_CERTIFICATE_DATA4,
	MFI_REG_CERTIFICATE_DATA5,
	MFI_REG_DEVICE_CERT_SERIAL_NUMBER,
};

struct mfi343s00176 {
	struct i2c_client *i2c;
	struct device *dev;
};

static int mfi_i2c_write(struct mfi343s00176 *uc, u16 reg_addr, u8 *wr_buf, int wr_len)
{
	int ret = 0;
	u8 u8_buf[129];
	uint8_t retry = 0;

	struct i2c_msg msg[] = {
		{
			.addr = 0x10,
			.flags = 0,
			.len = wr_len + 1,
			.buf = u8_buf,
		},
	};

	u8_buf[0] = reg_addr;
	memcpy(&u8_buf[1], wr_buf, wr_len);

	for(retry = 0; retry < 2; retry++) {
		ret = i2c_transfer(uc->i2c->adapter, msg, 1); 
		if (ret < 0){
			pr_err("%s: MFI I2C retry %d\n", __func__, retry + 1);
		}else if (ret == 1){
			pr_err("%s: MFI I2C retry %d\n", __func__, retry + 1);
			ret = wr_len;
		}
	}

	return ret;
}

static int mfi_i2c_read(struct mfi343s00176 *uc, uint8_t reg_addr, uint8_t *buf, uint32_t rd_len)
{
	int ret = -1;
	uint8_t retry = 0;
	struct i2c_msg msg_wr[] = {
		[0] = {
			.addr = uc->i2c->addr,
			.flags = 0,
			.len = sizeof(uint8_t),
			.buf = &reg_addr,
			},
	};
	struct i2c_msg msg_rd[] = {
		[0] = {
			.addr = uc->i2c->addr,
			.flags = I2C_M_RD,
			.len = rd_len,
			.buf = buf,
			},
	};

	for (retry = 0; retry < 2; retry++) {
		ret = i2c_transfer(uc->i2c->adapter, msg_wr, 1);
		if (ret == 1) {
			pr_err("transfer write success. reg_addr=0x%02X", reg_addr);
			break;
		}
		pr_err("%s: MFI I2C retry %d\n", __func__, retry + 1);
		usleep_range(500,501);	
		if (retry == 2) {
			pr_err("%s: MFI I2C write over retry limit\n", __func__);
			ret = -EIO;
		}
	}

	for (retry = 0; retry < 2; retry++) {
		ret = i2c_transfer(uc->i2c->adapter, msg_rd, 1);
		if (ret == 1) {
			pr_err("transfer read success. reg_addr=0x%02X", reg_addr);
			break;
		}
		pr_err("%s: MFI I2C retry %d\n", __func__, retry + 1);
		usleep_range(500,501);	
		if (retry == 2) {
			pr_err("%s: MFI I2C read over retry limit\n", __func__);
			ret = -EIO;
		}
	}

	return ret;
}

static ssize_t mfi343s00176_reg_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct mfi343s00176 *mfi343s00176 = dev_get_drvdata(dev);
	ssize_t len = 0;
	unsigned int i = 0;
	u8 reg_val[2] = { 0x00 };
	int reg_value = 0;
	int ret = 0;

	for (i = 0; i < MFI_REG_READ_NUM; i++) {
		ret = mfi_i2c_read(mfi343s00176, (u16)mfi343s00176_general_reg[i], reg_val, 2);		
		if (ret < 0)
			reg_value = -EIO;

		len += snprintf(buf + len, PAGE_SIZE - len,
				"reg:0x%04x=0x%04x\n", mfi343s00176_general_reg[i], reg_value);
	}
	return len;
}
//0x00->0x07
static ssize_t mfi343s00176_chipid_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct mfi343s00176 *mfi343s00176 = dev_get_drvdata(dev);
	ssize_t len = 0;
	u8 reg_val[5] = { 0x00 };
	int reg_value = 0;
	int ret = 0;

	ret = mfi_i2c_read(mfi343s00176, MFI_REG_DEVICE_VERSION, &reg_val[0], MFI_REG_LEN_1);

	if (ret < 0) 
		reg_value = -EIO;

	len += snprintf(buf + len, PAGE_SIZE - len, "0x%02x\n", reg_val[0]);
	return len;
}

static ssize_t mfi343s00176_deviceid_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct mfi343s00176 *mfi343s00176 = dev_get_drvdata(dev);
	ssize_t len = 0;
	u8 reg_val[5] = { 0x00 };
	int reg_value = 0;
	int ret = 0;

	ret = mfi_i2c_read(mfi343s00176, MFI_REG_DEVICE_ID, &reg_val[0], MFI_REG_LEN_4);

	if (ret < 0)
		reg_value = -EIO;

	len += snprintf(buf + len, PAGE_SIZE - len, "%02x%02x%02x%02x\n", reg_val[0],reg_val[1],reg_val[2],reg_val[3]);

	return len;
}
//0x10->0x00
static ssize_t mfi343s00176_auth_control_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct mfi343s00176 *mfi343s00176 = dev_get_drvdata(dev);
	ssize_t len = 0;
	u8 reg_val[2] = { 0x00 };
	int reg_value = 0;
	int ret = 0;

	ret = mfi_i2c_read(mfi343s00176, MFI_REG_AUTH_CONTROL_AND_STATUS, &reg_val[0], MFI_REG_LEN_1);//mtk error
	if (ret < 0)
		reg_value = -EIO;

	len += snprintf(buf + len, PAGE_SIZE - len, "0x%02x\n", reg_val[0]);
	return len;
}

static ssize_t mfi343s00176_auth_control_store(struct device *dev,
				 struct device_attribute *attr, const char *buf,
				 size_t count)
{
	struct mfi343s00176 *mfi343s00176 = dev_get_drvdata(dev);
	int value = 0;

	u8 wr_val[2] = { 0x01,0x00 };
	mfi_i2c_write(mfi343s00176, MFI_REG_AUTH_CONTROL_AND_STATUS, wr_val, MFI_REG_LEN_1);
	return count;
}
//0x12->undefined
static ssize_t mfi343s00176_challenge_response_data_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct mfi343s00176 *mfi343s00176 = dev_get_drvdata(dev);
	ssize_t len = 0;
	u8 reg_val[65] = { 0x00 };
	int reg_value = 0;
	int ret = 0;
	int i = 0;
	int k = 0;
	ret = mfi_i2c_read(mfi343s00176, MFI_REG_CHALLENGE_RESPONSE_DATA, &reg_val[0], MFI_REG_LEN_64);
	if (ret < 0)
		reg_value = -EIO;

	for(i=0; i<8; i++){
		for(k=0; k<8; k++){
			len += snprintf(buf + len, PAGE_SIZE - len, "%x", reg_val[i*8+k]);
			if (k == 7) {
				len += snprintf(buf + len, PAGE_SIZE - len, "\n");
			}else{
				len += snprintf(buf + len, PAGE_SIZE - len, "-");
			}
		}
	}

	return len;
}
//0x11->0
static ssize_t mfi343s00176_challenge_response_data_len_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct mfi343s00176 *mfi343s00176 = dev_get_drvdata(dev);
	ssize_t len = 0;
	u8 reg_val[3] = { 0x00 };
	int reg_value = 0;
	int ret = 0;

	ret = mfi_i2c_read(mfi343s00176, MFI_REG_CHALLENGE_RESPONSE_DATA_LEN, &reg_val[0], MFI_REG_LEN_2);
	if (ret < 0)
		reg_value = -EIO;

	len += snprintf(buf + len, PAGE_SIZE - len, "%x\n", reg_val[0]);

	return len;
}
//0x21->undefined
static ssize_t mfi343s00176_challenge_data_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct mfi343s00176 *mfi343s00176 = dev_get_drvdata(dev);
	ssize_t len = 0;
	u8 reg_val[33] = { 0x00 };
	int reg_value = 0;
	int ret = 0;
	int i = 0;

	ret = mfi_i2c_read(mfi343s00176, MFI_REG_CHALLENGE_DATA, &reg_val[0], MFI_REG_LEN_32);
	if (ret < 0)
		reg_value = -EIO;

	for(i=0;i<8;i++){
		len += snprintf(buf + len, PAGE_SIZE - len, "%x-%x-%x-%x\n", reg_val[i*4+0],reg_val[i*4+1],reg_val[i*4+2],reg_val[i*4+3]);
	}
	return len;
}

static ssize_t mfi343s00176_challenge_data_store(struct device *dev,
				 struct device_attribute *attr, const char *buf,
				 size_t count)
{
	struct mfi343s00176 *mfi343s00176 = dev_get_drvdata(dev);
	u8 wr_val[33] = { 0 };
	int i = 0;

	for(i = 0; i < MFI_REG_LEN_32 + 1; i++){
		wr_val[i] = i;
	}

	mfi_i2c_write(mfi343s00176, MFI_REG_CHALLENGE_DATA, wr_val, MFI_REG_LEN_32);
	return count;
}

//0x20->0x00
static ssize_t mfi343s00176_challenge_data_len_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct mfi343s00176 *mfi343s00176 = dev_get_drvdata(dev);
	ssize_t len = 0;
	u8 reg_val[3] = { 0x00 };
	int reg_value = 0;
	int ret = 0;

	ret = mfi_i2c_read(mfi343s00176, MFI_REG_CHALLENGE_DATA_LEN, &reg_val[0], MFI_REG_LEN_2);
	if (ret < 0)
		reg_value = -EIO;

	len += snprintf(buf + len, PAGE_SIZE - len, "0x%02x\n", reg_val[0]);

	return len;
}

//0x31->undefined
static ssize_t mfi343s00176_certificate_data1_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct mfi343s00176 *mfi343s00176 = dev_get_drvdata(dev);
	ssize_t len = 0;
	u8 reg_val[129] = { 0x00 };
	int reg_value = 0;
	int ret = 0;
	int i = 0;
	int k = 0;

	ret = mfi_i2c_read(mfi343s00176, MFI_REG_CERTIFICATE_DATA1, &reg_val[0], MFI_REG_LEN_128);
	if (ret < 0)
		reg_value = -EIO;

	for(i=0; i<16; i++){
		for(k=0; k<8; k++){
			len += snprintf(buf + len, PAGE_SIZE - len, "%x", reg_val[i*8+k]);
			if (k == 7) {
				len += snprintf(buf + len, PAGE_SIZE - len, "\n");
			}else{
				len += snprintf(buf + len, PAGE_SIZE - len, "-");
			}
		}
	}

	return len;
}

static ssize_t mfi343s00176_certificate_data2_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct mfi343s00176 *mfi343s00176 = dev_get_drvdata(dev);
	ssize_t len = 0;
	u8 reg_val[129] = { 0x00 };
	int reg_value = 0;
	int ret = 0;
	int i = 0;
	int k = 0;
	ret = mfi_i2c_read(mfi343s00176, MFI_REG_CERTIFICATE_DATA2, &reg_val[0], MFI_REG_LEN_128);
	if (ret < 0)
		reg_value = -EIO;

	for(i=0; i<16; i++){
		for(k=0; k<8; k++){
			len += snprintf(buf + len, PAGE_SIZE - len, "%x", reg_val[i*8+k]);
			if (k == 7) {
				len += snprintf(buf + len, PAGE_SIZE - len, "\n");
			}else{
				len += snprintf(buf + len, PAGE_SIZE - len, "-");
			}
		}
	}

	return len;
}

static ssize_t mfi343s00176_certificate_data3_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct mfi343s00176 *mfi343s00176 = dev_get_drvdata(dev);
	ssize_t len = 0;
	u8 reg_val[129] = { 0x00 };
	int reg_value = 0;
	int ret = 0;
	int i = 0;
	int k = 0;

	ret = mfi_i2c_read(mfi343s00176, MFI_REG_CERTIFICATE_DATA3, &reg_val[0], MFI_REG_LEN_128);
	if (ret < 0)
		reg_value = -EIO;

	for(i=0; i<16; i++){
		for(k=0; k<8; k++){
			len += snprintf(buf + len, PAGE_SIZE - len, "%x", reg_val[i*8+k]);
			if (k == 7) {
				len += snprintf(buf + len, PAGE_SIZE - len, "\n");
			}else{
				len += snprintf(buf + len, PAGE_SIZE - len, "-");
			}
		}
	}

	return len;
}

static ssize_t mfi343s00176_certificate_data4_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct mfi343s00176 *mfi343s00176 = dev_get_drvdata(dev);
	ssize_t len = 0;
	u8 reg_val[129] = { 0x00 };
	int reg_value = 0;
	int ret = 0;
	int i=0;
	int k=0;

	ret = mfi_i2c_read(mfi343s00176, MFI_REG_CERTIFICATE_DATA4, &reg_val[0], MFI_REG_LEN_128);

	if (ret < 0)
		reg_value = -EIO;

	for(i=0; i<16; i++){
		for(k=0; k<8; k++){
			len += snprintf(buf + len, PAGE_SIZE - len, "%x", reg_val[i*8+k]);
			if (k == 7) {
				len += snprintf(buf + len, PAGE_SIZE - len, "\n");
			}else{
				len += snprintf(buf + len, PAGE_SIZE - len, "-");
			}
		}
	}
	return len;
}

static ssize_t mfi343s00176_certificate_data5_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct mfi343s00176 *mfi343s00176 = dev_get_drvdata(dev);
	ssize_t len = 0;
	u8 reg_val[129] = { 0x00 };
	int reg_value = 0;
	int ret = 0;
	int i = 0;
	int k = 0;

	ret = mfi_i2c_read(mfi343s00176, MFI_REG_CERTIFICATE_DATA5, &reg_val[0], MFI_REG_LEN_128);

	if (ret < 0)
		reg_value = -EIO;

	for(i=0; i<16; i++){
		for(k=0; k<8; k++){
			len += snprintf(buf + len, PAGE_SIZE - len, "%x", reg_val[i*8+k]);
			if (k == 7) {
				len += snprintf(buf + len, PAGE_SIZE - len, "\n");
			}else{
				len += snprintf(buf + len, PAGE_SIZE - len, "-");
			}
		}
	}

	return len;
}
//0x30->607-609
static ssize_t mfi343s00176_certificate_data_len_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct mfi343s00176 *mfi343s00176 = dev_get_drvdata(dev);
	ssize_t len = 0;
	u8 reg_val[3] = { 0x00 };
	int reg_value = 0;
	int ret = 0;

	ret = mfi_i2c_read(mfi343s00176, MFI_REG_CERTIFICATE_RESPONSE_DATA_LEN, &reg_val[0], MFI_REG_LEN_2);

	if (ret < 0)
		reg_value = -EIO;

	len += snprintf(buf + len, PAGE_SIZE - len, "0x%x%x\n", reg_val[0],reg_val[1]);

	return len;
}
//0x4E->certificate
static ssize_t mfi343s00176_device_cert_serial_number_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct mfi343s00176 *mfi343s00176 = dev_get_drvdata(dev);
	ssize_t len = 0;
	u8 reg_val[33] = { 0x00 };
	int reg_value = 0;
	int ret = 0;
	int i = 0;

	ret = mfi_i2c_read(mfi343s00176, MFI_REG_DEVICE_CERT_SERIAL_NUMBER, &reg_val[0], MFI_REG_LEN_32);

	if (ret < 0)
		reg_value = -EIO;

	for(i=0;i<8;i++){
		len += snprintf(buf + len, PAGE_SIZE - len, "%x-%x-%x-%x\n", reg_val[i*4+0],reg_val[i*4+1],reg_val[i*4+2],reg_val[i*4+3]);
	}

	return len;
}

static DEVICE_ATTR(reg, S_IWUSR | S_IRUGO, mfi343s00176_reg_show, NULL);
static DEVICE_ATTR(chipid, 0444, mfi343s00176_chipid_show, NULL);
static DEVICE_ATTR(deviceid, 0444, mfi343s00176_deviceid_show, NULL);

static DEVICE_ATTR(auth_control, S_IWUSR | S_IRUGO, mfi343s00176_auth_control_show, mfi343s00176_auth_control_store);

static DEVICE_ATTR(challenge_response_data, S_IWUSR | S_IRUGO, mfi343s00176_challenge_response_data_show, NULL);
static DEVICE_ATTR(challenge_response_data_len, S_IWUSR | S_IRUGO, mfi343s00176_challenge_response_data_len_show, NULL);

static DEVICE_ATTR(challenge_data, S_IWUSR | S_IRUGO, mfi343s00176_challenge_data_show, mfi343s00176_challenge_data_store);
static DEVICE_ATTR(challenge_data_len, S_IWUSR | S_IRUGO, mfi343s00176_challenge_data_len_show, NULL);

static DEVICE_ATTR(certificate_data1, S_IWUSR | S_IRUGO, mfi343s00176_certificate_data1_show, NULL);
static DEVICE_ATTR(certificate_data2, S_IWUSR | S_IRUGO, mfi343s00176_certificate_data2_show, NULL);
static DEVICE_ATTR(certificate_data3, S_IWUSR | S_IRUGO, mfi343s00176_certificate_data3_show, NULL);
static DEVICE_ATTR(certificate_data4, S_IWUSR | S_IRUGO, mfi343s00176_certificate_data4_show, NULL);
static DEVICE_ATTR(certificate_data5, S_IWUSR | S_IRUGO, mfi343s00176_certificate_data5_show, NULL);
static DEVICE_ATTR(certificate_data_len, S_IWUSR | S_IRUGO, mfi343s00176_certificate_data_len_show, NULL);

static DEVICE_ATTR(device_cert_serial_number, S_IWUSR | S_IRUGO, mfi343s00176_device_cert_serial_number_show, NULL);

static struct attribute *mfi343s00176_attributes[] = {
	&dev_attr_reg.attr,
	&dev_attr_chipid.attr,
	&dev_attr_deviceid.attr,
	&dev_attr_auth_control.attr,
	&dev_attr_challenge_response_data.attr,
	&dev_attr_challenge_response_data_len.attr,
	&dev_attr_challenge_data.attr,
	&dev_attr_challenge_data_len.attr,
	&dev_attr_certificate_data1.attr,
	&dev_attr_certificate_data2.attr,
	&dev_attr_certificate_data3.attr,
	&dev_attr_certificate_data4.attr,
	&dev_attr_certificate_data5.attr,
	&dev_attr_certificate_data_len.attr,
	&dev_attr_device_cert_serial_number.attr,						
	NULL
};

static struct attribute_group mfi343s00176_attribute_group = {
	.attrs = mfi343s00176_attributes
};

static int mfi343s00176_i2c_probe(struct i2c_client *i2c,
			     const struct i2c_device_id *id)
{
	struct mfi343s00176 *mfi343s00176;
	int ret;

	pr_err("%s: enter\n", __func__);

	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C)) {
		dev_err(&i2c->dev, "check_functionality failed\n");
		return -EIO;
	}

	mfi343s00176 = devm_kzalloc(&i2c->dev, sizeof(struct mfi343s00176), GFP_KERNEL);
	if (mfi343s00176 == NULL)
		return -ENOMEM;

	mfi343s00176->dev = &i2c->dev;
	mfi343s00176->i2c = i2c;
	i2c_set_clientdata(i2c, mfi343s00176);
	dev_set_drvdata(&i2c->dev, mfi343s00176);

	ret = sysfs_create_group(&i2c->dev.kobj, &mfi343s00176_attribute_group);
	if (ret < 0) {
		dev_err(mfi343s00176->dev, "Can't create sysfs entries\n");
		return ret;
	}

	pr_info("%s probe completed successfully!\n", __func__);
	return 0;
}

static int mfi343s00176_remove(struct i2c_client *i2c)
{
	struct mfi343s00176 *mfi343s00176 = i2c_get_clientdata(i2c);

	pr_info("%s: enter\n", __func__);
	sysfs_remove_group(&i2c->dev.kobj, &mfi343s00176_attribute_group);
	devm_kfree(&i2c->dev, mfi343s00176);
	mfi343s00176 = NULL;

	return 0;
}

static const struct i2c_device_id mfi343s00176_i2c_id[] = {
	{MFI_I2C_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, mfi343s00176_i2c_id);

static const struct of_device_id of_mfi343s00176_match[] = {
	{.compatible = "mfi,mfi343s00176"},
	{},
};
MODULE_DEVICE_TABLE(of, of_mfi343s00176_match);

static struct i2c_driver mfi343s00176_driver = {
	.driver = {
		.name	= "mfi343s00176",
		.of_match_table = of_mfi343s00176_match,
	},
	.probe		= mfi343s00176_i2c_probe,
	.remove		= mfi343s00176_remove,
	.id_table	= mfi343s00176_i2c_id,
};
module_i2c_driver(mfi343s00176_driver);

MODULE_DESCRIPTION("mfi343s00176 Driver");
MODULE_LICENSE("GPL v2");
