// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2018-19 Linaro Limited
/* Copyright (c) 2021, The Linux Foundation. All rights reserved. */

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/phy.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mii.h>
#include <linux/of_mdio.h>
#include <linux/slab.h>
#include <linux/poll.h>
#include <linux/debugfs.h>
#include <linux/dma-iommu.h>
#include <linux/iommu.h>
#include <linux/micrel_phy.h>
#include <linux/tcp.h>
#include <linux/ip.h>
#include <linux/ipv6.h>
#include <linux/rtnetlink.h>
#include <asm-generic/io.h>
#include <linux/kthread.h>
#include "stmmac.h"
#include "stmmac_platform.h"
#include "dwmac-qcom-ethqos.h"
#include "stmmac_ptp.h"

#define RGMII_IO_MACRO_DEBUG1		0x20
#define EMAC_SYSTEM_LOW_POWER_DEBUG	0x28

/* RGMII_IO_MACRO_CONFIG fields */
#define RGMII_CONFIG_FUNC_CLK_EN		BIT(30)
#define RGMII_CONFIG_POS_NEG_DATA_SEL		BIT(23)
#define RGMII_CONFIG_GPIO_CFG_RX_INT		GENMASK(21, 20)
#define RGMII_CONFIG_GPIO_CFG_TX_INT		GENMASK(19, 17)
#define RGMII_CONFIG_MAX_SPD_PRG_9		GENMASK(16, 8)
#define RGMII_CONFIG_MAX_SPD_PRG_2		GENMASK(7, 6)
#define RGMII_CONFIG_INTF_SEL			GENMASK(5, 4)
#define RGMII_CONFIG_BYPASS_TX_ID_EN		BIT(3)
#define RGMII_CONFIG_LOOPBACK_EN		BIT(2)
#define RGMII_CONFIG_PROG_SWAP			BIT(1)
#define RGMII_CONFIG_DDR_MODE			BIT(0)

/*RGMII DLL CONFIG*/
#define HSR_DLL_CONFIG					0x000B642C
#define HSR_DLL_CONFIG_2					0xA001
#define HSR_MACRO_CONFIG_2					0x01
#define HSR_DLL_TEST_CTRL					0x1400000
#define HSR_DDR_CONFIG					0x80040868
#define HSR_SDCC_USR_CTRL					0x2C010800
#define MACRO_CONFIG_2_MASK				GENMASK(24, 17)
#define	DLL_CONFIG_2_MASK				GENMASK(22, 0)
#define HSR_SDCC_DLL_TEST_CTRL				0x1800000
#define DDR_CONFIG_PRG_RCLK_DLY			        115
#define DLL_BYPASS					BIT(30)

/* SDCC_HC_REG_DLL_CONFIG fields */
#define SDCC_DLL_CONFIG_DLL_RST			BIT(30)
#define SDCC_DLL_CONFIG_PDN			BIT(29)
#define SDCC_DLL_CONFIG_MCLK_FREQ		GENMASK(26, 24)
#define SDCC_DLL_CONFIG_CDR_SELEXT		GENMASK(23, 20)
#define SDCC_DLL_CONFIG_CDR_EXT_EN		BIT(19)
#define SDCC_DLL_CONFIG_CK_OUT_EN		BIT(18)
#define SDCC_DLL_CONFIG_CDR_EN			BIT(17)
#define SDCC_DLL_CONFIG_DLL_EN			BIT(16)
#define SDCC_DLL_MCLK_GATING_EN			BIT(5)
#define SDCC_DLL_CDR_FINE_PHASE			GENMASK(3, 2)

/* SDCC_HC_REG_DDR_CONFIG fields */
#define SDCC_DDR_CONFIG_PRG_DLY_EN		BIT(31)
#define SDCC_DDR_CONFIG_EXT_PRG_RCLK_DLY	GENMASK(26, 21)
#define SDCC_DDR_CONFIG_EXT_PRG_RCLK_DLY_CODE	GENMASK(29, 27)
#define SDCC_DDR_CONFIG_EXT_PRG_RCLK_DLY_EN	BIT(30)
#define SDCC_DDR_CONFIG_PRG_RCLK_DLY		GENMASK(8, 0)
#define SDCC_DDR_CONFIG_TCXO_CYCLES_DLY_LINE    GENMASK(20, 12)
#define SDCC_DDR_CONFIG_TCXO_CYCLES_CNT		GENMASK(11, 9)

/* SDCC_HC_REG_DLL_CONFIG2 fields */
#define SDCC_DLL_CONFIG2_DLL_CLOCK_DIS		BIT(21)
#define SDCC_DLL_CONFIG2_MCLK_FREQ_CALC		GENMASK(17, 10)
#define SDCC_DLL_CONFIG2_DDR_TRAFFIC_INIT_SEL	GENMASK(3, 2)
#define SDCC_DLL_CONFIG2_DDR_TRAFFIC_INIT_SW	BIT(1)
#define SDCC_DLL_CONFIG2_DDR_CAL_EN		BIT(0)

/* SDC4_STATUS bits */
#define SDC4_STATUS_DLL_LOCK			BIT(7)

/* RGMII_IO_MACRO_CONFIG2 fields */
#define RGMII_CONFIG2_RSVD_CONFIG15		GENMASK(31, 17)
#define RGMII_CONFIG2_RGMII_CLK_SEL_CFG		BIT(16)
#define RGMII_CONFIG2_TX_TO_RX_LOOPBACK_EN	BIT(13)
#define RGMII_CONFIG2_CLK_DIVIDE_SEL		BIT(12)
#define RGMII_CONFIG2_RX_PROG_SWAP		BIT(7)
#define RGMII_CONFIG2_DATA_DIVIDE_CLK_SEL	BIT(6)
#define RGMII_CONFIG2_TX_CLK_PHASE_SHIFT_EN	BIT(5)

#define EMAC_I0_EMAC_CORE_HW_VERSION_RGOFFADDR 0x00000070
#define EMAC_HW_v2_3_2_RG 0x20030002

#define MII_BUSY 0x00000001
#define MII_WRITE 0x00000002

/* GMAC4 defines */
#define MII_GMAC4_GOC_SHIFT		2
#define MII_GMAC4_WRITE			BIT(MII_GMAC4_GOC_SHIFT)
#define MII_GMAC4_READ			(3 << MII_GMAC4_GOC_SHIFT)

#define MII_BUSY 0x00000001
#define MII_WRITE 0x00000002

#define DWC_ETH_QOS_PHY_INTR_STATUS     0x0013

#define LINK_UP 1
#define LINK_DOWN 0

#define LINK_DOWN_STATE 0x800
#define LINK_UP_STATE 0x400

#define MICREL_PHY_ID PHY_ID_KSZ9031
#define DWC_ETH_QOS_MICREL_PHY_INTCS 0x1b
#define DWC_ETH_QOS_MICREL_PHY_CTL 0x1f
#define DWC_ETH_QOS_MICREL_INTR_LEVEL 0x4000
#define DWC_ETH_QOS_BASIC_STATUS     0x0001
#define LINK_STATE_MASK 0x4
#define AUTONEG_STATE_MASK 0x20
#define MICREL_LINK_UP_INTR_STATUS BIT(0)

void *ipc_emac_log_ctxt;

static struct emac_emb_smmu_cb_ctx emac_emb_smmu_ctx = {0};
struct plat_stmmacenet_data *plat_dat;
static struct qcom_ethqos *pethqos;

static unsigned char dev_addr[ETH_ALEN] = {
	0, 0x55, 0x7b, 0xb5, 0x7d, 0xf7};
static struct ip_params pparams;

struct qcom_ethqos *get_pethqos(void)
{
	return pethqos;
}

static inline unsigned int dwmac_qcom_get_eth_type(unsigned char *buf)
{
	return
		((((u16)buf[QTAG_ETH_TYPE_OFFSET] << 8) |
		  buf[QTAG_ETH_TYPE_OFFSET + 1]) == ETH_P_8021Q) ?
		(((u16)buf[QTAG_VLAN_ETH_TYPE_OFFSET] << 8) |
		 buf[QTAG_VLAN_ETH_TYPE_OFFSET + 1]) :
		 (((u16)buf[QTAG_ETH_TYPE_OFFSET] << 8) |
		  buf[QTAG_ETH_TYPE_OFFSET + 1]);
}

static inline unsigned int dwmac_qcom_get_vlan_ucp(unsigned char  *buf)
{
	return
		(((u16)buf[QTAG_UCP_FIELD_OFFSET] << 8)
		 | buf[QTAG_UCP_FIELD_OFFSET + 1]);
}

static struct ethqos_emac_por emac_por[] = {
	{ .offset = RGMII_IO_MACRO_CONFIG,	.value = 0x0 },
	{ .offset = SDCC_HC_REG_DLL_CONFIG,	.value = 0x0 },
	{ .offset = SDCC_HC_REG_DDR_CONFIG,	.value = 0x0 },
	{ .offset = SDCC_HC_REG_DLL_CONFIG2,	.value = 0x0 },
	{ .offset = SDCC_USR_CTL,		.value = 0x0 },
	{ .offset = RGMII_IO_MACRO_CONFIG2,	.value = 0x0},
};

static struct ethqos_emac_driver_data emac_por_data = {
	.por = emac_por,
	.num_por = ARRAY_SIZE(emac_por),
};

static void qcom_ethqos_read_iomacro_por_values(struct qcom_ethqos *ethqos)
{
	int i;

	ethqos->por = emac_por_data.por;
	ethqos->num_por = emac_por_data.num_por;

	/* Read to POR values and enable clk */
	for (i = 0; i < ethqos->num_por; i++)
		ethqos->por[i].value =
			readl_relaxed
			(ethqos->rgmii_base + ethqos->por[i].offset);
}

u16 dwmac_qcom_select_queue(struct net_device *dev,
			    struct sk_buff *skb,
			    struct net_device *sb_dev)
{
	u16 txqueue_select = ALL_OTHER_TRAFFIC_TX_CHANNEL;
	unsigned int eth_type, priority;

	/* Retrieve ETH type */
	eth_type = dwmac_qcom_get_eth_type(skb->data);

	if (eth_type == ETH_P_TSN) {
		/* Read VLAN priority field from skb->data */
		priority = dwmac_qcom_get_vlan_ucp(skb->data);

		priority >>= VLAN_TAG_UCP_SHIFT;
		if (priority == CLASS_A_TRAFFIC_UCP)
			txqueue_select = CLASS_A_TRAFFIC_TX_CHANNEL;
		else if (priority == CLASS_B_TRAFFIC_UCP)
			txqueue_select = CLASS_B_TRAFFIC_TX_CHANNEL;
		else
			txqueue_select = ALL_OTHER_TX_TRAFFIC_IPA_DISABLED;
	} else {
		/* VLAN tagged IP packet or any other non vlan packets (PTP)*/
		txqueue_select = ALL_OTHER_TX_TRAFFIC_IPA_DISABLED;
	}

	ETHQOSDBG("tx_queue %d\n", txqueue_select);
	return txqueue_select;
}

void dwmac_qcom_program_avb_algorithm(struct stmmac_priv *priv,
				      struct ifr_data_struct *req)
{
	struct dwmac_qcom_avb_algorithm l_avb_struct, *u_avb_struct =
		(struct dwmac_qcom_avb_algorithm *)req->ptr;
	struct dwmac_qcom_avb_algorithm_params *avb_params;

	ETHQOSDBG("\n");

	if (copy_from_user(&l_avb_struct, (void __user *)u_avb_struct,
			   sizeof(struct dwmac_qcom_avb_algorithm))) {
		ETHQOSERR("Failed to fetch AVB Struct\n");
		return;
	}

	if (priv->speed == SPEED_1000)
		avb_params = &l_avb_struct.speed1000params;
	else
		avb_params = &l_avb_struct.speed100params;

	/* Application uses 1 for CLASS A traffic and
	 * 2 for CLASS B traffic
	 * Configure right channel accordingly
	 */
	if (l_avb_struct.qinx == 1) {
		l_avb_struct.qinx = CLASS_A_TRAFFIC_TX_CHANNEL;
	} else if (l_avb_struct.qinx == 2) {
		l_avb_struct.qinx = CLASS_B_TRAFFIC_TX_CHANNEL;
	} else {
		ETHQOSERR("Invalid index [%u] in AVB struct from user\n",
			  l_avb_struct.qinx);
		return;
	}

	priv->plat->tx_queues_cfg[l_avb_struct.qinx].mode_to_use =
		MTL_QUEUE_AVB;
	priv->plat->tx_queues_cfg[l_avb_struct.qinx].send_slope =
		avb_params->send_slope,
	priv->plat->tx_queues_cfg[l_avb_struct.qinx].idle_slope =
		avb_params->idle_slope,
	priv->plat->tx_queues_cfg[l_avb_struct.qinx].high_credit =
		avb_params->hi_credit,
	priv->plat->tx_queues_cfg[l_avb_struct.qinx].low_credit =
		avb_params->low_credit,

	priv->hw->mac->config_cbs(priv->hw,
	priv->plat->tx_queues_cfg[l_avb_struct.qinx].send_slope,
	   priv->plat->tx_queues_cfg[l_avb_struct.qinx].idle_slope,
	   priv->plat->tx_queues_cfg[l_avb_struct.qinx].high_credit,
	   priv->plat->tx_queues_cfg[l_avb_struct.qinx].low_credit,
	   l_avb_struct.qinx);

	ETHQOSDBG("\n");
}

unsigned int dwmac_qcom_get_plat_tx_coal_frames(struct sk_buff *skb)
{
	unsigned int eth_type;

	eth_type = dwmac_qcom_get_eth_type(skb->data);

#ifdef CONFIG_PTPSUPPORT_OBJ
	if (eth_type == ETH_P_1588)
		return PTP_INT_MOD;
#endif

	if (eth_type == ETH_P_TSN)
		return AVB_INT_MOD;
	if (eth_type == ETH_P_IP || eth_type == ETH_P_IPV6) {
#ifdef CONFIG_PTPSUPPORT_OBJ
		bool is_udp = (((eth_type == ETH_P_IP) &&
				   (ip_hdr(skb)->protocol ==
					IPPROTO_UDP)) ||
				  ((eth_type == ETH_P_IPV6) &&
				   (ipv6_hdr(skb)->nexthdr ==
					IPPROTO_UDP)));

		if (is_udp && ((udp_hdr(skb)->dest ==
			htons(PTP_UDP_EV_PORT)) ||
			(udp_hdr(skb)->dest ==
			  htons(PTP_UDP_GEN_PORT))))
			return PTP_INT_MOD;
#endif
		return IP_PKT_INT_MOD;
	}
	return DEFAULT_INT_MOD;
}

static int ethqos_handle_prv_ioctl(struct net_device *dev, struct ifreq *ifr, int cmd)
{
	struct stmmac_priv *pdata = netdev_priv(dev);
	struct ifr_data_struct req;
	struct pps_cfg eth_pps_cfg;
	int ret = 0;

	if (copy_from_user(&req, ifr->ifr_ifru.ifru_data,
			   sizeof(struct ifr_data_struct)))
		return -EFAULT;

	switch (req.cmd) {
	case ETHQOS_CONFIG_PPSOUT_CMD:
		if (copy_from_user(&eth_pps_cfg, (void __user *)req.ptr,
				   sizeof(struct pps_cfg)))
			return -EFAULT;

		if (eth_pps_cfg.ppsout_ch < 0 ||
		    eth_pps_cfg.ppsout_ch >= pdata->dma_cap.pps_out_num)
			ret = -EOPNOTSUPP;
		else if ((eth_pps_cfg.ppsout_align == 1) &&
			 ((eth_pps_cfg.ppsout_ch != DWC_ETH_QOS_PPS_CH_0) &&
			 (eth_pps_cfg.ppsout_ch != DWC_ETH_QOS_PPS_CH_3)))
			ret = -EOPNOTSUPP;
		else
			ret = ppsout_config(pdata, &eth_pps_cfg);
		break;
	case ETHQOS_AVB_ALGORITHM:
		dwmac_qcom_program_avb_algorithm(pdata, &req);
		break;
	default:
		break;
	}
	return ret;
}

#ifndef MODULE
static int __init set_early_ethernet_ipv4(char *ipv4_addr_in)
{
	int ret = 1;

	pparams.is_valid_ipv4_addr = false;

	if (!ipv4_addr_in)
		return ret;

	strlcpy(pparams.ipv4_addr_str,
		ipv4_addr_in, sizeof(pparams.ipv4_addr_str));
	ETHQOSDBG("Early ethernet IPv4 addr: %s\n", pparams.ipv4_addr_str);

	ret = in4_pton(pparams.ipv4_addr_str, -1,
		       (u8 *)&pparams.ipv4_addr.s_addr, -1, NULL);
	if (ret != 1 || pparams.ipv4_addr.s_addr == 0) {
		ETHQOSERR("Invalid ipv4 address programmed: %s\n",
			  ipv4_addr_in);
		return ret;
	}

	pparams.is_valid_ipv4_addr = true;
	return ret;
}

__setup("eipv4=", set_early_ethernet_ipv4);

static int __init set_early_ethernet_ipv6(char *ipv6_addr_in)
{
	int ret = 1;

	pparams.is_valid_ipv6_addr = false;

	if (!ipv6_addr_in)
		return ret;

	strlcpy(pparams.ipv6_addr_str,
		ipv6_addr_in, sizeof(pparams.ipv6_addr_str));
	ETHQOSDBG("Early ethernet IPv6 addr: %s\n", pparams.ipv6_addr_str);

	ret = in6_pton(pparams.ipv6_addr_str, -1,
		       (u8 *)&pparams.ipv6_addr.ifr6_addr.s6_addr32, -1, NULL);
	if (ret != 1 || !pparams.ipv6_addr.ifr6_addr.s6_addr32)  {
		ETHQOSERR("Invalid ipv6 address programmed: %s\n",
			  ipv6_addr_in);
		return ret;
	}

	pparams.is_valid_ipv6_addr = true;
	return ret;
}

__setup("eipv6=", set_early_ethernet_ipv6);

static int __init set_early_ethernet_mac(char *mac_addr)
{
	bool valid_mac = false;

	pparams.is_valid_mac_addr = false;
	if (!mac_addr)
		return 1;

	valid_mac = mac_pton(mac_addr, pparams.mac_addr);
	if (!valid_mac)
		goto fail;

	valid_mac = is_valid_ether_addr(pparams.mac_addr);
	if (!valid_mac)
		goto fail;

	pparams.is_valid_mac_addr = true;
	return 0;

fail:
	ETHQOSERR("Invalid Mac address programmed: %s\n", mac_addr);
	return 1;
}

__setup("ermac=", set_early_ethernet_mac);
#endif

static int qcom_ethqos_add_ipaddr(struct ip_params *ip_info,
				  struct net_device *dev)
{
	int res = 0;
	struct ifreq ir;
	struct sockaddr_in *sin = (void *)&ir.ifr_ifru.ifru_addr;
	struct net *net = dev_net(dev);

	if (!net || !net->genl_sock || !net->genl_sock->sk_socket) {
		ETHQOSINFO("Sock is null, unable to assign ipv4 address\n");
		return res;
	}

	if (!net->ipv4.devconf_dflt) {
		ETHQOSERR("ipv4.devconf_dflt is null, schedule wq\n");
		schedule_delayed_work(&pethqos->ipv4_addr_assign_wq,
				      msecs_to_jiffies(1000));
		return res;
	}
	/*For valid Ipv4 address*/
	memset(&ir, 0, sizeof(ir));
	memcpy(&sin->sin_addr.s_addr, &ip_info->ipv4_addr,
	       sizeof(sin->sin_addr.s_addr));

	strlcpy(ir.ifr_ifrn.ifrn_name,
		dev->name, sizeof(ir.ifr_ifrn.ifrn_name));
	sin->sin_family = AF_INET;
	sin->sin_port = 0;

	res = inet_ioctl(net->genl_sock->sk_socket,
			 SIOCSIFADDR, (unsigned long)(void *)&ir);
		if (res) {
			ETHQOSERR("can't setup IPv4 address!: %d\r\n", res);
		} else {
			ETHQOSINFO("Assigned IPv4 address: %s\r\n",
				   ip_info->ipv4_addr_str);
#ifdef CONFIG_QGKI_MSM_BOOT_TIME_MARKER
place_marker("M - Etherent Assigned IPv4 address");
#endif
		}
	return res;
}

#ifdef CONFIG_IPV6
static int qcom_ethqos_add_ipv6addr(struct ip_params *ip_info,
				    struct net_device *dev)
{
	int ret = -EFAULT;
	struct in6_ifreq ir6;
	char *prefix;
	struct net *net = dev_net(dev);
	/*For valid IPv6 address*/

	if (!net || !net->genl_sock || !net->genl_sock->sk_socket) {
		ETHQOSERR("Sock is null, unable to assign ipv6 address\n");
		return -EFAULT;
	}

	if (!net->ipv6.devconf_dflt) {
		ETHQOSERR("ipv6.devconf_dflt is null, schedule wq\n");
		schedule_delayed_work(&pethqos->ipv6_addr_assign_wq,
				      msecs_to_jiffies(1000));
		return ret;
	}
	memset(&ir6, 0, sizeof(ir6));
	memcpy(&ir6, &ip_info->ipv6_addr, sizeof(struct in6_ifreq));
	ir6.ifr6_ifindex = dev->ifindex;

	prefix = strnchr(ip_info->ipv6_addr_str,
			 strlen(ip_info->ipv6_addr_str), '/');

	if (!prefix) {
		ir6.ifr6_prefixlen = 0;
	} else {
		kstrtoul(prefix + 1, 0, (unsigned long *)&ir6.ifr6_prefixlen);
		if (ir6.ifr6_prefixlen > 128)
			ir6.ifr6_prefixlen = 0;
	}
	ret = inet6_ioctl(net->genl_sock->sk_socket,
			  SIOCSIFADDR, (unsigned long)(void *)&ir6);
		if (ret) {
			ETHQOSDBG("Can't setup IPv6 address!\r\n");
		} else {
			ETHQOSDBG("Assigned IPv6 address: %s\r\n",
				  ip_info->ipv6_addr_str);
#ifdef CONFIG_QGKI_MSM_BOOT_TIME_MARKER
		place_marker("M - Ethernet Assigned IPv6 address");
#endif
		}
	return ret;
}
#endif

static int rgmii_readl(struct qcom_ethqos *ethqos, unsigned int offset)
{
	return readl(ethqos->rgmii_base + offset);
}

static void rgmii_writel(struct qcom_ethqos *ethqos,
			 int value, unsigned int offset)
{
	writel(value, ethqos->rgmii_base + offset);
}

static void rgmii_updatel(struct qcom_ethqos *ethqos,
			  int mask, int val, unsigned int offset)
{
	unsigned int temp;

	temp =  rgmii_readl(ethqos, offset);
	temp = (temp & ~(mask)) | val;
	rgmii_writel(ethqos, temp, offset);
}

static void rgmii_dump(struct qcom_ethqos *ethqos)
{
	dev_dbg(&ethqos->pdev->dev, "Rgmii register dump\n");
	dev_dbg(&ethqos->pdev->dev, "RGMII_IO_MACRO_CONFIG: %x\n",
		rgmii_readl(ethqos, RGMII_IO_MACRO_CONFIG));
	dev_dbg(&ethqos->pdev->dev, "SDCC_HC_REG_DLL_CONFIG: %x\n",
		rgmii_readl(ethqos, SDCC_HC_REG_DLL_CONFIG));
	dev_dbg(&ethqos->pdev->dev, "SDCC_HC_REG_DDR_CONFIG: %x\n",
		rgmii_readl(ethqos, SDCC_HC_REG_DDR_CONFIG));
	dev_dbg(&ethqos->pdev->dev, "SDCC_HC_REG_DLL_CONFIG2: %x\n",
		rgmii_readl(ethqos, SDCC_HC_REG_DLL_CONFIG2));
	dev_dbg(&ethqos->pdev->dev, "SDC4_STATUS: %x\n",
		rgmii_readl(ethqos, SDC4_STATUS));
	dev_dbg(&ethqos->pdev->dev, "SDCC_USR_CTL: %x\n",
		rgmii_readl(ethqos, SDCC_USR_CTL));
	dev_dbg(&ethqos->pdev->dev, "RGMII_IO_MACRO_CONFIG2: %x\n",
		rgmii_readl(ethqos, RGMII_IO_MACRO_CONFIG2));
	dev_dbg(&ethqos->pdev->dev, "RGMII_IO_MACRO_DEBUG1: %x\n",
		rgmii_readl(ethqos, RGMII_IO_MACRO_DEBUG1));
	dev_dbg(&ethqos->pdev->dev, "EMAC_SYSTEM_LOW_POWER_DEBUG: %x\n",
		rgmii_readl(ethqos, EMAC_SYSTEM_LOW_POWER_DEBUG));
}

/* Clock rates */
#define RGMII_1000_NOM_CLK_FREQ			(250 * 1000 * 1000UL)
#define RGMII_ID_MODE_100_LOW_SVS_CLK_FREQ	 (50 * 1000 * 1000UL)
#define RGMII_ID_MODE_10_LOW_SVS_CLK_FREQ	  (5 * 1000 * 1000UL)

static void
ethqos_update_rgmii_clk_and_bus_cfg(struct qcom_ethqos *ethqos,
				    unsigned int speed)
{
	switch (speed) {
	case SPEED_1000:
		ethqos->rgmii_clk_rate =  RGMII_1000_NOM_CLK_FREQ;
		break;

	case SPEED_100:
		ethqos->rgmii_clk_rate =  RGMII_ID_MODE_100_LOW_SVS_CLK_FREQ;
		break;

	case SPEED_10:
		ethqos->rgmii_clk_rate =  RGMII_ID_MODE_10_LOW_SVS_CLK_FREQ;
		break;
	}

	switch (speed) {
	case SPEED_1000:
		ethqos->vote_idx = VOTE_IDX_1000MBPS;
		break;
	case SPEED_100:
		ethqos->vote_idx = VOTE_IDX_100MBPS;
		break;
	case SPEED_10:
		ethqos->vote_idx = VOTE_IDX_10MBPS;
		break;
	case 0:
		ethqos->vote_idx = VOTE_IDX_0MBPS;
		ethqos->rgmii_clk_rate = 0;
		break;
	}
	clk_set_rate(ethqos->rgmii_clk, ethqos->rgmii_clk_rate);
}

static void ethqos_set_func_clk_en(struct qcom_ethqos *ethqos)
{
	rgmii_updatel(ethqos, RGMII_CONFIG_FUNC_CLK_EN,
		      RGMII_CONFIG_FUNC_CLK_EN, RGMII_IO_MACRO_CONFIG);
}

static int ethqos_dll_configure(struct qcom_ethqos *ethqos)
{
	/* Set CDR_EN */
	if (!ethqos->io_macro.config_cdr_en)
		rgmii_updatel(ethqos, SDCC_DLL_CONFIG_CDR_EN,
			      0, SDCC_HC_REG_DLL_CONFIG);
	else
		rgmii_updatel(ethqos, SDCC_DLL_CONFIG_CDR_EN,
			      SDCC_DLL_CONFIG_CDR_EN, SDCC_HC_REG_DLL_CONFIG);

	/* Set CDR_EXT_EN */
	rgmii_updatel(ethqos, SDCC_DLL_CONFIG_CDR_EXT_EN,
		      SDCC_DLL_CONFIG_CDR_EXT_EN, SDCC_HC_REG_DLL_CONFIG);

	if (ethqos->io_macro.mclk_gating_en)
		rgmii_updatel(ethqos, SDCC_DLL_MCLK_GATING_EN,
			      SDCC_DLL_MCLK_GATING_EN, SDCC_HC_REG_DLL_CONFIG);
	else
		rgmii_updatel(ethqos, SDCC_DLL_MCLK_GATING_EN,
			      0, SDCC_HC_REG_DLL_CONFIG);

	if (ethqos->io_macro.cdr_fine_phase)
		rgmii_updatel(ethqos, SDCC_DLL_CDR_FINE_PHASE,
			      SDCC_DLL_CDR_FINE_PHASE, SDCC_HC_REG_DLL_CONFIG);
	else
		rgmii_updatel(ethqos, SDCC_DLL_CDR_FINE_PHASE,
			      0, SDCC_HC_REG_DLL_CONFIG);

	if (ethqos->io_macro.ddr_cal_en)
		rgmii_updatel(ethqos, SDCC_DLL_CONFIG2_DDR_CAL_EN,
			      SDCC_DLL_CONFIG2_DDR_CAL_EN,
			      SDCC_HC_REG_DLL_CONFIG2);
	else
		rgmii_updatel(ethqos, SDCC_DLL_CONFIG2_DDR_CAL_EN,
			      0, SDCC_HC_REG_DLL_CONFIG2);

	if (ethqos->io_macro.dll_clock_dis)
		rgmii_updatel(ethqos, SDCC_DLL_CONFIG2_DLL_CLOCK_DIS,
			      SDCC_DLL_CONFIG2_DLL_CLOCK_DIS,
			      SDCC_HC_REG_DLL_CONFIG2);
	else
		rgmii_updatel(ethqos, SDCC_DLL_CONFIG2_DLL_CLOCK_DIS,
			      0, SDCC_HC_REG_DLL_CONFIG2);

	if (ethqos->io_macro.mclk_freq_calc)
		rgmii_updatel(ethqos, SDCC_DLL_CONFIG2_MCLK_FREQ_CALC,
			      ethqos->io_macro.mclk_freq_calc << 10,
			      SDCC_HC_REG_DLL_CONFIG2);

	if (ethqos->io_macro.ddr_traffic_init_sel)
		rgmii_updatel(ethqos, SDCC_DLL_CONFIG2_DDR_TRAFFIC_INIT_SEL,
			      SDCC_DLL_CONFIG2_DDR_TRAFFIC_INIT_SEL,
			      SDCC_HC_REG_DLL_CONFIG2);
	else
		rgmii_updatel(ethqos, SDCC_DLL_CONFIG2_DDR_TRAFFIC_INIT_SEL,
			      0, SDCC_HC_REG_DLL_CONFIG2);

	if (ethqos->io_macro.ddr_traffic_init_sw)
		rgmii_updatel(ethqos, SDCC_DLL_CONFIG2_DDR_TRAFFIC_INIT_SW,
			      SDCC_DLL_CONFIG2_DDR_TRAFFIC_INIT_SW,
			      SDCC_HC_REG_DLL_CONFIG2);
	else
		rgmii_updatel(ethqos, SDCC_DLL_CONFIG2_DDR_TRAFFIC_INIT_SW,
			      0, SDCC_HC_REG_DLL_CONFIG2);

	return 0;
}

void emac_rgmii_io_macro_config_1G(struct qcom_ethqos *ethqos)
{
	rgmii_updatel(ethqos, RGMII_CONFIG_DDR_MODE,
		      RGMII_CONFIG_DDR_MODE, RGMII_IO_MACRO_CONFIG);
	rgmii_updatel(ethqos, RGMII_CONFIG_BYPASS_TX_ID_EN,
		      0, RGMII_IO_MACRO_CONFIG);
	rgmii_updatel(ethqos, RGMII_CONFIG_POS_NEG_DATA_SEL,
		      RGMII_CONFIG_POS_NEG_DATA_SEL,
		      RGMII_IO_MACRO_CONFIG);
	rgmii_updatel(ethqos, RGMII_CONFIG_PROG_SWAP,
		      RGMII_CONFIG_PROG_SWAP, RGMII_IO_MACRO_CONFIG);
	rgmii_updatel(ethqos, RGMII_CONFIG2_DATA_DIVIDE_CLK_SEL,
		      0, RGMII_IO_MACRO_CONFIG2);
	rgmii_updatel(ethqos, RGMII_CONFIG2_TX_CLK_PHASE_SHIFT_EN,
		      RGMII_CONFIG2_TX_CLK_PHASE_SHIFT_EN,
		      RGMII_IO_MACRO_CONFIG2);
	rgmii_updatel(ethqos, RGMII_CONFIG2_RSVD_CONFIG15,
		      0, RGMII_IO_MACRO_CONFIG2);
	rgmii_updatel(ethqos, RGMII_CONFIG2_RX_PROG_SWAP,
		      RGMII_CONFIG2_RX_PROG_SWAP,
		      RGMII_IO_MACRO_CONFIG2);

	/* Set PRG_RCLK_DLY to 115 */
	rgmii_updatel(ethqos, SDCC_DDR_CONFIG_PRG_RCLK_DLY,
		      115, SDCC_HC_REG_DDR_CONFIG);
	rgmii_updatel(ethqos, SDCC_DDR_CONFIG_PRG_DLY_EN,
		      SDCC_DDR_CONFIG_PRG_DLY_EN,
		      SDCC_HC_REG_DDR_CONFIG);
	rgmii_updatel(ethqos, RGMII_CONFIG_LOOPBACK_EN,
		      0, RGMII_IO_MACRO_CONFIG);
}

void emac_rgmii_io_macro_config_100M(struct qcom_ethqos *ethqos)
{
	rgmii_updatel(ethqos, RGMII_CONFIG_DDR_MODE,
		      RGMII_CONFIG_DDR_MODE, RGMII_IO_MACRO_CONFIG);
	rgmii_updatel(ethqos, RGMII_CONFIG_BYPASS_TX_ID_EN,
		      RGMII_CONFIG_BYPASS_TX_ID_EN,
		      RGMII_IO_MACRO_CONFIG);
	rgmii_updatel(ethqos, RGMII_CONFIG_POS_NEG_DATA_SEL,
		      0, RGMII_IO_MACRO_CONFIG);
	rgmii_updatel(ethqos, RGMII_CONFIG_PROG_SWAP,
		      0, RGMII_IO_MACRO_CONFIG);
	rgmii_updatel(ethqos, RGMII_CONFIG2_DATA_DIVIDE_CLK_SEL,
		      0, RGMII_IO_MACRO_CONFIG2);
	rgmii_updatel(ethqos, RGMII_CONFIG2_TX_CLK_PHASE_SHIFT_EN,
		      RGMII_CONFIG2_TX_CLK_PHASE_SHIFT_EN,
		      RGMII_IO_MACRO_CONFIG2);
	rgmii_updatel(ethqos, RGMII_CONFIG_MAX_SPD_PRG_2,
		      BIT(6), RGMII_IO_MACRO_CONFIG);
	rgmii_updatel(ethqos, RGMII_CONFIG2_RSVD_CONFIG15,
		      0, RGMII_IO_MACRO_CONFIG2);
	rgmii_updatel(ethqos, RGMII_CONFIG2_RX_PROG_SWAP,
		      RGMII_CONFIG2_RX_PROG_SWAP,
		      RGMII_IO_MACRO_CONFIG2);

	/* Write 0x5 to PRG_RCLK_DLY_CODE */
	rgmii_updatel(ethqos, SDCC_DDR_CONFIG_EXT_PRG_RCLK_DLY_CODE,
		      (BIT(29) | BIT(27)), SDCC_HC_REG_DDR_CONFIG);
	rgmii_updatel(ethqos, SDCC_DDR_CONFIG_EXT_PRG_RCLK_DLY,
		      SDCC_DDR_CONFIG_EXT_PRG_RCLK_DLY,
		      SDCC_HC_REG_DDR_CONFIG);
	rgmii_updatel(ethqos, SDCC_DDR_CONFIG_EXT_PRG_RCLK_DLY_EN,
		      SDCC_DDR_CONFIG_EXT_PRG_RCLK_DLY_EN,
		      SDCC_HC_REG_DDR_CONFIG);
	rgmii_updatel(ethqos, RGMII_CONFIG_LOOPBACK_EN,
		      0, RGMII_IO_MACRO_CONFIG);
}

void emac_rgmii_io_macro_config_10M(struct qcom_ethqos *ethqos)
{
	rgmii_updatel(ethqos, RGMII_CONFIG_DDR_MODE,
		      RGMII_CONFIG_DDR_MODE, RGMII_IO_MACRO_CONFIG);
	rgmii_updatel(ethqos, RGMII_CONFIG_BYPASS_TX_ID_EN,
		      RGMII_CONFIG_BYPASS_TX_ID_EN,
		      RGMII_IO_MACRO_CONFIG);
	rgmii_updatel(ethqos, RGMII_CONFIG_POS_NEG_DATA_SEL,
		      0, RGMII_IO_MACRO_CONFIG);
	rgmii_updatel(ethqos, RGMII_CONFIG_PROG_SWAP,
		      0, RGMII_IO_MACRO_CONFIG);
	rgmii_updatel(ethqos, RGMII_CONFIG2_DATA_DIVIDE_CLK_SEL,
		      0, RGMII_IO_MACRO_CONFIG2);
	rgmii_updatel(ethqos,
		      RGMII_CONFIG2_TX_CLK_PHASE_SHIFT_EN,
		      RGMII_CONFIG2_TX_CLK_PHASE_SHIFT_EN,
		      RGMII_IO_MACRO_CONFIG2);
	rgmii_updatel(ethqos, RGMII_CONFIG_MAX_SPD_PRG_9,
		      BIT(12) | GENMASK(9, 8),
		      RGMII_IO_MACRO_CONFIG);
	rgmii_updatel(ethqos, RGMII_CONFIG2_RSVD_CONFIG15,
		      0, RGMII_IO_MACRO_CONFIG2);
	rgmii_updatel(ethqos, RGMII_CONFIG2_RX_PROG_SWAP,
		      RGMII_CONFIG2_RX_PROG_SWAP,
		      RGMII_IO_MACRO_CONFIG2);

	/* Write 0x5 to PRG_RCLK_DLY_CODE */
	rgmii_updatel(ethqos, SDCC_DDR_CONFIG_EXT_PRG_RCLK_DLY_CODE,
		      (BIT(29) | BIT(27)), SDCC_HC_REG_DDR_CONFIG);
}

static int ethqos_rgmii_macro_init(struct qcom_ethqos *ethqos)
{
	/* Disable loopback mode */
	rgmii_updatel(ethqos, RGMII_CONFIG2_TX_TO_RX_LOOPBACK_EN,
		      0, RGMII_IO_MACRO_CONFIG2);

	/* Select RGMII, write 0 to interface select */
	rgmii_updatel(ethqos, RGMII_CONFIG_INTF_SEL,
		      0, RGMII_IO_MACRO_CONFIG);

	switch (ethqos->speed) {
	case SPEED_1000:
		rgmii_updatel(ethqos, RGMII_CONFIG_DDR_MODE,
			      RGMII_CONFIG_DDR_MODE, RGMII_IO_MACRO_CONFIG);
		rgmii_updatel(ethqos, RGMII_CONFIG_BYPASS_TX_ID_EN,
			      0, RGMII_IO_MACRO_CONFIG);
		rgmii_updatel(ethqos, RGMII_CONFIG_POS_NEG_DATA_SEL,
			      RGMII_CONFIG_POS_NEG_DATA_SEL,
			      RGMII_IO_MACRO_CONFIG);
		rgmii_updatel(ethqos, RGMII_CONFIG_PROG_SWAP,
			      RGMII_CONFIG_PROG_SWAP, RGMII_IO_MACRO_CONFIG);
		if (ethqos->io_macro.data_divide_clk_sel)
			rgmii_updatel(ethqos, RGMII_CONFIG2_DATA_DIVIDE_CLK_SEL,
				      RGMII_CONFIG2_DATA_DIVIDE_CLK_SEL, RGMII_IO_MACRO_CONFIG2);
		else
			rgmii_updatel(ethqos, RGMII_CONFIG2_DATA_DIVIDE_CLK_SEL,
				      0, RGMII_IO_MACRO_CONFIG2);
		rgmii_updatel(ethqos, RGMII_CONFIG2_TX_CLK_PHASE_SHIFT_EN,
			      RGMII_CONFIG2_TX_CLK_PHASE_SHIFT_EN,
			      RGMII_IO_MACRO_CONFIG2);
		rgmii_updatel(ethqos, RGMII_CONFIG2_RSVD_CONFIG15,
			      0, RGMII_IO_MACRO_CONFIG2);
		rgmii_updatel(ethqos, RGMII_CONFIG2_RX_PROG_SWAP,
			      RGMII_CONFIG2_RX_PROG_SWAP,
			      RGMII_IO_MACRO_CONFIG2);

		if (ethqos->io_macro.tcx0_cycles_dly_line)
			rgmii_updatel(ethqos, SDCC_DDR_CONFIG_TCXO_CYCLES_DLY_LINE,
				      ethqos->io_macro.tcx0_cycles_dly_line << 12,
				      SDCC_HC_REG_DDR_CONFIG);

		if (ethqos->io_macro.tcx0_cycles_cnt)
			rgmii_updatel(ethqos, SDCC_DDR_CONFIG_TCXO_CYCLES_CNT,
				      ethqos->io_macro.tcx0_cycles_cnt << 9,
				      SDCC_HC_REG_DDR_CONFIG);

		if (ethqos->io_macro.prg_rclk_dly)
			rgmii_updatel(ethqos, SDCC_DDR_CONFIG_PRG_RCLK_DLY,
				      ethqos->io_macro.prg_rclk_dly,
				      SDCC_HC_REG_DDR_CONFIG);

		rgmii_updatel(ethqos, SDCC_DDR_CONFIG_PRG_DLY_EN,
			      SDCC_DDR_CONFIG_PRG_DLY_EN,
			      SDCC_HC_REG_DDR_CONFIG);

		if (!ethqos->io_macro.loopback_en)
			rgmii_updatel(ethqos, RGMII_CONFIG_LOOPBACK_EN,
				      0, RGMII_IO_MACRO_CONFIG);
		else
			rgmii_updatel(ethqos, RGMII_CONFIG_LOOPBACK_EN,
				      RGMII_CONFIG_LOOPBACK_EN,
				      RGMII_IO_MACRO_CONFIG);
		break;

	case SPEED_100:
		rgmii_updatel(ethqos, RGMII_CONFIG_DDR_MODE,
			      RGMII_CONFIG_DDR_MODE, RGMII_IO_MACRO_CONFIG);
		rgmii_updatel(ethqos, RGMII_CONFIG_BYPASS_TX_ID_EN,
			      RGMII_CONFIG_BYPASS_TX_ID_EN,
			      RGMII_IO_MACRO_CONFIG);
		rgmii_updatel(ethqos, RGMII_CONFIG_POS_NEG_DATA_SEL,
			      0, RGMII_IO_MACRO_CONFIG);
		rgmii_updatel(ethqos, RGMII_CONFIG_PROG_SWAP,
			      0, RGMII_IO_MACRO_CONFIG);
		if (!ethqos->io_macro.data_divide_clk_sel)
			rgmii_updatel(ethqos, RGMII_CONFIG2_DATA_DIVIDE_CLK_SEL,
				      0, RGMII_IO_MACRO_CONFIG2);
		rgmii_updatel(ethqos, RGMII_CONFIG2_TX_CLK_PHASE_SHIFT_EN,
			      RGMII_CONFIG2_TX_CLK_PHASE_SHIFT_EN,
			      RGMII_IO_MACRO_CONFIG2);
		rgmii_updatel(ethqos, RGMII_CONFIG_MAX_SPD_PRG_2,
			      BIT(6), RGMII_IO_MACRO_CONFIG);
		rgmii_updatel(ethqos, RGMII_CONFIG2_RSVD_CONFIG15,
			      0, RGMII_IO_MACRO_CONFIG2);

		if (ethqos->io_macro.rx_prog_swap)
			rgmii_updatel(ethqos, RGMII_CONFIG2_RX_PROG_SWAP,
				      RGMII_CONFIG2_RX_PROG_SWAP,
				      RGMII_IO_MACRO_CONFIG2);
		else
			rgmii_updatel(ethqos, RGMII_CONFIG2_RX_PROG_SWAP,
				      0, RGMII_IO_MACRO_CONFIG2);
		/* Write 0x5 to PRG_RCLK_DLY_CODE */
		rgmii_updatel(ethqos, SDCC_DDR_CONFIG_EXT_PRG_RCLK_DLY_CODE,
			      (BIT(29) | BIT(27)), SDCC_HC_REG_DDR_CONFIG);
		rgmii_updatel(ethqos, SDCC_DDR_CONFIG_EXT_PRG_RCLK_DLY,
			      SDCC_DDR_CONFIG_EXT_PRG_RCLK_DLY,
			      SDCC_HC_REG_DDR_CONFIG);
		rgmii_updatel(ethqos, SDCC_DDR_CONFIG_EXT_PRG_RCLK_DLY_EN,
			      SDCC_DDR_CONFIG_EXT_PRG_RCLK_DLY_EN,
			      SDCC_HC_REG_DDR_CONFIG);

		if (!ethqos->io_macro.loopback_en)
			rgmii_updatel(ethqos, RGMII_CONFIG_LOOPBACK_EN,
				      0, RGMII_IO_MACRO_CONFIG);
		else
			rgmii_updatel(ethqos, RGMII_CONFIG_LOOPBACK_EN,
				      RGMII_CONFIG_LOOPBACK_EN,
				      RGMII_IO_MACRO_CONFIG);
		break;

	case SPEED_10:
		rgmii_updatel(ethqos, RGMII_CONFIG_DDR_MODE,
			      RGMII_CONFIG_DDR_MODE, RGMII_IO_MACRO_CONFIG);
		rgmii_updatel(ethqos, RGMII_CONFIG_BYPASS_TX_ID_EN,
			      RGMII_CONFIG_BYPASS_TX_ID_EN,
			      RGMII_IO_MACRO_CONFIG);
		rgmii_updatel(ethqos, RGMII_CONFIG_POS_NEG_DATA_SEL,
			      0, RGMII_IO_MACRO_CONFIG);
		rgmii_updatel(ethqos, RGMII_CONFIG_PROG_SWAP,
			      0, RGMII_IO_MACRO_CONFIG);
		if (!ethqos->io_macro.data_divide_clk_sel)
			rgmii_updatel(ethqos, RGMII_CONFIG2_DATA_DIVIDE_CLK_SEL,
				      0, RGMII_IO_MACRO_CONFIG2);

		if (ethqos->io_macro.tx_clk_phase_shift_en)
			rgmii_updatel(ethqos,
				      RGMII_CONFIG2_TX_CLK_PHASE_SHIFT_EN,
				      RGMII_CONFIG2_TX_CLK_PHASE_SHIFT_EN,
				      RGMII_IO_MACRO_CONFIG2);
		else
			rgmii_updatel(ethqos,
				      RGMII_CONFIG2_TX_CLK_PHASE_SHIFT_EN,
				      0, RGMII_IO_MACRO_CONFIG2);
		rgmii_updatel(ethqos, RGMII_CONFIG_MAX_SPD_PRG_9,
			      BIT(12) | GENMASK(9, 8),
			      RGMII_IO_MACRO_CONFIG);
		rgmii_updatel(ethqos, RGMII_CONFIG2_RSVD_CONFIG15,
			      0, RGMII_IO_MACRO_CONFIG2);

		if (ethqos->io_macro.rx_prog_swap)
			rgmii_updatel(ethqos, RGMII_CONFIG2_RX_PROG_SWAP,
				      RGMII_CONFIG2_RX_PROG_SWAP,
				      RGMII_IO_MACRO_CONFIG2);
		else
			rgmii_updatel(ethqos, RGMII_CONFIG2_RX_PROG_SWAP,
				      0, RGMII_IO_MACRO_CONFIG2);
		/* Write 0x5 to PRG_RCLK_DLY_CODE */
		rgmii_updatel(ethqos, SDCC_DDR_CONFIG_EXT_PRG_RCLK_DLY_CODE,
			      (BIT(29) | BIT(27)), SDCC_HC_REG_DDR_CONFIG);
		rgmii_updatel(ethqos, SDCC_DDR_CONFIG_EXT_PRG_RCLK_DLY,
			      SDCC_DDR_CONFIG_EXT_PRG_RCLK_DLY,
			      SDCC_HC_REG_DDR_CONFIG);
		rgmii_updatel(ethqos, SDCC_DDR_CONFIG_EXT_PRG_RCLK_DLY_EN,
			      SDCC_DDR_CONFIG_EXT_PRG_RCLK_DLY_EN,
			      SDCC_HC_REG_DDR_CONFIG);

		if (!ethqos->io_macro.loopback_en)
			rgmii_updatel(ethqos, RGMII_CONFIG_LOOPBACK_EN,
				      0, RGMII_IO_MACRO_CONFIG);
		else
			rgmii_updatel(ethqos, RGMII_CONFIG_LOOPBACK_EN,
				      RGMII_CONFIG_LOOPBACK_EN,
				      RGMII_IO_MACRO_CONFIG);
		break;
	default:
		dev_err(&ethqos->pdev->dev,
			"Invalid speed %d\n", ethqos->speed);
		return -EINVAL;
	}

	return 0;
}

static int ethqos_rgmii_macro_init_v3(struct qcom_ethqos *ethqos)
{
	/* Disable loopback mode */
	rgmii_updatel(ethqos, RGMII_CONFIG2_TX_TO_RX_LOOPBACK_EN,
		      0, RGMII_IO_MACRO_CONFIG2);

	/* Select RGMII, write 0 to interface select */
	rgmii_updatel(ethqos, RGMII_CONFIG_INTF_SEL,
		      0, RGMII_IO_MACRO_CONFIG);

	switch (ethqos->speed) {
	case SPEED_1000:
		emac_rgmii_io_macro_config_1G(ethqos);
		break;

	case SPEED_100:
		emac_rgmii_io_macro_config_100M(ethqos);
		break;

	case SPEED_10:
		emac_rgmii_io_macro_config_10M(ethqos);
		break;
	default:
		dev_err(&ethqos->pdev->dev,
			"Invalid speed %d\n", ethqos->speed);
		return -EINVAL;
	}

	return 0;
}

static int ethqos_configure(struct qcom_ethqos *ethqos)
{
	volatile unsigned int dll_lock;
	unsigned int i, retry = 1000;
	unsigned int val;

	/* Reset to POR values and enable clk */
	for (i = 0; i < ethqos->num_por; i++)
		rgmii_writel(ethqos, ethqos->por[i].value,
			     ethqos->por[i].offset);
	ethqos_set_func_clk_en(ethqos);

	if (ethqos->speed != SPEED_100 && ethqos->speed != SPEED_10) {
		/* Disable CK_OUT_EN */
		rgmii_updatel(ethqos, SDCC_DLL_CONFIG_CK_OUT_EN,
			      0,
			      SDCC_HC_REG_DLL_CONFIG);

		/* Wait for CK_OUT_EN clear */
		do {
			val = rgmii_readl(ethqos, SDCC_HC_REG_DLL_CONFIG);
			val &= SDCC_DLL_CONFIG_CK_OUT_EN;
			if (!val)
				break;
			usleep_range(1000, 1500);
			retry--;
		} while (retry > 0);
		if (!retry)
			dev_err(&ethqos->pdev->dev, "Clear CK_OUT_EN timedout\n");

			/* Set DLL_EN */
			rgmii_updatel(ethqos, SDCC_DLL_CONFIG_DLL_EN,
				      SDCC_DLL_CONFIG_DLL_EN, SDCC_HC_REG_DLL_CONFIG);
	}

	if (ethqos->speed == SPEED_1000) {
		ethqos_dll_configure(ethqos);

		if (ethqos->io_macro.test_ctl)
			rgmii_writel(ethqos, ethqos->io_macro.test_ctl,
				     SDCC_TEST_CTL_RGOFFADDR_OFFSET);

		if (ethqos->io_macro.usr_ctl)
			rgmii_writel(ethqos, ethqos->io_macro.usr_ctl,
				     SDCC_USR_CTL_RGOFFADDR_OFFSET);
		else
			rgmii_updatel(ethqos, GENMASK(26, 24), BIT(26),
				      SDCC_USR_CTL);
	}

	ethqos_rgmii_macro_init(ethqos);

	/* Set DLL_RST */
	rgmii_updatel(ethqos, SDCC_DLL_CONFIG_DLL_RST,
		      SDCC_DLL_CONFIG_DLL_RST, SDCC_HC_REG_DLL_CONFIG);

	/* Set PDN */
	rgmii_updatel(ethqos, SDCC_DLL_CONFIG_PDN,
		      SDCC_DLL_CONFIG_PDN, SDCC_HC_REG_DLL_CONFIG);

	if (ethqos->speed != SPEED_100 && ethqos->speed != SPEED_10)
		usleep_range(1000, 1500);

	/* Clear DLL_RST */
	rgmii_updatel(ethqos, SDCC_DLL_CONFIG_DLL_RST, 0,
		      SDCC_HC_REG_DLL_CONFIG);

	/* Clear PDN */
	rgmii_updatel(ethqos, SDCC_DLL_CONFIG_PDN, 0,
		      SDCC_HC_REG_DLL_CONFIG);

	if (ethqos->speed != SPEED_100 && ethqos->speed != SPEED_10)
		usleep_range(1000, 1500);

	if (ethqos->speed != SPEED_100 && ethqos->speed != SPEED_10) {
		/* Set CK_OUT_EN */
		rgmii_updatel(ethqos, SDCC_DLL_CONFIG_CK_OUT_EN,
			      SDCC_DLL_CONFIG_CK_OUT_EN,
		      SDCC_HC_REG_DLL_CONFIG);

		/* Wait for CK_OUT_EN set */
		retry = 1000;
		do {
			val = rgmii_readl(ethqos, SDCC_HC_REG_DLL_CONFIG);
			val &= SDCC_DLL_CONFIG_CK_OUT_EN;
			if (val)
				break;
			usleep_range(1000, 1500);
			retry--;
		} while (retry > 0);
		if (!retry)
			dev_err(&ethqos->pdev->dev, "Set CK_OUT_EN timedout\n");

		/* wait for DLL LOCK */
		retry = 1000;
		do {
			usleep_range(1000, 1500);
			dll_lock = rgmii_readl(ethqos, SDC4_STATUS);
			if (dll_lock & SDC4_STATUS_DLL_LOCK)
				break;
			retry--;
		} while (retry > 0);
		if (!retry)
			dev_err(&ethqos->pdev->dev,
				"Timeout while waiting for DLL lock\n");
	}
	return 0;
}

/* for EMAC_HW_VER >= 3 */
static int ethqos_configure_mac_v3(struct qcom_ethqos *ethqos)
{
	unsigned int dll_lock;
	unsigned int i, retry = 1000;
	int ret = 0;
	/* Reset to POR values and enable clk */
	for (i = 0; i < ethqos->num_por; i++)
		rgmii_writel(ethqos, ethqos->por[i].value,
			     ethqos->por[i].offset);
	ethqos_set_func_clk_en(ethqos);

	/* Put DLL into Reset and Powerdown */
	rgmii_updatel(ethqos, SDCC_DLL_CONFIG_DLL_RST,
		      SDCC_DLL_CONFIG_DLL_RST, SDCC_HC_REG_DLL_CONFIG);
	rgmii_updatel(ethqos, SDCC_DLL_CONFIG_PDN,
		      SDCC_DLL_CONFIG_PDN, SDCC_HC_REG_DLL_CONFIG)
		;
	/*Power on and set DLL, Set->RST & PDN to '0' */
	rgmii_updatel(ethqos, SDCC_DLL_CONFIG_DLL_RST,
		      0, SDCC_HC_REG_DLL_CONFIG);
	rgmii_updatel(ethqos, SDCC_DLL_CONFIG_PDN,
		      0, SDCC_HC_REG_DLL_CONFIG);

	/* for 10 or 100Mbps further configuration not required */
	if (ethqos->speed == SPEED_1000) {
		/* Disable DLL output clock */
		rgmii_updatel(ethqos, SDCC_DLL_CONFIG_CK_OUT_EN,
			      0, SDCC_HC_REG_DLL_CONFIG);

		/* Configure SDCC_DLL_TEST_CTRL */
		rgmii_writel(ethqos, HSR_SDCC_DLL_TEST_CTRL, SDCC_TEST_CTL);

		/* Configure SDCC_USR_CTRL */
		rgmii_writel(ethqos, HSR_SDCC_USR_CTRL, SDCC_USR_CTL);

		/* Configure DDR_CONFIG */
		rgmii_writel(ethqos, HSR_DDR_CONFIG, SDCC_HC_REG_DDR_CONFIG);

		/* Configure PRG_RCLK_DLY */
		rgmii_updatel(ethqos, SDCC_DDR_CONFIG_PRG_RCLK_DLY,
			      DDR_CONFIG_PRG_RCLK_DLY, SDCC_HC_REG_DDR_CONFIG);
		/*Enable PRG_RCLK_CLY */
		rgmii_updatel(ethqos, SDCC_DDR_CONFIG_PRG_DLY_EN,
			      SDCC_DDR_CONFIG_PRG_DLY_EN, SDCC_HC_REG_DDR_CONFIG);

		/* Configure DLL_CONFIG */
		rgmii_writel(ethqos, HSR_DLL_CONFIG, SDCC_HC_REG_DLL_CONFIG);

		/*Set -> DLL_CONFIG_2 MCLK_FREQ_CALC*/
		rgmii_writel(ethqos, HSR_DLL_CONFIG_2, SDCC_HC_REG_DLL_CONFIG2);

		/*Power Down and Reset DLL*/
		rgmii_updatel(ethqos, SDCC_DLL_CONFIG_DLL_RST,
			      SDCC_DLL_CONFIG_DLL_RST, SDCC_HC_REG_DLL_CONFIG);
		rgmii_updatel(ethqos, SDCC_DLL_CONFIG_PDN,
			      SDCC_DLL_CONFIG_PDN, SDCC_HC_REG_DLL_CONFIG);

		/*wait for 52us*/
		usleep_range(52, 55);

		/*Power on and set DLL, Set->RST & PDN to '0' */
		rgmii_updatel(ethqos, SDCC_DLL_CONFIG_DLL_RST,
			      0, SDCC_HC_REG_DLL_CONFIG);
		rgmii_updatel(ethqos, SDCC_DLL_CONFIG_PDN,
			      0, SDCC_HC_REG_DLL_CONFIG);

		/*Wait for 8000 input clock cycles, 8000 cycles of 100 MHz = 80us*/
		usleep_range(80, 85);

		/* Enable DLL output clock */
		rgmii_updatel(ethqos, SDCC_DLL_CONFIG_CK_OUT_EN,
			      SDCC_DLL_CONFIG_CK_OUT_EN, SDCC_HC_REG_DLL_CONFIG);

		/* Check for DLL lock */
		do {
			udelay(1);
			dll_lock = rgmii_readl(ethqos, SDC4_STATUS);
			if (dll_lock & SDC4_STATUS_DLL_LOCK)
				break;
			retry--;
		} while (retry > 0);
		if (!retry)
			dev_err(&ethqos->pdev->dev,
				"Timeout while waiting for DLL lock\n");
	}

	/* DLL bypass mode for 10Mbps and 100Mbps
	 * 1.   Write 1 to PDN bit of SDCC_HC_REG_DLL_CONFIG register.
	 * 2.   Write 1 to bypass bit of SDCC_USR_CTL register
	 * 3.   Default value of this register is 0x00010800
	 */
	if (ethqos->speed == SPEED_10 || ethqos->speed == SPEED_100) {
		rgmii_updatel(ethqos, SDCC_DLL_CONFIG_PDN,
			      SDCC_DLL_CONFIG_PDN, SDCC_HC_REG_DLL_CONFIG);
		rgmii_updatel(ethqos, DLL_BYPASS,
			      DLL_BYPASS, SDCC_USR_CTL);
	}

	ret = ethqos_rgmii_macro_init_v3(ethqos);

	return ret;
}

static void ethqos_fix_mac_speed(void *priv, unsigned int speed)
{
	struct qcom_ethqos *ethqos = priv;
	int ret = 0;

	ethqos->speed = speed;
	ethqos_update_rgmii_clk_and_bus_cfg(ethqos, speed);
	if (ethqos->emac_ver == EMAC_HW_v3_0_0_RG)
		ret = ethqos_configure_mac_v3(ethqos);
	else
		ethqos_configure(ethqos);
	if (ret != 0)
		ETHQOSERR("HSR configuration has failed\n");
}

static int ethqos_mdio_read(struct stmmac_priv  *priv, int phyaddr, int phyreg)
{
	unsigned int mii_address = priv->hw->mii.addr;
	unsigned int mii_data = priv->hw->mii.data;
	u32 v;
	int data;
	u32 value = MII_BUSY;

	value |= (phyaddr << priv->hw->mii.addr_shift)
		& priv->hw->mii.addr_mask;
	value |= (phyreg << priv->hw->mii.reg_shift) & priv->hw->mii.reg_mask;
	value |= (priv->clk_csr << priv->hw->mii.clk_csr_shift)
		& priv->hw->mii.clk_csr_mask;
	if (priv->plat->has_gmac4)
		value |= MII_GMAC4_READ;

	if (readl_poll_timeout(priv->ioaddr + mii_address, v, !(v & MII_BUSY),
			       100, 10000))
		return -EBUSY;

	writel_relaxed(value, priv->ioaddr + mii_address);

	if (readl_poll_timeout(priv->ioaddr + mii_address, v, !(v & MII_BUSY),
			       100, 10000))
		return -EBUSY;

	/* Read the data from the MII data register */
	data = (int)readl_relaxed(priv->ioaddr + mii_data);

	return data;
}

static int ethqos_phy_intr_config(struct qcom_ethqos *ethqos)
{
	int ret = 0;
	struct platform_device *pdev = ethqos->pdev;
	struct net_device *dev = platform_get_drvdata(pdev);
	struct stmmac_priv *priv = netdev_priv(dev);

	ethqos->phy_intr = platform_get_irq_byname(ethqos->pdev, "phy-intr");

	if (ethqos->phy_intr < 0) {
		if (ethqos->phy_intr != -EPROBE_DEFER) {
			dev_err(&ethqos->pdev->dev,
				"PHY IRQ configuration information not found\n");
		}
		ret = 1;
	} else {
		priv->phy_intr_wol_irq = ethqos->phy_intr;
	}

	return ret;
}

static void ethqos_handle_phy_interrupt(struct qcom_ethqos *ethqos)
{
	int phy_intr_status = 0;
	struct platform_device *pdev = ethqos->pdev;

	struct net_device *dev = platform_get_drvdata(pdev);
	struct stmmac_priv *priv = netdev_priv(dev);
	int micrel_intr_status = 0;

	if ((priv->phydev && (priv->phydev->phy_id &
	     priv->phydev->drv->phy_id_mask)
	     == MICREL_PHY_ID) ||
	    (priv->phydev && (priv->phydev->phy_id &
	     priv->phydev->drv->phy_id_mask)
	     == PHY_ID_KSZ9131)) {
		phy_intr_status = ethqos_mdio_read(priv,
						   priv->plat->phy_addr,
						   DWC_ETH_QOS_BASIC_STATUS);
		ETHQOSDBG("Basic Status Reg (%#x) = %#x\n",
			  DWC_ETH_QOS_BASIC_STATUS, phy_intr_status);
		micrel_intr_status = ethqos_mdio_read(priv,
						      priv->plat->phy_addr,
						      DWC_ETH_QOS_MICREL_PHY_INTCS);
		ETHQOSDBG("MICREL PHY Intr EN Reg (%#x) = %#x\n",
			  DWC_ETH_QOS_MICREL_PHY_INTCS, micrel_intr_status);

		/**
		 * Call ack interrupt to clear the WOL
		 * interrupt status fields
		 */
		if (priv->phydev->drv->ack_interrupt)
			priv->phydev->drv->ack_interrupt(priv->phydev);

		/* Interrupt received for link state change */
		if (phy_intr_status & LINK_STATE_MASK) {
			if (micrel_intr_status & MICREL_LINK_UP_INTR_STATUS)
				ETHQOSDBG("Intr for link UP state\n");
			phy_mac_interrupt(priv->phydev);
		} else if (!(phy_intr_status & LINK_STATE_MASK)) {
			ETHQOSDBG("Intr for link DOWN state\n");
			phy_mac_interrupt(priv->phydev);
		} else if (!(phy_intr_status & AUTONEG_STATE_MASK)) {
			ETHQOSDBG("Intr for link down with auto-neg err\n");
		}
	} else {
		phy_intr_status =
		 ethqos_mdio_read(priv, priv->plat->phy_addr,
				  DWC_ETH_QOS_PHY_INTR_STATUS);

		if (!priv->plat->mac2mac_en) {
			if (phy_intr_status & LINK_UP_STATE)
				phy_mac_interrupt(priv->phydev);
			else if (phy_intr_status & LINK_DOWN_STATE)
				phy_mac_interrupt(priv->phydev);
		}
	}
}

static void ethqos_defer_phy_isr_work(struct work_struct *work)
{
	struct qcom_ethqos *ethqos =
		container_of(work, struct qcom_ethqos, emac_phy_work);

	if (ethqos->clks_suspended)
		wait_for_completion(&ethqos->clk_enable_done);

	ethqos_handle_phy_interrupt(ethqos);
}

static irqreturn_t ETHQOS_PHY_ISR(int irq, void *dev_data)
{
	struct qcom_ethqos *ethqos = (struct qcom_ethqos *)dev_data;

	pm_wakeup_event(&ethqos->pdev->dev, 5000);

	queue_work(system_wq, &ethqos->emac_phy_work);
	return IRQ_HANDLED;
}

static void ethqos_phy_irq_enable(void *priv_n)
{
	struct stmmac_priv *priv = priv_n;
	struct qcom_ethqos *ethqos = priv->plat->bsp_priv;

	if (ethqos->phy_intr) {
		ETHQOSINFO("enabling irq = %d\n", priv->phy_irq_enabled);
		enable_irq(ethqos->phy_intr);
		priv->phy_irq_enabled = true;
	}
}

static void ethqos_phy_irq_disable(void *priv_n)
{
	struct stmmac_priv *priv = priv_n;
	struct qcom_ethqos *ethqos = priv->plat->bsp_priv;

	if (ethqos->phy_intr) {
		ETHQOSINFO("disabling irq = %d\n", priv->phy_irq_enabled);
		disable_irq(ethqos->phy_intr);
		priv->phy_irq_enabled = false;
	}
}

static int ethqos_phy_intr_enable(void *priv_n)
{
	int ret = 0;
	struct stmmac_priv *priv = priv_n;
	struct qcom_ethqos *ethqos = priv->plat->bsp_priv;

	if (ethqos_phy_intr_config(ethqos)) {
		ret = 1;
		return ret;
	}

	INIT_WORK(&ethqos->emac_phy_work, ethqos_defer_phy_isr_work);
	init_completion(&ethqos->clk_enable_done);

	ret = request_irq(ethqos->phy_intr, ETHQOS_PHY_ISR,
			  IRQF_SHARED, "stmmac", ethqos);
	if (ret) {
		ETHQOSERR("Unable to register PHY IRQ %d\n",
			  ethqos->phy_intr);
		return ret;
	}
	priv->plat->phy_intr_en_extn_stm = true;
	priv->phy_irq_enabled = true;
	return ret;
}

static const struct of_device_id qcom_ethqos_match[] = {
	{ .compatible = "qcom,qcs404-ethqos",},
	{ .compatible = "qcom,sdxprairie-ethqos",},
	{ .compatible = "qcom,stmmac-ethqos", },
	{ .compatible = "qcom,emac-smmu-embedded", },
	{ }
};

static void emac_emb_smmu_exit(void)
{
	emac_emb_smmu_ctx.valid = false;
	emac_emb_smmu_ctx.pdev_master = NULL;
	emac_emb_smmu_ctx.smmu_pdev = NULL;
	emac_emb_smmu_ctx.iommu_domain = NULL;
}

static int emac_emb_smmu_cb_probe(struct platform_device *pdev,
				  struct plat_stmmacenet_data *plat_dat)
{
	int result = 0;
	u32 iova_ap_mapping[2];
	struct device *dev = &pdev->dev;
	const char *str = NULL;

	ETHQOSDBG("EMAC EMB SMMU CB probe: smmu pdev=%p\n", pdev);
	result = of_property_read_string(dev->of_node, "qcom,iommu-dma", &str);

	if (result == 0 && !strcmp(str, "bypass")) {
		ETHQOSINFO("iommu-dma-addr-pool not required in bypass mode\n");
	} else {
		result = of_property_read_u32_array(dev->of_node,
						    "qcom,iommu-dma-addr-pool",
						    iova_ap_mapping,
						    ARRAY_SIZE(iova_ap_mapping));
		if (result) {
			ETHQOSERR("Failed to read EMB start/size iova addresses\n");
			return result;
		}
	}

	emac_emb_smmu_ctx.smmu_pdev = pdev;

	if (dma_set_mask(dev, DMA_BIT_MASK(32)) ||
	    dma_set_coherent_mask(dev, DMA_BIT_MASK(32))) {
		ETHQOSERR("DMA set 32bit mask failed\n");
		return -EOPNOTSUPP;
	}

	emac_emb_smmu_ctx.valid = true;

	emac_emb_smmu_ctx.iommu_domain =
		iommu_get_domain_for_dev(&emac_emb_smmu_ctx.smmu_pdev->dev);

	ETHQOSINFO("Successfully attached to IOMMU\n");
	plat_dat->stmmac_emb_smmu_ctx = emac_emb_smmu_ctx;
	if (emac_emb_smmu_ctx.pdev_master)
		goto smmu_probe_done;

smmu_probe_done:
	emac_emb_smmu_ctx.ret = result;
	return result;
}

static void ethqos_pps_irq_config(struct qcom_ethqos *ethqos)
{
	ethqos->pps_class_a_irq =
	platform_get_irq_byname(ethqos->pdev, "ptp_pps_irq_0");
	if (ethqos->pps_class_a_irq < 0) {
		if (ethqos->pps_class_a_irq != -EPROBE_DEFER)
			ETHQOSERR("class_a_irq config info not found\n");
	}
	ethqos->pps_class_b_irq =
	platform_get_irq_byname(ethqos->pdev, "ptp_pps_irq_1");
	if (ethqos->pps_class_b_irq < 0) {
		if (ethqos->pps_class_b_irq != -EPROBE_DEFER)
			ETHQOSERR("class_b_irq config info not found\n");
	}
}

static void qcom_ethqos_phy_suspend_clks(struct qcom_ethqos *ethqos)
{
	struct stmmac_priv *priv = qcom_ethqos_get_priv(ethqos);

	ETHQOSINFO("Enter\n");

	if (priv->plat->phy_intr_en_extn_stm)
		reinit_completion(&ethqos->clk_enable_done);

	ethqos->clks_suspended = 1;

	ethqos_update_rgmii_clk_and_bus_cfg(ethqos, 0);

	if (ethqos->phy_wol_supported) {
		if (priv->plat->stmmac_clk)
			clk_disable_unprepare(priv->plat->stmmac_clk);

		if (priv->plat->pclk)
			clk_disable_unprepare(priv->plat->pclk);

		if (priv->plat->clk_ptp_ref)
			clk_disable_unprepare(priv->plat->clk_ptp_ref);
	}
	if (ethqos->rgmii_clk)
		clk_disable_unprepare(ethqos->rgmii_clk);

	ETHQOSINFO("Exit\n");
}

inline void *qcom_ethqos_get_priv(struct qcom_ethqos *ethqos)
{
	struct platform_device *pdev = ethqos->pdev;
	struct net_device *dev = platform_get_drvdata(pdev);
	struct stmmac_priv *priv = netdev_priv(dev);

	return priv;
}

inline bool qcom_ethqos_is_phy_link_up(struct qcom_ethqos *ethqos)
{
	/* PHY driver initializes phydev->link=1.
	 * So, phydev->link is 1 even on bootup with no PHY connected.
	 * phydev->link is valid only after adjust_link is called once.
	 */
	struct stmmac_priv *priv = qcom_ethqos_get_priv(ethqos);

	if (priv->plat->mac2mac_en) {
		return priv->plat->mac2mac_link;
	} else {
		return (priv->dev->phydev &&
			priv->dev->phydev->link);
	}
}

static void qcom_ethqos_phy_resume_clks(struct qcom_ethqos *ethqos)
{
	struct stmmac_priv *priv = qcom_ethqos_get_priv(ethqos);

	ETHQOSINFO("Enter\n");

	if (ethqos->phy_wol_supported) {
		if (priv->plat->stmmac_clk)
			clk_prepare_enable(priv->plat->stmmac_clk);

		if (priv->plat->pclk)
			clk_prepare_enable(priv->plat->pclk);

		if (priv->plat->clk_ptp_ref)
			clk_prepare_enable(priv->plat->clk_ptp_ref);
	}

	if (ethqos->rgmii_clk)
		clk_prepare_enable(ethqos->rgmii_clk);

	if (qcom_ethqos_is_phy_link_up(ethqos))
		ethqos_update_rgmii_clk_and_bus_cfg(ethqos, ethqos->speed);
	else
		ethqos_update_rgmii_clk_and_bus_cfg(ethqos, SPEED_10);

	ethqos->clks_suspended = 0;

	if (priv->plat->phy_intr_en_extn_stm)
		complete_all(&ethqos->clk_enable_done);

	ETHQOSINFO("Exit\n");
}

static void qcom_ethqos_request_phy_wol(void *plat_n)
{
	struct plat_stmmacenet_data *plat = plat_n;
	struct qcom_ethqos *ethqos;
	struct platform_device *pdev;
	struct net_device *ndev;

	if (!plat)
		return;

	ethqos = plat->bsp_priv;

	pdev = ethqos->pdev;
	ndev = platform_get_drvdata(pdev);

	ethqos->phy_wol_supported = 0;
	ethqos->phy_wol_wolopts = 0;
	/* Check if phydev is valid*/
	/* Check and enable Wake-on-LAN functionality in PHY*/

	if (ndev->phydev) {
		struct ethtool_wolinfo wol = {.cmd = ETHTOOL_GWOL};

		wol.supported = 0;
		wol.wolopts = 0;
		ETHQOSINFO("phydev addr: 0x%pK\n", ndev->phydev);
		phy_ethtool_get_wol(ndev->phydev, &wol);
		ethqos->phy_wol_supported = wol.supported;
		ETHQOSINFO("Get WoL[0x%x] in %s\n", wol.supported,
			   ndev->phydev->drv->name);

	/* Try to enable supported Wake-on-LAN features in PHY*/
		if (wol.supported) {
			device_set_wakeup_capable(&ethqos->pdev->dev, 1);

			wol.cmd = ETHTOOL_SWOL;
			wol.wolopts = wol.supported;

			if (!phy_ethtool_set_wol(ndev->phydev, &wol)) {
				ethqos->phy_wol_wolopts = wol.wolopts;

				enable_irq_wake(ethqos->phy_intr);
				device_set_wakeup_enable(&ethqos->pdev->dev, 1);

				ETHQOSINFO("Enabled WoL[0x%x] in %s\n",
					   wol.wolopts,
					   ndev->phydev->drv->name);
			} else {
				ETHQOSINFO("Disabled WoL[0x%x] in %s\n",
					   wol.wolopts,
					   ndev->phydev->drv->name);
			}
		} else {
			ETHQOSINFO("WoL Not Supported\n");
		}
	}
}

static void read_rgmii_io_macro_node_setting(struct device_node *np_hw, struct qcom_ethqos *ethqos)
{
	int ret = 0;
	unsigned int rclk_dly_read_ps;

	ret = of_property_read_u32(np_hw, "prg-rclk-dly",
				   &rclk_dly_read_ps);
	if (!ret && rclk_dly_read_ps) {
		ethqos->io_macro.prg_rclk_dly =
		(RGMII_PRG_RCLK_CONST * 1000) / rclk_dly_read_ps;
	} else {
		ethqos->io_macro.prg_rclk_dly = 57;
	}

	ret = of_property_read_u32(np_hw,
				   "config-cdr-en",
				   &ethqos->io_macro.config_cdr_en);
	if (ret) {
		ETHQOSDBG("default config_cdr_en\n");
		ethqos->io_macro.config_cdr_en = 1;
	}

	ret = of_property_read_u32(np_hw,
				   "mclk-gating-en",
				   &ethqos->io_macro.mclk_gating_en);
	if (ret) {
		ETHQOSDBG("default mclk_gating_en\n");
		ethqos->io_macro.mclk_gating_en = 0;
	}

	ret = of_property_read_u32(np_hw,
				   "cdr-fine-phase",
				   &ethqos->io_macro.cdr_fine_phase);
	if (ret) {
		ETHQOSDBG("default cdr_fine_phase\n");
		ethqos->io_macro.cdr_fine_phase = 0;
	}

	ret = of_property_read_u32(np_hw,
				   "skip-calc-traffic",
				   &ethqos->io_macro.skip_calc_traffic);
	if (ret) {
		ETHQOSDBG("default skip_calc_traffic\n");
		ethqos->io_macro.skip_calc_traffic = 0;
	}

	ret = of_property_read_u32(np_hw,
				   "data-divide-clk-sel",
				   &ethqos->io_macro.data_divide_clk_sel);
	if (ret) {
		ETHQOSDBG("default data_divide_clk_sel\n");
		ethqos->io_macro.data_divide_clk_sel = 0;
	}

	ret = of_property_read_u32(np_hw,
				   "loopback-en",
				   &ethqos->io_macro.loopback_en);
	if (ret) {
		ETHQOSDBG("default loopback_en\n");
		ethqos->io_macro.loopback_en = 1;
	}

	ret = of_property_read_u32(np_hw,
				   "rx-prog-swap",
				   &ethqos->io_macro.rx_prog_swap);
	if (ret) {
		ETHQOSDBG("default rx_prog_swap\n");
		ethqos->io_macro.rx_prog_swap = 0;
	}

	ret = of_property_read_u32(np_hw,
				   "tx-clk-phase-shift-en",
				   &ethqos->io_macro.tx_clk_phase_shift_en);
	if (ret) {
		ETHQOSDBG("default tx_clk_phase_shift_en\n");
		ethqos->io_macro.tx_clk_phase_shift_en = 0;
	}

	ret = of_property_read_u32(np_hw,
				   "dll-clock-dis",
				   &ethqos->io_macro.dll_clock_dis);
	if (ret) {
		ETHQOSDBG("default dll_clock_dis\n");
		ethqos->io_macro.dll_clock_dis = 0;
	}

	ret = of_property_read_u32(np_hw,
				   "mclk-freq-calc",
				   &ethqos->io_macro.mclk_freq_calc);
	if (ret) {
		ETHQOSDBG("default mclk_freq_calc\n");
		ethqos->io_macro.mclk_freq_calc = 0x1A;
	}

	ret = of_property_read_u32(np_hw,
				   "ddr-traffic-init-sel",
				   &ethqos->io_macro.ddr_traffic_init_sel);
	if (ret) {
		ETHQOSDBG("default ddr_traffic_init_sel\n");
		ethqos->io_macro.ddr_traffic_init_sel = 1;
	}

	ret = of_property_read_u32(np_hw,
				   "ddr-traffic-init-sw",
				   &ethqos->io_macro.ddr_traffic_init_sw);
	if (ret) {
		ETHQOSDBG("default ddr_traffic_init_sw\n");
		ethqos->io_macro.ddr_traffic_init_sw = 1;
	}

	ret = of_property_read_u32(np_hw,
				   "ddr-cal-en",
				   &ethqos->io_macro.ddr_cal_en);
	if (ret) {
		ETHQOSDBG("default ddr_cal_en\n");
		ethqos->io_macro.ddr_cal_en = 1;
	}

	ret = of_property_read_u32(np_hw,
				   "tcx0-cycles-dly-line",
				   &ethqos->io_macro.tcx0_cycles_dly_line);
	if (ret) {
		ETHQOSDBG("default tcx0_cycles_dly_line\n");
		ethqos->io_macro.tcx0_cycles_dly_line = 0;
	}

	ret = of_property_read_u32(np_hw,
				   "tcx0-cycles-cnt",
				   &ethqos->io_macro.tcx0_cycles_cnt);
	if (ret) {
		ETHQOSDBG("default tcx0_cycles_cnt\n");
		ethqos->io_macro.tcx0_cycles_cnt = 0;
	}

	ret = of_property_read_u32(np_hw,
				   "test-ctl",
				   &ethqos->io_macro.test_ctl);
	if (ret) {
		ETHQOSDBG("default test_ctl\n");
		ethqos->io_macro.test_ctl = 0;
	}

	ret = of_property_read_u32(np_hw,
				   "usr-ctl",
				   &ethqos->io_macro.usr_ctl);
	if (ret) {
		ETHQOSDBG("default usr_ctl\n");
		ethqos->io_macro.usr_ctl = 0;
	}

	ret = of_property_read_u32(np_hw,
				   "pps-create",
				   &ethqos->io_macro.pps_create);
	if (ret) {
		ETHQOSDBG("default pps_create\n");
		ethqos->io_macro.pps_create = 0;
	}

	ret = of_property_read_u32(np_hw,
				   "pps-remove",
				   &ethqos->io_macro.pps_remove);
	if (ret) {
		ETHQOSDBG("default pps_remove\n");
		ethqos->io_macro.pps_remove = 0;
	}

	ret = of_property_read_u32(np_hw,
				   "l3-master-dev",
				   &ethqos->io_macro.l3_master_dev);
	if (ret) {
		ETHQOSDBG("default l3_master_dev\n");
		ethqos->io_macro.l3_master_dev = 0;
	}

	ret = of_property_read_u32(np_hw,
				   "ipv6-wq",
				   &ethqos->io_macro.ipv6_wq);
	if (ret) {
		ETHQOSDBG("default ipv6_wq\n");
		ethqos->io_macro.ipv6_wq = 0;
	}

}

static void qcom_ethqos_bringup_iface(struct work_struct *work)
{
	struct platform_device *pdev = NULL;
	struct net_device *ndev = NULL;
	struct qcom_ethqos *ethqos =
		container_of(work, struct qcom_ethqos, early_eth);

	ETHQOSINFO("entry\n");
	if (!ethqos)
		return;
	pdev = ethqos->pdev;
	if (!pdev)
		return;
	ndev = platform_get_drvdata(pdev);
	if (!ndev || netif_running(ndev))
		return;
	rtnl_lock();
	if (dev_change_flags(ndev, ndev->flags | IFF_UP, NULL) < 0)
		ETHQOSINFO("ERROR\n");
	rtnl_unlock();
	ETHQOSINFO("exit\n");
}

static void read_mac_addr_from_fuse_reg(struct device_node *np)
{
	int ret, i, count, x;
	u32 mac_efuse_prop, efuse_size = 8;
	u64 mac_addr;

	/* If the property doesn't exist or empty return */
	count = of_property_count_u32_elems(np, "mac-efuse-addr");
	if (!count || count < 0)
		return;

	/* Loop over all addresses given until we get valid address */
	for (x = 0; x < count; x++) {
		void __iomem *mac_efuse_addr;

		ret = of_property_read_u32_index(np, "mac-efuse-addr",
						 x, &mac_efuse_prop);
		if (!ret) {
			mac_efuse_addr = ioremap(mac_efuse_prop, efuse_size);
			if (!mac_efuse_addr)
				continue;

			mac_addr = readq(mac_efuse_addr);
			ETHQOSINFO("Mac address read: %llx\n", mac_addr);

			/* create byte array out of value read from efuse */
			for (i = 0; i < ETH_ALEN ; i++) {
				pparams.mac_addr[ETH_ALEN - 1 - i] =
					mac_addr & 0xff;
				mac_addr = mac_addr >> 8;
			}

			iounmap(mac_efuse_addr);

			/* if valid address is found set cookie & return */
			pparams.is_valid_mac_addr =
				is_valid_ether_addr(pparams.mac_addr);
			if (pparams.is_valid_mac_addr)
				return;
		}
	}
}

static void ethqos_is_ipv4_NW_stack_ready(struct work_struct *work)
{
	struct delayed_work *dwork;
	struct qcom_ethqos *ethqos;
	struct platform_device *pdev = NULL;
	struct net_device *ndev = NULL;
	int ret;

	ETHQOSDBG("\n");
	dwork = container_of(work, struct delayed_work, work);
	ethqos = container_of(dwork, struct qcom_ethqos, ipv4_addr_assign_wq);

	if (!ethqos)
		return;

	pdev = ethqos->pdev;

	if (!pdev)
		return;

	ndev = platform_get_drvdata(pdev);

	ret = qcom_ethqos_add_ipaddr(&pparams, ndev);
	if (ret)
		return;

	cancel_delayed_work_sync(&ethqos->ipv4_addr_assign_wq);
	flush_delayed_work(&ethqos->ipv4_addr_assign_wq);
}

#ifdef CONFIG_IPV6
static void ethqos_is_ipv6_NW_stack_ready(struct work_struct *work)
{
	struct delayed_work *dwork;
	struct qcom_ethqos *ethqos;
	struct platform_device *pdev = NULL;
	struct net_device *ndev = NULL;
	int ret;

	ETHQOSDBG("\n");
	dwork = container_of(work, struct delayed_work, work);
	ethqos = container_of(dwork, struct qcom_ethqos, ipv6_addr_assign_wq);

	if (!ethqos)
		return;

	pdev = ethqos->pdev;

	if (!pdev)
		return;

	ndev = platform_get_drvdata(pdev);

	ret = qcom_ethqos_add_ipv6addr(&pparams, ndev);
	if (ret)
		return;

	cancel_delayed_work_sync(&ethqos->ipv6_addr_assign_wq);
	flush_delayed_work(&ethqos->ipv6_addr_assign_wq);
}
#endif

static void ethqos_set_early_eth_param(struct stmmac_priv *priv,
				       struct qcom_ethqos *ethqos)
{
	int ret = 0;

	if (priv->plat && priv->plat->mdio_bus_data)
		priv->plat->mdio_bus_data->phy_mask =
		 priv->plat->mdio_bus_data->phy_mask | DUPLEX_FULL | SPEED_100;


	if (pparams.is_valid_ipv4_addr) {
		INIT_DELAYED_WORK(&ethqos->ipv4_addr_assign_wq,
				  ethqos_is_ipv4_NW_stack_ready);
		schedule_delayed_work(&ethqos->ipv4_addr_assign_wq, 0);
	}

#ifdef CONFIG_IPV6
	if (pparams.is_valid_ipv6_addr) {
		INIT_DELAYED_WORK(&ethqos->ipv6_addr_assign_wq,
				  ethqos_is_ipv6_NW_stack_ready);
		if (ethqos->io_macro.ipv6_wq) {
			schedule_delayed_work(&ethqos->ipv6_addr_assign_wq,
					      msecs_to_jiffies(1000));
		} else {
			ret = qcom_ethqos_add_ipv6addr(&pparams, priv->dev);
			if (ret)
				schedule_delayed_work(&ethqos->ipv6_addr_assign_wq,
						      msecs_to_jiffies(1000));
		}
	}
#endif
	return;
}

static ssize_t read_phy_reg_dump(struct file *file, char __user *user_buf,
				 size_t count, loff_t *ppos)
{
	struct qcom_ethqos *ethqos = file->private_data;
	struct platform_device *pdev;
	struct net_device *dev;
	struct stmmac_priv *priv;
	unsigned int len = 0, buf_len = 2000;
	char *buf;
	ssize_t ret_cnt;
	int phydata = 0;
	int i = 0;

	if (!ethqos) {
		ETHQOSERR("NULL Pointer\n");
		return -EINVAL;
	}

	pdev = ethqos->pdev;
	dev = platform_get_drvdata(pdev);
	priv = netdev_priv(dev);

	if (!dev->phydev) {
		ETHQOSERR("NULL Pointer\n");
		return -EINVAL;
	}

	buf = kzalloc(buf_len, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	len += scnprintf(buf + len, buf_len - len,
					 "\n************* PHY Reg dump *************\n");

	for (i = 0; i < 32; i++) {
		phydata = ethqos_mdio_read(priv, priv->plat->phy_addr, i);
		len += scnprintf(buf + len, buf_len - len,
					 "MII Register (%#x) = %#x\n",
					 i, phydata);
	}

	if (len > buf_len) {
		ETHQOSERR("(len > buf_len) buffer not sufficient\n");
		len = buf_len;
	}

	ret_cnt = simple_read_from_buffer(user_buf, count, ppos, buf, len);
	kfree(buf);
	return ret_cnt;
}

static ssize_t read_rgmii_reg_dump(struct file *file,
				   char __user *user_buf, size_t count,
				   loff_t *ppos)
{
	struct qcom_ethqos *ethqos = file->private_data;
	struct platform_device *pdev;
	struct net_device *dev;
	unsigned int len = 0, buf_len = 2000;
	char *buf;
	ssize_t ret_cnt;
	int rgmii_data = 0;

	if (!ethqos) {
		ETHQOSERR("NULL Pointer\n");
		return -EINVAL;
	}

	pdev = ethqos->pdev;
	dev = platform_get_drvdata(pdev);

	if (!dev->phydev) {
		ETHQOSERR("NULL Pointer\n");
		return -EINVAL;
	}

	buf = kzalloc(buf_len, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	len += scnprintf(buf + len, buf_len - len,
					 "\n************* RGMII Reg dump *************\n");
	rgmii_data = rgmii_readl(ethqos, RGMII_IO_MACRO_CONFIG);
	len += scnprintf(buf + len, buf_len - len,
					 "RGMII_IO_MACRO_CONFIG Register = %#x\n",
					 rgmii_data);
	rgmii_data = rgmii_readl(ethqos, SDCC_HC_REG_DLL_CONFIG);
	len += scnprintf(buf + len, buf_len - len,
					 "SDCC_HC_REG_DLL_CONFIG Register = %#x\n",
					 rgmii_data);
	rgmii_data = rgmii_readl(ethqos, SDCC_HC_REG_DDR_CONFIG);
	len += scnprintf(buf + len, buf_len - len,
					 "SDCC_HC_REG_DDR_CONFIG Register = %#x\n",
					 rgmii_data);
	rgmii_data = rgmii_readl(ethqos, SDCC_HC_REG_DLL_CONFIG2);
	len += scnprintf(buf + len, buf_len - len,
					 "SDCC_HC_REG_DLL_CONFIG2 Register = %#x\n",
					 rgmii_data);
	rgmii_data = rgmii_readl(ethqos, SDC4_STATUS);
	len += scnprintf(buf + len, buf_len - len,
					 "SDC4_STATUS Register = %#x\n",
					 rgmii_data);
	rgmii_data = rgmii_readl(ethqos, SDCC_TEST_CTL);
	len += scnprintf(buf + len, buf_len - len,
					 "SDCC_TEST_CTL Register = %#x\n",
					 rgmii_data);
	rgmii_data = rgmii_readl(ethqos, SDCC_USR_CTL);
	len += scnprintf(buf + len, buf_len - len,
					 "SDCC_USR_CTL Register = %#x\n",
					 rgmii_data);
	rgmii_data = rgmii_readl(ethqos, RGMII_IO_MACRO_CONFIG2);
	len += scnprintf(buf + len, buf_len - len,
					 "RGMII_IO_MACRO_CONFIG2 Register = %#x\n",
					 rgmii_data);
	rgmii_data = rgmii_readl(ethqos, RGMII_IO_MACRO_DEBUG1);
	len += scnprintf(buf + len, buf_len - len,
					 "RGMII_IO_MACRO_DEBUG1 Register = %#x\n",
					 rgmii_data);
	rgmii_data = rgmii_readl(ethqos, EMAC_SYSTEM_LOW_POWER_DEBUG);
	len += scnprintf(buf + len, buf_len - len,
					 "EMAC_SYSTEM_LOW_POWER_DEBUG Register = %#x\n",
					 rgmii_data);

	if (len > buf_len) {
		ETHQOSERR("(len > buf_len) buffer not sufficient\n");
		len = buf_len;
	}

	ret_cnt = simple_read_from_buffer(user_buf, count, ppos, buf, len);
	kfree(buf);
	return ret_cnt;
}

static const struct file_operations fops_phy_reg_dump = {
	.read = read_phy_reg_dump,
	.open = simple_open,
	.owner = THIS_MODULE,
	.llseek = default_llseek,
};

static const struct file_operations fops_rgmii_reg_dump = {
	.read = read_rgmii_reg_dump,
	.open = simple_open,
	.owner = THIS_MODULE,
	.llseek = default_llseek,
};

static int ethqos_create_debugfs(struct qcom_ethqos        *ethqos)
{
	static struct dentry *phy_reg_dump;
	static struct dentry *rgmii_reg_dump;

	if (!ethqos) {
		ETHQOSERR("Null Param\n");
		return -ENOMEM;
	}

	ethqos->debugfs_dir = debugfs_create_dir("eth", NULL);

	if (!ethqos->debugfs_dir || IS_ERR(ethqos->debugfs_dir)) {
		ETHQOSERR("Can't create debugfs dir\n");
		return -ENOMEM;
	}

	phy_reg_dump = debugfs_create_file("phy_reg_dump", 0400,
					   ethqos->debugfs_dir, ethqos,
					   &fops_phy_reg_dump);
	if (!phy_reg_dump || IS_ERR(phy_reg_dump)) {
		ETHQOSERR("Can't create phy_dump %p\n", phy_reg_dump);
		goto fail;
	}

	rgmii_reg_dump = debugfs_create_file("rgmii_reg_dump", 0400,
					     ethqos->debugfs_dir, ethqos,
					     &fops_rgmii_reg_dump);
	if (!rgmii_reg_dump || IS_ERR(rgmii_reg_dump)) {
		ETHQOSERR("Can't create rgmii_dump %p\n", rgmii_reg_dump);
		goto fail;
	}
	return 0;

fail:
	debugfs_remove_recursive(ethqos->debugfs_dir);
	return -ENOMEM;
}

static int ethqos_cleanup_debugfs(struct qcom_ethqos *ethqos)
{
	if (!ethqos) {
		ETHQOSERR("Null Param");
		return -ENODEV;
	}

	debugfs_remove_recursive(ethqos->debugfs_dir);
	ethqos->debugfs_dir = NULL;

	ETHQOSDBG("debugfs Deleted Successfully");
	return 0;
}

static u32 l3mdev_fib_table1(const struct net_device *dev)
{
	return RT_TABLE_LOCAL;
}

static const struct l3mdev_ops l3mdev_op1 = {.l3mdev_fib_table = l3mdev_fib_table1};

static inline u32 qcom_ethqos_rgmii_io_macro_num_of_regs(u32 emac_hw_version)
{
	switch (emac_hw_version) {
	case EMAC_HW_v2_1_1:
	case EMAC_HW_v2_1_2:
	case EMAC_HW_v2_3_1:
		return 27;
	case EMAC_HW_v2_3_0:
		return 28;
	case EMAC_HW_NONE:
	default:
		return 0;
	}
}

static int qcom_ethqos_init_panic_notifier(struct qcom_ethqos *ethqos)
{
	u32 size_iomacro_regs;
	int ret = 0;

	if (pethqos) {
		size_iomacro_regs =
		qcom_ethqos_rgmii_io_macro_num_of_regs(pethqos->emac_ver) * 4;

		pethqos->emac_reg_base_address =
			kzalloc(pethqos->emac_mem_size, GFP_KERNEL);

		if (!pethqos->emac_reg_base_address)
			ret = -ENOMEM;

		pethqos->rgmii_reg_base_address =
			kzalloc(size_iomacro_regs, GFP_KERNEL);

		if (!pethqos->rgmii_reg_base_address)
			ret = -ENOMEM;
	}
	return ret;
}

static int qcom_ethqos_panic_notifier(struct notifier_block *this,
				      unsigned long event, void *ptr)
{
	u32 size_iomacro_regs;
	struct stmmac_priv *priv = NULL;

	if (pethqos) {
		priv = qcom_ethqos_get_priv(pethqos);

		pr_info("qcom-ethqos: ethqos 0x%p\n", pethqos);
		pr_info("qcom-ethqos: stmmac_priv 0x%p\n", priv);

		pethqos->iommu_domain = priv->plat->stmmac_emb_smmu_ctx.iommu_domain;

		pr_info("qcom-ethqos: emac iommu domain 0x%p\n",
			pethqos->iommu_domain);
		pr_info("qcom-ethqos: emac register mem 0x%p\n",
			pethqos->emac_reg_base_address);

		if (pethqos->emac_reg_base_address)
			memcpy_fromio(pethqos->emac_reg_base_address,
				      pethqos->ioaddr,
				      pethqos->emac_mem_size);

		pr_info("qcom-ethqos: rgmii register mem 0x%p\n",
			pethqos->rgmii_reg_base_address);

		size_iomacro_regs =
		qcom_ethqos_rgmii_io_macro_num_of_regs(pethqos->emac_ver) * 4;

		if (pethqos->rgmii_reg_base_address)
			memcpy(pethqos->rgmii_reg_base_address,
			       __io_virt(pethqos->rgmii_base),
			       size_iomacro_regs);
	}
	return NOTIFY_DONE;
}

static struct notifier_block qcom_ethqos_panic_blk = {
	.notifier_call  = qcom_ethqos_panic_notifier,
};

static int _qcom_ethqos_probe(void *arg)
{
	struct platform_device *pdev = (struct platform_device *)arg;
	struct device_node *np = pdev->dev.of_node;
	struct device_node *rgmii_io_macro_node = NULL;
	struct stmmac_resources stmmac_res;
	struct qcom_ethqos *ethqos = NULL;
	struct resource *res = NULL;
	int ret;
	struct net_device *ndev;
	struct stmmac_priv *priv;

	if (of_device_is_compatible(pdev->dev.of_node,
				    "qcom,emac-smmu-embedded"))
		return emac_emb_smmu_cb_probe(pdev, plat_dat);

#ifdef CONFIG_QGKI_MSM_BOOT_TIME_MARKER
	place_marker("M - Ethernet probe start");
#endif

#if IS_ENABLED(CONFIG_IPC_LOGGING)
	ipc_emac_log_ctxt = ipc_log_context_create(IPCLOG_STATE_PAGES,
						   "emac", 0);
	if (!ipc_emac_log_ctxt)
		ETHQOSERR("Error creating logging context for emac\n");
	else
		ETHQOSINFO("IPC logging has been enabled for emac\n");
#endif

	ret = stmmac_get_platform_resources(pdev, &stmmac_res);
	if (ret)
		return ret;

	ethqos = devm_kzalloc(&pdev->dev, sizeof(*ethqos), GFP_KERNEL);
	if (!ethqos) {
		return -ENOMEM;
	}

	ethqos->pdev = pdev;

	ethqos->phyad_change = false;
	if (of_property_read_bool(np, "qcom,phyad_change")) {
		ethqos->phyad_change = true;
		ETHQOSDBG("qcom,phyad_change present\n");
	}

	ethqos->is_gpio_phy_reset = false;
	if (of_property_read_bool(np, "snps,reset-gpios")) {
		ethqos->is_gpio_phy_reset = true;
		ETHQOSDBG("qcom,phy-reset present\n");
	}

	if (of_property_read_u32(np, "qcom,phyvoltage_min",
				 &ethqos->phyvoltage_min))
		ethqos->phyvoltage_min = 3075000;
	else
		ETHQOSINFO("qcom,phyvoltage_min = %d\n",
			   ethqos->phyvoltage_min);

	if (of_property_read_u32(np, "qcom,phyvoltage_max",
				 &ethqos->phyvoltage_max))
		ethqos->phyvoltage_max = 3200000;
	else
		ETHQOSINFO("qcom,phyvoltage_max = %d\n",
			   ethqos->phyvoltage_max);

	ethqos_init_regulators(ethqos);
	ethqos_init_gpio(ethqos);

	plat_dat = stmmac_probe_config_dt(pdev, &stmmac_res.mac);
	if (IS_ERR(plat_dat)) {
		dev_err(&pdev->dev, "dt configuration failed\n");
		return PTR_ERR(plat_dat);
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "rgmii");
	ethqos->rgmii_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(ethqos->rgmii_base)) {
		dev_err(&pdev->dev, "Can't get rgmii base\n");
		ret = PTR_ERR(ethqos->rgmii_base);
		goto err_mem;
	}

	ethqos->rgmii_clk = devm_clk_get(&pdev->dev, "rgmii");
	if (IS_ERR(ethqos->rgmii_clk)) {
		ret = PTR_ERR(ethqos->rgmii_clk);
		goto err_mem;
	}

	ret = clk_prepare_enable(ethqos->rgmii_clk);
	if (ret)
		goto err_mem;

	/* Read mac address from fuse register */
	read_mac_addr_from_fuse_reg(np);

	/*Initialize Early ethernet to false*/
	ethqos->early_eth_enabled = false;

	/*Check for valid mac, ip address to enable Early eth*/
	if (pparams.is_valid_mac_addr &&
	    (pparams.is_valid_ipv4_addr || pparams.is_valid_ipv6_addr)) {
		/* For 1000BASE-T mode, auto-negotiation is required and
		 * always used to establish a link.
		 * Configure phy and MAC in 100Mbps mode with autoneg
		 * disable as link up takes more time with autoneg
		 * enabled.
		 */
		ethqos->early_eth_enabled = true;
		ETHQOSINFO("Early ethernet is enabled\n");
	}

	ethqos->speed = SPEED_10;
	ethqos_update_rgmii_clk_and_bus_cfg(ethqos, SPEED_10);
	ethqos_set_func_clk_en(ethqos);

	plat_dat->bsp_priv = ethqos;
	plat_dat->fix_mac_speed = ethqos_fix_mac_speed;
	plat_dat->tx_select_queue = dwmac_qcom_select_queue;
	plat_dat->get_plat_tx_coal_frames =  dwmac_qcom_get_plat_tx_coal_frames;
	plat_dat->has_gmac4 = 1;
	plat_dat->tso_en = of_property_read_bool(np, "snps,tso");
	plat_dat->early_eth = ethqos->early_eth_enabled;
	plat_dat->handle_prv_ioctl = ethqos_handle_prv_ioctl;
	plat_dat->request_phy_wol = qcom_ethqos_request_phy_wol;
	plat_dat->init_pps = ethqos_init_pps;
	plat_dat->phy_intr_enable = ethqos_phy_intr_enable;
	plat_dat->phy_irq_enable = ethqos_phy_irq_enable;
	plat_dat->phy_irq_disable = ethqos_phy_irq_disable;
	plat_dat->phyad_change = ethqos->phyad_change;
	plat_dat->is_gpio_phy_reset = ethqos->is_gpio_phy_reset;

	/* Get rgmii interface speed for mac2c from device tree */
	if (of_property_read_u32(np, "mac2mac-rgmii-speed",
				 &plat_dat->mac2mac_rgmii_speed))
		plat_dat->mac2mac_rgmii_speed = -1;
	else
		ETHQOSINFO("mac2mac rgmii speed = %d\n",
			   plat_dat->mac2mac_rgmii_speed);

	if (of_property_read_bool(pdev->dev.of_node,
				  "emac-core-version")) {
		/* Read emac core version value from dtsi */
		ret = of_property_read_u32(pdev->dev.of_node,
					   "emac-core-version",
					   &ethqos->emac_ver);
		if (ret) {
			ETHQOSDBG(":resource emac-hw-ver! not in dtsi\n");
			ethqos->emac_ver = EMAC_HW_NONE;
			WARN_ON(1);
		}
	} else {
		ethqos->emac_ver =
		rgmii_readl(ethqos, EMAC_I0_EMAC_CORE_HW_VERSION_RGOFFADDR);
	}
	ETHQOSDBG(": emac_core_version = %d\n", ethqos->emac_ver);

	/* Update io-macro settings from device tree */
	 rgmii_io_macro_node = of_find_node_by_name(pdev->dev.of_node,
						    "rgmii-io-macro-info");
	if (rgmii_io_macro_node) {
		ETHQOSINFO("rgmii_io_macro_node found in dt\n");
		read_rgmii_io_macro_node_setting(rgmii_io_macro_node, ethqos);
	} else {
		ETHQOSINFO("rgmii_io_macro_node not found in dt\n");
	}

	if (of_property_read_bool(pdev->dev.of_node, "qcom,arm-smmu")) {
		emac_emb_smmu_ctx.pdev_master = pdev;
		ret = of_platform_populate(pdev->dev.of_node,
					   qcom_ethqos_match, NULL, &pdev->dev);
		if (ret)
			ETHQOSERR("Failed to populate EMAC platform\n");
		if (emac_emb_smmu_ctx.ret) {
			ETHQOSERR("smmu probe failed\n");
			of_platform_depopulate(&pdev->dev);
			ret = emac_emb_smmu_ctx.ret;
			emac_emb_smmu_ctx.ret = 0;
		}
	}

	ret = stmmac_dvr_probe(&pdev->dev, plat_dat, &stmmac_res);
	if (ret)
		goto err_clk;

	pethqos = ethqos;
	ethqos_create_debugfs(ethqos);

	ndev = dev_get_drvdata(&ethqos->pdev->dev);
	priv = netdev_priv(ndev);

	rgmii_dump(ethqos);

	ethqos->ioaddr = (&stmmac_res)->addr;

	if (ethqos->io_macro.pps_create) {
		ethqos_pps_irq_config(ethqos);
		create_pps_interrupt_device_node(&ethqos->avb_class_a_dev_t,
						 &ethqos->avb_class_a_cdev,
						 &ethqos->avb_class_a_class,
						 AVB_CLASS_A_POLL_DEV_NODE);

		create_pps_interrupt_device_node(&ethqos->avb_class_b_dev_t,
						 &ethqos->avb_class_b_cdev,
						 &ethqos->avb_class_b_class,
						 AVB_CLASS_B_POLL_DEV_NODE);
	}

	qcom_ethqos_read_iomacro_por_values(ethqos);

	if (pparams.is_valid_mac_addr) {
		priv->dev->addr_assign_type = NET_ADDR_PERM;
		ether_addr_copy(dev_addr, pparams.mac_addr);
		memcpy(priv->dev->dev_addr, dev_addr, ETH_ALEN);
		ETHQOSINFO("using partition device MAC address %pM\n", priv->dev->dev_addr);
	}

	res = platform_get_resource(ethqos->pdev, IORESOURCE_MEM, 0);
	if (!res) {
		ETHQOSERR("get emac-base resource failed\n");
		ret = -ENOMEM;
		goto err_clk;
	}

	ethqos->emac_mem_size = resource_size(res);

	if (ethqos->early_eth_enabled) {
		/* Initialize work*/
		INIT_WORK(&ethqos->early_eth,
			  qcom_ethqos_bringup_iface);
		/* Queue the work*/
		queue_work(system_wq, &ethqos->early_eth);
		/*Set early eth parameters*/
		ethqos_set_early_eth_param(priv, ethqos);
	}

	if (priv->plat->mac2mac_en)
		priv->plat->mac2mac_link = -1;

	if (!qcom_ethqos_init_panic_notifier(ethqos))
		atomic_notifier_chain_register(&panic_notifier_list,
					       &qcom_ethqos_panic_blk);

#ifdef CONFIG_QGKI_MSM_BOOT_TIME_MARKER
	place_marker("M - Ethernet probe end");
#endif

#ifdef CONFIG_NET_L3_MASTER_DEV
	if (ethqos->early_eth_enabled &&
	    ethqos->io_macro.l3_master_dev) {
		ETHQOSINFO("l3mdev_op1 set\n");
		ndev->priv_flags = IFF_L3MDEV_MASTER;
		ndev->l3mdev_ops = &l3mdev_op1;
	}
#endif

	return ret;

err_clk:
	clk_disable_unprepare(ethqos->rgmii_clk);

err_mem:
	stmmac_remove_config_dt(pdev, plat_dat);

	return ret;
}

static int qcom_ethqos_probe(struct platform_device *pdev)
{
#ifdef CONFIG_PLATFORM_AUTO
	struct task_struct *ethqos_task = kthread_run(_qcom_ethqos_probe, pdev,
			"ethqos_probe");
	if (PTR_ERR_OR_ZERO(ethqos_task))
		return PTR_ERR(ethqos_task);
	else
		return 0;
#else
	return _qcom_ethqos_probe(pdev);
#endif
}

static int qcom_ethqos_remove(struct platform_device *pdev)
{
	struct qcom_ethqos *ethqos;
	int ret;
	struct stmmac_priv *priv;

	if (of_device_is_compatible(pdev->dev.of_node, "qcom,emac-smmu-embedded")) {
		of_platform_depopulate(&pdev->dev);
		return 0;
	}

	ethqos = get_stmmac_bsp_priv(&pdev->dev);
	if (!ethqos)
		return -ENODEV;

	priv = qcom_ethqos_get_priv(pethqos);

	ret = stmmac_pltfr_remove(pdev);
	clk_disable_unprepare(ethqos->rgmii_clk);

	if (priv->plat->phy_intr_en_extn_stm)
		free_irq(ethqos->phy_intr, ethqos);
	priv->phy_irq_enabled = false;

	if (priv->plat->phy_intr_en_extn_stm)
		cancel_work_sync(&ethqos->emac_phy_work);

	if (ethqos->io_macro.pps_remove)
		ethqos_remove_pps_dev(ethqos);

	ethqos_cleanup_debugfs(ethqos);
	ethqos_free_gpios(ethqos);
	emac_emb_smmu_exit();
	ethqos_disable_regulators(ethqos);

	atomic_notifier_chain_unregister(&panic_notifier_list,
					 &qcom_ethqos_panic_blk);

	platform_set_drvdata(pdev, NULL);
	of_platform_depopulate(&pdev->dev);

	return ret;
}

static int qcom_ethqos_suspend(struct device *dev)
{
	struct qcom_ethqos *ethqos;
	struct net_device *ndev = NULL;
	int ret;
	struct stmmac_priv *priv;

	if (of_device_is_compatible(dev->of_node, "qcom,emac-smmu-embedded")) {
		ETHQOSDBG("smmu return\n");
		return 0;
	}

	ethqos = get_stmmac_bsp_priv(dev);
	if (!ethqos)
		return -ENODEV;

	ndev = dev_get_drvdata(dev);

	if (!ndev || !netif_running(ndev))
		return -EINVAL;

	priv = netdev_priv(ndev);
	ret = stmmac_suspend(dev);
	qcom_ethqos_phy_suspend_clks(ethqos);

	/* Suspend the PHY TXC clock. */
	if (ethqos->rgmii_txc_suspend_state) {
		/* Remove TXC clock source from Phy.*/
		ret = pinctrl_select_state(ethqos->pinctrl,
					   ethqos->rgmii_txc_suspend_state);
	if (ret)
		ETHQOSERR("Unable to set rgmii_txc_suspend_state state, err = %d\n", ret);
	else
		ETHQOSINFO("Set rgmii_txc_suspend_state succeed\n");
	}

	priv->boot_kpi = false;
	ETHQOSDBG(" ret = %d\n", ret);
	return ret;
}

static int qcom_ethqos_resume(struct device *dev)
{
	struct net_device *ndev = NULL;
	struct qcom_ethqos *ethqos;
	int ret;

	ETHQOSDBG("Resume Enter\n");
	if (of_device_is_compatible(dev->of_node, "qcom,emac-smmu-embedded"))
		return 0;

	ethqos = get_stmmac_bsp_priv(dev);

	if (!ethqos)
		return -ENODEV;

	ndev = dev_get_drvdata(dev);

	if (!ndev || !netif_running(ndev)) {
		ETHQOSERR(" Resume not possible\n");
		return -EINVAL;
	}

	/* Resume the PhY TXC clock. */
	if (ethqos->rgmii_txc_resume_state) {
		/* Enable TXC clock source from Phy.*/
		ret = pinctrl_select_state(ethqos->pinctrl,
					   ethqos->rgmii_txc_resume_state);
		if (ret)
			ETHQOSERR("Unable to set rgmii_rxc_resume_state state, err = %d\n", ret);
		else
			ETHQOSINFO("Set rgmii_rxc_resume_state succeed\n");
	}

	qcom_ethqos_phy_resume_clks(ethqos);

	ret = stmmac_resume(dev);

	ETHQOSDBG("<--Resume Exit\n");
	return ret;
}

static int qcom_ethqos_enable_clks(struct qcom_ethqos *ethqos, struct device *dev)
{
	struct stmmac_priv *priv = qcom_ethqos_get_priv(ethqos);
	int ret = 0;

	/* clock setup */
	priv->plat->stmmac_clk = devm_clk_get(dev,
					      STMMAC_RESOURCE_NAME);
	if (IS_ERR(priv->plat->stmmac_clk)) {
		dev_warn(dev, "stmmac_clk clock failed\n");
		ret = PTR_ERR(priv->plat->stmmac_clk);
		priv->plat->stmmac_clk = NULL;
	} else {
		ret = clk_prepare_enable(priv->plat->stmmac_clk);
		if (ret)
			ETHQOSINFO("stmmac_clk clk failed\n");
	}

	priv->plat->pclk = devm_clk_get(dev, "pclk");
	if (IS_ERR(priv->plat->pclk)) {
		dev_warn(dev, "pclk clock failed\n");
		ret = PTR_ERR(priv->plat->pclk);
		priv->plat->pclk = NULL;
		goto error_pclk_get;
	} else {
		ret = clk_prepare_enable(priv->plat->pclk);
		if (ret) {
			ETHQOSINFO("pclk clk failed\n");
			goto error_pclk_get;
		}
	}

	ethqos->rgmii_clk = devm_clk_get(dev, "rgmii");
	if (IS_ERR(ethqos->rgmii_clk)) {
		dev_warn(dev, "rgmii clock failed\n");
		ret = PTR_ERR(ethqos->rgmii_clk);
		goto error_rgmii_get;
	} else {
		ret = clk_prepare_enable(ethqos->rgmii_clk);
		if (ret) {
			ETHQOSINFO("rgmmi clk failed\n");
			goto error_rgmii_get;
		}
	}
	return 0;

error_rgmii_get:
	clk_disable_unprepare(priv->plat->pclk);
error_pclk_get:
	clk_disable_unprepare(priv->plat->stmmac_clk);
	return ret;
}

static void qcom_ethqos_disable_clks(struct qcom_ethqos *ethqos, struct device *dev)
{
	struct stmmac_priv *priv = qcom_ethqos_get_priv(ethqos);

	ETHQOSINFO("Enter\n");

	if (priv->plat->stmmac_clk)
		clk_disable_unprepare(priv->plat->stmmac_clk);

	if (priv->plat->pclk)
		clk_disable_unprepare(priv->plat->pclk);

	if (ethqos->rgmii_clk)
		clk_disable_unprepare(ethqos->rgmii_clk);

	ETHQOSINFO("Exit\n");
}

static int qcom_ethqos_hib_restore(struct device *dev)
{
	struct qcom_ethqos *ethqos;
	struct stmmac_priv *priv;
	struct net_device *ndev = NULL;
	int ret = 0;

	if (of_device_is_compatible(dev->of_node, "qcom,emac-smmu-embedded"))
		return 0;

	ETHQOSINFO(" start\n");
	ethqos = get_stmmac_bsp_priv(dev);
	if (!ethqos)
		return -ENODEV;

	ndev = dev_get_drvdata(dev);

	if (!ndev)
		return -EINVAL;

	priv = netdev_priv(ndev);

	ret = ethqos_init_regulators(ethqos);
	if (ret)
		return ret;

	ret = ethqos_init_gpio(ethqos);
	if (ret)
		return ret;

	ret = qcom_ethqos_enable_clks(ethqos, dev);
	if (ret)
		return ret;

	ethqos_update_rgmii_clk_and_bus_cfg(ethqos, ethqos->speed);

	ethqos_set_func_clk_en(ethqos);

#ifdef DWC_ETH_QOS_CONFIG_PTP
	if (priv->plat->clk_ptp_ref) {
		ret = clk_prepare_enable(priv->plat->clk_ptp_ref);
		if (ret < 0)
			netdev_warn(priv->dev, "failed to enable PTP reference clock: %d\n", ret);
	}
	ret = stmmac_init_ptp(priv);
	if (ret == -EOPNOTSUPP) {
		netdev_warn(priv->dev, "PTP not supported by HW\n");
	} else if (ret) {
		netdev_warn(priv->dev, "PTP init failed\n");
	} else {
		clk_set_rate(priv->plat->clk_ptp_ref,
			     priv->plat->clk_ptp_rate);
	}

	ret = priv->plat->init_pps(priv);
#endif /* end of DWC_ETH_QOS_CONFIG_PTP */

	/* issue software reset to device */
	ret = stmmac_reset(priv, priv->ioaddr);
	if (ret) {
		dev_err(priv->device, "Failed to reset\n");
		return ret;
	}

	if (!netif_running(ndev)) {
		rtnl_lock();
		dev_open(ndev, NULL);
		rtnl_unlock();
		ETHQOSINFO("calling open\n");
	}

	ETHQOSINFO("end\n");

	return ret;
}

static int qcom_ethqos_hib_freeze(struct device *dev)
{
	struct qcom_ethqos *ethqos;
	struct stmmac_priv *priv;
	int ret = 0;
	struct net_device *ndev = NULL;

	if (of_device_is_compatible(dev->of_node, "qcom,emac-smmu-embedded"))
		return 0;

	ethqos = get_stmmac_bsp_priv(dev);
	if (!ethqos)
		return -ENODEV;

	ndev = dev_get_drvdata(dev);

	if (!ndev)
		return -EINVAL;

	priv = netdev_priv(ndev);

	ETHQOSINFO("start\n");

	if (netif_running(ndev)) {
		rtnl_lock();
		dev_close(ndev);
		rtnl_unlock();
		ETHQOSINFO("calling netdev off\n");
	}

#ifdef DWC_ETH_QOS_CONFIG_PTP
	stmmac_release_ptp(priv);
#endif /* end of DWC_ETH_QOS_CONFIG_PTP */

	qcom_ethqos_disable_clks(ethqos, dev);

	ethqos_disable_regulators(ethqos);

	ethqos_free_gpios(ethqos);

	ETHQOSINFO("end\n");

	return ret;
}

MODULE_DEVICE_TABLE(of, qcom_ethqos_match);

static const struct dev_pm_ops qcom_ethqos_pm_ops = {
	.freeze = qcom_ethqos_hib_freeze,
	.restore = qcom_ethqos_hib_restore,
	.thaw = qcom_ethqos_hib_restore,
	.suspend = qcom_ethqos_suspend,
	.resume = qcom_ethqos_resume,
};

static struct platform_driver qcom_ethqos_driver = {
	.probe  = qcom_ethqos_probe,
	.remove = qcom_ethqos_remove,
	.driver = {
		.name           = DRV_NAME,
		.pm		= &qcom_ethqos_pm_ops,
		.of_match_table = of_match_ptr(qcom_ethqos_match),
	},
};

static int __init qcom_ethqos_init_module(void)
{
	int ret = 0;

	ETHQOSDBG("enter\n");

	ret = platform_driver_register(&qcom_ethqos_driver);
	if (ret < 0) {
		ETHQOSINFO("qcom-ethqos: Driver registration failed");
		return ret;
	}

	ETHQOSDBG("Exit\n");

	return ret;
}

static void __exit qcom_ethqos_exit_module(void)
{
	ETHQOSDBG("Enter\n");

	platform_driver_unregister(&qcom_ethqos_driver);

	ETHQOSDBG("Exit\n");
}

/*!
 * \brief Macro to register the driver registration function.
 *
 * \details A module always begin with either the init_module or the function
 * you specify with module_init call. This is the entry function for modules;
 * it tells the kernel what functionality the module provides and sets up the
 * kernel to run the module's functions when they're needed. Once it does this,
 * entry function returns and the module does nothing until the kernel wants
 * to do something with the code that the module provides.
 */

module_init(qcom_ethqos_init_module)

/*!
 * \brief Macro to register the driver un-registration function.
 *
 * \details All modules end by calling either cleanup_module or the function
 * you specify with the module_exit call. This is the exit function for modules;
 * it undoes whatever entry function did. It unregisters the functionality
 * that the entry function registered.
 */

module_exit(qcom_ethqos_exit_module)

MODULE_DESCRIPTION("Qualcomm ETHQOS driver");
MODULE_LICENSE("GPL v2");
