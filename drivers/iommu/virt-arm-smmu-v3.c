// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2021 The Linux Foundation. All rights reserved.
 *
 * IOMMU API for ARM architected SMMUv3 implementations.
 *
 * Copyright (C) 2015 ARM Limited
 *
 * Author: Will Deacon <will.deacon@arm.com>
 *
 * This driver is powered by bad coffee and bombay mix.
 */

#include <linux/acpi.h>
#include <linux/acpi_iort.h>
#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/crash_dump.h>
#include <linux/delay.h>
#include <linux/dma-iommu.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io-pgtable.h>
#include <linux/iommu.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/msi.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_iommu.h>
#include <linux/of_platform.h>
#include <linux/pci.h>
#include <linux/pci-ats.h>
#include <linux/platform_device.h>
#include <linux/qcom_scm.h>
#include <linux/amba/bus.h>


/* Convert between AArch64 (CPU) TCR format and SMMU CD format */
#define ARM_SMMU_TCR2CD(tcr, fld)	FIELD_PREP(CTXDESC_CD_0_TCR_##fld, \
					FIELD_GET(ARM64_TCR_##fld, tcr))

/*
 * Stream table.
 *
 * Linear: Enough to cover 1 << IDR1.SIDSIZE entries
 * 2lvl: 128k L1 entries,
 *       256 lazy entries per table (each table covers a PCI bus)
 */
#define STRTAB_L1_SZ_SHIFT		20
#define STRTAB_SPLIT			8

#define STRTAB_L1_DESC_DWORDS		1
#define STRTAB_L1_DESC_SPAN		GENMASK_ULL(4, 0)
#define STRTAB_L1_DESC_L2PTR_MASK	GENMASK_ULL(51, 6)

#define STRTAB_STE_DWORDS		8
#define STRTAB_STE_0_V			(1UL << 0)
#define STRTAB_STE_0_CFG		GENMASK_ULL(3, 1)
#define STRTAB_STE_0_CFG_ABORT		0
#define STRTAB_STE_0_CFG_BYPASS		4
#define STRTAB_STE_0_CFG_S1_TRANS	5
#define STRTAB_STE_0_CFG_S2_TRANS	6

#define STRTAB_STE_0_S1FMT		GENMASK_ULL(5, 4)
#define STRTAB_STE_0_S1FMT_LINEAR	0
#define STRTAB_STE_0_S1CTXPTR_MASK	GENMASK_ULL(51, 6)
#define STRTAB_STE_0_S1CDMAX		GENMASK_ULL(63, 59)

#define STRTAB_STE_1_S1C_CACHE_NC	0UL
#define STRTAB_STE_1_S1C_CACHE_WBRA	1UL
#define STRTAB_STE_1_S1C_CACHE_WT	2UL
#define STRTAB_STE_1_S1C_CACHE_WB	3UL
#define STRTAB_STE_1_S1CIR		GENMASK_ULL(3, 2)
#define STRTAB_STE_1_S1COR		GENMASK_ULL(5, 4)
#define STRTAB_STE_1_S1CSH		GENMASK_ULL(7, 6)

#define STRTAB_STE_1_S1STALLD		(1UL << 27)

#define STRTAB_STE_1_EATS		GENMASK_ULL(29, 28)
#define STRTAB_STE_1_EATS_ABT		0UL
#define STRTAB_STE_1_EATS_TRANS		1UL
#define STRTAB_STE_1_EATS_S1CHK		2UL

#define STRTAB_STE_1_STRW		GENMASK_ULL(31, 30)
#define STRTAB_STE_1_STRW_NSEL1		0UL
#define STRTAB_STE_1_STRW_EL2		2UL

#define STRTAB_STE_1_SHCFG		GENMASK_ULL(45, 44)
#define STRTAB_STE_1_SHCFG_INCOMING	1UL

#define STRTAB_STE_2_S2VMID		GENMASK_ULL(15, 0)
#define STRTAB_STE_2_VTCR		GENMASK_ULL(50, 32)
#define STRTAB_STE_2_S2AA64		(1UL << 51)
#define STRTAB_STE_2_S2ENDI		(1UL << 52)
#define STRTAB_STE_2_S2PTW		(1UL << 54)
#define STRTAB_STE_2_S2R		(1UL << 58)

#define STRTAB_STE_3_S2TTB_MASK		GENMASK_ULL(51, 4)

/* Context descriptor (stage-1 only) */
#define CTXDESC_CD_DWORDS		8
#define CTXDESC_CD_0_TCR_T0SZ		GENMASK_ULL(5, 0)
#define ARM64_TCR_T0SZ			GENMASK_ULL(5, 0)
#define CTXDESC_CD_0_TCR_TG0		GENMASK_ULL(7, 6)
#define ARM64_TCR_TG0			GENMASK_ULL(15, 14)
#define CTXDESC_CD_0_TCR_IRGN0		GENMASK_ULL(9, 8)
#define ARM64_TCR_IRGN0			GENMASK_ULL(9, 8)
#define CTXDESC_CD_0_TCR_ORGN0		GENMASK_ULL(11, 10)
#define ARM64_TCR_ORGN0			GENMASK_ULL(11, 10)
#define CTXDESC_CD_0_TCR_SH0		GENMASK_ULL(13, 12)
#define ARM64_TCR_SH0			GENMASK_ULL(13, 12)
#define CTXDESC_CD_0_TCR_EPD0		(1ULL << 14)
#define ARM64_TCR_EPD0			(1ULL << 7)
#define CTXDESC_CD_0_TCR_EPD1		(1ULL << 30)
#define ARM64_TCR_EPD1			(1ULL << 23)

#define CTXDESC_CD_0_ENDI		(1UL << 15)
#define CTXDESC_CD_0_V			(1UL << 31)

#define CTXDESC_CD_0_TCR_IPS		GENMASK_ULL(34, 32)
#define ARM64_TCR_IPS			GENMASK_ULL(34, 32)
#define CTXDESC_CD_0_TCR_TBI0		(1ULL << 38)
#define ARM64_TCR_TBI0			(1ULL << 37)

#define CTXDESC_CD_0_AA64		(1UL << 41)
#define CTXDESC_CD_0_S			(1UL << 44)
#define CTXDESC_CD_0_R			(1UL << 45)
#define CTXDESC_CD_0_A			(1UL << 46)
#define CTXDESC_CD_0_ASET		(1UL << 47)
#define CTXDESC_CD_0_ASID		GENMASK_ULL(63, 48)

#define CTXDESC_CD_1_TTB0_MASK		GENMASK_ULL(51, 4)

/* Common memory attribute values */
#define ARM_SMMU_SH_NSH			0
#define ARM_SMMU_SH_OSH			2
#define ARM_SMMU_SH_ISH			3
#define ARM_SMMU_MEMATTR_DEVICE_nGnRE	0x1
#define ARM_SMMU_MEMATTR_OIWB		0xf

struct arm_smmu_s1_cfg {
	__le64				*cdptr;
	dma_addr_t			cdptr_dma;

	struct arm_smmu_ctx_desc {
		u16	asid;
		u64	ttbr;
		u64	tcr;
		u64	mair;
	} cd;
};

struct arm_smmu_ste_cfg {
	__le64				*ste;
	dma_addr_t			stedma;
	u32				sid;
};

struct virt_arm_smmu_device {
	struct device		*dev;
	unsigned long		ias;
	unsigned long		oas;
#define ARM_SMMU_MAX_ASIDS              (1 << 16)
	unsigned int		asid_bits;
	DECLARE_BITMAP(asid_map, ARM_SMMU_MAX_ASIDS);

#define ARM_SMMU_MAX_VMIDS              (1 << 16)
	unsigned int		vmid_bits;
	DECLARE_BITMAP(vmid_map, ARM_SMMU_MAX_VMIDS);
	unsigned long		pgsize_bitmap;

	struct iommu_device	iommu;
};

struct virt_arm_smmu_domain {
	struct virt_arm_smmu_device	*smmu;
	struct arm_smmu_s1_cfg		s1_cfg;
	struct mutex			init_mutex;
	struct msm_io_pgtable_info      pgtbl_info;
	struct io_pgtable_ops		*pgtbl_ops;
	struct iommu_domain		domain;
	struct list_head		devices;
	spinlock_t			devices_lock;
};

struct virt_arm_smmu_master {
	struct virt_arm_smmu_device	*smmu;
	struct device			*dev;
	struct virt_arm_smmu_domain	*domain;
	struct arm_smmu_ste_cfg		*ste_cfg;
	struct list_head domain_head;
	u32				*sids;
	unsigned int			num_sids;
};

static int arm_smmu_bitmap_alloc(unsigned long *map, int span)
{
	int idx, size = 1 << span;

	do {
		idx = find_first_zero_bit(map, size);
		if (idx == size)
			return -ENOSPC;
	} while (test_and_set_bit(idx, map));

	return idx;
}

static void arm_smmu_bitmap_free(unsigned long *map, int idx)
{
	clear_bit(idx, map);
}

static struct virt_arm_smmu_domain *to_smmu_domain(struct iommu_domain *dom)
{
	return container_of(dom, struct virt_arm_smmu_domain, domain);
}

static int virt_arm_smmu_domain_finalise_s1(struct virt_arm_smmu_domain *smmu_domain,
				       struct io_pgtable_cfg *pgtbl_cfg)
{
	int ret;
	int asid;
	struct virt_arm_smmu_device *smmu = smmu_domain->smmu;
	struct arm_smmu_s1_cfg *cfg = &smmu_domain->s1_cfg;

	asid = arm_smmu_bitmap_alloc(smmu->asid_map, smmu->asid_bits);
	if (asid < 0)
		return asid;

	cfg->cdptr = dmam_alloc_coherent(smmu->dev, CTXDESC_CD_DWORDS << 3,
					 &cfg->cdptr_dma,
					 GFP_KERNEL | __GFP_ZERO);
	if (!cfg->cdptr) {
		dev_warn(smmu->dev, "failed to allocate context descriptor\n");
		ret = -ENOMEM;
		goto out_free_asid;
	}

	cfg->cd.asid	= (u16)asid;
	cfg->cd.ttbr	= pgtbl_cfg->arm_lpae_s1_cfg.ttbr[0];
	cfg->cd.tcr	= pgtbl_cfg->arm_lpae_s1_cfg.tcr;
	cfg->cd.mair	= pgtbl_cfg->arm_lpae_s1_cfg.mair[0];
	return 0;

out_free_asid:
	arm_smmu_bitmap_free(smmu->asid_map, asid);
	return ret;
}

static u64 arm_smmu_cpu_tcr_to_cd(u64 tcr)
{
	u64 val = 0;

	val |= ARM_SMMU_TCR2CD(tcr, T0SZ);
	val |= ARM_SMMU_TCR2CD(tcr, TG0);
	val |= ARM_SMMU_TCR2CD(tcr, IRGN0);
	val |= ARM_SMMU_TCR2CD(tcr, ORGN0);
	val |= ARM_SMMU_TCR2CD(tcr, SH0);
	val |= ARM_SMMU_TCR2CD(tcr, EPD0);
	val |= ARM_SMMU_TCR2CD(tcr, EPD1);
	val |= ARM_SMMU_TCR2CD(tcr, IPS);

	return val;
}

static void virt_arm_smmu_write_ctx_desc(struct virt_arm_smmu_device *smmu,
				    struct arm_smmu_s1_cfg *cfg)
{
	u64 val;

	val = arm_smmu_cpu_tcr_to_cd(cfg->cd.tcr) |
#ifdef __BIG_ENDIAN
	      CTXDESC_CD_0_ENDI |
#endif
	      CTXDESC_CD_0_R | CTXDESC_CD_0_A | CTXDESC_CD_0_ASET |
	      CTXDESC_CD_0_AA64 | FIELD_PREP(CTXDESC_CD_0_ASID, cfg->cd.asid) |
	      CTXDESC_CD_0_V;

	cfg->cdptr[0] = cpu_to_le64(val);

	val = cfg->cd.ttbr & CTXDESC_CD_1_TTB0_MASK;
	cfg->cdptr[1] = cpu_to_le64(val);

	cfg->cdptr[3] = cpu_to_le64(cfg->cd.mair);
}

static const struct iommu_flush_ops virt_arm_smmu_flush_ops;

static int arm_smmu_domain_finalise(struct iommu_domain *domain)
{
	int ret;
	unsigned long ias, oas;
	enum io_pgtable_fmt fmt;
	struct io_pgtable_ops *pgtbl_ops;
	int (*finalise_stage_fn)(struct virt_arm_smmu_domain *func,
				 struct io_pgtable_cfg *cfg);
	struct virt_arm_smmu_domain *smmu_domain = to_smmu_domain(domain);
	struct virt_arm_smmu_device *smmu = smmu_domain->smmu;
	struct msm_io_pgtable_info *pgtbl_info = &smmu_domain->pgtbl_info;

	if (domain->type == IOMMU_DOMAIN_IDENTITY)
		return 0;

	ias = min_t(unsigned long, 48, VA_BITS);
	oas = smmu->ias;
	fmt = ARM_64_LPAE_S1;
	finalise_stage_fn = virt_arm_smmu_domain_finalise_s1;
	pgtbl_info->iova_base = 0;
	pgtbl_info->iova_end = SZ_4G - 1;
	pgtbl_info->pgtbl_cfg = (struct io_pgtable_cfg) {
		.pgsize_bitmap	= smmu->pgsize_bitmap,
		.ias		= ias,
		.oas		= oas,
		.coherent_walk	= false,
		.tlb		= &virt_arm_smmu_flush_ops,
		.iommu_dev	= smmu->dev,
	};

	pgtbl_ops = alloc_io_pgtable_ops(fmt, &pgtbl_info->pgtbl_cfg, smmu_domain);
	if (!pgtbl_ops)
		return -ENOMEM;

	domain->pgsize_bitmap = pgtbl_info->pgtbl_cfg.pgsize_bitmap;
	domain->geometry.aperture_end = (1UL << pgtbl_info->pgtbl_cfg.ias) - 1;
	domain->geometry.force_aperture = true;

	ret = finalise_stage_fn(smmu_domain, &pgtbl_info->pgtbl_cfg);
	if (ret < 0) {
		free_io_pgtable_ops(pgtbl_ops);
		return ret;
	}

	smmu_domain->pgtbl_ops = pgtbl_ops;
	return 0;
}

static struct platform_driver virt_arm_smmu_driver;

static struct virt_arm_smmu_device *virt_arm_smmu_get_by_fwnode(struct fwnode_handle *fwnode)
{
	struct device *dev = driver_find_device_by_fwnode(&virt_arm_smmu_driver.driver,
							  fwnode);
	put_device(dev);
	return dev ? dev_get_drvdata(dev) : NULL;
}

static void virt_arm_smmu_put_resv_regions(struct device *dev,
				      struct list_head *head)
{

}
static void virt_arm_smmu_get_resv_regions(struct device *dev,
				      struct list_head *head)
{

}

static int virt_arm_smmu_of_xlate(struct device *dev, struct of_phandle_args *args)
{
	return iommu_fwspec_add_ids(dev, args->args, 1);
}

/* Currently no attributes are supported for clients only default S1 translation */

static int virt_arm_smmu_domain_set_attr(struct iommu_domain *domain,
				    enum iommu_attr attr, void *data)
{
	return 0;
}

static int virt_arm_smmu_domain_get_attr(struct iommu_domain *domain,
				    enum iommu_attr attr, void *data)
{
	if (data)
		*(int *)data = 0;
	return 0;
}

static struct iommu_group *virt_arm_smmu_device_group(struct device *dev)
{
	struct iommu_group *group;

	if (dev_is_pci(dev))
		group = pci_device_group(dev);
	else
		group = generic_device_group(dev);

	return group;
}

static void remove_ste_for_dev(struct virt_arm_smmu_master *master)
{
	struct virt_arm_smmu_device *smmu = master->smmu;
	struct arm_smmu_ste_cfg *iter;
	int i;

	iter = master->ste_cfg;

	for (i = 0; i < master->num_sids; i++) {
		if (qcom_scm_paravirt_smmu_detach(iter->sid))
			pr_err("Failed to detach SID:0x%lx\n", iter->sid);
		if (iter->ste)
			dmam_free_coherent(smmu->dev, (STRTAB_STE_DWORDS << 3),
					iter->ste, iter->stedma);
		iter++;
	}
	kfree(master->ste_cfg);
}

static struct msm_iommu_ops virt_arm_smmu_ops;

static void virt_arm_smmu_detach_dev(struct iommu_domain *domain,
					struct device *dev)
{
	unsigned long flags;
	struct iommu_fwspec *fwspec = dev_iommu_fwspec_get(dev);
	struct virt_arm_smmu_domain *smmu_domain = to_smmu_domain(domain);
	struct virt_arm_smmu_master *master;

	if (!fwspec || !smmu_domain)
		return;
	master = fwspec->iommu_priv;

	remove_ste_for_dev(master);
	spin_lock_irqsave(&smmu_domain->devices_lock, flags);
	list_del(&master->domain_head);
	spin_unlock_irqrestore(&smmu_domain->devices_lock, flags);
	master->domain = NULL;
}

static void virt_arm_smmu_remove_device(struct device *dev)
{
	struct iommu_fwspec *fwspec = dev_iommu_fwspec_get(dev);
	struct virt_arm_smmu_master *master;
	struct virt_arm_smmu_device *smmu;

	if (!fwspec || fwspec->ops != &virt_arm_smmu_ops.iommu_ops)
		return;

	master = fwspec->iommu_priv;
	smmu = master->smmu;
	virt_arm_smmu_detach_dev(&master->domain->domain, dev);
	iommu_group_remove_device(dev);
	iommu_device_unlink(&smmu->iommu, dev);
	kfree(master);
	iommu_fwspec_free(dev);
}

static int virt_arm_smmu_add_device(struct device *dev)
{
	struct virt_arm_smmu_device *smmu;
	struct virt_arm_smmu_master *master;
	struct iommu_fwspec *fwspec = dev_iommu_fwspec_get(dev);
	struct iommu_group *group;

	if (!fwspec || fwspec->ops != &virt_arm_smmu_ops.iommu_ops)
		return -ENODEV;
	if (WARN_ON_ONCE(fwspec->iommu_priv)) {
		master = fwspec->iommu_priv;
		smmu = master->smmu;
	} else {
		smmu = virt_arm_smmu_get_by_fwnode(fwspec->iommu_fwnode);
		if (!smmu)
			return -ENODEV;
		master = kzalloc(sizeof(*master), GFP_KERNEL);
		if (!master)
			return -ENOMEM;

		master->dev = dev;
		master->smmu = smmu;
		master->sids = fwspec->ids;
		master->num_sids = fwspec->num_ids;
		fwspec->iommu_priv = master;
	}

	group = iommu_group_get_for_dev(dev);
	if (!IS_ERR(group)) {
		iommu_group_put(group);
		iommu_device_link(&smmu->iommu, dev);
	}

	return PTR_ERR_OR_ZERO(group);
}

static phys_addr_t virt_arm_smmu_iova_to_phys(struct iommu_domain *domain, dma_addr_t iova)
{
	struct io_pgtable_ops *ops = to_smmu_domain(domain)->pgtbl_ops;

	if (domain->type == IOMMU_DOMAIN_IDENTITY)
		return iova;

	if (!ops)
		return 0;

	return ops->iova_to_phys(ops, iova);
}

static void virt_arm_smmu_tlb_inv_sync(unsigned long iova, size_t size,
				   size_t granule, bool leaf,
				   struct virt_arm_smmu_domain *smmu_domain)
{
	u32 asid;

	if (!size)
		return;
	asid = smmu_domain->s1_cfg.cd.asid;

	if (qcom_scm_paravirt_tlb_inv(asid))
		pr_err("SCM called failed for TLB inv: asid:0x%lx\n", asid);
}


static void virt_arm_smmu_tlb_inv_context(void *cookie)
{

	struct virt_arm_smmu_domain *smmu_domain = cookie;
	u32 asid = smmu_domain->s1_cfg.cd.asid;

	if (qcom_scm_paravirt_tlb_inv(asid))
		pr_err("SCM called failed for TLB inv: asid:0x%lxi\n", asid);
}

static void virt_arm_smmu_iotlb_sync(struct iommu_domain *domain,
				struct iommu_iotlb_gather *gather)
{
	struct virt_arm_smmu_domain *smmu_domain = to_smmu_domain(domain);

	virt_arm_smmu_tlb_inv_sync(gather->start, gather->end - gather->start,
			       gather->pgsize, true, smmu_domain);
}

static void virt_arm_smmu_flush_iotlb_all(struct iommu_domain *domain)
{
	struct virt_arm_smmu_domain *smmu_domain = to_smmu_domain(domain);

	if (smmu_domain->smmu)
		virt_arm_smmu_tlb_inv_context(smmu_domain);
}

static size_t virt_arm_smmu_unmap(struct iommu_domain *domain, unsigned long iova,
			     size_t size, struct iommu_iotlb_gather *gather)
{
	struct virt_arm_smmu_domain *smmu_domain = to_smmu_domain(domain);
	struct io_pgtable_ops *ops = smmu_domain->pgtbl_ops;

	if (!ops)
		return 0;

	return ops->unmap(ops, iova, size, gather);
}

static int virt_arm_smmu_map(struct iommu_domain *domain, unsigned long iova,
			phys_addr_t paddr, size_t size, int prot)
{
	struct io_pgtable_ops *ops = to_smmu_domain(domain)->pgtbl_ops;

	if (!ops)
		return -ENODEV;

	return ops->map(ops, iova, paddr, size, prot);
}

static int virt_arm_smmu_write_strtab_ent(struct virt_arm_smmu_master *master,
					 struct arm_smmu_ste_cfg *ste_cfg)
{
	u64 val = le64_to_cpu(ste_cfg->ste[0]);
	struct virt_arm_smmu_device *smmu = NULL;
	struct arm_smmu_s1_cfg *s1_cfg = NULL;
	struct virt_arm_smmu_domain *smmu_domain = NULL;

	if (master) {
		smmu_domain = master->domain;
		smmu = master->smmu;
	}

	s1_cfg = &smmu_domain->s1_cfg;
	val = STRTAB_STE_0_V;

	if (s1_cfg) {
		ste_cfg->ste[1] = cpu_to_le64(
			 FIELD_PREP(STRTAB_STE_1_S1CIR, STRTAB_STE_1_S1C_CACHE_WBRA) |
			 FIELD_PREP(STRTAB_STE_1_S1COR, STRTAB_STE_1_S1C_CACHE_WBRA) |
			 FIELD_PREP(STRTAB_STE_1_S1CSH, ARM_SMMU_SH_ISH) |
			 FIELD_PREP(STRTAB_STE_1_STRW, STRTAB_STE_1_STRW_NSEL1));

		ste_cfg->ste[1] |= cpu_to_le64(STRTAB_STE_1_S1STALLD);

		/* S1 Translate S2 Bypass only supported*/
		val |= (s1_cfg->cdptr_dma & STRTAB_STE_0_S1CTXPTR_MASK) |
			FIELD_PREP(STRTAB_STE_0_CFG, STRTAB_STE_0_CFG_S1_TRANS);
	}
	WRITE_ONCE(ste_cfg->ste[0], cpu_to_le64(val));
	if (qcom_scm_paravirt_smmu_attach(ste_cfg->sid, 0, ste_cfg->stedma,
		(STRTAB_STE_DWORDS << 3), s1_cfg->cdptr_dma, (CTXDESC_CD_DWORDS << 3))) {
		pr_err("SCM call failed to attach for SID:0x%lx\n", ste_cfg->sid);
		return -EINVAL;
	}
	return 0;
}

static int virt_arm_smmu_install_ste_for_dev(struct virt_arm_smmu_master *master)
{
	struct virt_arm_smmu_device *smmu = master->smmu;
	int i, j;
	struct arm_smmu_ste_cfg *iter;

	struct arm_smmu_ste_cfg *ste_cfg = kzalloc(sizeof(*ste_cfg) * master->num_sids,
						GFP_KERNEL);
	if (!ste_cfg)
		return -ENOMEM;

	iter = ste_cfg;

	for (i = 0; i < master->num_sids; i++) {
		iter->ste = dmam_alloc_coherent(smmu->dev,
				(STRTAB_STE_DWORDS << 3), &iter->stedma,
				GFP_KERNEL | __GFP_ZERO);
		if (!iter->ste) {
			dev_err(smmu->dev, "failed to allocate memory for stream table entry\n");
			goto free_mem;
		}
		iter->sid = master->sids[i];
		if (virt_arm_smmu_write_strtab_ent(master, iter))
			goto free_mem;
		iter++;
	}
	master->ste_cfg = ste_cfg;
	return 0;
free_mem:
	iter = ste_cfg;
	for (j = 0; j <= i; j++) {
		if (iter->ste)
			dmam_free_coherent(smmu->dev, (STRTAB_STE_DWORDS << 3),
				iter->ste, iter->stedma);
		iter++;
	}
	kfree(ste_cfg);
	return -ENOMEM;
}

static int virt_arm_smmu_attach_dev(struct iommu_domain *domain, struct device *dev)
{
	int ret = 0;
	unsigned long flags;
	struct iommu_fwspec *fwspec = dev_iommu_fwspec_get(dev);
	struct virt_arm_smmu_device *smmu;
	struct virt_arm_smmu_domain *smmu_domain = to_smmu_domain(domain);
	struct virt_arm_smmu_master *master;

	if (!fwspec)
		return -ENOENT;

	master = fwspec->iommu_priv;
	smmu = master->smmu;
	mutex_lock(&smmu_domain->init_mutex);

	if (!smmu_domain->smmu) {
		smmu_domain->smmu = smmu;
		ret = arm_smmu_domain_finalise(domain);
		if (ret) {
			smmu_domain->smmu = NULL;
			goto out_unlock;
		}
	} else if (smmu_domain->smmu != smmu) {
		dev_err(dev,
			"cannot attach to SMMU %s (upstream of %s)\n",
			dev_name(smmu_domain->smmu->dev),
			dev_name(smmu->dev));
		ret = -ENXIO;
		goto out_unlock;
	}

	master->domain = smmu_domain;

	virt_arm_smmu_write_ctx_desc(smmu, &smmu_domain->s1_cfg);
	ret = virt_arm_smmu_install_ste_for_dev(master);
	if (ret)
		goto out_unlock;

	spin_lock_irqsave(&smmu_domain->devices_lock, flags);
	list_add(&master->domain_head, &smmu_domain->devices);
	spin_unlock_irqrestore(&smmu_domain->devices_lock, flags);

out_unlock:
	mutex_unlock(&smmu_domain->init_mutex);
	return ret;
}

static void virt_arm_smmu_domain_free(struct iommu_domain *domain)
{
	struct virt_arm_smmu_domain *smmu_domain = to_smmu_domain(domain);
	struct virt_arm_smmu_device *smmu = smmu_domain->smmu;
	struct arm_smmu_s1_cfg *cfg = &smmu_domain->s1_cfg;

	iommu_put_dma_cookie(domain);
	free_io_pgtable_ops(smmu_domain->pgtbl_ops);

	if (cfg->cdptr) {
		dmam_free_coherent(smmu_domain->smmu->dev,
				   CTXDESC_CD_DWORDS << 3,
				   cfg->cdptr,
				   cfg->cdptr_dma);

		arm_smmu_bitmap_free(smmu->asid_map, cfg->cd.asid);
	}
	kfree(smmu_domain);
}

static struct iommu_domain *virt_arm_smmu_domain_alloc(unsigned int type)
{
	struct virt_arm_smmu_domain *smmu_domain;

	if (type != IOMMU_DOMAIN_UNMANAGED &&
	    type != IOMMU_DOMAIN_DMA &&
	    type != IOMMU_DOMAIN_IDENTITY)
		return NULL;

	smmu_domain = kzalloc(sizeof(*smmu_domain), GFP_KERNEL);
	if (!smmu_domain)
		return NULL;

	if (type == IOMMU_DOMAIN_DMA &&
	    iommu_get_dma_cookie(&smmu_domain->domain)) {
		kfree(smmu_domain);
		return NULL;
	}

	mutex_init(&smmu_domain->init_mutex);
	INIT_LIST_HEAD(&smmu_domain->devices);
	spin_lock_init(&smmu_domain->devices_lock);

	return &smmu_domain->domain;
}

static void virt_arm_smmu_tlb_inv_page_nosync(struct iommu_iotlb_gather *gather,
					 unsigned long iova, size_t granule,
					 void *cookie)
{
	struct virt_arm_smmu_domain *smmu_domain = cookie;
	struct iommu_domain *domain = &smmu_domain->domain;

	iommu_iotlb_gather_add_page(domain, gather, iova, granule);
}

static void virt_arm_smmu_tlb_inv_walk(unsigned long iova, size_t size,
				  size_t granule, void *cookie)
{
	virt_arm_smmu_tlb_inv_sync(iova, size, granule, false, cookie);
}

static void virt_arm_smmu_tlb_inv_leaf(unsigned long iova, size_t size,
				  size_t granule, void *cookie)
{
	virt_arm_smmu_tlb_inv_sync(iova, size, granule, true, cookie);
}

static const struct iommu_flush_ops virt_arm_smmu_flush_ops = {
	.tlb_flush_all	= virt_arm_smmu_tlb_inv_context,
	.tlb_flush_walk = virt_arm_smmu_tlb_inv_walk,
	.tlb_flush_leaf = virt_arm_smmu_tlb_inv_leaf,
	.tlb_add_page	= virt_arm_smmu_tlb_inv_page_nosync,
};

static bool virt_arm_smmu_capable(enum iommu_cap cap)
{
	switch (cap) {
	case IOMMU_CAP_CACHE_COHERENCY:
		return true;
	case IOMMU_CAP_NOEXEC:
		return true;
	default:
		return false;
	}
}
#define MAX_MAP_SG_BATCH_SIZE (SZ_4M)
static size_t virt_arm_smmu_map_sg(struct iommu_domain *domain, unsigned long iova,
			   struct scatterlist *sg, unsigned int nents, int prot)
{
	int ret;
	size_t size, batch_size, size_to_unmap = 0;
	struct virt_arm_smmu_domain *smmu_domain = to_smmu_domain(domain);
	struct io_pgtable_ops *ops;
	struct msm_io_pgtable_info *pgtbl_info = &smmu_domain->pgtbl_info;
	unsigned int idx_start, idx_end;
	struct scatterlist *sg_start, *sg_end;
	unsigned long __saved_iova_start;
	unsigned int ias;
	unsigned long mask;

	if (!pgtbl_info->map_sg)
		return 0;

	ops = smmu_domain->pgtbl_ops;
	if (IS_ERR_OR_NULL(ops))
		return 0;

	ias = pgtbl_info->pgtbl_cfg.ias;
	mask = (1UL << ias) - 1;
	iova = iova & mask;

	__saved_iova_start = iova;
	idx_start = idx_end = 0;
	sg_start = sg_end = sg;
	while (idx_end < nents) {
		batch_size = sg_end->length;
		sg_end = sg_next(sg_end);
		idx_end++;
		while ((idx_end < nents) &&
		       (batch_size + sg_end->length < MAX_MAP_SG_BATCH_SIZE)) {

			batch_size += sg_end->length;
			sg_end = sg_next(sg_end);
			idx_end++;
		}

		ret = pgtbl_info->map_sg(ops, iova, sg_start,
					 idx_end - idx_start, prot, &size);
		pgtbl_info->pgtbl_cfg.tlb->tlb_flush_all(smmu_domain);

		if (ret == -ENOMEM) {
			/* unmap any partially mapped iova */
			if (size)
				virt_arm_smmu_unmap(domain, iova, size, NULL);
			ret = pgtbl_info->map_sg(ops, iova, sg_start,
						 idx_end - idx_start, prot,
						 &size);
			pgtbl_info->pgtbl_cfg.tlb->tlb_flush_all(smmu_domain);
		}

		/* Returns -ve val on error */
		if (ret < 0) {
			size_to_unmap = iova + size - __saved_iova_start;
			goto out;
		}

		iova += batch_size;
		idx_start = idx_end;
		sg_start = sg_end;
		size = 0;
	}

out:
	if (size_to_unmap) {
		virt_arm_smmu_unmap(domain, __saved_iova_start, size_to_unmap, NULL);
		iova = __saved_iova_start;
	}
	return iova - __saved_iova_start;
}

static struct msm_iommu_ops  virt_arm_smmu_ops = {
	.map_sg			= virt_arm_smmu_map_sg,
	.iommu_ops = {
		.capable		= virt_arm_smmu_capable,
		.domain_alloc		= virt_arm_smmu_domain_alloc,
		.domain_free		= virt_arm_smmu_domain_free,
		.attach_dev		= virt_arm_smmu_attach_dev,
		.detach_dev		= virt_arm_smmu_detach_dev,
		.map			= virt_arm_smmu_map,
		.unmap			= virt_arm_smmu_unmap,
		.flush_iotlb_all	= virt_arm_smmu_flush_iotlb_all,
		.iotlb_sync		= virt_arm_smmu_iotlb_sync,
		.iova_to_phys		= virt_arm_smmu_iova_to_phys,
		.add_device		= virt_arm_smmu_add_device,
		.remove_device		= virt_arm_smmu_remove_device,
		.device_group		= virt_arm_smmu_device_group,
		.domain_get_attr	= virt_arm_smmu_domain_get_attr,
		.domain_set_attr	= virt_arm_smmu_domain_set_attr,
		.of_xlate		= virt_arm_smmu_of_xlate,
		.get_resv_regions	= virt_arm_smmu_get_resv_regions,
		.put_resv_regions	= virt_arm_smmu_put_resv_regions,
		.pgsize_bitmap		= -1UL, /* Restricted during device attach */
	}
};

static int virt_arm_smmu_set_bus_ops(struct iommu_ops *ops)
{
	int err;

#ifdef CONFIG_PCI
	if (pci_bus_type.iommu_ops != ops) {
		err = bus_set_iommu(&pci_bus_type, ops);
		if (err)
			return err;
	}
#endif
	return 0;
}

static int virt_arm_smmu_device_probe(struct platform_device *pdev)
{
	int ret;
	struct virt_arm_smmu_device *smmu;
	struct device *dev = &pdev->dev;

	smmu = devm_kzalloc(dev, sizeof(*smmu), GFP_KERNEL);
	if (!smmu)
		return -ENOMEM;

	smmu->dev = dev;
	smmu->ias = 48;
	smmu->oas = 48;
	smmu->asid_bits = 16;
	smmu->vmid_bits = 16;
	smmu->pgsize_bitmap |= SZ_4K | SZ_2M | SZ_1G;
	virt_arm_smmu_ops.iommu_ops.pgsize_bitmap = smmu->pgsize_bitmap;

	if (dma_set_mask_and_coherent(smmu->dev, DMA_BIT_MASK(smmu->oas)))
		dev_warn(smmu->dev, "failed to set DMA mask for table walker\n");

	/* Record our private device structure */
	platform_set_drvdata(pdev, smmu);

	ret = iommu_device_sysfs_add(&smmu->iommu, dev, NULL,
				     "virt-smmuv3");
	if (ret)
		return ret;

	iommu_set_default_translated(false);
	iommu_device_set_ops(&smmu->iommu, &virt_arm_smmu_ops.iommu_ops);
	iommu_device_set_fwnode(&smmu->iommu, dev->fwnode);

	ret = iommu_device_register(&smmu->iommu);
	if (ret) {
		dev_err(dev, "Failed to register iommu\n");
		return ret;
	}

	return virt_arm_smmu_set_bus_ops(&virt_arm_smmu_ops.iommu_ops);
}

static int virt_arm_smmu_device_remove(struct platform_device *pdev)
{
	struct virt_arm_smmu_device *smmu = platform_get_drvdata(pdev);

	virt_arm_smmu_set_bus_ops(NULL);
	iommu_device_unregister(&smmu->iommu);
	iommu_device_sysfs_remove(&smmu->iommu);

	return 0;
}

static const struct of_device_id virt_arm_smmu_of_match[] = {
	{ .compatible = "arm,virt-smmu-v3", },
	{ },
};
MODULE_DEVICE_TABLE(of, virt_arm_smmu_of_match);

static struct platform_driver virt_arm_smmu_driver = {
	.driver	= {
		.name			= "arm,virt-smmu-v3",
		.of_match_table		= of_match_ptr(virt_arm_smmu_of_match),
		.suppress_bind_attrs    = true,
	},
	.probe	= virt_arm_smmu_device_probe,
	.remove	= virt_arm_smmu_device_remove,
};

static int __init virt_arm_smmu_init(void)
{
	return platform_driver_register(&virt_arm_smmu_driver);
}
arch_initcall(virt_arm_smmu_init);

static void __exit virt_arm_smmu_exit(void)
{
	platform_driver_unregister(&virt_arm_smmu_driver);
}
module_exit(virt_arm_smmu_exit);

MODULE_DESCRIPTION("Paravirtualized-IOMMU API for ARM architected SMMU-v3 implementations");
MODULE_LICENSE("GPL v2");
