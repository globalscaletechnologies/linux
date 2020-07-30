// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2018 Marvell
 *
 * Authors:
 *   Evan Wang <xswang@marvell.com>
 *   Miquèl Raynal <miquel.raynal@bootlin.com>
 *
 * Structure inspired from phy-mvebu-cp110-comphy.c written by Antoine Tenart.
 * SMC call initial support done by Grzegorz Jaszczyk.
 */

#include <linux/arm-smccc.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/phy.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>

#define MVEBU_A3700_COMPHY_LANES		3
#define MVEBU_A3700_COMPHY_PORTS		2

/* COMPHY Fast SMC function identifiers */
#define COMPHY_SIP_POWER_ON			0x82000001
#define COMPHY_SIP_POWER_OFF			0x82000002
#define COMPHY_SIP_PLL_LOCK			0x82000003
#define COMPHY_FW_NOT_SUPPORTED			(-1)

#define COMPHY_FW_MODE_SATA			0x1
#define COMPHY_FW_MODE_SGMII			0x2
#define COMPHY_FW_MODE_HS_SGMII			0x3
#define COMPHY_FW_MODE_USB3H			0x4
#define COMPHY_FW_MODE_USB3D			0x5
#define COMPHY_FW_MODE_PCIE			0x6
#define COMPHY_FW_MODE_RXAUI			0x7
#define COMPHY_FW_MODE_XFI			0x8
#define COMPHY_FW_MODE_SFI			0x9
#define COMPHY_FW_MODE_USB3			0xa

#define COMPHY_FW_SPEED_1_25G			0 /* SGMII 1G */
#define COMPHY_FW_SPEED_2_5G			1
#define COMPHY_FW_SPEED_3_125G			2 /* SGMII 2.5G */
#define COMPHY_FW_SPEED_5G			3
#define COMPHY_FW_SPEED_5_15625G		4 /* XFI 5G */
#define COMPHY_FW_SPEED_6G			5
#define COMPHY_FW_SPEED_10_3125G		6 /* XFI 10G */
#define COMPHY_FW_SPEED_MAX			0x3F

#define COMPHY_FW_MODE(mode)			((mode) << 12)
#define COMPHY_FW_NET(mode, idx, speed)		(COMPHY_FW_MODE(mode) | \
						 ((idx) << 8) |	\
						 ((speed) << 2))
#define COMPHY_FW_PCIE(mode, idx, speed, width)	(COMPHY_FW_NET(mode, idx, speed) | \
						 ((width) << 18))

#define PLL_SET_DELAY_US			600
#define A3700_COMPHY_PLL_LOCK_TIMEOUT		1000
#define REG_16_BIT_MASK				0xFFFF

#define COMPHY_SELECTOR_PHY_REG_OFFSET		0xFC
/* bit0: 0: Lane0 is GBE0; 1: Lane1 is PCIE */
#define COMPHY_SELECTOR_PCIE_GBE0_SEL_BIT	BIT(0)
/* bit4: 0: Lane1 is GBE1; 1: Lane1 is USB3 */
#define COMPHY_SELECTOR_USB3_GBE1_SEL_BIT	BIT(4)
/* bit8: 0: Lane1 is USB, Lane2 is SATA; 1: Lane2 is USB3 */
#define COMPHY_SELECTOR_USB3_PHY_SEL_BIT	BIT(8)

/* SATA PHY register offset */
#define SATAPHY_LANE2_REG_BASE_OFFSET		0x200

/* USB3 PHY offset compared to SATA PHY */
#define USB3PHY_LANE2_REG_BASE_OFFSET		0x200

/* Comphy lane2 indirect access register offset */
#define COMPHY_LANE2_INDIR_ADDR_OFFSET		0x0
#define COMPHY_LANE2_INDIR_DATA_OFFSET		0x4

/* PHY shift to get related register address */
enum {
	PCIE = 1,
	USB3,
};

#define PCIEPHY_SHFT				2
#define USB3PHY_SHFT				2
#define PHY_SHFT(unit)				((unit == PCIE) ? PCIEPHY_SHFT : USB3PHY_SHFT)

/* PHY register */
#define COMPHY_POWER_PLL_CTRL			0x01
#define PWR_PLL_CTRL_ADDR(unit)			(COMPHY_POWER_PLL_CTRL * PHY_SHFT(unit))
#define PU_IVREF_BIT				BIT(15)
#define PU_PLL_BIT				BIT(14)
#define PU_RX_BIT				BIT(13)
#define PU_TX_BIT				BIT(12)
#define PU_TX_INTP_BIT				BIT(11)
#define PU_DFE_BIT				BIT(10)
#define RESET_DTL_RX_BIT			BIT(9)
#define PLL_LOCK_BIT				BIT(8)
#define REF_FREF_SEL_OFFSET			0
#define REF_FREF_SEL_MASK			(0x1F << REF_FREF_SEL_OFFSET)
#define REF_CLOCK_SPEED_25M			(0x1 << REF_FREF_SEL_OFFSET)
#define REF_CLOCK_SPEED_30M			(0x2 << REF_FREF_SEL_OFFSET)
#define PCIE_REF_CLOCK_SPEED_25M		REF_CLOCK_SPEED_30M
#define USB3_REF_CLOCK_SPEED_25M		REF_CLOCK_SPEED_30M
#define REF_CLOCK_SPEED_40M			(0x3 << REF_FREF_SEL_OFFSET)
#define COMPHY_MODE_OFFSET			5
#define COMPHY_MODE_MASK			(7 << COMPHY_MODE_OFFSET)
#define COMPHY_MODE_SATA			(0x0 << COMPHY_MODE_OFFSET)
#define COMPHY_MODE_PCIE			(0x3 << COMPHY_MODE_OFFSET)
#define COMPHY_MODE_SGMII			(0x4 << COMPHY_MODE_OFFSET)
#define COMPHY_MODE_USB3			(0x5 << COMPHY_MODE_OFFSET)

#define COMPHY_KVCO_CAL_CTRL			0x02
#define KVCO_CAL_CTRL_ADDR(unit)		(COMPHY_KVCO_CAL_CTRL * PHY_SHFT(unit))
#define USE_MAX_PLL_RATE_BIT			BIT(12)
#define SPEED_PLL_OFFSET			2
#define SPEED_PLL_MASK				(0x3F << SPEED_PLL_OFFSET)
#define SPEED_PLL_VALUE_16			(0x10 << SPEED_PLL_OFFSET)

#define COMPHY_RESERVED_REG			0x0e
#define PHYCTRL_FRM_PIN_BIT			BIT(13)

#define COMPHY_LOOPBACK_REG0			0x23
#define DIG_LB_EN_ADDR(unit)			(COMPHY_LOOPBACK_REG0 * PHY_SHFT(unit))
#define SEL_DATA_WIDTH_OFFSET			10
#define SEL_DATA_WIDTH_MASK			(0x3 << SEL_DATA_WIDTH_OFFSET)
#define DATA_WIDTH_10BIT			(0x0 << SEL_DATA_WIDTH_OFFSET)
#define DATA_WIDTH_20BIT			(0x1 << SEL_DATA_WIDTH_OFFSET)
#define DATA_WIDTH_40BIT			(0x2 << SEL_DATA_WIDTH_OFFSET)
#define PLL_READY_TX_BIT			BIT(4)

#define COMPHY_SYNC_PATTERN_REG			0x24
#define SYNC_PATTERN_REG_ADDR(unit)		(COMPHY_SYNC_PATTERN_REG * PHY_SHFT(unit))
#define TXD_INVERT_BIT				BIT(10)
#define RXD_INVERT_BIT				BIT(11)

#define COMPHY_SYNC_MASK_GEN_REG		0x25
#define PHY_GEN_MAX_OFFSET			10
#define PHY_GEN_MAX_MASK			(3 << PHY_GEN_MAX_OFFSET)
#define PHY_GEN_USB3_5G				(1 << PHY_GEN_MAX_OFFSET)

#define COMPHY_ISOLATION_CTRL_REG		0x26
#define ISOLATION_CTRL_REG_ADDR(unit)		(COMPHY_ISOLATION_CTRL_REG * PHY_SHFT(unit))
#define PHY_ISOLATE_MODE			BIT(15)

#define COMPHY_MISC_REG0_ADDR			0x4F
#define MISC_REG0_ADDR(unit)			(COMPHY_MISC_REG0_ADDR * PHY_SHFT(unit))
#define CLK100M_125M_EN				BIT(4)
#define CLK500M_EN				BIT(7)
#define PHY_REF_CLK_SEL				BIT(10)
#define MISC_REG0_DEFAULT_VALUE			0xA00D

#define COMPHY_REG_GEN2_SETTINGS_2		0x3e
#define GEN2_SETTING_2_ADDR(unit)		(COMPHY_REG_GEN2_SETTINGS_2 * PHY_SHFT(unit))
#define G2_TX_SSC_AMP_VALUE_20			BIT(14)
#define G2_TX_SSC_AMP_OFF			9
#define G2_TX_SSC_AMP_LEN			7
#define G2_TX_SSC_AMP_MASK			(((1 << G2_TX_SSC_AMP_LEN) - 1) << G2_TX_SSC_AMP_OFF)

#define COMPHY_REG_GEN2_SETTINGS_3		0x3f
#define GEN2_SETTING_3_ADDR(unit)		(COMPHY_REG_GEN2_SETTINGS_3 * PHY_SHFT(unit))
#define G3_TX_SSC_AMP_OFF			9
#define G3_TX_SSC_AMP_LEN			7
#define G3_TX_SSC_AMP_MASK			(((1 << G2_TX_SSC_AMP_LEN) - 1) << G2_TX_SSC_AMP_OFF)
#define G3_VREG_RXTX_MAS_ISET_OFF		7
#define G3_VREG_RXTX_MAS_ISET_60U		(0 << G3_VREG_RXTX_MAS_ISET_OFF)
#define G3_VREG_RXTX_MAS_ISET_80U		(1 << G3_VREG_RXTX_MAS_ISET_OFF)
#define G3_VREG_RXTX_MAS_ISET_100U		(2 << G3_VREG_RXTX_MAS_ISET_OFF)
#define G3_VREG_RXTX_MAS_ISET_120U		(3 << G3_VREG_RXTX_MAS_ISET_OFF)
#define G3_VREG_RXTX_MAS_ISET_MASK		(BIT(7) | BIT(8))
#define RSVD_PH03FH_6_0_OFF			0
#define RSVD_PH03FH_6_0_LEN			7
#define RSVD_PH03FH_6_0_MASK			(((1 << RSVD_PH03FH_6_0_LEN) - 1) << RSVD_PH03FH_6_0_OFF)

#define COMPHY_REG_UNIT_CTRL_ADDR		0x48
#define UNIT_CTRL_ADDR(unit)			(COMPHY_REG_UNIT_CTRL_ADDR * PHY_SHFT(unit))
#define IDLE_SYNC_EN				BIT(12)
#define UNIT_CTRL_DEFAULT_VALUE			0x60

#define COMPHY_MISC_REG1_ADDR			0x73
#define MISC_REG1_ADDR(unit)			(COMPHY_MISC_REG1_ADDR * PHY_SHFT(unit))
#define SEL_BITS_PCIE_FORCE			BIT(15)

#define COMPHY_REG_GEN3_SETTINGS_3		0x112
#define COMPHY_GEN_FFE_CAP_SEL_MASK		0xF
#define COMPHY_GEN_FFE_CAP_SEL_VALUE		0xF

#define COMPHY_REG_LANE_CFG0_ADDR		0x180
#define LANE_CFG0_ADDR(unit)			(COMPHY_REG_LANE_CFG0_ADDR * PHY_SHFT(unit))
#define PRD_TXDEEMPH0_MASK			BIT(0)
#define PRD_TXMARGIN_MASK			(BIT(1) | BIT(2) | BIT(3))
#define PRD_TXSWING_MASK			BIT(4)
#define CFG_TX_ALIGN_POS_MASK			(BIT(5) | BIT(6) | BIT(7) | BIT(8))

#define COMPHY_REG_LANE_CFG1_ADDR		0x181
#define LANE_CFG1_ADDR(unit)			(COMPHY_REG_LANE_CFG1_ADDR * PHY_SHFT(unit))
#define PRD_TXDEEMPH1_MASK			BIT(15)
#define USE_MAX_PLL_RATE_EN			BIT(9)
#define TX_DET_RX_MODE				BIT(6)
#define GEN2_TX_DATA_DLY_MASK			(BIT(3) | BIT(4))
#define GEN2_TX_DATA_DLY_DEFT			(2 << 3)
#define TX_ELEC_IDLE_MODE_EN			BIT(0)

#define COMPHY_REG_LANE_STATUS1_ADDR		0x183
#define LANE_STATUS1_ADDR(unit)			(COMPHY_REG_LANE_STATUS1_ADDR * PHY_SHFT(unit))
#define TXDCLK_PCLK_EN					BIT(0)

#define COMPHY_REG_LANE_CFG4_ADDR		0x188
#define LANE_CFG4_ADDR(unit)			(COMPHY_REG_LANE_CFG4_ADDR * PHY_SHFT(unit))
#define SPREAD_SPECTRUM_CLK_EN			BIT(7)

#define COMPHY_REG_GLOB_PHY_CTRL0_ADDR		0x1C1
#define GLOB_PHY_CTRL0_ADDR(unit)		(COMPHY_REG_GLOB_PHY_CTRL0_ADDR * PHY_SHFT(unit))
#define SOFT_RESET				BIT(0)
#define MODE_REFDIV				0x30
#define MODE_CORE_CLK_FREQ_SEL			BIT(9)
#define MODE_PIPE_WIDTH_32			BIT(3)
#define MODE_REFDIV_OFFSET			4
#define MODE_REFDIV_LEN				2
#define MODE_REFDIV_MASK			(0x3 << MODE_REFDIV_OFFSET)
#define MODE_REFDIV_BY_4			(0x2 << MODE_REFDIV_OFFSET)

#define COMPHY_REG_TEST_MODE_CTRL_ADDR		0x1C2
#define TEST_MODE_CTRL_ADDR(unit)		(COMPHY_REG_TEST_MODE_CTRL_ADDR * PHY_SHFT(unit))
#define MODE_MARGIN_OVERRIDE			BIT(2)

#define COMPHY_REG_GLOB_CLK_SRC_LO_ADDR		0x1C3
#define GLOB_CLK_SRC_LO_ADDR(unit)		(COMPHY_REG_GLOB_CLK_SRC_LO_ADDR * PHY_SHFT(unit))
#define MODE_CLK_SRC				BIT(0)
#define BUNDLE_PERIOD_SEL			BIT(1)
#define BUNDLE_PERIOD_SCALE			(BIT(2) | BIT(3))
#define BUNDLE_SAMPLE_CTRL			BIT(4)
#define PLL_READY_DLY				(BIT(5) | BIT(6) | BIT(7))
#define CFG_SEL_20B				BIT(15)

#define COMPHY_REG_PWR_MGM_TIM1_ADDR		0x1D0
#define PWR_MGM_TIM1_ADDR(unit)			(COMPHY_REG_PWR_MGM_TIM1_ADDR * PHY_SHFT(unit))
#define CFG_PM_OSCCLK_WAIT_OFF			12
#define CFG_PM_OSCCLK_WAIT_LEN			4
#define CFG_PM_OSCCLK_WAIT_MASK			(((1 << CFG_PM_OSCCLK_WAIT_LEN) - 1) << CFG_PM_OSCCLK_WAIT_OFF)
#define CFG_PM_RXDEN_WAIT_OFF			8
#define CFG_PM_RXDEN_WAIT_LEN			4
#define CFG_PM_RXDEN_WAIT_MASK			(((1 << CFG_PM_RXDEN_WAIT_LEN) - 1) << CFG_PM_RXDEN_WAIT_OFF)
#define CFG_PM_RXDEN_WAIT_1_UNIT		(1 << CFG_PM_RXDEN_WAIT_OFF)
#define CFG_PM_RXDLOZ_WAIT_OFF			0
#define CFG_PM_RXDLOZ_WAIT_LEN			8
#define CFG_PM_RXDLOZ_WAIT_MASK			(((1 << CFG_PM_RXDLOZ_WAIT_LEN) - 1) << CFG_PM_RXDLOZ_WAIT_OFF)
#define CFG_PM_RXDLOZ_WAIT_7_UNIT		(7 << CFG_PM_RXDLOZ_WAIT_OFF)
#define CFG_PM_RXDLOZ_WAIT_12_UNIT		(0xC << CFG_PM_RXDLOZ_WAIT_OFF)

/* SGMII */
#define COMPHY_PHY_CFG1_OFFSET(lane)		((1 - (lane)) * 0x28)
#define PIN_PU_IVEREF_BIT			BIT(1)
#define PIN_RESET_CORE_BIT			BIT(11)
#define PIN_RESET_COMPHY_BIT			BIT(12)
#define PIN_PU_PLL_BIT				BIT(16)
#define PIN_PU_RX_BIT				BIT(17)
#define PIN_PU_TX_BIT				BIT(18)
#define PIN_TX_IDLE_BIT				BIT(19)
#define GEN_RX_SEL_OFFSET			22
#define GEN_RX_SEL_MASK				(0xF << GEN_RX_SEL_OFFSET)
#define GEN_TX_SEL_OFFSET			26
#define GEN_TX_SEL_MASK				(0xF << GEN_TX_SEL_OFFSET)
#define PHY_RX_INIT_BIT				BIT(30)
#define SD_SPEED_1_25_G				0x6
#define SD_SPEED_2_5_G				0x8

/* COMPHY status reg:
 * lane0: PCIe/GbE0 PHY Status 1
 * lane1: USB3/GbE1 PHY Status 1
 */
#define COMPHY_PHY_STATUS_OFFSET(lane)		(0x18 + (1 - (lane)) * 0x28)
#define PHY_RX_INIT_DONE_BIT			BIT(0)
#define PHY_PLL_READY_RX_BIT			BIT(2)
#define PHY_PLL_READY_TX_BIT			BIT(3)

#define SGMIIPHY_ADDR(lane, off, base)		(((off & 0x00007FF) * 2) + base)

/* Polarity invert macro */
#define COMPHY_POLARITY_NO_INVERT	0
#define COMPHY_POLARITY_TXD_INVERT	1
#define COMPHY_POLARITY_RXD_INVERT	2
#define COMPHY_POLARITY_ALL_INVERT	(COMPHY_POLARITY_TXD_INVERT | COMPHY_POLARITY_RXD_INVERT)

enum {
	COMPHY_LANE0 = 0,
	COMPHY_LANE1,
	COMPHY_LANE2,
	COMPHY_LANE_MAX,
};

enum reg_width_type {
	REG_16BIT = 0,
	REG_32BIT,
};

struct mvebu_a3700_comphy_conf {
	unsigned int lane;
	enum phy_mode mode;
	int submode;
	unsigned int port;
	u32 fw_mode;
};

#define MVEBU_A3700_COMPHY_CONF(_lane, _mode, _smode, _port, _fw)	\
	{								\
		.lane = _lane,						\
		.mode = _mode,						\
		.submode = _smode,					\
		.port = _port,						\
		.fw_mode = _fw,						\
	}

#define MVEBU_A3700_COMPHY_CONF_GEN(_lane, _mode, _port, _fw) \
	MVEBU_A3700_COMPHY_CONF(_lane, _mode, PHY_INTERFACE_MODE_NA, _port, _fw)

#define MVEBU_A3700_COMPHY_CONF_ETH(_lane, _smode, _port, _fw) \
	MVEBU_A3700_COMPHY_CONF(_lane, PHY_MODE_ETHERNET, _smode, _port, _fw)

static const struct mvebu_a3700_comphy_conf mvebu_a3700_comphy_modes[] = {
	/* lane 0 */
	MVEBU_A3700_COMPHY_CONF_GEN(0, PHY_MODE_USB_HOST_SS, 0,
				    COMPHY_FW_MODE_USB3H),
	MVEBU_A3700_COMPHY_CONF_ETH(0, PHY_INTERFACE_MODE_SGMII, 1,
				    COMPHY_FW_MODE_SGMII),
	MVEBU_A3700_COMPHY_CONF_ETH(0, PHY_INTERFACE_MODE_2500BASEX, 1,
				    COMPHY_FW_MODE_HS_SGMII),
	/* lane 1 */
	MVEBU_A3700_COMPHY_CONF_GEN(1, PHY_MODE_PCIE, 0,
				    COMPHY_FW_MODE_PCIE),
	MVEBU_A3700_COMPHY_CONF_ETH(1, PHY_INTERFACE_MODE_SGMII, 0,
				    COMPHY_FW_MODE_SGMII),
	MVEBU_A3700_COMPHY_CONF_ETH(1, PHY_INTERFACE_MODE_2500BASEX, 0,
				    COMPHY_FW_MODE_HS_SGMII),
	/* lane 2 */
	MVEBU_A3700_COMPHY_CONF_GEN(2, PHY_MODE_SATA, 0,
				    COMPHY_FW_MODE_SATA),
	MVEBU_A3700_COMPHY_CONF_GEN(2, PHY_MODE_USB_HOST_SS, 0,
				    COMPHY_FW_MODE_USB3H),
};

struct mvebu_comphy_priv {
	void __iomem *comphy_regs;
	void __iomem *pcie_gbe0_phy_regs;
	void __iomem *usb3_gbe1_phy_regs;
	void __iomem *sata_usb3_phy_regs;
};

struct mvebu_a3700_comphy_lane {
	struct mvebu_comphy_priv *priv;
	struct device *dev;
	unsigned int id;
	enum phy_mode mode;
	int submode;
	int port;
	int invert;
};

static int mvebu_a3700_comphy_smc(unsigned long function, unsigned long lane,
				  unsigned long mode)
{
	struct arm_smccc_res res;

	arm_smccc_smc(function, lane, mode, 0, 0, 0, 0, 0, &res);

	return res.a0;
}

static int mvebu_a3700_comphy_get_fw_mode(int lane, int port,
					  enum phy_mode mode,
					  int submode)
{
	int i, n = ARRAY_SIZE(mvebu_a3700_comphy_modes);

	/* Unused PHY mux value is 0x0 */
	if (mode == PHY_MODE_INVALID)
		return -EINVAL;

	for (i = 0; i < n; i++) {
		if (mvebu_a3700_comphy_modes[i].lane == lane &&
		    mvebu_a3700_comphy_modes[i].port == port &&
		    mvebu_a3700_comphy_modes[i].mode == mode &&
		    mvebu_a3700_comphy_modes[i].submode == submode)
			break;
	}

	if (i == n)
		return -EINVAL;

	return mvebu_a3700_comphy_modes[i].fw_mode;
}

static int mvebu_a3700_comphy_set_mode(struct phy *phy, enum phy_mode mode,
				       int submode)
{
	struct mvebu_a3700_comphy_lane *lane = phy_get_drvdata(phy);
	int fw_mode;

	if (submode == PHY_INTERFACE_MODE_1000BASEX)
		submode = PHY_INTERFACE_MODE_SGMII;

	fw_mode = mvebu_a3700_comphy_get_fw_mode(lane->id, lane->port, mode,
						 submode);
	if (fw_mode < 0) {
		dev_err(lane->dev, "invalid COMPHY mode\n");
		return fw_mode;
	}

	/* Just remember the mode, ->power_on() will do the real setup */
	lane->mode = mode;
	lane->submode = submode;

	return 0;
}

/* PHY selector configures with corresponding modes */
static void mvebu_a3700_comphy_set_phy_selector(struct phy *phy)
{
	u32 reg;
	struct mvebu_a3700_comphy_lane *lane = phy_get_drvdata(phy);
	struct mvebu_comphy_priv *priv = lane->priv;

	reg = readl(priv->comphy_regs + COMPHY_SELECTOR_PHY_REG_OFFSET);
	switch (lane->mode) {
	case PHY_MODE_SATA:
		/* SATA must be in Lane2 */
		if (lane->id == COMPHY_LANE2)
			reg &= ~COMPHY_SELECTOR_USB3_PHY_SEL_BIT;
		else
			dev_err(lane->dev, "COMPHY[%d] mode[%d] is invalid\n", lane->id,
					lane->mode);
		break;

	case PHY_MODE_ETHERNET:
		switch (lane->submode) {
			case PHY_INTERFACE_MODE_SGMII:
			case PHY_INTERFACE_MODE_2500BASEX:
				if (lane->id == COMPHY_LANE0)
					reg &= ~COMPHY_SELECTOR_USB3_GBE1_SEL_BIT;
				else if (lane->id == COMPHY_LANE1)
					reg &= ~COMPHY_SELECTOR_PCIE_GBE0_SEL_BIT;
				else
					dev_err(lane->dev, "COMPHY[%d] mode[%d] is invalid\n",
							lane->id, lane->mode);
				break;
			default:
				dev_err(lane->dev, "unsupported PHY submode (%d)\n",
						lane->submode);
				break;
		}
		break;

	case PHY_MODE_USB_HOST_SS:
	case PHY_MODE_USB_HOST:
	case PHY_MODE_USB_DEVICE:
		if (lane->id == COMPHY_LANE2)
			reg |= COMPHY_SELECTOR_USB3_PHY_SEL_BIT;
		else if (lane->id == COMPHY_LANE0)
			reg |= COMPHY_SELECTOR_USB3_GBE1_SEL_BIT;
		else
			dev_err(lane->dev, "COMPHY[%d] mode[%d] is invalid\n", lane->id,
					lane->mode);
		break;

	case (PHY_MODE_PCIE):
		/* PCIE must be in Lane1 */
		if (lane->id == COMPHY_LANE1)
			reg |= COMPHY_SELECTOR_PCIE_GBE0_SEL_BIT;
		else
			dev_err(lane->dev, "COMPHY[%d] mode[%d] is invalid\n", lane->id,
					lane->mode);
		break;

	default:
		dev_err(lane->dev, "COMPHY[%d] mode[%d] is invalid\n", lane->id,
				lane->mode);
		break;
	}

	writel(reg, priv->comphy_regs + COMPHY_SELECTOR_PHY_REG_OFFSET);
}

static inline void __maybe_unused reg_set(void __iomem *addr, u32 data, u32 mask)
{
	u32 reg_data;

	reg_data = readl(addr);
	reg_data &= ~mask;
	reg_data |= data;
	writel(reg_data, addr);
}

static inline void __maybe_unused reg_set16(void __iomem *addr, u16 data, u16 mask)
{
	u16 reg_data;

	reg_data = readw(addr);
	reg_data &= ~mask;
	reg_data |= data;
	writew(reg_data, addr);
}

static inline u32 __maybe_unused polling_with_timeout(void __iomem *addr,
						      u32 val,
						      u32 mask,
						      unsigned long usec_timout,
						      enum reg_width_type type)
{
	u32 data;

	do {
		udelay(1);
		if (type == REG_16BIT)
			data = readw(addr) & mask;
		else
			data = readl(addr) & mask;
	} while (data != val  && --usec_timout > 0);

	if (usec_timout == 0)
		return data;

	return 0;
}

/***************************************************************************************************
  * mvebu_comphy_reg_set_indirect
  * It is only used for SATA and USB3 on comphy lane2.
  * return: void
 ***************************************************************************************************/
static void mvebu_comphy_reg_set_indirect(void __iomem *addr, u32 reg_offset, u16 data, u16 mask, int mode)
{
	/*
	 * When Lane 2 PHY is for USB3, access the PHY registers
	 * through indirect Address and Data registers INDIR_ACC_PHY_ADDR (RD00E0178h [31:0]) and
	 * INDIR_ACC_PHY_DATA (RD00E017Ch [31:0]) within the SATA Host Controller registers, Lane 2
	 * base register offset is 0x200
	 */
	if (mode == PHY_MODE_INVALID)
		return;

	if (mode == PHY_MODE_SATA)
		writel(reg_offset, addr + COMPHY_LANE2_INDIR_ADDR_OFFSET);
	else
		writel(reg_offset + USB3PHY_LANE2_REG_BASE_OFFSET, addr + COMPHY_LANE2_INDIR_ADDR_OFFSET);

	reg_set(addr + COMPHY_LANE2_INDIR_DATA_OFFSET, data, mask);
}

/***************************************************************************************************
  * mvebu_comphy_usb3_reg_set_direct
  * It is only used USB3 direct access not on comphy lane2.
  * return: void
 ***************************************************************************************************/
static void mvebu_comphy_usb3_reg_set_direct(void __iomem *addr, u32 reg_offset, u16 data, u16 mask, int mode)
{
	reg_set16((reg_offset * PHY_SHFT(USB3) + addr), data, mask);
}

static int mvebu_a3700_comphy_sata_power_on(struct phy *phy)
{
	int ret = 0;
	u32 reg_offset, data = 0;
	struct mvebu_a3700_comphy_lane *lane = phy_get_drvdata(phy);
	struct mvebu_comphy_priv *priv = lane->priv;
	int mode = lane->mode;

	dev_dbg(lane->dev, "%s: Enter\n", __func__);

	/* Configure phy selector for SATA */
	mvebu_a3700_comphy_set_phy_selector(phy);

	/* Clear phy isolation mode to make it work in normal mode */
	mvebu_comphy_reg_set_indirect(priv->sata_usb3_phy_regs,
					COMPHY_ISOLATION_CTRL_REG + SATAPHY_LANE2_REG_BASE_OFFSET,
					0,
					PHY_ISOLATE_MODE,
					mode);

	/*
	 * 0. Check the Polarity invert bits
	 */
	if (lane->invert & COMPHY_POLARITY_TXD_INVERT)
		data |= TXD_INVERT_BIT;
	if (lane->invert & COMPHY_POLARITY_RXD_INVERT)
		data |= RXD_INVERT_BIT;

	reg_offset = COMPHY_SYNC_PATTERN_REG + SATAPHY_LANE2_REG_BASE_OFFSET;
	mvebu_comphy_reg_set_indirect(priv->sata_usb3_phy_regs,
					reg_offset,
					data,
					TXD_INVERT_BIT | RXD_INVERT_BIT,
					mode);

	/*
	 * 1. Select 40-bit data width width
	 */
	reg_offset = COMPHY_LOOPBACK_REG0 + SATAPHY_LANE2_REG_BASE_OFFSET;
	mvebu_comphy_reg_set_indirect(priv->sata_usb3_phy_regs,
					reg_offset,
					DATA_WIDTH_40BIT,
					SEL_DATA_WIDTH_MASK,
					mode);

	/*
	 * 2. Select reference clock(25M) and PHY mode (SATA)
	 */
	reg_offset = COMPHY_POWER_PLL_CTRL + SATAPHY_LANE2_REG_BASE_OFFSET;
	mvebu_comphy_reg_set_indirect(priv->sata_usb3_phy_regs,
					reg_offset,
					REF_CLOCK_SPEED_25M | COMPHY_MODE_SATA,
					REF_FREF_SEL_MASK | COMPHY_MODE_MASK,
					mode);

	/*
	 * 3. Use maximum PLL rate (no power save)
	 */
	reg_offset = COMPHY_KVCO_CAL_CTRL + SATAPHY_LANE2_REG_BASE_OFFSET;
	mvebu_comphy_reg_set_indirect(priv->sata_usb3_phy_regs,
					reg_offset,
					USE_MAX_PLL_RATE_BIT,
					USE_MAX_PLL_RATE_BIT,
					mode);

	/*
	 * 4. Reset reserved bit
	 */
	mvebu_comphy_reg_set_indirect(priv->sata_usb3_phy_regs,
					COMPHY_RESERVED_REG,
					0,
					PHYCTRL_FRM_PIN_BIT,
					mode);

	/*
	 * 5. Set vendor-specific configuration (It is done in sata driver)
	 */

	/* Wait for > 55 us to allow PLL be enabled */
	udelay(PLL_SET_DELAY_US);

	/* Polling status */
	writel(COMPHY_LOOPBACK_REG0 + SATAPHY_LANE2_REG_BASE_OFFSET,
	       priv->sata_usb3_phy_regs + COMPHY_LANE2_INDIR_ADDR_OFFSET);
	ret = polling_with_timeout(
					priv->sata_usb3_phy_regs + COMPHY_LANE2_INDIR_DATA_OFFSET,
					PLL_READY_TX_BIT,
					PLL_READY_TX_BIT,
					A3700_COMPHY_PLL_LOCK_TIMEOUT,
					REG_32BIT);

	dev_dbg(lane->dev, "%s: Exit\n", __func__);

	return ret;
}

static int mvebu_a3700_comphy_sata_power_off(struct phy *phy)
{
	struct mvebu_a3700_comphy_lane *lane = phy_get_drvdata(phy);
	struct mvebu_comphy_priv *priv = lane->priv;
	int mode = lane->mode;

	dev_dbg(lane->dev, "%s: Enter\n", __func__);

	/* Set phy isolation mode */
	mvebu_comphy_reg_set_indirect(priv->sata_usb3_phy_regs,
					COMPHY_ISOLATION_CTRL_REG + SATAPHY_LANE2_REG_BASE_OFFSET,
					PHY_ISOLATE_MODE,
					PHY_ISOLATE_MODE,
					mode);

	/* Power off PLL, Tx, Rx */
	mvebu_comphy_reg_set_indirect(priv->sata_usb3_phy_regs,
					COMPHY_POWER_PLL_CTRL + SATAPHY_LANE2_REG_BASE_OFFSET,
					0,
					PU_PLL_BIT | PU_RX_BIT | PU_TX_BIT,
					mode);

	dev_dbg(lane->dev, "%s: Exit\n", __func__);

	return 0;
}

static int mvebu_a3700_comphy_sgmii_power_on(struct phy *phy)
{
	int ret = 0;
	u32 mask, data;
	void __iomem *sd_ip_addr;
	struct mvebu_a3700_comphy_lane *lane = phy_get_drvdata(phy);
	struct mvebu_comphy_priv *priv = lane->priv;
	int mode = lane->submode;

	dev_dbg(lane->dev, "%s: Enter\n", __func__);

	/* Set selector */
	mvebu_a3700_comphy_set_phy_selector(phy);

	/* Serdes IP Base address
	 * COMPHY Lane0 -- USB3/GBE1
	 * COMPHY Lane1 -- PCIe/GBE0
	 */
	if (lane->id == COMPHY_LANE0) {
		/* Get usb3 and gbe register base address */
		sd_ip_addr = priv->usb3_gbe1_phy_regs;
	} else {
		/* Get pcie and gbe register base address */
		sd_ip_addr = priv->pcie_gbe0_phy_regs;
	}

	/*
	 * 1. Reset PHY by setting PHY input port PIN_RESET=1.
	 * 2. Set PHY input port PIN_TX_IDLE=1, PIN_PU_IVREF=1 to keep
	 *    PHY TXP/TXN output to idle state during PHY initialization
	 * 3. Set PHY input port PIN_PU_PLL=0, PIN_PU_RX=0, PIN_PU_TX=0.
	 */
	data = PIN_PU_IVEREF_BIT | PIN_TX_IDLE_BIT | PIN_RESET_COMPHY_BIT;
	mask = PIN_RESET_CORE_BIT | PIN_PU_PLL_BIT | PIN_PU_RX_BIT | PIN_PU_TX_BIT;
	reg_set(priv->comphy_regs + COMPHY_PHY_CFG1_OFFSET(lane->id), data, mask);

	/*
	 * 4. Release reset to the PHY by setting PIN_RESET=0.
	 */
	data = 0;
	mask = PIN_RESET_COMPHY_BIT;
	reg_set(priv->comphy_regs + COMPHY_PHY_CFG1_OFFSET(lane->id), data, mask);

	/*
	 * 5. Set PIN_PHY_GEN_TX[3:0] and PIN_PHY_GEN_RX[3:0] to decide COMPHY bit rate
	 */
	if (mode == PHY_INTERFACE_MODE_SGMII) {
		/* SGMII 1G, SerDes speed 1.25G */
		data |= SD_SPEED_1_25_G << GEN_RX_SEL_OFFSET;
		data |= SD_SPEED_1_25_G << GEN_TX_SEL_OFFSET;
	} else if (mode == PHY_INTERFACE_MODE_2500BASEX) {
		/* HS SGMII (2.5G), SerDes speed 3.125G */
		data |= SD_SPEED_2_5_G << GEN_RX_SEL_OFFSET;
		data |= SD_SPEED_2_5_G << GEN_TX_SEL_OFFSET;
	} else {
		/* Other rates are not supported */
		dev_err(lane->dev, "unsupported SGMII speed on comphy lane%d\n",
				lane->id);
		return -EINVAL;
	}
	mask = GEN_RX_SEL_MASK | GEN_TX_SEL_MASK;
	reg_set(priv->comphy_regs + COMPHY_PHY_CFG1_OFFSET(lane->id),
		data, mask);

	/* 6. Wait 10mS for bandgap and reference clocks to stabilize; then start SW programming. */
	mdelay(10);

	/* 7. Program COMPHY register PHY_MODE */
	data = COMPHY_MODE_SGMII;
	mask = COMPHY_MODE_MASK;
	reg_set16(SGMIIPHY_ADDR(lane->id, COMPHY_POWER_PLL_CTRL, sd_ip_addr), data, mask);

	/* 8. Set COMPHY register REFCLK_SEL to select the correct REFCLK source */
	data = 0;
	mask = PHY_REF_CLK_SEL;
	reg_set16(SGMIIPHY_ADDR(lane->id, COMPHY_MISC_REG0_ADDR, sd_ip_addr), data, mask);

	/* 9. Set correct reference clock frequency in COMPHY register REF_FREF_SEL. */
	data = REF_CLOCK_SPEED_25M;
	mask = REF_FREF_SEL_MASK;
	reg_set16(SGMIIPHY_ADDR(lane->id, COMPHY_POWER_PLL_CTRL, sd_ip_addr), data, mask);

	/* 10. Program COMPHY register PHY_GEN_MAX[1:0]
	 * This step is mentioned in the flow received from verification team.
	 *  However the PHY_GEN_MAX value is only meaningful for other interfaces (not SGMII)
	 *  For instance, it selects SATA speed 1.5/3/6 Gbps or PCIe speed  2.5/5 Gbps
	 */

	/* 11. Program COMPHY register SEL_BITS to set correct parallel data bus width */
	data = DATA_WIDTH_10BIT;
	mask = SEL_DATA_WIDTH_MASK;
	reg_set16(SGMIIPHY_ADDR(lane->id, COMPHY_LOOPBACK_REG0, sd_ip_addr), data, mask);

	/* 12. As long as DFE function needs to be enabled in any mode,
	 *  COMPHY register DFE_UPDATE_EN[5:0] shall be programmed to 0x3F
	 *  for real chip during COMPHY power on.
	 * The step 14 exists (and empty) in the original initialization flow obtained from
	 *  the verification team. According to the functional specification DFE_UPDATE_EN
	 *  already has the default value 0x3F
	 */

	/* 13. Program COMPHY GEN registers.
	 *  These registers should be programmed based on the lab testing result
	 *  to achieve optimal performance. Please contact the CEA group to get
	 *  the related GEN table during real chip bring-up.
	 *  We only required to run though the entire registers programming flow
	 *  defined by "comphy_sgmii_phy_init" when the REF clock is 40 MHz.
	 *  For REF clock 25 MHz the default values stored in PHY registers are OK.
	 *  For REF clock 40MHz, it is in TODO list, because no API to get REF clock.
	 */

	/* 14. [Simulation Only] should not be used for real chip.
	 *  By pass power up calibration by programming EXT_FORCE_CAL_DONE
	 *  (R02h[9]) to 1 to shorten COMPHY simulation time.
	 */

	/* 15. [Simulation Only: should not be used for real chip]
	 *  Program COMPHY register FAST_DFE_TIMER_EN=1 to shorten RX training simulation time.
	 */

	/*
	 * 16. Check the PHY Polarity invert bit
	 */
	data = 0x0;
	if (lane->invert & COMPHY_POLARITY_TXD_INVERT)
		data |= TXD_INVERT_BIT;
	if (lane->invert & COMPHY_POLARITY_RXD_INVERT)
		data |= RXD_INVERT_BIT;
	reg_set16(SGMIIPHY_ADDR(lane->id, COMPHY_SYNC_PATTERN_REG, sd_ip_addr), data, 0);

	/*
	 *  17. Set PHY input ports PIN_PU_PLL, PIN_PU_TX and PIN_PU_RX to 1 to start
	 *  PHY power up sequence. All the PHY register programming should be done before
	 *  PIN_PU_PLL=1.
	 *  There should be no register programming for normal PHY operation from this point.
	 */
	reg_set(priv->comphy_regs + COMPHY_PHY_CFG1_OFFSET(lane->id),
		PIN_PU_PLL_BIT | PIN_PU_RX_BIT | PIN_PU_TX_BIT,
		PIN_PU_PLL_BIT | PIN_PU_RX_BIT | PIN_PU_TX_BIT);

	/*
	 * 18. Wait for PHY power up sequence to finish by checking output ports
	 * PIN_PLL_READY_TX=1 and PIN_PLL_READY_RX=1.
	 */
	ret = polling_with_timeout(priv->comphy_regs + COMPHY_PHY_STATUS_OFFSET(lane->id),
				   PHY_PLL_READY_TX_BIT | PHY_PLL_READY_RX_BIT,
				   PHY_PLL_READY_TX_BIT | PHY_PLL_READY_RX_BIT,
				   A3700_COMPHY_PLL_LOCK_TIMEOUT,
				   REG_32BIT);
	if (ret) {
		dev_err(lane->dev, "Failed to lock PLL for SGMII PHY %d\n", lane->id);
	}

	/*
	 * 19. Set COMPHY input port PIN_TX_IDLE=0
	 */
	reg_set(priv->comphy_regs + COMPHY_PHY_CFG1_OFFSET(lane->id), 0x0, PIN_TX_IDLE_BIT);

	/*
	 * 20. After valid data appear on PIN_RXDATA bus, set PIN_RX_INIT=1.
	 * to start RX initialization. PIN_RX_INIT_DONE will be cleared to 0 by the PHY
	 * After RX initialization is done, PIN_RX_INIT_DONE will be set to 1 by COMPHY
	 * Set PIN_RX_INIT=0 after PIN_RX_INIT_DONE= 1.
	 * Please refer to RX initialization part for details.
	 */
	reg_set(priv->comphy_regs + COMPHY_PHY_CFG1_OFFSET(lane->id), PHY_RX_INIT_BIT, 0x0);

	ret = polling_with_timeout(priv->comphy_regs + COMPHY_PHY_STATUS_OFFSET(lane->id),
				  PHY_PLL_READY_TX_BIT | PHY_PLL_READY_RX_BIT,
				  PHY_PLL_READY_TX_BIT | PHY_PLL_READY_RX_BIT,
				  A3700_COMPHY_PLL_LOCK_TIMEOUT,
				  REG_32BIT);
	if (ret) {
		dev_err(lane->dev, "Failed to lock PLL for SGMII PHY %d\n", lane->id);
	}


	ret = polling_with_timeout(priv->comphy_regs + COMPHY_PHY_STATUS_OFFSET(lane->id),
				   PHY_RX_INIT_DONE_BIT,
				   PHY_RX_INIT_DONE_BIT,
				   A3700_COMPHY_PLL_LOCK_TIMEOUT,
				   REG_32BIT);
	if (ret) {
		dev_err(lane->dev, "Failed to init RX of SGMII PHY %d\n", lane->id);
	}

	dev_dbg(lane->dev, "%s: Exit\n", __func__);

	return ret;
}

static int mvebu_a3700_comphy_sgmii_power_off(struct phy *phy)
{
	u32 mask, data;
	struct mvebu_a3700_comphy_lane *lane = phy_get_drvdata(phy);
	struct mvebu_comphy_priv *priv = lane->priv;

	dev_dbg(lane->dev, "%s: Enter\n", __func__);

	data = PIN_RESET_CORE_BIT | PIN_RESET_COMPHY_BIT;
	mask = 0;
	reg_set(priv->comphy_regs + COMPHY_PHY_CFG1_OFFSET(lane->id), data, mask);

	dev_dbg(lane->dev, "%s: Exit\n", __func__);

	return 0;
}

static int mvebu_a3700_comphy_usb3_power_on(struct phy *phy)
{
	int ret = 0;
	void __iomem *reg_base = NULL;
	struct mvebu_a3700_comphy_lane *lane = phy_get_drvdata(phy);
	struct mvebu_comphy_priv *priv = lane->priv;
	void (*usb3_reg_set)(void __iomem *addr, u32 reg_offset, u16 data, u16 mask, int mode);
	int mode = lane->mode;

	dev_dbg(lane->dev, "%s: Enter\n", __func__);

	/* Set phy seclector */
	mvebu_a3700_comphy_set_phy_selector(phy);

	/* Set usb3 reg access func, Lane2 is indirect access */
	if (lane->id == COMPHY_LANE2) {
		usb3_reg_set = &mvebu_comphy_reg_set_indirect;
		reg_base = priv->sata_usb3_phy_regs;
	} else {
		usb3_reg_set = &mvebu_comphy_usb3_reg_set_direct;
		reg_base = priv->usb3_gbe1_phy_regs;
	}

	/*
	 * 0. Set PHY OTG Control(0x5d034), bit 4, Power up OTG module
	 *    The register belong to UTMI module, so it is set
	 *    in UTMI phy driver.
	 */

	/*
	 * 1. Set PRD_TXDEEMPH (3.5db de-emph)
	 */
	usb3_reg_set(reg_base,
			COMPHY_REG_LANE_CFG0_ADDR,
			PRD_TXDEEMPH0_MASK,
			(PRD_TXDEEMPH0_MASK | PRD_TXMARGIN_MASK |
			PRD_TXSWING_MASK | CFG_TX_ALIGN_POS_MASK),
			mode);

	/*
	 * 2. Set BIT0: enable transmitter in high impedance mode
	 *    Set BIT[3:4]: delay 2 clock cycles for HiZ off latency
	 *    Set BIT6: Tx detect Rx at HiZ mode
	 *    Unset BIT15: set to 0 to set USB3 De-emphasize level to -3.5db
	 *                 together with bit 0 of COMPHY_REG_LANE_CFG0_ADDR register
	 */
	usb3_reg_set(reg_base,
			COMPHY_REG_LANE_CFG1_ADDR,
			TX_DET_RX_MODE | GEN2_TX_DATA_DLY_DEFT | TX_ELEC_IDLE_MODE_EN,
			PRD_TXDEEMPH1_MASK | TX_DET_RX_MODE | GEN2_TX_DATA_DLY_MASK | TX_ELEC_IDLE_MODE_EN,
			mode);

	/*
	 * 3. Set Spread Spectrum Clock Enabled
	 */
	usb3_reg_set(reg_base,
			COMPHY_REG_LANE_CFG4_ADDR,
			SPREAD_SPECTRUM_CLK_EN,
			SPREAD_SPECTRUM_CLK_EN,
			mode);

	/*
	 * 4. Set Override Margining Controls From the MAC:
	 *    Use margining signals from lane configuration
	 */
	usb3_reg_set(reg_base,
			COMPHY_REG_TEST_MODE_CTRL_ADDR,
			MODE_MARGIN_OVERRIDE,
			REG_16_BIT_MASK,
			mode);

	/*
	 * 5. Set Lane-to-Lane Bundle Clock Sampling Period = per PCLK cycles
	 *    set Mode Clock Source = PCLK is generated from REFCLK
	 */
	usb3_reg_set(reg_base,
			COMPHY_REG_GLOB_CLK_SRC_LO_ADDR,
			0x0,
			(MODE_CLK_SRC | BUNDLE_PERIOD_SEL | BUNDLE_PERIOD_SCALE |
			BUNDLE_SAMPLE_CTRL | PLL_READY_DLY),
			mode);

	/*
	 * 6. Set G2 Spread Spectrum Clock Amplitude at 4K
	 */
	usb3_reg_set(reg_base,
			COMPHY_REG_GEN2_SETTINGS_2,
			G2_TX_SSC_AMP_VALUE_20,
			G2_TX_SSC_AMP_MASK,
			mode);

	/*
	 * 7. Unset G3 Spread Spectrum Clock Amplitude
	 *    set G3 TX and RX Register Master Current Select
	 */
	usb3_reg_set(reg_base,
			COMPHY_REG_GEN2_SETTINGS_3,
			G3_VREG_RXTX_MAS_ISET_60U,
			G3_TX_SSC_AMP_MASK | G3_VREG_RXTX_MAS_ISET_MASK | RSVD_PH03FH_6_0_MASK,
			mode);

	/*
	 * 8. Check crystal jumper setting and program the Power and PLL Control accordingly
	 *    Change RX wait
	 */
	usb3_reg_set(reg_base,
			COMPHY_POWER_PLL_CTRL,
			(PU_IVREF_BIT | PU_PLL_BIT | PU_RX_BIT |
			PU_TX_BIT | PU_TX_INTP_BIT | PU_DFE_BIT |
			COMPHY_MODE_USB3 | USB3_REF_CLOCK_SPEED_25M),
			(PU_IVREF_BIT | PU_PLL_BIT | PU_RX_BIT |
			PU_TX_BIT | PU_TX_INTP_BIT | PU_DFE_BIT |
			PLL_LOCK_BIT | COMPHY_MODE_MASK | REF_FREF_SEL_MASK),
			mode);
	usb3_reg_set(reg_base,
			COMPHY_REG_PWR_MGM_TIM1_ADDR,
			CFG_PM_RXDEN_WAIT_1_UNIT | CFG_PM_RXDLOZ_WAIT_7_UNIT,
			CFG_PM_OSCCLK_WAIT_MASK | CFG_PM_RXDEN_WAIT_MASK | CFG_PM_RXDLOZ_WAIT_MASK,
			mode);

	/*
	 * 9. Enable idle sync
	 */
	usb3_reg_set(reg_base,
			COMPHY_REG_UNIT_CTRL_ADDR,
			UNIT_CTRL_DEFAULT_VALUE | IDLE_SYNC_EN,
			REG_16_BIT_MASK,
			mode);

	/*
	 * 10. Enable the output of 500M clock
	 */
	usb3_reg_set(reg_base,
			COMPHY_MISC_REG0_ADDR,
			MISC_REG0_DEFAULT_VALUE | CLK500M_EN,
			REG_16_BIT_MASK,
			mode);

	/*
	 * 11. Set 20-bit data width
	 */
	usb3_reg_set(reg_base,
			COMPHY_LOOPBACK_REG0,
			DATA_WIDTH_20BIT,
			REG_16_BIT_MASK,
			mode);

	/*
	 * 12. Override Speed_PLL value and use MAC PLL
	 */
	usb3_reg_set(reg_base,
			COMPHY_KVCO_CAL_CTRL,
			SPEED_PLL_VALUE_16 | USE_MAX_PLL_RATE_BIT,
			REG_16_BIT_MASK,
			mode);

	/*
	 * 13. Check the Polarity invert bit
	 */
	if (lane->invert & COMPHY_POLARITY_TXD_INVERT)
		usb3_reg_set(reg_base,
				COMPHY_SYNC_PATTERN_REG,
				TXD_INVERT_BIT,
				TXD_INVERT_BIT,
				mode);
	if (lane->invert & COMPHY_POLARITY_RXD_INVERT)
		usb3_reg_set(reg_base,
				COMPHY_SYNC_PATTERN_REG,
				RXD_INVERT_BIT,
				RXD_INVERT_BIT,
				mode);

	/*
	 * 14. Set max speed generation to USB3.0 5Gbps
	 */
	usb3_reg_set(reg_base,
			COMPHY_SYNC_MASK_GEN_REG,
			PHY_GEN_USB3_5G,
			PHY_GEN_MAX_MASK,
			mode);

	/*
	 * 15. Set capacitor value for FFE gain peaking to 0xF
	 */
	usb3_reg_set(reg_base,
			COMPHY_REG_GEN3_SETTINGS_3,
			COMPHY_GEN_FFE_CAP_SEL_VALUE,
			COMPHY_GEN_FFE_CAP_SEL_MASK,
			mode);

	/*
	 * 16. Release SW reset
	 */
	usb3_reg_set(reg_base,
			COMPHY_REG_GLOB_PHY_CTRL0_ADDR,
			MODE_CORE_CLK_FREQ_SEL | MODE_PIPE_WIDTH_32 | MODE_REFDIV_BY_4,
			REG_16_BIT_MASK,
			mode);

	/* Wait for > 55 us to allow PCLK be enabled */
	udelay(PLL_SET_DELAY_US);

	if (lane->id == COMPHY_LANE2) {
		writel(COMPHY_REG_LANE_STATUS1_ADDR + USB3PHY_LANE2_REG_BASE_OFFSET,
			   priv->sata_usb3_phy_regs + COMPHY_LANE2_INDIR_ADDR_OFFSET);
		ret = polling_with_timeout(priv->sata_usb3_phy_regs + COMPHY_LANE2_INDIR_DATA_OFFSET,
						TXDCLK_PCLK_EN,
						TXDCLK_PCLK_EN,
						A3700_COMPHY_PLL_LOCK_TIMEOUT,
						REG_32BIT);
	} else {
		ret = polling_with_timeout(LANE_STATUS1_ADDR(USB3) + priv->usb3_gbe1_phy_regs,
						TXDCLK_PCLK_EN,
						TXDCLK_PCLK_EN,
						A3700_COMPHY_PLL_LOCK_TIMEOUT,
						REG_16BIT);
	}
	if (ret) {
		dev_err(lane->dev, "Failed to lock USB3 PLL\n");
	}

	dev_dbg(lane->dev, "%s: Exit\n", __func__);

	return ret;
}

static int mvebu_a3700_comphy_usb3_power_off(struct phy *phy)
{
	struct mvebu_a3700_comphy_lane *lane = phy_get_drvdata(phy);

	/*
	 * Currently the USB3 MAC will control the USB3 PHY to set it to low state,
	 * thus do not need to power off USB3 PHY again.
	 */
	dev_dbg(lane->dev, "%s: Enter\n", __func__);

	dev_dbg(lane->dev, "%s: Exit\n", __func__);

	return 0;
}

static int mvebu_a3700_comphy_pcie_power_on(struct phy *phy)
{
	int ret = 0;
	struct mvebu_a3700_comphy_lane *lane = phy_get_drvdata(phy);
	struct mvebu_comphy_priv *priv = lane->priv;

	dev_dbg(lane->dev, "%s: Enter\n", __func__);

	/*
	 * 1. Enable max PLL.
	 */
	reg_set16(LANE_CFG1_ADDR(PCIE) + priv->pcie_gbe0_phy_regs, USE_MAX_PLL_RATE_EN, 0x0);

	/*
	 * 2. Select 20 bit SERDES interface.
	 */
	reg_set16(GLOB_CLK_SRC_LO_ADDR(PCIE) + priv->pcie_gbe0_phy_regs, CFG_SEL_20B, 0);

	/*
	 * 3. Force to use reg setting for PCIe mode
	 */
	reg_set16(MISC_REG1_ADDR(PCIE) + priv->pcie_gbe0_phy_regs, SEL_BITS_PCIE_FORCE, 0);

	/*
	 * 4. Change RX wait
	 */
	reg_set16(PWR_MGM_TIM1_ADDR(PCIE) + priv->pcie_gbe0_phy_regs,
			CFG_PM_RXDEN_WAIT_1_UNIT | CFG_PM_RXDLOZ_WAIT_12_UNIT,
			CFG_PM_OSCCLK_WAIT_MASK | CFG_PM_RXDEN_WAIT_MASK | CFG_PM_RXDLOZ_WAIT_MASK);

	/*
	 * 5. Enable idle sync
	 */
	reg_set16(UNIT_CTRL_ADDR(PCIE) + priv->pcie_gbe0_phy_regs,
			UNIT_CTRL_DEFAULT_VALUE | IDLE_SYNC_EN, REG_16_BIT_MASK);

	/*
	 * 6. Enable the output of 100M/125M/500M clock
	 */
	reg_set16(MISC_REG0_ADDR(PCIE) + priv->pcie_gbe0_phy_regs,
			MISC_REG0_DEFAULT_VALUE | CLK500M_EN | CLK100M_125M_EN,
			REG_16_BIT_MASK);

	/*
	 * 7. Enable TX, PCIE global register, 0xd0074814, it is done in  PCI-E driver
	 */

	/*
	 * 8. Check crystal jumper setting and program the Power and PLL Control accordingly
	 */
	reg_set16(PWR_PLL_CTRL_ADDR(PCIE) + priv->pcie_gbe0_phy_regs,
			(PU_IVREF_BIT | PU_PLL_BIT | PU_RX_BIT | PU_TX_BIT |
			PU_TX_INTP_BIT | PU_DFE_BIT | PCIE_REF_CLOCK_SPEED_25M | COMPHY_MODE_PCIE),
			REG_16_BIT_MASK);

	/*
	 * 9. Override Speed_PLL value and use MAC PLL
	 */
	reg_set16(KVCO_CAL_CTRL_ADDR(PCIE) + priv->pcie_gbe0_phy_regs,
			SPEED_PLL_VALUE_16 | USE_MAX_PLL_RATE_BIT, REG_16_BIT_MASK);

	/*
	 * 10. Check the Polarity invert bit
	 */
	if (lane->invert & COMPHY_POLARITY_TXD_INVERT)
		reg_set16(SYNC_PATTERN_REG_ADDR(PCIE) + priv->pcie_gbe0_phy_regs,
				TXD_INVERT_BIT, 0x0);

	if (lane->invert & COMPHY_POLARITY_RXD_INVERT)
		reg_set16(SYNC_PATTERN_REG_ADDR(PCIE) + priv->pcie_gbe0_phy_regs,
				RXD_INVERT_BIT, 0x0);

	/*
	 * 11. Release SW reset
	 */
	reg_set16(GLOB_PHY_CTRL0_ADDR(PCIE) + priv->pcie_gbe0_phy_regs,
			MODE_CORE_CLK_FREQ_SEL | MODE_PIPE_WIDTH_32,
			SOFT_RESET | MODE_REFDIV);

	/* Wait for > 55 us to allow PCLK be enabled */
	udelay(PLL_SET_DELAY_US);

	ret = polling_with_timeout(LANE_STATUS1_ADDR(PCIE) + priv->pcie_gbe0_phy_regs,
					TXDCLK_PCLK_EN,
					TXDCLK_PCLK_EN,
					A3700_COMPHY_PLL_LOCK_TIMEOUT,
					REG_16BIT);
	if (ret) {
		dev_err(lane->dev, "Failed to lock PCIE PLL\n");
	}

	dev_dbg(lane->dev, "%s: Exit\n", __func__);

	return ret;
}

static int mvebu_a3700_comphy_get_mode(struct phy *phy,
									   unsigned int comphy_index)
{
	struct mvebu_a3700_comphy_lane *lane = phy_get_drvdata(phy);
	struct mvebu_comphy_priv *priv = lane->priv;
	u32 reg;

	reg = readl(priv->comphy_regs + COMPHY_SELECTOR_PHY_REG_OFFSET);

	switch (comphy_index) {
	case COMPHY_LANE0:
		if ((reg & COMPHY_SELECTOR_USB3_GBE1_SEL_BIT) != 0)
			return PHY_MODE_USB_HOST_SS;
		else
			return PHY_MODE_ETHERNET;
	case COMPHY_LANE1:
		if ((reg & COMPHY_SELECTOR_PCIE_GBE0_SEL_BIT) != 0)
			return PHY_MODE_PCIE;
		else
			return PHY_MODE_ETHERNET;
	case COMPHY_LANE2:
		if ((reg & COMPHY_SELECTOR_USB3_PHY_SEL_BIT) != 0)
			return PHY_MODE_USB_HOST_SS;
		else
			return PHY_MODE_SATA;
	}

	return PHY_MODE_INVALID;
}

static int mvebu_a3700_comphy_power_on_legacy(struct phy *phy)
{
	struct mvebu_a3700_comphy_lane *lane = phy_get_drvdata(phy);
	int err = 0;

	dev_dbg(lane->dev, "%s: Enter\n", __func__);

	switch (lane->mode) {
	case PHY_MODE_SATA:
		err = mvebu_a3700_comphy_sata_power_on(phy);
		break;

	case PHY_MODE_ETHERNET:
		switch (lane->submode) {
		case PHY_INTERFACE_MODE_SGMII:
		case PHY_INTERFACE_MODE_2500BASEX:
			err = mvebu_a3700_comphy_sgmii_power_on(phy);
			break;
		default:
			dev_err(lane->dev, "unsupported PHY submode (%d)\n", lane->submode);
			err = -ENOTSUPP;
			break;
		}
		break;

	case PHY_MODE_USB_HOST_SS:
		err = mvebu_a3700_comphy_usb3_power_on(phy);
		break;

	case PHY_MODE_PCIE:
		err = mvebu_a3700_comphy_pcie_power_on(phy);
		break;

	default:
		dev_err(lane->dev, "comphy%d: unsupported comphy mode\n", lane->id);
		err = -EINVAL;
		break;
	}

	dev_dbg(lane->dev, "%s: Exit\n", __func__);

	return err;
}

static int mvebu_a3700_comphy_power_off_legacy(struct phy *phy)
{
	struct mvebu_a3700_comphy_lane *lane = phy_get_drvdata(phy);
	int mode = lane->mode;
	int err = 0;

	dev_dbg(lane->dev, "%s: Enter\n", __func__);

	if (!mode) {
		/*
		 * The user did not specify which mode should be powered off.
		 * In this case we can identify this by reading the phy selector
		 * register.
		 */
		mode = mvebu_a3700_comphy_get_mode(phy, lane->id);
	}

	switch (mode) {
	case PHY_MODE_ETHERNET:
		err = mvebu_a3700_comphy_sgmii_power_off(phy);
		break;
	case PHY_MODE_USB_HOST_SS:
		err = mvebu_a3700_comphy_usb3_power_off(phy);
		break;
	case PHY_MODE_SATA:
		err = mvebu_a3700_comphy_sata_power_off(phy);
		break;

	default:
		dev_dbg(lane->dev,
			"comphy%d: power off is not implemented for mode %d\n",
			lane->id, mode);
		break;
	}

	dev_dbg(lane->dev, "%s: Exit\n", __func__);

	return err;
}

static int mvebu_a3700_comphy_power_on(struct phy *phy)
{
	struct mvebu_a3700_comphy_lane *lane = phy_get_drvdata(phy);
	u32 fw_param;
	int fw_mode;
	int ret;

	fw_mode = mvebu_a3700_comphy_get_fw_mode(lane->id, lane->port,
						 lane->mode, lane->submode);
	if (fw_mode < 0) {
		dev_err(lane->dev, "invalid COMPHY mode\n");
		return fw_mode;
	}

	switch (lane->mode) {
	case PHY_MODE_USB_HOST_SS:
		dev_dbg(lane->dev, "set lane %d to USB3 host mode\n", lane->id);
		fw_param = COMPHY_FW_MODE(fw_mode);
		break;
	case PHY_MODE_SATA:
		dev_dbg(lane->dev, "set lane %d to SATA mode\n", lane->id);
		fw_param = COMPHY_FW_MODE(fw_mode);
		break;
	case PHY_MODE_ETHERNET:
		switch (lane->submode) {
		case PHY_INTERFACE_MODE_SGMII:
			dev_dbg(lane->dev, "set lane %d to SGMII mode\n",
				lane->id);
			fw_param = COMPHY_FW_NET(fw_mode, lane->port,
						 COMPHY_FW_SPEED_1_25G);
			break;
		case PHY_INTERFACE_MODE_2500BASEX:
			dev_dbg(lane->dev, "set lane %d to HS SGMII mode\n",
				lane->id);
			fw_param = COMPHY_FW_NET(fw_mode, lane->port,
						 COMPHY_FW_SPEED_3_125G);
			break;
		default:
			dev_err(lane->dev, "unsupported PHY submode (%d)\n",
				lane->submode);
			return -ENOTSUPP;
		}
		break;
	case PHY_MODE_PCIE:
		dev_dbg(lane->dev, "set lane %d to PCIe mode\n", lane->id);
		fw_param = COMPHY_FW_PCIE(fw_mode, lane->port,
					  COMPHY_FW_SPEED_5G,
					  phy->attrs.bus_width);
		break;
	default:
		dev_err(lane->dev, "unsupported PHY mode (%d)\n", lane->mode);
		return -ENOTSUPP;
	}

	ret = mvebu_a3700_comphy_smc(COMPHY_SIP_POWER_ON, lane->id, fw_param);

	if (!ret) {
		return ret;
	}
	/* Fallback to Linux's implementation */
	return mvebu_a3700_comphy_power_on_legacy(phy);
}

static int mvebu_a3700_comphy_power_off(struct phy *phy)
{
	struct mvebu_a3700_comphy_lane *lane = phy_get_drvdata(phy);
	int ret;

	ret = mvebu_a3700_comphy_smc(COMPHY_SIP_POWER_OFF, lane->id, 0);

	if (!ret) {
		return ret;
	}
	/* Fallback to Linux's implementation */
	return mvebu_a3700_comphy_power_off_legacy(phy);
}

static const struct phy_ops mvebu_a3700_comphy_ops = {
	.power_on	= mvebu_a3700_comphy_power_on,
	.power_off	= mvebu_a3700_comphy_power_off,
	.set_mode	= mvebu_a3700_comphy_set_mode,
	.owner		= THIS_MODULE,
};

static const struct phy_ops mvebu_a3700_comphy_ops_deprecated = {
	.owner		= THIS_MODULE,
};

static struct phy *mvebu_a3700_comphy_xlate(struct device *dev,
					    struct of_phandle_args *args)
{
	struct mvebu_a3700_comphy_lane *lane;
	struct phy *phy;

	if (WARN_ON(args->args[0] >= MVEBU_A3700_COMPHY_PORTS))
		return ERR_PTR(-EINVAL);

	phy = of_phy_simple_xlate(dev, args);
	if (IS_ERR(phy))
		return phy;

	lane = phy_get_drvdata(phy);
	lane->port = args->args[0];

	return phy;
}

static int mvebu_a3700_comphy_probe(struct platform_device *pdev)
{
	struct mvebu_comphy_priv *priv;
	struct phy_provider *provider;
	struct device_node *child;
	struct resource *res;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		return -ENOMEM;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "comphy");
	priv->comphy_regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(priv->comphy_regs)) {
		return PTR_ERR(priv->comphy_regs);
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "lane1_pcie_gbe");
	priv->pcie_gbe0_phy_regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(priv->pcie_gbe0_phy_regs)) {
		return PTR_ERR(priv->pcie_gbe0_phy_regs);
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "lane0_usb3_gbe");
	priv->usb3_gbe1_phy_regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(priv->usb3_gbe1_phy_regs)) {
		return PTR_ERR(priv->usb3_gbe1_phy_regs);
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "lane2_sata_usb3");
	priv->sata_usb3_phy_regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(priv->sata_usb3_phy_regs)) {
		return PTR_ERR(priv->sata_usb3_phy_regs);
	}

	for_each_available_child_of_node(pdev->dev.of_node, child) {
		struct mvebu_a3700_comphy_lane *lane;
		struct phy *phy;
		int ret;
		u32 lane_id;

		ret = of_property_read_u32(child, "reg", &lane_id);
		if (ret < 0) {
			dev_err(&pdev->dev, "missing 'reg' property (%d)\n",
				ret);
			continue;
		}

		if (lane_id >= MVEBU_A3700_COMPHY_LANES) {
			dev_err(&pdev->dev, "invalid 'reg' property\n");
			continue;
		}

		lane = devm_kzalloc(&pdev->dev, sizeof(*lane), GFP_KERNEL);
		if (!lane) {
			of_node_put(child);
			return -ENOMEM;
		}

		phy = devm_phy_create(&pdev->dev, child,
				      &mvebu_a3700_comphy_ops);
		if (IS_ERR(phy)) {
			of_node_put(child);
			return PTR_ERR(phy);
		}

		lane->dev = &pdev->dev;
		lane->mode = PHY_MODE_INVALID;
		lane->submode = PHY_INTERFACE_MODE_NA;
		lane->id = lane_id;
		lane->port = -1;
		lane->invert = COMPHY_POLARITY_NO_INVERT;
		lane->priv = priv;
		phy_set_drvdata(phy, lane);

		/*
		 * To avoid relying on the bootloader/firmware configuration,
		 * power off all comphys.
		 */
		ret = mvebu_a3700_comphy_power_off(phy);
		if (ret == COMPHY_FW_NOT_SUPPORTED) {
			dev_warn(&pdev->dev, "RELYING ON BOTLOADER SETTINGS\n");
			dev_WARN(&pdev->dev, "firmware updated needed\n");

			/*
			 * If comphy power off fails it means that the
			 * deprecated firmware is used and we should rely on
			 * bootloader settings, therefore we are switching to
			 * empty ops.
			 */
			phy_destroy(phy);
			phy = devm_phy_create(&pdev->dev, child,
					      &mvebu_a3700_comphy_ops_deprecated);
			phy_set_drvdata(phy, lane);
		}
	}

	dev_set_drvdata(&pdev->dev, priv);
	provider = devm_of_phy_provider_register(&pdev->dev,
						 mvebu_a3700_comphy_xlate);
	return PTR_ERR_OR_ZERO(provider);
}

static const struct of_device_id mvebu_a3700_comphy_of_match_table[] = {
	{ .compatible = "marvell,comphy-a3700" },
	{ },
};
MODULE_DEVICE_TABLE(of, mvebu_a3700_comphy_of_match_table);

static struct platform_driver mvebu_a3700_comphy_driver = {
	.probe	= mvebu_a3700_comphy_probe,
	.driver	= {
		.name = "mvebu-a3700-comphy",
		.of_match_table = mvebu_a3700_comphy_of_match_table,
	},
};
module_platform_driver(mvebu_a3700_comphy_driver);

MODULE_AUTHOR("Miquèl Raynal <miquel.raynal@bootlin.com>");
MODULE_DESCRIPTION("Common PHY driver for A3700");
MODULE_LICENSE("GPL v2");
