/*
 * Copyright © 2004-2008 Simtec Electronics
 *	http://armlinux.simtec.co.uk/
 *	Ben Dooks <ben@simtec.co.uk>
 *
 * Samsung S3C2410/S3C2440/S3C2412 NAND driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

#define pr_fmt(fmt) "nand-s5pv210: " fmt

#ifdef CONFIG_MTD_NAND_S5PV210_DEBUG
#define DEBUG
#endif

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/cpufreq.h>
#include <linux/of.h>
#include <linux/of_device.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/rawnand.h>
#include <linux/mtd/nand_ecc.h>
#include <linux/mtd/partitions.h>

#include <linux/platform_data/mtd-nand-s5pv210.h>



#define S5PV210_NFREG(x) (x)



/* add by dazhi 2022-01-02*/
#define S5PV210_NFCONF           S5PV210_NFREG(0x00)
#define S5PV210_NFCONT           S5PV210_NFREG(0x04)
#define S5PV210_NFCMD            S5PV210_NFREG(0x08)
#define S5PV210_NFADDR           S5PV210_NFREG(0x0C)
#define S5PV210_NFDATA           S5PV210_NFREG(0x10)
#define S5PV210_NFSTAT           S5PV210_NFREG(0x28)
 
#define S5PV210_NFECC            S5PV210_NFREG(0x20000)
#define S5PV210_NFECCCONF        S5PV210_NFREG(0x00) + (S5PV210_NFECC)
#define S5PV210_NFECCCONT        S5PV210_NFREG(0x20) + (S5PV210_NFECC)
#define S5PV210_NFECCSTAT        S5PV210_NFREG(0x30) + (S5PV210_NFECC)
#define S5PV210_NFECCSECSTAT     S5PV210_NFREG(0x40) + (S5PV210_NFECC)
#define S5PV210_NFECCPRGECC0     S5PV210_NFREG(0x90) + (S5PV210_NFECC)
#define S5PV210_NFECCPRGECC1     S5PV210_NFREG(0x94) + (S5PV210_NFECC)
#define S5PV210_NFECCPRGECC2     S5PV210_NFREG(0x98) + (S5PV210_NFECC)
#define S5PV210_NFECCPRGECC3     S5PV210_NFREG(0x9C) + (S5PV210_NFECC)
#define S5PV210_NFECCERL0        S5PV210_NFREG(0xC0) + (S5PV210_NFECC)
#define S5PV210_NFECCERL1        S5PV210_NFREG(0xC4) + (S5PV210_NFECC)
#define S5PV210_NFECCERL2        S5PV210_NFREG(0xC8) + (S5PV210_NFECC)
#define S5PV210_NFECCERL3        S5PV210_NFREG(0xCC) + (S5PV210_NFECC)
#define S5PV210_NFECCERP0        S5PV210_NFREG(0xF0) + (S5PV210_NFECC)
#define S5PV210_NFECCERP1        S5PV210_NFREG(0xF4) + (S5PV210_NFECC)

//#define S5PV210_NFSTAT_BUSY		(1<<0)

#define S5PV210_NFCONF_EN		(1<<15)
#define S5PV210_NFCONF_INITECC		(1<<12)
#define S5PV210_NFCONF_nFCE		(1<<11)
#define S5PV210_NFSTAT_BUSY		(1<<0)
#define S5PV210_NFCONF_TACLS(x)		((x)<<12)
#define S5PV210_NFCONF_TWRPH0(x)	((x)<<8)
#define S5PV210_NFCONF_TWRPH1(x)	((x)<<4)
#define S5PV210_NFCONT_INITECC		(1<<4)
#define S5PV210_NFCONT_nFCE		(1<<1)
#define S5PV210_NFCONT_ENABLE		(1<<0)
#define S5PV210_NFSTAT_READY		(1<<0)
#define S5PV210_NFCONF_NANDBOOT		(1<<31)
#define S5PV210_NFCONT_INIT_MAIN_ECC	(1<<5)
#define S5PV210_NFCONT_nFCE0		(1<<1)
#define S5PV210_NFSTAT_READY		(1<<0)





/* new oob placement block for use with hardware ecc generation
 */
static int s5pv210_ooblayout_ecc(struct mtd_info *mtd, int section,
				 struct mtd_oob_region *oobregion)
{
	if (section)
		return -ERANGE;

	oobregion->offset = 0;
	oobregion->length = 3;

	return 0;
}

static int s5pv210_ooblayout_free(struct mtd_info *mtd, int section,
				  struct mtd_oob_region *oobregion)
{
	if (section)
		return -ERANGE;

	oobregion->offset = 8;
	oobregion->length = 8;

	return 0;
}

static const struct mtd_ooblayout_ops s5pv210_ooblayout_ops = {
	.ecc = s5pv210_ooblayout_ecc,
	.free = s5pv210_ooblayout_free,
};

/* controller and mtd information */

struct s5pv210_nand_info;

/**
 * struct s3c2410_nand_mtd - driver MTD structure
 * @mtd: The MTD instance to pass to the MTD layer.
 * @chip: The NAND chip information.
 * @set: The platform information supplied for this set of NAND chips.
 * @info: Link back to the hardware information.
*/
struct s5pv210_nand_mtd {
	struct nand_chip		chip;
	struct s5pv210_nand_set		*set;
	struct s5pv210_nand_info	*info;
};

enum s3c_cpu_type {
	TYPE_S3C2410,
	TYPE_S3C2412,
	TYPE_S3C2440,
	TYPE_S5PV210,
};

enum s3c_nand_clk_state {
	CLOCK_DISABLE	= 0,
	CLOCK_ENABLE,
	CLOCK_SUSPEND,
};

/* overview of the s3c2410 nand state */

/**
 * struct s3c2410_nand_info - NAND controller state.
 * @mtds: An array of MTD instances on this controoler.
 * @platform: The platform data for this board.
 * @device: The platform device we bound to.
 * @clk: The clock resource for this controller.
 * @regs: The area mapped for the hardware registers.
 * @sel_reg: Pointer to the register controlling the NAND selection.
 * @sel_bit: The bit in @sel_reg to select the NAND chip.
 * @mtd_count: The number of MTDs created from this controller.
 * @save_sel: The contents of @sel_reg to be saved over suspend.
 * @clk_rate: The clock rate from @clk.
 * @clk_state: The current clock state.
 * @cpu_type: The exact type of this controller.
 */
struct s5pv210_nand_info {
	/* mtd info */
	struct nand_controller		controller;
	struct s5pv210_nand_mtd		*mtds;
	struct s5pv210_platform_nand	*platform;

	/* device info */
	struct device			*device;
	struct clk			*clk;
	void __iomem			*regs;
	void __iomem			*sel_reg;
	int				sel_bit;
	int				mtd_count;
	unsigned long			save_sel;
	unsigned long			clk_rate;
	enum s3c_nand_clk_state		clk_state;

//	enum s3c_cpu_type		cpu_type;   //2022-0103

#ifdef CONFIG_ARM_S3C24XX_CPUFREQ
	struct notifier_block	freq_transition;
#endif
};

struct s5pv210_nand_devtype_data {
	enum s3c_cpu_type type;
};



//2022-01-02 
static const struct s5pv210_nand_devtype_data s5pv210_nand_devtype_data = {
	.type = TYPE_S5PV210,
};



/* conversion functions */

static struct s5pv210_nand_mtd *s5pv210_nand_mtd_toours(struct mtd_info *mtd)
{
	return container_of(mtd_to_nand(mtd), struct s5pv210_nand_mtd,
			    chip);
}

static struct s5pv210_nand_info *s5pv210_nand_mtd_toinfo(struct mtd_info *mtd)
{
	return s5pv210_nand_mtd_toours(mtd)->info;
}

static struct s5pv210_nand_info *to_nand_info(struct platform_device *dev)
{
	return platform_get_drvdata(dev);
}

static struct s5pv210_platform_nand *to_nand_plat(struct platform_device *dev)
{
	return dev_get_platdata(&dev->dev);
}

static inline int allow_clk_suspend(struct s5pv210_nand_info *info)
{
#ifdef CONFIG_MTD_NAND_S5PV210_CLKSTOP
	return 1;
#else
	return 0;
#endif
}

/**
 * s3c2410_nand_clk_set_state - Enable, disable or suspend NAND clock.
 * @info: The controller instance.
 * @new_state: State to which clock should be set.
 */
static void s5pv210_nand_clk_set_state(struct s5pv210_nand_info *info,
		enum s3c_nand_clk_state new_state)
{
	if (!allow_clk_suspend(info) && new_state == CLOCK_SUSPEND)
		return;

	if (info->clk_state == CLOCK_ENABLE) {
		if (new_state != CLOCK_ENABLE)
			clk_disable_unprepare(info->clk);
	} else {
		if (new_state == CLOCK_ENABLE)
			clk_prepare_enable(info->clk);
	}

	info->clk_state = new_state;
}

/* timing calculations */

#define NS_IN_KHZ 1000000
#if 0
/**
 * s3c_nand_calc_rate - calculate timing data.
 * @wanted: The cycle time in nanoseconds.
 * @clk: The clock rate in kHz.
 * @max: The maximum divider value.
 *
 * Calculate the timing value from the given parameters.
 */
static int s5p_nand_calc_rate(int wanted, unsigned long clk, int max)
{
	int result;

	result = DIV_ROUND_UP((wanted * clk), NS_IN_KHZ);

	pr_debug("result %d from %ld, %d\n", result, clk, wanted);

	if (result > max) {
		pr_err("%d ns is too big for current clock rate %ld\n",
			wanted, clk);
		return -1;
	}

	if (result < 1)
		result = 1;

	return result;
}
#endif
#define to_ns(ticks, clk) (((ticks) * NS_IN_KHZ) / (unsigned int)(clk))

/* controller setup */

/**
 * s3c2410_nand_setrate - setup controller timing information.
 * @info: The controller instance.
 *
 * Given the information supplied by the platform, calculate and set
 * the necessary timing registers in the hardware to generate the
 * necessary timing cycles to the hardware.
 */
static int s5pv210_nand_setrate(struct s5pv210_nand_info *info)
{
	struct s5pv210_platform_nand *plat = info->platform;
    unsigned long clkrate = clk_get_rate(info->clk);
    unsigned long cfg;  // flags,
    unsigned int mask ;
    unsigned long set;

    
    info->clk_rate = clkrate;
    clkrate /= 1000;
#if 0
	int tacls_max = 4;
	int tacls, twrph0, twrph1;
	unsigned long clkrate = clk_get_rate(info->clk);
	unsigned long uninitialized_var(set), cfg, uninitialized_var(mask);
	unsigned long flags;

	/* calculate the timing information for the controller */

	info->clk_rate = clkrate;
	clkrate /= 1000;	/* turn clock into kHz for ease of use */

	if (plat != NULL) {
		tacls = s5p_nand_calc_rate(plat->tacls, clkrate, tacls_max);
		twrph0 = s5p_nand_calc_rate(plat->twrph0, clkrate, 8);
		twrph1 = s5p_nand_calc_rate(plat->twrph1, clkrate, 8);
	} else {
		/* default timings */
		tacls = tacls_max;
		twrph0 = 8;
		twrph1 = 8;
	}

	if (tacls < 0 || twrph0 < 0 || twrph1 < 0) {
		dev_err(info->device, "cannot get suitable timings\n");
		return -EINVAL;
	}
#endif


    dev_info(info->device, "Tacls=%d, %dns Twrph0=%d %dns, Twrph1=%d %dns\n",
		plat->tacls, to_ns(plat->tacls, clkrate), plat->twrph0, to_ns(plat->twrph0, clkrate),
						plat->twrph1, to_ns(plat->twrph1, clkrate));


    /* add by dazhi 2022-01-02 */

    mask = (0xF << 12) | (0xF << 8) | (0xF << 4);

    set = S5PV210_NFCONF_TACLS(plat->tacls + 1) ;
    set |= S5PV210_NFCONF_TWRPH0(plat->twrph0 ) ;
    set |= S5PV210_NFCONF_TWRPH1(plat->twrph1) ;     
    /*add end*/


//	local_irq_save(flags);

	cfg = readl(info->regs + S5PV210_NFCONF);
	cfg &= ~mask;
	cfg |= set;
	writel(cfg, info->regs + S5PV210_NFCONF);

//	local_irq_restore(flags);

	dev_dbg(info->device, "NF_CONF is 0x%lx\n", cfg);

	return 0;
}

/**
 * s3c2410_nand_inithw - basic hardware initialisation
 * @info: The hardware state.
 *
 * Do the basic initialisation of the hardware, using s3c2410_nand_setrate()
 * to setup the hardware access speeds and set the controller to be enabled.
*/
static int s5pv210_nand_inithw(struct s5pv210_nand_info *info)
{
//	int ret;
    int cfg;


    dev_info(info->device, "s5pv210_nand_inithw\n");


    cfg = readl(info->regs+S5PV210_NFCONF);
    if(nand_is_slc(info->mtds->chip))
    {
        cfg &= ~(1<<3) ;  //SLC nand flash

        if(info->mtds->chip->mtd->writesize == 2048)
        {
           cfg &= ~(1<<2);  //2k page size
           cfg |= (1<<1) ;   //5 address cycles
        }
    }
    //cfg &= ~(1<<2);  //2k page size    
    writel(cfg, info->regs + S5PV210_NFCONF);

    writel((1<<1) |(1<<0) , info->regs + S5PV210_NFCONT);


	return 0;
}

/**
 * s3c2410_nand_select_chip - select the given nand chip
 * @mtd: The MTD instance for this chip.
 * @chip: The chip number.
 *
 * This is called by the MTD layer to either select a given chip for the
 * @mtd instance, or to indicate that the access has finished and the
 * chip can be de-selected.
 *
 * The routine ensures that the nFCE line is correctly setup, and any
 * platform specific selection code is called to route nFCE to the specific
 * chip.
 */
static void s5pv210_nand_select_chip(struct mtd_info *mtd, int chip)
{
	struct s5pv210_nand_info *info;
	struct s5pv210_nand_mtd *nmtd;
	struct nand_chip *this = mtd_to_nand(mtd);
	unsigned long cur;

	nmtd = nand_get_controller_data(this);
	info = nmtd->info;

	if (chip != -1)
		s5pv210_nand_clk_set_state(info, CLOCK_ENABLE);

	cur = readl(info->sel_reg);

	if (chip == -1) {
		cur |= info->sel_bit;
	} else {
		if (nmtd->set != NULL && chip > nmtd->set->nr_chips) {
			dev_err(info->device, "invalid chip %d\n", chip);
			return;
		}

		if (info->platform != NULL) {
			if (info->platform->select_chip != NULL)
				(info->platform->select_chip) (nmtd->set, chip);
		}

		cur &= ~info->sel_bit;
	}

	writel(cur, info->sel_reg);

	if (chip == -1)
		s5pv210_nand_clk_set_state(info, CLOCK_SUSPEND);
}

/* s3c2410_nand_hwcontrol
 *
 * Issue command and address cycles to the chip
*/




static void s5pv210_nand_hwcontrol(struct mtd_info *mtd, int cmd,
                  unsigned int ctrl)
{
   struct s5pv210_nand_info *info = s5pv210_nand_mtd_toinfo(mtd);

   if (cmd == NAND_CMD_NONE)
       return;

   if (ctrl & NAND_CLE)
       writeb(cmd, info->regs + S5PV210_NFCMD);
   else
       writeb(cmd, info->regs + S5PV210_NFADDR);
}



static int s5pv210_nand_devready(struct mtd_info *mtd)
{
  struct s5pv210_nand_info *info = s5pv210_nand_mtd_toinfo(mtd);
  return readb(info->regs + S5PV210_NFSTAT) & S5PV210_NFSTAT_BUSY;
}






/* add by dazhi 2022-01-02 */
static void s5pv210_nand_enable_hwecc(struct mtd_info *mtd, int mode)
{
   struct s5pv210_nand_info *info = s5pv210_nand_mtd_toinfo(mtd);
   u32 cfg;
   
   if (mode == NAND_ECC_READ)
   {
       /* set 8/12/16bit Ecc direction to Encoding */
       cfg = readl(info->regs + S5PV210_NFECCCONT) & (~(0x1 << 16));
       writel(cfg, info->regs + S5PV210_NFECCCONT);
       
       /* clear 8/12/16bit ecc encode done */
       cfg = readl(info->regs + S5PV210_NFECCSTAT) | (0x1 << 24);
       writel(cfg, info->regs + S5PV210_NFECCSTAT);
   }
   else
   {
       /* set 8/12/16bit Ecc direction to Encoding */
       cfg = readl(info->regs + S5PV210_NFECCCONT) | (0x1 << 16);
       writel(cfg, info->regs + S5PV210_NFECCCONT);
       
       /* clear 8/12/16bit ecc encode done */
       cfg = readl(info->regs + S5PV210_NFECCSTAT) | (0x1 << 25);
       writel(cfg, info->regs + S5PV210_NFECCSTAT);
   }
   
   /* Initialize main area ECC decoder/encoder */
   cfg = readl(info->regs + S5PV210_NFCONT) | (0x1 << 5);
   writel(cfg, info->regs + S5PV210_NFCONT);
   
   /* The ECC message size(For 512-byte message, you should set 511) 8-bit ECC/512B  */
   writel((511 << 16) | 0x3, info->regs + S5PV210_NFECCCONF);
           

   /* Initialize main area ECC decoder/ encoder */
   cfg = readl(info->regs + S5PV210_NFECCCONT) | (0x1 << 2);
   writel(cfg, info->regs + S5PV210_NFECCCONT);
   
   /* Unlock Main area ECC   */
   cfg = readl(info->regs + S5PV210_NFCONT) & (~(0x1 << 7));
   writel(cfg, info->regs + S5PV210_NFCONT);
}

/* add by JerryGou */
static int s5pv210_nand_calculate_ecc(struct mtd_info *mtd, const u_char *dat,
                     u_char *ecc_calc)
{
   u32 cfg;
   struct s5pv210_nand_info *info = s5pv210_nand_mtd_toinfo(mtd);
   u32 nfeccprgecc0 = 0, nfeccprgecc1 = 0, nfeccprgecc2 = 0, nfeccprgecc3 = 0;
   
   /* Lock Main area ECC */
   cfg = readl(info->regs + S5PV210_NFCONT) | (0x1 << 7);
   writel(cfg, info->regs + S5PV210_NFCONT);
   
   if (ecc_calc)   /* NAND_ECC_WRITE */
   {
       /* ECC encoding is completed  */
       while (!(readl(info->regs + S5PV210_NFECCSTAT) & (1 << 25)));
           
       /* 读取13 Byte的Ecc Code */
       nfeccprgecc0 = readl(info->regs + S5PV210_NFECCPRGECC0);
       nfeccprgecc1 = readl(info->regs + S5PV210_NFECCPRGECC1);
       nfeccprgecc2 = readl(info->regs + S5PV210_NFECCPRGECC2);
       nfeccprgecc3 = readl(info->regs + S5PV210_NFECCPRGECC3);

       ecc_calc[0] = nfeccprgecc0 & 0xFF;
       ecc_calc[1] = (nfeccprgecc0 >> 8) & 0xFF;
       ecc_calc[2] = (nfeccprgecc0 >> 16) & 0xFF;
       ecc_calc[3] = (nfeccprgecc0 >> 24) & 0xFF;
       ecc_calc[4] = nfeccprgecc1 & 0xFF;
       ecc_calc[5] = (nfeccprgecc1 >> 8) & 0xFF;
       ecc_calc[6] = (nfeccprgecc1 >> 16) & 0xFF;
       ecc_calc[7] = (nfeccprgecc1 >> 24) & 0xFF;
       ecc_calc[8] = nfeccprgecc2 & 0xFF;
       ecc_calc[9] = (nfeccprgecc2 >> 8) & 0xFF;
       ecc_calc[10] = (nfeccprgecc2 >> 16) & 0xFF;
       ecc_calc[11] = (nfeccprgecc2 >> 24) & 0xFF;
       ecc_calc[12] = nfeccprgecc3 & 0xFF;
   }
   else    /* NAND_ECC_READ */
   {
       /* ECC decoding is completed  */
       while (!(readl(info->regs + S5PV210_NFECCSTAT) & (1 << 24)));
   }
   return 0;
}

/* add by JerryGou */
static int s5pv210_nand_correct_data(struct mtd_info *mtd, u_char *dat,
                    u_char *read_ecc, u_char *calc_ecc)
{
   int ret = 0;
   u32 errNo;
   u32 erl0, erl1, erl2, erl3, erp0, erp1;
   struct s5pv210_nand_info *info = s5pv210_nand_mtd_toinfo(mtd);

   /* Wait until the 8-bit ECC decoding engine is Idle */
   while (readl(info->regs + S5PV210_NFECCSTAT) & (1 << 31));
   
   errNo = readl(info->regs + S5PV210_NFECCSECSTAT) & 0x1F;
   erl0 = readl(info->regs + S5PV210_NFECCERL0);
   erl1 = readl(info->regs + S5PV210_NFECCERL1);
   erl2 = readl(info->regs + S5PV210_NFECCERL2);
   erl3 = readl(info->regs + S5PV210_NFECCERL3);
   
   erp0 = readl(info->regs + S5PV210_NFECCERP0);
   erp1 = readl(info->regs + S5PV210_NFECCERP1);
   
   switch (errNo)
   {
   case 8:
       dat[(erl3 >> 16) & 0x3FF] ^= (erp1 >> 24) & 0xFF;
   case 7:
       dat[erl3 & 0x3FF] ^= (erp1 >> 16) & 0xFF;
   case 6:
       dat[(erl2 >> 16) & 0x3FF] ^= (erp1 >> 8) & 0xFF;
   case 5:
       dat[erl2 & 0x3FF] ^= erp1 & 0xFF;
   case 4:
       dat[(erl1 >> 16) & 0x3FF] ^= (erp0 >> 24) & 0xFF;
   case 3:
       dat[erl1 & 0x3FF] ^= (erp0 >> 16) & 0xFF;
   case 2:
       dat[(erl0 >> 16) & 0x3FF] ^= (erp0 >> 8) & 0xFF;
   case 1:
       dat[erl0 & 0x3FF] ^= erp0 & 0xFF;
   case 0:
       break;
   default:
       ret = -1;
       printk("ECC uncorrectable error detected:%d\n", errNo);
       break;
   }
   
   return ret;
}




uint32_t my_eccpos[64] = { 12, 13, 14, 15, 16, 17, 18, 19, 20, 21,
            22, 23, 24, 25, 26, 27, 28, 29, 30, 31,
            32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 
            42, 43, 44, 45, 46, 47, 48, 49, 50, 51,
            52, 53, 54, 55, 56, 57, 58, 59, 60, 61,
            62, 63};


#if 0
                    
                    
/* add by dazhi */
static struct nand_ecclayout nand_oob_64 = {
    .eccbytes = 52,     /* 2048 / 512 * 13 */
    .eccpos = { 12, 13, 14, 15, 16, 17, 18, 19, 20, 21,
                22, 23, 24, 25, 26, 27, 28, 29, 30, 31,
                32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 
                42, 43, 44, 45, 46, 47, 48, 49, 50, 51,
                52, 53, 54, 55, 56, 57, 58, 59, 60, 61,
                62, 63},
    /* 0和1用于保存坏块标记，12~63保存ecc，剩余2~11为free */
    .oobfree = {
            {.offset = 2,
            .length = 10}
        }
};
                    
#endif






/* add by dazhi2022 */
static int s5pv210_nand_read_page_hwecc(struct mtd_info *mtd, struct nand_chip *chip,
               uint8_t *buf, int oob_required, int page)                
{
   int i, eccsize = chip->ecc.size;
   int eccbytes = chip->ecc.bytes;
   int eccsteps = chip->ecc.steps;
   int col = 0;
   int stat;
   uint8_t *p = buf;
   uint8_t ecccode_buf[64] = {0};
   uint8_t *ecc_code = ecccode_buf;  // chip->buffers->ecccode;
   uint32_t *eccpos =my_eccpos; //chip->ecc.layout->eccpos;

   /* Read the OOB area first */
   col = mtd->writesize;
   chip->cmdfunc(mtd, NAND_CMD_RNDOUT, col, -1);
   chip->read_buf(mtd, chip->oob_poi, mtd->oobsize);
   
   for (i = 0; i < chip->ecc.total; i++)
       ecc_code[i] = chip->oob_poi[eccpos[i]];

   for (i = 0, col = 0; eccsteps; eccsteps--, i += eccbytes, p += eccsize, col += eccsize)
   {   
       chip->cmdfunc(mtd, NAND_CMD_RNDOUT, col, -1);
       chip->ecc.hwctl(mtd, NAND_ECC_READ);
       chip->read_buf(mtd, p, eccsize);
       chip->write_buf(mtd, ecc_code + i, eccbytes);
       chip->ecc.calculate(mtd, NULL, NULL);
       stat = chip->ecc.correct(mtd, p, NULL, NULL);
       if (stat < 0)
           mtd->ecc_stats.failed++;
       else
           mtd->ecc_stats.corrected += stat;
   }
   return 0;
}







static void s5pv210_nand_read_buf(struct mtd_info *mtd, u_char *buf, int len)
{
	struct s5pv210_nand_info *info = s5pv210_nand_mtd_toinfo(mtd);

	readsb(info->regs + S5PV210_NFDATA, buf, len);

	/* cleanup if we've got less than a word to do */
//	if (len & 3) {
//		buf += len & ~3;

//		for (; len & 3; len--)
//			*buf++ = readb(info->regs + S5PV210_NFDATA);
//	}
}



static void s5pv210_nand_write_buf(struct mtd_info *mtd, const u_char *buf,
				   int len)
{
	struct s5pv210_nand_info *info = s5pv210_nand_mtd_toinfo(mtd);

	writesb(info->regs + S5PV210_NFDATA, buf, len);

	/* cleanup any fractional write */
//	if (len & 3) {
//		buf += len & ~3;
//
//		for (; len & 3; len--, buf++)
//			writeb(*buf, info->regs + S5PV210_NFDATA);
//	}
}

/* cpufreq driver support */

#ifdef CONFIG_ARM_S3C24XX_CPUFREQ

static int s5pv210_nand_cpufreq_transition(struct notifier_block *nb,
					  unsigned long val, void *data)
{
	struct s5pv210_nand_info *info;
	unsigned long newclk;

	info = container_of(nb, struct s5pv210_nand_info, freq_transition);
	newclk = clk_get_rate(info->clk);

	if ((val == CPUFREQ_POSTCHANGE && newclk < info->clk_rate) ||
	    (val == CPUFREQ_PRECHANGE && newclk > info->clk_rate)) {
		s5pv210_nand_setrate(info);
	}

	return 0;
}

static inline int s5pv210_nand_cpufreq_register(struct s5pv210_nand_info *info)
{
	info->freq_transition.notifier_call = s5pv210_nand_cpufreq_transition;

	return cpufreq_register_notifier(&info->freq_transition,
					 CPUFREQ_TRANSITION_NOTIFIER);
}

static inline void
s5pv210_nand_cpufreq_deregister(struct s5pv210_nand_info *info)
{
	cpufreq_unregister_notifier(&info->freq_transition,
				    CPUFREQ_TRANSITION_NOTIFIER);
}

#else
static inline int s5pv210_nand_cpufreq_register(struct s5pv210_nand_info *info)
{
	return 0;
}

static inline void
s5pv210_nand_cpufreq_deregister(struct s5pv210_nand_info *info)
{
}
#endif

/* device management functions */

static int s5pv210_nand_remove(struct platform_device *pdev)
{
	struct s5pv210_nand_info *info = to_nand_info(pdev);

	if (info == NULL)
		return 0;

	s5pv210_nand_cpufreq_deregister(info);

	/* Release all our mtds  and their partitions, then go through
	 * freeing the resources used
	 */

	if (info->mtds != NULL) {
		struct s5pv210_nand_mtd *ptr = info->mtds;
		int mtdno;

		for (mtdno = 0; mtdno < info->mtd_count; mtdno++, ptr++) {
			pr_debug("releasing mtd %d (%p)\n", mtdno, ptr);
			nand_release(&ptr->chip);
		}
	}

	/* free the common resources */

	if (!IS_ERR(info->clk))
		s5pv210_nand_clk_set_state(info, CLOCK_DISABLE);

	return 0;
}

static int s5pv210_nand_add_partition(struct s5pv210_nand_info *info,
				      struct s5pv210_nand_mtd *mtd,
				      struct s5pv210_nand_set *set)
{
	if (set) {
		struct mtd_info *mtdinfo = nand_to_mtd(&mtd->chip);

		mtdinfo->name = set->name;

		return mtd_device_register(mtdinfo, set->partitions,
					   set->nr_partitions);
	}

	return -ENODEV;
}

static int s5pv210_nand_setup_data_interface(struct mtd_info *mtd, int csline,
					const struct nand_data_interface *conf)
{
	struct s5pv210_nand_info *info = s5pv210_nand_mtd_toinfo(mtd);
//	struct s5pv210_platform_nand *pdata = info->platform;
//	const struct nand_sdr_timings *timings;
//	int tacls;


    dev_info(info->device, "s5pv210_nand_setup_data_interface\n");
    return 0;
#if 0
	timings = nand_get_sdr_timings(conf);
	if (IS_ERR(timings))
		return -ENOTSUPP;

	tacls = timings->tCLS_min - timings->tWP_min;
	if (tacls < 0)
		tacls = 0;

	pdata->tacls  = DIV_ROUND_UP(tacls, 1000);
	pdata->twrph0 = DIV_ROUND_UP(timings->tWP_min, 1000);
	pdata->twrph1 = DIV_ROUND_UP(timings->tCLH_min, 1000);
#endif
//	return s5pv210_nand_setrate(info);
}

/**
 * s5pv210_nand_init_chip - initialise a single instance of an chip
 * @info: The base NAND controller the chip is on.
 * @nmtd: The new controller MTD instance to fill in.
 * @set: The information passed from the board specific platform data.
 *
 * Initialise the given @nmtd from the information in @info and @set. This
 * readies the structure for use with the MTD layer functions by ensuring
 * all pointers are setup and the necessary control routines selected.
 */
static void s5pv210_nand_init_chip(struct s5pv210_nand_info *info,
				   struct s5pv210_nand_mtd *nmtd,
				   struct s5pv210_nand_set *set)
{
	struct device_node *np = info->device->of_node;
	struct nand_chip *chip = &nmtd->chip;
	void __iomem *regs = info->regs;

	nand_set_flash_node(chip, set->of_node);

	chip->write_buf    = s5pv210_nand_write_buf;
	chip->read_buf     = s5pv210_nand_read_buf;
	chip->select_chip  = s5pv210_nand_select_chip;
	chip->chip_delay   = 50;
	nand_set_controller_data(chip, nmtd);
	chip->options	   = set->options;
	chip->controller   = &info->controller;

	/*
	 * let's keep behavior unchanged for legacy boards booting via pdata and
	 * auto-detect timings only when booting with a device tree.
	 */
	if (np)
		chip->setup_data_interface = s5pv210_nand_setup_data_interface;


	chip->IO_ADDR_W = regs + S5PV210_NFDATA;
	info->sel_reg   = regs + S5PV210_NFCONT;
	info->sel_bit	= (1 << 1);
	chip->cmd_ctrl  = s5pv210_nand_hwcontrol;
	chip->dev_ready = s5pv210_nand_devready;


	chip->IO_ADDR_R = chip->IO_ADDR_W;

	nmtd->info	   = info;
	nmtd->set	   = set;

	chip->ecc.mode = info->platform->ecc_mode;

	/*
	 * If you use u-boot BBT creation code, specifying this flag will
	 * let the kernel fish out the BBT from the NAND.
	 */
	if (set->flash_bbt)
		chip->bbt_options |= NAND_BBT_USE_FLASH;
}

/**
 * s3c2410_nand_attach_chip - Init the ECC engine after NAND scan
 * @chip: The NAND chip
 *
 * This hook is called by the core after the identification of the NAND chip,
 * once the relevant per-chip information is up to date.. This call ensure that
 * we update the internal state accordingly.
 *
 * The internal state is currently limited to the ECC state information.
*/
static int s5pv210_nand_attach_chip(struct nand_chip *chip)
{
	struct mtd_info *mtd = nand_to_mtd(chip);
	struct s5pv210_nand_info *info = s5pv210_nand_mtd_toinfo(mtd);

	switch (chip->ecc.mode) {

	case NAND_ECC_NONE:
		dev_info(info->device, "ECC disabled\n");
		break;

	case NAND_ECC_SOFT:
		/*
		 * This driver expects Hamming based ECC when ecc_mode is set
		 * to NAND_ECC_SOFT. Force ecc.algo to NAND_ECC_HAMMING to
		 * avoid adding an extra ecc_algo field to
		 * s3c2410_platform_nand.
		 */
		chip->ecc.algo = NAND_ECC_HAMMING;
		dev_info(info->device, "soft ECC\n");
		break;

	case NAND_ECC_HW:
		chip->ecc.calculate = s5pv210_nand_calculate_ecc;
		chip->ecc.correct   = s5pv210_nand_correct_data;
        chip->ecc.read_page = s5pv210_nand_read_page_hwecc;
		chip->ecc.strength  = 1;

		
		chip->ecc.hwctl = s5pv210_nand_enable_hwecc;
		

		dev_dbg(info->device, "chip %p => page shift %d\n",
			chip, chip->page_shift);

		/* change the behaviour depending on whether we are using
		 * the large or small page nand device */
		if (chip->page_shift > 10) {
			chip->ecc.size	    = 256;
			chip->ecc.bytes	    = 3;
		} else {
			chip->ecc.size	    = 512;
			chip->ecc.bytes	    = 3;
			mtd_set_ooblayout(nand_to_mtd(chip),
					  &s5pv210_ooblayout_ops);
		}

		dev_info(info->device, "hardware ECC\n");
		break;

	default:
		dev_err(info->device, "invalid ECC mode!\n");
		return -EINVAL;
	}

	if (chip->bbt_options & NAND_BBT_USE_FLASH)
		chip->options |= NAND_SKIP_BBTSCAN;

	return 0;
}

static const struct nand_controller_ops s5pv210_nand_controller_ops = {
	.attach_chip = s5pv210_nand_attach_chip,
};

static const struct of_device_id s5pv210_nand_dt_ids[] = {
	{
		.compatible = "samsung,s5pv210-nand",
		.data = &s5pv210_nand_devtype_data,
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, s5pv210_nand_dt_ids);

static int s5pv210_nand_probe_dt(struct platform_device *pdev)
{
	const struct s5pv210_nand_devtype_data *devtype_data;
	struct s5pv210_platform_nand *pdata;
//	struct s5pv210_nand_info *info = platform_get_drvdata(pdev);
	struct device_node *np = pdev->dev.of_node, *child;
	struct s5pv210_nand_set *sets;

	devtype_data = of_device_get_match_data(&pdev->dev);
	if (!devtype_data)
		return -ENODEV;

	//info->cpu_type = devtype_data->type;

	pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	pdev->dev.platform_data = pdata;



    if (of_property_read_u32(np, "samsung,nandc-tacls", &pdata->tacls))
		pdata->tacls = 1;
    if (of_property_read_u32(np, "samsung,nandc-twrph0", &pdata->twrph0))
		pdata->twrph0 = 7;
    if (of_property_read_u32(np, "samsung,nandc-twrph1", &pdata->twrph1))
		pdata->twrph1 = 3;

    dev_info(&pdev->dev, "tacls =%d twrph0 = %d twrph1=%d \n",pdata->tacls,pdata->twrph0,pdata->twrph1);

	pdata->nr_sets = of_get_child_count(np);
	if (!pdata->nr_sets)
		return 0;

	sets = devm_kcalloc(&pdev->dev, pdata->nr_sets, sizeof(*sets),
			    GFP_KERNEL);
	if (!sets)
		return -ENOMEM;

	pdata->sets = sets;

	for_each_available_child_of_node(np, child) {
		sets->name = (char *)child->name;
		sets->of_node = child;
		sets->nr_chips = 1;

		of_node_get(child);

		sets++;
	}

	return 0;
}

static int s5pv210_nand_probe_pdata(struct platform_device *pdev)
{
//	struct s5pv210_nand_info *info = platform_get_drvdata(pdev);

	//info->cpu_type = platform_get_device_id(pdev)->driver_data;

	return 0;
}

/* s3c24xx_nand_probe
 *
 * called by device layer when it finds a device matching
 * one our driver can handled. This code checks to see if
 * it can allocate all necessary resources then calls the
 * nand layer to look for devices
*/
static int s5pv210_nand_probe(struct platform_device *pdev)
{
	struct s5pv210_platform_nand *plat;
	struct s5pv210_nand_info *info;
	struct s5pv210_nand_mtd *nmtd;
	struct s5pv210_nand_set *sets;
	struct resource *res;
	int err = 0;
	int size;
	int nr_sets;
	int setno;
    const char* clk_name;

  

	info = devm_kzalloc(&pdev->dev, sizeof(*info), GFP_KERNEL);
	if (info == NULL) {
		err = -ENOMEM;
		goto exit_error;
	}

	platform_set_drvdata(pdev, info);

	nand_controller_init(&info->controller);
	info->controller.ops = &s5pv210_nand_controller_ops;

#if 1
	/* get the clock source and enable it */
    if (!of_property_read_string(pdev->dev.of_node, "clock-names", &clk_name)) {
            struct clk *clk = devm_clk_get(&pdev->dev, clk_name);
            if (IS_ERR(clk)) {
                dev_err(&pdev->dev, "cannot get clock of %s\n", clk_name);
                err = PTR_ERR(clk);
                goto exit_error;  //2022-0103
            }
            //clk_prepare_enable(clk);
            dev_info(&pdev->dev, "enable clock '%s'\n", clk_name);
            info->clk = clk;
    }

    else  //设备树分析失败
#endif
    {
    	info->clk = devm_clk_get(&pdev->dev, "nand");
    	if (IS_ERR(info->clk)) {
    		dev_err(&pdev->dev, "failed to get clock\n");
    		err = -ENOENT;
    		goto exit_error;
    	}
    }
   
	s5pv210_nand_clk_set_state(info, CLOCK_ENABLE);

    dev_info(&pdev->dev, "s5pv210_nand_clk_set_state enable clock \n");


	if (pdev->dev.of_node)
	{
        err = s5pv210_nand_probe_dt(pdev);
        dev_info(&pdev->dev, "s5pv210_nand_probe_dt ret = %d \n",err);
	}
	else
		err = s5pv210_nand_probe_pdata(pdev);

	if (err)
		goto exit_error;

	plat = to_nand_plat(pdev);

	/* allocate and map the resource */

	/* currently we assume we have the one resource */
	res = pdev->resource;
	size = resource_size(res);


    dev_info(&pdev->dev, "res->start = %#x end = %#x\n",res->start,res->end);

	info->device	= &pdev->dev;
	info->platform	= plat;

	info->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(info->regs)) {
		err = PTR_ERR(info->regs);
		goto exit_error;
	}

	dev_dbg(&pdev->dev, "mapped registers at %p\n", info->regs);

	if (!plat->sets || plat->nr_sets < 1) {
		err = -EINVAL;
		goto exit_error;
	}

	sets = plat->sets;
	nr_sets = plat->nr_sets;

	info->mtd_count = nr_sets;

	/* allocate our information */
	size = nr_sets * sizeof(*info->mtds);
	info->mtds = devm_kzalloc(&pdev->dev, size, GFP_KERNEL);
	if (info->mtds == NULL) {
		err = -ENOMEM;
		goto exit_error;
	}


    //2022-0108 移动到这个位置
	ret = s5pv210_nand_setrate(info);
	if (ret < 0)
		return ret;

	/* initialise all possible chips */
    
	nmtd = info->mtds;

	for (setno = 0; setno < nr_sets; setno++, nmtd++, sets++) {
		struct mtd_info *mtd = nand_to_mtd(&nmtd->chip);

		pr_debug("initialising set %d (%p, info %p)\n",
			 setno, nmtd, info);

		mtd->dev.parent = &pdev->dev;
		s5pv210_nand_init_chip(info, nmtd, sets);
   

		err = nand_scan(&nmtd->chip, sets ? sets->nr_chips : 1);    //2022-01-06   youwenz
		if (err)
			goto exit_error;
       
		s5pv210_nand_add_partition(info, nmtd, sets);      
	}

	/* initialise the hardware */
	err = s5pv210_nand_inithw(info);
	if (err != 0)
		goto exit_error;

	err = s5pv210_nand_cpufreq_register(info);
	if (err < 0) {
		dev_err(&pdev->dev, "failed to init cpufreq support\n");
		goto exit_error;
	}

	if (allow_clk_suspend(info)) {
		dev_info(&pdev->dev, "clock idle support enabled\n");
		s5pv210_nand_clk_set_state(info, CLOCK_SUSPEND);
	}

	return 0;

 exit_error:
	s5pv210_nand_remove(pdev);

	if (err == 0)
		err = -EINVAL;
	return err;
}

/* PM Support */
#ifdef CONFIG_PM

static int s5pv210_nand_suspend(struct platform_device *dev, pm_message_t pm)
{
	struct s5pv210_nand_info *info = platform_get_drvdata(dev);

	if (info) {
		info->save_sel = readl(info->sel_reg);

		/* For the moment, we must ensure nFCE is high during
		 * the time we are suspended. This really should be
		 * handled by suspending the MTDs we are using, but
		 * that is currently not the case. */

		writel(info->save_sel | info->sel_bit, info->sel_reg);

		s5pv210_nand_clk_set_state(info, CLOCK_DISABLE);
	}

	return 0;
}

static int s5pv210_nand_resume(struct platform_device *dev)
{
	struct s5pv210_nand_info *info = platform_get_drvdata(dev);
	unsigned long sel;

	if (info) {
		s5pv210_nand_clk_set_state(info, CLOCK_ENABLE);
		s5pv210_nand_inithw(info);

		/* Restore the state of the nFCE line. */

		sel = readl(info->sel_reg);
		sel &= ~info->sel_bit;
		sel |= info->save_sel & info->sel_bit;
		writel(sel, info->sel_reg);

		s5pv210_nand_clk_set_state(info, CLOCK_SUSPEND);
	}

	return 0;
}

#else
#define s5pv210_nand_suspend NULL
#define s5pv210_nand_resume NULL
#endif

/* driver device registration */

static const struct platform_device_id s5pv210_driver_ids[] = {
	{
		.name		= "s5pv210-nand",
		.driver_data	= TYPE_S5PV210, /* compatible with s5pv210 */
	},
	{ }
};

MODULE_DEVICE_TABLE(platform, s5pv210_driver_ids);

static struct platform_driver s5pv210_nand_driver = {
	.probe		= s5pv210_nand_probe,
	.remove		= s5pv210_nand_remove,
	.suspend	= s5pv210_nand_suspend,
	.resume		= s5pv210_nand_resume,
	.id_table	= s5pv210_driver_ids,
	.driver		= {
		.name	= "s5pv210-nand",
		.of_match_table = s5pv210_nand_dt_ids,
	},
};

module_platform_driver(s5pv210_nand_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ben Dooks <ben@simtec.co.uk>");
MODULE_DESCRIPTION("S3C24XX MTD NAND driver");
