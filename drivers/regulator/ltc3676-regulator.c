/*
 * Copyright 2013 Linear Technology Corp. All rights reserved.
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
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

/*!
 * @file ltc3676-regulator.c
 * @brief This is the main file for the LTC3676 PMIC regulator driver.
 * i2c should be providing the interface between the PMIC and the MCU.
 *
 * @ingroup PMIC_CORE
 */

#include <linux/slab.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/driver.h>
//#include <linux/regulator/ltc3676.h>
//#include <linux/mfd/ltc3676/core.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/delay.h>
//#include <mach/hardware.h>
//#include <mach/common.h>

#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/regulator/of_regulator.h>
#include "internal.h"

/*
 * STOP -- LOOK at the hardware!  UPDATE these voltage tables!
 *
 * The voltage setting of LTC3676 depend on the R1/R2 values
 * for each switcher.  Look at the hardware design
 * and calculate the correct values for these tables:
 *
 *
 * ltc_sw_[] look-up tables define switching regulator output voltages
 * per DAC code DVBx(0-31) and external resistor divider per formula:
 *
 * Vout = (1+R1/R2)(DVBx*12.5+412.5)(mV) scaled by 1000 to microvolts
 *
 * Following are the feedback resistor values on the NovPek i.MX6 board -- SW1 DDR_VTT
 * DCDC_1_TOP_FB_RESISTOR_R1     100000 (in Ohms)
 * DCDC_1_BOTTOM_FB_RESISTOR_R2  100000 (Do Not Populate, SW1 in Unity Gain for DDR_VTT)
 *
 */
/*static const int ltc_sw1[] = {
	 825000, 850000, 875000, 900000, 925000, 950000, 975000,1000000,
	1025000,1050000,1075000,1100000,1125000,1150000,1175000,1200000,
	1225000,1250000,1275000,1300000,1325000,1350000,1375000,1400000,
	1425000,1450000,1475000,1500000,1525000,1550000,1575000,1600000,
};

/* 
 * Following are the feedback resistor values on the NovPek i.MX6 board -- SW2 VDDSOC_IN
 * DCDC_2_TOP_FB_RESISTOR_R1     100000
 * DCDC_2_BOTTOM_FB_RESISTOR_R2  107000
 */
/*static const int ltc_sw2[] = {
	798014,822196,846379,870561,894743,918925,943107,967290,991472,1015654,
	1039836,1064019,1088201,1112383,1136565,1160748,1184930,1209112,1233294,
	1257477,1281659,1305841,1330023,1354206,1378388,1402570,1426752,1450935,
	1475117,1499299,1523481,1547664,
};

/* 
 * Following are the feedback resistor values on the NovPek i.MX6 board -- SW3 VDDARM_IN
 * DCDC_3_TOP_FB_RESISTOR_R1     100000
 * DCDC_3_BOTTOM_FB_RESISTOR_R2  107000
 */
/*static const int ltc_sw3[] = {
	 798014, 822196, 846379, 870561, 894743, 918925, 943107, 967290, 991472,1015654,
	1039836,1064019,1088201,1112383,1136565,1160748,1184930,1209112,1233294,1257477,
	1281659,1305841,1330023,1354206,1378388,1402570,1426752,1450935,1475117,1499299,
	1523481,1547664,
};

/* 
 * Following are the feedback resistor values on the UIB i.MX6 board -- SW4 VDDARM and VDDSOC
 */
/*static const int ltc_sw4[] = {
	 775000,  798000,  821000,  844000,  867000,  890000,  914000,  937000,
	 960000,  983000, 1010000, 1030000, 1050000, 1075000, 1100000, 1120000,
	1150000, 1175000, 1190000, 1210000, 1240000, 1250000, 1275000, 1310000,
	1330000, 1350000, 1380000, 1400000, 1420000, 1450000, 1470000, 1490000
};

/* Supported voltage values for LD04 regulator in mV */
/*static const u16 LDO4_VSEL_table[] = {
	1200, 2500, 2800, 3000,
};

struct i2c_client *ltc3676_client;

/*
 * LTC3676 Device IO
 */
/*static DEFINE_MUTEX(io_mutex);

static int ltc_3676_set_bits(struct ltc3676_regulator *ltc, u8 reg, u8 mask)
{
	int err, data;

	mutex_lock(&io_mutex);

	data = ltc->read_dev(ltc, reg);
	if (data < 0) {
		dev_err(&ltc->i2c_client->dev,
			"Read from reg 0x%x failed\n", reg);
		err = data;
		goto out;
	}

	data |= mask;
	err = ltc->write_dev(ltc, reg, data);
	if (err)
		dev_err(&ltc->i2c_client->dev,
			"Write for reg 0x%x failed\n", reg);

out:
	mutex_unlock(&io_mutex);
	return err;
}

static int ltc_3676_clear_bits(struct ltc3676_regulator *ltc, u8 reg, u8 mask)
{
	int err, data;

	mutex_lock(&io_mutex);

	data = ltc->read_dev(ltc, reg);
	if (data < 0) {
		dev_err(&ltc->i2c_client->dev,
			"Read from reg 0x%x failed\n", reg);
		err = data;
		goto out;
	}

	data &= ~mask;

	err = ltc->write_dev(ltc, reg, data);
	if (err)
		dev_err(&ltc->i2c_client->dev,
			"Write for reg 0x%x failed\n", reg);

out:
	mutex_unlock(&io_mutex);
	return err;

}

static int ltc3676_reg_read(struct ltc3676_regulator *ltc, u8 reg)
{
	int data;

	mutex_lock(&io_mutex);

	data = ltc->read_dev(ltc, reg);
	if (data < 0)
		dev_err(&ltc->i2c_client->dev,
			"Read from reg 0x%x failed\n", reg);

	mutex_unlock(&io_mutex);
	return data;
}

static int ltc_3676_reg_write(struct ltc3676_regulator *ltc, u8 reg, u8 val)
{
	int err;

	mutex_lock(&io_mutex);

	err = ltc->write_dev(ltc, reg, val);
	if (err < 0)
		dev_err(&ltc->i2c_client->dev,
			"Write for reg 0x%x failed\n", reg);

	mutex_unlock(&io_mutex);
	return err;
}
/* 
 * This set_slew_rate funciton set the switch edge rate on each DC-DC
 * The default edge rate is slow for low EMI.  The fast setting improves
 * efficiency. LTC3676 has slew limiting a regulator startup (soft start)
 * and also a controlled slew rate during voltage changes (Dynamic Voltage
 * Scaling) but neither slew rate is adjustable so cannot be set here.
 */

/*int ltc3676_set_slew_rate(struct regulator_dev *dev, int rate)
{
	struct ltc3676_regulator *ltc = rdev_get_drvdata(dev);
	int dcdc = rdev_get_id(dev);

	// Since there is no slew rate control for LDOs, we return here
	// if the ID is of an LDO
	if (dcdc == LTC3676_LDO_1 || dcdc == LTC3676_LDO_2 ||
		dcdc == LTC3676_LDO_3 || dcdc == LTC3676_LDO_4)
		return 0;
        // For the LTC3676 the slow rate is slow '0' or fast '1'
	if (rate > 0x1)
		return -EINVAL;

	switch (dcdc)
	{ 
		case LTC3676_DCDC_1:
		{
		#ifdef LTC3676_1  //DCDC_1 has a fix function in LTC3676-1, see datasheet.
			return 0;
			break;
		#endif				   
			if (rate)  // Rate is set to FAST
				return ltc_3676_set_bits(ltc, LTC3676_REG_BUCK1, BIT_0_MASK);
			else       // Rate is set to Slow
				return ltc_3676_clear_bits(ltc, LTC3676_REG_BUCK1, BIT_0_MASK);

			break;
		}
		case LTC3676_DCDC_2:
		{
			if (rate)  // Rate is set to FAST
				return ltc_3676_set_bits(ltc, LTC3676_REG_BUCK2, BIT_0_MASK);
			else       // Rate is set to Slow
				return ltc_3676_clear_bits(ltc, LTC3676_REG_BUCK2, BIT_0_MASK);
			break;
		}
		case LTC3676_DCDC_3:
		{
			if (rate)  // Rate is set to FAST
				return ltc_3676_set_bits(ltc, LTC3676_REG_BUCK3, BIT_0_MASK);
			else       // Rate is set to Slow
				return ltc_3676_clear_bits(ltc, LTC3676_REG_BUCK3, BIT_0_MASK);
			break;
		}	

		case LTC3676_DCDC_4:
		{
			if (rate)  // Rate is set to FAST
				return ltc_3676_set_bits(ltc, LTC3676_REG_BUCK4, BIT_0_MASK);
			else       // Rate is set to Slow
				return ltc_3676_clear_bits(ltc, LTC3676_REG_BUCK4, BIT_0_MASK);
			break;
		}	
		default:
		return -EINVAL;			 
	}
	return 0;
}

/* 
 * If hard-wire startup control is used initialization code should 
 * set the Software Control Mode bit in CNTRL Reg 0x09 and set the
 * appropriate regulator enable bits so that this function works
 * correctly.
 *
 * If only hardware enable is used then this function will return
 * an incorrect disabled status because the software enable bits will be low.
 */
/*static int ltc3676_dcdc_is_enabled(struct regulator_dev *dev)
{
	struct ltc3676_regulator *ltc = rdev_get_drvdata(dev);
	int data, dcdc = rdev_get_id(dev);
	
	if (dcdc < LTC3676_DCDC_1 || dcdc > LTC3676_DCDC_4)
		return -EINVAL;

	switch (dcdc)
	{
		case LTC3676_DCDC_1:
	     	{
		#ifdef LTC3676_1  //DCDC_1 has a fixed function in LTC3676-1, see datasheet.
	   		return 1;
	   		break;
		#endif				    
	    		data = ltc3676_reg_read(ltc, LTC3676_REG_BUCK1);
	    		if (data < 0)
				return data;
		
			return (data & 0x80) ? 1 : 0;  //Checking B[7] status
			break;
		}		
		case LTC3676_DCDC_2:
		{
			data = ltc3676_reg_read(ltc, LTC3676_REG_BUCK2);
		   	if (data < 0)
				return data;
			return (data & 0x80) ? 1 : 0;  //Checking B[7] status
			break;
		}			
		case LTC3676_DCDC_3:
		{
			data = ltc3676_reg_read(ltc, LTC3676_REG_BUCK3);
		    	if (data < 0)
				return data;
			return (data & 0x80) ? 1 : 0;  //Checking B[7] status
			break;
		}
		case LTC3676_DCDC_4:
		{
			data = ltc3676_reg_read(ltc, LTC3676_REG_BUCK4);
		    	if (data < 0)
				return data;
			return (data & 0x80) ? 1 : 0;  //Checking B[7] status
			break;
		}	
		default:
		     return -EINVAL;			 
	}
	return 0;
}

/* 
 * If hard-wire startup control is used initialization code should 
 * set the Software Control Mode bit in CNTRL Reg 0x09 and set the
 * appropriate regulator enable bits so that this function works
 * correctly.
 *
 * If only hardware enable is used then this function will return
 * an incorrect disabled status because the software enable bits will be low.
 */
/*static int ltc3676_ldo_is_enabled(struct regulator_dev *dev)
{
	struct ltc3676_regulator *ltc = rdev_get_drvdata(dev);
	int data, ldo = rdev_get_id(dev);

	if (ldo < LTC3676_LDO_1 || ldo > LTC3676_LDO_4)
		return -EINVAL;

	if (ldo == LTC3676_LDO_1)
		return 1;
		
    	switch (ldo)
	{
		case LTC3676_LDO_2:
	     	{
			data = ltc3676_reg_read(ltc, LTC3676_REG_LDOA);
	    		if (data < 0)
				return data;
		
			return (data & 0x04) ? 1 : 0;  //Checking B[2] status
			break;
		}	
		case LTC3676_LDO_3:
	     	{
		    	data = ltc3676_reg_read(ltc, LTC3676_REG_LDOA);
	    		if (data < 0)
				return data;
		
			return (data & 0x20) ? 1 : 0;  //Checking B[5] status
			break;
		}
		case LTC3676_LDO_4:
		{
			data = ltc3676_reg_read(ltc, LTC3676_REG_LDOB);
		    	if (data < 0)
				return data;
			return (data & 0x04) ? 1 : 0;  //Checking B[2] status
			break;
		}	
	 	default:
	     		return -EINVAL;	  
        }			 
	return 0;
}

/* 
 * If hard-wire startup control is used, initialization code should 
 * set the Software Control Mode bit in CNTRL Reg 0x09 and set the
 * appropriate regulator enable bits so that this function works
 * correctly.
 *
 * If only hardware enable is used then this function can still enable
 * any regulator that is not already enabled by the hardware enable pin.
 */
/*static int ltc3676_dcdc_enable(struct regulator_dev *dev)
{
	struct ltc3676_regulator *ltc = rdev_get_drvdata(dev);
	int dcdc = rdev_get_id(dev);

	if (dcdc < LTC3676_DCDC_1 || dcdc > LTC3676_DCDC_4)
		return -EINVAL;

    	switch (dcdc)
	{
		case LTC3676_DCDC_1:
		{
		#ifdef LTC3676_1  //DCDC_1 has a fix function in LTC3676-1, see datasheet.
			break;
		#endif					
			return ltc_3676_set_bits(ltc, LTC3676_REG_BUCK1, BIT_7_MASK);
			break;
		}		
		case LTC3676_DCDC_2:
		{
			return ltc_3676_set_bits(ltc, LTC3676_REG_BUCK2, BIT_7_MASK);
			break;
		}
		case LTC3676_DCDC_3:
		{
			return ltc_3676_set_bits(ltc, LTC3676_REG_BUCK3, BIT_7_MASK);
			break;
		}
		case LTC3676_DCDC_4:
		{
			return ltc_3676_set_bits(ltc, LTC3676_REG_BUCK4, BIT_7_MASK);
			break;
		}	
		default:
		return -EINVAL;			 
	}
    	return 1;
}

/* 
 * If hard-wire startup control is used initialization code should 
 * set the Software Control Mode bit in CNTRL Reg 0x09 and set the
 * appropriate regulator enable bits so that this function works
 * correctly.
 *
 * If only hardware enable is used then this function cannot disable
 * any regulator that is enabled by its hardware enable pin.
 */

/*static int ltc3676_dcdc_disable(struct regulator_dev *dev)
{
	struct ltc3676_regulator *ltc = rdev_get_drvdata(dev);
	int dcdc = rdev_get_id(dev);

	if (dcdc < LTC3676_DCDC_1 || dcdc > LTC3676_DCDC_4)
		return -EINVAL;

    	switch (dcdc)
	{
		case LTC3676_DCDC_1:
		{
		#ifdef LTC3676_1  //DCDC_1 has a fix function in LTC3676-1, see datasheet.
			return 0;
			break;
		#endif					
			return ltc_3676_clear_bits(ltc, LTC3676_REG_BUCK1, BIT_7_MASK);
			break;
		}		
		case LTC3676_DCDC_2:
		{
			return ltc_3676_clear_bits(ltc, LTC3676_REG_BUCK2, BIT_7_MASK);
			break;
		}
		case LTC3676_DCDC_3:
		{
			return ltc_3676_clear_bits(ltc, LTC3676_REG_BUCK3, BIT_7_MASK);
			break;
		}
		case LTC3676_DCDC_4:
		{
			return ltc_3676_clear_bits(ltc, LTC3676_REG_BUCK4, BIT_7_MASK);
			break;
		}	
		default:
		return -EINVAL;			 
	 
	}
	return 0;	
}

/* 
 * If hard-wire startup control is used, initialization code should 
 * set the Software Control Mode bit in CNTRL Reg 0x09 and set the
 * appropriate regulator enable bits so that this function works
 * correctly.
 *
 * If only hardware enable is used then this function can still enable
 * any regulator that is not already enabled by the hardware enable pin.
 */
/*static int ltc3676_ldo_enable(struct regulator_dev *dev)
{
	struct ltc3676_regulator *ltc = rdev_get_drvdata(dev);
	int ldo = rdev_get_id(dev);

	if (ldo < LTC3676_LDO_1 || ldo > LTC3676_LDO_4)
		return -EINVAL;

	if (ldo == LTC3676_LDO_1) {
		printk(KERN_ERR "LDO1 is always enabled\n");
		return 0;
	}
	switch (ldo)
	{
		case LTC3676_LDO_2:
		{
			return ltc_3676_set_bits(ltc, LTC3676_REG_LDOA, BIT_2_MASK);
			break;
             	}		
	
	      	case LTC3676_LDO_3:
		{
			return ltc_3676_set_bits(ltc, LTC3676_REG_LDOA, BIT_5_MASK);
			break;
             	}

	      	case LTC3676_LDO_4:
		{
			return ltc_3676_set_bits(ltc, LTC3676_REG_LDOB, BIT_2_MASK);
			break;
             	}	
     
          	default:
		     return -EINVAL;	 
	}	
   	return 0;
}

/* 
 * If hard-wire startup control is used initialization code should 
 * set the Software Control Mode bit in CNTRL Reg 0x09 and set the
 * appropriate regulator enable bits so that this function works
 * correctly.
 *
 * If only hardware enable is used then this function cannot disable
 * any regulator that is enabled by its hardware enable pin.
 */
/*static int ltc3676_ldo_disable(struct regulator_dev *dev)
{
	struct ltc3676_regulator *ltc = rdev_get_drvdata(dev);
	int ldo = rdev_get_id(dev);

	if (ldo < LTC3676_LDO_1 || ldo > LTC3676_LDO_4)
		return -EINVAL;

	if (ldo == LTC3676_LDO_1) {
		printk(KERN_ERR "LDO1 is always enabled\n");
		return 0;
	}

	switch (ldo)
	{
		case LTC3676_LDO_2:
		{
			return ltc_3676_clear_bits(ltc, LTC3676_REG_LDOA, BIT_2_MASK);
			break;
		}		
		case LTC3676_LDO_3:
		{
			return ltc_3676_clear_bits(ltc, LTC3676_REG_LDOA, BIT_5_MASK);
			break;
		}
		case LTC3676_LDO_4:
		{
			return ltc_3676_clear_bits(ltc, LTC3676_REG_LDOB, BIT_2_MASK);
			break;
		}	
		default:
			return -EINVAL;			 
	}	
	return 0;	
}
/* 
 * This dcdc_set_mode is a basic implementation that puts the regulator in the best mode
 * for the 4 Linux pre-defined operating states.  This function could (should?) be more
 * sophisticated.  The Linux operating mode could also define specific DVFS operating points.
 * FAST mode should raise the core supply voltage to accomodate a fast processor frequency.
 * NORMAL mode should set the regualtor output voltage to a nominal value.
 * IDLE and STANDBY modes should ideally lower the output voltage to reduce processor leakage
 * current and power disipation to extend battery life in mobile applications.
 * Some of the above consideration might be handled in a specific DVFS driver.
 */
/*static int ltc3676_dcdc_set_mode(struct regulator_dev *dev,
				       unsigned int mode)
{
	struct ltc3676_regulator *ltc = rdev_get_drvdata(dev);
	int dcdc = rdev_get_id(dev);
	u16 reg_offset;
	int return_value;

	if (dcdc < LTC3676_DCDC_1 || dcdc > LTC3676_DCDC_4)
		return -EINVAL;
		
	#ifdef LTC3676_1  //DCDC_1 has a fix function in LTC3676-1, see datasheet.
	if (dcdc == LTC3676_DCDC_1)
		return -1;
	#endif			

	reg_offset = dcdc - LTC3676_DCDC_1;

	switch (mode) 
	{
		case REGULATOR_MODE_FAST: 
			/* Force Continuous mode, value of '10' for bits 6/5, lowest ripple */
			/*return_value = ltc_3676_clear_bits(ltc, (LTC3676_REG_BUCK1+reg_offset), BIT_5_MASK);
			if (return_value < 0)
				return return_value;
			return ltc_3676_set_bits(ltc, (LTC3676_REG_BUCK1+reg_offset), BIT_6_MASK);
			break;
		case REGULATOR_MODE_NORMAL:
			/* Pulse Skip mode, value of 00 for bits 6/5, normal fixed frequency operation */
			/*return_value = ltc_3676_clear_bits(ltc, (LTC3676_REG_BUCK1+reg_offset), BIT_6_MASK);
			if (return_value < 0)
				return return_value;
			return ltc_3676_clear_bits(ltc, (LTC3676_REG_BUCK1+reg_offset), BIT_5_MASK);
			break;
		case REGULATOR_MODE_IDLE:
		case REGULATOR_MODE_STANDBY:
			/* Burst mode, value of 01 for bits 6/5, highest light load efficiency */
		/*	return_value = ltc_3676_clear_bits(ltc, (LTC3676_REG_BUCK1+reg_offset), BIT_6_MASK);
			if (return_value < 0)
				return return_value;
			return ltc_3676_set_bits(ltc, (LTC3676_REG_BUCK1+reg_offset), BIT_5_MASK);
			break;
		default:
			return -EINVAL;	 
	}
	return 0;		
}

static unsigned int ltc3676_dcdc_get_mode(struct regulator_dev *dev)
{
	struct ltc3676_regulator *ltc = rdev_get_drvdata(dev);
	int dcdc = rdev_get_id(dev);
	u16 reg_offset;
	int data;
	
	if (dcdc < LTC3676_DCDC_1 || dcdc > LTC3676_DCDC_4)
		return -EINVAL;
	
	#ifdef LTC3676_1  //DCDC_1 has a fix function in LTC3676-1, see datasheet.
	if (dcdc == LTC3676_DCDC_1)
		return -1;
	#endif	

	reg_offset = dcdc - LTC3676_DCDC_1;
	data = ltc3676_reg_read(ltc, LTC3676_REG_BUCK1+reg_offset);

	if (data < 0)
		return data;

	data = (data & 0x60) >> 5;  // Retrieving bits 6/5 

	switch (data) 
	{
		case 0:
		     	return REGULATOR_MODE_NORMAL;
		case 1:
		     	return REGULATOR_MODE_IDLE;
		case 2:
		     	return REGULATOR_MODE_FAST;
		default:
			return -EINVAL;	 
	}

return 0;
}
 
/*int lookUpTable(int table, int min_uV, int max_uV)
{
	int i = 0;
	for (i=0; i<32; i++)
	{
		switch(table)
		{
			case 1:
				if( (ltc_sw1[i] >= min_uV) && (ltc_sw1[i] <= max_uV) )
					return i;
			case 2:
				if( (ltc_sw2[i] >= min_uV) && (ltc_sw2[i] <= max_uV) )
					return i;
			case 3:
				if( (ltc_sw3[i] >= min_uV) && (ltc_sw3[i] <= max_uV) )
					return i;
			case 4:
				if( (ltc_sw4[i] >= min_uV) && (ltc_sw4[i] <= max_uV) )
					return i;
		}
	}
	return -1;
}

static int ltc3676_dcdc_get_voltage(struct regulator_dev *dev)
{
	struct ltc3676_regulator *ltc = rdev_get_drvdata(dev);
	int  volt_reg, data, dcdc = rdev_get_id(dev);
	int  table_uV = 0;

	if (dcdc < LTC3676_DCDC_1 || dcdc > LTC3676_DCDC_4)
		return -EINVAL;

	switch (dcdc) 
	{
		case LTC3676_DCDC_1:
		#ifdef LTC3676_1  //DCDC_1 has a fix function in LTC3676-1, see datasheet.
		       return -1;
		#else			
		
	        	data = ltc3676_reg_read(ltc, LTC3676_REG_DVB1A);
	           	if (data < 0)
		          return data;
		   
		       	volt_reg = data & 0x1F;    //Storing the reference setting for VA reference
		       	data = data & BIT_5_MASK;  //BIT 5 indicates to use VA or VB reference for DCDC_1 
        
               		if (data) // VB is being use, volt_reg will receive the VB value.
		        {
                     		volt_reg = ltc3676_reg_read(ltc, LTC3676_REG_DVB1B);
           	            	if (volt_reg < 0)
		                   return volt_reg;
                     		volt_reg &= 0x1F; //Storing the reference setting for VB reference					
                  	}
			
			table_uV = ltc_sw1[volt_reg];
	        	break;		 
	    	#endif
	    	case LTC3676_DCDC_2:
	       		data = ltc3676_reg_read(ltc, LTC3676_REG_DVB2A);
			if (data < 0)
		        	return data;
		   
		       	volt_reg = data & 0x1F;   	//Storing the reference setting for VA reference
			data = data & BIT_5_MASK;  	//BIT 5 indicates if we use VA or VB reference for DCDC_2 
        
               		if (data) // VB is being use, volt_reg will receive the VB value.
		        {
                     		volt_reg = ltc3676_reg_read(ltc, LTC3676_REG_DVB2B);
           	         	if (volt_reg < 0)
		                   return volt_reg;
                     		volt_reg &= 0x1F;    //Storing the reference setting for VB reference		
                  	}
				  
			table_uV = ltc_sw2[volt_reg];	  
	        	break;	
	    	case LTC3676_DCDC_3:
	        	data = ltc3676_reg_read(ltc, LTC3676_REG_DVB3A);
	           	if (data < 0)
		          	return data;
		   
		       	volt_reg = data & 0x1F;     //Storing the reference setting for VA reference
		      	 data = data & BIT_5_MASK;  //BIT 5 indicates if we use VA or VB reference for DCDC_3 
        
               		if (data) //VB is being use, volt_reg will receive the VB value.
		        {
                     		volt_reg = ltc3676_reg_read(ltc, LTC3676_REG_DVB3B);
           	            	if (volt_reg < 0)
		                   return volt_reg;
                    		 volt_reg &= 0x1F;    //Storing the reference setting for VB reference	
                  	}
				  
			table_uV = ltc_sw3[volt_reg];	  
	        	break;
	   	case LTC3676_DCDC_4:
	    		data = ltc3676_reg_read(ltc, LTC3676_REG_DVB4A);
	           	if (data < 0)
		          return data;
		   
		       	volt_reg = data & 0x1F;     //Storing the reference setting for VA reference
		       	data = data & BIT_5_MASK;   //BIT 5 indicates if we use VA or VB reference for DCDC_4 
        
               		if (data) //VB is being use, volt_reg will receive the VB value.
		        {
                     		volt_reg = ltc3676_reg_read(ltc, LTC3676_REG_DVB4B);
           	            	if (volt_reg < 0)
		                   return volt_reg;
                     		volt_reg &= 0x1F;    //Storing the reference setting for VB reference		
                  	}
			table_uV = ltc_sw4[volt_reg];	  
		        break;			
	    	default:
		   return -EINVAL;
	}
	return table_uV;
}

static int ltc3676_dcdc_set_voltage(struct regulator_dev *dev, int min_uV, int max_uV, unsigned *selector)
{
	struct ltc3676_regulator *ltc = rdev_get_drvdata(dev);
	int dcdc = rdev_get_id(dev);
	int dac_vol, data;

	if (dcdc < LTC3676_DCDC_1 || dcdc > LTC3676_DCDC_4)
		return -EINVAL;

	switch (dcdc) 
	{
      		case LTC3676_DCDC_1: 
            	#ifdef LTC3676_1  //DCDC_1 has a fix function in LTC3676-1, see datasheet.
               		return -1;
			break;
            	#endif	
			dac_vol = lookUpTable(1, min_uV, max_uV); //ltc_sw1[data];	
			if ((dac_vol > 0x1F) || (dac_vol < 0))
		        	return -1;
	  
             		data = ltc3676_reg_read(ltc, LTC3676_REG_DVB1A);
		     	if (data < 0)
		        	return data;

		    	data &= BIT_5_MASK; //Keep the Buck1 Reference Select bit 
		     	data |= dac_vol;    //Now adding the Feedback Reference value
		     	return ltc_3676_reg_write(ltc, LTC3676_REG_DVB1A, data);
		     	break;
	      case LTC3676_DCDC_2:
			dac_vol = lookUpTable(2, min_uV, max_uV); //ltc_sw2[data];
			if ((dac_vol > 0x1F) || (dac_vol < 0))
		        	return -1;

             	    	data = ltc3676_reg_read(ltc, LTC3676_REG_DVB2A);
		     	if (data < 0)
		        	return data;

		     	data &= BIT_5_MASK; //Keep the Buck2 Reference Select bit 	 
		     	data |= dac_vol;    //Now adding the Feedback Reference value
		     	return ltc_3676_reg_write(ltc, LTC3676_REG_DVB2A, data);	
			break;
	      case LTC3676_DCDC_3:
			dac_vol = lookUpTable(3, min_uV, max_uV); //ltc_sw3[data];	
			if ((dac_vol > 0x1F) || (dac_vol < 0))
		        	return -1;

             		data = ltc3676_reg_read(ltc, LTC3676_REG_DVB3A);
		     	if (data < 0)
		        	return data;

		     	data &= BIT_5_MASK; //Keep the Buck3 Reference Select bit 	 
		     	data |= dac_vol;    //Now adding the Feedback Reference value
		     	return ltc_3676_reg_write(ltc, LTC3676_REG_DVB3A, data);	
			break;
	      case LTC3676_DCDC_4:
			if (min_uV == 543210) {
				printk(KERN_ERR "ltc3676 pmic: force hard reset\n");
				ltc_3676_reg_write(ltc, LTC3676_REG_HRST, 0);
			}
			printk(KERN_ERR "ltc3676 pmic: set voltage on Buck4: %d, %d\n", min_uV, max_uV);
			dac_vol = lookUpTable(4, min_uV, max_uV); //ltc_sw4[data];	
			if ((dac_vol > 0x1F) || (dac_vol < 0))
		        	return -1;

             		data = ltc3676_reg_read(ltc, LTC3676_REG_DVB4A);
		     	if (data < 0)
		        	return data;

		     	data &= BIT_5_MASK; //Keep the Buck4 Reference Select bit 	 
		     	data |= dac_vol;    //Now adding the Feedback Reference value
		     	return ltc_3676_reg_write(ltc, LTC3676_REG_DVB4A, data);	
			break; 			 
		default:
			return -EINVAL;
	}		
	return 0;
}

/*
 * _set_suspend_voltage loads the Dynamic Voltage Buck[1-4] Register B (DVBxB)
 * with the requested voltage set point in uV.  However, it does not activate
 * the suspend votlage.  The API must call _set_suspend_enable to activate
 * the suspend voltage level.  This allows the suspend voltage to be configured
 * once and then activated (enabled) easily when the suspend state is entered.
 */

/*int ltc3676_set_suspend_voltage_ex(struct ltc3676_regulator *ltc, int dcdc, int uV)
{
	int dac_vol, data;
	int maxuV = uV + 30000;

	if (dcdc < LTC3676_DCDC_1 || dcdc > LTC3676_DCDC_4)
		return -EINVAL;

	switch (dcdc) 
	{
      		case LTC3676_DCDC_1: 
#ifdef LTC3676_1  //DCDC_1 has a fix function in LTC3676-1, see datasheet.
               		return -1;
			break;
#endif	
			dac_vol = lookUpTable(1, uV, maxuV); //ltc_sw1[data];	
			if ((dac_vol > 0x1F) || (dac_vol < 0))
		        	return -1;
	  
             		data = ltc3676_reg_read(ltc, LTC3676_REG_DVB1B);
		     	if (data < 0)
		        	return data;

		    	data &= BIT_5_MASK; //Keep the Buck1 PGood Mask bit 
		     	data |= dac_vol;    //Now adding the Feedback Reference value
		     	return ltc_3676_reg_write(ltc, LTC3676_REG_DVB1B, data);
		     	break;
	      case LTC3676_DCDC_2:
			dac_vol = lookUpTable(2, uV, maxuV); //ltc_sw2[data];
			if ((dac_vol > 0x1F) || (dac_vol < 0))
		        	return -1;

             	    	data = ltc3676_reg_read(ltc, LTC3676_REG_DVB2B);
		     	if (data < 0)
		        	return data;

		     	data &= BIT_5_MASK; //Keep the Buck2 PGood Mask bit 	 
		     	data |= dac_vol;    //Now adding the Feedback Reference value
		     	return ltc_3676_reg_write(ltc, LTC3676_REG_DVB2B, data);	
			break;
	      case LTC3676_DCDC_3:
			dac_vol = lookUpTable(3, uV, maxuV); //ltc_sw3[data];	
			if ((dac_vol > 0x1F) || (dac_vol < 0))
		        	return -1;

             		data = ltc3676_reg_read(ltc, LTC3676_REG_DVB3B);
		     	if (data < 0)
		        	return data;

		     	data &= BIT_5_MASK; //Keep the Buck3 PGood Mask bit 	 
		     	data |= dac_vol;    //Now adding the Feedback Reference value
		     	return ltc_3676_reg_write(ltc, LTC3676_REG_DVB3B, data);	
			break;
	      case LTC3676_DCDC_4:
			dac_vol = lookUpTable(4, uV, maxuV); //ltc_sw4[data];	
			printk(KERN_ERR "ltc3676 pmic: set suspend voltage on Buck4: %d\n", ltc_sw4[dac_vol]);
			if ((dac_vol > 0x1F) || (dac_vol < 0))
		        	return -1;

             		data = ltc3676_reg_read(ltc, LTC3676_REG_DVB4B);
		     	if (data < 0)
		        	return data;

		     	data &= BIT_5_MASK; //Keep the Buck4 PGood Mask bit 	 
		     	data |= dac_vol;    //Now adding the Feedback Reference value
		     	return ltc_3676_reg_write(ltc, LTC3676_REG_DVB4B, data);	
			break; 			 
		default:
			return -EINVAL;
	}		
	return 0;
}
EXPORT_SYMBOL_GPL(ltc3676_set_suspend_voltage_ex);

static int ltc3676_set_suspend_voltage(struct regulator_dev *dev, int uV)
{
	struct ltc3676_regulator *ltc = rdev_get_drvdata(dev);
	int dcdc = rdev_get_id(dev);

	return ltc3676_set_suspend_voltage_ex(ltc, dcdc, uV);
}

/*
 * _set_suspend_enable sets the DC-DC Reference Select Bit in Reg DVBxA to activate
 * the suspend voltage reference stored in DVBxB to move the output to that pre-loaded
 * suspend voltage (usually a lower voltage for the processor suspend state).
 */
/*static int ltc3676_set_suspend_enable(struct regulator_dev *dev)
{
	struct ltc3676_regulator *ltc = rdev_get_drvdata(dev);
	int dcdc = rdev_get_id(dev);

	if (dcdc < LTC3676_DCDC_1 || dcdc > LTC3676_DCDC_4)
		return -EINVAL;

	switch (dcdc) 
	{
      		case LTC3676_DCDC_1: 
            	#ifdef LTC3676_1  //DCDC_1 has a fix function in LTC3676-1, see datasheet.
               		return -1;
			break;
            	#endif	 
             		//Set the Buck1 Reference Select bit to activate DVB1B suspend voltage
		     	return ltc_3676_set_bits(ltc, LTC3676_REG_DVB1A, BIT_5_MASK);
		     	break;
	      case LTC3676_DCDC_2:
			//Set the Buck2 Reference Select bit to activate DVB2B suspend voltage
		     	return ltc_3676_set_bits(ltc, LTC3676_REG_DVB2A, BIT_5_MASK);	
			break;
	      case LTC3676_DCDC_3:
             		//Set the Buck3 Reference Select bit to activate DVB3B suspend voltage	 
		     	return ltc_3676_set_bits(ltc, LTC3676_REG_DVB3A, BIT_5_MASK);	
			break;
	      case LTC3676_DCDC_4:
			//Set the Buck4 Reference Select bit to activate DVB4B suspend voltage	 
				printk(KERN_ERR "ltc3676 pmic: set suspend enable on Buck4\n");
		     	return ltc_3676_set_bits(ltc, LTC3676_REG_DVB4A, BIT_5_MASK);
			break; 			 
		default:
			return -EINVAL;
	}		
	return 0;
}

/*
 * _set_suspend_disable clears the DC-DC Reference Select Bit in Reg DVBxA to disable
 * the suspend voltage reference stored in DVBxB and select the normal voltage refence
 * stored in DVBxA.
 */
/*static int ltc3676_set_suspend_disable(struct regulator_dev *dev)
{
	struct ltc3676_regulator *ltc = rdev_get_drvdata(dev);
	int dcdc = rdev_get_id(dev);

	if (dcdc < LTC3676_DCDC_1 || dcdc > LTC3676_DCDC_4)
		return -EINVAL;
	  
	switch (dcdc) 
	{
      		case LTC3676_DCDC_1: 
            	#ifdef LTC3676_1  //DCDC_1 has a fix function in LTC3676-1, see datasheet.
               		return -1;
			break;
            	#endif	 
             		//Clear the Buck1 Reference Select bit to activate DVB1A normal voltage
		     	return ltc_3676_clear_bits(ltc, LTC3676_REG_DVB1A, BIT_5_MASK);
		     	break;
	      case LTC3676_DCDC_2:
			//Clear the Buck2 Reference Select bit to activate DVB2A normal voltage
		     	return ltc_3676_clear_bits(ltc, LTC3676_REG_DVB2A, BIT_5_MASK);	
			break;
	      case LTC3676_DCDC_3:
             		//Clear the Buck3 Reference Select bit to activate DVB3A normal voltage	 
		     	return ltc_3676_clear_bits(ltc, LTC3676_REG_DVB3A, BIT_5_MASK);	
			break;
	      case LTC3676_DCDC_4:
			//Clear the Buck4 Reference Select bit to activate DVB4A normal voltage	 
				printk(KERN_ERR "ltc3676 pmic: set suspend disable on Buck4\n");
		     	return ltc_3676_clear_bits(ltc, LTC3676_REG_DVB4A, BIT_5_MASK);
			break; 			 
		default:
			return -EINVAL;
	}		
	return 0;
}

/*
 * suspend mode doesn't exist.  Suspend voltage must be set and then enabled or disabled.
 */
/*static int ltc3676_set_suspend_mode(struct regulator_dev *dev,unsigned int mode)
{
	return 0;
}

#ifdef LTC3676_1
static int ltc3676_ldo_get_voltage(struct regulator_dev *dev)
{
	struct ltc3676_regulator *ltc = rdev_get_drvdata(dev);
	int data, ldo = rdev_get_id(dev);
	//int uV;
		
	// Only LDO4 can be set by software in LTC3676-1
	
	#ifdef LTC3676_1
	if (ldo != LTC3676_LDO_4)  //Only address request for LDO4
		return -EINVAL;

	data = ltc3676_reg_read(ltc, LTC3676_REG_LDOB);
	if (data < 0)
		return data;
			 
	data >>= 3;     // Shift bits 3/4 
       	data &= 0x03;   // Keep the LDO4 voltage bits value	
		
	switch (data) 
	{
		case 0:
			return 1800000;
		case 1:
		    	return 2500000;
		case 2:
		    	return 2800000;
		case 3:
		    	return 3000000;
		default:
		    	return -EINVAL;		
	}				
	#else
	    return -EINVAL;   
	#endif	
}

static int ltc3676_ldo_set_voltage(struct regulator_dev *dev, int min_uV, int max_uV)
{
	struct ltc3676_regulator *ltc = rdev_get_drvdata(dev);
	int data, vsel, ldo = rdev_get_id(dev);
	
	// Only LDO4 can be set by software in LTC3676-1
	
	#ifdef LTC3676_1
	if (ldo != LTC3676_LDO_4)  //Only address request for LDO4
		return -EINVAL;

	for (vsel = 0; vsel < 4; vsel++) 
	{
	 	int mV = LDO4_VSEL_table[vsel];
	     	int uV = mV * 1000;

	     	/* Break at the first in-range value */
	     	/*if (min_uV <= uV && uV <= max_uV)
			break;
	}
		  
	if (vsel == 4)
		return -EINVAL;	

     	data = ltc3676_reg_read(ltc, LTC3676_REG_LDOB);
	if (data < 0)
		return data;
			 
	data &= 0xE7; // Clear the LDO4 voltage select bits
	vsel <<= 3;   // Shift the LDO4 voltage select bit to the correct location within the register
        data |= vsel; 	   
			 		  
	return ltc_3676_reg_write(ltc, LTC3676_REG_LDOB, data);	 		
	
	#else
	    return -EINVAL;   
	#endif		
}

static int ltc3676_ldo4_list_voltage(struct regulator_dev *dev,
					unsigned selector)
{
	int ldo = rdev_get_id(dev);

	#ifdef LTC3676_1	
	if (ldo != LTC3676_LDO_4)
		return -EINVAL;

	if (selector >= 4)
		return -EINVAL;
	else
		return LDO4_VSEL_table[selector] * 1000;
	#endif
	
	return -EINVAL;
}
#endif // LTC3676_1

/* Operations permitted on SWx */
/*static struct regulator_ops ltc3676_sw_ops = 
{
	.is_enabled = ltc3676_dcdc_is_enabled,
	.enable = ltc3676_dcdc_enable,
	.disable = ltc3676_dcdc_disable,
	.get_mode = ltc3676_dcdc_get_mode,
	.set_mode = ltc3676_dcdc_set_mode,
	.get_voltage = ltc3676_dcdc_get_voltage,
	.set_voltage = ltc3676_dcdc_set_voltage,
	.set_suspend_voltage = ltc3676_set_suspend_voltage,
	.set_suspend_enable = ltc3676_set_suspend_enable,
	.set_suspend_disable = ltc3676_set_suspend_disable,
	.set_suspend_mode = ltc3676_set_suspend_mode,
};

// Operations permitted on SW4 
// static struct regulator_ops ltc3676_sw4_ops = {
//	.is_enabled = ltc3676_dcdc_is_enabled,
//	.enable = ltc3676_dcdc_enable,
//	.disable = ltc3676_dcdc_disable,
//	.get_mode = ltc3676_dcdc_get_mode,
//	.set_mode = ltc3676_dcdc_set_mode,
// };

// Operations permitted on LDO1_STBY, LDO3  // LDO1 cannot be controlled
// static struct regulator_ops ltc3676_ldo13_ops = {
//	.is_enabled = ltc3676_ldo_is_enabled,
//	.enable = ltc3676_ldo_enable,
//	.disable = ltc3676_ldo_disable,
// };

#if 0
/* Operations permitted on LDO2 */
/*static struct regulator_ops ltc3676_ldo2_ops = 
{
	.is_enabled = ltc3676_ldo_is_enabled,
	.enable = ltc3676_ldo_enable,
	.disable = ltc3676_ldo_disable,
};

/* Operations permitted on LDO3 */
/*static struct regulator_ops ltc3676_ldo3_ops = 
{
	.is_enabled = ltc3676_ldo_is_enabled,
	.enable = ltc3676_ldo_enable,
	.disable = ltc3676_ldo_disable,
};

/* Operations permitted on LDO4 */
/*static struct regulator_ops ltc3676_ldo4_ops = 
{
	.is_enabled = ltc3676_ldo_is_enabled,
	.enable = ltc3676_ldo_enable,
	.disable = ltc3676_ldo_disable,
	#ifdef LTC3676_1
	   .get_voltage = ltc3676_ldo_get_voltage,      
	   .set_voltage = ltc3676_ldo_set_voltage,      
	   .list_voltage = ltc3676_ldo4_list_voltage,
	#endif   
};
#endif

/*static struct regulator_desc ltc3676_reg_desc[] = {
	{
		.name = "SW1",
		.id = LTC3676_VSW1,
		.ops = &ltc3676_sw_ops,
		.irq = 0,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE
	},
	{
		.name = "SW2",
		.id = LTC3676_VSW2,
		.ops = &ltc3676_sw_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = 0x1F + 1,
		.owner = THIS_MODULE,
	},
	{
		.name = "SW3",
		.id = LTC3676_VSW3,
		.ops = &ltc3676_sw_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = 0x1F + 1,
		.owner = THIS_MODULE,
	},
	{
		.name = "vddarmsoc_ext",
		.id = LTC3676_VSW4,
		.ops = &ltc3676_sw_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = 0x1F + 1,
		.owner = THIS_MODULE,
	},
};

int ltc3676_regulator_probe(struct platform_device *pdev)
{
	struct regulator_desc *rdesc;
	struct regulator_dev *rdev;
	struct ltc3676_regulator *sreg;
	struct regulator_init_data *initdata;
	struct regulator_config config = { };

	//sreg = platform_get_drvdata(pdev);
	sreg = dev_get_drvdata(&pdev->dev);
	initdata = pdev->dev.platform_data;

	init_waitqueue_head(&sreg->wait_q);
	spin_lock_init(&sreg->lock);

	if (pdev->id > LTC3676_SUPPLY_NUM) {		//declare define supply_num 
		rdesc = kzalloc(sizeof(struct regulator_desc), GFP_KERNEL);
		memcpy(rdesc, &ltc3676_reg_desc[LTC3676_SUPPLY_NUM],
			sizeof(struct regulator_desc));
		rdesc->name = kstrdup(sreg->rdata->name, GFP_KERNEL);
	} else
		rdesc = &ltc3676_reg_desc[pdev->id];

	pr_debug("probing regulator %s %s %d\n",
			sreg->rdata->name,
			rdesc->name,
			pdev->id);

	config.dev = &pdev->dev;
	config.driver_data = sreg;
	config.init_data = initdata;


	/* register regulator */
	//rdev = regulator_register(rdesc, &pdev->dev,
	//			  initdata, sreg);
/*	rdev = regulator_register(rdesc, &config );

	if (IS_ERR(rdev)) {
		dev_err(&pdev->dev, "failed to register %s\n",
			rdesc->name);
		return PTR_ERR(rdev);
	}

	return 0;
}


int ltc3676_regulator_remove(struct platform_device *pdev)
{
	struct regulator_dev *rdev = platform_get_drvdata(pdev);

	regulator_unregister(rdev);

	return 0;

}*/

#define DRIVER_NAME		"ltc3676"

/* LTC3676 Registers */
#define LTC3676_BUCK1     0x01
#define LTC3676_BUCK2     0x02
#define LTC3676_BUCK3     0x03
#define LTC3676_BUCK4     0x04
#define LTC3676_LDOA      0x05
#define LTC3676_LDOB      0x06
#define LTC3676_SQD1      0x07
#define LTC3676_SQD2      0x08
#define LTC3676_CNTRL     0x09
#define LTC3676_DVB1A     0x0A
#define LTC3676_DVB1B     0x0B
#define LTC3676_DVB2A     0x0C
#define LTC3676_DVB2B     0x0D
#define LTC3676_DVB3A     0x0E
#define LTC3676_DVB3B     0x0F
#define LTC3676_DVB4A     0x10
#define LTC3676_DVB4B     0x11
#define LTC3676_MSKIRQ    0x12
#define LTC3676_MSKPG     0x13
#define LTC3676_USER      0x14
#define LTC3676_IRQSTAT   0x15
#define LTC3676_PGSTATL   0x16
#define LTC3676_PGSTATRT  0x17
#define LTC3676_HRST      0x1E
#define LTC3676_CLIRQ     0x1F

#define LTC3676_DVBxA_REF_SELECT_MASK	BIT(5)

#define LTC3676_IRQSTAT_PGOOD_TIMEOUT	BIT(3)
#define LTC3676_IRQSTAT_UNDERVOLT_WARN	BIT(4)
#define LTC3676_IRQSTAT_UNDERVOLT_FAULT	BIT(5)
#define LTC3676_IRQSTAT_THERMAL_WARN	BIT(6)
#define LTC3676_IRQSTAT_THERMAL_FAULT	BIT(7)

enum ltc3676_reg {
	LTC3676_SW1,
	LTC3676_SW2,
	LTC3676_SW3,
	LTC3676_vddarmsoc_ext,
	LTC3676_LDO1,
	LTC3676_LDO2,
	LTC3676_LDO3,
	LTC3676_LDO4,
	LTC3676_NUM_REGULATORS,
};

struct ltc3676_regulator {
	struct regulator_desc desc;
	struct device_node *np;

	/* External feedback voltage divider */
	unsigned int r1;
	unsigned int r2;
};

struct ltc3676 {
	struct regmap *regmap;
	struct device *dev;
	struct ltc3676_regulator regulator_descs[LTC3676_NUM_REGULATORS];
	struct regulator_dev *regulators[LTC3676_NUM_REGULATORS];
};


/*
 * NOTE: If the VSTB pin is tied to an input rather than ground, IT IS used to set the suspend voltage.
 * When VSTB is low, the DAC reference voltages are set to DVBxA for BUCKs 1-4,
 * When VSTB is high, the DAC reference voltages are set to DVBxB for BUCKs 1-4.
 * Because the I2C bus is not required to set the suspend/resume voltages,
 * suspend voltages can potentially be lower, thus extending battery life.
 */
static int ltc3676_set_suspend_voltage(struct regulator_dev *rdev, int uV)
{
	struct ltc3676 *ltc3676 = rdev_get_drvdata(rdev);
	struct device *dev = ltc3676->dev;
	int dcdc = rdev_get_id(rdev);
	int sel;

	dev_warn(dev, "%s id=%d uV=%d\n", __func__, dcdc, uV);
	sel = regulator_map_voltage_linear(rdev, uV-25000, uV+25000);
	if (sel < 0)
	{
		dev_err(dev, "%s selection failed: %d", __func__, sel);
		return sel;
	}

	/* DVBB register follows right after the corresponding DVBA register */
	return regmap_update_bits(ltc3676->regmap, rdev->desc->vsel_reg + 1,
				  rdev->desc->vsel_mask, sel);
}

int ltc3676_set_suspend_voltage_ex(struct regulator *reg, int uV)
{
	return ltc3676_set_suspend_voltage(reg->rdev, uV);
}
EXPORT_SYMBOL_GPL(ltc3676_set_suspend_voltage_ex);

static int ltc3676_set_suspend_mode(struct regulator_dev *rdev,
				    unsigned int mode)
{
	struct ltc3676 *ltc3676= rdev_get_drvdata(rdev);
	struct device *dev = ltc3676->dev;
	int mask, bit = 0;
	int dcdc = rdev_get_id(rdev);

	dev_warn(dev, "%s id=%d mode=%d\n", __func__, dcdc, mode);

	/* DVB reference select is bit5 of DVBA reg */
	mask = 1 << 5;

	if (mode != REGULATOR_MODE_STANDBY)
		bit = mask;	/* Select DVBB */

	return regmap_update_bits(ltc3676->regmap, rdev->desc->vsel_reg,
				  mask, bit);
}


static int ltc3676_enable(struct regulator_dev *rdev)
{
	struct ltc3676 *ltc3676= rdev_get_drvdata(rdev);
	struct device *dev = ltc3676->dev;
	int mask, bit = 0;
	int rc = 0;

	//Select DVBA as reference input voltage (when coming out of reset/suspend)
	mask = LTC3676_DVBxA_REF_SELECT_MASK;
	rc = regmap_update_bits(ltc3676->regmap, rdev->desc->vsel_reg,
			  mask, bit);
	if (rc != 0)
		return rc;
	else
		return regulator_enable_regmap(rdev);
}


/* SW1, SW2, SW3, SW4 linear 0.8V-3.3V with scalar via R1/R2 feeback res */
static struct regulator_ops ltc3676_linear_regulator_ops =
{
	.enable = ltc3676_enable, //regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
	.list_voltage = regulator_list_voltage_linear,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
	.get_voltage = regulator_get_voltage,
        .set_voltage = regulator_set_voltage,
	.set_suspend_voltage = ltc3676_set_suspend_voltage,
	.set_suspend_mode = ltc3676_set_suspend_mode,
};

/* always on fixed regulators */
static struct regulator_ops ltc3676_fixed_standby_regulator_ops = {
};

#define LTC3676_REG(_name, _ops, en_reg, en_bit, dvba_reg, dvb_mask)   \
	[LTC3676_ ## _name] = {                                        \
		.desc = {                                              \
			.name = #_name,                                \
			.n_voltages = (dvb_mask) + 1,                  \
			.min_uV = (dvba_reg) ? 412500 : 0,             \
			.uV_step = (dvba_reg) ? 12500 : 0,             \
			.ramp_delay = (dvba_reg) ? 800 : 0,            \
			.fixed_uV = (dvb_mask) ? 0 : 725000,           \
			.ops = &ltc3676_ ## _ops ## _regulator_ops,    \
			.type = REGULATOR_VOLTAGE,                     \
			.id = LTC3676_ ## _name,                       \
			.owner = THIS_MODULE,                          \
			.vsel_reg = (dvba_reg),                        \
			.vsel_mask = (dvb_mask),                       \
			.enable_reg = (en_reg),                        \
			.enable_mask = (1 << en_bit),                  \
		},                                                     \
	}

#define LTC3676_LINEAR_REG(_name, _en, _dvba)                          \
	LTC3676_REG(_name, linear,                                     \
		    LTC3676_ ## _en, 7,                                \
		    LTC3676_ ## _dvba, 0x1f)

#define LTC3676_FIXED_REG(_name) \
	LTC3676_REG(_name, fixed_standby, 0, 0, 0, 0)

static struct ltc3676_regulator ltc3676_regulators[LTC3676_NUM_REGULATORS] = {
	LTC3676_LINEAR_REG(SW1, BUCK1, DVB1A),
	LTC3676_LINEAR_REG(SW2, BUCK2, DVB2A),
	LTC3676_LINEAR_REG(SW3, BUCK3, DVB3A),
	LTC3676_LINEAR_REG(vddarmsoc_ext, BUCK4, DVB4A),
	LTC3676_FIXED_REG(LDO1),
	LTC3676_FIXED_REG(LDO2),
	LTC3676_FIXED_REG(LDO3),
	LTC3676_FIXED_REG(LDO4),
};

#ifdef CONFIG_OF
static struct of_regulator_match ltc3676_matches[] = {
	{ .name = "sw1",	},
	{ .name = "sw2",	},
	{ .name = "sw3",	},
	{ .name = "vddarmsoc_ext",	},
	{ .name = "ldo1",	},
	{ .name = "ldo2",	},
	{ .name = "ldo3",	},
	{ .name = "ldo4",	},
};

static int ltc3676_parse_regulators_dt(struct ltc3676 *ltc3676)
{
	struct device *dev = ltc3676->dev;
	struct device_node *node;
	int i, ret;

	node = of_find_node_by_name(dev->of_node, "regulators");
	if (!node) {
		dev_err(dev, "regulators node not found\n");
		return -EINVAL;
	}

	ret = of_regulator_match(dev, node, ltc3676_matches,
				 ARRAY_SIZE(ltc3676_matches));
	of_node_put(node);
	if (ret < 0) {
		dev_err(dev, "Error parsing regulator init data: %d\n", ret);
		return -EINVAL;
	}

	/* parse feedback voltage deviders: LDO3 doesn't have them */
	for (i = 0; i < LTC3676_NUM_REGULATORS; i++) {
		struct ltc3676_regulator *rdesc = &ltc3676->regulator_descs[i];
		struct device_node *np = ltc3676_matches[i].of_node;
		u32 vdiv[2];
		u32 suspend_mem_microvolts;

		rdesc->np = ltc3676_matches[i].of_node;
		if (i == LTC3676_LDO3 || !rdesc->np)
			continue;
		ret = of_property_read_u32_array(np, "lltc,fb-voltage-divider",
						 vdiv, 2);
		if (ret) {
			dev_err(dev, "Failed to parse voltage divider: %d\n",
				ret);
			return ret;
		}

		rdesc->r1 = vdiv[0];
		rdesc->r2 = vdiv[1];

		if ( i >= LTC3676_LDO1) //Suspend settings should NOT be applied to LDOs
			continue;
		ret = of_property_read_u32(np, "regulator-suspend-mem-microvolt",
				&suspend_mem_microvolts);
		if (ret)
			dev_dbg(dev, "regulator does not have suspend voltage for mem state configured");
		else {
			ltc3676_matches[i].init_data->constraints.state_mem.uV = suspend_mem_microvolts;	/* suspend voltage */
			ltc3676_matches[i].init_data->constraints.state_mem.mode =	REGULATOR_MODE_NORMAL|REGULATOR_MODE_IDLE|REGULATOR_MODE_FAST; /* suspend regulator operating mode */
			ltc3676_matches[i].init_data->constraints.state_mem.enabled = 1; /* is regulator enabled in this suspend state */
			ltc3676_matches[i].init_data->constraints.state_mem.disabled = 0; /* is the regulator disabled in this suspend state */
		}

	}

	return 0;
}

static inline struct regulator_init_data *match_init_data(int index)
{
	return ltc3676_matches[index].init_data;
}

static inline struct device_node *match_of_node(int index)
{
	return ltc3676_matches[index].of_node;
}
#else
static int ltc3676_parse_regulators_dt(struct ltc3676_chip *chip)
{
	return 0;
}

static inline struct regulator_init_data *match_init_data(int index)
{
	return NULL;
}

static inline struct device_node *match_of_node(int index)
{
	return NULL;
}
#endif

static bool ltc3676_writeable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
		case LTC3676_IRQSTAT:
		case LTC3676_BUCK1:
		case LTC3676_BUCK2:
		case LTC3676_BUCK3:
		case LTC3676_BUCK4:
		case LTC3676_LDOA:
		case LTC3676_LDOB:
		case LTC3676_SQD1:
		case LTC3676_SQD2:
		case LTC3676_CNTRL:
		case LTC3676_DVB1A:
		case LTC3676_DVB1B:
		case LTC3676_DVB2A:
		case LTC3676_DVB2B:
		case LTC3676_DVB3A:
		case LTC3676_DVB3B:
		case LTC3676_DVB4A:
		case LTC3676_DVB4B:
		case LTC3676_MSKIRQ:
		case LTC3676_MSKPG:
		case LTC3676_USER:
		case LTC3676_HRST:
		case LTC3676_CLIRQ:
			return true;
	}
	return false;
}

static bool ltc3676_readable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
		case LTC3676_IRQSTAT:
		case LTC3676_BUCK1:
		case LTC3676_BUCK2:
		case LTC3676_BUCK3:
		case LTC3676_BUCK4:
		case LTC3676_LDOA:
		case LTC3676_LDOB:
		case LTC3676_SQD1:
		case LTC3676_SQD2:
		case LTC3676_CNTRL:
		case LTC3676_DVB1A:
		case LTC3676_DVB1B:
		case LTC3676_DVB2A:
		case LTC3676_DVB2B:
		case LTC3676_DVB3A:
		case LTC3676_DVB3B:
		case LTC3676_DVB4A:
		case LTC3676_DVB4B:
		case LTC3676_MSKIRQ:
		case LTC3676_MSKPG:
		case LTC3676_USER:
		case LTC3676_HRST:
		case LTC3676_CLIRQ:
			return true;
	}
	return false;
}

static bool ltc3676_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
		case LTC3676_IRQSTAT:
		case LTC3676_PGSTATL:
		case LTC3676_PGSTATRT:
			return true;
	}
	return false;
}

static struct reg_default ltc3676_reg_defaults[] = {
	/* BUCK 1-4 are set to pulse skipping mode, fixed frequency except at light loads
	 * Burst Mode- 0x20 — Highest light load efficiency, wider load range of variable frequency
	 * Continuous Mode- 0x40 — Constant frequency, low ripple, worst light load efficiency
	 */
	{ LTC3676_BUCK1, 0x0 },
	{ LTC3676_BUCK2, 0x0 },
	{ LTC3676_BUCK3, 0x0 },
	{ LTC3676_BUCK4, 0x0 },

	/* Power down sequence for BUCKs 4-1(SQD1), and LDOs 4-2(SQD2), following low WAKE
	 * General practice is to set the turn-off order the reverse of the turn-on order
	 * The first supply ON should be that last to turn OFF
	 * 00 = With WAKE
	 * 01 = WAKE + 100ms
	 * 10 = WAKE + 200ms
	 * 11 = WAKE + 300ms
	 */
	{ LTC3676_SQD1, 0x0 },
	{ LTC3676_SQD2, 0x0 },

	//PGOOD NOT forced low when slewing: Bit(5)=1
	{ LTC3676_DVB1B, 0x39 },
	{ LTC3676_DVB2B, 0x39 },
	{ LTC3676_DVB3B, 0x39 },
	{ LTC3676_DVB4B, 0x39 },

	/* Control register configuration
	 * Default UV Warn = 000 = 2.7V, Over Temp Warn = 00 = 10 deg C below OT shutdown
	 * Software Control Mode (Bit 5) Inhibits Pin Control so software can manage each regulator state
	 * Pushbutton Hard Reset (Bit 6) 10 seconds (default), set to reduce to 5 seconds
	 */
	{ LTC3676_CNTRL, 0x0 },
};

static const struct regmap_config ltc3676_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.writeable_reg = ltc3676_writeable_reg,
	.readable_reg = ltc3676_readable_reg,
	.volatile_reg = ltc3676_volatile_reg,
	.max_register = LTC3676_CLIRQ,
	.reg_defaults = ltc3676_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(ltc3676_reg_defaults),
	.use_single_rw = true,
	.cache_type = REGCACHE_RBTREE,
};

static irqreturn_t ltc3676_isr(int irq, void *dev_id)
{
	struct ltc3676 *ltc3676 = dev_id;
	struct device *dev = ltc3676->dev;
	unsigned int i, irqstat, event;

	regmap_read(ltc3676->regmap, LTC3676_IRQSTAT, &irqstat);

	dev_dbg(dev, "irq%d irqstat=0x%02x\n", irq, irqstat);
	if (irqstat & LTC3676_IRQSTAT_THERMAL_WARN) {
		dev_info(dev, "Over-temperature Warning\n");
		event = REGULATOR_EVENT_OVER_TEMP;
		for (i = 0; i < LTC3676_NUM_REGULATORS; i++)
			regulator_notifier_call_chain(ltc3676->regulators[i],
						      event, NULL);
	}

	if (irqstat & LTC3676_IRQSTAT_UNDERVOLT_WARN) {
		dev_info(dev, "Undervoltage Warning\n");
		event = REGULATOR_EVENT_UNDER_VOLTAGE;
		for (i = 0; i < LTC3676_NUM_REGULATORS; i++)
			regulator_notifier_call_chain(ltc3676->regulators[i],
						      event, NULL);
	}

	/* Clear warning condition */
	regmap_write(ltc3676->regmap, LTC3676_CLIRQ, 0);

	return IRQ_HANDLED;
}

static inline unsigned int ltc3676_scale(unsigned int uV, u32 r1, u32 r2)
{
	uint64_t tmp;
	if (uV == 0)
		return 0;
	tmp = (uint64_t)uV * r1;
	do_div(tmp, r2);
	return uV + (unsigned int)tmp;
}

static void ltc3676_apply_fb_voltage_divider(struct ltc3676_regulator *rdesc)
{
	struct regulator_desc *desc = &rdesc->desc;

	if (!rdesc->r1 || !rdesc->r2)
		return;

	desc->min_uV = ltc3676_scale(desc->min_uV, rdesc->r1, rdesc->r2);
	desc->uV_step = ltc3676_scale(desc->uV_step, rdesc->r1, rdesc->r2);
	desc->fixed_uV = ltc3676_scale(desc->fixed_uV, rdesc->r1, rdesc->r2);
}

static int ltc3676_regulator_probe(struct i2c_client *client,
				    const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct ltc3676_regulator *descs;
	struct ltc3676 *ltc3676;
	int i, ret;

	ltc3676 = devm_kzalloc(dev, sizeof(*ltc3676), GFP_KERNEL);
	if (!ltc3676)
		return -ENOMEM;

	i2c_set_clientdata(client, ltc3676);
	ltc3676->dev = dev;

	descs = ltc3676->regulator_descs;
	memcpy(descs, ltc3676_regulators, sizeof(ltc3676_regulators));

	ltc3676->regmap = devm_regmap_init_i2c(client, &ltc3676_regmap_config);
	if (IS_ERR(ltc3676->regmap)) {
		ret = PTR_ERR(ltc3676->regmap);
		dev_err(dev, "failed to initialize regmap: %d\n", ret);
		return ret;
	}

	ret = ltc3676_parse_regulators_dt(ltc3676);
	if (ret)
		return ret;

	for (i = 0; i < LTC3676_NUM_REGULATORS; i++) {
		struct ltc3676_regulator *rdesc = &ltc3676->regulator_descs[i];
		struct regulator_desc *desc = &rdesc->desc;
		struct regulator_init_data *init_data;
		struct regulator_config config = { };

		init_data = match_init_data(i);

		if (!rdesc->np)
			continue;

		if (i != LTC3676_LDO3) {
			/* skip unused (defined by r1=r2=0) */
			if (rdesc->r1 == 0 && rdesc->r2 == 0)
				continue;
			ltc3676_apply_fb_voltage_divider(rdesc);
		}

		config.dev = dev;
		config.init_data = init_data;
		config.driver_data = ltc3676;
		config.of_node = match_of_node(i);

		ltc3676->regulators[i] = regulator_register(desc, &config);
		if (IS_ERR(ltc3676->regulators[i])) {
			ret = PTR_ERR(ltc3676->regulators[i]);
			dev_err(dev, "failed to register regulator %s: %d\n",
				desc->name, ret);
			return ret;
		}
	}

	regmap_write(ltc3676->regmap, LTC3676_CLIRQ, 0);
	ret = devm_request_threaded_irq(dev, client->irq, NULL, ltc3676_isr,
					IRQF_TRIGGER_LOW | IRQF_ONESHOT,
					client->name, ltc3676);
	if (ret) {
		dev_err(dev, "Failed to request IRQ: %d\n", ret);
		return ret;
	}

	return 0;
}

static const struct i2c_device_id ltc3676_i2c_id[] = {
	{ "ltc3676" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ltc3676_i2c_id);

static struct i2c_driver ltc3676_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
	},
	.probe = ltc3676_regulator_probe,
	.id_table = ltc3676_i2c_id,
};
module_i2c_driver(ltc3676_driver);

MODULE_AUTHOR("Tim Harvey <tharvey@gateworks.com>");
MODULE_AUTHOR("Jaffer Kapasi <jkapasi@linear.com>");
MODULE_DESCRIPTION("Regulator Driver for Linear Technology LTC1376");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("i2c:ltc3676");


/*int ltc3676_register_regulator(struct ltc3676_regulator *reg_data, int reg,
			       struct regulator_init_data *initdata)
{
	struct platform_device *pdev;
	int ret;

	if (reg_data->pmic.pdev[reg])
		return -EBUSY;

	pdev = platform_device_alloc("ltc3676_reg", reg);
	if (!pdev)
		return -ENOMEM;

	printk(KERN_INFO "%s: Trying to register regulator %s, %d\n", __func__,
			initdata->constraints.name, reg);
	
	reg_data->pmic.pdev[reg] = pdev;
	initdata->driver_data = reg_data;

	pdev->dev.platform_data = initdata;
	pdev->dev.parent = reg_data->dev;
	
	platform_set_drvdata(pdev, reg_data);
	ret = platform_device_add(pdev);

	if (ret != 0) {
		pr_debug("Failed to register regulator %d: %d\n",
			reg, ret);
		platform_device_del(pdev);
	}
	printk(KERN_INFO "register regulator %s, %d: %d\n",
			reg_data->rdata->name, reg, ret);

	return ret;
}
EXPORT_SYMBOL_GPL(ltc3676_register_regulator);

struct platform_driver ltc3676_reg = {
	.driver = {
		.name	= "ltc3676_reg",
		.owner	= THIS_MODULE,
	},
	.probe	= ltc3676_regulator_probe,
	.remove	= ltc3676_regulator_remove,
};

int ltc3676_regulator_init(void)
{
	printk(KERN_INFO "%s Initializing LTC3676 regulator_init\n", __func__);
	return platform_driver_register(&ltc3676_reg);
}
subsys_initcall_sync(ltc3676_regulator_init);
//postcore_initcall(ltc3676_regulator_init);

void ltc3676_regulator_exit(void)
{
	platform_driver_unregister(&ltc3676_reg);
}
module_exit(ltc3676_regulator_exit);

MODULE_AUTHOR("Linear Technology & NovTech, Inc.");
MODULE_DESCRIPTION("LTC3676 Regulator driver");
MODULE_LICENSE("GPL");*/


