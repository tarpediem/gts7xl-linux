// SPDX-License-Identifier: GPL-2.0
/*
 * Based on max77650-charger.c
 *
 * Copyright (C) 2025 Dzmitry Sankouski <dsankouski@gmail.org>
 *
 * Battery charger driver for MAXIM 77705 charger/power-supply.
 */

#include <linux/devm-helpers.h>
#include <linux/i2c.h>
#include <linux/iio/consumer.h>
#include <linux/interrupt.h>
#include <linux/mfd/max77693-common.h>
#include <linux/mfd/max77705-private.h>
#include <linux/power/max77705_charger.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/timekeeping.h>

static const char *max77705_charger_model		= "max77705";
static const char *max77705_charger_manufacturer	= "Maxim Integrated";

/* Safety monitor (see struct max77705_charger_data) */
#define MAX77705_SAFETY_POLL_MS		10000	/* also kicks the charger watchdog */
#define MAX77705_TEMP_HYST_MC		2000	/* recovery 2 C inside the alert window */
#define MAX77705_TEMP_FAULT_LOW_MC	(-35000) /* open or shorted thermistor */
#define MAX77705_TEMP_FAULT_HIGH_MC	95000
#define MAX77705_CHG_DTLS_WDT_EXPIRED	0x0b
/* USB-C block (0x25): BC_STATUS, as the vendor MUIC driver reads it */
#define MAX77705_USBC_ADDR		0x25
#define MAX77705_USBC_REG_BC_STATUS	0x08
#define MAX77705_BC_CHGTYP_MASK		GENMASK(1, 0)
#define MAX77705_BC_CHGTYP_SDP		1
#define MAX77705_BC_CHGTYP_CDP		2
#define MAX77705_BC_CHGTYP_DCP		3
#define MAX77705_BC_VBUSDET		BIT(7)
#define MAX77705_SOC_END_DEFAULT	80	/* % of capacity, battery longevity */
#define MAX77705_SOC_START_DEFAULT	75
/*
 * Highest charge current on a positively detected DCP. Stock charges at
 * 2100 mA on a Samsung TA (cable-info default_charging_current); this stays
 * well below it.
 */
#define MAX77705_CC_DCP_CEIL_UA	1500000

static const struct regmap_config max77705_chg_regmap_config = {
	.reg_base = MAX77705_CHG_REG_BASE,
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = MAX77705_CHG_REG_SAFEOUT_CTRL,
};

static enum power_supply_property max77705_charger_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD,
	POWER_SUPPLY_PROP_USB_TYPE,
};

#define MAX77705_CHGIN_LIM_REG_MIN	3	/* 100 mA, the lowest input limit */
#define MAX77705_AICL_MAX_STEPS		128

static irqreturn_t max77705_aicl_irq(int irq, void *irq_drv_data)
{
	struct max77705_charger_data *chg = irq_drv_data;
	unsigned int regval, irq_status;
	int err, steps = 0;

	mutex_lock(&chg->lock);
	err = regmap_read(chg->regmap, MAX77705_CHG_REG_INT_OK, &irq_status);
	if (err < 0)
		goto out;

	// irq is fiered at the end of current decrease sequence too
	// early check AICL_I bit to guard against that excess irq call
	while (!(irq_status & BIT(MAX77705_AICL_I))) {
		err = regmap_field_read(chg->rfield[MAX77705_CHG_CHGIN_LIM], &regval);
		if (err < 0)
			goto out;

		/* never wrap below the minimum (0 - 1 would mask to 3.2 A) */
		if (regval <= MAX77705_CHGIN_LIM_REG_MIN ||
		    ++steps > MAX77705_AICL_MAX_STEPS)
			break;

		regval--;

		err = regmap_field_write(chg->rfield[MAX77705_CHG_CHGIN_LIM], regval);
		if (err < 0)
			goto out;

		msleep(AICL_WORK_DELAY_MS);

		err = regmap_read(chg->regmap, MAX77705_CHG_REG_INT_OK, &irq_status);
		if (err < 0)
			goto out;
	}
out:
	mutex_unlock(&chg->lock);
	return IRQ_HANDLED;
}

static irqreturn_t max77705_chgin_irq(int irq, void *irq_drv_data)
{
	struct max77705_charger_data *chg = irq_drv_data;

	queue_work(chg->wqueue, &chg->chgin_work);

	return IRQ_HANDLED;
}

static const struct regmap_irq max77705_charger_irqs[] = {
	REGMAP_IRQ_REG_LINE(MAX77705_BYP_I, BITS_PER_BYTE),
	REGMAP_IRQ_REG_LINE(MAX77705_INP_LIMIT_I, BITS_PER_BYTE),
	REGMAP_IRQ_REG_LINE(MAX77705_BATP_I, BITS_PER_BYTE),
	REGMAP_IRQ_REG_LINE(MAX77705_BAT_I, BITS_PER_BYTE),
	REGMAP_IRQ_REG_LINE(MAX77705_CHG_I, BITS_PER_BYTE),
	REGMAP_IRQ_REG_LINE(MAX77705_WCIN_I, BITS_PER_BYTE),
	REGMAP_IRQ_REG_LINE(MAX77705_CHGIN_I, BITS_PER_BYTE),
	REGMAP_IRQ_REG_LINE(MAX77705_AICL_I, BITS_PER_BYTE),
};

static const struct regmap_irq_chip max77705_charger_irq_chip = {
	.name			= "max77705-charger",
	.status_base		= MAX77705_CHG_REG_INT,
	.mask_base		= MAX77705_CHG_REG_INT_MASK,
	.num_regs		= 1,
	.irqs			= max77705_charger_irqs,
	.num_irqs		= ARRAY_SIZE(max77705_charger_irqs),
};

static int max77705_charger_enable(struct max77705_charger_data *chg)
{
	int rv;

	rv = regmap_field_write(chg->rfield[MAX77705_CHG_EN], 1);
	if (rv)
		dev_err(chg->dev, "unable to enable the charger: %d\n", rv);

	return rv;
}

/*
 * Charger off. The vendor driver switches charging off with CNFG_00 MODE
 * 0x05 -> 0x04 (buck on, charger off) and never clears CHG_EN; do both.
 */
static int max77705_charger_force_off(struct max77705_charger_data *chg)
{
	int rv, rv2;

	regmap_field_write(chg->rfield[MAX77705_CHGPROT], MAX77705_CHGPROT_UNLOCKED);
	rv = regmap_field_write(chg->rfield[MAX77705_MODE], MAX77705_BUCK_MASK);
	rv2 = regmap_field_write(chg->rfield[MAX77705_CHG_EN], MAX77705_CHG_DISABLE);

	return rv ? rv : rv2;
}

static void max77705_charger_disable(void *data)
{
	struct max77705_charger_data *chg = data;
	int rv;

	rv = max77705_charger_force_off(chg);
	if (rv)
		dev_err(chg->dev, "unable to disable the charger: %d\n", rv);
}

static int max77705_get_online(struct regmap *regmap, int *val)
{
	unsigned int data;
	int ret;

	ret = regmap_read(regmap, MAX77705_CHG_REG_INT_OK, &data);
	if (ret < 0)
		return ret;

	*val = !!(data & MAX77705_CHGIN_OK);

	return 0;
}

static unsigned int max77705_cc_ua_to_reg(unsigned int ua)
{
	return clamp_val(ua, MAX77705_CURRENT_CHGIN_MIN, MAX77705_CURRENT_CHGIN_MAX) /
		MAX77705_CURRENT_CHG_STEP;
}

static unsigned int max77705_chgin_ua_to_reg(unsigned int ua)
{
	/* read back as (reg + 1) * 25 mA, as in max77705_get_input_current() */
	return clamp_val(ua, MAX77705_CURRENT_CHGIN_MIN, MAX77705_CURRENT_CHGIN_MAX) /
		MAX77705_CURRENT_CHGIN_STEP - 1;
}

static int max77705_check_battery(struct max77705_charger_data *chg, int *val)
{
	unsigned int reg_data;
	unsigned int reg_data2;
	struct regmap *regmap = chg->regmap;

	regmap_read(regmap, MAX77705_CHG_REG_INT_OK, &reg_data);

	dev_dbg(chg->dev, "CHG_INT_OK(0x%x)\n", reg_data);

	regmap_read(regmap, MAX77705_CHG_REG_DETAILS_00, &reg_data2);

	dev_dbg(chg->dev, "CHG_DETAILS00(0x%x)\n", reg_data2);

	if ((reg_data & MAX77705_BATP_OK) || !(reg_data2 & MAX77705_BATP_DTLS))
		*val = true;
	else
		*val = false;

	return 0;
}

static int max77705_get_charge_type(struct max77705_charger_data *chg, int *val)
{
	struct regmap *regmap = chg->regmap;
	unsigned int reg_data, chg_en;
	int ret;

	ret = regmap_field_read(chg->rfield[MAX77705_CHG_EN], &chg_en);
	if (ret)
		return ret;
	if (!chg_en) {
		*val = POWER_SUPPLY_CHARGE_TYPE_NONE;
		return 0;
	}

	regmap_read(regmap, MAX77705_CHG_REG_DETAILS_01, &reg_data);
	reg_data &= MAX77705_CHG_DTLS;

	switch (reg_data) {
	case 0x0:
	case MAX77705_CHARGER_CONSTANT_CURRENT:
	case MAX77705_CHARGER_CONSTANT_VOLTAGE:
		*val = POWER_SUPPLY_CHARGE_TYPE_FAST;
		return 0;
	default:
		*val = POWER_SUPPLY_CHARGE_TYPE_NONE;
		return 0;
	}

	return 0;
}

static int max77705_get_status(struct max77705_charger_data *chg, int *val)
{
	struct regmap *regmap = chg->regmap;
	unsigned int reg_data, chg_en;
	int online = 0, ret;

	ret = regmap_field_read(chg->rfield[MAX77705_CHG_EN], &chg_en);
	if (ret)
		return ret;
	if (!chg_en || READ_ONCE(chg->block)) {
		max77705_get_online(regmap, &online);
		*val = online ? POWER_SUPPLY_STATUS_NOT_CHARGING :
				POWER_SUPPLY_STATUS_DISCHARGING;
		return 0;
	}

	regmap_read(regmap, MAX77705_CHG_REG_DETAILS_01, &reg_data);
	reg_data &= MAX77705_CHG_DTLS;

	switch (reg_data) {
	case 0x0:
	case MAX77705_CHARGER_CONSTANT_CURRENT:
	case MAX77705_CHARGER_CONSTANT_VOLTAGE:
		*val = POWER_SUPPLY_STATUS_CHARGING;
		return 0;
	case MAX77705_CHARGER_END_OF_CHARGE:
	case MAX77705_CHARGER_DONE:
		*val = POWER_SUPPLY_STATUS_FULL;
		return 0;
	/* those values hard coded as in vendor kernel, because of */
	/* failure to determine it's actual meaning. */
	case 0x05:
	case 0x06:
	case 0x07:
		*val = POWER_SUPPLY_STATUS_NOT_CHARGING;
		return 0;
	case 0x08:
	case 0xA:
	case 0xB:
		*val = POWER_SUPPLY_STATUS_DISCHARGING;
		return 0;
	default:
		*val = POWER_SUPPLY_STATUS_UNKNOWN;
		return 0;
	}

	return 0;
}

static int max77705_get_vbus_state(struct regmap *regmap, int *value)
{
	int ret;
	unsigned int charge_dtls;

	ret = regmap_read(regmap, MAX77705_CHG_REG_DETAILS_00, &charge_dtls);
	if (ret)
		return ret;

	charge_dtls = ((charge_dtls & MAX77705_CHGIN_DTLS) >>
			MAX77705_CHGIN_DTLS_SHIFT);

	switch (charge_dtls) {
	case 0x00:
		*value = POWER_SUPPLY_HEALTH_UNDERVOLTAGE;
		break;
	case 0x01:
		*value = POWER_SUPPLY_HEALTH_UNDERVOLTAGE;
		break;
	case 0x02:
		*value = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		break;
	case 0x03:
		*value = POWER_SUPPLY_HEALTH_GOOD;
		break;
	default:
		return 0;
	}
	return 0;
}

static int max77705_get_battery_health(struct max77705_charger_data *chg,
					int *value)
{
	struct regmap *regmap = chg->regmap;
	unsigned int bat_dtls;

	regmap_read(regmap, MAX77705_CHG_REG_DETAILS_01, &bat_dtls);
	bat_dtls = ((bat_dtls & MAX77705_BAT_DTLS) >> MAX77705_BAT_DTLS_SHIFT);

	switch (bat_dtls) {
	case MAX77705_BATTERY_NOBAT:
		dev_dbg(chg->dev, "%s: No battery and the chg is suspended\n",
			__func__);
		*value = POWER_SUPPLY_HEALTH_NO_BATTERY;
		break;
	case MAX77705_BATTERY_PREQUALIFICATION:
		dev_dbg(chg->dev, "%s: battery is okay but its voltage is low(~VPQLB)\n",
			__func__);
		break;
	case MAX77705_BATTERY_DEAD:
		dev_dbg(chg->dev, "%s: battery dead\n", __func__);
		*value = POWER_SUPPLY_HEALTH_DEAD;
		break;
	case MAX77705_BATTERY_GOOD:
	case MAX77705_BATTERY_LOWVOLTAGE:
		*value = POWER_SUPPLY_HEALTH_GOOD;
		break;
	case MAX77705_BATTERY_OVERVOLTAGE:
		dev_dbg(chg->dev, "%s: battery ovp\n", __func__);
		*value = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		break;
	default:
		dev_dbg(chg->dev, "%s: battery unknown\n", __func__);
		*value = POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
		break;
	}

	return 0;
}

static int max77705_get_health(struct max77705_charger_data *chg, int *val)
{
	struct regmap *regmap = chg->regmap;
	unsigned long block = READ_ONCE(chg->block);
	int ret, is_online = 0;

	if (block & BIT(MAX77705_BLOCK_HOT)) {
		*val = POWER_SUPPLY_HEALTH_OVERHEAT;
		return 0;
	}
	if (block & BIT(MAX77705_BLOCK_COLD)) {
		*val = POWER_SUPPLY_HEALTH_COLD;
		return 0;
	}
	if (block & BIT(MAX77705_BLOCK_TIMER)) {
		*val = POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE;
		return 0;
	}
	if (block & ~(BIT(MAX77705_BLOCK_USER) | BIT(MAX77705_BLOCK_SOC))) {
		*val = POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
		return 0;
	}

	ret = max77705_get_online(regmap, &is_online);
	if (ret)
		return ret;
	if (is_online) {
		ret = max77705_get_vbus_state(regmap, val);
		if (ret || (*val != POWER_SUPPLY_HEALTH_GOOD))
			return ret;
	}
	return max77705_get_battery_health(chg, val);
}

static int max77705_get_input_current(struct max77705_charger_data *chg,
					int *val)
{
	unsigned int reg_data;
	int get_current = 0;

	regmap_field_read(chg->rfield[MAX77705_CHG_CHGIN_LIM], &reg_data);

	if (reg_data <= 3)
		get_current = MAX77705_CURRENT_CHGIN_MIN;
	else
		get_current = (reg_data + 1) * MAX77705_CURRENT_CHGIN_STEP;

	*val = get_current;

	return 0;
}

static int max77705_get_charge_current(struct max77705_charger_data *chg,
					int *val)
{
	unsigned int reg_data;

	regmap_field_read(chg->rfield[MAX77705_CHG_CC_LIM], &reg_data);

	*val = reg_data <= 0x2 ? MAX77705_CURRENT_CHGIN_MIN : reg_data * MAX77705_CURRENT_CHG_STEP;

	return 0;
}

static unsigned int max77705_cv_uv_to_reg(int float_voltage)
{
	int float_voltage_mv = float_voltage / 1000;

	return float_voltage_mv <= 4000 ? 0x0 :
		float_voltage_mv >= 4500 ? 0x23 :
		(float_voltage_mv <= 4200) ? (float_voltage_mv - 4000) / 50 :
		(((float_voltage_mv - 4200) / 10) + 0x04);
}

/* write a field and read it back (the chip ACKs writes it ignores) */
static int max77705_write_verify(struct max77705_charger_data *chg,
				 enum max77705_field_idx fidx, unsigned int val)
{
	unsigned int rb;
	int err;

	err = regmap_field_write(chg->rfield[fidx], val);
	if (!err)
		err = regmap_field_read(chg->rfield[fidx], &rb);
	if (!err && rb != val)
		err = -EIO;
	return err;
}

static int max77705_get_float_voltage(struct max77705_charger_data *chg,
					int *val)
{
	unsigned int reg_data = 0;
	int voltage_mv;

	regmap_field_read(chg->rfield[MAX77705_CHG_CV_PRM], &reg_data);
	voltage_mv = reg_data <= 0x04 ? reg_data * 50 + 4000 :
					(reg_data - 4) * 10 + 4200;
	*val = voltage_mv * 1000;

	return 0;
}

static int max77705_chg_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	struct max77705_charger_data *chg = power_supply_get_drvdata(psy);
	struct regmap *regmap = chg->regmap;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		return max77705_get_online(regmap, &val->intval);
	case POWER_SUPPLY_PROP_PRESENT:
		return max77705_check_battery(chg, &val->intval);
	case POWER_SUPPLY_PROP_STATUS:
		return max77705_get_status(chg, &val->intval);
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		return max77705_get_charge_type(chg, &val->intval);
	case POWER_SUPPLY_PROP_HEALTH:
		return max77705_get_health(chg, &val->intval);
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		return max77705_get_input_current(chg, &val->intval);
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		return max77705_get_charge_current(chg, &val->intval);
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		return max77705_get_float_voltage(chg, &val->intval);
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		/* the supply is registered before the battery info is read */
		if (!READ_ONCE(chg->bat_info))
			return -ENODATA;
		val->intval = chg->bat_info->voltage_max_design_uv;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		/* battery thermistor, as read by the safety monitor */
		if (!chg->temp_chan || !READ_ONCE(chg->temp_valid))
			return -ENODATA;
		val->intval = READ_ONCE(chg->temp_mc) / 100;
		break;
	case POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR:
		val->intval = (READ_ONCE(chg->block) & BIT(MAX77705_BLOCK_USER)) ?
			POWER_SUPPLY_CHARGE_BEHAVIOUR_INHIBIT_CHARGE :
			POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO;
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD:
		if (!chg->temp_chan)
			return -ENODATA;
		val->intval = READ_ONCE(chg->soc_start);
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD:
		if (!chg->temp_chan)
			return -ENODATA;
		val->intval = READ_ONCE(chg->soc_end);
		break;
	case POWER_SUPPLY_PROP_USB_TYPE:
		val->intval = READ_ONCE(chg->usb_type);
		break;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = max77705_charger_model;
		break;
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = max77705_charger_manufacturer;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int max77705_set_property(struct power_supply *psy,
				 enum power_supply_property psp,
				 const union power_supply_propval *val)
{
	struct max77705_charger_data *chg = power_supply_get_drvdata(psy);
	unsigned int ua;
	int err = 0;

	if (psp == POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR) {
		/* only through the safety monitor, which can turn it back on */
		if (!chg->temp_chan)
			return -EOPNOTSUPP;
		if (val->intval != POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO &&
		    val->intval != POWER_SUPPLY_CHARGE_BEHAVIOUR_INHIBIT_CHARGE)
			return -EINVAL;
		mutex_lock(&chg->lock);
		if (val->intval == POWER_SUPPLY_CHARGE_BEHAVIOUR_INHIBIT_CHARGE)
			chg->user_inhibit = true;
		else
			chg->user_inhibit = false;
		if (chg->safety_ready)
			mod_delayed_work(system_wq, &chg->safety_work, 0);
		mutex_unlock(&chg->lock);
		return 0;
	}

	if (psp == POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD ||
	    psp == POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD) {
		if (!chg->temp_chan)
			return -EOPNOTSUPP;
		mutex_lock(&chg->lock);
		/* start < end, both within 0..100 */
		if (psp == POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD &&
		    val->intval >= 0 && val->intval < chg->soc_end)
			chg->soc_start = val->intval;
		else if (psp == POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD &&
			 val->intval > chg->soc_start && val->intval <= 100)
			chg->soc_end = val->intval;
		else
			err = -EINVAL;
		if (!err && chg->safety_ready)
			mod_delayed_work(system_wq, &chg->safety_work, 0);
		mutex_unlock(&chg->lock);
		return err;
	}

	if (val->intval < 0)
		return -EINVAL;
	ua = val->intval;

	mutex_lock(&chg->lock);
	switch (psp) {
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		/* never above the battery's constant-charge-current-max */
		if (chg->cc_max_ua)
			ua = min(ua, chg->cc_max_ua);
		err = regmap_field_write(chg->rfield[MAX77705_CHG_CC_LIM],
					 max77705_cc_ua_to_reg(ua));
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		/* never above input-current-limit-microamp */
		if (chg->chgin_max_ua)
			ua = min(ua, chg->chgin_max_ua);
		err = regmap_field_write(chg->rfield[MAX77705_CHG_CHGIN_LIM],
					 max77705_chgin_ua_to_reg(ua));
		break;
	default:
		err = -EINVAL;
	}
	mutex_unlock(&chg->lock);

	return err;
};

static int max77705_property_is_writeable(struct power_supply *psy,
					  enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
	case POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR:
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD:
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD:
		return true;
	default:
		return false;
	}
}

static const struct power_supply_desc max77705_charger_psy_desc = {
	.name = "max77705-charger",
	.type = POWER_SUPPLY_TYPE_USB,
	.properties = max77705_charger_props,
	.property_is_writeable = max77705_property_is_writeable,
	.num_properties = ARRAY_SIZE(max77705_charger_props),
	.get_property = max77705_chg_get_property,
	.set_property = max77705_set_property,
	.charge_behaviours = BIT(POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO) |
			     BIT(POWER_SUPPLY_CHARGE_BEHAVIOUR_INHIBIT_CHARGE),
	.usb_types = BIT(POWER_SUPPLY_USB_TYPE_UNKNOWN) |
		     BIT(POWER_SUPPLY_USB_TYPE_SDP) |
		     BIT(POWER_SUPPLY_USB_TYPE_CDP) |
		     BIT(POWER_SUPPLY_USB_TYPE_DCP),
};

static void max77705_chgin_isr_work(struct work_struct *work)
{
	struct max77705_charger_data *chg =
		container_of(work, struct max77705_charger_data, chgin_work);

	/* re-evaluate at once on plug / unplug (resets the safety timer) */
	mutex_lock(&chg->lock);
	if (chg->safety_ready)
		mod_delayed_work(system_wq, &chg->safety_work, 0);
	mutex_unlock(&chg->lock);

	power_supply_changed(chg->psy_chg);
}

static int max77705_charger_initialize(struct max77705_charger_data *chg)
{
	struct power_supply_battery_info *info;
	struct regmap *regmap = chg->regmap;
	int err;

	err = power_supply_get_battery_info(chg->psy_chg, &info);
	if (err)
		return dev_err_probe(chg->dev, err, "error on getting battery info");

	chg->bat_info = info;

	/* unlock charger setting protect */
	/* slowest LX slope */
	err = regmap_field_write(chg->rfield[MAX77705_CHGPROT], MAX77705_CHGPROT_UNLOCKED);
	if (err)
		goto err;

	/*
	 * With the safety monitor, no charging while the limits are being
	 * set: the monitor enables it once it has read the thermistor.
	 */
	if (chg->temp_chan) {
		err = max77705_charger_force_off(chg);
		if (err)
			goto err;
	}

	err = regmap_field_write(chg->rfield[MAX77705_LX_SLOPE], MAX77705_SLOWEST_LX_SLOPE);
	if (err)
		goto err;

	/* fast charge timer disable */
	/* restart threshold disable */
	/* pre-qual charge disable */
	err = regmap_field_write(chg->rfield[MAX77705_FCHGTIME], MAX77705_FCHGTIME_DISABLE);
	if (err)
		goto err;

	err = regmap_field_write(chg->rfield[MAX77705_CHG_RSTRT], MAX77705_CHG_RSTRT_DISABLE);
	if (err)
		goto err;

	err = regmap_field_write(chg->rfield[MAX77705_CHG_PQEN], MAX77705_CHG_PQEN_DISABLE);
	if (err)
		goto err;

	/* with the safety monitor, the charger stays off until it says so */
	err = regmap_field_write(chg->rfield[MAX77705_MODE], chg->temp_chan ?
			MAX77705_BUCK_MASK : MAX77705_CHG_MASK | MAX77705_BUCK_MASK);
	if (err)
		goto err;

	/* charge current 450mA(default) */
	/* otg current limit 900mA */
	err = regmap_field_write(chg->rfield[MAX77705_OTG_ILIM], MAX77705_OTG_ILIM_900);
	if (err)
		goto err;

	/* BAT to SYS OCP 4.80A */
	err = regmap_field_write(chg->rfield[MAX77705_REG_B2SOVRC], MAX77705_B2SOVRC_4_8A);
	if (err)
		goto err;

	/* top off current 150mA */
	/* top off timer 30min */
	err = regmap_field_write(chg->rfield[MAX77705_TO], MAX77705_TO_ITH_150MA);
	if (err)
		goto err;

	err = regmap_field_write(chg->rfield[MAX77705_TO_TIME], MAX77705_TO_TIME_30M);
	if (err)
		goto err;

	err = regmap_field_write(chg->rfield[MAX77705_SYS_TRACK], MAX77705_SYS_TRACK_DISABLE);
	if (err)
		goto err;

	/* cv voltage 4.2V or 4.35V */
	/* MINVSYS 3.6V(default) */
	if (info->constant_charge_voltage_max_uv > 0) {
		chg->cv_reg = max77705_cv_uv_to_reg(info->constant_charge_voltage_max_uv);
	} else if (info->voltage_max_design_uv < 0) {
		dev_warn(chg->dev, "missing battery:voltage-max-design-microvolt\n");
		chg->cv_reg = max77705_cv_uv_to_reg(4200000);
	} else {
		chg->cv_reg = max77705_cv_uv_to_reg(info->voltage_max_design_uv);
	}
	/*
	 * With the safety monitor, every limit is read back and must equal the
	 * target from DT; the monitor compares against these targets, never
	 * against whatever the register held.
	 */
	if (chg->temp_chan)
		err = max77705_write_verify(chg, MAX77705_CHG_CV_PRM, chg->cv_reg);
	else
		err = regmap_field_write(chg->rfield[MAX77705_CHG_CV_PRM], chg->cv_reg);
	if (err)
		goto err;

	/* fast charge current: the battery's constant-charge-current-max */
	if (info->constant_charge_current_max_ua > 0) {
		chg->cc_max_ua = info->constant_charge_current_max_ua;
		chg->cc_sdp_ua = chg->cc_max_ua;
		err = max77705_write_verify(chg, MAX77705_CHG_CC_LIM,
					    max77705_cc_ua_to_reg(chg->cc_max_ua));
		if (err)
			goto err;
	}

	/* input current limit of CHGIN */
	if (!device_property_read_u32(chg->dev, "input-current-limit-microamp",
				      &chg->chgin_max_ua) && chg->chgin_max_ua) {
		err = max77705_write_verify(chg, MAX77705_CHG_CHGIN_LIM,
					    max77705_chgin_ua_to_reg(chg->chgin_max_ua));
		if (err)
			goto err;
		chg->chgin_sdp_ua = chg->chgin_max_ua;
	}

	err = regmap_field_write(chg->rfield[MAX77705_VCHGIN], MAX77705_VCHGIN_4_5);
	if (err)
		goto err;

	err = regmap_field_write(chg->rfield[MAX77705_WCIN], MAX77705_WCIN_4_5);
	if (err)
		goto err;

	/*
	 * Watchdog timer. With the safety monitor it is on, as in the vendor
	 * driver, and kicked by every check: if the kernel stops, the charger
	 * stops charging (CHG_DTLS = watchdog expired) until it is kicked.
	 */
	if (chg->temp_chan) {
		err = regmap_update_bits(regmap, MAX77705_CHG_REG_CNFG_06,
					 MAX77705_WDTCLR_MASK, MAX77705_WDTCLR);
		if (!err)
			err = regmap_update_bits(regmap, MAX77705_CHG_REG_CNFG_00,
						 MAX77705_WDTEN_MASK, MAX77705_WDTEN_MASK);
		if (err)
			goto err;
	} else {
		regmap_update_bits(regmap, MAX77705_CHG_REG_CNFG_00,
				   MAX77705_WDTEN_MASK, 0);
	}

	/* VBYPSET=5.0V */
	err = regmap_field_write(chg->rfield[MAX77705_VBYPSET], 0);
	if (err)
		goto err;

	/* Switching Frequency : 1.5MHz */
	err = regmap_field_write(chg->rfield[MAX77705_REG_FSW], MAX77705_CHG_FSW_1_5MHz);
	if (err)
		goto err;

	/* Auto skip mode */
	err = regmap_field_write(chg->rfield[MAX77705_REG_DISKIP], MAX77705_AUTO_SKIP);
	if (err)
		goto err;

	return 0;

err:
	return dev_err_probe(chg->dev, err, "error while configuring");

}

/*
 * BC1.2 result of the USB-C block's firmware (BC_STATUS, read only, the
 * address is not claimed so the initramfs MUIC hook keeps using it).
 * Returns the input limit for this cable: the DCP value only when a DCP is
 * positively detected with VBUS present, the SDP value otherwise, read
 * errors included.
 */
static unsigned int max77705_cable_input_limit(struct max77705_charger_data *chg)
{
	union i2c_smbus_data data;
	int type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
	unsigned int ua = chg->chgin_sdp_ua;

	if (chg->usbc_adap &&
	    !i2c_smbus_xfer(chg->usbc_adap, MAX77705_USBC_ADDR, 0, I2C_SMBUS_READ,
			    MAX77705_USBC_REG_BC_STATUS, I2C_SMBUS_BYTE_DATA, &data) &&
	    (data.byte & MAX77705_BC_VBUSDET)) {
		switch (data.byte & MAX77705_BC_CHGTYP_MASK) {
		case MAX77705_BC_CHGTYP_SDP:
			type = POWER_SUPPLY_USB_TYPE_SDP;
			break;
		case MAX77705_BC_CHGTYP_CDP:
			type = POWER_SUPPLY_USB_TYPE_CDP;
			break;
		case MAX77705_BC_CHGTYP_DCP:
			type = POWER_SUPPLY_USB_TYPE_DCP;
			if (chg->chgin_dcp_ua)
				ua = chg->chgin_dcp_ua;
			break;
		}
	}
	WRITE_ONCE(chg->usb_type, type);
	return ua;
}

/*
 * Charge current for the cable just read by max77705_cable_input_limit():
 * the DCP value only on a positively detected DCP, the battery's
 * constant-charge-current-max otherwise.
 */
static unsigned int max77705_cable_charge_current(struct max77705_charger_data *chg,
						  bool online)
{
	if (online && chg->cc_dcp_ua &&
	    READ_ONCE(chg->usb_type) == POWER_SUPPLY_USB_TYPE_DCP)
		return chg->cc_dcp_ua;
	return chg->cc_sdp_ua;
}

/*
 * The limits in the registers must not be above the targets from DT. The
 * AICL handler only lowers the input limit, so a lower value is kept.
 * Returns 1 if fine, 0 if above a target, < 0 on a read error.
 */
static int max77705_safety_limits_ok(struct max77705_charger_data *chg)
{
	unsigned int cc, chgin, cv;

	if (regmap_field_read(chg->rfield[MAX77705_CHG_CC_LIM], &cc) ||
	    regmap_field_read(chg->rfield[MAX77705_CHG_CHGIN_LIM], &chgin) ||
	    regmap_field_read(chg->rfield[MAX77705_CHG_CV_PRM], &cv))
		return -EIO;

	return (!chg->cc_max_ua || cc <= max77705_cc_ua_to_reg(chg->cc_max_ua)) &&
	       (!chg->chgin_max_ua || chgin <= max77705_chgin_ua_to_reg(chg->chgin_max_ua)) &&
	       cv <= chg->cv_reg;
}

static void max77705_safety_restore_limits(struct max77705_charger_data *chg)
{
	chg->limit_rewrites++;
	dev_warn_ratelimited(chg->dev,
			     "charge limits above the configured ones, rewriting them (%lu so far)\n",
			     chg->limit_rewrites);

	regmap_field_write(chg->rfield[MAX77705_CHGPROT], MAX77705_CHGPROT_UNLOCKED);
	if (chg->cc_max_ua)
		regmap_field_write(chg->rfield[MAX77705_CHG_CC_LIM],
				   max77705_cc_ua_to_reg(chg->cc_max_ua));
	if (chg->chgin_max_ua)
		regmap_field_write(chg->rfield[MAX77705_CHG_CHGIN_LIM],
				   max77705_chgin_ua_to_reg(chg->chgin_max_ua));
	regmap_field_write(chg->rfield[MAX77705_CHG_CV_PRM], chg->cv_reg);
}

/* read MODE and CHG_EN back: 1 = charger on, 0 = off, < 0 = unknown or mixed */
static int max77705_safety_charger_state(struct max77705_charger_data *chg)
{
	unsigned int mode, en;

	if (regmap_field_read(chg->rfield[MAX77705_MODE], &mode) ||
	    regmap_field_read(chg->rfield[MAX77705_CHG_EN], &en))
		return -EIO;
	if (!(mode & MAX77705_CHG_MASK) && !en)
		return 0;
	if (mode == (MAX77705_CHG_MASK | MAX77705_BUCK_MASK) && en)
		return 1;
	return -EINVAL;
}

/*
 * One check, every MAX77705_SAFETY_POLL_MS. Order:
 *  1. a watchdog expiry switches the charger off before anything else;
 *  2. thermistor, input, safety timer; any read error blocks charging;
 *  3. limits re-checked (and rewritten) before charging may be on;
 *  4. charger switched off (MODE 0x04 + CHG_EN 0) or on (CHG_EN 1 + MODE 0x05)
 *     and read back;
 *  5. the watchdog is kicked only if the read-back state is safe: off, or
 *     on with the limits verified. Otherwise the watchdog runs out and the
 *     charger stops by itself.
 */
static void max77705_safety_check(struct max77705_charger_data *chg)
{
	struct power_supply_battery_info *info = chg->bat_info;
	unsigned long block, old_block;
	unsigned int dtls = 0, wden = 0;
	int online = 0, temp = 0, ret, state, lim;
	bool want_on, safe = false;
	u64 now, delta;

	mutex_lock(&chg->lock);
	block = old_block = chg->block;
	block &= ~(BIT(MAX77705_BLOCK_IO) | BIT(MAX77705_BLOCK_USER));
	if (chg->user_inhibit)
		block |= BIT(MAX77705_BLOCK_USER);

	/* 1. watchdog expired: off first, limits are checked below */
	ret = regmap_read(chg->regmap, MAX77705_CHG_REG_DETAILS_01, &dtls);
	if (ret) {
		block |= BIT(MAX77705_BLOCK_IO);
	} else if ((dtls & MAX77705_CHG_DTLS) == MAX77705_CHG_DTLS_WDT_EXPIRED) {
		if (!chg->wdt_expired_logged)
			dev_warn(chg->dev, "charger watchdog had expired, checking before a kick\n");
		chg->wdt_expired_logged = true;
		max77705_charger_force_off(chg);
	} else {
		chg->wdt_expired_logged = false;
	}

	/* 2a. battery temperature, with the hysteresis of the vendor tables */
	ret = iio_read_channel_processed(chg->temp_chan, &temp);
	if (ret < 0 || temp <= MAX77705_TEMP_FAULT_LOW_MC ||
	    temp >= MAX77705_TEMP_FAULT_HIGH_MC) {
		block |= BIT(MAX77705_BLOCK_SENSOR);
		WRITE_ONCE(chg->temp_valid, false);
	} else {
		block &= ~BIT(MAX77705_BLOCK_SENSOR);
		WRITE_ONCE(chg->temp_mc, temp);
		WRITE_ONCE(chg->temp_valid, true);

		if (info->temp_alert_min > INT_MIN) {
			int lo = info->temp_alert_min * 1000;

			if (temp < lo)
				block |= BIT(MAX77705_BLOCK_COLD);
			else if (temp >= lo + MAX77705_TEMP_HYST_MC)
				block &= ~BIT(MAX77705_BLOCK_COLD);
		}
		if (info->temp_alert_max < INT_MAX) {
			int hi = info->temp_alert_max * 1000;

			if (temp >= hi)
				block |= BIT(MAX77705_BLOCK_HOT);
			else if (temp <= hi - MAX77705_TEMP_HYST_MC)
				block &= ~BIT(MAX77705_BLOCK_HOT);
		}
	}

	/*
	 * 2b. capacity window (fuel gauge): stop at soc_end, resume below
	 * soc_start. No fuel gauge or a read error: no charging.
	 */
	if (!chg->fg_psy)
		chg->fg_psy = power_supply_get_by_name("max170xx_battery");
	if (chg->fg_psy) {
		union power_supply_propval pv;

		if (power_supply_get_property(chg->fg_psy, POWER_SUPPLY_PROP_CAPACITY, &pv) ||
		    pv.intval < 0 || pv.intval > 100) {
			block |= BIT(MAX77705_BLOCK_IO);
		} else {
			chg->soc = pv.intval;
			if (pv.intval >= chg->soc_end)
				block |= BIT(MAX77705_BLOCK_SOC);
			else if (pv.intval < chg->soc_start)
				block &= ~BIT(MAX77705_BLOCK_SOC);
		}
	} else {
		block |= BIT(MAX77705_BLOCK_IO);
	}

	/* 2c. safety timer: charging time since the input was plugged in */
	now = ktime_get_boottime_seconds();
	delta = now - chg->last_check_s;
	chg->last_check_s = now;

	state = max77705_safety_charger_state(chg);
	if (max77705_get_online(chg->regmap, &online) || state == -EIO) {
		/* unknown input: keep the timer as it is, and do not charge */
		block |= BIT(MAX77705_BLOCK_IO);
	} else if (!online) {
		chg->charge_time_s = 0;
		block &= ~BIT(MAX77705_BLOCK_TIMER);
	} else if (state == 1 && (dtls & MAX77705_CHG_DTLS) <= MAX77705_CHARGER_CONSTANT_VOLTAGE) {
		chg->charge_time_s += delta;
		if (chg->safety_timer_s && chg->charge_time_s >= chg->safety_timer_s)
			block |= BIT(MAX77705_BLOCK_TIMER);
	}

	/* watchdog left enabled (a reset could clear it) */
	if (!regmap_read(chg->regmap, MAX77705_CHG_REG_CNFG_00, &wden) &&
	    !(wden & MAX77705_WDTEN_MASK))
		regmap_update_bits(chg->regmap, MAX77705_CHG_REG_CNFG_00,
				   MAX77705_WDTEN_MASK, MAX77705_WDTEN_MASK);

	/*
	 * 2d. input limit for the attached cable. On a change (plug, unplug,
	 * other cable) the register is set to the new reference; otherwise only
	 * the "not above" check below applies, so a lower AICL value stays.
	 */
	if (chg->chgin_sdp_ua) {
		unsigned int ref = online ? max77705_cable_input_limit(chg) : chg->chgin_sdp_ua;

		if (!online)
			WRITE_ONCE(chg->usb_type, POWER_SUPPLY_USB_TYPE_UNKNOWN);
		if (ref != chg->chgin_max_ua) {
			dev_info(chg->dev, "input limit %u uA (USB type %d)\n", ref,
				 READ_ONCE(chg->usb_type));
			chg->chgin_max_ua = ref;
			if (regmap_field_write(chg->rfield[MAX77705_CHG_CHGIN_LIM],
					       max77705_chgin_ua_to_reg(ref)))
				block |= BIT(MAX77705_BLOCK_IO);
		}
	}

	/*
	 * 2e. charge current for the attached cable, read from the same
	 * BC_STATUS result. Written on a change only; the check below keeps
	 * the register at or under the reference.
	 */
	if (chg->cc_sdp_ua) {
		unsigned int ref = max77705_cable_charge_current(chg, online);

		if (ref != chg->cc_max_ua) {
			dev_info(chg->dev, "charge current %u uA (USB type %d)\n", ref,
				 READ_ONCE(chg->usb_type));
			chg->cc_max_ua = ref;
			if (regmap_field_write(chg->rfield[MAX77705_CHG_CC_LIM],
					       max77705_cc_ua_to_reg(ref)))
				block |= BIT(MAX77705_BLOCK_IO);
		}
	}

	/* 3. limits */
	lim = max77705_safety_limits_ok(chg);
	if (lim == 0) {
		max77705_charger_force_off(chg);
		max77705_safety_restore_limits(chg);
		lim = max77705_safety_limits_ok(chg);
	}
	if (lim == 1)
		block &= ~BIT(MAX77705_BLOCK_LIMITS);
	else
		block |= BIT(MAX77705_BLOCK_LIMITS);

	/* 4. apply, always written, then read back */
	want_on = !block;
	if (want_on) {
		ret = regmap_field_write(chg->rfield[MAX77705_CHG_EN], 1);
		if (!ret)
			ret = regmap_field_write(chg->rfield[MAX77705_MODE],
						 MAX77705_CHG_MASK | MAX77705_BUCK_MASK);
	} else {
		ret = max77705_charger_force_off(chg);
	}
	state = max77705_safety_charger_state(chg);

	/* 5. kick only from a verified safe state */
	if (!ret && state == 0)
		safe = true;
	else if (!ret && state == 1 && want_on && max77705_safety_limits_ok(chg) == 1)
		safe = true;

	if (safe) {
		ret = regmap_update_bits(chg->regmap, MAX77705_CHG_REG_CNFG_06,
					 MAX77705_WDTCLR_MASK, MAX77705_WDTCLR);
		if (ret)
			dev_err(chg->dev, "watchdog kick failed: %d\n", ret);
	} else {
		max77705_charger_force_off(chg);
		dev_crit_ratelimited(chg->dev,
				     "charger state not verified (want %s, state %d): watchdog not kicked\n",
				     want_on ? "on" : "off", state);
	}

	WRITE_ONCE(chg->block, block);
	if (block != old_block) {
		if (block)
			dev_warn(chg->dev,
				 "charging disabled: block 0x%lx, battery %d mC, %d %%, %llu s charging\n",
				 block, temp, chg->soc, chg->charge_time_s);
		else
			dev_info(chg->dev, "charging enabled, battery %d mC, %d %%\n",
				 temp, chg->soc);
	}

	if (chg->safety_ready)
		queue_delayed_work(system_wq, &chg->safety_work,
				   msecs_to_jiffies(MAX77705_SAFETY_POLL_MS));
	mutex_unlock(&chg->lock);

	if (block != old_block)
		power_supply_changed(chg->psy_chg);
}

static void max77705_safety_work(struct work_struct *work)
{
	struct max77705_charger_data *chg =
		container_of(to_delayed_work(work), struct max77705_charger_data,
			     safety_work);

	max77705_safety_check(chg);
}

static void max77705_safety_stop(void *data)
{
	struct max77705_charger_data *chg = data;

	/* no re-queue after this, from the work itself or from chgin_work */
	mutex_lock(&chg->lock);
	chg->safety_ready = false;
	mutex_unlock(&chg->lock);
	cancel_delayed_work_sync(&chg->safety_work);
	/* chgin_work only re-queues while safety_ready; let a running one end */
	flush_work(&chg->chgin_work);
	if (chg->fg_psy) {
		power_supply_put(chg->fg_psy);
		chg->fg_psy = NULL;
	}
}

static int max77705_charger_probe(struct i2c_client *i2c)
{
	struct power_supply_config pscfg = {};
	struct max77705_charger_data *chg;
	struct regmap_irq_chip *chip_desc;
	struct device *dev;
	struct regmap_irq_chip_data *irq_data;
	int ret;

	dev = &i2c->dev;

	chg = devm_kzalloc(dev, sizeof(*chg), GFP_KERNEL);
	if (!chg)
		return -ENOMEM;

	chg->dev = dev;
	i2c_set_clientdata(i2c, chg);

	chip_desc = devm_kmemdup(dev, &max77705_charger_irq_chip,
				 sizeof(max77705_charger_irq_chip),
				 GFP_KERNEL);
	if (!chip_desc)
		return -ENOMEM;
	chip_desc->irq_drv_data = chg;

	chg->regmap = devm_regmap_init_i2c(i2c, &max77705_chg_regmap_config);
	if (IS_ERR(chg->regmap))
		return PTR_ERR(chg->regmap);

	for (int i = 0; i < MAX77705_N_REGMAP_FIELDS; i++) {
		chg->rfield[i] = devm_regmap_field_alloc(dev, chg->regmap,
							 max77705_reg_field[i]);
		if (IS_ERR(chg->rfield[i]))
			return dev_err_probe(dev, PTR_ERR(chg->rfield[i]),
					     "cannot allocate regmap field\n");
	}

	ret = devm_mutex_init(dev, &chg->lock);
	if (ret)
		return ret;
	INIT_DELAYED_WORK(&chg->safety_work, max77705_safety_work);

	chg->temp_chan = devm_iio_channel_get(dev, "battery-temp");
	if (IS_ERR(chg->temp_chan)) {
		ret = PTR_ERR(chg->temp_chan);
		if (ret != -ENODEV)
			return dev_err_probe(dev, ret, "cannot get the battery-temp channel\n");
		chg->temp_chan = NULL;	/* no safety monitor */
	}
	if (chg->temp_chan)
		device_property_read_u32(dev, "maxim,safety-timer-sec", &chg->safety_timer_s);
	if (chg->temp_chan &&
	    !device_property_read_u32(dev, "maxim,dcp-input-current-limit-microamp",
				      &chg->chgin_dcp_ua))
		chg->usbc_adap = i2c->adapter;
	if (chg->usbc_adap &&
	    !device_property_read_u32(dev, "maxim,dcp-constant-charge-current-microamp",
				      &chg->cc_dcp_ua) &&
	    chg->cc_dcp_ua > MAX77705_CC_DCP_CEIL_UA) {
		dev_warn(dev, "DCP charge current %u uA above %u uA, not used\n",
			 chg->cc_dcp_ua, MAX77705_CC_DCP_CEIL_UA);
		chg->cc_dcp_ua = 0;
	}
	chg->usb_type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
	chg->soc_end = MAX77705_SOC_END_DEFAULT;
	chg->soc_start = MAX77705_SOC_START_DEFAULT;
	chg->soc = -1;

	pscfg.fwnode = dev_fwnode(dev);
	pscfg.drv_data = chg;

	chg->psy_chg = devm_power_supply_register(dev, &max77705_charger_psy_desc, &pscfg);
	if (IS_ERR(chg->psy_chg))
		return PTR_ERR(chg->psy_chg);

	ret = devm_regmap_add_irq_chip(chg->dev, chg->regmap, i2c->irq,
					IRQF_ONESHOT, 0,
					chip_desc, &irq_data);
	if (ret)
		return dev_err_probe(dev, ret, "failed to add irq chip\n");

	chg->wqueue = devm_alloc_ordered_workqueue(dev, "%s", 0, dev_name(dev));
	if (!chg->wqueue)
		return -ENOMEM;

	ret = devm_work_autocancel(dev, &chg->chgin_work, max77705_chgin_isr_work);
	if (ret)
		return dev_err_probe(dev, ret, "failed to initialize interrupt work\n");

	ret = max77705_charger_initialize(chg);
	if (ret)
		return dev_err_probe(dev, ret, "failed to initialize charger IC\n");

	ret = devm_request_threaded_irq(dev, regmap_irq_get_virq(irq_data, MAX77705_CHGIN_I),
					NULL, max77705_chgin_irq,
					IRQF_TRIGGER_NONE,
					"chgin-irq", chg);
	if (ret)
		return ret;

	ret = devm_request_threaded_irq(dev, regmap_irq_get_virq(irq_data, MAX77705_AICL_I),
					NULL, max77705_aicl_irq,
					IRQF_TRIGGER_NONE,
					"aicl-irq", chg);
	if (ret)
		return ret;

	if (!chg->temp_chan) {
		ret = max77705_charger_enable(chg);
		if (ret)
			return dev_err_probe(dev, ret, "failed to enable charge\n");
	}

	ret = devm_add_action_or_reset(dev, max77705_charger_disable, chg);
	if (ret)
		return ret;

	if (!chg->temp_chan)
		return 0;

	/* runs before the disable action on unbind */
	ret = devm_add_action_or_reset(dev, max77705_safety_stop, chg);
	if (ret)
		return ret;

	chg->last_check_s = ktime_get_boottime_seconds();
	mutex_lock(&chg->lock);
	chg->safety_ready = true;
	mutex_unlock(&chg->lock);
	/* first check now: enables charging if everything is in range */
	mod_delayed_work(system_wq, &chg->safety_work, 0);

	dev_info(dev, "safety monitor: CC %u uA (DCP %u uA), CHGIN %u uA (DCP %u uA), CV reg 0x%x, battery %d..%d C, timer %u s, capacity %d..%d %%\n",
		 chg->cc_sdp_ua, chg->cc_dcp_ua, chg->chgin_sdp_ua, chg->chgin_dcp_ua, chg->cv_reg,
		 chg->bat_info->temp_alert_min, chg->bat_info->temp_alert_max,
		 chg->safety_timer_s, chg->soc_start, chg->soc_end);

	return 0;
}

static const struct of_device_id max77705_charger_of_match[] = {
	{ .compatible = "maxim,max77705-charger" },
	{ }
};
MODULE_DEVICE_TABLE(of, max77705_charger_of_match);

static struct i2c_driver max77705_charger_driver = {
	.driver = {
		.name = "max77705-charger",
		.of_match_table = max77705_charger_of_match,
	},
	.probe = max77705_charger_probe,
};
module_i2c_driver(max77705_charger_driver);

MODULE_AUTHOR("Dzmitry Sankouski <dsankouski@gmail.com>");
MODULE_DESCRIPTION("Maxim MAX77705 charger driver");
MODULE_LICENSE("GPL");
