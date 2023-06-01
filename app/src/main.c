/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor/ens160.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS 5000
#define LED0_NODE DT_ALIAS(uart_bridge_led0)
// #define LED0_NODE DT_ALIAS(led0)

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
// static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

// const struct device* const dev_th = DEVICE_DT_GET_ONE(DT_ALIAS(sensor_th));
static bool th_ready;
static struct sensor_value curr_temp = {0}, curr_hum = {0};

static void measure_th_(const struct device* const dev);
static void measure_thp_(const struct device* const dev);
static void measure_voc_eco2_(const struct device* const dev);

int main(void)
{
    int ret;

    printk("initializing...\n");

    const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
    const struct device* const dev_voc_eco2 = DEVICE_DT_GET_ONE(sciosense_ens160);
    const struct device* const dev_th = DEVICE_DT_GET_ONE(sensirion_sht3xd);
    const struct device* const dev_thp = DEVICE_DT_GET_ONE(bosch_bme680);
    int rc;

    k_msleep(SLEEP_TIME_MS);

    printf("Is device %s ready?\n", dev_th->name);
    if (!device_is_ready(dev_th)) {
        printf("Device %s is not ready\n", dev_th->name);
        return 0;
    }

    printf("Is device %s ready?\n", dev_thp->name);
    if (!device_is_ready(dev_thp)) {
        printf("Device %s is not ready\n", dev_thp->name);
        return 0;
    }

    printk("Is device %s ready?\n", dev_voc_eco2->name);
    if (!device_is_ready(dev_voc_eco2)) {
        printf("Device %s is not ready\n", dev_voc_eco2->name);
        return 0;
    }

    printk("Is GPIO device <%s %d> ready?\n", led.port->name, led.pin);
	if (!gpio_is_ready_dt(&led)) {
        printk("failed to initialize led device!\n");
		return 0;
	}

    printk("Configuring GPIO device <%s %d>...\n", led.port->name, led.pin);
	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
        printk("failed to configure led pin!\n");
		return 0;
	}

    k_msleep(SLEEP_TIME_MS);

    while (true) {
        th_ready = false;
        measure_th_(dev_th);
        measure_thp_(dev_thp);
        measure_voc_eco2_(dev_voc_eco2);

		ret = gpio_pin_toggle_dt(&led);
		if (ret < 0) {
            printk("failed to toggle led!\n");
			return 0;
		}

        k_msleep(SLEEP_TIME_MS);
        printk("sleeping...\n");
    }

    return 0;
}

void measure_th_(const struct device* const dev)
{
    int rc;
    struct sensor_value temp, hum;

    rc = sensor_sample_fetch(dev);
    if (rc == 0) { rc = sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temp); }
    if (rc == 0) { rc = sensor_channel_get(dev, SENSOR_CHAN_HUMIDITY, &hum); }
    if (rc != 0) {
        printf("Failed to get measurements from sensor %s!\n", dev->name);
        return;
    }

    printf("SHT3XD: T = %.2f C ; RH = %0.2f %%\n", sensor_value_to_double(&temp), sensor_value_to_double(&hum));
}

void measure_thp_(const struct device* const dev)
{
    int rc;
    struct sensor_value temp, press, humidity, gas_res;

    rc = sensor_sample_fetch(dev);
    if (rc == 0) rc = sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);
    if (rc == 0) rc = sensor_channel_get(dev, SENSOR_CHAN_PRESS, &press);
    if (rc == 0) rc = sensor_channel_get(dev, SENSOR_CHAN_HUMIDITY, &humidity);
    if (rc == 0) rc = sensor_channel_get(dev, SENSOR_CHAN_GAS_RES, &gas_res);

    if (rc == 0) {
        th_ready = true;
        curr_temp = temp;
        curr_hum = humidity;
        printf(
            "BMP680: T = %d.%06d C; P = %d.%06d kPa; H = %d.%06d %%; G = %d.%06d R\n",
            temp.val1,
            temp.val2,
            press.val1,
            press.val2,
            humidity.val1,
            humidity.val2,
            gas_res.val1,
            gas_res.val2
        );
    } else {
        printf("Failed to get measurements from sensor %s!\n", dev->name);
    }
}

void measure_voc_eco2_(const struct device* const dev)
{
    int rc;
    struct sensor_value tvoc, eco2, aqi_uba, part_id, fw_major, fw_minor, fw_build, opmode, status;
    struct sensor_value hp0_bl, hp1_bl, hp2_bl, hp3_bl, hp0_rs, hp1_rs, hp2_rs, hp3_rs;

    if (th_ready) {
        rc = sensor_attr_set(dev, SENSOR_CHAN_AMBIENT_TEMP, 0, &curr_temp);
        if (rc == 0) sensor_attr_set(dev, SENSOR_CHAN_HUMIDITY, 0, &curr_hum);

        if (rc != 0) {
            printf("ENS160: Failed to setup temperature and humidity! Code: %d\n", rc);
        }
    }

    rc = sensor_sample_fetch(dev);
    if (rc == 0) rc = sensor_channel_get(dev, SENSOR_CHAN_ENS160_REG_STATUS, &status);
    if (rc == 0) rc = sensor_channel_get(dev, SENSOR_CHAN_VOC, &tvoc);
    if (rc == 0) rc = sensor_channel_get(dev, SENSOR_CHAN_CO2, &eco2);
    if (rc == 0) rc = sensor_channel_get(dev, SENSOR_CHAN_ENS160_AQI_UBA, &aqi_uba);
    if (rc == 0) rc = sensor_channel_get(dev, SENSOR_CHAN_ENS160_PART_ID, &part_id);
    if (rc == 0) rc = sensor_channel_get(dev, SENSOR_CHAN_ENS160_REVISION_MAJOR, &fw_major);
    if (rc == 0) rc = sensor_channel_get(dev, SENSOR_CHAN_ENS160_REVISION_MINOR, &fw_minor);
    if (rc == 0) rc = sensor_channel_get(dev, SENSOR_CHAN_ENS160_REVISION_BUILD, &fw_build);
    if (rc == 0) rc = sensor_channel_get(dev, SENSOR_CHAN_ENS160_OPMODE, &opmode);
    if (rc == 0) rc = sensor_channel_get(dev, SENSOR_CHAN_ENS160_HOTPLATE0_BASELINE, &hp0_bl);
    if (rc == 0) rc = sensor_channel_get(dev, SENSOR_CHAN_ENS160_HOTPLATE1_BASELINE, &hp1_bl);
    if (rc == 0) rc = sensor_channel_get(dev, SENSOR_CHAN_ENS160_HOTPLATE2_BASELINE, &hp2_bl);
    if (rc == 0) rc = sensor_channel_get(dev, SENSOR_CHAN_ENS160_HOTPLATE3_BASELINE, &hp3_bl);
    if (rc == 0) rc = sensor_channel_get(dev, SENSOR_CHAN_ENS160_HOTPLATE0_RESISTANCE, &hp0_rs);
    if (rc == 0) rc = sensor_channel_get(dev, SENSOR_CHAN_ENS160_HOTPLATE1_RESISTANCE, &hp1_rs);
    if (rc == 0) rc = sensor_channel_get(dev, SENSOR_CHAN_ENS160_HOTPLATE2_RESISTANCE, &hp2_rs);
    if (rc == 0) rc = sensor_channel_get(dev, SENSOR_CHAN_ENS160_HOTPLATE3_RESISTANCE, &hp3_rs);

    if (rc == 0) {
        // printf(
        //     "ENS160: sensor '%s': status = %d, part_id = %d, major = %d, minor = %d, build = %d, opmode = %d\n",
        //     dev->name,
        //     status.val1,
        //     part_id.val1,
        //     fw_major.val1,
        //     fw_minor.val1,
        //     fw_build.val1,
        //     opmode.val1
        // );

        printf(
            "ENS160: status: %d, TVOC = %d.%06d ppb; eCO2 = %d.%06d ppm; AQI-UBA: %d\n",
            status.val1,
            tvoc.val1,
            tvoc.val2,
            eco2.val1,
            eco2.val2,
            aqi_uba.val1
        );
        printf(
            "ENS160: HP0_BL = %d; HP1_BL = %d; HP2_BL = %d; HP3_BL = %d; \n",
            hp0_bl.val1,
            hp1_bl.val1,
            hp2_bl.val1,
            hp3_bl.val1
        );
        printf(
            "ENS160: HP0_RS = %d; HP1_RS = %d; HP2_RS = %d; HP3_RS = %d; \n",
            hp0_rs.val1,
            hp1_rs.val1,
            hp2_rs.val1,
            hp3_rs.val1
        );
    } else {
        sensor_channel_get(dev, SENSOR_CHAN_ENS160_REG_STATUS, &status);
        sensor_channel_get(dev, SENSOR_CHAN_ENS160_OPMODE, &opmode);
        printf("Failed to get measurements from sensor %s, error code %d, opmode = %d, status = %d!\n", dev->name, rc, opmode.val1, status.val1);
    }
}
