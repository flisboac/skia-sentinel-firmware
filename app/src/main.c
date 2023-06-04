/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor/ens160.h>

/* 1000 msec = 1 sec */
#define CYCLE_TIME_MS 1000
#define POLL_CYCLES_COUNT 5
#define LED0_NODE DT_ALIAS(uart_bridge_led0)
// #define LED0_NODE DT_ALIAS(led0)

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
// static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

// const struct device* const dev_th = DEVICE_DT_GET_ONE(DT_ALIAS(sensor_th));
static bool th_ready;
static bool gpio_ready;
static struct sensor_value curr_temp = {0}, curr_hum = {0};

static void i2c_scan();
static void measure_th_(const struct device* const dev);
static void measure_thp_(const struct device* const dev);
static void measure_voc_eco2_(const struct device* const dev);

int main(void)
{
    int ret;
    int cycle = 0;
    gpio_ready = false;

    printk("initializing...\n");

    // i2c_scan();

    const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
    const struct device* const dev_voc_eco2 = DEVICE_DT_GET_ONE(sciosense_ens160);
    const struct device* const dev_th = DEVICE_DT_GET_ONE(sensirion_sht3xd);
    const struct device* const dev_thp = DEVICE_DT_GET_ONE(bosch_bme680);
    int rc;

    k_msleep(CYCLE_TIME_MS * POLL_CYCLES_COUNT);

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
	} else {
        printk("Configuring GPIO device <%s %d>...\n", led.port->name, led.pin);
        ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
        if (ret < 0) {
            printk("failed to configure led pin!\n");
        } else {
            gpio_ready = true;
        }
    }

    k_msleep(CYCLE_TIME_MS * POLL_CYCLES_COUNT);

    while (true) {
        if (cycle % POLL_CYCLES_COUNT == 0) {
            printk("\n");
            th_ready = false;
            measure_th_(dev_th);
            measure_thp_(dev_thp);
            measure_voc_eco2_(dev_voc_eco2);
            printk("\n");
        }

        cycle++;

        if (gpio_ready) {
            ret = gpio_pin_toggle_dt(&led);
            if (ret) {
                printk("failed to toggle led! Error code = %d\n", ret);
                return 0;
            }
        }

        printk(" * ");
        k_msleep(CYCLE_TIME_MS);
    }

    return 0;
}

#define I2C_NODE DT_ALIAS(i2c0)
#define I2C_DEV	DT_LABEL(I2C_NODE)

void i2c_scan() {
    const struct device *i2c_dev = DEVICE_DT_GET(DT_ALIAS(i2c0));

    if (!device_is_ready(i2c_dev)) {
		printk("I2C: Device driver not found.\n");
		return;
	}

	printk("\n    | 0x00 0x01 0x02 0x03 0x04 0x05 0x06 0x07 0x08 0x09 0x0a 0x0b 0x0c 0x0d 0x0e 0x0f |\n");
	printk(  "----|---------------------------------------------------------------------------------");

	uint8_t error = 0u;
	uint8_t dst;
	uint8_t i2c_dev_cnt = 0;
	struct i2c_msg msgs[1];
	msgs[0].buf = &dst;
	msgs[0].len = 1U;
	msgs[0].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	for (uint16_t x = 0; x <= 0x7f; x++) {
		if (x % 0x10 == 0) {
			printk("|\n0x%02x| ",x);
		}

		if (x >= 1 && x <= 0x7f) {
			error = i2c_transfer(i2c_dev, &msgs[0], 1, x);
            if (error == 0) {
                printk("0x%02x ",x);
                i2c_dev_cnt++;
            } else {
                printk(" --  ");
            }
		} else {
			printk("     ");
		}

        k_msleep(50);
	}
    printk("|\n");
	printk("\nI2C device(s) found on the bus: %d\nScanning done.\n\n", i2c_dev_cnt);
	printk("Find the registered I2C address on: https://i2cdevices.org/addresses\n\n");
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
