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

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS 2000

/* The devicetree node identifier for the "led0" alias. */
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

    const struct device* const dev_th = DEVICE_DT_GET_ONE(sensirion_sht3xd);
    const struct device* const dev_thp = DEVICE_DT_GET_ONE(bosch_bme680);
    int rc;

    if (!device_is_ready(dev_th)) {
        printf("Device %s is not ready\n", dev_th->name);
        return 0;
    }

    if (!device_is_ready(dev_thp)) {
        printf("Device %s is not ready\n", dev_thp->name);
        return 0;
    }

    while (true) {
        th_ready = false;
        measure_th_(dev_th);
        measure_thp_(dev_thp);
        measure_voc_eco2_(dev_thp);
        k_msleep(SLEEP_TIME_MS);
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
        printf("SHT3XD: failed: %d\n", rc);
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
        printf("Failed to get measurements from sensor %s!", dev->name);
    }
}

void measure_voc_eco2_(const struct device* const dev)
{
}
