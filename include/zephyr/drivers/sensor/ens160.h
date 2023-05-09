#ifndef ZEPHYR_INCLUDE_DRIVERS_SENSOR_ENS160_H_
#define ZEPHYR_INCLUDE_DRIVERS_SENSOR_ENS160_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/drivers/sensor.h>

#define SENSOR_VALUE_ENS160_OPMODE_V(opmode) { .val1 = opmode, .val2 = 0 }
#define SENSOR_VALUE_ENS160_SET_OPMODE(val, opmode) do { (val)->val1 = opmode; (val)->val2 = 0; } while (0)
#define SENSOR_VALUE_ENS160_TO_OPMODE(val) ((enum sensor_value_ens160_opmode)((val)->val1))

enum sensor_value_ens160_opmode {
    SENSOR_ENS160_OPMODE_UNKNOWN = 0x1FF,
    SENSOR_ENS160_OPMODE_DEEP_SLEEP = 0x00,
    SENSOR_ENS160_OPMODE_IDLE = 0x01,
    SENSOR_ENS160_OPMODE_STANDARD = 0x02,
    SENSOR_ENS160_OPMODE_LP = 0x03,
    SENSOR_ENS160_OPMODE_CUSTOM = 0xC0,
    SENSOR_ENS160_OPMODE_RESET = 0xF0
};

#define SENSOR_ENS160_STATUS_OPMODE_RUNNING 0x80
#define SENSOR_ENS160_STATUS_OPMODE_ERROR 0x40
#define SENSOR_ENS160_STATUS_VALIDITY 0x0C
#define SENSOR_ENS160_STATUS_NEW_DATA 0x02
#define SENSOR_ENS160_STATUS_NEW_GPR 0x01

enum sensor_channel_ens160 {
    /** Part ID. */
    SENSOR_CHAN_ENS160_PART_ID = SENSOR_CHAN_PRIV_START,
    /** Major revision. */
    SENSOR_CHAN_ENS160_REVISION_MAJOR,
    /** Minor revision. */
    SENSOR_CHAN_ENS160_REVISION_MINOR,
    /** Build revision. */
    SENSOR_CHAN_ENS160_REVISION_BUILD,

    /** GE-UPA (German Environmental Agency, Umweltbundesamt) Air quality index (1-5). */
	SENSOR_CHAN_ENS160_AQI_UBA,
    /** US-EPA (U.S. Environmental Protection Agency) Air quality index (0-500). */
	SENSOR_CHAN_ENS160_AQI_EPA,

    /** Sensor's current operational status. */
	SENSOR_CHAN_ENS160_REG_STATUS,

    /** Configures/sets the current operational mode. */
    SENSOR_CHAN_ENS160_OPMODE,
    /** Used to configure custom mode. */
    SENSOR_CHAN_ENS160_OPMODE_CUSTOM,

    /** Hotplate 0 Baseline. */
    SENSOR_CHAN_ENS160_HOTPLATE0_BASELINE,
    /** Hotplate 1 Baseline. */
    SENSOR_CHAN_ENS160_HOTPLATE1_BASELINE,
    /** Hotplate 2 Baseline. */
    SENSOR_CHAN_ENS160_HOTPLATE2_BASELINE,
    /** Hotplate 3 Baseline. */
    SENSOR_CHAN_ENS160_HOTPLATE3_BASELINE,

    /** Hotplate 0 Resistance. */
    SENSOR_CHAN_ENS160_HOTPLATE0_RESISTANCE,
    /** Hotplate 1 Resistance. */
    SENSOR_CHAN_ENS160_HOTPLATE1_RESISTANCE,
    /** Hotplate 2 Resistance. */
    SENSOR_CHAN_ENS160_HOTPLATE2_RESISTANCE,
    /** Hotplate 3 Resistance. */
    SENSOR_CHAN_ENS160_HOTPLATE3_RESISTANCE,
};

enum sensor_attribute_ens160 {
    /** Indicates how many steps there are in the custom mode. */
    SENSOR_ATTR_ENS160_CUSTOM_STEP_COUNT = SENSOR_ATTR_PRIV_START,

    /** Indicates how much time the current step will last. */
    SENSOR_ATTR_ENS160_CUSTOM_STEP_TIME,

    /** If true, sets up the resistance associated with hotplate 0 to be measured in the current step. */
    SENSOR_ATTR_ENS160_CUSTOM_HOTPLATE0_MEASURING,
    /** If true, sets up the resistance associated with hotplate 1 to be measured in the current step. */
    SENSOR_ATTR_ENS160_CUSTOM_HOTPLATE1_MEASURING,
    /** If true, sets up the resistance associated with hotplate 2 to be measured in the current step. */
    SENSOR_ATTR_ENS160_CUSTOM_HOTPLATE2_MEASURING,
    /** If true, sets up the resistance associated with hotplate 3 to be measured in the current step. */
    SENSOR_ATTR_ENS160_CUSTOM_HOTPLATE3_MEASURING,

    /** Hotplate 0 Temperature. */
    SENSOR_ATTR_ENS160_CUSTOM_HOTPLATE0_TEMPERATURE,
    /** Hotplate 1 Temperature. */
    SENSOR_ATTR_ENS160_CUSTOM_HOTPLATE1_TEMPERATURE,
    /** Hotplate 2 Temperature. */
    SENSOR_ATTR_ENS160_CUSTOM_HOTPLATE2_TEMPERATURE,
    /** Hotplate 3 Temperature. */
    SENSOR_ATTR_ENS160_CUSTOM_HOTPLATE3_TEMPERATURE,

    /** Commits the current step configuration (e.g. sends the custom step to the device), and sets up the next step. */
    SENSOR_ATTR_ENS160_CUSTOM_STEP_NEXT,
};

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_SENSOR_ENS160_H_ */
