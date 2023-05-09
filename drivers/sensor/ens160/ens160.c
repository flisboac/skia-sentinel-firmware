/* ens160.c - Driver for Sciosense's ENS160 TVOC, eCO2 and AQI-UBA air
 * quality sensor.
 *
 * https://www.sciosense.com/products/environmental-sensors/ens160-digital-multi-gas-sensor/
 */

#include "ens160.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ens160, CONFIG_SENSOR_LOG_LEVEL);

static inline void ens160_default_wait()
{
    k_sleep(K_MSEC(100));
}

#if ENS160_BUS_SPI
static inline bool ens160_is_on_spi(const struct device* dev)
{
    const struct ens160_config* config = dev->config;

    return config->bus_io == &ens160_bus_io_spi;
}
#endif

static inline int ens160_bus_check(const struct device* dev)
{
    const struct ens160_config* config = dev->config;

    return config->bus_io->check(&config->bus);
}

static inline int ens160_reg_read(const struct device* dev, uint8_t start, uint8_t* buf, int size)
{
    const struct ens160_config* config = dev->config;

    return config->bus_io->read(dev, start, buf, size);
}

static inline int ens160_reg_write(const struct device* dev, uint8_t reg, uint8_t val)
{
    const struct ens160_config* config = dev->config;

    return config->bus_io->write(dev, reg, val);
}

static inline int ens160_reg_write_buf(const struct device* dev, uint8_t reg, const uint8_t* buf, int size)
{
    const struct ens160_config* config = dev->config;

    return config->bus_io->write_buf(dev, reg, buf, size);
}

static inline bool ens160_is_valid_measurement_opmode(const struct device* dev)
{
    struct ens160_data* data = dev->data;
    return (
        data->opmode == SENSOR_ENS160_OPMODE_STANDARD || data->opmode == SENSOR_ENS160_OPMODE_LP
        || data->opmode == SENSOR_ENS160_OPMODE_CUSTOM
    );
}

static int ens160_clear_command(const struct device* dev)
{
    int err;

    err = ens160_reg_write(dev, ENS160_REG_COMMAND, ENS160_COMMAND_NOP);
    if (err) { return err; }

    err = ens160_reg_write(dev, ENS160_REG_COMMAND, ENS160_COMMAND_CLRGPR);
    if (err) { return err; }

    ens160_default_wait();

    return 0;
}

static int ens160_get_opmode(const struct device* dev)
{
    struct ens160_data* data = dev->data;
    int err;
    uint8_t buf;
    err = ens160_reg_read(dev, ENS160_REG_OPMODE, &buf, 1);
    if (err) { return err; }
    data->opmode = buf;
    return 0;
}

static int ens160_set_opmode(const struct device* dev, enum sensor_value_ens160_opmode opmode)
{
    struct ens160_data* data = dev->data;
    int err;

    if ((opmode == SENSOR_ENS160_OPMODE_LP) && (data->rev_ens16x == 0)) {
        LOG_DBG("ens160_set_opmode: Sensor '%s' has no support for operation mode %d", dev->name, opmode);
        return -EINVAL;
    }

    err = ens160_reg_write(dev, ENS160_REG_OPMODE, opmode);
    if (err) {
        LOG_DBG("ens160_set_opmode: Could not set mode %d for sensor '%s'", opmode, dev->name);
        data->opmode = SENSOR_ENS160_OPMODE_UNKNOWN;
        return err;
    }

    ens160_default_wait();
    data->opmode = opmode;
    return err;
}

static int ens160_initialize_custom_opmode(const struct device* dev, uint8_t step_count)
{
    struct ens160_data* data = dev->data;
    int err;

    err = ens160_set_opmode(dev, SENSOR_ENS160_OPMODE_IDLE);
    if (err) { return err; }

    err = ens160_clear_command(dev);
    if (err) { return err; }

    if (!step_count) {
        // Goes back to standard mode for the next measurements.

        data->step_count = step_count;
        data->stage = SENSOR_ENS160_STAGE_RUNNING;
        data->measurement_opmode = SENSOR_ENS160_OPMODE_STANDARD;
        memset(&data->step_config, 0, sizeof(data->step_config));

        return 0;
    }

    err = ens160_reg_write(dev, ENS160_REG_COMMAND, ENS160_COMMAND_SETSEQ);
    if (err) { return err; }

    ens160_default_wait();

    data->step_count = step_count;
    data->stage = SENSOR_ENS160_STAGE_CUSTOM_CONFIGURING;
    data->measurement_opmode = SENSOR_ENS160_OPMODE_CUSTOM;
    memset(&data->step_config, 0, sizeof(data->step_config));

    return 0;
}

static int ens160_add_custom_opmode_step(const struct device* dev)
{
    struct ens160_data* data = dev->data;
    struct ens160_step_config* step_config = &data->step_config;
    int err;
    uint8_t whatisthis_value;
    uint8_t seq_ack;
    uint8_t temp;

    if (data->stage != SENSOR_ENS160_STAGE_CUSTOM_CONFIGURING) {
        LOG_ERR("ens160_add_custom_opmode_step: sensor '%s' is in the wrong state for starting custom mode", dev->name);
        return -EINVAL;
    }

    temp = (uint8_t) (((step_config->time / 24) - 1) << 6);
    if (step_config->measuring_hp0) temp = temp | 0x20;
    if (step_config->measuring_hp1) temp = temp | 0x10;
    if (step_config->measuring_hp2) temp = temp | 0x8;
    if (step_config->measuring_hp3) temp = temp | 0x4;

    err = ens160_reg_write(dev, ENS160_REG_GPR_WRITE_0, temp);
    if (err) { return err; }

    temp = (uint8_t) (((step_config->time / 24) - 1) >> 2);

    err = ens160_reg_write(dev, ENS160_REG_GPR_WRITE_1, temp);
    if (err) { return err; }

    err = ens160_reg_write(dev, ENS160_REG_GPR_WRITE_2, (uint8_t) (step_config->temperature_hp0 / 2));
    if (err) { return err; }

    err = ens160_reg_write(dev, ENS160_REG_GPR_WRITE_3, (uint8_t) (step_config->temperature_hp1 / 2));
    if (err) { return err; }

    err = ens160_reg_write(dev, ENS160_REG_GPR_WRITE_4, (uint8_t) (step_config->temperature_hp2 / 2));
    if (err) { return err; }

    err = ens160_reg_write(dev, ENS160_REG_GPR_WRITE_5, (uint8_t) (step_config->temperature_hp3 / 2));
    if (err) { return err; }

    err = ens160_reg_write(dev, ENS160_REG_GPR_WRITE_6, (uint8_t) (data->step_count - 1));
    if (err) { return err; }

    if (data->step_count == 1) {
        whatisthis_value = 128;
    } else {
        whatisthis_value = 0;
    }

    err = ens160_reg_write(dev, ENS160_REG_GPR_WRITE_7, whatisthis_value);
    if (err) { return err; }

    ens160_default_wait();

    err = ens160_reg_read(dev, ENS160_REG_GPR_READ_7, &seq_ack, 1);
    if (err) { return err; }

    // TODO Check if this is really necessary (e.g. perhaps wait is only needed when configuring the last step?)
    ens160_default_wait();

    if ((ENS160_SEQ_ACK_COMPLETE | data->step_count) != seq_ack) {
        data->step_count -= 1;
        memset(&data->step_config, 0, sizeof(data->step_config));

        if (data->step_count == 0) {
            err = ens160_get_opmode(dev);
            if (err) { return err; }
        }

        return 0;
    }

    LOG_ERR(
        "ens160_add_custom_opmode_step: sensor '%s' did not acknowledge the new custom step configuration.", dev->name
    );
    return -EIO;
}

static int ens160_reset(const struct device* dev)
{
    struct ens160_data* data = dev->data;
    enum sensor_value_ens160_opmode opmode;
    uint8_t interrupt_flags;
    int err;

    err = ens160_set_opmode(dev, SENSOR_ENS160_OPMODE_RESET);
    if (err) {
        LOG_ERR("ens160_reset: Could not reset sensor '%s'; code: %d", dev->name, err);
        return err;
    }

    opmode = SENSOR_ENS160_OPMODE_IDLE;
    err = ens160_set_opmode(dev, opmode);
    if (err) {
        LOG_ERR("ens160_reset: Could not set initial opmode %d for sensor '%s'; code: %d", opmode, dev->name, err);
        return err;
    }

    err = ens160_clear_command(dev);
    if (err) { return err; }

#ifdef CONFIG_ENS160_INTERRUPTS
    interrupt_flags = 0x0b;
#else
    interrupt_flags = 0x00;
#endif

        err = ens160_reg_write(dev, ENS160_REG_CONFIG, interrupt_flags);
    if (err) {
        LOG_ERR("ens160_reset: Could not set up interrupt configuration for sensor '%s'; code: %d", dev->name, err);
    }

    data->step_count = 0;
    data->stage = SENSOR_ENS160_STAGE_RUNNING;
    data->measurement_opmode = SENSOR_ENS160_OPMODE_STANDARD;
    memset(&data->step_config, 0, sizeof(data->step_config));

    return 0;
}

static int ens160_get_part_id(const struct device* dev)
{
    struct ens160_data* data = dev->data;
    int err;
    uint8_t buf[2];

    err = ens160_reg_read(dev, ENS160_REG_PART_ID, buf, 2);
    if (err) { return err; }

    data->part_id = buf[0] | ((uint16_t) buf[1] << 8);

    if (data->part_id == ENS160_PARTID) {
        data->rev_ens16x = 0;
    } else if (data->part_id == ENS161_PARTID) {
        data->rev_ens16x = 1;
    }

    return err;
}

static int ens160_get_firmware_info(const struct device* dev)
{
    struct ens160_data* data = dev->data;
    uint8_t buf[3];
    int err;

    err = ens160_reg_write(dev, ENS160_REG_COMMAND, ENS160_COMMAND_GET_APPVER);
    if (err) { return err; }

    ens160_default_wait();

    err = ens160_reg_read(dev, ENS160_REG_GPR_READ_4, buf, 3);
    if (err) { return err; }

    data->fw_ver_major = buf[0];
    data->fw_ver_minor = buf[1];
    data->fw_ver_build = buf[2];

    if (data->fw_ver_major > 6) {
        data->rev_ens16x = 1;
    } else {
        data->rev_ens16x = 0;
    }

    return 0;
}

static int ens160_get_sensor_status(const struct device* dev)
{
    struct ens160_data* data = dev->data;
    uint8_t status;
    int err;

    err = ens160_reg_read(dev, ENS160_REG_DATA_STATUS, &status, 1);
    if (err) { return err; }

    data->sensor_status = status;
    return 0;
}

static int ens160_wait_for_status(const struct device* dev, uint8_t expected_type)
{
#define IS_NEW (expected_type == (expected_type & status))
    struct ens160_data* data = dev->data;
    uint8_t status;
    int err;
    uint32_t max_cycles;

    if (data->waiting_for_new_data) {
        max_cycles = data->max_wait_cycles;

        do {
            k_sleep(data->wait_cycle_time);
            err = ens160_reg_read(dev, ENS160_REG_DATA_STATUS, &status, 1);
        }
        while (max_cycles-- && !err && !IS_NEW);
    } else {
        err = ens160_reg_read(dev, ENS160_REG_DATA_STATUS, &status, 1);
        if (err) { return err; }
    }

    data->sensor_status = status;

    if (!err && !IS_NEW) { return -EAGAIN; }

    return err;
#undef IS_NEW
}

static void ens160_warn_incomplete_custom_configuration(const struct device* dev, const char* function_name) {
    struct ens160_data* data = dev->data;

    if ((int) data->stage != (int) SENSOR_ENS160_STAGE_CUSTOM_CONFIGURING) {
        LOG_DBG(
            "%s: sensor '%s': custom mode was not configured; changing the sensor's operation "
            "mode to CUSTOM at this moment is most likely an user error",
            function_name,
            dev->name
        );
    } else if (data->step_count > 0) {
        LOG_DBG(
            "%s: sensor '%s': custom mode is missing %d step(s); changing the sensor's "
            "operation mode to CUSTOM at this moment is most likely an user error",
            function_name,
            dev->name,
            data->step_count
        );
    }
}

static int ens160_begin_measurement_cycle(const struct device* dev, uint8_t expected_type)
{
    struct ens160_data* data = dev->data;
    int err;

    ens160_warn_incomplete_custom_configuration(dev, "ens160_begin_measurement_cycle");

    err = ens160_set_opmode(dev, data->measurement_opmode);
    if (err) { return err; }

    err = ens160_wait_for_status(dev, expected_type);
    if (err) {
        if (err != -EAGAIN) { return err; }
        LOG_DBG(
            "*** ens160_read_data: sensor '%s': could not detect new information; data may be outdated\n", dev->name
        );
        err = 0;
    }

    return err;
}

static int ens160_read_data(const struct device* dev)
{
    struct ens160_data* data = dev->data;
    uint8_t buf[7];
    int err;

    err = ens160_reg_read(dev, ENS160_REG_DATA_AQI, buf, 7);
    if (err) { return err; }

    data->aqi_uba = buf[0];
    data->tvoc = buf[1] | ((uint16_t) buf[2] << 8);
    data->eco2 = buf[3] | ((uint16_t) buf[4] << 8);

    if (data->rev_ens16x > 0) {
        data->aqi_epa = ((uint16_t) buf[5]) | ((uint16_t) buf[6] << 8);
    } else {
        data->aqi_epa = 0;
    }

    return 0;
}

static int ens160_read_gpr(const struct device* dev)
{
    struct ens160_data* data = dev->data;
    uint8_t buf[8];
    uint8_t status;
    int err;

    err = ens160_reg_read(dev, ENS160_REG_GPR_READ_0, buf, 8);
    if (err) { return err; }

    data->hotplate0_rs = CONVERT_RS_RAW2OHMS_I((uint32_t) (buf[0] | ((uint16_t) buf[1] << 8)));
    data->hotplate1_rs = CONVERT_RS_RAW2OHMS_I((uint32_t) (buf[2] | ((uint16_t) buf[3] << 8)));
    data->hotplate2_rs = CONVERT_RS_RAW2OHMS_I((uint32_t) (buf[4] | ((uint16_t) buf[5] << 8)));
    data->hotplate3_rs = CONVERT_RS_RAW2OHMS_I((uint32_t) (buf[6] | ((uint16_t) buf[7] << 8)));

    err = ens160_reg_read(dev, ENS160_REG_DATA_BL, buf, 8);
    if (err) { return err; }

    data->hotplate0_bl = CONVERT_RS_RAW2OHMS_I((uint32_t) (buf[0] | ((uint16_t) buf[1] << 8)));
    data->hotplate1_bl = CONVERT_RS_RAW2OHMS_I((uint32_t) (buf[2] | ((uint16_t) buf[3] << 8)));
    data->hotplate2_bl = CONVERT_RS_RAW2OHMS_I((uint32_t) (buf[4] | ((uint16_t) buf[5] << 8)));
    data->hotplate3_bl = CONVERT_RS_RAW2OHMS_I((uint32_t) (buf[6] | ((uint16_t) buf[7] << 8)));

    err = ens160_reg_read(dev, ENS160_REG_DATA_MISR, buf, 1);
    if (err) { return err; }

    return 0;
}

static int ens160_set_temperature(const struct device* dev, const struct sensor_value* temperature)
{
    struct ens160_data* data = dev->data;
    int err;
    uint8_t buf[2];
    uint16_t value;

    value = (uint16_t) ((sensor_value_to_double(temperature) + 273.15) * 64.0);
    buf[0] = value & 0xff;
    buf[1] = (value >> 8) & 0xff;

    err = ens160_reg_write_buf(dev, ENS160_REG_TEMP_IN, buf, 2);
    if (err) { return err; }

    data->temp_in.val1 = temperature->val1;
    data->temp_in.val2 = temperature->val2;

    return 0;
}

static int ens160_set_humidity(const struct device* dev, const struct sensor_value* humidity)
{
    struct ens160_data* data = dev->data;
    int err;
    uint8_t buf[2];
    uint16_t value;

    value = (uint16_t) (sensor_value_to_double(humidity) * 512.0);
    buf[0] = value & 0xff;
    buf[1] = (value >> 8) & 0xff;

    err = ens160_reg_write_buf(dev, ENS160_REG_RH_IN, buf, 2);
    if (err) { return err; }

    data->rh_in.val1 = humidity->val1;
    data->rh_in.val2 = humidity->val2;

    return 0;
}

static int ens160_init(const struct device* dev)
{
    struct ens160_data* data = dev->data;
    enum sensor_value_ens160_opmode opmode;
    uint8_t interrupt_flags;
    int err;

    LOG_DBG("ens160_init: initializing sensor '%s'...", dev->name);

    data->available = false;
    data->waiting_for_new_data = true;
    data->stage = SENSOR_ENS160_STAGE_UNKNOWN;
    data->sensor_status = 0;
    data->opmode = SENSOR_ENS160_OPMODE_UNKNOWN;
    data->measurement_opmode = SENSOR_ENS160_OPMODE_STANDARD;
    data->rev_ens16x = 0;
    data->fw_ver_build = 0;
    data->fw_ver_major = 0;
    data->fw_ver_minor = 0;
    data->part_id = 0;
    data->step_count = 0;
    data->tvoc = 0;
    data->eco2 = 0;
    data->aqi_uba = 0;
    data->aqi_epa = 0;
    data->max_wait_cycles = 200;
    data->wait_cycle_time = K_MSEC(10);
    memset(&data->step_config, 0, sizeof(data->step_config));
    memset(&data->hotplates_rs, 0, sizeof(data->hotplates_rs));
    memset(&data->hotplates_bl, 0, sizeof(data->hotplates_bl));

#if ENS160_BUS_SPI
    data->mem_page = 0;
#endif

    err = ens160_bus_check(dev);
    if (err < 0) {
        LOG_ERR("ens160_init: Bus not ready for '%s'", dev->name);
        return err;
    }

    err = ens160_reset(dev);
    if (err) {
        LOG_ERR("ens160_init: Could not reset sensor '%s'; code: %d", dev->name, err);
        return err;
    }

    err = ens160_get_part_id(dev);
    if (err) {
        LOG_ERR("ens160_init: Could not get part-id for sensor '%s'; code: %d", dev->name, err);
        return err;
    }

    opmode = SENSOR_ENS160_OPMODE_IDLE;
    err = ens160_set_opmode(dev, opmode);
    if (err) {
        LOG_ERR("ens160_init: Could not set initial opmode %d for sensor '%s'; code: %d", opmode, dev->name, err);
        return err;
    }

    err = ens160_get_firmware_info(dev);
    if (err) {
        LOG_ERR("ens160_init: Could not get firmware info for sensor '%s'; code: %d", dev->name, err);
        return err;
    }

    data->stage = SENSOR_ENS160_STAGE_RUNNING;
    data->available = true;

    return 0;
}

static int ens160_sample_fetch(const struct device* dev, enum sensor_channel chan)
{
    struct ens160_data* data = dev->data;
    int err;

    if (!data->available) {
        LOG_ERR(
            "ens160_sample_fetch: sensor '%s' is not available yet. Be sure to initialize it before fetching samples.",
            dev->name
        );
        return -EINVAL;
    }

    if (data->stage != SENSOR_ENS160_STAGE_RUNNING) {
        LOG_ERR(
            "ens160_sample_fetch: sensor '%s' is currently in an invalid stage '%d', cannot fetch samples right now.",
            dev->name,
            data->stage
        );
        return -EINVAL;
    }

    switch ((int) chan) {
    case SENSOR_CHAN_ALL:
        err = ens160_begin_measurement_cycle(dev, SENSOR_ENS160_STATUS_NEW_DATA | SENSOR_ENS160_STATUS_NEW_GPR);
        if (err) { return err; }
        err = ens160_read_data(dev);
        if (err) { return err; }
        err = ens160_read_gpr(dev);
        if (err) { return err; }
        err = ens160_get_sensor_status(dev);
        if (err) { return err; }
        err = ens160_get_opmode(dev);
        return err;
    case SENSOR_CHAN_VOC:
    case SENSOR_CHAN_CO2:
    case SENSOR_CHAN_ENS160_AQI_UBA:
    case SENSOR_CHAN_ENS160_AQI_EPA:
        err = ens160_begin_measurement_cycle(dev, SENSOR_ENS160_STATUS_NEW_DATA);
        if (err) { return err; }
        err = ens160_read_data(dev);
        if (err) { return err; }
        err = ens160_get_sensor_status(dev);
        if (err) { return err; }
        err = ens160_get_opmode(dev);
        return err;
    case SENSOR_CHAN_ENS160_HOTPLATE0_BASELINE:
    case SENSOR_CHAN_ENS160_HOTPLATE1_BASELINE:
    case SENSOR_CHAN_ENS160_HOTPLATE2_BASELINE:
    case SENSOR_CHAN_ENS160_HOTPLATE3_BASELINE:
    case SENSOR_CHAN_ENS160_HOTPLATE0_RESISTANCE:
    case SENSOR_CHAN_ENS160_HOTPLATE1_RESISTANCE:
    case SENSOR_CHAN_ENS160_HOTPLATE2_RESISTANCE:
    case SENSOR_CHAN_ENS160_HOTPLATE3_RESISTANCE:
        err = ens160_begin_measurement_cycle(dev, SENSOR_ENS160_STATUS_NEW_GPR);
        if (err) { return err; }
        err = ens160_read_gpr(dev);
        if (err) { return err; }
        err = ens160_get_sensor_status(dev);
        if (err) { return err; }
        err = ens160_get_opmode(dev);
        return err;
    case SENSOR_CHAN_ENS160_REG_STATUS: return ens160_get_sensor_status(dev);
    case SENSOR_CHAN_ENS160_OPMODE: return ens160_get_opmode(dev);
    default: return -EINVAL;
    }
}

static int ens160_channel_get(const struct device* dev, enum sensor_channel chan, struct sensor_value* val)
{
    struct ens160_data* data = dev->data;
    int err;

    if (!data->available) {
        LOG_ERR(
            "ens160_channel_get: sensor '%s' is not available yet. Be sure to initialize it before fetching samples.",
            dev->name
        );
        return -EINVAL;
    }

    if (data->stage != SENSOR_ENS160_STAGE_RUNNING) {
        LOG_ERR(
            "ens160_channel_get: sensor '%s' is currently in an invalid stage '%d', cannot fetch samples right now.",
            dev->name,
            data->stage
        );
        return -EINVAL;
    }

    switch ((int) chan) {
    case SENSOR_CHAN_AMBIENT_TEMP:
        val->val1 = data->temp_in.val1;
        val->val2 = data->temp_in.val2;
        break;
    case SENSOR_CHAN_HUMIDITY:
        val->val1 = data->rh_in.val1;
        val->val2 = data->rh_in.val2;
        break;
    case SENSOR_CHAN_VOC:
        val->val1 = data->tvoc;
        val->val2 = 0;
        break;
    case SENSOR_CHAN_CO2:
        val->val1 = data->eco2;
        val->val2 = 0;
        break;
    case SENSOR_CHAN_ENS160_AQI_UBA:
        val->val1 = data->aqi_uba;
        val->val2 = 0;
        break;
    case SENSOR_CHAN_ENS160_AQI_EPA:
        val->val1 = data->aqi_epa;
        val->val2 = 0;
        break;
    case SENSOR_CHAN_ENS160_HOTPLATE0_BASELINE:
        val->val1 = data->hotplate0_bl;
        val->val2 = 0;
        break;
    case SENSOR_CHAN_ENS160_HOTPLATE1_BASELINE:
        val->val1 = data->hotplate1_bl;
        val->val2 = 0;
        break;
    case SENSOR_CHAN_ENS160_HOTPLATE2_BASELINE:
        val->val1 = data->hotplate2_bl;
        val->val2 = 0;
        break;
    case SENSOR_CHAN_ENS160_HOTPLATE3_BASELINE:
        val->val1 = data->hotplate3_bl;
        val->val2 = 0;
        break;
    case SENSOR_CHAN_ENS160_HOTPLATE0_RESISTANCE:
        val->val1 = data->hotplate0_rs;
        val->val2 = 0;
        break;
    case SENSOR_CHAN_ENS160_HOTPLATE1_RESISTANCE:
        val->val1 = data->hotplate1_rs;
        val->val2 = 0;
        break;
    case SENSOR_CHAN_ENS160_HOTPLATE2_RESISTANCE:
        val->val1 = data->hotplate2_rs;
        val->val2 = 0;
        break;
    case SENSOR_CHAN_ENS160_HOTPLATE3_RESISTANCE:
        val->val1 = data->hotplate3_rs;
        val->val2 = 0;
        break;
    case SENSOR_CHAN_ENS160_PART_ID:
        val->val1 = data->part_id;
        val->val2 = 0;
        break;
    case SENSOR_CHAN_ENS160_REVISION_MAJOR:
        val->val1 = data->fw_ver_major;
        val->val2 = 0;
        break;
    case SENSOR_CHAN_ENS160_REVISION_MINOR:
        val->val1 = data->fw_ver_minor;
        val->val2 = 0;
        break;
    case SENSOR_CHAN_ENS160_REVISION_BUILD:
        val->val1 = data->fw_ver_build;
        val->val2 = 0;
        break;
    case SENSOR_CHAN_ENS160_OPMODE:
        val->val1 = data->opmode;
        val->val2 = 0;
        break;
    case SENSOR_CHAN_ENS160_REG_STATUS:
        val->val1 = data->sensor_status;
        val->val2 = 0;
        break;
    default: return -EINVAL;
    }

    return 0;
}

static int
ens160_attr_set_custom_config(const struct device* dev, enum sensor_attribute attr, const struct sensor_value* val)
{
    struct ens160_data* data = dev->data;
    int err;

    if (data->stage == SENSOR_ENS160_STAGE_RUNNING) {
        if ((int) attr != (int) SENSOR_ATTR_ENS160_CUSTOM_STEP_COUNT) {
            LOG_ERR(
                "ens160_attr_set_custom_config: sensor '%s': invalid attribute '%d' for current internal state; custom "
                "mode settings must be started by setting the "
                "attribute SENSOR_ATTR_ENS160_CUSTOM_STEP_COUNT first",
                dev->name,
                attr
            );
            return -EINVAL;
        }
        return ens160_initialize_custom_opmode(dev, val->val1);

    } else if (data->stage == SENSOR_ENS160_STAGE_CUSTOM_CONFIGURING) {
        switch ((int) attr) {
        case SENSOR_ATTR_ENS160_CUSTOM_STEP_TIME: data->step_config.time = val->val1; break;
        case SENSOR_ATTR_ENS160_CUSTOM_HOTPLATE0_MEASURING: data->step_config.measuring_hp0 = val->val1; break;
        case SENSOR_ATTR_ENS160_CUSTOM_HOTPLATE1_MEASURING: data->step_config.measuring_hp1 = val->val1; break;
        case SENSOR_ATTR_ENS160_CUSTOM_HOTPLATE2_MEASURING: data->step_config.measuring_hp2 = val->val1; break;
        case SENSOR_ATTR_ENS160_CUSTOM_HOTPLATE3_MEASURING: data->step_config.measuring_hp3 = val->val1; break;
        case SENSOR_ATTR_ENS160_CUSTOM_HOTPLATE0_TEMPERATURE: data->step_config.temperature_hp0 = val->val1; break;
        case SENSOR_ATTR_ENS160_CUSTOM_HOTPLATE1_TEMPERATURE: data->step_config.temperature_hp1 = val->val1; break;
        case SENSOR_ATTR_ENS160_CUSTOM_HOTPLATE2_TEMPERATURE: data->step_config.temperature_hp2 = val->val1; break;
        case SENSOR_ATTR_ENS160_CUSTOM_HOTPLATE3_TEMPERATURE: data->step_config.temperature_hp3 = val->val1; break;
        case SENSOR_ATTR_ENS160_CUSTOM_STEP_NEXT: return ens160_add_custom_opmode_step(dev);
        default: return -EINVAL;
        }
    } else {
        LOG_ERR(
            "ens160_attr_set_custom_config: sensor '%s': wrong internal state for configuring custom mode; "
            "be sure to initialize custom mode by setting the attribute SENSOR_ATTR_ENS160_CUSTOM_STEP_COUNT first",
            dev->name
        );
        return -EINVAL;
    }

    return 0;
}

static int ens160_attr_set_opmode(const struct device* dev, enum sensor_value_ens160_opmode opmode)
{
    struct ens160_data* data = dev->data;
    int err;

    switch ((int) opmode) {
    case SENSOR_ENS160_OPMODE_DEEP_SLEEP:
    case SENSOR_ENS160_OPMODE_IDLE:
    case SENSOR_ENS160_OPMODE_LP:
    case SENSOR_ENS160_OPMODE_STANDARD: break;
    case SENSOR_ENS160_OPMODE_RESET: return ens160_reset(dev);
    case SENSOR_ENS160_OPMODE_CUSTOM:
        ens160_warn_incomplete_custom_configuration(dev, "ens160_attr_set_opmode");
        break;
    default: return -EINVAL;
    }

    err = ens160_set_opmode(dev, opmode);
    if (err) { return err; }

    data->stage = SENSOR_ENS160_STAGE_RUNNING;

    return 0;
}

static int ens160_attr_set(
    const struct device* dev,
    enum sensor_channel chan,
    enum sensor_attribute attr,
    const struct sensor_value* val
)
{
    struct ens160_data* data = dev->data;
    int err;

    if (!data->available) {
        LOG_ERR(
            "ens160_channel_get: sensor '%s' is not available yet. Be sure to initialize it before fetching samples.",
            dev->name
        );
        return -EINVAL;
    }

    if (!((int) chan == (int) SENSOR_CHAN_ENS160_OPMODE || (int) chan == (int) SENSOR_CHAN_ENS160_OPMODE_CUSTOM)
        && data->stage != (int) SENSOR_ENS160_STAGE_RUNNING) {
        LOG_ERR(
            "ens160_channel_get: sensor '%s' is currently in an invalid stage '%d', cannot fetch samples right now.",
            dev->name,
            data->stage
        );
        return -EINVAL;
    }

    switch ((int) chan) {
    case SENSOR_CHAN_AMBIENT_TEMP: return ens160_set_temperature(dev, val);
    case SENSOR_CHAN_HUMIDITY: return ens160_set_humidity(dev, val);
    case SENSOR_CHAN_ENS160_OPMODE: return ens160_attr_set_opmode(dev, SENSOR_VALUE_ENS160_TO_OPMODE(val));
    case SENSOR_CHAN_ENS160_OPMODE_CUSTOM: return ens160_attr_set_custom_config(dev, attr, val);
    default: return -EINVAL;
    }

    return 0;
}

static const struct sensor_driver_api ens160_api_funcs = {
    .sample_fetch = ens160_sample_fetch,
    .channel_get = ens160_channel_get,
    .attr_set = ens160_attr_set,
};

/* Initializes a struct ens160_config for an instance on a SPI bus. */
#define ENS160_CONFIG_SPI(inst) \
    { \
        .bus.spi = SPI_DT_SPEC_INST_GET(inst, ENS160_SPI_OPERATION, 0), .bus_io = &ens160_bus_io_spi, \
    }

/* Initializes a struct ens160_config for an instance on an I2C bus. */
#define ENS160_CONFIG_I2C(inst) \
    { \
        .bus.i2c = I2C_DT_SPEC_INST_GET(inst), .bus_io = &ens160_bus_io_i2c, \
    }

/*
 * Main instantiation macro, which selects the correct bus-specific
 * instantiation macros for the instance.
 */
#define ENS160_DEFINE(inst) \
    static struct ens160_data ens160_data_##inst; \
    static const struct ens160_config ens160_config_##inst = \
        COND_CODE_1(DT_INST_ON_BUS(inst, spi), (ENS160_CONFIG_SPI(inst)), (ENS160_CONFIG_I2C(inst))); \
    SENSOR_DEVICE_DT_INST_DEFINE( \
        inst, \
        ens160_init, \
        NULL, \
        &ens160_data_##inst, \
        &ens160_config_##inst, \
        POST_KERNEL, \
        CONFIG_SENSOR_INIT_PRIORITY, \
        &ens160_api_funcs \
    );

/* Create the struct device for every status "okay" node in the devicetree. */
DT_INST_FOREACH_STATUS_OKAY(ENS160_DEFINE)
