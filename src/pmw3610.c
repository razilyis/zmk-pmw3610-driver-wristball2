/* pmw3610.c
 *
 * Copyright (c) 2022 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT pixart_pmw3610

// 12-bit two's complement value to int16_t
// adapted from https://stackoverflow.com/questions/70802306/convert-a-12-bit-signed-number-in-c
#define TOINT16(val, bits) (((struct { int16_t value : bits; }){val}).value)

#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/input/input.h>
#include <zephyr/device.h>
#include <zephyr/sys/dlist.h>
#include <drivers/behavior.h>
#include <zmk/keymap.h>
#include <zmk/behavior.h>
#include <zmk/keys.h>
#include <zmk/behavior_queue.h>
#include <zmk/event_manager.h>
#include <zmk/events/position_state_changed.h>
#include <zmk/events/layer_state_changed.h>
#include "pmw3610.h"

#include <limits.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(pmw3610, CONFIG_INPUT_LOG_LEVEL);

// 外部からアクセス可能なorientation関数の宣言
extern uint16_t pmw3610_get_orientation(void);
extern void pmw3610_set_orientation(uint16_t orientation);

// ---- 45deg rotation support (Q15 fixed point) ----
// cos(45°) = sin(45°) ≈ 0.70710678
#define Q15_COS45 23170 // round(0.70710678 * 32768)

static inline int32_t q15_mul(int32_t v, int32_t q15) {
    return (v * q15) >> 15;
}

static inline void rotate_xy_45step(int16_t in_x, int16_t in_y, uint16_t deg,
                                    int16_t *out_x, int16_t *out_y) {
    uint16_t r = (uint16_t)(deg % 360);

    // 90°単位は整数変換が速いので先に処理
    switch (r) {
    case 0:   *out_x = in_x;  *out_y = in_y;  return;
    case 90:  *out_x = -in_y; *out_y = in_x;  return;
    case 180: *out_x = -in_x; *out_y = -in_y; return;
    case 270: *out_x = in_y;  *out_y = -in_x; return;
    default:
        break;
    }

    // 45°刻みのみサポート（それ以外は素通し）
    if ((r % 45) != 0) {
        *out_x = in_x;
        *out_y = in_y;
        return;
    }

    // 45/135/225/315 を Q15 係数で回す
    // x' = x*c - y*s
    // y' = x*s + y*c
    int32_t c = Q15_COS45;
    int32_t s = Q15_COS45;

    // 45:  c=+C, s=+C
    // 135: c=-C, s=+C
    // 225: c=-C, s=-C
    // 315: c=+C, s=-C
    if (r == 135 || r == 225) c = -c;
    if (r == 225 || r == 315) s = -s;

    int32_t ix = in_x;
    int32_t iy = in_y;

    int32_t xo = q15_mul(ix, c) - q15_mul(iy, s);
    int32_t yo = q15_mul(ix, s) + q15_mul(iy, c);

    // 念のためクリップ（安全策）
    if (xo > INT16_MAX) xo = INT16_MAX;
    if (xo < INT16_MIN) xo = INT16_MIN;
    if (yo > INT16_MAX) yo = INT16_MAX;
    if (yo < INT16_MIN) yo = INT16_MIN;

    *out_x = (int16_t)xo;
    *out_y = (int16_t)yo;
}

//////// Sensor initialization steps definition //////////
// init is done in non-blocking manner (i.e., async), a //
// delayable work is defined for this purpose           //
enum pmw3610_init_step {
    ASYNC_INIT_STEP_POWER_UP,  // reset cs line and assert power-up reset
    ASYNC_INIT_STEP_CLEAR_OB1, // clear observation1 register for self-test check
    ASYNC_INIT_STEP_CHECK_OB1, // check the value of observation1 register after self-test check
    ASYNC_INIT_STEP_CONFIGURE, // set other registes like cpi and donwshift time (run, rest1, rest2)
                               // and clear motion registers

    ASYNC_INIT_STEP_COUNT // end flag
};

/* Timings (in ms) needed in between steps to allow each step finishes succussfully. */
static const int32_t async_init_delay[ASYNC_INIT_STEP_COUNT] = {
    [ASYNC_INIT_STEP_POWER_UP] = 10,
    [ASYNC_INIT_STEP_CLEAR_OB1] = 200,
    [ASYNC_INIT_STEP_CHECK_OB1] = 50,
    [ASYNC_INIT_STEP_CONFIGURE] = 0,
};

static int pmw3610_async_init_power_up(const struct device *dev);
static int pmw3610_async_init_clear_ob1(const struct device *dev);
static int pmw3610_async_init_check_ob1(const struct device *dev);
static int pmw3610_async_init_configure(const struct device *dev);

static int (*const async_init_fn[ASYNC_INIT_STEP_COUNT])(const struct device *dev) = {
    [ASYNC_INIT_STEP_POWER_UP] = pmw3610_async_init_power_up,
    [ASYNC_INIT_STEP_CLEAR_OB1] = pmw3610_async_init_clear_ob1,
    [ASYNC_INIT_STEP_CHECK_OB1] = pmw3610_async_init_check_ob1,
    [ASYNC_INIT_STEP_CONFIGURE] = pmw3610_async_init_configure,
};

//////// Function definitions //////////

static int spi_cs_ctrl(const struct device *dev, bool enable) {
    const struct pixart_config *config = dev->config;
    int err;

    if (!enable) {
        k_busy_wait(T_NCS_SCLK);
    }

    err = gpio_pin_set_dt(&config->cs_gpio, (int)enable);
    if (err) {
        LOG_ERR("SPI CS ctrl failed");
    }

    if (enable) {
        k_busy_wait(T_NCS_SCLK);
    }

    return err;
}

static int reg_read(const struct device *dev, uint8_t reg, uint8_t *buf) {
    int err;
    const struct pixart_config *config = dev->config;

    __ASSERT_NO_MSG((reg & SPI_WRITE_BIT) == 0);

    err = spi_cs_ctrl(dev, true);
    if (err) {
        return err;
    }

    const struct spi_buf tx_buf = {.buf = &reg, .len = 1};
    const struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};

    err = spi_write_dt(&config->bus, &tx);
    if (err) {
        LOG_ERR("Reg read failed on SPI write");
        return err;
    }

    k_busy_wait(T_SRAD);

    struct spi_buf rx_buf = {
        .buf = buf,
        .len = 1,
    };
    const struct spi_buf_set rx = {
        .buffers = &rx_buf,
        .count = 1,
    };

    err = spi_read_dt(&config->bus, &rx);
    if (err) {
        LOG_ERR("Reg read failed on SPI read");
        return err;
    }

    err = spi_cs_ctrl(dev, false);
    if (err) {
        return err;
    }

    k_busy_wait(T_SRX);

    return 0;
}

static int _reg_write(const struct device *dev, uint8_t reg, uint8_t val) {
    int err;
    const struct pixart_config *config = dev->config;

    __ASSERT_NO_MSG((reg & SPI_WRITE_BIT) == 0);

    err = spi_cs_ctrl(dev, true);
    if (err) {
        return err;
    }

    uint8_t buf[] = {SPI_WRITE_BIT | reg, val};
    const struct spi_buf tx_buf = {.buf = buf, .len = ARRAY_SIZE(buf)};
    const struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};

    err = spi_write_dt(&config->bus, &tx);
    if (err) {
        LOG_ERR("Reg write failed on SPI write");
        return err;
    }

    k_busy_wait(T_SCLK_NCS_WR);

    err = spi_cs_ctrl(dev, false);
    if (err) {
        return err;
    }

    k_busy_wait(T_SWX);

    return 0;
}

static int reg_write(const struct device *dev, uint8_t reg, uint8_t val) {
    int err;

    err = _reg_write(dev, PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_ENABLE);
    if (unlikely(err != 0)) {
        return err;
    }

    err = _reg_write(dev, reg, val);
    if (unlikely(err != 0)) {
        return err;
    }

    err = _reg_write(dev, PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_DISABLE);
    if (unlikely(err != 0)) {
        return err;
    }

    return 0;
}

static int motion_burst_read(const struct device *dev, uint8_t *buf, size_t burst_size) {
    int err;
    const struct pixart_config *config = dev->config;

    __ASSERT_NO_MSG(burst_size <= PMW3610_MAX_BURST_SIZE);

    err = spi_cs_ctrl(dev, true);
    if (err) {
        return err;
    }

    uint8_t reg_buf[] = {PMW3610_REG_MOTION_BURST};
    const struct spi_buf tx_buf = {.buf = reg_buf, .len = ARRAY_SIZE(reg_buf)};
    const struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};

    err = spi_write_dt(&config->bus, &tx);
    if (err) {
        LOG_ERR("Motion burst failed on SPI write");
        return err;
    }

    k_busy_wait(T_SRAD_MOTBR);

    const struct spi_buf rx_buf = {
        .buf = buf,
        .len = burst_size,
    };
    const struct spi_buf_set rx = {.buffers = &rx_buf, .count = 1};

    err = spi_read_dt(&config->bus, &rx);
    if (err) {
        LOG_ERR("Motion burst failed on SPI read");
        return err;
    }

    err = spi_cs_ctrl(dev, false);
    if (err) {
        return err;
    }

    k_busy_wait(T_BEXIT);

    return 0;
}

static int burst_write(const struct device *dev, const uint8_t *addr, const uint8_t *buf,
                       size_t size) {
    int err;

    err = _reg_write(dev, PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_ENABLE);
    if (unlikely(err != 0)) {
        return err;
    }

    for (size_t i = 0; i < size; i++) {
        err = _reg_write(dev, addr[i], buf[i]);

        if (err) {
            LOG_ERR("Burst write failed on SPI write (data)");
            return err;
        }
    }

    err = _reg_write(dev, PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_DISABLE);
    if (unlikely(err != 0)) {
        return err;
    }

    return 0;
}

static int check_product_id(const struct device *dev) {
    uint8_t product_id = 0x01;
    int err = reg_read(dev, PMW3610_REG_PRODUCT_ID, &product_id);
    if (err) {
        LOG_ERR("Cannot obtain product id");
        return err;
    }

    if (product_id != PMW3610_PRODUCT_ID) {
        LOG_ERR("Incorrect product id 0x%x (expecting 0x%x)!", product_id, PMW3610_PRODUCT_ID);
        return -EIO;
    }

    return 0;
}

static int set_cpi(const struct device *dev, uint32_t cpi) {
    if ((cpi > PMW3610_MAX_CPI) || (cpi < PMW3610_MIN_CPI)) {
        LOG_ERR("CPI value %u out of range", cpi);
        return -EINVAL;
    }

    uint8_t value = (cpi / 200);
    LOG_INF("Setting CPI to %u (reg value 0x%x)", cpi, value);

    uint8_t addr[] = {0x7F, PMW3610_REG_RES_STEP, 0x7F};
    uint8_t data[] = {0xFF, value, 0x00};
    int err = burst_write(dev, addr, data, 3);
    if (err) {
        LOG_ERR("Failed to set CPI");
        return err;
    }

    struct pixart_data *dev_data = dev->data;
    dev_data->curr_cpi = cpi;

    return 0;
}

static int set_cpi_if_needed(const struct device *dev, uint32_t cpi) {
    struct pixart_data *data = dev->data;
    if (cpi != data->curr_cpi) {
        return set_cpi(dev, cpi);
    }
    return 0;
}

static int set_sample_time(const struct device *dev, uint8_t reg_addr, uint32_t sample_time) {
    uint32_t maxtime = 2550;
    uint32_t mintime = 10;
    if ((sample_time > maxtime) || (sample_time < mintime)) {
        LOG_WRN("Sample time %u out of range [%u, %u]", sample_time, mintime, maxtime);
        return -EINVAL;
    }

    uint8_t value = sample_time / mintime;
    LOG_INF("Set sample time to %u ms (reg value: 0x%x)", sample_time, value);

    int err = reg_write(dev, reg_addr, value);
    if (err) {
        LOG_ERR("Failed to change sample time");
    }

    return err;
}

static int set_downshift_time(const struct device *dev, uint8_t reg_addr, uint32_t time) {
    uint32_t maxtime;
    uint32_t mintime;

    switch (reg_addr) {
    case PMW3610_REG_RUN_DOWNSHIFT:
        maxtime = 32 * 255;
        mintime = 32;
        break;

    case PMW3610_REG_REST1_DOWNSHIFT:
        maxtime = 255 * 16 * CONFIG_PMW3610_REST1_SAMPLE_TIME_MS;
        mintime = 16 * CONFIG_PMW3610_REST1_SAMPLE_TIME_MS;
        break;

    case PMW3610_REG_REST2_DOWNSHIFT:
        maxtime = 255 * 128 * CONFIG_PMW3610_REST2_SAMPLE_TIME_MS;
        mintime = 128 * CONFIG_PMW3610_REST2_SAMPLE_TIME_MS;
        break;

    default:
        LOG_ERR("Not supported");
        return -ENOTSUP;
    }

    if ((time > maxtime) || (time < mintime)) {
        LOG_WRN("Downshift time %u out of range", time);
        return -EINVAL;
    }

    __ASSERT_NO_MSG((mintime > 0) && (maxtime / mintime <= UINT8_MAX));

    uint8_t value = time / mintime;
    LOG_INF("Set downshift time to %u ms (reg value 0x%x)", time, value);

    int err = reg_write(dev, reg_addr, value);
    if (err) {
        LOG_ERR("Failed to change downshift time");
    }

    return err;
}

static void set_interrupt(const struct device *dev, const bool en) {
    const struct pixart_config *config = dev->config;
    int ret = gpio_pin_interrupt_configure_dt(&config->irq_gpio,
                                              en ? GPIO_INT_LEVEL_ACTIVE : GPIO_INT_DISABLE);
    if (ret < 0) {
        LOG_ERR("can't set interrupt");
    }
}

static int pmw3610_async_init_power_up(const struct device *dev) {
    LOG_INF("async_init_power_up");

    spi_cs_ctrl(dev, false);
    spi_cs_ctrl(dev, true);

    return reg_write(dev, PMW3610_REG_POWER_UP_RESET, PMW3610_POWERUP_CMD_RESET);
}

static int pmw3610_async_init_clear_ob1(const struct device *dev) {
    LOG_INF("async_init_clear_ob1");

    return reg_write(dev, PMW3610_REG_OBSERVATION, 0x00);
}

static int pmw3610_async_init_check_ob1(const struct device *dev) {
    LOG_INF("async_init_check_ob1");

    uint8_t value;
    int err = reg_read(dev, PMW3610_REG_OBSERVATION, &value);
    if (err) {
        LOG_ERR("Can't do self-test");
        return err;
    }

    if ((value & 0x0F) != 0x0F) {
        LOG_ERR("Failed self-test (0x%x)", value);
        return -EINVAL;
    }

    err = check_product_id(dev);
    if (err) {
        LOG_ERR("Failed checking product id");
        return err;
    }

    return 0;
}

static int pmw3610_async_init_configure(const struct device *dev) {
    LOG_INF("async_init_configure");

    int err = 0;

    for (uint8_t reg = 0x02; (reg <= 0x05) && !err; reg++) {
        uint8_t buf[1];
        err = reg_read(dev, reg, buf);
    }

    if (!err) {
        err = set_cpi(dev, CONFIG_PMW3610_CPI);
    }

    if (!err) {
        err = reg_write(dev, PMW3610_REG_PERFORMANCE, PMW3610_PERFORMANCE_VALUE);
        LOG_INF("Set performance register (reg value 0x%x)", PMW3610_PERFORMANCE_VALUE);
    }

    if (!err) {
        err = set_downshift_time(dev, PMW3610_REG_RUN_DOWNSHIFT,
                                 CONFIG_PMW3610_RUN_DOWNSHIFT_TIME_MS);
    }
    if (!err) {
        err = set_sample_time(dev, PMW3610_REG_REST1_PERIOD, CONFIG_PMW3610_REST1_SAMPLE_TIME_MS);
    }
    if (!err) {
        err = set_downshift_time(dev, PMW3610_REG_REST1_DOWNSHIFT,
                                 CONFIG_PMW3610_REST1_DOWNSHIFT_TIME_MS);
    }

#if CONFIG_PMW3610_REST2_DOWNSHIFT_TIME_MS > 0
    if (!err) {
        err = set_downshift_time(dev, PMW3610_REG_REST2_DOWNSHIFT,
                                 CONFIG_PMW3610_REST2_DOWNSHIFT_TIME_MS);
    }
#endif
#if CONFIG_PMW3610_REST2_SAMPLE_TIME_MS >= 10
    if (!err) {
        err = set_sample_time(dev, PMW3610_REG_REST2_PERIOD, CONFIG_PMW3610_REST2_SAMPLE_TIME_MS);
    }
#endif
#if CONFIG_PMW3610_REST3_SAMPLE_TIME_MS >= 10
    if (!err) {
        err = set_sample_time(dev, PMW3610_REG_REST3_PERIOD, CONFIG_PMW3610_REST3_SAMPLE_TIME_MS);
    }
#endif
    if (err) {
        LOG_ERR("Config the sensor failed");
        return err;
    }

    return 0;
}

static void pmw3610_async_init(struct k_work *work) {
    struct k_work_delayable *work2 = (struct k_work_delayable *)work;
    struct pixart_data *data = CONTAINER_OF(work2, struct pixart_data, init_work);
    const struct device *dev = data->dev;

    LOG_INF("PMW3610 async init step %d", data->async_init_step);

    data->err = async_init_fn[data->async_init_step](dev);
    if (data->err) {
        LOG_ERR("PMW3610 initialization failed");
    } else {
        data->async_init_step++;

        if (data->async_init_step == ASYNC_INIT_STEP_COUNT) {
            data->ready = true;
            LOG_INF("PMW3610 initialized");
            set_interrupt(dev, true);
        } else {
            k_work_schedule(&data->init_work, K_MSEC(async_init_delay[data->async_init_step]));
        }
    }
}

#define AUTOMOUSE_LAYER (DT_PROP(DT_DRV_INST(0), automouse_layer))
#if AUTOMOUSE_LAYER > 0
struct k_timer automouse_layer_timer;
static bool automouse_triggered = false;

static void activate_automouse_layer() {
    automouse_triggered = true;
    zmk_keymap_layer_activate(AUTOMOUSE_LAYER);
    k_timer_start(&automouse_layer_timer, K_MSEC(CONFIG_PMW3610_AUTOMOUSE_TIMEOUT_MS), K_NO_WAIT);
}

static void deactivate_automouse_layer(struct k_timer *timer) {
    automouse_triggered = false;
    zmk_keymap_layer_deactivate(AUTOMOUSE_LAYER);
}

K_TIMER_DEFINE(automouse_layer_timer, deactivate_automouse_layer, NULL);
#endif

int ball_action_idx = -1;
static enum pixart_input_mode get_input_mode_for_current_layer(const struct device *dev) {
    const struct pixart_config *config = dev->config;
    uint8_t curr_layer = zmk_keymap_highest_layer_active();
    ball_action_idx = -1;
    for (size_t i = 0; i < config->scroll_layers_len; i++) {
        if (curr_layer == config->scroll_layers[i]) {
            return SCROLL;
        }
    }
    for (size_t i = 0; i < config->snipe_layers_len; i++) {
        if (curr_layer == config->snipe_layers[i]) {
            return SNIPE;
        }
    }
    for (size_t i = 0; i < config->ball_actions_len; i++) {
        for (size_t j = 0; j < config->ball_actions[i]->layers_len; j++) {
            if (curr_layer == config->ball_actions[i]->layers[j]) {
                ball_action_idx = (int)i;
                return BALL_ACTION;
            }
        }
    }
    return MOVE;
}

static int pmw3610_report_data(const struct device *dev) {
    struct pixart_data *data = dev->data;
    uint8_t buf[PMW3610_BURST_SIZE];

    if (unlikely(!data->ready)) {
        LOG_WRN("Device is not initialized yet");
        return -EBUSY;
    }

    int32_t dividor;
    enum pixart_input_mode input_mode = get_input_mode_for_current_layer(dev);
    bool input_mode_changed = data->curr_mode != input_mode;
    switch (input_mode) {
    case MOVE:
        set_cpi_if_needed(dev, CONFIG_PMW3610_CPI);
        dividor = CONFIG_PMW3610_CPI_DIVIDOR;
        break;
    case SCROLL:
        set_cpi_if_needed(dev, CONFIG_PMW3610_CPI);
        if (input_mode_changed) {
            data->scroll_delta_x = 0;
            data->scroll_delta_y = 0;
        }
        dividor = 1;
        break;
    case SNIPE:
        set_cpi_if_needed(dev, CONFIG_PMW3610_SNIPE_CPI);
        dividor = CONFIG_PMW3610_SNIPE_CPI_DIVIDOR;
        break;
    case BALL_ACTION:
        set_cpi_if_needed(dev, CONFIG_PMW3610_CPI);
        if (input_mode_changed) {
            data->ball_action_delta_x = 0;
            data->ball_action_delta_y = 0;
        }
        dividor = 1;
        break;
    default:
        return -ENOTSUP;
    }

    data->curr_mode = input_mode;

    int16_t x = 0;
    int16_t y = 0;

    int err = motion_burst_read(dev, buf, sizeof(buf));
    if (err) {
        return err;
    }

    int16_t raw_x =
        TOINT16((buf[PMW3610_X_L_POS] + ((buf[PMW3610_XY_H_POS] & 0xF0) << 4)), 12) / dividor;
    int16_t raw_y =
        TOINT16((buf[PMW3610_Y_L_POS] + ((buf[PMW3610_XY_H_POS] & 0x0F) << 8)), 12) / dividor;

    // まず元のCONFIG設定に基づく変換を適用
    if (IS_ENABLED(CONFIG_PMW3610_ORIENTATION_0)) {
        x = -raw_x;
        y = raw_y;
    } else if (IS_ENABLED(CONFIG_PMW3610_ORIENTATION_90)) {
        x = raw_y;
        y = -raw_x;
    } else if (IS_ENABLED(CONFIG_PMW3610_ORIENTATION_180)) {
        x = raw_x;
        y = -raw_y;
    } else if (IS_ENABLED(CONFIG_PMW3610_ORIENTATION_270)) {
        x = -raw_y;
        y = raw_x;
    }

    // 動的なorientation設定による追加回転を適用（0/45/90...315）
    uint16_t dynamic_rotation = pmw3610_get_orientation();
    if (dynamic_rotation != 0) {
        int16_t rx, ry;
        rotate_xy_45step(x, y, dynamic_rotation, &rx, &ry);
        x = rx;
        y = ry;
    }

    if (IS_ENABLED(CONFIG_PMW3610_INVERT_X)) {
        x = -x;
    }

    if (IS_ENABLED(CONFIG_PMW3610_INVERT_Y)) {
        y = -y;
    }

#ifdef CONFIG_PMW3610_SMART_ALGORITHM
    int16_t shutter =
        ((int16_t)(buf[PMW3610_SHUTTER_H_POS] & 0x01) << 8) + buf[PMW3610_SHUTTER_L_POS];
    if (data->sw_smart_flag && shutter < 45) {
        reg_write(dev, 0x32, 0x00);
        data->sw_smart_flag = false;
    }

    if (!data->sw_smart_flag && shutter > 45) {
        reg_write(dev, 0x32, 0x80);
        data->sw_smart_flag = true;
    }
#endif

#ifdef CONFIG_PMW3610_POLLING_RATE_125_SW
    int64_t curr_time = k_uptime_get();
    if (data->last_poll_time == 0 || curr_time - data->last_poll_time > 128) {
        data->last_poll_time = curr_time;
        data->last_x = x;
        data->last_y = y;
        return 0;
    } else {
        x += data->last_x;
        y += data->last_y;
        data->last_poll_time = 0;
        data->last_x = 0;
        data->last_y = 0;
    }
#endif

    if (x != 0 || y != 0) {
        if (input_mode == MOVE || input_mode == SNIPE) {
#if AUTOMOUSE_LAYER > 0
            int16_t movement_size = abs(x) + abs(y);
            if (input_mode == MOVE &&
                (automouse_triggered || zmk_keymap_highest_layer_active() != AUTOMOUSE_LAYER) &&
                movement_size > CONFIG_PMW3610_MOVEMENT_THRESHOLD) {
                activate_automouse_layer();
            }
#endif
            input_report_rel(dev, INPUT_REL_X, x, false, K_FOREVER);
            input_report_rel(dev, INPUT_REL_Y, y, true, K_FOREVER);
        } else if (input_mode == SCROLL) {
            data->scroll_delta_x += x;
            data->scroll_delta_y += y;
            if (abs(data->scroll_delta_y) > CONFIG_PMW3610_SCROLL_TICK) {
                input_report_rel(dev, INPUT_REL_WHEEL,
                                 data->scroll_delta_y > 0 ? PMW3610_SCROLL_Y_NEGATIVE
                                                          : PMW3610_SCROLL_Y_POSITIVE,
                                 true, K_FOREVER);
                data->scroll_delta_x = 0;
                data->scroll_delta_y = 0;
            } else if (abs(data->scroll_delta_x) > CONFIG_PMW3610_SCROLL_TICK) {
                input_report_rel(dev, INPUT_REL_HWHEEL,
                                 data->scroll_delta_x > 0 ? PMW3610_SCROLL_X_NEGATIVE
                                                          : PMW3610_SCROLL_X_POSITIVE,
                                 true, K_FOREVER);
                data->scroll_delta_x = 0;
                data->scroll_delta_y = 0;
            }
        } else if (input_mode == BALL_ACTION) {
            data->ball_action_delta_x += x;
            data->ball_action_delta_y += y;

            const struct pixart_config *config = dev->config;

            if (ball_action_idx != -1) {
                const struct ball_action_cfg action_cfg = *config->ball_actions[ball_action_idx];

                LOG_DBG("invoking ball action [%d], layer=%d", ball_action_idx,
                        zmk_keymap_highest_layer_active());

                struct zmk_behavior_binding_event event = {
                    .position = INT32_MAX,
                    .timestamp = k_uptime_get(),
#if IS_ENABLED(CONFIG_ZMK_SPLIT)
                    .source = ZMK_POSITION_STATE_CHANGE_SOURCE_LOCAL,
#endif
                };

                int idx = -1;
                if (abs(data->ball_action_delta_x) > action_cfg.tick) {
                    idx = data->ball_action_delta_x > 0 ? 0 : 1;
                } else if (abs(data->ball_action_delta_y) > action_cfg.tick) {
                    idx = data->ball_action_delta_y > 0 ? 3 : 2;
                }

                if (idx != -1) {
                    zmk_behavior_queue_add(&event, action_cfg.bindings[idx], true, action_cfg.tap_ms);
                    zmk_behavior_queue_add(&event, action_cfg.bindings[idx], false, action_cfg.wait_ms);

                    data->ball_action_delta_x = 0;
                    data->ball_action_delta_y = 0;
                }
            }
        }
    }

    return err;
}

static void pmw3610_gpio_callback(const struct device *gpiob, struct gpio_callback *cb,
                                  uint32_t pins) {
    struct pixart_data *data = CONTAINER_OF(cb, struct pixart_data, irq_gpio_cb);
    const struct device *dev = data->dev;

    set_interrupt(dev, false);

    k_work_submit(&data->trigger_work);
}

static void pmw3610_work_callback(struct k_work *work) {
    struct pixart_data *data = CONTAINER_OF(work, struct pixart_data, trigger_work);
    const struct device *dev = data->dev;

    pmw3610_report_data(dev);
    set_interrupt(dev, true);
}

static int pmw3610_init_irq(const struct device *dev) {
    LOG_INF("Configure irq...");

    int err;
    struct pixart_data *data = dev->data;
    const struct pixart_config *config = dev->config;

    if (!device_is_ready(config->irq_gpio.port)) {
        LOG_ERR("IRQ GPIO device not ready");
        return -ENODEV;
    }

    err = gpio_pin_configure_dt(&config->irq_gpio, GPIO_INPUT);
    if (err) {
        LOG_ERR("Cannot configure IRQ GPIO");
        return err;
    }

    gpio_init_callback(&data->irq_gpio_cb, pmw3610_gpio_callback, BIT(config->irq_gpio.pin));

    err = gpio_add_callback(config->irq_gpio.port, &data->irq_gpio_cb);
    if (err) {
        LOG_ERR("Cannot add IRQ GPIO callback");
    }

    LOG_INF("Configure irq done");

    return err;
}

static int pmw3610_init(const struct device *dev) {
    LOG_INF("Start initializing...");

    struct pixart_data *data = dev->data;
    const struct pixart_config *config = dev->config;
    int err;

    data->dev = dev;
    data->sw_smart_flag = false;

    k_work_init(&data->trigger_work, pmw3610_work_callback);

    if (!device_is_ready(config->cs_gpio.port)) {
        LOG_ERR("SPI CS device not ready");
        return -ENODEV;
    }

    err = gpio_pin_configure_dt(&config->cs_gpio, GPIO_OUTPUT_INACTIVE);
    if (err) {
        LOG_ERR("Cannot configure SPI CS GPIO");
        return err;
    }

    err = pmw3610_init_irq(dev);
    if (err) {
        return err;
    }

    k_work_init_delayable(&data->init_work, pmw3610_async_init);
    k_work_schedule(&data->init_work, K_MSEC(async_init_delay[data->async_init_step]));

    return err;
}

#define TRANSFORMED_BINDINGS(n)                                                                    \
    { LISTIFY(DT_PROP_LEN(n, bindings), ZMK_KEYMAP_EXTRACT_BINDING, (, ), n) }

#define BALL_ACTIONS_INST(n)                                                                       \
    static struct zmk_behavior_binding                                                             \
        ball_action_config_##n##_bindings[DT_PROP_LEN(n, bindings)] = TRANSFORMED_BINDINGS(n);     \
                                                                                                   \
    static struct ball_action_cfg ball_action_cfg_##n = {                                          \
        .bindings_len = DT_PROP_LEN(n, bindings),                                                  \
        .bindings = ball_action_config_##n##_bindings,                                             \
        .layers = DT_PROP(n, layers),                                                              \
        .layers_len = DT_PROP_LEN(n, layers),                                                      \
        .tick = DT_PROP_OR(n, tick, CONFIG_PMW3610_BALL_ACTION_TICK),                              \
        .wait_ms = DT_PROP_OR(n, wait_ms, 0),                                                      \
        .tap_ms = DT_PROP_OR(n, tap_ms, 0),                                                        \
    };

DT_INST_FOREACH_CHILD(0, BALL_ACTIONS_INST)

#define BALL_ACTIONS_ITEM(n) &ball_action_cfg_##n,
#define BALL_ACTIONS_UTIL_ONE(n) 1 +
#define BALL_ACTIONS_LEN (DT_INST_FOREACH_CHILD(0, BALL_ACTIONS_UTIL_ONE) 0)

#define PMW3610_DEFINE(n)                                                                          \
    static struct pixart_data data##n;                                                             \
    static int32_t scroll_layers##n[] = DT_PROP(DT_DRV_INST(n), scroll_layers);                    \
    static int32_t snipe_layers##n[] = DT_PROP(DT_DRV_INST(n), snipe_layers);                      \
    static struct ball_action_cfg *ball_actions[] = {DT_INST_FOREACH_CHILD(0, BALL_ACTIONS_ITEM)}; \
    static const struct pixart_config config##n = {                                                \
        .irq_gpio = GPIO_DT_SPEC_INST_GET(n, irq_gpios),                                           \
        .bus =                                                                                     \
            {                                                                                      \
                .bus = DEVICE_DT_GET(DT_INST_BUS(n)),                                              \
                .config =                                                                          \
                    {                                                                              \
                        .frequency = DT_INST_PROP(n, spi_max_frequency),                           \
                        .operation =                                                               \
                            SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_MODE_CPOL | SPI_MODE_CPHA,    \
                        .slave = DT_INST_REG_ADDR(n),                                              \
                    },                                                                             \
            },                                                                                     \
        .cs_gpio = SPI_CS_GPIOS_DT_SPEC_GET(DT_DRV_INST(n)),                                       \
        .scroll_layers = scroll_layers##n,                                                         \
        .scroll_layers_len = DT_PROP_LEN(DT_DRV_INST(n), scroll_layers),                           \
        .snipe_layers = snipe_layers##n,                                                           \
        .snipe_layers_len = DT_PROP_LEN(DT_DRV_INST(n), snipe_layers),                             \
        .ball_actions = ball_actions,                                                              \
        .ball_actions_len = BALL_ACTIONS_LEN,                                                      \
    };                                                                                             \
                                                                                                   \
    DEVICE_DT_INST_DEFINE(n, pmw3610_init, NULL, &data##n, &config##n, POST_KERNEL,                \
                          CONFIG_SENSOR_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(PMW3610_DEFINE)
