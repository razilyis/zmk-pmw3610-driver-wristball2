/* behavior_pmw_rotation.c
 *
 * Copyright (c) 2024 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_behavior_pmw_rotation

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>

#include <drivers/behavior.h>
#include <zmk/behavior.h>
#include <zmk/event_manager.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

struct behavior_pmw_rotation_config {};

struct behavior_pmw_rotation_data {
    uint16_t current_orientation;
};

// グローバル変数として現在の向きを管理（0,45,90,...,315）
static uint16_t current_pmw_orientation = 0;
static bool orientation_loaded_from_settings = false;

// settings用のキー
#define PMW_ORIENTATION_SETTING_KEY "pmw/orientation"

static inline bool valid_orientation(uint16_t deg) {
    // 0..359 かつ 45度刻み
    return (deg < 360) && ((deg % 45) == 0);
}

static inline uint16_t normalize_orientation(uint16_t deg) {
    // 念のため0..359に丸める
    return (uint16_t)(deg % 360);
}

// Settings handler function
static int pmw_rotation_settings_handler(const char *key, size_t len, settings_read_cb read_cb,
                                        void *cb_arg) {
    const char *next;

    // Parse the key to match "orientation"
    if (settings_name_steq(key, "orientation", &next) && !next) {
        // Validate data size
        if (len != sizeof(current_pmw_orientation)) {
            LOG_WRN("Invalid orientation settings size: %d", (int)len);
            return -EINVAL;
        }

        // Read the data using the callback
        int ret = read_cb(cb_arg, &current_pmw_orientation, sizeof(current_pmw_orientation));
        if (ret >= 0) {
            current_pmw_orientation = normalize_orientation(current_pmw_orientation);

            // Validate orientation value (0,45,90,...,315)
            if (!valid_orientation(current_pmw_orientation)) {
                LOG_WRN("Invalid orientation value %u, using default 0",
                        (unsigned int)current_pmw_orientation);
                current_pmw_orientation = 0;
            }

            orientation_loaded_from_settings = true;
            LOG_INF("Loaded orientation from settings: %u degrees",
                    (unsigned int)current_pmw_orientation);
        } else {
            LOG_WRN("Failed to read orientation from settings: %d", ret);
        }
        return ret;
    }
    return 0;
}

// Register settings handler
SETTINGS_STATIC_HANDLER_DEFINE(pmw_rotation, "pmw", NULL, pmw_rotation_settings_handler, NULL, NULL);

static int save_orientation_setting(uint16_t orientation) {
    int ret = settings_save_one(PMW_ORIENTATION_SETTING_KEY, &orientation, sizeof(orientation));
    if (ret) {
        LOG_WRN("Failed to save orientation to settings: %d", ret);
    } else {
        LOG_DBG("Orientation %u degrees saved to settings", (unsigned int)orientation);
    }
    return ret;
}

uint16_t pmw3610_get_orientation(void) {
    return current_pmw_orientation;
}

void pmw3610_set_orientation(uint16_t orientation) {
    uint16_t o = normalize_orientation(orientation);

    // 45度刻み以外が来たら無視/0にする（好みで変えてOK）
    if (!valid_orientation(o)) {
        LOG_WRN("Reject invalid orientation %u (must be 0..359, step 45). Using 0",
                (unsigned int)o);
        o = 0;
    }

    current_pmw_orientation = o;
    save_orientation_setting(o);
    LOG_INF("PMW3610 orientation changed to %u degrees", (unsigned int)o);
}

static int behavior_pmw_rotation_init(const struct device *dev) {
    struct behavior_pmw_rotation_data *data = dev->data;

    // settings がまだロードされていないタイミングで 0 を保存して上書きしない
    // （ロード後に settings_handler が current_pmw_orientation を反映する）
    if (!orientation_loaded_from_settings) {
        current_pmw_orientation = 0; // RAM上のデフォルトだけ
    }

    data->current_orientation = current_pmw_orientation;

    LOG_INF("PMW rotation behavior initialized with orientation: %u",
            (unsigned int)current_pmw_orientation);
    return 0;
}

static int on_keymap_binding_pressed(struct zmk_behavior_binding *binding,
                                     struct zmk_behavior_binding_event event) {
    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    struct behavior_pmw_rotation_data *data = dev->data;

    // 現在の角度を取得
    uint16_t current_angle = current_pmw_orientation;

    // 角度を45度ずつ回転: 0 -> 45 -> ... -> 315 -> 0
    uint16_t new_angle = (uint16_t)((current_angle + 45) % 360);

    // 新しい角度を設定
    pmw3610_set_orientation(new_angle);
    data->current_orientation = new_angle;

    LOG_INF("PMW rotation changed from %u to %u degrees",
            (unsigned int)current_angle, (unsigned int)new_angle);

    return ZMK_BEHAVIOR_OPAQUE;
}

static int on_keymap_binding_released(struct zmk_behavior_binding *binding,
                                      struct zmk_behavior_binding_event event) {
    // キー解放時は何もしない
    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api behavior_pmw_rotation_driver_api = {
    .binding_pressed = on_keymap_binding_pressed,
    .binding_released = on_keymap_binding_released,
};

#define DEFINE_PMW_ROTATION(inst)                                                                  \
    static struct behavior_pmw_rotation_data behavior_pmw_rotation_data_##inst = {};               \
    static const struct behavior_pmw_rotation_config behavior_pmw_rotation_config_##inst = {};    \
    BEHAVIOR_DT_INST_DEFINE(inst, behavior_pmw_rotation_init, NULL,                                \
                            &behavior_pmw_rotation_data_##inst,                                    \
                            &behavior_pmw_rotation_config_##inst,                                  \
                            POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,                      \
                            &behavior_pmw_rotation_driver_api);

DT_INST_FOREACH_STATUS_OKAY(DEFINE_PMW_ROTATION)
