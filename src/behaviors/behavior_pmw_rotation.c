/*
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

// グローバル変数として現在の向きを管理
static uint16_t current_pmw_orientation = 0; // 0, 90, 180, 270
static bool orientation_loaded_from_settings = false;

// settings用のキー
#define PMW_ORIENTATION_SETTING_KEY "pmw/orientation"

// Settings handler function
static int pmw_rotation_settings_handler(const char *key, size_t len, settings_read_cb read_cb,
                                        void *cb_arg) {
    const char *next;
    
    // Parse the key to match "orientation"
    if (settings_name_steq(key, "orientation", &next) && !next) {
        // Validate data size
        if (len != sizeof(current_pmw_orientation)) {
            LOG_WRN("Invalid orientation settings size: %d", len);
            return -EINVAL;
        }
        
        // Read the data using the callback
        int ret = read_cb(cb_arg, &current_pmw_orientation, sizeof(current_pmw_orientation));
        if (ret >= 0) {
            // Validate orientation value (0, 90, 180, 270)
            if (current_pmw_orientation != 0 && current_pmw_orientation != 90 && 
                current_pmw_orientation != 180 && current_pmw_orientation != 270) {
                LOG_WRN("Invalid orientation value %d, using default 0", current_pmw_orientation);
                current_pmw_orientation = 0;
            }
            orientation_loaded_from_settings = true;
            LOG_INF("Loaded orientation from settings: %d degrees", current_pmw_orientation);
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
        LOG_DBG("Orientation %d degrees saved to settings", orientation);
    }
    return ret;
}

uint16_t pmw3610_get_orientation(void) {
    return current_pmw_orientation;
}

void pmw3610_set_orientation(uint16_t orientation) {
    current_pmw_orientation = orientation;
    save_orientation_setting(orientation);
    LOG_INF("PMW3610 orientation changed to %d degrees", orientation);
}

static int behavior_pmw_rotation_init(const struct device *dev) {
    struct behavior_pmw_rotation_data *data = dev->data;
    
    // 設定が読み込まれていない場合は0度に設定
    if (!orientation_loaded_from_settings) {
        current_pmw_orientation = 0;
        save_orientation_setting(0);
    }
    
    data->current_orientation = current_pmw_orientation;
    
    LOG_INF("PMW rotation behavior initialized with orientation: %d", current_pmw_orientation);
    return 0;
}

static int on_keymap_binding_pressed(struct zmk_behavior_binding *binding,
                                     struct zmk_behavior_binding_event event) {
    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    struct behavior_pmw_rotation_data *data = dev->data;

    // 現在の角度を取得
    uint16_t current_angle = current_pmw_orientation;
    
    // 角度を90度ずつ回転: 0 -> 90 -> 180 -> 270 -> 0
    uint16_t new_angle;
    switch (current_angle) {
        case 0:
            new_angle = 90;
            break;
        case 90:
            new_angle = 180;
            break;
        case 180:
            new_angle = 270;
            break;
        case 270:
        default:
            new_angle = 0;
            break;
    }
    
    // 新しい角度を設定
    pmw3610_set_orientation(new_angle);
    data->current_orientation = new_angle;
    
    LOG_INF("PMW rotation changed from %d to %d degrees", current_angle, new_angle);

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

#define DEFINE_PMW_ROTATION(inst) \
    static struct behavior_pmw_rotation_data behavior_pmw_rotation_data_##inst = {}; \
    static const struct behavior_pmw_rotation_config behavior_pmw_rotation_config_##inst = {}; \
    BEHAVIOR_DT_INST_DEFINE(inst, behavior_pmw_rotation_init, NULL, \
        &behavior_pmw_rotation_data_##inst, &behavior_pmw_rotation_config_##inst, \
        POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, \
        &behavior_pmw_rotation_driver_api);

DT_INST_FOREACH_STATUS_OKAY(DEFINE_PMW_ROTATION)
