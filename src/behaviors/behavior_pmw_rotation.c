/* behavior_pmw_rotation.c */

#define DT_DRV_COMPAT zmk_behavior_pmw_rotation

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <drivers/behavior.h>
#include <zmk/behavior.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

// ドライバ側の関数を使用するために宣言
extern uint16_t pmw3610_get_orientation(void);
extern void pmw3610_set_orientation(uint16_t orientation);

struct behavior_pmw_rotation_config {};
struct behavior_pmw_rotation_data {};

static int behavior_pmw_rotation_init(const struct device *dev) {
    return 0;
}

static int on_keymap_binding_pressed(struct zmk_behavior_binding *binding,
                                     struct zmk_behavior_binding_event event) {
    // 現在の角度をドライバから取得
    uint16_t current = pmw3610_get_orientation();

    // 45度加算して更新
    uint16_t next = (current + 45) % 360;

    // ドライバに新しい角度をセット
    pmw3610_set_orientation(next);

    LOG_INF("PMW rotation changed: %u -> %u", current, next);

    return ZMK_BEHAVIOR_OPAQUE;
}

static int on_keymap_binding_released(struct zmk_behavior_binding *binding,
                                      struct zmk_behavior_binding_event event) {
    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api behavior_pmw_rotation_driver_api = {
    .binding_pressed = on_keymap_binding_pressed,
    .binding_released = on_keymap_binding_released,
};

#define DEFINE_PMW_ROTATION(inst)                                              \
    static struct behavior_pmw_rotation_data behavior_pmw_rotation_data_##inst = {}; \
    static const struct behavior_pmw_rotation_config behavior_pmw_rotation_config_##inst = {}; \
    BEHAVIOR_DT_INST_DEFINE(inst, behavior_pmw_rotation_init, NULL,            \
                            &behavior_pmw_rotation_data_##inst,                \
                            &behavior_pmw_rotation_config_##inst,              \
                            POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,  \
                            &behavior_pmw_rotation_driver_api);

DT_INST_FOREACH_STATUS_OKAY(DEFINE_PMW_ROTATION)
