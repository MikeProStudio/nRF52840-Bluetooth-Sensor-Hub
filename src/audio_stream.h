#ifndef AUDIO_STREAM_H
#define AUDIO_STREAM_H

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/toolchain.h>
#include <stdint.h>
#include <stdbool.h>

struct bt_gatt_attr;

struct oled_settings {
	uint8_t beacon_dwell_s;
	uint8_t qr_dwell_s;
	uint8_t imu_dwell_s;
	uint8_t mic_dwell_s;
	uint8_t view_override;
	uint8_t qr_enable;
	uint8_t contrast;
	uint8_t invert;
	uint8_t display_sleep_min;
	uint8_t led_enable;
	uint8_t deepsleep_enable;
	uint8_t deepsleep_interval;
	uint8_t deepsleep_oled;
} __packed;

struct audio_stream_config {
	const struct device *mic_dev;
	const struct device *gpio1_dev;
	const struct gpio_dt_spec *led_red;
	const struct gpio_dt_spec *led_green;
	const struct gpio_dt_spec *led_blue;
	uint8_t *audio_level_buf;
	struct k_mem_slab *mem_slab;
	const struct bt_gatt_attr *gatt_attrs;
	uint16_t audio_val_idx;
	uint16_t audio_stream_val_idx;
	volatile bool *bt_connected;
	volatile bool *deepsleep_block_notify;
	volatile bool *deepsleep_send_once_audio;
	struct oled_settings *oled_settings;
};

void audio_stream_start(const struct audio_stream_config *cfg);

#endif
