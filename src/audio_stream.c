#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/audio/dmic.h>
#include <zephyr/sys/byteorder.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdio.h>

#include "audio_stream.h"
#include "adpcm.h"

#define AUDIO_GAIN_DEFAULT 8.0f

#define AUDIO_SAMPLE_RATE 16000
#define AUDIO_SAMPLES_PER_BLOCK 512
#define AUDIO_BLOCK_SIZE (AUDIO_SAMPLES_PER_BLOCK * sizeof(int16_t))

#define ADPCM_SAMPLES_PER_NOTIFY 120
#define ADPCM_BYTES_PER_NOTIFY (ADPCM_SAMPLES_PER_NOTIFY / 2)
#define NOTIFY_HEADER_SIZE 1
#define NOTIFY_PAYLOAD_SIZE (NOTIFY_HEADER_SIZE + ADPCM_BYTES_PER_NOTIFY)

#define MIC_PWR_PIN 10

struct stream_ctx {
	struct audio_stream_config cfg;
	struct adpcm_state adpcm;
	uint8_t seq;
	struct k_mem_slab *slab;
};

static void audio_stream_thread_impl(void *p1, void *p2, void *p3)
{
	struct stream_ctx *ctx = (struct stream_ctx *)p1;
	struct audio_stream_config *cfg = &ctx->cfg;
	int ret;
	bool mic_powered = false;

	while (1) {
		if (device_is_ready(cfg->gpio1_dev)) {
			if (!mic_powered) {
				gpio_pin_configure(cfg->gpio1_dev, MIC_PWR_PIN, GPIO_OUTPUT_HIGH);
				gpio_pin_set(cfg->gpio1_dev, MIC_PWR_PIN, 1);
				k_msleep(100);
				mic_powered = true;
			}
		} else {
			printk("AUDIO_STREAM: gpio1_dev not ready\n");
			k_sleep(K_SECONDS(1));
			continue;
		}

		if (!device_is_ready(cfg->mic_dev)) {
			printk("AUDIO_STREAM: mic_dev not ready\n");
			k_sleep(K_SECONDS(1));
			continue;
		}

		if (!cfg->mem_slab) {
			printk("AUDIO_STREAM: no mem_slab provided\n");
			k_sleep(K_SECONDS(1));
			continue;
		}
		ctx->slab = cfg->mem_slab;

		struct pcm_stream_cfg stream_cfg = {
			.pcm_rate = AUDIO_SAMPLE_RATE,
			.pcm_width = 16,
			.block_size = AUDIO_BLOCK_SIZE,
			.mem_slab = ctx->slab,
		};

		struct dmic_cfg dcfg;
		memset(&dcfg, 0, sizeof(dcfg));
		dcfg.io.min_pdm_clk_freq = 1000000;
		dcfg.io.max_pdm_clk_freq = 3200000;
		dcfg.streams = &stream_cfg;
		dcfg.channel.req_chan_map_lo = dmic_build_channel_map(0, 0, PDM_CHAN_LEFT);
		dcfg.channel.req_num_chan = 1;
		dcfg.channel.req_num_streams = 1;

		int dmic_init_retries = 0;
		while (dmic_init_retries < 10) {
			if (dmic_configure(cfg->mic_dev, &dcfg) == 0) {
				break;
			}
			printk("AUDIO_STREAM: dmic_configure failed (retry %d)\n", dmic_init_retries);
			dmic_init_retries++;
			k_sleep(K_MSEC(200));
		}
		if (dmic_init_retries >= 10) {
			printk("AUDIO_STREAM: dmic_configure giving up, retry later\n");
			k_sleep(K_SECONDS(5));
			continue;
		}

		dmic_init_retries = 0;
		while (dmic_init_retries < 5) {
			if (dmic_trigger(cfg->mic_dev, DMIC_TRIGGER_START) == 0) {
				break;
			}
			printk("AUDIO_STREAM: dmic_trigger failed (retry %d)\n", dmic_init_retries);
			dmic_init_retries++;
			k_sleep(K_MSEC(200));
		}
		if (dmic_init_retries >= 5) {
			printk("AUDIO_STREAM: dmic_trigger giving up\n");
			dmic_trigger(cfg->mic_dev, DMIC_TRIGGER_STOP);
			k_sleep(K_SECONDS(5));
			continue;
		}

		memset(&ctx->adpcm, 0, sizeof(ctx->adpcm));
		ctx->seq = 0;

		printk("AUDIO_STREAM: PDM streaming started at %u Hz\n", AUDIO_SAMPLE_RATE);

		uint8_t adpcm_buf[AUDIO_SAMPLES_PER_BLOCK / 2];
		uint8_t notify_buf[NOTIFY_PAYLOAD_SIZE];

		while (1) {
			void *buffer;
			size_t size;
			ret = dmic_read(cfg->mic_dev, 0, &buffer, &size, SYS_FOREVER_MS);
			if (ret != 0) {
				printk("AUDIO_STREAM: DMIC read error: %d\n", ret);
				dmic_trigger(cfg->mic_dev, DMIC_TRIGGER_STOP);
				k_msleep(100);
				break;
			}

			int16_t *samples = (int16_t *)buffer;
			uint32_t count = size / sizeof(int16_t);

			for (uint32_t i = 0; i < count; i++) {
				const float fwGain = (ctx->cfg.oled_settings && ctx->cfg.oled_settings->mic_gain > 0)
					? (float)ctx->cfg.oled_settings->mic_gain
					: AUDIO_GAIN_DEFAULT;
				int32_t g = (int32_t)(samples[i] * fwGain);
				if (g > 32767) g = 32767;
				if (g < -32768) g = -32768;
				samples[i] = (int16_t)g;
			}

			adpcm_encode(&ctx->adpcm, samples, adpcm_buf, count);

			double sum_sq = 0;
			uint32_t zero_crossings = 0;
			int16_t last_sample = 0;

			for (uint32_t i = 0; i < count; i++) {
				double val = (double)samples[i] / 32768.0;
				sum_sq += val * val;

				if (i > 0) {
					if ((samples[i] > 0 && last_sample < 0) ||
					    (samples[i] < 0 && last_sample > 0)) {
						zero_crossings++;
					}
				}
				last_sample = samples[i];
			}

			double rms = sqrt(sum_sq / (double)count);

			uint32_t rms_raw = (uint32_t)(rms * 1000.0);
			sys_put_le32(rms_raw, &cfg->audio_level_buf[0]);
			sys_put_le32(zero_crossings, &cfg->audio_level_buf[4]);

			static double smoothed_rms = 0.0;
			const double decay_factor = 0.885;
			const double attack_factor = 0.12;

			if (rms > smoothed_rms) {
				smoothed_rms = (attack_factor * rms) + ((1.0 - attack_factor) * smoothed_rms);
			} else {
				smoothed_rms *= decay_factor;
			}

			if (cfg->oled_settings->led_enable &&
			    gpio_is_ready_dt(cfg->led_red) &&
			    gpio_is_ready_dt(cfg->led_green) &&
			    gpio_is_ready_dt(cfg->led_blue)) {
				if (smoothed_rms < 0.004) {
					gpio_pin_set_dt(cfg->led_red, 0);
					gpio_pin_set_dt(cfg->led_green, 0);
					gpio_pin_set_dt(cfg->led_blue, 0);
				} else if (smoothed_rms < 0.012) {
					gpio_pin_set_dt(cfg->led_red, 0);
					gpio_pin_set_dt(cfg->led_green, 1);
					gpio_pin_set_dt(cfg->led_blue, 1);
				} else if (smoothed_rms < 0.025) {
					gpio_pin_set_dt(cfg->led_red, 0);
					gpio_pin_set_dt(cfg->led_green, 1);
					gpio_pin_set_dt(cfg->led_blue, 0);
				} else if (smoothed_rms < 0.045) {
					gpio_pin_set_dt(cfg->led_red, 1);
					gpio_pin_set_dt(cfg->led_green, 1);
					gpio_pin_set_dt(cfg->led_blue, 0);
				} else {
					gpio_pin_set_dt(cfg->led_red, 1);
					gpio_pin_set_dt(cfg->led_green, 0);
					gpio_pin_set_dt(cfg->led_blue, 0);
				}
			}

			for (uint32_t offset = 0; offset < count; offset += ADPCM_SAMPLES_PER_NOTIFY) {
				uint32_t chunk_samples = ADPCM_SAMPLES_PER_NOTIFY;
				if (offset + chunk_samples > count) {
					chunk_samples = count - offset;
				}

				uint32_t chunk_bytes = chunk_samples / 2;
				uint32_t notify_len = NOTIFY_HEADER_SIZE + chunk_bytes;

				notify_buf[0] = 0x80 | ctx->seq;
				ctx->seq = (ctx->seq + 1) & 0x7F;
				memcpy(&notify_buf[1], &adpcm_buf[offset / 2], chunk_bytes);

				if (*cfg->deepsleep_send_once_audio) {
					*cfg->deepsleep_send_once_audio = false;
					bt_gatt_notify(NULL, &cfg->gatt_attrs[cfg->audio_stream_val_idx],
						       notify_buf, notify_len);
					bt_gatt_notify(NULL, &cfg->gatt_attrs[cfg->audio_val_idx],
						       cfg->audio_level_buf, 8);
				} else if (*cfg->bt_connected && !*cfg->deepsleep_block_notify) {
					int notify_ret;
					do {
						notify_ret = bt_gatt_notify(NULL,
							&cfg->gatt_attrs[cfg->audio_stream_val_idx],
							notify_buf, notify_len);
						if (notify_ret == -ENOBUFS) {
							k_yield();
						}
					} while (notify_ret == -ENOBUFS);
					if (notify_ret < 0 && notify_ret != -ENOTCONN) {
						printk("AUDIO_STREAM: notify err %d\n", notify_ret);
					}

					bt_gatt_notify(NULL, &cfg->gatt_attrs[cfg->audio_val_idx],
						       cfg->audio_level_buf, 8);
				}
			}

			k_mem_slab_free(ctx->slab, buffer);

			if (k_uptime_get_32() % 2000 < 100) {
				printk("AUDIO_STREAM: seq=%u rms=%u zcr=%u\n",
				       ctx->seq, rms_raw, zero_crossings);
			}
		}
	}
}

K_THREAD_STACK_DEFINE(audio_stream_stack, 3072);
static struct k_thread audio_stream_thread_data;
static struct stream_ctx audio_stream_ctx;

void audio_stream_start(const struct audio_stream_config *cfg)
{
	audio_stream_ctx.cfg = *cfg;
	audio_stream_ctx.seq = 0;
	memset(&audio_stream_ctx.adpcm, 0, sizeof(audio_stream_ctx.adpcm));
	audio_stream_ctx.slab = cfg->mem_slab;

	k_thread_create(&audio_stream_thread_data, audio_stream_stack,
			K_THREAD_STACK_SIZEOF(audio_stream_stack),
			audio_stream_thread_impl, &audio_stream_ctx, NULL, NULL,
			2, K_FP_REGS, K_NO_WAIT);
}
