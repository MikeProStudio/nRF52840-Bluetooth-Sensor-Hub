#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/hci_vs.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/settings/settings.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/audio/dmic.h>
#include <zephyr/sys/byteorder.h>
#include <sdc_hci_vs.h>
#include <hal/nrf_power.h>
#include <hal/nrf_ficr.h>
#include <nfc_t2t_lib.h>
#include <nfc/ndef/uri_msg.h>
#include <nfc/ndef/le_oob_rec.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <zephyr/drivers/display.h>
#include "font8x8.h"
#include "font8x16.h"
#include "qrcodegen.h"
#include "audio_stream.h"

/* --- Hardware Definitions --- */
#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)
#define LED2_NODE DT_ALIAS(led2)
static const struct gpio_dt_spec led_red_spec = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec led_green_spec = GPIO_DT_SPEC_GET(LED1_NODE, gpios);
static const struct gpio_dt_spec led_blue_spec = GPIO_DT_SPEC_GET(LED2_NODE, gpios);

const struct device *const imu_dev = DEVICE_DT_GET(DT_ALIAS(imu));
const struct device *const mic_dev = DEVICE_DT_GET(DT_ALIAS(mic));

/* GPIOs */
static const struct device *gpio0_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
static const struct device *gpio1_dev = DEVICE_DT_GET(DT_NODELABEL(gpio1));

/* Pins */
#define SENSE_PWR_PIN 10 /* P1.10 - Sensor Power Enable */
#define READ_BAT_PIN  14 /* P0.14 - Battery Voltage Divider Enable (Active Low) */
#define CHG_STAT_PIN  17 /* P0.17 - BQ25100 /CHG Pin (Active Low) */
#define HICHG_PIN     13 /* P0.13 - High Charge Current Enable (Active Low) */

/* Battery Reading (ADC) */
static const struct adc_dt_spec adc_channel = ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 0);

/* --- BLE GATT Service --- */
#define BT_UUID_CUSTOM_SERVICE_VAL BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef0)
#define BT_UUID_ACCEL_CHRC_VAL     BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef1)
#define BT_UUID_GYRO_CHRC_VAL      BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef2)
#define BT_UUID_AUDIO_CHRC_VAL     BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef3)
#define BT_UUID_TX_POWER_CHRC_VAL  BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef4)
#define BT_UUID_BATTERY_CHRC_VAL   BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef5)
#define BT_UUID_RESET_CHRC_VAL     BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef6)
#define BT_UUID_AUDIO_STREAM_CHRC_VAL BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef8)
#define BT_UUID_OLED_SETTINGS_CHRC_VAL BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef7)
#define BT_UUID_LIVE_AUDIO_CTRL_CHRC_VAL BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef9)

#define BT_UUID_CUSTOM_SERVICE  BT_UUID_DECLARE_128(BT_UUID_CUSTOM_SERVICE_VAL)
#define BT_UUID_ACCEL_CHRC      BT_UUID_DECLARE_128(BT_UUID_ACCEL_CHRC_VAL)
#define BT_UUID_GYRO_CHRC       BT_UUID_DECLARE_128(BT_UUID_GYRO_CHRC_VAL)
#define BT_UUID_AUDIO_CHRC      BT_UUID_DECLARE_128(BT_UUID_AUDIO_CHRC_VAL)
#define BT_UUID_AUDIO_STREAM_CHRC BT_UUID_DECLARE_128(BT_UUID_AUDIO_STREAM_CHRC_VAL)
#define BT_UUID_TX_POWER_CHRC   BT_UUID_DECLARE_128(BT_UUID_TX_POWER_CHRC_VAL)
#define BT_UUID_BATTERY_CHRC    BT_UUID_DECLARE_128(BT_UUID_BATTERY_CHRC_VAL)
#define BT_UUID_RESET_CHRC      BT_UUID_DECLARE_128(BT_UUID_RESET_CHRC_VAL)
#define BT_UUID_OLED_SETTINGS_CHRC BT_UUID_DECLARE_128(BT_UUID_OLED_SETTINGS_CHRC_VAL)
#define BT_UUID_LIVE_AUDIO_CTRL_CHRC BT_UUID_DECLARE_128(BT_UUID_LIVE_AUDIO_CTRL_CHRC_VAL)

/* Sichere Indizes für GATT-Charakteristiken (verhindert Magic Numbers) */
enum {
	IDX_CUSTOM_SVC = 0,
	IDX_ACCEL_VAL = 2,
	IDX_GYRO_VAL = 5,
	IDX_AUDIO_VAL = 8,
	IDX_AUDIO_STREAM_VAL = 11,
	IDX_TX_POWER_VAL = 14,
	IDX_BATTERY_VAL = 17,
	IDX_RESET_VAL = 20,
	IDX_OLED_SETTINGS_VAL = 22,
	IDX_LIVE_AUDIO_CTRL_VAL = 24,
};

static volatile bool live_audio_active;

/* Data structures for BLE transmission */
struct imu_data_t {
	uint8_t data[12]; /* 3x int32_t (x,y,z) serialized as LE */
};
struct batt_data_t {
    uint16_t voltage_mv;
    uint8_t soc;
    uint8_t status; /* 0=Discharging, 1=Charging, 2=USB Full */
} __packed;

static struct imu_data_t accel_data, gyro_data;
static uint8_t audio_level_buf[8]; /* uint32_t RMS + uint32_t ZCR */
static int8_t tx_power_level = 0; /* Default start value */
static struct batt_data_t battery_status = {0};

/* OLED Settings struct defined in audio_stream.h */

static struct oled_settings oled_settings = {
	.beacon_dwell_s = 2,
	.qr_dwell_s = 10,
	.imu_dwell_s = 8,
	.power_dwell_s = 5,
	.mic_dwell_s = 5,
	.view_override = 0,
	.qr_enable = 1,
	.contrast = 128,
	.invert = 0,
	.display_sleep_min = 0,
	.led_enable = 1,
	.deepsleep_enable = 0,
	.deepsleep_interval = 1,
	.deepsleep_oled = 0,
	.tx_power = 0,
};

static uint64_t oled_last_activity;

/* Deepsleep */
#define DEEP_SLEEP_MAGIC 0x42
static struct k_work_delayable deepsleep_work;

static uint32_t deepsleep_interval_sec(uint8_t idx)
{
	switch (idx) {
	case 0:  return 10;
	case 1:  return 60;
	case 2:  return 600;
	case 3:  return 3600;
	case 4:  return 36000;
	case 5:  return 86400;
	default: return 60;
	}
}

/* OLED Display (SSD1306 128x64, P0.04=SDA, P0.05=SCL, VCC=P1.11) */
#include <hal/nrf_gpio.h>
#define OLED_NODE DT_NODELABEL(ssd1306)
static const struct device *const oled_dev = DEVICE_DT_GET(OLED_NODE);
#define OLED_W 128
#define OLED_H 64
#define OLED_FB_SIZE (OLED_W * OLED_H / 8)
static uint8_t oled_fb[OLED_FB_SIZE];

/* Eigene preemptive Workqueue für OLED (Prio 4, niedriger als Audio-Thread Prio 2),
   damit Audio-Durchsatz nicht durch I2C-Bitbang-Updates blockiert wird */
static struct k_work_q oled_work_q;
K_THREAD_STACK_DEFINE(oled_work_q_stack, 2048);

/* VCC fuer OLED P1.11 via HAL vor allen anderen Treibern einschalten */
static int oled_vcc_init(void)
{
	nrf_gpio_cfg(43,
		     NRF_GPIO_PIN_DIR_OUTPUT,
		     NRF_GPIO_PIN_INPUT_DISCONNECT,
		     NRF_GPIO_PIN_NOPULL,
		     NRF_GPIO_PIN_H0H1,
		     NRF_GPIO_PIN_NOSENSE);
	nrf_gpio_pin_set(43);
	return 0;
}
SYS_INIT(oled_vcc_init, POST_KERNEL, 70);

/* Forward Declarations */
static void read_battery_voltage_impl(void);
static void update_tx_power_based_on_battery_impl(void);
static void start_advertising(void);
static void oled_check_sleep(void);
static void enter_deep_sleep(uint32_t sec);
static void wake_from_deep_sleep(void);
static void oled_clear(void);
static void oled_puts(int16_t x, int16_t y, const char *str);
static void oled_flush(void);
static void oled_apply_settings(void);
static void update_tx_power_based_on_battery_impl(void);
static int oled_init(void);
static k_tid_t main_thread_id;

/* OLED + BT state */
static bool bt_connected_flag;
static bool adv_should_run = true;
static uint8_t oled_phase;
static struct k_work_delayable oled_work;
static uint8_t oled_sub_phase;
static uint64_t imu_view_start;
static uint64_t mic_view_start;
static uint64_t page_start_time;


/* Phase definitions */
#define PHASE_WELCOME   0
#define PHASE_BEACON    1
#define PHASE_QR        2
#define PHASE_CONNECTED 3

/* QR Code */
#define QR_VERSION 5
#define QR_BUF_LEN qrcodegen_BUFFER_LEN_FOR_VERSION(QR_VERSION)
static uint8_t qr_temp_buf[QR_BUF_LEN];
static uint8_t qr_code_buf[QR_BUF_LEN];
static bool qr_code_ready;
static uint64_t qr_view_start;

/* --- Power Management (Asynchron) --- */
static struct k_work_delayable batt_work;
static struct k_work_delayable status_work;
static uint8_t last_power_status = 255;

static void status_work_handler(struct k_work *work)
{
	uint8_t current_status = 0;
	if (gpio_pin_configure(gpio0_dev, CHG_STAT_PIN, GPIO_INPUT | GPIO_PULL_UP) == 0) {
		bool is_charging = (gpio_pin_get(gpio0_dev, CHG_STAT_PIN) == 0);
		bool vbus_present = (nrf_power_usbregstatus_get(NRF_POWER) & NRF_POWER_USBREGSTATUS_VBUSDETECT_MASK);

		if (is_charging) {
			current_status = 1; /* Charging */
			gpio_pin_configure(gpio0_dev, HICHG_PIN, GPIO_OUTPUT_HIGH); 
		} else if (vbus_present) {
			current_status = 2; /* USB Power / Full */
			gpio_pin_configure(gpio0_dev, HICHG_PIN, GPIO_OUTPUT_LOW);
		} else {
			current_status = 0; /* Battery Mode */
			gpio_pin_configure(gpio0_dev, HICHG_PIN, GPIO_OUTPUT_LOW);
		}
	}
	
	/* Notify immediately if status changes */
	if (current_status != last_power_status) {
		battery_status.status = current_status;
		last_power_status = current_status;
		update_tx_power_based_on_battery_impl();
	}
	/* Poll state every 1 second */
	k_work_reschedule(&status_work, K_SECONDS(1));
}

static void batt_work_handler(struct k_work *work)
{
	read_battery_voltage_impl();
	update_tx_power_based_on_battery_impl();
	
	/* Smart Polling: 1s interval for the first 30 seconds, then 10s interval as requested */
	if (k_uptime_get_32() < 30000) {
		k_work_reschedule(&batt_work, K_SECONDS(1));
	} else {
		k_work_reschedule(&batt_work, K_SECONDS(10));
	}
}

/* --- BLE Security Callbacks --- */
static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	printk("*********************************\n");
	printk("STATIC PASSKEY REQUIRED: %06u\n", passkey);
	printk("*********************************\n");
}

static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	printk("Pairing cancelled: %s\n", addr);
}

static void pairing_complete(struct bt_conn *conn, bool bonded)
{
	printk("Pairing complete. Bonded: %s\n", bonded ? "yes" : "no");
}

static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
	printk("Pairing failed (reason %d)\n", reason);
}

static struct bt_conn_auth_cb auth_cb_display = {
	.passkey_display = auth_passkey_display,
	.passkey_entry = NULL,
	.cancel = auth_cancel,
};

static struct bt_conn_auth_info_cb auth_info_cb = {
	.pairing_complete = pairing_complete,
	.pairing_failed = pairing_failed,
};

/* Set TX Power via SDC direct API (avoids HCI transport deadlock during active connections) */
static int set_bt_tx_power(int8_t power_dbm) {
    uint8_t ret;
    sdc_hci_cmd_vs_zephyr_write_tx_power_t params;
    sdc_hci_cmd_vs_zephyr_write_tx_power_return_t resp;

    params.handle_type = SDC_HCI_VS_TX_POWER_HANDLE_TYPE_ADV;
    params.handle = 0;
    params.tx_power_level = power_dbm;

    ret = sdc_hci_cmd_vs_zephyr_write_tx_power(&params, &resp);
    if (ret) {
        printk("TX Power (ADV) SDC failed: 0x%02x\n", ret);
    } else {
        printk("TX Power (ADV): selected=%d dBm (req=%d dBm)\n",
               resp.selected_tx_power, power_dbm);
        tx_power_level = resp.selected_tx_power;
    }

    params.handle_type = SDC_HCI_VS_TX_POWER_HANDLE_TYPE_CONN;
    ret = sdc_hci_cmd_vs_zephyr_write_tx_power(&params, &resp);
    if (ret) {
        printk("TX Power (CONN) SDC failed: 0x%02x\n", ret);
    } else {
        printk("TX Power (CONN): selected=%d dBm (req=%d dBm)\n",
               resp.selected_tx_power, power_dbm);
        tx_power_level = resp.selected_tx_power;
    }

    return 0;
}

/* ADC filter globals */
static int16_t sample_buffer[1];
static struct adc_sequence sequence = {
	.buffer = sample_buffer,
	.buffer_size = sizeof(sample_buffer),
};
#define FILTER_SIZE 12
static int32_t adc_history[FILTER_SIZE] = {0};
static int filter_idx = 0;
static bool filter_full = false;

/* ── Persist OLED settings to flash via Zephyr settings subsystem ── */
#define SETTINGS_KEY_OLED "skynet/oled"

static void ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value) {}

static int settings_set_oled(const char *key, size_t len, settings_read_cb read_cb, void *cb_arg)
{
	if (strcmp(key, "oled") != 0) return 0;
	ssize_t sz = read_cb(cb_arg, &oled_settings, sizeof(oled_settings));
	if (sz < 0) return sz;
	oled_apply_settings();
	printk("OLED settings restored from flash\n");
	return 0;
}

static int settings_export_oled(int (*cb)(const char *name, const void *value, size_t val_len))
{
	return cb(SETTINGS_KEY_OLED, &oled_settings, sizeof(oled_settings));
}

static struct settings_handler skynet_settings = {
	.name = "skynet",
	.h_set = settings_set_oled,
	.h_export = settings_export_oled,
};

static ssize_t read_tx_power(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			     void *buf, uint16_t len, uint16_t offset)
{
	/* Report our last known tx_power_level which is set via Nordic VS commands in update_tx_power_based_on_battery_impl */
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &tx_power_level, sizeof(tx_power_level));
}

static ssize_t read_battery(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			     void *buf, uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &battery_status, sizeof(battery_status));
}

static ssize_t write_reset(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			 const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	if (len != 1) return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	uint8_t val = ((uint8_t *)buf)[0];
	if (val == 1) {
		printk("SYSTEM RESET TO BOOTLOADER REQUESTED...\n");
		nrf_power_gpregret_set(NRF_POWER, 0, 0x57);
		NVIC_SystemReset();
	}
	if (val == 3) {
		printk("DEEP SLEEP TRIGGERED VIA RESET CHAR\n");
		oled_settings.deepsleep_enable = 1;
		k_work_cancel_delayable(&deepsleep_work);
		k_work_reschedule(&deepsleep_work, K_MSEC(200));
	}
	if (val == 4) {
		printk("WAKE FROM DEEP SLEEP VIA RESET CHAR\n");
		wake_from_deep_sleep();
	}
	if (val == 5) {
		printk("FORCE DISCONNECT + RESTART ADVERTISING\n");
		bt_connected_flag = false;
		adv_should_run = true;
		oled_phase = PHASE_BEACON;
		oled_sub_phase = 0;
		k_work_reschedule_for_queue(&oled_work_q, &oled_work, K_NO_WAIT);
		start_advertising();
	}
	return len;
}

static void oled_apply_settings(void)
{
	if (device_is_ready(oled_dev)) {
		display_set_contrast(oled_dev, oled_settings.contrast);
		display_set_pixel_format(oled_dev, oled_settings.invert ? PIXEL_FORMAT_MONO10 : PIXEL_FORMAT_MONO01);
		if (oled_settings.display_sleep_min == 0) {
			display_blanking_off(oled_dev);
		}
	}
	oled_last_activity = k_uptime_get();
	printk("OLED settings applied: contrast=%u invert=%u sleep=%umin led=%u\n",
	       oled_settings.contrast, oled_settings.invert, oled_settings.display_sleep_min, oled_settings.led_enable);
}

static ssize_t read_oled_settings(struct bt_conn *conn, const struct bt_gatt_attr *attr,
				  void *buf, uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &oled_settings, sizeof(oled_settings));
}

static ssize_t write_oled_settings(struct bt_conn *conn, const struct bt_gatt_attr *attr,
				   const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	if (offset + len > sizeof(oled_settings))
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	int8_t old_tx = oled_settings.tx_power;
	memcpy((uint8_t *)&oled_settings + offset, buf, len);
	oled_apply_settings();
	k_work_reschedule_for_queue(&oled_work_q, &oled_work, K_NO_WAIT);
	if (!oled_settings.deepsleep_enable) {
		k_work_cancel_delayable(&deepsleep_work);
		printk("Deepsleep disabled\n");
	}
	if (oled_settings.tx_power != 0 || old_tx != 0) {
		/* Re-evaluate TX power via battery workqueue (correct async context) */
		k_work_reschedule(&batt_work, K_NO_WAIT);
	}
	settings_save_one(SETTINGS_KEY_OLED, &oled_settings, sizeof(oled_settings));
	printk("OLED settings updated and saved to flash\n");
	return len;
}

static ssize_t write_live_audio_ctrl(struct bt_conn *conn,
				     const struct bt_gatt_attr *attr,
				     const void *buf, uint16_t len,
				     uint16_t offset, uint8_t flags)
{
	if (len != 1) return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	uint8_t val = ((uint8_t *)buf)[0];
	live_audio_active = (val != 0);
	printk("Live Audio: %s\n", live_audio_active ? "START" : "STOP");
	return len;
}

/* Service declaration */
BT_GATT_SERVICE_DEFINE(custom_svc,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_CUSTOM_SERVICE),
	/* Kein AUTHEN mehr: Windows-Bonding-Konflikt verhindert Verbindung nach erneutem Pairing */
	BT_GATT_CHARACTERISTIC(BT_UUID_ACCEL_CHRC, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_READ, NULL, NULL, NULL),
	BT_GATT_CCC(ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CHARACTERISTIC(BT_UUID_GYRO_CHRC, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_READ, NULL, NULL, NULL),
	BT_GATT_CCC(ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CHARACTERISTIC(BT_UUID_AUDIO_CHRC, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_READ, NULL, NULL, NULL),
	BT_GATT_CCC(ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CHARACTERISTIC(BT_UUID_AUDIO_STREAM_CHRC, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_READ, NULL, NULL, NULL),
	BT_GATT_CCC(ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CHARACTERISTIC(BT_UUID_TX_POWER_CHRC, BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_READ, read_tx_power, NULL, NULL),
	BT_GATT_CCC(ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CHARACTERISTIC(BT_UUID_BATTERY_CHRC, BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_READ, read_battery, NULL, NULL),
	BT_GATT_CCC(ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CHARACTERISTIC(BT_UUID_RESET_CHRC, BT_GATT_CHRC_WRITE, BT_GATT_PERM_WRITE, NULL, write_reset, NULL),
	BT_GATT_CHARACTERISTIC(BT_UUID_OLED_SETTINGS_CHRC, BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
			       BT_GATT_PERM_READ | BT_GATT_PERM_WRITE, read_oled_settings, write_oled_settings, &oled_settings),
	BT_GATT_CHARACTERISTIC(BT_UUID_LIVE_AUDIO_CTRL_CHRC, BT_GATT_CHRC_WRITE,
			       BT_GATT_PERM_WRITE, NULL, write_live_audio_ctrl, NULL),
);

/* --- Calibration & Timing --- */
static float calibration_factor = 2.961f; /* Standard 1M/510k divider */
#define BURST_SAMPLES 2                   /* Reduziert von 10, um Blocking zu minimieren */

static void read_battery_voltage_impl(void)
{
	int err;
	int32_t raw_sum = 0;
	int32_t val_mv;
	int samples_taken = 0;

	if (!adc_is_ready_dt(&adc_channel) || !device_is_ready(gpio0_dev)) {
		printk("Devices not ready for Batt read\n");
		return;
	}

	/* 1. Hardware vorbereiten */
	err = gpio_pin_configure(gpio0_dev, READ_BAT_PIN, GPIO_OUTPUT);
	gpio_pin_set(gpio0_dev, READ_BAT_PIN, 0);
	k_msleep(50); /* Erhöht auf 50ms zum sauberen Einschwingen des Teilers */

	err = adc_channel_setup_dt(&adc_channel);
	if (err != 0) {
		gpio_pin_configure(gpio0_dev, READ_BAT_PIN, GPIO_INPUT);
		return;
	}

	/* 2. Burst-Messung */
	for (int i = 0; i < BURST_SAMPLES; i++) {
		(void)adc_sequence_init_dt(&adc_channel, &sequence);
		err = adc_read(adc_channel.dev, &sequence);
		if (err == 0) {
			int32_t m_mv = sample_buffer[0];
			adc_raw_to_millivolts_dt(&adc_channel, &m_mv);
			raw_sum += m_mv;
			samples_taken++;
		}
		k_msleep(10);
	}
	
	/* 3. Hardware sofort wieder aus */
	gpio_pin_configure(gpio0_dev, READ_BAT_PIN, GPIO_INPUT);

	if (samples_taken > 0) {
		val_mv = raw_sum / samples_taken;
		
		/* 4. Kalibrierung anwenden */
		val_mv = (int32_t)((float)val_mv * calibration_factor);

		int32_t old_val = (int32_t)battery_status.voltage_mv;

		/* 5. Bei erstmaligem oder großem Sprung (>500mV) SMA-Filter sofort füllen */
		if (old_val == 0 || abs(val_mv - old_val) > 500) {
			for (int i = 0; i < FILTER_SIZE; i++) {
				adc_history[i] = val_mv;
			}
			filter_idx = 0;
			filter_full = true;
		} else {
			/* SMA Filter (Langzeit-Glättung) */
			adc_history[filter_idx] = val_mv;
			filter_idx = (filter_idx + 1) % FILTER_SIZE;
			if (filter_idx == 0) filter_full = true;

			int32_t sum = 0;
			int count = filter_full ? FILTER_SIZE : filter_idx;
			for (int i = 0; i < count; i++) {
				sum += adc_history[i];
			}
			val_mv = sum / count;
		}

		battery_status.voltage_mv = (uint16_t)val_mv;

		/* SoC Logic: 3.3V to 4.15V LiPo range */
		if (val_mv >= 4150) battery_status.soc = 100;
		else if (val_mv <= 3300) battery_status.soc = 0;
		else battery_status.soc = (uint8_t)((val_mv - 3300) * 100 / (4150 - 3300));
	}
}

static void update_tx_power_based_on_battery_impl(void)
{
	if (oled_settings.tx_power != 0) {
		if (tx_power_level != oled_settings.tx_power) {
			set_bt_tx_power(oled_settings.tx_power);
		}
		return;
	}
	/* High power if > 20% OR on external power */
	int8_t target_power;
	if (battery_status.soc < 20 && battery_status.status == 0) {
		target_power = -4;
	} else {
		target_power = 8;
	}
	
	if (tx_power_level != target_power) {
		printk("Power Logic: SoC %d%% Status %d -> Set TX %d dBm\n", 
			battery_status.soc, battery_status.status, target_power);
		set_bt_tx_power(target_power);
	}

	/* Notify Battery Data */
	bt_gatt_notify(NULL, &custom_svc.attrs[IDX_BATTERY_VAL], &battery_status, sizeof(battery_status));
}

static uint32_t generated_passkey = 0;
static char device_name_with_pin[32] = "Skynet AI Beacon"; /* Wird überschrieben, bleibt als Fallback */

static struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, device_name_with_pin, 16) /* Name ins Hauptpaket für Smartphones */
};

static struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_CUSTOM_SERVICE_VAL), /* UUID ins Scan-Response */
};

/* Forward reference for advertising work */
static struct k_work adv_start_work;
static void adv_start_handler(struct k_work *work);

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		printk("Connection failed (err 0x%02x)\n", err);
	} else {
		printk("Connected.\n");
		bt_connected_flag = true;
		adv_should_run = false;
		oled_phase = PHASE_CONNECTED;
		oled_sub_phase = 0;
		page_start_time = k_uptime_get();
		imu_view_start = k_uptime_get();
		oled_check_sleep();
		k_work_reschedule_for_queue(&oled_work_q, &oled_work, K_NO_WAIT);
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	printk("Disconnected (reason 0x%02x)\n", reason);
	bt_connected_flag = false;
	adv_should_run = true;
	oled_phase = PHASE_BEACON;
	oled_sub_phase = 0;
	k_work_reschedule_for_queue(&oled_work_q, &oled_work, K_NO_WAIT);
	start_advertising();
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

static void phy_update(struct bt_conn *conn, struct bt_conn_le_phy_info *param)
{
	printk("PHY update: TX PHY %d, RX PHY %d\n", param->tx_phy, param->rx_phy);
}

static struct bt_conn_cb conn_callbacks_phy = {
	.le_phy_updated = phy_update,
};

static void start_advertising(void)
{
	int err;

	err = bt_le_adv_stop();
	if (err < 0 && err != -EALREADY) {
		printk("Adv stop err: %d\n", err);
	}

	k_sleep(K_MSEC(50));

	struct bt_le_adv_param adv_param = {
		.id = BT_ID_DEFAULT,
		.sid = 0,
		.options = BT_LE_ADV_OPT_CONN,
		.interval_min = BT_GAP_ADV_FAST_INT_MIN_2,
		.interval_max = BT_GAP_ADV_FAST_INT_MAX_2,
	};

	err = bt_le_adv_start(&adv_param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
	} else {
		printk("Advertising started. Skynet Beacon is visible.\n");
	}
}

static void adv_start_handler(struct k_work *work)
{
	start_advertising();
}

static void bt_ready_init(void)
{
		/* RGB Startup Sequence: R->G->B fade pulse, 10 cycles */
	for (int i = 0; i < 10; i++) {
		const struct gpio_dt_spec *specs[] = {&led_red_spec, &led_green_spec, &led_blue_spec};
		for (int s = 0; s < 3; s++) {
			if (!gpio_is_ready_dt(specs[s])) continue;

			/* Pseudo-fade using quick pulses (approx 75ms total per color) */
			for (int j = 0; j < 8; j++) { /* Fade Up */
				gpio_pin_set_dt(specs[s], 1);
				k_usleep(j * 550);
				gpio_pin_set_dt(specs[s], 0);
				k_usleep((8 - j) * 550);
			}
			gpio_pin_set_dt(specs[s], 1);
			k_msleep(15); /* Peak */
			for (int j = 8; j > 0; j--) { /* Fade Down */
				gpio_pin_set_dt(specs[s], 1);
				k_usleep(j * 550);
				gpio_pin_set_dt(specs[s], 0);
				k_usleep((8 - j) * 550);
			}
			gpio_pin_set_dt(specs[s], 0);
			k_msleep(50); /* Gap */
		}
	}	

	set_bt_tx_power(8);

	snprintf(device_name_with_pin, sizeof(device_name_with_pin), "Skynet AI Beacon");
	ad[1].data_len = strlen(device_name_with_pin);

	printk("************************************************\n");
	printk("SECURITY: Passkey Display (L4) enabled\n");
	printk("************************************************\n");

	start_advertising();
}

static int32_t my_sensor_value_to_milli(const struct sensor_value *val) { return (val->val1 * 1000) + (val->val2 / 1000); }

/* --- Audio Streaming --- */
K_MEM_SLAB_DEFINE(audio_mem_slab, 1024, 24, 4);

static struct k_thread sensor_thread_data;
K_THREAD_STACK_DEFINE(sensor_stack, 2048);

/* Deepsleep: OLED-only cycling, threads keep running for BLE stability */
#define DEEPSLEEP_DATA_WINDOW_MS 2500
static volatile bool deepsleep_pending;
static uint32_t deepsleep_sec_save;
static volatile bool deepsleep_block_notify;
static volatile bool deepsleep_send_once_audio;
static volatile bool deepsleep_send_once_sensor;

static void deepsleep_oled_sleep(void)
{
	if (!device_is_ready(oled_dev)) return;
	display_blanking_on(oled_dev);
}

static void deepsleep_oled_wake(void)
{
	if (!device_is_ready(oled_dev)) return;
	display_blanking_off(oled_dev);
	display_set_contrast(oled_dev, oled_settings.contrast);
	display_set_pixel_format(oled_dev, oled_settings.invert ? PIXEL_FORMAT_MONO10 : PIXEL_FORMAT_MONO01);
	oled_clear();
	oled_flush();
}

static void enter_deep_sleep(uint32_t sec)
{
	printk("DEEP SLEEP: entering cyclic OLED sleep interval=%us oled=%u\n", sec, oled_settings.deepsleep_oled);

	oled_settings.deepsleep_enable = 1;
	nrf_power_gpregret_set(NRF_POWER, 0, DEEP_SLEEP_MAGIC);
	nrf_power_gpregret_set(NRF_POWER, 1, oled_settings.deepsleep_interval);

	deepsleep_sec_save = sec;
	deepsleep_pending = true;
}

static void deepsleep_work_handler(struct k_work *work)
{
	if (oled_settings.deepsleep_enable) {
		enter_deep_sleep(deepsleep_interval_sec(oled_settings.deepsleep_interval));
	}
}

static void wake_from_deep_sleep(void)
{
	if (!deepsleep_pending) return;
	oled_settings.deepsleep_enable = 0;
	deepsleep_pending = false;
	deepsleep_block_notify = false;
	deepsleep_oled_wake();
	k_work_reschedule_for_queue(&oled_work_q, &oled_work, K_NO_WAIT);
	k_wakeup(main_thread_id);
	printk("WAKE FROM DEEP SLEEP completed\n");
}

static void sensor_thread(void *p1, void *p2, void *p3)
{
	struct sensor_value accel[3], gyro[3];
	int32_t ax, ay, az, gx, gy, gz;

	if (device_is_ready(imu_dev)) {
		struct sensor_value odr = { .val1 = 52, .val2 = 0 };
		sensor_attr_set(imu_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &odr);
		sensor_attr_set(imu_dev, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &odr);
	}

	while (1) {
		if (sensor_sample_fetch(imu_dev) == 0) {
			sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_XYZ, accel);
			sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_XYZ, gyro);
			
			ax = my_sensor_value_to_milli(&accel[0]);
			ay = my_sensor_value_to_milli(&accel[1]);
			az = my_sensor_value_to_milli(&accel[2]);
			gx = my_sensor_value_to_milli(&gyro[0]);
			gy = my_sensor_value_to_milli(&gyro[1]);
			gz = my_sensor_value_to_milli(&gyro[2]);
			
			sys_put_le32(ax, &accel_data.data[0]);
			sys_put_le32(ay, &accel_data.data[4]);
			sys_put_le32(az, &accel_data.data[8]);
			sys_put_le32(gx, &gyro_data.data[0]);
			sys_put_le32(gy, &gyro_data.data[4]);
			sys_put_le32(gz, &gyro_data.data[8]);
			
			if (deepsleep_send_once_sensor) {
				deepsleep_send_once_sensor = false;
				bt_gatt_notify(NULL, &custom_svc.attrs[IDX_ACCEL_VAL], &accel_data, sizeof(accel_data));
				bt_gatt_notify(NULL, &custom_svc.attrs[IDX_GYRO_VAL], &gyro_data, sizeof(gyro_data));
			} else if (!deepsleep_block_notify) {
				bt_gatt_notify(NULL, &custom_svc.attrs[IDX_ACCEL_VAL], &accel_data, sizeof(accel_data));
				bt_gatt_notify(NULL, &custom_svc.attrs[IDX_GYRO_VAL], &gyro_data, sizeof(gyro_data));
			}
		}
		k_msleep(20); /* Wiederhergestellt auf 50Hz wie vom User gewünscht */
	}
}

/* --- NFC Logic --- */
static uint8_t nfc_ndef_msg_buf[256]; /* Vergrößert auf 256 Bytes */

static void nfc_callback(void *context, nfc_t2t_event_t event, const uint8_t *data, size_t data_len)
{
	switch (event) {
	case NFC_T2T_EVENT_FIELD_ON:
		printk("NFC: Field detected (Smartphone nearby)\n");
		break;
	case NFC_T2T_EVENT_FIELD_OFF:
		printk("NFC: Field lost\n");
		break;
	default:
		break;
	}
}

static int setup_nfc(void)
{
	int err;
	size_t len = sizeof(nfc_ndef_msg_buf);

	err = nfc_t2t_setup(nfc_callback, NULL);
	if (err) {
		printk("NFC setup failed (err %d)\n", err);
		return err;
	}

	/* Create URI Record with dynamic PIN */
	char url[128];
	snprintf(url, sizeof(url), "mikeprostudio.github.io/nRF52840-Bluetooth-Sensor-Hub/?pin=%06u", generated_passkey);

	err = nfc_ndef_uri_msg_encode(NFC_URI_HTTPS, url, strlen(url), nfc_ndef_msg_buf, &len);
	if (err) {
		printk("NFC NDEF encode failed (err %d)\n", err);
		return err;
	}

	err = nfc_t2t_payload_set(nfc_ndef_msg_buf, len);
	if (err) {
		printk("NFC payload set failed (err %d)\n", err);
		return err;
	}

	err = nfc_t2t_emulation_start();
	if (err) {
		printk("NFC emulation start failed (err %d)\n", err);
		return err;
	}

	printk("NFC URI Tag ready. URL: https://%s\n", url);
	return 0;
}

/* --- OLED Display Functions --- */
static void oled_set_pixel(int16_t x, int16_t y, uint8_t on)
{
	if (x < 0 || x >= OLED_W || y < 0 || y >= OLED_H) return;
	uint16_t page = (uint16_t)y / 8;
	uint8_t bit = (uint8_t)(y % 8);
	uint16_t idx = page * OLED_W + (uint16_t)x;
	if (on) oled_fb[idx] |= (1 << bit);
	else    oled_fb[idx] &= ~(1 << bit);
}

static void oled_putc_w(int16_t x, int16_t y, char c, uint8_t w)
{
	if (c < 0x20 || c > 0x7E) c = ' ';
	if (w > 8) w = 8;
	uint8_t idx = c - 0x20;
	for (int row = 0; row < 8; row++) {
		int16_t cy = y + row;
		if (cy < 0 || cy >= OLED_H) continue;
		uint8_t bits = font8x8[idx][row];
		for (int col = 0; col < w; col++) {
			int16_t cx = x + col;
			if (cx >= OLED_W) break;
			if (cx >= 0 && (bits & (1 << (7 - col))))
				oled_set_pixel(cx, cy, 1);
		}
	}
}

static void oled_puts_w(int16_t x, int16_t y, const char *str, uint8_t w)
{
	while (*str) {
		oled_putc_w(x, y, *str, w);
		x += w;
		if (x + w > OLED_W) { x = 0; y += 8; }
		str++;
	}
}

static void oled_putc(int16_t x, int16_t y, char c)
{
	oled_putc_w(x, y, c, 8);
}

static void oled_puts(int16_t x, int16_t y, const char *str)
{
	oled_puts_w(x, y, str, 8);
}

static void oled_putc_8x16(int16_t x, int16_t y, char c)
{
	if (c < 0x20 || c > 0x7E) c = ' ';
	uint8_t idx = c - 0x20;
	for (int row = 0; row < 16; row++) {
		int16_t cy = y + row;
		if (cy < 0 || cy >= OLED_H) continue;
		uint8_t bits = font8x16[idx][row];
		for (int col = 0; col < 8; col++) {
			int16_t cx = x + col;
			if (cx >= OLED_W) break;
			if (cx >= 0 && (bits & (1 << (7 - col))))
				oled_set_pixel(cx, cy, 1);
		}
	}
}

static void oled_puts_8x16(int16_t x, int16_t y, const char *str)
{
	while (*str) {
		oled_putc_8x16(x, y, *str);
		x += 8;
		if (x + 8 > OLED_W) { x = 0; y += 16; }
		str++;
	}
}

static void oled_clear(void)
{
	memset(oled_fb, 0, sizeof(oled_fb));
}

static void oled_flush(void)
{
	struct display_buffer_descriptor desc = {
		.width = OLED_W,
		.height = OLED_H,
		.pitch = OLED_W,
		.buf_size = OLED_FB_SIZE,
	};
	display_write(oled_dev, 0, 0, &desc, oled_fb);
}

static int oled_init(void)
{
	if (!device_is_ready(oled_dev)) {
		printk("OLED device not ready\n");
		return -ENODEV;
	}
	display_blanking_off(oled_dev);
	oled_clear();
	oled_flush();
	if (IS_ENABLED(CONFIG_LOG)) printk("OLED initialized\n");
	return 0;
}

/* --- QR Code --- */
static void generate_qr_code(void)
{
	char url[96];
	snprintf(url, sizeof(url), "https://mikeprostudio.github.io/nRF52840-Bluetooth-Sensor-Hub/");
	qr_code_ready = qrcodegen_encodeText(url, qr_temp_buf, qr_code_buf,
	                                      qrcodegen_Ecc_LOW, 1, QR_VERSION, qrcodegen_Mask_AUTO, true);
}

static void oled_draw_qr_code(void)
{
	char buf[24];

	oled_clear();

	if (qr_code_ready) {
		int size = qrcodegen_getSize(qr_code_buf);
		int module_px = 2;
		int qr_pixels = size * module_px;
		int qr_x = 2, qr_y = (OLED_H - qr_pixels) / 2;
		for (int my = 0; my < size; my++) {
			for (int mx = 0; mx < size; mx++) {
				if (qrcodegen_getModule(qr_code_buf, mx, my)) {
					for (int dy = 0; dy < module_px; dy++) {
						for (int dx = 0; dx < module_px; dx++) {
							oled_set_pixel(qr_x + mx * module_px + dx, qr_y + my * module_px + dy, 1);
						}
					}
				}
			}
		}
	}

	int pin_x = (qr_code_ready && qrcodegen_getSize(qr_code_buf) * 2 + 2 < OLED_W)
	            ? qrcodegen_getSize(qr_code_buf) * 2 + 8 : OLED_W - 48;
	oled_puts_8x16(pin_x, (OLED_H - 32) / 2, "PIN:");
	snprintf(buf, sizeof(buf), "%06u", generated_passkey);
	oled_puts_8x16(pin_x, (OLED_H - 32) / 2 + 16, buf);

	oled_flush();
}

static void oled_update_display(void)
{
	char buf[24];

	oled_clear();

	snprintf(buf, sizeof(buf), "Skynet AI Beacon");
	oled_puts_8x16(0, 0, buf);

	snprintf(buf, sizeof(buf), "PIN: %06u", generated_passkey);
	oled_puts_8x16(0, 16, buf);

	snprintf(buf, sizeof(buf), "BAT: %umV %u%%", battery_status.voltage_mv, battery_status.soc);
	oled_puts_8x16(0, 32, buf);

	const char *status_str;
	switch (battery_status.status) {
	case 1:  status_str = "Charging";   break;
	case 2:  status_str = "USB Power";  break;
	default: status_str = "Battery";    break;
	}
	snprintf(buf, sizeof(buf), "PWR: %s", status_str);
	oled_puts_8x16(0, 48, buf);

	oled_flush();
}

/* --- OLED Mic PSD Visualization --- */
#define PSD_BINS 112
#define CHART_X 16
static uint8_t psd_ema[PSD_BINS];

static void oled_update_mic(void)
{
	uint32_t rms_raw = sys_get_le32(&audio_level_buf[0]);
	uint32_t zcr = sys_get_le32(&audio_level_buf[4]);
	double rms = (double)rms_raw / 1000.0;
	double freq_est = (double)zcr * 8000.0 / 1024.0;
	if (freq_est < 300.0) freq_est = 300.0;

	double dbfs = 20.0 * log10(rms + 0.0001);
	double dbspl = dbfs + 135.0;
	if (dbspl < 30.0) dbspl = 30.0;
	if (dbspl > 120.0) dbspl = 120.0;
	double max_bar = (dbspl - 30.0) / 90.0 * 47.0;

	oled_clear();

	for (int i = 0; i < PSD_BINS; i++) {
		double bin_freq = (double)i * 16000.0 / (double)(PSD_BINS * 2);
		double dist = bin_freq - freq_est;
		double val = exp(-(dist * dist) / 40000.0) * max_bar;
		double dist2 = bin_freq - freq_est * 2.0;
		val += exp(-(dist2 * dist2) / 20000.0) * max_bar * 0.35;
		val += exp(-(bin_freq * bin_freq) / 20000.0) * max_bar * 0.12;
		if (val < 0.3) val = 0.0;
		if (val > 47.0) val = 47.0;
		uint8_t v = (uint8_t)val;

		uint8_t ema = (uint8_t)((v > psd_ema[i]) ?
			(v * 0.5f + psd_ema[i] * 0.5f) :
			(psd_ema[i] * 0.90f));
		if (ema > 47) ema = 47;
		psd_ema[i] = ema;

		for (int y = 0; y < ema && y < 48; y++) {
			oled_set_pixel(CHART_X + i, 47 - y, 1);
		}
	}

	oled_puts_w(0, 0, "120", 4);
	oled_puts_w(0, 20, "75", 4);
	oled_puts_w(0, 39, "30", 4);
	oled_puts_w(CHART_X, 48, "0  2  4  6  8kHz", 6);
	char rms_str[16];
	snprintf(rms_str, sizeof(rms_str), "RMS %u", rms_raw);
	oled_puts_w(0, 56, rms_str, 6);

	oled_flush();
}

static void fmt_val_signed(char *buf, int32_t val)
{
	int neg = (val < 0);
	if (neg) val = -val;
	int ip = val / 1000;
	int fp = (val % 1000 + 5) / 10;
	if (fp >= 100) { ip++; fp = 0; }
	snprintf(buf, 10, "%s%d.%02u", neg ? "-" : "+", ip, (unsigned int)fp);
}

/* ── Scroll state ── */
#define SCROLL_STEPS 12
#define SCROLL_MS 30
static bool oled_scroll_active;
static uint8_t oled_scroll_step;
static uint8_t oled_scroll_from;
static uint8_t oled_scroll_to;

/* Cubic ease-out: slow start, fast middle, smooth stop */
static int ease_out_cubic(int step, int total)
{
	float t = (float)step / total;
	float eased = 1.0f - (1.0f - t) * (1.0f - t) * (1.0f - t);
	return (int)(eased * total + 0.5f);
}

/* IMU page: Acc X/Y/Z + Battery */
static void oled_render_imu_at(int y_offs)
{
	char buf[24], xs[10], ys[10], zs[10];

	int32_t ax = (int32_t)sys_get_le32(&accel_data.data[0]);
	int32_t ay = (int32_t)sys_get_le32(&accel_data.data[4]);
	int32_t az = (int32_t)sys_get_le32(&accel_data.data[8]);

	fmt_val_signed(xs, ax); fmt_val_signed(ys, ay); fmt_val_signed(zs, az);

	snprintf(buf, sizeof(buf), "Acc X%sm/s2", xs);
	oled_puts_8x16(0, 0 + y_offs, buf);
	snprintf(buf, sizeof(buf), "Acc Y%sm/s2", ys);
	oled_puts_8x16(0, 16 + y_offs, buf);
	snprintf(buf, sizeof(buf), "Acc Z%sm/s2", zs);
	oled_puts_8x16(0, 32 + y_offs, buf);
	snprintf(buf, sizeof(buf), "Bat %umV %u%%", battery_status.voltage_mv, battery_status.soc);
	oled_puts_8x16(0, 48 + y_offs, buf);
}

/* Gyro page: Gyr X, Y, Z each on own line + Power/TX */
static void oled_render_power_at(int y_offs)
{
	char buf[24], gxs[10], gys[10], gzs[10];

	int32_t gx = (int32_t)sys_get_le32(&gyro_data.data[0]);
	int32_t gy = (int32_t)sys_get_le32(&gyro_data.data[4]);
	int32_t gz = (int32_t)sys_get_le32(&gyro_data.data[8]);

	fmt_val_signed(gxs, gx); fmt_val_signed(gys, gy); fmt_val_signed(gzs, gz);

	const char *pwr;
	switch (battery_status.status) {
	case 1:  pwr = "CHG";  break;
	case 2:  pwr = "USB";  break;
	default: pwr = "BAT";  break;
	}

	snprintf(buf, sizeof(buf), "Gyr X%sd/s", gxs);
	oled_puts_8x16(0, 0 + y_offs, buf);
	snprintf(buf, sizeof(buf), "Gyr Y%sd/s", gys);
	oled_puts_8x16(0, 16 + y_offs, buf);
	snprintf(buf, sizeof(buf), "Gyr Z%sd/s", gzs);
	oled_puts_8x16(0, 32 + y_offs, buf);
	snprintf(buf, sizeof(buf), "P:%s TX %ddBm", pwr, tx_power_level);
	oled_puts_8x16(0, 48 + y_offs, buf);
}

/* Static display wrappers */
static void oled_show_imu(void)
{
	oled_clear();
	oled_render_imu_at(0);
	oled_flush();
}

static void oled_show_power(void)
{
	oled_clear();
	oled_render_power_at(0);
	oled_flush();
}

/* Scroll animation: new content scrolls DOWN from above into view.
   Old content goes the opposite direction (UP) out of view. */

static void oled_scroll_frame(void)
{
	int total = SCROLL_STEPS;
	int pos = ease_out_cubic(oled_scroll_step, total);
	int from_offs = -pos;           /* 0 → -64: old scrolls UP out (opposite of entry) */
	int to_offs = pos - OLED_H;     /* -64 → 0: new scrolls DOWN into view from above */

	oled_clear();
	if (oled_scroll_from == 0) oled_render_imu_at(from_offs);
	else oled_render_power_at(from_offs);
	if (oled_scroll_to == 0) oled_render_imu_at(to_offs);
	else oled_render_power_at(to_offs);
	oled_flush();

	oled_scroll_step++;
	if (oled_scroll_step >= total) {
		oled_scroll_active = false;
	}
}

static void oled_check_sleep(void)
{
	oled_last_activity = k_uptime_get();
	if (oled_settings.display_sleep_min > 0 && device_is_ready(oled_dev)) {
		display_blanking_off(oled_dev);
	}
}

static void oled_work_handler(struct k_work *work)
{
	if (oled_settings.display_sleep_min > 0 &&
	    k_uptime_get() - oled_last_activity > (int64_t)oled_settings.display_sleep_min * 60000) {
		if (device_is_ready(oled_dev)) {
			display_blanking_on(oled_dev);
		}
		k_work_reschedule_for_queue(&oled_work_q, &oled_work, K_SECONDS(30));
		return;
	}

	if (oled_phase == PHASE_WELCOME) {
		oled_clear();
		oled_puts_8x16(20, 24, "Welcome!");
		oled_flush();
		generate_qr_code();
		oled_sub_phase = 0;
		if (oled_settings.qr_enable) {
			oled_phase = PHASE_QR;
			qr_view_start = k_uptime_get();
		} else {
			oled_phase = PHASE_BEACON;
		}
		oled_last_activity = k_uptime_get();
		k_work_reschedule_for_queue(&oled_work_q, &oled_work, K_SECONDS(2));
	} else if (bt_connected_flag) {
		oled_last_activity = k_uptime_get();
		uint64_t now = k_uptime_get();

		/* During active scroll, advance animation step */
		if (oled_scroll_active) {
			oled_scroll_frame();
			if (oled_scroll_active) {
				k_work_reschedule_for_queue(&oled_work_q, &oled_work, K_MSEC(SCROLL_MS));
			} else {
				/* Scroll finished — show target page and start its timer */
				page_start_time = now;
				k_work_reschedule_for_queue(&oled_work_q, &oled_work, K_MSEC(150));
			}
		} else if (oled_sub_phase == 0) {
			/* Acc page: Acc X/Y/Z + Battery */
			oled_show_imu();
			if (oled_settings.view_override == 2) {
				oled_scroll_from = 0; oled_scroll_to = 2;
				oled_scroll_step = 0; oled_scroll_active = true;
				oled_sub_phase = 2;
				k_work_reschedule_for_queue(&oled_work_q, &oled_work, K_NO_WAIT);
			} else if (oled_settings.view_override == 1) {
				if (now - page_start_time >= (uint64_t)oled_settings.imu_dwell_s * 1000) {
					oled_scroll_from = 0; oled_scroll_to = 1;
					oled_scroll_step = 0; oled_scroll_active = true;
					oled_sub_phase = 1;
					k_work_reschedule_for_queue(&oled_work_q, &oled_work, K_NO_WAIT);
				} else {
					k_work_reschedule_for_queue(&oled_work_q, &oled_work, K_MSEC(150));
				}
			} else {
				if (now - page_start_time >= (uint64_t)oled_settings.imu_dwell_s * 1000) {
					oled_scroll_from = 0; oled_scroll_to = 1;
					oled_scroll_step = 0; oled_scroll_active = true;
					oled_sub_phase = 1;
					k_work_reschedule_for_queue(&oled_work_q, &oled_work, K_NO_WAIT);
				} else {
					k_work_reschedule_for_queue(&oled_work_q, &oled_work, K_MSEC(150));
				}
			}
		} else if (oled_sub_phase == 1) {
			/* Gyro page: Gyr X/Y/Z (each own line) + Pwr/TX */
			oled_show_power();
			if (oled_settings.view_override == 2) {
				oled_scroll_from = 1; oled_scroll_to = 2;
				oled_scroll_step = 0; oled_scroll_active = true;
				oled_sub_phase = 2;
				k_work_reschedule_for_queue(&oled_work_q, &oled_work, K_NO_WAIT);
			} else if (oled_settings.view_override == 1) {
				if (now - page_start_time >= (uint64_t)oled_settings.power_dwell_s * 1000) {
					oled_scroll_from = 1; oled_scroll_to = 0;
					oled_scroll_step = 0; oled_scroll_active = true;
					oled_sub_phase = 0;
					k_work_reschedule_for_queue(&oled_work_q, &oled_work, K_NO_WAIT);
				} else {
					k_work_reschedule_for_queue(&oled_work_q, &oled_work, K_MSEC(150));
				}
			} else {
				if (now - page_start_time >= (uint64_t)oled_settings.power_dwell_s * 1000) {
					oled_scroll_from = 1; oled_scroll_to = 2;
					oled_scroll_step = 0; oled_scroll_active = true;
					oled_sub_phase = 2;
					k_work_reschedule_for_queue(&oled_work_q, &oled_work, K_NO_WAIT);
				} else {
					k_work_reschedule_for_queue(&oled_work_q, &oled_work, K_MSEC(150));
				}
			}
		} else {
			/* Mic PSD page */
			oled_update_mic();
			if (oled_settings.view_override == 2) {
				/* Stay in mic */
				k_work_reschedule_for_queue(&oled_work_q, &oled_work, K_MSEC(150));
			} else {
				/* Auto or Dashboard: cycle back to IMU */
				if (now - page_start_time >= (uint64_t)oled_settings.mic_dwell_s * 1000) {
					oled_scroll_from = 2; oled_scroll_to = 0;
					oled_scroll_step = 0; oled_scroll_active = true;
					oled_sub_phase = 0;
					k_work_reschedule_for_queue(&oled_work_q, &oled_work, K_NO_WAIT);
				} else {
					k_work_reschedule_for_queue(&oled_work_q, &oled_work, K_MSEC(150));
				}
			}
		}
	} else if (oled_phase == PHASE_QR && oled_settings.qr_enable) {
		oled_draw_qr_code();
		if (k_uptime_get() - qr_view_start < (int64_t)oled_settings.qr_dwell_s * 1000) {
			k_work_reschedule_for_queue(&oled_work_q, &oled_work, K_MSEC(500));
		} else {
			oled_phase = PHASE_BEACON;
			k_work_reschedule_for_queue(&oled_work_q, &oled_work, K_NO_WAIT);
		}
	} else {
		oled_update_display();
		oled_sub_phase = 0;
		if (oled_settings.view_override == 1 || !oled_settings.qr_enable) {
			oled_phase = PHASE_BEACON;
			k_work_reschedule_for_queue(&oled_work_q, &oled_work, K_SECONDS(oled_settings.beacon_dwell_s));
		} else {
			oled_phase = PHASE_QR;
			qr_view_start = k_uptime_get();
			k_work_reschedule_for_queue(&oled_work_q, &oled_work, K_SECONDS(oled_settings.beacon_dwell_s));
		}
	}
}

int main(void) {
	main_thread_id = k_current_get();
	if (gpio_is_ready_dt(&led_red_spec)) { gpio_pin_configure_dt(&led_red_spec, GPIO_OUTPUT_INACTIVE); }
	if (gpio_is_ready_dt(&led_green_spec)) { gpio_pin_configure_dt(&led_green_spec, GPIO_OUTPUT_INACTIVE); }
	if (gpio_is_ready_dt(&led_blue_spec)) { gpio_pin_configure_dt(&led_blue_spec, GPIO_OUTPUT_INACTIVE); }

	nrf_power_resetreas_clear(NRF_POWER,
		NRF_POWER_RESETREAS_OFF_MASK | NRF_POWER_RESETREAS_DOG_MASK);
	if (nrf_power_gpregret_get(NRF_POWER, 0) == DEEP_SLEEP_MAGIC) {
		oled_settings.deepsleep_enable = 1;
		oled_settings.deepsleep_interval = nrf_power_gpregret_get(NRF_POWER, 1);
		nrf_power_gpregret_set(NRF_POWER, 0, 0);
		nrf_power_gpregret_set(NRF_POWER, 1, 0);
		printk("WOKE FROM DEEP SLEEP (interval idx %u)\n", oled_settings.deepsleep_interval);
	}

	usb_enable(NULL);

	bt_conn_auth_cb_register(&auth_cb_display);
	bt_conn_auth_info_cb_register(&auth_info_cb);

	/* Generate a unique, constant 6-digit PIN from Hardware FICR DeviceAddress */
	generated_passkey = (nrf_ficr_deviceaddr_get(NRF_FICR, 0) % 900000) + 100000;
	bt_passkey_set(generated_passkey);
	printk("----------------------------------\n");
	printk("DEVICE UNIQUE PIN: %06u\n", generated_passkey);
	printk("----------------------------------\n");

	/* Register custom settings handler and load settings from flash.
	   Settings must load BEFORE bt_enable() so BT identity is restored. */
	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_register(&skynet_settings);
		settings_load();
	}

	int err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

	bt_conn_cb_register(&conn_callbacks_phy);

	/* Initialize NFC after BT is ready so we can fetch the device address */
	setup_nfc();

	/* Initialize OLED display */
	oled_init();
	oled_apply_settings();

	k_work_init(&adv_start_work, adv_start_handler);

	bt_ready_init();

	{
		static const struct audio_stream_config stream_cfg = {
			.mic_dev = DEVICE_DT_GET(DT_ALIAS(mic)),
			.gpio1_dev = DEVICE_DT_GET(DT_NODELABEL(gpio1)),
			.led_red = &led_red_spec,
			.led_green = &led_green_spec,
			.led_blue = &led_blue_spec,
			.audio_level_buf = audio_level_buf,
			.mem_slab = &audio_mem_slab,
			.gatt_attrs = custom_svc.attrs,
			.audio_val_idx = IDX_AUDIO_VAL,
			.audio_stream_val_idx = IDX_AUDIO_STREAM_VAL,
			.bt_connected = &bt_connected_flag,
			.deepsleep_block_notify = &deepsleep_block_notify,
			.deepsleep_send_once_audio = &deepsleep_send_once_audio,
			.live_audio_active = &live_audio_active,
			.oled_settings = &oled_settings,
		};
		audio_stream_start(&stream_cfg);
	}

	k_thread_create(&sensor_thread_data, sensor_stack, K_THREAD_STACK_SIZEOF(sensor_stack), sensor_thread, NULL, NULL, NULL, 3, K_FP_REGS, K_NO_WAIT);

	/* Initialize and start periodic battery monitoring */
	k_work_init_delayable(&batt_work, batt_work_handler);
	k_work_reschedule(&batt_work, K_NO_WAIT);

	/* Initialize and start periodic status monitoring */
	k_work_init_delayable(&status_work, status_work_handler);
	k_work_reschedule(&status_work, K_NO_WAIT);

	/* Initialize and start periodic OLED update */
	k_work_queue_start(&oled_work_q, oled_work_q_stack,
			   K_THREAD_STACK_SIZEOF(oled_work_q_stack),
			   4, NULL);
	k_work_init_delayable(&oled_work, oled_work_handler);
	k_work_reschedule_for_queue(&oled_work_q, &oled_work, K_SECONDS(1));

	k_work_init_delayable(&deepsleep_work, deepsleep_work_handler);

	while (1) {
		if (deepsleep_pending) {
			deepsleep_block_notify = true;
			deepsleep_oled_sleep();

			while (deepsleep_pending && oled_settings.deepsleep_enable) {
				k_sleep(K_SECONDS(deepsleep_sec_save));
				if (!deepsleep_pending || !oled_settings.deepsleep_enable) break;

				deepsleep_send_once_audio = true;
				deepsleep_send_once_sensor = true;
				if (oled_settings.deepsleep_oled) {
					deepsleep_oled_wake();
				}
				k_work_reschedule_for_queue(&oled_work_q, &oled_work, K_NO_WAIT);
				k_sleep(K_MSEC(DEEPSLEEP_DATA_WINDOW_MS));

				if (!deepsleep_pending || !oled_settings.deepsleep_enable) break;
				if (oled_settings.deepsleep_oled) {
					deepsleep_oled_sleep();
				}
			}

			deepsleep_block_notify = false;
			deepsleep_pending = false;
			deepsleep_oled_wake();
			k_work_reschedule_for_queue(&oled_work_q, &oled_work, K_NO_WAIT);
		} else {
			if (adv_should_run && !bt_connected_flag) {
				start_advertising();
			}
			k_msleep(500);
		}
	}
	return 0;
}
