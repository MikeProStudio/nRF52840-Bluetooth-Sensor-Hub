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
#include <nfc_t2t_lib.h>
#include <nfc/ndef/uri_msg.h>
#include <nfc/ndef/le_oob_rec.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* --- Hardware Definitions --- */
#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)
#define LED2_NODE DT_ALIAS(led2)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
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

#define BT_UUID_CUSTOM_SERVICE  BT_UUID_DECLARE_128(BT_UUID_CUSTOM_SERVICE_VAL)
#define BT_UUID_ACCEL_CHRC      BT_UUID_DECLARE_128(BT_UUID_ACCEL_CHRC_VAL)
#define BT_UUID_GYRO_CHRC       BT_UUID_DECLARE_128(BT_UUID_GYRO_CHRC_VAL)
#define BT_UUID_AUDIO_CHRC      BT_UUID_DECLARE_128(BT_UUID_AUDIO_CHRC_VAL)
#define BT_UUID_TX_POWER_CHRC   BT_UUID_DECLARE_128(BT_UUID_TX_POWER_CHRC_VAL)
#define BT_UUID_BATTERY_CHRC    BT_UUID_DECLARE_128(BT_UUID_BATTERY_CHRC_VAL)
#define BT_UUID_RESET_CHRC      BT_UUID_DECLARE_128(BT_UUID_RESET_CHRC_VAL)

/* Sichere Indizes für GATT-Charakteristiken (verhindert Magic Numbers) */
enum {
	IDX_CUSTOM_SVC = 0,
	IDX_ACCEL_VAL = 2,
	IDX_GYRO_VAL = 5,
	IDX_AUDIO_VAL = 8,
	IDX_TX_POWER_VAL = 11,
	IDX_BATTERY_VAL = 14,
	IDX_RESET_VAL = 17,
};

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

/* Forward Declarations */
static void read_battery_voltage_impl(void);
static void update_tx_power_based_on_battery_impl(void);
static void bt_ready(int err);

/* --- Power Management (Asynchron) --- */
static struct k_work_delayable batt_work;
static struct k_work_delayable status_work;
static uint8_t last_power_status = 255;

static void status_work_handler(struct k_work *work)
{
	uint8_t current_status = 0;
	if (gpio_pin_configure(gpio0_dev, CHG_STAT_PIN, GPIO_INPUT | GPIO_PULL_UP) == 0) {
		bool is_charging = (gpio_pin_get(gpio0_dev, CHG_STAT_PIN) == 0);
		bool vbus_present = (NRF_POWER->USBREGSTATUS & 1);

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

/* Set TX Power via Nordic HCI VS Command */
static int set_bt_tx_power(int8_t power_dbm) {
    struct net_buf *buf, *rsp = NULL;
    struct bt_hci_cp_vs_write_tx_power_level *cp;
    struct bt_hci_rp_vs_write_tx_power_level *rp;
    int err;

    /* Create HCI Command (Vendor Specific for nRF) */
    buf = bt_hci_cmd_create(BT_HCI_OP_VS_WRITE_TX_POWER_LEVEL, sizeof(*cp));
    if (!buf) {
        printk("Unable to allocate command buffer\n");
        return -ENOBUFS;
    }

    cp = net_buf_add(buf, sizeof(*cp));
    cp->handle_type = BT_HCI_VS_LL_HANDLE_TYPE_ADV; /* Advertising Set */
    cp->handle = 0; /* Default handle */
    cp->tx_power_level = power_dbm;

    err = bt_hci_cmd_send_sync(BT_HCI_OP_VS_WRITE_TX_POWER_LEVEL, buf, &rsp);
    if (err) {
        printk("Set TX Power HCI failed (%d), assuming static config. Target: %d dBm\n", err, power_dbm);
        tx_power_level = power_dbm;
        return 0; // Fake success to continue logic
    }

    if (rsp) {
        rp = (void *)rsp->data;
        printk("Set TX Power success. Selected: %d dBm\n", rp->selected_tx_power);
        tx_power_level = rp->selected_tx_power; /* Update global variable for reporting */
        net_buf_unref(rsp);
    }
    return 0;
}

static int16_t sample_buffer[1];
static struct adc_sequence sequence = {
	.buffer = sample_buffer,
	.buffer_size = sizeof(sample_buffer),
};

/* Filter storage */
#define FILTER_SIZE 12
static int32_t adc_history[FILTER_SIZE] = {0};
static int filter_idx = 0;
static bool filter_full = false;

static void ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value) {}

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
		/* Nordic-specific: Jump to Bootloader via GPREGRET */
		NRF_POWER->GPREGRET = 0x57; /* Standard Nordic Bootloader Magic (Enter Bootloader) */
		NVIC_SystemReset();
	}
	return len;
}

/* Service declaration */
BT_GATT_SERVICE_DEFINE(custom_svc,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_CUSTOM_SERVICE),
	/* Sicherheits-Architektur: Authentifizierung (Pairing) für ALLE Sensordaten erforderlich. */
	BT_GATT_CHARACTERISTIC(BT_UUID_ACCEL_CHRC, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_READ_AUTHEN, NULL, NULL, NULL),
	BT_GATT_CCC(ccc_cfg_changed, BT_GATT_PERM_READ_AUTHEN | BT_GATT_PERM_WRITE_AUTHEN),
	BT_GATT_CHARACTERISTIC(BT_UUID_GYRO_CHRC, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_READ_AUTHEN, NULL, NULL, NULL),
	BT_GATT_CCC(ccc_cfg_changed, BT_GATT_PERM_READ_AUTHEN | BT_GATT_PERM_WRITE_AUTHEN),
	BT_GATT_CHARACTERISTIC(BT_UUID_AUDIO_CHRC, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_READ_AUTHEN, NULL, NULL, NULL),
	BT_GATT_CCC(ccc_cfg_changed, BT_GATT_PERM_READ_AUTHEN | BT_GATT_PERM_WRITE_AUTHEN),
	BT_GATT_CHARACTERISTIC(BT_UUID_TX_POWER_CHRC, BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_READ_AUTHEN, read_tx_power, NULL, NULL),
	BT_GATT_CCC(ccc_cfg_changed, BT_GATT_PERM_READ_AUTHEN | BT_GATT_PERM_WRITE_AUTHEN),
	BT_GATT_CHARACTERISTIC(BT_UUID_BATTERY_CHRC, BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_READ_AUTHEN, read_battery, NULL, NULL),
	BT_GATT_CCC(ccc_cfg_changed, BT_GATT_PERM_READ_AUTHEN | BT_GATT_PERM_WRITE_AUTHEN),
	BT_GATT_CHARACTERISTIC(BT_UUID_RESET_CHRC, BT_GATT_CHRC_WRITE, BT_GATT_PERM_WRITE_AUTHEN, NULL, write_reset, NULL),
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
		
		/* 5. SMA Filter (Langzeit-Glättung) */
		adc_history[filter_idx] = val_mv;
		filter_idx = (filter_idx + 1) % FILTER_SIZE;
		if (filter_idx == 0) filter_full = true;
		
		int32_t sum = 0;
		int count = filter_full ? FILTER_SIZE : filter_idx;
		for (int i = 0; i < count; i++) {
			sum += adc_history[i];
		}
		val_mv = sum / count;
		
		battery_status.voltage_mv = (uint16_t)val_mv;

		/* SoC Logic: 3.3V to 4.15V LiPo range */
		if (val_mv >= 4150) battery_status.soc = 100;
		else if (val_mv <= 3300) battery_status.soc = 0;
		else battery_status.soc = (uint8_t)((val_mv - 3300) * 100 / (4150 - 3300));
	}
}

static void update_tx_power_based_on_battery_impl(void)
{
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
static char device_name_with_pin[32] = "Skynet AI Beacon";

static struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, device_name_with_pin, 16) /* Name ins Hauptpaket für Smartphones */
};

static struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_CUSTOM_SERVICE_VAL), /* UUID ins Scan-Response */
};

static struct bt_conn *current_conn = NULL;

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		printk("Connection failed (err 0x%02x)\n", err);
	} else {
		printk("Connected. Waiting for GATT access to trigger security...\n");
		if (current_conn) {
			bt_conn_unref(current_conn);
		}
		current_conn = bt_conn_ref(conn);
	}
}

static struct k_work adv_start_work;

static void adv_start_handler(struct k_work *work)
{
	bt_ready(0);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	printk("Disconnected (reason 0x%02x)\n", reason);
	
	if (current_conn == conn) {
		bt_conn_unref(current_conn);
		current_conn = NULL;
	}

	/* SENIOR-DEV: Zuerst Advertising explizit stoppen, falls noch Reste laufen,
	   dann über Workqueue neu starten. Das löst den Zombie-Modus. */
	bt_le_adv_stop();
	k_work_submit(&adv_start_work);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

static void bt_ready_work_handler(struct k_work *work)
{
	bt_ready(0);
}
static K_WORK_DEFINE(bt_ready_work, bt_ready_work_handler);

static void bt_ready(int err)
{
	if (err) {
		printk("Bluetooth ready failed (err %d)\n", err);
		return;
	}
	printk("Bluetooth initialized. Starting advertising...\n");

	/* RGB Startup Sequence: Kurzer Flash statt langer Loop */
	const struct gpio_dt_spec *specs[] = {&led_red_spec, &led_green_spec, &led_blue_spec};
	for (int s = 0; s < 3; s++) {
		if (gpio_is_ready_dt(specs[s])) {
			gpio_pin_set_dt(specs[s], 1);
			k_msleep(50);
			gpio_pin_set_dt(specs[s], 0);
		}
	}

	set_bt_tx_power(8);

	snprintf(device_name_with_pin, sizeof(device_name_with_pin), "Skynet AI Beacon");
	ad[1].data_len = strlen(device_name_with_pin);

	printk("************************************************\n");
	printk("SECURITY: Passkey Display (L4) enabled\n");
	printk("************************************************\n");

	/* SENIOR-DEV: Nutze Standard-Parameter für maximale Kompatibilität */
	struct bt_le_adv_param adv_param = {
		.id = BT_ID_DEFAULT,
		.sid = 0,
		.options = (1UL << 0) | (1UL << 2),
		.interval_min = BT_GAP_ADV_FAST_INT_MIN_2,
		.interval_max = BT_GAP_ADV_FAST_INT_MAX_2,
	};
	int adv_err = bt_le_adv_start(&adv_param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (adv_err) {
		printk("Advertising failed to start (err %d)\n", adv_err);
	} else {
		printk("Advertising started. Skynet Beacon is visible.\n");
	}
}

static int32_t my_sensor_value_to_milli(const struct sensor_value *val) { return (val->val1 * 1000) + (val->val2 / 1000); }

/* --- Audio Thread --- */
#define AUDIO_SAMPLE_RATE 16000
#define AUDIO_SAMPLES_PER_BLOCK 512
#define AUDIO_BLOCK_SIZE (AUDIO_SAMPLES_PER_BLOCK * sizeof(int16_t))
K_MEM_SLAB_DEFINE(audio_mem_slab, AUDIO_BLOCK_SIZE, 8, 4);

static struct k_thread audio_thread_data;
static struct k_thread sensor_thread_data;
K_THREAD_STACK_DEFINE(audio_stack, 2048);
K_THREAD_STACK_DEFINE(sensor_stack, 2048);

static void audio_thread(void *p1, void *p2, void *p3)
{
	void *buffer;
	size_t size;
	int ret;
	uint32_t val_u32;

	if (device_is_ready(gpio1_dev)) {
		gpio_pin_configure(gpio1_dev, SENSE_PWR_PIN, GPIO_OUTPUT_HIGH);
		gpio_pin_set(gpio1_dev, SENSE_PWR_PIN, 1);
		k_msleep(100);
	}

	if (!device_is_ready(mic_dev)) return;

	struct pcm_stream_cfg stream_cfg = {
		.pcm_rate = AUDIO_SAMPLE_RATE,
		.pcm_width = 16,
		.block_size = AUDIO_BLOCK_SIZE,
		.mem_slab = &audio_mem_slab,
	};

	struct dmic_cfg cfg;
	memset(&cfg, 0, sizeof(cfg));
	cfg.io.min_pdm_clk_freq = 1000000;
	cfg.io.max_pdm_clk_freq = 3200000;
	cfg.streams = &stream_cfg;
	cfg.channel.req_chan_map_lo = dmic_build_channel_map(0, 0, PDM_CHAN_LEFT);
	cfg.channel.req_num_chan = 1;
	cfg.channel.req_num_streams = 1;

	if (dmic_configure(mic_dev, &cfg) < 0) return;
	if (dmic_trigger(mic_dev, DMIC_TRIGGER_START) < 0) return;

	while (1) {
		ret = dmic_read(mic_dev, 0, &buffer, &size, SYS_FOREVER_MS);
		if (ret != 0) {
			printk("DMIC Read Error: %d\n", ret);
			k_msleep(10);
			continue;
		}
		
		if (ret == 0) {
			int16_t *samples = (int16_t *)buffer;
			uint32_t count = size / sizeof(int16_t);
			
			double sum_sq = 0;
			uint32_t zero_crossings = 0;
			int16_t last_sample = 0;

			for (uint32_t i = 0; i < count; i++) {
				/* RMS Calculation */
				double val = (double)samples[i] / 32768.0;
				sum_sq += val * val;

				/* Zero-Crossing Rate (ZCR) Calculation */
				/* Check if the sign changed compared to the last sample, ignore exactly 0 to avoid noise jitter */
				if (i > 0) {
					if ((samples[i] > 0 && last_sample < 0) || (samples[i] < 0 && last_sample > 0)) {
						zero_crossings++;
					}
				}
				last_sample = samples[i];
			}
			
			double rms = sqrt(sum_sq / (double)count);
			
			/* Pack data for BLE: [4 bytes RMS] [4 bytes ZCR] */
			val_u32 = (uint32_t)(rms * 1000.0);
			sys_put_le32(val_u32, &audio_level_buf[0]);
			sys_put_le32(zero_crossings, &audio_level_buf[4]);
			
			/* RGB LED Audio Logic (Highly Smoothed for Fade-out effect) */
			static double smoothed_rms = 0.0;
			const double decay_factor = 0.885; /* Very slow fade-out */
			const double attack_factor = 0.12; /* Smoother build-up */

			if (rms > smoothed_rms) {
				smoothed_rms = (attack_factor * rms) + ((1.0 - attack_factor) * smoothed_rms);
			} else {
				smoothed_rms *= decay_factor;
			}

			if (gpio_is_ready_dt(&led_red_spec) && gpio_is_ready_dt(&led_green_spec) && gpio_is_ready_dt(&led_blue_spec)) {
				if (smoothed_rms < 0.004) { /* Silence */
					gpio_pin_set_dt(&led_red_spec, 0); gpio_pin_set_dt(&led_green_spec, 0); gpio_pin_set_dt(&led_blue_spec, 0);
				} else if (smoothed_rms < 0.012) { /* Low: Cyan/Blue */
					gpio_pin_set_dt(&led_red_spec, 0); gpio_pin_set_dt(&led_green_spec, 1); gpio_pin_set_dt(&led_blue_spec, 1);
				} else if (smoothed_rms < 0.025) { /* Mid: Green */
					gpio_pin_set_dt(&led_red_spec, 0); gpio_pin_set_dt(&led_green_spec, 1); gpio_pin_set_dt(&led_blue_spec, 0);
				} else if (smoothed_rms < 0.045) { /* High: Yellow */
					gpio_pin_set_dt(&led_red_spec, 1); gpio_pin_set_dt(&led_green_spec, 1); gpio_pin_set_dt(&led_blue_spec, 0);
				} else { /* Peak: Red */
					gpio_pin_set_dt(&led_red_spec, 1); gpio_pin_set_dt(&led_green_spec, 0); gpio_pin_set_dt(&led_blue_spec, 0);
				}
			}

			/* Debug print microphone level */
			if (k_uptime_get_32() % 1000 < 50) {
				int status = -1;
				if (current_conn && bt_gatt_is_subscribed(current_conn, &custom_svc.attrs[IDX_AUDIO_VAL], BT_GATT_CCC_NOTIFY)) {
					status = bt_gatt_notify(current_conn, &custom_svc.attrs[IDX_AUDIO_VAL], audio_level_buf, sizeof(audio_level_buf));
				}
				printk("Mic RMS: %u (raw scale), Notify status: %d\n", val_u32, status);
			} else {
				if (current_conn && bt_gatt_is_subscribed(current_conn, &custom_svc.attrs[IDX_AUDIO_VAL], BT_GATT_CCC_NOTIFY)) {
					bt_gatt_notify(current_conn, &custom_svc.attrs[IDX_AUDIO_VAL], audio_level_buf, sizeof(audio_level_buf));
				}
			}
			k_mem_slab_free(&audio_mem_slab, buffer);
		}
		k_yield();
	}
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
			
			if (current_conn) {
				if (bt_gatt_is_subscribed(current_conn, &custom_svc.attrs[IDX_ACCEL_VAL], BT_GATT_CCC_NOTIFY)) {
					bt_gatt_notify(current_conn, &custom_svc.attrs[IDX_ACCEL_VAL], &accel_data, sizeof(accel_data));
				}
				if (bt_gatt_is_subscribed(current_conn, &custom_svc.attrs[IDX_GYRO_VAL], BT_GATT_CCC_NOTIFY)) {
					bt_gatt_notify(current_conn, &custom_svc.attrs[IDX_GYRO_VAL], &gyro_data, sizeof(gyro_data));
				}
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

int main(void) {
	if (gpio_is_ready_dt(&led)) { gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE); }
	if (gpio_is_ready_dt(&led_red_spec)) { gpio_pin_configure_dt(&led_red_spec, GPIO_OUTPUT_INACTIVE); }
	if (gpio_is_ready_dt(&led_green_spec)) { gpio_pin_configure_dt(&led_green_spec, GPIO_OUTPUT_INACTIVE); }
	if (gpio_is_ready_dt(&led_blue_spec)) { gpio_pin_configure_dt(&led_blue_spec, GPIO_OUTPUT_INACTIVE); }
	
	usb_enable(NULL);

	bt_conn_auth_cb_register(&auth_cb_display);
	bt_conn_auth_info_cb_register(&auth_info_cb);

	/* Generate a unique, constant 6-digit PIN from Hardware FICR DeviceAddress */
	generated_passkey = (NRF_FICR->DEVICEADDR[0] % 900000) + 100000;
	bt_passkey_set(generated_passkey);
	printk("----------------------------------\n");
	printk("DEVICE UNIQUE PIN: %06u\n", generated_passkey);
	printk("----------------------------------\n");

	int err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

	if (IS_ENABLED(CONFIG_BT_SETTINGS)) {
		settings_load();
	}

	/* Initialize NFC after BT is ready so we can fetch the device address */
	setup_nfc();

	k_work_init(&adv_start_work, adv_start_handler);

	k_work_submit(&bt_ready_work);
	k_thread_create(&audio_thread_data, audio_stack, K_THREAD_STACK_SIZEOF(audio_stack), audio_thread, NULL, NULL, NULL, 2, K_FP_REGS, K_NO_WAIT);
	k_thread_create(&sensor_thread_data, sensor_stack, K_THREAD_STACK_SIZEOF(sensor_stack), sensor_thread, NULL, NULL, NULL, 3, K_FP_REGS, K_NO_WAIT);

	/* Initialize and start periodic battery monitoring */
	k_work_init_delayable(&batt_work, batt_work_handler);
	k_work_reschedule(&batt_work, K_NO_WAIT);

	/* Initialize and start periodic status monitoring */
	k_work_init_delayable(&status_work, status_work_handler);
	k_work_reschedule(&status_work, K_NO_WAIT);

	while (1) {
		/* The main loop is now very light, everything is handled by threads and workqueues.
		   We sleep for a longer time to save power. */
		k_msleep(1000);
	}
	return 0;
}
