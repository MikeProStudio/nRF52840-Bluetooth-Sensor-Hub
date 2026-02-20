#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/hci_vs.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/audio/dmic.h>
#include <zephyr/sys/byteorder.h>
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
static uint8_t audio_level_buf[4]; /* uint32_t serialized */
static int8_t tx_power_level = 0; /* Default start value */
static struct batt_data_t battery_status = {0};

/* Forward Declarations */
static void read_battery_voltage_impl(void);
static void update_tx_power_based_on_battery_impl(void);
static void bt_ready(int err);

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
	/* BT_GATT_PERM_READ_ENCRYPT erfordert Kopplung für Datenzugriff */
	BT_GATT_CHARACTERISTIC(BT_UUID_ACCEL_CHRC, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_READ_ENCRYPT, NULL, NULL, NULL),
	BT_GATT_CCC(ccc_cfg_changed, BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),
	BT_GATT_CHARACTERISTIC(BT_UUID_GYRO_CHRC, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_READ_ENCRYPT, NULL, NULL, NULL),
	BT_GATT_CCC(ccc_cfg_changed, BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),
	BT_GATT_CHARACTERISTIC(BT_UUID_AUDIO_CHRC, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_READ_ENCRYPT, NULL, NULL, NULL),
	BT_GATT_CCC(ccc_cfg_changed, BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),
	BT_GATT_CHARACTERISTIC(BT_UUID_TX_POWER_CHRC, BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_READ_ENCRYPT, read_tx_power, NULL, NULL),
	BT_GATT_CCC(ccc_cfg_changed, BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),
	BT_GATT_CHARACTERISTIC(BT_UUID_BATTERY_CHRC, BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_READ_ENCRYPT, read_battery, NULL, NULL),
	BT_GATT_CCC(ccc_cfg_changed, BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),
	BT_GATT_CHARACTERISTIC(BT_UUID_RESET_CHRC, BT_GATT_CHRC_WRITE, BT_GATT_PERM_WRITE_ENCRYPT, NULL, write_reset, NULL),
);

/* --- Calibration & Timing --- */
static float calibration_factor = 2.961f; /* Standard 1M/510k divider */
#define BURST_SAMPLES 10                  /* Messungen innerhalb 1 Sekunde */

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
	k_msleep(10); /* Kurze Wartezeit zum Einschwingen */

	err = adc_channel_setup_dt(&adc_channel);
	if (err != 0) {
		gpio_pin_configure(gpio0_dev, READ_BAT_PIN, GPIO_INPUT);
		return;
	}

	/* 2. Burst-Messung: 10 Samples über ca. 1 Sekunde verteilt */
	for (int i = 0; i < BURST_SAMPLES; i++) {
		(void)adc_sequence_init_dt(&adc_channel, &sequence);
		err = adc_read(adc_channel.dev, &sequence);
		if (err == 0) {
			int32_t m_mv = sample_buffer[0];
			adc_raw_to_millivolts_dt(&adc_channel, &m_mv);
			raw_sum += m_mv;
			samples_taken++;
		}
		k_msleep(100); /* 100ms * 10 = 1000ms (1 Sekunde Gesamtdauer) */
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

	/* 7. Determine Supply Status via /CHG (P0.17) and VBUS */
	err = gpio_pin_configure(gpio0_dev, CHG_STAT_PIN, GPIO_INPUT | GPIO_PULL_UP);
	if (err == 0) {
		bool is_charging = (gpio_pin_get(gpio0_dev, CHG_STAT_PIN) == 0);
		bool vbus_present = (NRF_POWER->USBREGSTATUS & 1);

		if (is_charging) {
			battery_status.status = 1; /* Charging */
			gpio_pin_configure(gpio0_dev, HICHG_PIN, GPIO_OUTPUT_HIGH); 
		} else if (vbus_present) {
			battery_status.status = 2; /* USB Power / Full */
			gpio_pin_configure(gpio0_dev, HICHG_PIN, GPIO_OUTPUT_LOW);
		} else {
			battery_status.status = 0; /* Battery Mode */
			gpio_pin_configure(gpio0_dev, HICHG_PIN, GPIO_OUTPUT_LOW);
		}
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

	/* Notify Battery Data (Index 14) */
	bt_gatt_notify(NULL, &custom_svc.attrs[14], &battery_status, sizeof(battery_status));
}

static uint32_t generated_passkey = 0;
static char device_name_with_pin[32] = "Skynet Beacon";

static struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_CUSTOM_SERVICE_VAL),
};

static struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, device_name_with_pin, 13)
};

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		printk("Connection failed (err 0x%02x)\n", err);
	} else {
		printk("Connected, requesting security level 4...\n");
		
		/* SENIOR-DEV: Falls bereits gekoppelt, Name zurücksetzen.
		   Hier prüfen wir, ob die Verbindung verschlüsselt ist. */
		bt_conn_set_security(conn, BT_SECURITY_L4);
	}
}

static struct k_work adv_start_work;

static void adv_start_handler(struct k_work *work)
{
	/* SENIOR-DEV: Name in Scan Response setzen */
	sd[0].data_len = strlen(device_name_with_pin);

	struct bt_le_adv_param adv_param = {
		.id = BT_ID_DEFAULT,
		.sid = 0,
		.options = (1UL << 0), /* Nur CONNECTABLE */
		.interval_min = BT_GAP_ADV_FAST_INT_MIN_2,
		.interval_max = BT_GAP_ADV_FAST_INT_MAX_2,
	};
	int err = bt_le_adv_start(&adv_param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err) {
		printk("Advertising failed to restart (err %d)\n", err);
	} else {
		printk("Advertising restarted successfully\n");
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	printk("Disconnected (reason 0x%02x)\n", reason);
	
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

/* Security Callbacks für Passkey-Authentifizierung */
static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
	printk("Security: Passkey for pairing is %06u\n", passkey);
}

static void auth_cancel(struct bt_conn *conn)
{
	printk("Security: Pairing cancelled\n");
}

static struct bt_conn_auth_cb auth_cb_display = {
	.passkey_display = auth_passkey_display,
	.passkey_entry = NULL,
	.cancel = auth_cancel,
};

static void bt_ready(int err)
{
	if (err) {
		printk("Bluetooth ready failed (err %d)\n", err);
		return;
	}
	printk("Bluetooth initialized. Starting advertising...\n");

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
	/* SENIOR-DEV: Hardware-PIN aus DeviceID generieren */
	uint32_t dev_id = NRF_FICR->DEVICEID[0];
	uint32_t short_pin = (dev_id % 9000) + 1000;
	generated_passkey = short_pin * 100;
	
	snprintf(device_name_with_pin, sizeof(device_name_with_pin), "Skynet [%u]", short_pin);
	sd[0].data_len = strlen(device_name_with_pin);

	printk("************************************************\n");
	printk("SECURITY: Name=%s, Passkey=%06u\n", device_name_with_pin, generated_passkey);
	printk("************************************************\n");

	bt_passkey_set(generated_passkey);

	/* SENIOR-DEV: Nutze Standard-Parameter für maximale Kompatibilität */
	struct bt_le_adv_param adv_param = {
		.id = BT_ID_DEFAULT,
		.sid = 0,
		.options = (1UL << 0),
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
K_THREAD_STACK_DEFINE(sensor_stack, 1024);

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
		if (ret == 0) {
			int16_t *samples = (int16_t *)buffer;
			uint32_t count = size / sizeof(int16_t);
			double sum_sq = 0;
			for (uint32_t i = 0; i < count; i++) {
				double val = (double)samples[i] / 32768.0;
				sum_sq += val * val;
			}
			double rms = sqrt(sum_sq / (double)count);
			val_u32 = (uint32_t)(rms * 1000.0);
			sys_put_le32(val_u32, audio_level_buf);
			
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
				printk("Mic RMS: %u (raw scale)\n", val_u32);
			}

			bt_gatt_notify(NULL, &custom_svc.attrs[8], audio_level_buf, sizeof(audio_level_buf));
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
			
			bt_gatt_notify(NULL, &custom_svc.attrs[2], &accel_data, sizeof(accel_data));
			bt_gatt_notify(NULL, &custom_svc.attrs[5], &gyro_data, sizeof(gyro_data));
		}
		k_msleep(20);
	}
}

int main(void) {
	if (gpio_is_ready_dt(&led)) { gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE); }
	if (gpio_is_ready_dt(&led_red_spec)) { gpio_pin_configure_dt(&led_red_spec, GPIO_OUTPUT_INACTIVE); }
	if (gpio_is_ready_dt(&led_green_spec)) { gpio_pin_configure_dt(&led_green_spec, GPIO_OUTPUT_INACTIVE); }
	if (gpio_is_ready_dt(&led_blue_spec)) { gpio_pin_configure_dt(&led_blue_spec, GPIO_OUTPUT_INACTIVE); }
	
	usb_enable(NULL);
	int err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

	/* SENIOR-DEV: Settings laden für persistentes Bonding */
	settings_load();

	/* SENIOR-DEV: Auth Callbacks registrieren (Passkey wird dynamisch in bt_ready gesetzt) */
	bt_conn_auth_cb_register(&auth_cb_display);

	k_work_submit(&bt_ready_work);
	k_thread_create(&audio_thread_data, audio_stack, K_THREAD_STACK_SIZEOF(audio_stack), audio_thread, NULL, NULL, NULL, 2, 0, K_NO_WAIT);
	k_thread_create(&sensor_thread_data, sensor_stack, K_THREAD_STACK_SIZEOF(sensor_stack), sensor_thread, NULL, NULL, NULL, 3, 0, K_NO_WAIT);

	uint32_t battery_timer = 0;
	/* VARIABLE HIER ÄNDERN: Sendeintervall in Sekunden */
	int battery_send_interval_sec = 30;

	while (1) {
		/* Burst-Messung (Dauert ca. 1 Sekunde durch k_msleep in der Funktion) */
		read_battery_voltage_impl();

		/* Senden und Logik-Update basierend auf Intervall */
		if (battery_timer == 0 || battery_timer >= battery_send_interval_sec) {
			update_tx_power_based_on_battery_impl();
			battery_timer = 0;
		}

		battery_timer++;
		/* Pause zwischen den Mess-Zyklen */
		k_msleep(1000);
	}
	return 0;
}
