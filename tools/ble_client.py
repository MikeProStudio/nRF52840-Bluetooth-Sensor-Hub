import asyncio
import logging
import struct
import time
from bleak import BleakScanner, BleakClient, BleakError

logger = logging.getLogger("ble")

AUDIO_STREAM_CHRC_UUID = "12345678-1234-5678-1234-56789abcdef8"
ACCEL_CHRC_UUID = "12345678-1234-5678-1234-56789abcdef1"
GYRO_CHRC_UUID = "12345678-1234-5678-1234-56789abcdef2"
AUDIO_CHRC_UUID = "12345678-1234-5678-1234-56789abcdef3"
TX_POWER_CHRC_UUID = "12345678-1234-5678-1234-56789abcdef4"
BATTERY_CHRC_UUID = "12345678-1234-5678-1234-56789abcdef5"
DEVICE_NAME = "Skynet AI Beacon"

STEP_SIZE_TABLE = [
    7, 8, 9, 10, 11, 12, 13, 14,
    16, 17, 19, 21, 23, 25, 28, 31,
    34, 37, 41, 45, 50, 55, 60, 66,
    73, 80, 88, 97, 107, 118, 130, 143,
    157, 173, 190, 209, 230, 253, 279, 307,
    337, 371, 408, 449, 494, 544, 598, 658,
    724, 796, 876, 963, 1060, 1166, 1282, 1411,
    1552, 1707, 1878, 2066, 2272, 2499, 2749, 3024,
    3327, 3660, 4026, 4428, 4871, 5358, 5894, 6484,
    7132, 7845, 8630, 9493, 10442, 11487, 12635, 13899,
    15289, 16818, 18500, 20350, 22385, 24623, 27086, 29794,
    32767
]

INDEX_ADJUST = [-1, -1, -1, -1, 2, 4, 6, 8]


class ADPCMDecoder:
    def __init__(self):
        self.predictor = 0
        self.step_index = 0

    def reset(self):
        self.predictor = 0
        self.step_index = 0

    def decode(self, adpcm_data: bytes) -> bytes:
        pcm_samples = []
        predictor = self.predictor
        step_index = self.step_index

        for byte in adpcm_data:
            for shift in (4, 0):
                nibble = (byte >> shift) & 0x0F

                step = STEP_SIZE_TABLE[step_index]
                diff = step >> 3

                if nibble & 4:
                    diff += step
                if nibble & 2:
                    diff += step >> 1
                if nibble & 1:
                    diff += step >> 2

                if nibble & 8:
                    predictor -= diff
                else:
                    predictor += diff

                if predictor > 32767:
                    predictor = 32767
                elif predictor < -32768:
                    predictor = -32768

                step_index += INDEX_ADJUST[nibble & 7]
                if step_index > 88:
                    step_index = 88
                elif step_index < 0:
                    step_index = 0

                pcm_samples.append(predictor)

        self.predictor = predictor
        self.step_index = step_index

        return struct.pack(f"<{len(pcm_samples)}h", *pcm_samples)


class BLEClient:
    def __init__(self, device_name=DEVICE_NAME, gain=8.0):
        self.device_name = device_name
        self.decoder = ADPCMDecoder()
        self.client = None
        self.gain = gain
        self.last_seq = -1
        self.total_samples = 0
        self.total_packets = 0
        self._last_notify_time = 0
        self.start_time = None
        # Sensor callbacks
        self.on_accel = None
        self.on_gyro = None
        self.on_audio_level = None
        self.on_battery = None
        self.on_tx_power = None
        self.on_audio = None
        self.on_connected = None
        self.on_disconnected = None
        self._running = False

    def set_gain(self, gain: float):
        self.gain = max(0.1, min(100.0, gain))

    def _audio_stream_handler(self, sender, data):
        """Handle audio stream notifications (ADPCM encoded)"""
        try:
            self._last_notify_time = time.time()
            if self.start_time is None:
                self.start_time = time.time()
            if not data or len(data) < 2:
                return
            self.total_packets += 1
            if self.total_packets <= 5 or self.total_packets % 100 == 1:
                logger.info(f"Audio notify #{self.total_packets}: len={len(data)} seq={data[0] & 0x7F}")
            header = data[0]
            is_sync = (header & 0x80) == 0
            seq = header & 0x7F
            adpcm_payload = data[1:]
            if is_sync:
                self.decoder.reset()
                self.last_seq = seq
            else:
                expected = (self.last_seq + 1) & 0x7F
                if self.last_seq >= 0 and seq != expected:
                    lost = ((seq - expected - 1) & 0x7F) + 1
            self.last_seq = seq
            pcm_bytes = self.decoder.decode(adpcm_payload)
            if self.gain != 1.0:
                samples = struct.unpack(f"<{len(pcm_bytes)//2}h", pcm_bytes)
                gained = [max(-32768, min(32767, int(s * self.gain))) for s in samples]
                pcm_bytes = struct.pack(f"<{len(gained)}h", *gained)
            self.total_samples += len(pcm_bytes) // 2
            if self.on_audio:
                self.on_audio(pcm_bytes)
        except Exception as e:
            logger.error(f"audio_stream handler: {e}", exc_info=True)

    def _accel_handler(self, sender, data):
        """3x int32 LE = accel X/Y/Z * 1000"""
        try:
            if len(data) >= 12:
                x = struct.unpack_from('<i', data, 0)[0]
                y = struct.unpack_from('<i', data, 4)[0]
                z = struct.unpack_from('<i', data, 8)[0]
                if self.on_accel:
                    self.on_accel(x / 1000.0, y / 1000.0, z / 1000.0)
        except Exception as e:
            logger.error(f"accel handler: {e}")

    def _gyro_handler(self, sender, data):
        """3x int32 LE = gyro X/Y/Z * 1000 (in rad/s, convert to deg/s * 57.3)"""
        try:
            if len(data) >= 12:
                x = struct.unpack_from('<i', data, 0)[0]
                y = struct.unpack_from('<i', data, 4)[0]
                z = struct.unpack_from('<i', data, 8)[0]
                if self.on_gyro:
                    self.on_gyro(x / 1000.0 * 57.3, y / 1000.0 * 57.3, z / 1000.0 * 57.3)
        except Exception as e:
            logger.error(f"gyro handler: {e}")

    def _audio_level_handler(self, sender, data):
        """uint32 RMS (x1000) + uint32 ZCR"""
        try:
            if len(data) >= 8:
                rms = struct.unpack_from('<I', data, 0)[0]
                zcr = struct.unpack_from('<I', data, 4)[0]
                if self.on_audio_level:
                    self.on_audio_level(rms, zcr)
        except Exception as e:
            logger.error(f"audio_level handler: {e}")

    def _battery_handler(self, sender, data):
        """uint16 voltage_mv + uint8 soc + uint8 status"""
        try:
            if len(data) >= 4:
                mv = struct.unpack_from('<H', data, 0)[0]
                soc = data[2]
                status = data[3]
                if self.on_battery:
                    self.on_battery(mv, soc, status)
        except Exception as e:
            logger.error(f"battery handler: {e}")

    def _tx_power_handler(self, sender, data):
        """int8 dBm"""
        try:
            if len(data) >= 1:
                dbm = struct.unpack_from('<b', data, 0)[0]
                if self.on_tx_power:
                    self.on_tx_power(dbm)
        except Exception as e:
            logger.error(f"tx_power handler: {e}")

    async def connect(self, timeout=15.0):
        logger.info(f"Scanning for '{self.device_name}' (timeout={timeout}s)...")
        device = await BleakScanner.find_device_by_name(
            self.device_name, timeout=timeout
        )
        if not device:
            logger.warning(f"Device '{self.device_name}' not found via name scan, trying filter scan...")
            device = await BleakScanner.find_device_by_filter(
                lambda d, ad: d.name and self.device_name in d.name,
                timeout=timeout
            )
        if not device:
            raise ConnectionError(
                f"Device '{self.device_name}' not found after scanning. "
                "Check: 1) device is powered on 2) BLE is enabled on PC "
                "3) device is advertising (check serial output)"
            )

        logger.info(f"Found: {device.name} ({device.address})")
        logger.info(f"Connecting...")

        self.client = BleakClient(device)
        try:
            await self.client.connect()
        except Exception as e:
            raise ConnectionError(f"BLE connect failed: {e}")

        if not self.client.is_connected:
            raise ConnectionError("BLE connection failed: is_connected is False after connect()")

        async def _sub(uuid, handler):
            try:
                ch = None
                try:
                    ch = self.client.services.get_characteristic(uuid)
                except Exception:
                    pass
                if ch:
                    await self.client.start_notify(ch, handler)
                else:
                    await self.client.start_notify(uuid, handler)
                return True
            except Exception as e:
                logger.warning(f"subscribe {uuid} failed: {e}")
                return False

        logger.info(f"Subscribing to all characteristics...")
        subs = [
            (AUDIO_STREAM_CHRC_UUID, self._audio_stream_handler, "audio stream"),
            (ACCEL_CHRC_UUID, self._accel_handler, "accel"),
            (GYRO_CHRC_UUID, self._gyro_handler, "gyro"),
            (AUDIO_CHRC_UUID, self._audio_level_handler, "audio level"),
            (BATTERY_CHRC_UUID, self._battery_handler, "battery"),
            (TX_POWER_CHRC_UUID, self._tx_power_handler, "TX power"),
        ]
        ok = 0
        for uuid, handler, name in subs:
            if await _sub(uuid, handler):
                ok += 1
                logger.info(f"  {name}: subscribed")
            else:
                logger.warning(f"  {name}: FAILED")
        if ok == 0:
            await self.client.disconnect()
            raise ConnectionError("Failed to subscribe to ANY characteristic")
        logger.info(f"Subscribed to {ok}/{len(subs)} characteristics")

        self._running = True
        self.start_time = time.time()
        logger.info(f"All sensors subscribed. Streaming started.")

        if self.on_connected:
            self.on_connected()

    async def disconnect(self):
        self._running = False
        if self.client and self.client.is_connected:
            try:
                await self.client.stop_notify(AUDIO_STREAM_CHRC_UUID)
            except Exception:
                pass
            try:
                await self.client.disconnect()
            except Exception:
                pass
        if self.on_disconnected:
            self.on_disconnected()

    async def run_forever(self):
        try:
            while self._running and self.client and self.client.is_connected:
                await asyncio.sleep(0.1)
        except (KeyboardInterrupt, asyncio.CancelledError):
            pass
        finally:
            await self.disconnect()

    @property
    def is_connected(self) -> bool:
        return self.client is not None and self.client.is_connected

    @property
    def notifications_alive(self) -> bool:
        """True if notifications received within last 5 seconds"""
        if not self.is_connected:
            return False
        if self._last_notify_time == 0:
            return False  # never received a notification
        return (time.time() - self._last_notify_time) < 5.0

    @property
    def elapsed(self) -> float:
        if self.start_time is None:
            return 0.0
        return time.time() - self.start_time
