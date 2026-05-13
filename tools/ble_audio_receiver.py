#!/usr/bin/env python3
"""
BLE Audio Stream Receiver for nRF52840 Sense Wireless Mic.

Connects to "Skynet AI Beacon" via BLE, subscribes to the audio stream
characteristic, decodes IMA ADPCM to 16-bit PCM, and provides the raw
audio buffer for further processing (e.g. speech-to-text, file output).

Usage:
    python ble_audio_receiver.py                          # Connect and stream to stdout
    python ble_audio_receiver.py --output recording.wav   # Save to WAV file
    python ble_audio_receiver.py --device "My Device"     # Custom device name
    python ble_audio_receiver.py --timeout 30             # Record for 30 seconds

Dependencies:
    pip install bleak
"""

import asyncio
import argparse
import struct
import sys
import time
import wave
from bleak import BleakScanner, BleakClient

AUDIO_STREAM_CHRC_UUID = "12345678-1234-5678-1234-56789abcdef8"
DEVICE_NAME = "Skynet AI Beacon"

SAMPLE_RATE = 16000
BITS_PER_SAMPLE = 16
NUM_CHANNELS = 1

# IMA ADPCM step size table (89 entries)
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


class AudioReceiver:
    def __init__(self, device_name=DEVICE_NAME, on_audio=None):
        self.device_name = device_name
        self.decoder = ADPCMDecoder()
        self.audio_buffer = bytearray()
        self.on_audio = on_audio
        self.last_seq = -1
        self.total_samples = 0
        self.start_time = None

    def notification_handler(self, sender, data):
        if self.start_time is None:
            self.start_time = time.time()

        if not data or len(data) < 2:
            return

        header = data[0]
        is_sync = (header & 0x80) == 0
        seq = header & 0x7F
        adpcm_payload = data[1:]

        if is_sync:
            print(f"[Sync] Decoder reset at seq={seq}")
            self.decoder.reset()
            self.last_seq = seq
        else:
            expected = (self.last_seq + 1) & 0x7F
            if self.last_seq >= 0 and seq != expected:
                print(f"[Gap] Expected seq={expected}, got seq={seq} "
                      f"(lost {((seq - expected - 1) & 0x7F) + 1} packets)")
            self.last_seq = seq

        pcm_bytes = self.decoder.decode(adpcm_payload)
        self.audio_buffer.extend(pcm_bytes)
        self.total_samples += len(pcm_bytes) // 2

        if self.on_audio:
            self.on_audio(pcm_bytes)

    async def connect_and_stream(self, timeout=None):
        print(f"Scanning for '{self.device_name}'...")
        device = await BleakScanner.find_device_by_filter(
            lambda d, ad: d.name and self.device_name in d.name,
            timeout=10.0
        )

        if not device:
            print(f"Device '{self.device_name}' not found.")
            return

        print(f"Found: {device.name} ({device.address})")
        print(f"Connecting...")

        async with BleakClient(device) as client:
            if not client.is_connected:
                print("Connection failed.")
                return

            print(f"Connected. Subscribing to audio stream...")
            await client.start_notify(AUDIO_STREAM_CHRC_UUID,
                                      self.notification_handler)
            print("Streaming started. Press Ctrl+C to stop.\n")

            try:
                if timeout:
                    await asyncio.sleep(timeout)
                else:
                    while client.is_connected:
                        await asyncio.sleep(0.1)
            except (KeyboardInterrupt, asyncio.CancelledError):
                print("\nStopped by user.")
            finally:
                await client.stop_notify(AUDIO_STREAM_CHRC_UUID)

        elapsed = time.time() - self.start_time if self.start_time else 0
        print(f"\nStream ended. Received {self.total_samples} samples "
              f"({elapsed:.1f}s, {self.total_samples / max(elapsed, 0.001):.0f} Hz)")
        print(f"Buffer size: {len(self.audio_buffer)} bytes")

    def get_audio(self) -> bytes:
        buf = bytes(self.audio_buffer)
        self.audio_buffer.clear()
        return buf

    def save_wav(self, filename):
        if not self.audio_buffer:
            print("No audio data to save.")
            return
        with wave.open(filename, 'wb') as wf:
            wf.setnchannels(NUM_CHANNELS)
            wf.setsampwidth(BITS_PER_SAMPLE // 8)
            wf.setframerate(SAMPLE_RATE)
            wf.writeframes(self.audio_buffer)
        print(f"Saved {len(self.audio_buffer)} bytes to {filename}")


async def main():
    parser = argparse.ArgumentParser(
        description="BLE Audio Stream Receiver for nRF52840 Sense")
    parser.add_argument("--device", "-d", default=DEVICE_NAME,
                        help="BLE device name to connect to")
    parser.add_argument("--output", "-o", default=None,
                        help="Output WAV file (omit for stdout raw PCM)")
    parser.add_argument("--timeout", "-t", type=float, default=None,
                        help="Recording duration in seconds")
    args = parser.parse_args()

    receiver = AudioReceiver(device_name=args.device)

    if args.output is None:
        def on_audio(pcm):
            sys.stdout.buffer.write(pcm)
            sys.stdout.buffer.flush()
        receiver.on_audio = on_audio

    try:
        await receiver.connect_and_stream(timeout=args.timeout)
    finally:
        if args.output:
            receiver.save_wav(args.output)


if __name__ == "__main__":
    asyncio.run(main())
