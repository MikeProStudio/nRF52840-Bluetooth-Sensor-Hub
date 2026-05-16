import threading
import time

class AudioRingBuffer:
    def __init__(self, max_duration_s=30, sample_rate=16000, channels=1):
        self.sample_rate = sample_rate
        self.channels = channels
        self.bytes_per_sample = 2
        self.capacity = max_duration_s * sample_rate * channels * self.bytes_per_sample
        self.buffer = bytearray(self.capacity)
        self.write_pos = 0
        self.read_pos = 0
        self.filled = 0
        self.lock = threading.Lock()

    def write(self, data: bytes):
        with self.lock:
            length = len(data)
            if length > self.capacity:
                data = data[-self.capacity:]
                length = self.capacity

            if self.write_pos + length <= self.capacity:
                self.buffer[self.write_pos:self.write_pos + length] = data
            else:
                first_chunk = self.capacity - self.write_pos
                self.buffer[self.write_pos:] = data[:first_chunk]
                self.buffer[:length - first_chunk] = data[first_chunk:]

            self.write_pos = (self.write_pos + length) % self.capacity
            self.filled = min(self.filled + length, self.capacity)
            if self.filled == self.capacity:
                self.read_pos = self.write_pos

    def read(self, length: int) -> bytes:
        with self.lock:
            if length > self.filled:
                length = self.filled

            if self.read_pos + length <= self.capacity:
                data = bytes(self.buffer[self.read_pos:self.read_pos + length])
            else:
                first_chunk = self.capacity - self.read_pos
                data = bytes(self.buffer[self.read_pos:] + self.buffer[:length - first_chunk])

            self.read_pos = (self.read_pos + length) % self.capacity
            self.filled -= length
            return data

    def read_all(self) -> bytes:
        return self.read(self.filled)

    def clear(self):
        with self.lock:
            self.write_pos = 0
            self.read_pos = 0
            self.filled = 0

    @property
    def available(self) -> int:
        return self.filled

    @property
    def duration_s(self) -> float:
        return self.filled / (self.sample_rate * self.bytes_per_sample * self.channels)
