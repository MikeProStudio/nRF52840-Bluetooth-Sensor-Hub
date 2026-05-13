import io
import logging
import os
import queue
import threading
import wave

logger = logging.getLogger("stt")

try:
    from faster_whisper import WhisperModel
    _whisper_available = True
except ImportError:
    _whisper_available = False
    logger.warning("faster-whisper not installed. Install: pip install faster-whisper")


def _pcm_to_wav_bytes(pcm_data: bytes, sample_rate: int = 16000) -> bytes:
    """Convert raw PCM16 mono to WAV bytes for Whisper."""
    buf = io.BytesIO()
    with wave.open(buf, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(sample_rate)
        w.writeframes(pcm_data)
    return buf.getvalue()


class STTEngine:
    def __init__(self, model_size="small", device="auto", compute_type="default", lang="de"):
        """
        model_size: "tiny" | "base" | "small" | "medium" | "large-v3"
        device: "cpu" | "cuda" | "auto"
        compute_type: "default" | "float16" | "int8_float16"
        """
        self.model_size = model_size
        self.device = device
        self.compute_type = compute_type
        self.lang = lang
        self.model = None
        self._running = False
        self._thread = None
        self._audio_queue = queue.Queue()
        self.on_result = None
        self.on_partial = None
        self._available = _whisper_available

    def load_model(self):
        if not self._available:
            logger.error("faster-whisper not installed")
            return False
        try:
            logger.info(f"Loading faster-whisper model '{self.model_size}' on {self.device}...")
            self.model = WhisperModel(
                self.model_size,
                device=self.device,
                compute_type=self.compute_type,
                download_root=os.environ.get("WHISPER_MODEL_DIR", None),
            )
            logger.info("Whisper model loaded")
            return True
        except Exception as e:
            logger.error(f"Failed to load whisper model '{self.model_size}': {e}")
            self._available = False
            return False

    def start(self):
        """Start streaming mode (unused for now, placeholder)."""
        self._running = True
        self._thread = threading.Thread(target=self._run_stream, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None

    def feed_audio(self, pcm_bytes: bytes):
        if self._running:
            self._audio_queue.put(pcm_bytes)

    def _run_stream(self):
        """Streaming mode thread (placeholder — batch transcribe is recommended)."""
        chunks = []
        while self._running:
            try:
                data = self._audio_queue.get(timeout=0.5)
                chunks.append(data)
            except queue.Empty:
                continue
        if chunks:
            combined = b"".join(chunks)
            text = self.process_file(combined)
            if text and self.on_result:
                self.on_result(text)

    def process_file(self, pcm_data: bytes) -> str:
        """Transcribe PCM16 mono audio. Returns transcribed text."""
        if not self._available:
            return "(STT unavailable — install faster-whisper)"
        if self.model is None:
            if not self.load_model():
                return "(STT model failed to load)"
        try:
            wav_bytes = _pcm_to_wav_bytes(pcm_data)
            segments, info = self.model.transcribe(
                io.BytesIO(wav_bytes),
                language=self.lang,
                beam_size=5,
                vad_filter=True,
                vad_parameters=dict(min_silence_duration_ms=500),
            )
            text = " ".join(seg.text for seg in segments)
            return text.strip()
        except Exception as e:
            logger.error(f"Whisper transcribe error: {e}")
            return ""

    @property
    def is_running(self) -> bool:
        return self._running

    @property
    def available(self) -> bool:
        return self._available
