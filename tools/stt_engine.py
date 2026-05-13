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
    logger.warning("faster-whisper not installed")


def _pcm_to_wav_bytes(pcm_data, sample_rate=16000):
    buf = io.BytesIO()
    with wave.open(buf, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(sample_rate)
        w.writeframes(pcm_data)
    return buf.getvalue()


class STTEngine:
    def __init__(self, model_size="small", device="cpu", compute_type="int8", lang="de"):
        self.model_size = model_size
        self.device = device
        self.compute_type = compute_type
        self.lang = lang
        self.model = None
        self._running = False
        self._loading = False
        self._loaded = threading.Event()
        self._thread = None
        self._audio_queue = queue.Queue()
        self.on_result = None
        self.on_partial = None
        self._available = _whisper_available

    def load_model(self):
        if not self._available:
            return False
        if self._loaded.is_set():
            return True
        if self._loading:
            return True
        self._loading = True
        logger.info("Loading faster-whisper {} in background...".format(self.model_size))
        threading.Thread(target=self._do_load, daemon=True).start()
        return True

    def _do_load(self):
        try:
            self.model = WhisperModel(
                self.model_size, device=self.device,
                compute_type=self.compute_type,
                download_root=os.environ.get("WHISPER_MODEL_DIR"),
            )
            self._loaded.set()
            logger.info("Whisper model loaded")
        except Exception as e:
            logger.error("Whisper model load failed: %s", e)
            self._available = False
            self._loaded.set()
        finally:
            self._loading = False

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._run_stream, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None

    def feed_audio(self, pcm_bytes):
        if self._running:
            self._audio_queue.put(pcm_bytes)

    def _run_stream(self):
        chunks = []
        while self._running:
            try:
                chunks.append(self._audio_queue.get(timeout=0.5))
            except queue.Empty:
                continue
        if chunks:
            text = self.process_file(b"".join(chunks))
            if text and self.on_result:
                self.on_result(text)

    def process_file(self, pcm_data):
        if not self._available:
            return "(STT unavailable)"
        if self.model is None:
            self.load_model()
        if not self._loaded.wait(timeout=600):
            return "(STT download timed out)"
        try:
            wav = _pcm_to_wav_bytes(pcm_data)
            segments, _ = self.model.transcribe(
                io.BytesIO(wav), language=self.lang,
                beam_size=5, vad_filter=True,
                vad_parameters=dict(min_silence_duration_ms=500),
            )
            return " ".join(s.text for s in segments).strip()
        except Exception as e:
            logger.error("Transcribe error: %s", e)
            return ""

    @property
    def is_running(self):
        return self._running

    @property
    def available(self):
        return self._available

    @property
    def is_loaded(self):
        return self._loaded.is_set()
