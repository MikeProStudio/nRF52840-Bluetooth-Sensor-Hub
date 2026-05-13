import json
import logging
import queue
import threading

logger = logging.getLogger("stt")

try:
    from vosk import Model, KaldiRecognizer
    _vosk_available = True
except ImportError:
    _vosk_available = False
    logger.warning("vosk not installed — STT unavailable. Run: pip install vosk")

SAMPLE_RATE = 16000


class STTEngine:
    def __init__(self, model_path="vosk-model-small-de-0.15", lang="de"):
        self.model_path = model_path
        self.lang = lang
        self.model = None
        self.recognizer = None
        self._running = False
        self._thread = None
        self._audio_queue = queue.Queue()
        self.on_result = None
        self.on_partial = None
        self._available = _vosk_available

    def load_model(self):
        if not self._available:
            logger.error("vosk not installed — cannot load model")
            return False
        try:
            self.model = Model(self.model_path)
            self.recognizer = KaldiRecognizer(self.model, SAMPLE_RATE)
            self.recognizer.SetWords(False)
            return True
        except Exception as e:
            logger.error(f"Failed to load Vosk model '{self.model_path}': {e}")
            self._available = False
            return False

    def start(self):
        if not self._available:
            logger.error("vosk not available")
            return
        if self.model is None:
            if not self.load_model():
                return
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None

    def feed_audio(self, pcm_bytes: bytes):
        if self._running:
            self._audio_queue.put(pcm_bytes)

    def _run(self):
        while self._running:
            try:
                data = self._audio_queue.get(timeout=0.5)
            except queue.Empty:
                continue

            if self.recognizer.AcceptWaveform(data):
                result = json.loads(self.recognizer.Result())
                text = result.get("text", "").strip()
                if text and self.on_result:
                    self.on_result(text)
            else:
                partial = json.loads(self.recognizer.PartialResult())
                ptext = partial.get("partial", "").strip()
                if ptext and self.on_partial:
                    self.on_partial(ptext)

    def process_file(self, pcm_data: bytes) -> str:
        if not self._available:
            return "(STT unavailable — install vosk)"
        if self.model is None:
            if not self.load_model():
                return "(STT unavailable)"
        rec = KaldiRecognizer(self.model, SAMPLE_RATE)
        rec.SetWords(False)
        rec.AcceptWaveform(pcm_data)
        result = json.loads(rec.FinalResult())
        return result.get("text", "").strip()

    @property
    def is_running(self) -> bool:
        return self._running

    @property
    def available(self) -> bool:
        return self._available
