import json
import logging

logger = logging.getLogger("ai")
try:
    import ollama
except ImportError:
    ollama = None
    logger.warning("ollama not installed")

SYSTEM_PROMPT = "Du bist ein hilfreicher KI-Assistent. Antworte kurz und präzise auf Deutsch."


class AIClient:
    def __init__(self, model="llama3:8b", host="http://localhost:11434"):
        self.model = model
        self.host = host
        self.conversation_history = []
        self._available = ollama is not None

    def set_model(self, model: str):
        self.model = model

    def clear_history(self):
        self.conversation_history.clear()

    def query(self, text: str, system_prompt=SYSTEM_PROMPT, model=None, raw=False) -> str:
        if not self._available:
            return "(Ollama nicht verfügbar)"
        messages = [{"role": "system", "content": system_prompt}]
        if not raw:
            messages.extend(self.conversation_history)
        messages.append({"role": "user", "content": text})

        response = ollama.chat(
            model=model or self.model,
            messages=messages,
            options={"temperature": 0.1} if raw else {},
        )

        reply = response["message"]["content"].strip()

        if not raw:
            self.conversation_history.append({"role": "user", "content": text})
            self.conversation_history.append({"role": "assistant", "content": reply})
            if len(self.conversation_history) > 20:
                self.conversation_history = self.conversation_history[-20:]

        return reply

    def check_available(self) -> bool:
        if not self._available:
            return False
        try:
            ollama.list()
            return True
        except Exception:
            return False

    def list_models(self):
        if not self._available:
            return []
        try:
            models = ollama.list()
            return [m["name"] for m in models.get("models", [])]
        except Exception:
            return []

    @property
    def available(self) -> bool:
        return self._available
