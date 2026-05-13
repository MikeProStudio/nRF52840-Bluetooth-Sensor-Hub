import ollama

SYSTEM_PROMPT = "Du bist ein hilfreicher KI-Assistent. Antworte kurz und präzise auf Deutsch."


class AIClient:
    def __init__(self, model="llama3:8b", host="http://localhost:11434"):
        self.model = model
        self.host = host
        self.conversation_history = []

    def set_model(self, model: str):
        self.model = model

    def clear_history(self):
        self.conversation_history.clear()

    def query(self, text: str, system_prompt=SYSTEM_PROMPT) -> str:
        messages = [{"role": "system", "content": system_prompt}]
        messages.extend(self.conversation_history)
        messages.append({"role": "user", "content": text})

        response = ollama.chat(
            model=self.model,
            messages=messages,
        )

        reply = response["message"]["content"].strip()

        self.conversation_history.append({"role": "user", "content": text})
        self.conversation_history.append({"role": "assistant", "content": reply})

        if len(self.conversation_history) > 20:
            self.conversation_history = self.conversation_history[-20:]

        return reply

    def check_available(self) -> bool:
        try:
            ollama.list()
            return True
        except Exception:
            return False

    def list_models(self):
        try:
            models = ollama.list()
            return [m["name"] for m in models.get("models", [])]
        except Exception:
            return []
