import json
import logging
import re

logger = logging.getLogger("intent")

SYSTEM_PROMPT = """Du bist ein PC-Assistant. Übersetze den Sprachbefehl in ein JSON-Kommando.
Mögliche Aktionen: browser_open, browser_search, browser_click, browser_scroll, browser_type, browser_back, browser_close, browser_refresh, app_launch, app_close, keyboard_type, mouse_click, screenshot, volume_set, system_command.

Antworte NUR mit einem JSON-Objekt. Keine Erklärungen, kein Markdown.
Standard-Suchmaschine ist duckduckgo, nicht google.
Beispiele:
"Öffne Chrome Browser" -> {"action": "app_launch", "app": "chrome"}
"Suche nach Hunden im Internet" -> {"action": "browser_search", "query": "Hunde", "engine": "duckduckgo"}
"Scrolle runter" -> {"action": "browser_scroll", "direction": "down", "amount": "half"}
"Mach einen Screenshot" -> {"action": "screenshot"}
"Lautstärke auf 50" -> {"action": "volume_set", "level": 50}
"Schließe den Browser" -> {"action": "app_close", "app": "chrome"}
""".strip()

# Keyword patterns for commands when Ollama is unavailable
_KEYWORD_RULES = [
    (r'(?:öffne|oeffne|öffnen|oeffnen|starte|mach\s+auf|laufen\s+lassen)\s+(?:den\s+)?(?:chrome|browser|google\s+chrome)', lambda m: {"action": "app_launch", "app": "chrome"}),
    (r'(?:öffne|oeffne|öffnen|oeffnen|starte|mach\s+auf)\s+(?:den\s+)?(?:firefox|feuerfuchs)', lambda m: {"action": "app_launch", "app": "firefox"}),
    (r'(?:öffne|oeffne|öffnen|oeffnen|starte|mach\s+auf)\s+(?:den\s+)?(?:edge|msedge|microsoft\s+edge)', lambda m: {"action": "app_launch", "app": "edge"}),
    (r'(?:öffne|oeffne|öffnen|oeffnen|starte|mach\s+auf)\s+(?:den\s+)?(?:notepad|editor|texteditor)', lambda m: {"action": "app_launch", "app": "notepad"}),
    (r'(?:öffne|oeffne|öffnen|oeffnen|starte|mach\s+auf)\s+(?:den\s+)?(?:task.manager|taskmanager|aufgabenmanager)', lambda m: {"action": "app_launch", "app": "taskmgr"}),
    (r'(?:öffne|oeffne|öffnen|oeffnen|starte|mach\s+auf)\s+(?:den\s+)?(?:rechner|taschenrechner|calc)', lambda m: {"action": "app_launch", "app": "calc"}),
    (r'(?:öffne|oeffne|öffnen|oeffnen|starte|mach\s+auf)\s+(?:den\s+)?(?:explorer|dateimanager)', lambda m: {"action": "app_launch", "app": "explorer"}),
    (r'(?:öffne|oeffne|öffnen|oeffnen|starte)\s+(?:die\s+)?(?:einstellungen|settings)', lambda m: {"action": "app_launch", "app": "settings"}),
    (r'(?:öffne|oeffne|öffnen|oeffnen|starte)\s+(?:das\s+)?(?:terminal|cmd|kommandozeile)', lambda m: {"action": "app_launch", "app": "cmd"}),
    (r'(?:öffne|oeffne|öffnen|oeffnen|starte)\s+(?:den\s+)?(?:code|vs\s*code|visual\s+studio)', lambda m: {"action": "app_launch", "app": "code"}),
    (r'(?:google|suche|such)\s*(?:nach|mal)?\s+(.+)', lambda m: {"action": "browser_search", "query": m.group(1).strip(), "engine": "duckduckgo"}),
    (r'(?:suche\s+nach|such\s+mal)\s+(.+)', lambda m: {"action": "browser_search", "query": m.group(1).strip(), "engine": "duckduckgo"}),
    (r'(?:scroll|blätter|blatter)\s*(?:runter|nach\s+unten)?', lambda m: {"action": "browser_scroll", "direction": "down", "amount": "half"}),
    (r'(?:scroll|blätter|blatter)\s+(?:nach\s+)?(?:oben|rauf|hoch)', lambda m: {"action": "browser_scroll", "direction": "up", "amount": "half"}),
    (r'(?:schließ|schliess|zumachen|beenden)\s+(?:den\s+)?(?:browser|chrome|tab|fenster)', lambda m: {"action": "browser_close"}),
    (r'(?:mach|erstelle|nehmen\s+sie)\s+(?:einen|ein)\s+(?:screenshot|bildschirmfoto)', lambda m: {"action": "screenshot"}),
    (r'(?:lautstärke|lautstaerke|volume|leiser|lauter)\s*(?:auf|:)?\s*(\d+)', lambda m: {"action": "volume_set", "level": int(m.group(1))}),
    (r'(?:leiser|leise)', lambda m: {"action": "volume_set", "level": 30}),
    (r'(?:lauter|laut)', lambda m: {"action": "volume_set", "level": 70}),
    (r'(?:zurück|zurueck|back)', lambda m: {"action": "browser_back"}),
    (r'(?:aktualisieren|refresh|neu\s+laden)', lambda m: {"action": "browser_refresh"}),
    (r'(?:hilfe|help|was\s+kannst\s+du)', lambda m: {"action": "help"}),
]


class IntentParser:
    def __init__(self, ai_client, model="llama3:8b", system_prompt=SYSTEM_PROMPT):
        self.ai_client = ai_client
        self.model = model
        self.system_prompt = system_prompt

    def parse(self, text: str) -> dict:
        if not text or not text.strip():
            return {"action": "none", "reason": "empty_transcript"}
        text = text.strip()

        # 1. Try Ollama if available
        if self.ai_client.available:
            try:
                response = self.ai_client.query(
                    text=text,
                    system_prompt=self.system_prompt,
                    model=self.model,
                    raw=True
                )
                cmd = self._extract_json(response)
                if cmd.get("action") and cmd["action"] != "none":
                    logger.info("Ollama parsed: %s", json.dumps(cmd))
                    return cmd
            except Exception as e:
                logger.warning("Ollama parsing failed: %s", e)

        # 2. Fallback: keyword matching
        cmd = self._keyword_match(text)
        if cmd:
            logger.info("Keyword match: %s", json.dumps(cmd))
            return cmd

        # 3. Final fallback: treat as browser search
        logger.info("No match, treating as browser_search: %s", text)
        return {"action": "browser_search", "query": text, "engine": "duckduckgo"}

    def _keyword_match(self, text: str) -> dict | None:
        text_lower = text.lower().strip()
        for pattern, handler in _KEYWORD_RULES:
            m = re.search(pattern, text_lower)
            if m:
                return handler(m)
        return None

    def _extract_json(self, text: str) -> dict:
        json_match = re.search(r'\{.*\}', text, re.DOTALL)
        if json_match:
            try:
                return json.loads(json_match.group())
            except json.JSONDecodeError:
                pass
        try:
            return json.loads(text.strip())
        except (json.JSONDecodeError, ValueError):
            return {"action": "none", "reason": "no_json_in_response"}
