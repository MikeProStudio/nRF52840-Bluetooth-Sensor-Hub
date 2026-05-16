# Architekturplan: Lokaler KI-Assistent mit nRF52-Sprachsteuerung

## Ziel
Vollständig lokale KI-Assistant-Software: Sprache via nRF52-Mikrofon → Whisper STT → Ollama/Llama3 Intent-Parsing → PC-Automation (Browser, Apps, System).

---

## 1. Gesamtarchitektur

```
┌──────────────────────────────────────────────────────────────────────────┐
│  nRF52840 Sense (Mikrofon + BLE)                                        │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────────┐              │
│  │ PDM Mikrofon  │→ │ ADPCM Encoder│→ │ BLE GATT Notify │              │
│  │ 16kHz/16bit   │  │ IMA-ADPCM    │  │ 61 Bytes/Paket  │              │
│  └──────────────┘  └──────────────┘  └────────┬─────────┘              │
│                                               │ BLE                     │
└───────────────────────────────────────────────┼──────────────────────────┘
                                                │
                                                ▼ BLE Connection
┌──────────────────────────────────────────────────────────────────────────┐
│  Python Backend (PC) - server.py                                       │
│                                                                         │
│  ┌────────────────────┐   ┌────────────────────┐   ┌─────────────────┐ │
│  │ bleak BLE Client   │ → │ ADPCM Decoder      │ → │ PCM Ringbuffer  │ │
│  │ (alle 3.1 KBit/s)  │   │ → PCM 16kHz/16bit  │   │ (60s Dauer)     │ │
│  └────────────────────┘   └─────────┬──────────┘   └────────┬────────┘ │
│                                     │                         │         │
│                       ┌─────────────▼─────────────┐           │         │
│                       │ Voice Activity Detection   │           │         │
│                       │ (RMS-Energie, ~5 Frames)   │           │         │
│                       └─────────────┬─────────────┘           │         │
│                                     │                         │         │
│                       ┌─────────────▼─────────────┐           │         │
│                       │ Whisper STT (faster-       │           │         │
│                       │ whisper: small/medium/large)│          │         │
│                       │ → Transkribierter Text     │           │         │
│                       └─────────────┬─────────────┘           │         │
│                                     │                         │         │
│                       ┌─────────────▼─────────────────────────▼──┐      │
│                       │ AI Intent Parser (Ollama + Llama3)       │      │
│                       │ → Strukturierte JSON-Kommandos           │      │
│                       │   {"action": "search",                   │      │
│                       │    "target": "browser",                  │      │
│                       │    "query": "Hunde",                     │      │
│                       │    "parameters": {...}}                  │      │
│                       └─────────────┬────────────────────────────┘      │
│                                     │                                   │
│                       ┌─────────────▼────────────────────┐              │
│                       │ Action Executor                   │              │
│                       │ ┌─────────────────────────────┐   │              │
│                       │ │ Browser Control (Playwright) │   │              │
│                       │ │ - Navigate, Search, Click    │   │              │
│                       │ │ - Scroll, Type, Screenshot   │   │              │
│                       │ ├─────────────────────────────┤   │              │
│                       │ │ System Control (PyAutoGUI)   │   │              │
│                       │ │ - App starten/schließen     │   │              │
│                       │ │ - Fenster fokussieren       │   │              │
│                       │ │ - Maus/Tastatur-Simulation  │   │              │
│                       │ ├─────────────────────────────┤   │              │
│                       │ │ OS Integration (subprocess)  │   │              │
│                       │ │ - Dateien öffnen            │   │              │
│                       │ │ - Systembefehle             │   │              │
│                       │ └─────────────────────────────┘   │              │
│                       └────────────────────────────────────┘              │
└──────────────────────────────────────────────────────────────────────────┘
```

---

## 2. Komponenten im Detail

### 2.1 Audio-Pipeline (bereits implementiert)
| Komponente | Status | Beschreibung |
|---|---|---|
| nRF52 PDM Mic → BLE | ✅ Fertig | 16kHz/16bit, IMA-ADPCM, ~33 Pakete/s |
| BLE Client (bleak) | ✅ Fertig | Verbindung, Notification-Handler, ADPCM-Decode |
| Audio Ringbuffer | ✅ Fertig | 60s PCM Puffer |
| VAD (Voice Activity) | ✅ Fertig | RMS-basiert, Schwellwert konfigurierbar |
| Session Recording | ✅ Fertig | HTTP-basiert, WAV-Rückgabe |

### 2.2 STT (bereits implementiert)
| Komponente | Status | Beschreibung |
|---|---|---|
| faster-whisper | ✅ Installiert | small/medium/large-v3 Modelle |
| Transcribe API | ✅ Fertig | WAV-Upload → Text |
| Auto-STT (VAD-Trigger) | ✅ Fertig | Startet bei Spracherkennung |

### 2.3 NEU: AI Intent Parser

**Ollama + Llama3** ist das richtige Tool dafür, weil:
- Lokal, keine Cloud-Abhängigkeit
- Llama3 8B versteht komplexe deutsche Befehle
- Strukturierte JSON-Ausgabe via Prompt-Engineering
- ~6 GB RAM, läuft auf CPU (langsam) oder GPU (empfohlen)

**Prompt-Design für Intent-Parsing:**
```
System: Du bist ein PC-Assistant. Übersetze den Sprachbefehl in ein JSON-Kommando.
Mögliche Aktionen: browser_open, browser_search, browser_click, browser_scroll, 
browser_type, browser_back, browser_close, app_launch, app_close, 
keyboard_type, mouse_click, screenshot, scroll, volume_set, system_command.

Beispiele:
"Öffne Chrome und suche nach Hunden"
→ {"action": "browser_search", "query": "Hunde", "engine": "google"}

"Scrolle runter"
→ {"action": "browser_scroll", "direction": "down", "amount": "half"}

"Öffne den Task-Manager"
→ {"action": "app_launch", "app": "taskmgr"}
```

**Integration in `ai_client.py`:**
- `parse_intent(text: str) → dict` — sendet an Ollama, erwartet JSON
- `execute_command(cmd: dict) → str` — führt die Aktion aus, gibt Status zurück
- Vollständiger Kreislauf: Audio → Text → Intent → Aktion

### 2.4 NEU: Action Executor

| Kategorie | Bibliothek | Aktionen |
|---|---|---|
| **Browser** | Playwright | `browser_open(url)`, `browser_search(query)`, `browser_click(target)`, `browser_scroll(direction)`, `browser_type(text)`, `browser_back()`, `browser_close()` |
| **System** | PyAutoGUI | `app_launch(name)`, `app_close(name)`, `mouse_click(x,y)`, `keyboard_type(text)`, `screenshot()` |
| **OS** | subprocess | `system_command(cmd)`, `file_open(path)`, `volume_set(level)` |

**Browser Control (Playwright):**
```python
import asyncio
from playwright.async_api import async_playwright

class BrowserController:
    def __init__(self):
        self.browser = None
        self.page = None
    
    async def launch(self):
        p = await async_playwright().start()
        self.browser = await p.chromium.launch(headless=False)
        self.page = await self.browser.new_page()
    
    async def search(self, query, engine="google"):
        url = f"https://www.google.com/search?q={query}"
        await self.page.goto(url)
    
    async def scroll(self, direction="down", amount="half"):
        delta = 500 if direction == "down" else -500
        await self.page.evaluate(f"window.scrollBy(0, {delta})")
```

**System Control (PyAutoGUI):**
```python
import pyautogui
import subprocess

class SystemController:
    def launch_app(self, name):
        apps = {
            "chrome": "start chrome",
            "notepad": "notepad",
            "rechner": "calc",
            "task-manager": "taskmgr",
            "explorer": "explorer"
        }
        subprocess.Popen(apps.get(name, name), shell=True)
    
    def volume_set(self, level):
        from ctypes import cast, POINTER
        from comtypes import CLSCTX_ALL
        from pycaw.pycaw import AudioUtilities, IAudioEndpointVolume
        devices = AudioUtilities.GetSpeakers()
        interface = devices.Activate(IAudioEndpointVolume._iid_, CLSCTX_ALL, None)
        volume = cast(interface, POINTER(IAudioEndpointVolume))
        volume.SetMasterVolumeLevelScalar(level / 100, None)
```

### 2.5 NEU: "Hey Jarvis" Wake Word

Zwei Optionen:

| Option | Vorteil | Nachteil |
|---|---|---|
| **Porcupine (Picovoice)** | Sehr leicht (~200 KB), 10ms Latenz | Braucht Access-Key (kostenlos) + Custom-Training für "Hey Jarvis" |
| **faster-whisper tiny** | Kein neues Modell, bereits installiert | ~2s Latenz, ~200 MB RAM |

**Empfohlen: Porcupine** mit einem kostenlosen Access-Key von console.picovoice.ai
- Custom "Hey Jarvis" Modell trainieren (3x einsingen, sofort fertig)
- Integration: Audio-PCM-Frames → porcupine.process() → wenn detected → VAD starten

---

## 3. Dateistruktur (Erweiterung)

| Datei | Zweck |
|---|---|
| `tools/intent_parser.py` | **NEU** — Ollama-Prompt + JSON-Parsing für Sprachbefehle |
| `tools/browser_controller.py` | **NEU** — Playwright-basierte Browser-Steuerung |
| `tools/system_controller.py` | **NEU** — PyAutoGUI + subprocess Systemsteuerung |
| `tools/action_executor.py` | **NEU** — Dispatch-Tabelle für alle Aktionen |
| `tools/wake_word.py` | **NEU** — Porcupine "Hey Jarvis" Detection |
| `tools/server.py` | **ERWEITERN** — Intent-Endpunkt, Executor-Aufruf |
| `tools/ai_client.py` | **ERWEITERN** — Intent-Parsing via Ollama |
| `tools/requirements.txt` | **ERWEITERN** — playwright, pyautogui, pvporcupine |
| `index.html` | **ERWEITERN** — Assistant-Status, Chat-Verlauf |

---

## 4. Datenfluss (vollständig)

```
1. User spricht "Öffne Chrome und suche nach Hunden" (oder beliebiger Befehl)
2. nRF52 Mikrofon → ADPCM → BLE Notification (~33 Pakete/s)
3. Python ble_client decodiert → PCM Ringbuffer
4. VAD erkennt Sprache nach ~150ms → startet Session Recording
5. Sprache endet → VAD erkennt Stille → stoppt Recording
6. WAV wird erstellt → an Whisper gesendet (faster-whisper medium)
7. Text "Öffne Chrome und suche nach Hunden" → an Ollama + Llama3
8. Llama3 gibt JSON: {"action": "browser_search", "query": "Hunde"}
9. Action Executor: Playwright öffnet Chrome → navigiert zu Google → sucht "Hunde"
10. Status "✅ Chrome geöffnet, Suche nach 'Hunde'" → an UI
```

---

## 5. Geschwindigkeit & Ressourcen

| Schritt | Dauer | CPU-Last |
|---|---|---|
| BLE Übertragung | ~0ms (parallel) | 0% |
| ADPCM Decode | ~0.1ms | 0% |
| VAD Erkennung | ~150ms | 0.1% |
| Whisper STT (medium) | ~8-15s | 1-2 Kerne |
| Ollama Llama3 (8B CPU) | ~5-15s | 4-8 Kerne, ~6 GB RAM |
| Aktion ausführen | ~0.5-3s | variabel |

**Gesamtzeit:** ~15-35s von Sprache bis Aktion (je nach Modell).
Mit CUDA GPU: Whisper ~2s + Ollama ~1s = ~3s Gesamtzeit.

---

## 6. Empfehlungen

1. **Ollama + Llama3 8B** ist das richtige Tool — versteht komplexe deutsche Sätze, läuft lokal, gibt strukturiertes JSON aus
2. **Playwright** für Browser-Steuerung (besser als Selenium: schneller, moderner, cross-browser)
3. **VAD vor Whisper** ist richtig — spart Rechenzeit (keine STT bei Stille)
4. **"Hey Jarvis"** später via Porcupine (kostenloser Access-Key + Custom Model)
5. **GPU empfohlen** (NVIDIA CUDA) — beschleunigt Whisper + Ollama um Faktor 5-10x
