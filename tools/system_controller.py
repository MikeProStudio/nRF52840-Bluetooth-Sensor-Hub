import logging
import subprocess
import sys

logger = logging.getLogger("system")

_pyautogui_available = False
try:
    import pyautogui
    _pyautogui_available = True
except ImportError:
    logger.warning("pyautogui not installed. Install: pip install pyautogui")

# Windows: use PowerShell Start-Process for proper window foreground + maximized
_PS_START = 'powershell -Command "Start-Process'

# Windows app name map
APP_MAP = {
    "chrome": f'{_PS_START} chrome -WindowStyle Maximized"',
    "firefox": f'{_PS_START} firefox -WindowStyle Maximized"',
    "edge": f'{_PS_START} msedge -WindowStyle Maximized"',
    "notepad": "notepad",
    "rechner": "calc",
    "task-manager": "taskmgr",
    "taskmanager": "taskmgr",
    "explorer": "explorer",
    "code": "code",
    "cmd": "cmd",
    "terminal": "wt",
    "einstellungen": f'{_PS_START} ms-settings: -WindowStyle Maximized"',
    "settings": f'{_PS_START} ms-settings: -WindowStyle Maximized"',
}

BROWSER_NAMES = ["chrome", "firefox", "edge", "brave", "opera"]


class SystemController:
    def __init__(self):
        self._available = True

    def launch_app(self, name: str) -> str:
        name = name.lower().strip()
        cmd = APP_MAP.get(name)
        if cmd:
            subprocess.Popen(cmd, shell=True)
            logger.info("Launched: %s", name)
            if name in BROWSER_NAMES:
                return f"{name.capitalize()} gestartet"
            return f"Anwendung '{name}' gestartet"

        try:
            subprocess.Popen(f"start {name}", shell=True)
            return f"'{name}' gestartet"
        except Exception as e:
            logger.error("Launch failed: %s", e)
            return f"Konnte '{name}' nicht starten: {e}"

    def close_app(self, name: str) -> str:
        name = name.lower().strip()
        try:
            subprocess.run(f"taskkill /f /im {name}.exe" if sys.platform == "win32"
                           else f"pkill -f {name}",
                           shell=True, capture_output=True)
            return f"'{name}' geschlossen"
        except Exception as e:
            return f"Fehler beim Schließen: {e}"

    def volume_set(self, level: int) -> str:
        if not 0 <= level <= 100:
            return "Lautstärke muss zwischen 0 und 100 liegen"
        try:
            from ctypes import cast, POINTER
            from comtypes import CLSCTX_ALL
            from pycaw.pycaw import AudioUtilities, IAudioEndpointVolume
            devices = AudioUtilities.GetSpeakers()
            interface = devices.Activate(IAudioEndpointVolume._iid_, CLSCTX_ALL, None)
            volume = cast(interface, POINTER(IAudioEndpointVolume))
            volume.SetMasterVolumeLevelScalar(level / 100.0, None)
            return f"Lautstärke auf {level}% gesetzt"
        except ImportError:
            logger.warning("pycaw not installed, using pyautogui fallback")
            if _pyautogui_available:
                import pyautogui as pg
                pg.press("volumeup", presses=max(0, (level - 50) // 2))
                pg.press("volumedown", presses=max(0, (50 - level) // 2))
                return f"Lautstärke ~{level}%"
            return "Lautstärke-Änderung nicht verfügbar (pycaw fehlt)"
        except Exception as e:
            return f"Lautstärke-Fehler: {e}"

    def keyboard_type(self, text: str) -> str:
        if not _pyautogui_available:
            return "pyautogui nicht installiert"
        import pyautogui as pg
        pg.typewrite(text, interval=0.05)
        return f"Text eingegeben: {text[:50]}..."

    def mouse_click(self, x: int = None, y: int = None) -> str:
        if not _pyautogui_available:
            return "pyautogui nicht installiert"
        import pyautogui as pg
        if x is not None and y is not None:
            pg.click(x, y)
            return f"Geklickt bei ({x}, {y})"
        pg.click()
        return "Geklickt"

    def screenshot(self, path: str = None) -> str:
        if not _pyautogui_available:
            return "pyautogui nicht installiert"
        import pyautogui as pg
        if not path:
            import tempfile
            path = tempfile.mktemp(suffix=".png")
        pg.screenshot(path)
        return f"Screenshot: {path}"

    def run_command(self, cmd: str) -> str:
        try:
            result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=10)
            output = result.stdout.strip() or result.stderr.strip()
            logger.info("Command '%s' returned: %s", cmd, output[:100])
            return output or f"Befehl ausgeführt: {cmd}"
        except subprocess.TimeoutExpired:
            return f"Befehl '{cmd}' zeitüberschreitung"
        except Exception as e:
            return f"Fehler: {e}"
