# Architektur-Konzept: Web-Dashboard für XIAO BLE Sense

## 1. Übersicht
Da das nRF52840-Modul (XIAO BLE Sense) kein natives WLAN besitzt, kann es keine Webseite per IP (HTTP/HTTPS) direkt hosten. Um dennoch eine professionelle Web-Oberfläche zu bieten, nutzen wir die **Web Bluetooth API**.

## 2. Architektur-Komponenten

### A. Embedded Firmware (XIAO BLE Sense)
- **Rolle:** GATT Server.
- **Dienst:** Custom IMU Service (UUID: `...def0`).
- **Characteristics:** 
  - Accel Data (Notify)
  - Gyro Data (Notify)
- **Status:** Das Board sendet Rohdaten (milli-units) via BLE an den Client.

### B. Web-Client (Der Browser)
- **Technologie:** HTML5, CSS3, JavaScript (Web Bluetooth API).
- **Funktionsweise:** 
  - Der Browser agiert als Gateway. Er verbindet sich per Bluetooth direkt mit dem XIAO-Board.
  - Die Daten werden lokal im Browser verarbeitet, umgerechnet (z.B. Rad -> Deg) und visualisiert.

## 3. Web-Zugriff über IP
Um die Web-UI über eine IP-Adresse im Netzwerk aufrufbar zu machen, gibt es zwei Wege:

### Option 1: Lokales Hosting (Entwicklung)
Die Datei `web_ui.html` wird auf einem lokalen PC/Server gehostet (z.B. via Nginx, Apache oder Python `http.server`).
- **URL:** `http://192.168.x.x:8000/web_ui.html`
- **Wichtig:** Web Bluetooth erfordert zwingend **HTTPS** (außer für `localhost`). Für den Netzwerkzugriff muss ein SSL-Zertifikat vorhanden sein.

### Option 2: Gateway-Architektur (Professionell)
Ein Raspberry Pi oder ein PC dient als Bridge:
1. **Backend:** Ein Python-Skript (mittels `bleak`) verbindet sich mit dem XIAO Board.
2. **Webserver:** Ein Flask/FastAPI Server stellt die Webseite bereit und streamt die Daten via **WebSockets** an alle verbundenen Browser.
3. **Vorteil:** Mehrere Nutzer können gleichzeitig über die IP die Daten sehen, ohne dass jeder eine eigene BLE-Verbindung aufbauen muss.

## 4. Aktueller Implementierungsstatus
Die bereitgestellte `web_ui.html` nutzt **Option 1 (Direkte Web Bluetooth Verbindung)**. Dies ist die effizienteste Lösung ohne zusätzliche Hardware-Gateways.

**Sicherheits-Hinweis:** Stellen Sie sicher, dass Sie die Seite über einen Webserver aufrufen, nicht direkt als lokale Datei (`file://`), da Browser den Zugriff auf Bluetooth aus Sicherheitsgründen für lokale Dateien blockieren.
