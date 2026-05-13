import argparse
import asyncio
import json
import logging
import os
import struct
import time
from contextlib import asynccontextmanager

import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException, UploadFile, File
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, JSONResponse, Response
from pydantic import BaseModel

from bleak import BleakScanner
from ble_client import BLEClient, AUDIO_STREAM_CHRC_UUID
from audio_buffer import AudioRingBuffer
from stt_engine import STTEngine
from ai_client import AIClient

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("server")

SAMPLE_RATE = 16000
_here = os.path.dirname(os.path.abspath(__file__))
_project_root = os.path.dirname(_here)

ble_client = BLEClient()
audio_buffer = AudioRingBuffer(max_duration_s=60)
stt_engine = STTEngine(model_size="small", device="cpu", compute_type="int8", lang="de")
ai_client = AIClient()

_loop = None

audio_websockets = set()
stt_websockets = set()
event_websockets = set()
sensor_websockets = set()


def _safe_create_task(coro):
    if _loop is None:
        logger.warning("_safe_create_task: _loop is None, cannot schedule coroutine")
        return
    try:
        asyncio.run_coroutine_threadsafe(coro, _loop)
    except Exception as e:
        logger.error(f"_safe_create_task failed: {e}")


def broadcast_event(event_type: str, data: dict):
    msg = json.dumps({"type": event_type, "data": data})
    for ws in list(event_websockets):
        try:
            _safe_create_task(ws.send_text(msg))
        except Exception:
            event_websockets.discard(ws)


def broadcast_stt_text(text: str, partial: bool = False):
    msg = json.dumps({"type": "partial" if partial else "result", "text": text})
    for ws in list(stt_websockets):
        try:
            _safe_create_task(ws.send_text(msg))
        except Exception:
            stt_websockets.discard(ws)


_audio_chunk_count = 0
_broadcast_sent_count = 0
_session_active = False
_session_buffer = bytearray()

def on_audio_chunk(pcm_bytes: bytes):
    global _audio_chunk_count, _session_active, _session_buffer
    _audio_chunk_count += 1
    if _audio_chunk_count % 50 == 1:
        logger.info(f"on_audio_chunk #{_audio_chunk_count}: {len(pcm_bytes)} bytes, {len(audio_websockets)} ws")
    audio_buffer.write(pcm_bytes)
    if _session_active:
        _session_buffer.extend(pcm_bytes)
    stt_engine.feed_audio(pcm_bytes)
    _safe_create_task(broadcast_audio(pcm_bytes))


async def broadcast_audio(pcm_bytes: bytes):
    global _broadcast_sent_count
    for ws in list(audio_websockets):
        try:
            await ws.send_bytes(pcm_bytes)
            _broadcast_sent_count += 1
            if _broadcast_sent_count % 100 == 1:
                logger.info(f"broadcast_audio sent #{_broadcast_sent_count}: {len(pcm_bytes)}B to WS client")
        except Exception:
            audio_websockets.discard(ws)


def on_ble_connected():
    broadcast_event("ble_connected", {})
    logger.info("BLE connected")


def on_ble_disconnected():
    broadcast_event("ble_disconnected", {})
    logger.info("BLE disconnected")


def on_stt_result(text: str):
    broadcast_stt_text(text, partial=False)
    broadcast_event("stt_result", {"text": text})
    logger.info(f"STT: {text}")


def on_stt_partial(text: str):
    broadcast_stt_text(text, partial=True)


ble_client.on_audio = on_audio_chunk
ble_client.on_connected = on_ble_connected
ble_client.on_disconnected = on_ble_disconnected
ble_client.on_accel = lambda x, y, z: _broadcast_sensor("accel", {"x": round(x,2), "y": round(y,2), "z": round(z,2)})
ble_client.on_gyro = lambda x, y, z: _broadcast_sensor("gyro", {"x": round(x,1), "y": round(y,1), "z": round(z,1)})
ble_client.on_audio_level = lambda rms, zcr: _broadcast_sensor("audio_level", {"rms": rms, "zcr": zcr})
ble_client.on_battery = lambda mv, soc, status: _broadcast_sensor("battery", {"voltage_mv": mv, "soc": soc, "status": status})
ble_client.on_tx_power = lambda dbm: _broadcast_sensor("tx_power", {"dbm": dbm})
stt_engine.on_result = on_stt_result
stt_engine.on_partial = on_stt_partial

_sensor_skip = 0
def _broadcast_sensor(stype: str, data: dict):
    global _sensor_skip
    _sensor_skip += 1
    # Throttle: send every 3rd sensor update (keeps ~20 updates/s)
    if _sensor_skip % 3 != 0:
        return
    msg = json.dumps({"type": stype, "data": data})
    for ws in list(sensor_websockets):
        try:
            _safe_create_task(ws.send_text(msg))
        except Exception:
            sensor_websockets.discard(ws)


class GainModel(BaseModel):
    gain: float


class AIQueryModel(BaseModel):
    text: str
    system_prompt: str | None = None


class DeviceModel(BaseModel):
    name: str = "Skynet AI Beacon"


@asynccontextmanager
async def lifespan(app: FastAPI):
    global _loop
    _loop = asyncio.get_event_loop()
    logger.info("=" * 50)
    logger.info("Skynet AI Audio Bridge starting...")
    logger.info(f"Open http://localhost:8765/ in your browser")
    logger.info(f"Click 'CONNECT' to connect to nRF52 (BLE)")
    logger.info("=" * 50)
    yield
    logger.info("Shutting down...")
    if ble_client.is_connected:
        await ble_client.disconnect()
    stt_engine.stop()
    audio_websockets.clear()
    stt_websockets.clear()
    event_websockets.clear()


app = FastAPI(title="Skynet AI Audio Bridge", version="1.0.0", lifespan=lifespan)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

@app.get("/")
async def serve_index():
    index_path = os.path.join(_project_root, "index.html")
    if os.path.exists(index_path):
        return FileResponse(index_path, media_type="text/html")
    return {"name": "Skynet AI Audio Bridge", "status": "running"}


@app.get("/api/status")
async def get_status():
    return {
        "connected": ble_client.is_connected,
        "notifications_alive": ble_client.notifications_alive,
        "gain": ble_client.gain,
        "audio_duration_s": round(audio_buffer.duration_s, 1),
        "audio_bytes": audio_buffer.available,
        "stt_running": stt_engine.is_running,
        "uptime_s": round(ble_client.elapsed),
        "device_name": ble_client.device_name,
    }


@app.post("/api/gain")
async def set_gain(body: GainModel):
    ble_client.set_gain(body.gain)
    broadcast_event("gain_changed", {"gain": body.gain})
    logger.info(f"Gain set to {body.gain}")
    return {"gain": body.gain}


@app.post("/api/ble/connect")
async def ble_connect(body: DeviceModel = DeviceModel()):
    if ble_client.is_connected:
        raise HTTPException(400, "Already connected")

    ble_client.device_name = body.name
    logger.info(f"Connecting to BLE device '{body.name}'...")

    try:
        await asyncio.wait_for(ble_client.connect(), timeout=20.0)
        logger.info(f"BLE connected: {ble_client.device_name}")

        # Verify characteristic exists by discovering services
        try:
            for service in ble_client.client.services:
                if AUDIO_STREAM_CHRC_UUID.lower() in [str(c.uuid).lower() for c in service.characteristics]:
                    logger.info(f"Audio stream characteristic found in service {service.uuid}")
                    break
        except Exception as e:
            logger.warning(f"Service discovery failed: {e}")

        return {"status": "connected", "device": ble_client.device_name}
    except asyncio.TimeoutError:
        logger.error("BLE connect timed out after 20s")
        raise HTTPException(504, "BLE connect timed out. Check that the device is advertising and in range.")
    except ConnectionError as e:
        logger.error(f"BLE connect failed: {e}")
        raise HTTPException(502, str(e))
    except Exception as e:
        logger.error(f"BLE connect unexpected error: {e}")
        raise HTTPException(500, f"Unexpected error: {e}")


@app.get("/api/ble/scan")
async def ble_scan(timeout: int = 8):
    logger.info(f"Scanning for BLE devices (timeout={timeout}s)...")
    try:
        devices = await BleakScanner.discover(timeout=timeout, return_adv=True)
        results = []
        for addr, (device, adv_data) in devices.items():
            if device.name:
                results.append({"name": device.name, "address": addr, "rssi": adv_data.rssi if adv_data else None})
        results.sort(key=lambda d: d["rssi"] if d["rssi"] is not None else -999, reverse=True)
        logger.info(f"Found {len(results)} named BLE devices")
        return {"devices": results}
    except Exception as e:
        logger.error(f"BLE scan error: {e}")
        raise HTTPException(500, f"Scan failed: {e}")


@app.post("/api/ble/connect")
async def ble_connect(body: DeviceModel = DeviceModel()):
    if ble_client.is_connected:
        raise HTTPException(400, "Already connected")
    ble_client.device_name = body.name
    logger.info(f"Connecting to BLE device '{body.name}'...")
    try:
        await asyncio.wait_for(ble_client.connect(), timeout=20.0)
        logger.info(f"BLE connected: {ble_client.device_name}")
        return {"status": "connected", "device": ble_client.device_name, "notifications_alive": ble_client.notifications_alive}
    except asyncio.TimeoutError:
        raise HTTPException(504, "BLE connect timed out")
    except ConnectionError as e:
        raise HTTPException(502, str(e))
    except Exception as e:
        raise HTTPException(500, f"Unexpected error: {e}")


@app.post("/api/ble/disconnect")
async def ble_disconnect():
    await ble_client.disconnect()


# ── Device Control (reset, deepsleep, settings) ──
RESET_CHAR_UUID = "12345678-1234-5678-1234-56789abcdef6"
OLED_SETTINGS_CHAR_UUID = "12345678-1234-5678-1234-56789abcdef7"

@app.post("/api/device/control")
async def device_control(body: dict):
    mode = body.get("mode", 0)
    logger.info(f"Device control: mode={mode}")
    try:
        ch = ble_client.client.services.get_characteristic(RESET_CHAR_UUID)
        if ch:
            await ble_client.client.write_gatt_char(ch, bytearray([mode]))
            return {"status": "ok", "mode": mode}
        else:
            raise HTTPException(404, "Reset characteristic not found")
    except Exception as e:
        raise HTTPException(500, f"Device control failed: {e}")


@app.get("/api/device/settings")
async def device_settings_get():
    try:
        ch = ble_client.client.services.get_characteristic(OLED_SETTINGS_CHAR_UUID)
        if not ch:
            raise HTTPException(404, "Settings characteristic not found")
        data = await ble_client.client.read_gatt_char(ch)
        return {"settings": list(data)}
    except Exception as e:
        raise HTTPException(500, str(e))


@app.post("/api/device/settings")
async def device_settings_set(body: dict):
    raw = body.get("raw")
    if not raw or not isinstance(raw, list) or len(raw) != 13:
        raise HTTPException(400, "Expected 13-byte settings array")
    try:
        ch = ble_client.client.services.get_characteristic(OLED_SETTINGS_CHAR_UUID)
        if not ch:
            raise HTTPException(404, "Settings characteristic not found")
        await ble_client.client.write_gatt_char(ch, bytearray(raw))
        return {"status": "ok"}
    except Exception as e:
        raise HTTPException(500, str(e))
    return {"status": "disconnected"}


@app.post("/api/stt/start")
async def stt_start():
    if stt_engine.is_running:
        return {"status": "already_running"}
    stt_engine.start()
    broadcast_event("stt_started", {})
    logger.info("STT started")
    return {"status": "started"}


@app.post("/api/stt/stop")
async def stt_stop():
    stt_engine.stop()
    broadcast_event("stt_stopped", {})
    logger.info("STT stopped")
    return {"status": "stopped"}


@app.post("/api/stt/process")
async def stt_process():
    data = audio_buffer.read_all()
    if len(data) < 16000:
        raise HTTPException(400, "Not enough audio data (need ~1s)")
    text = stt_engine.process_file(data)
    logger.info(f"STT batch result: {text}")
    return {"text": text}


@app.post("/api/stt/transcribe")
async def stt_transcribe(file: UploadFile = File(...)):
    import io, wave
    content = await file.read()
    logger.info(f"STT transcribe: {len(content)} bytes from {file.filename}")
    try:
        with wave.open(io.BytesIO(content), 'rb') as wf:
            if wf.getnchannels() != 1 or wf.getsampwidth() != 2:
                raise HTTPException(400, f"Expected mono 16-bit WAV, got {wf.getnchannels()}ch {wf.getsampwidth()*8}bit")
            if wf.getframerate() != 16000:
                logger.warning(f"Sample rate {wf.getframerate()}Hz, resampling not implemented")
            pcm = wf.readframes(wf.getnframes())
        logger.info(f"STT transcribe: {len(pcm)} bytes PCM")
        text = stt_engine.process_file(pcm)
        logger.info(f"STT result: {text}")
        return {"text": text}
    except wave.Error as e:
        raise HTTPException(400, f"Invalid WAV: {e}")


@app.post("/api/ai/query")
async def ai_query(body: AIQueryModel):
    try:
        reply = ai_client.query(body.text, body.system_prompt or None)
        logger.info(f"AI query: {body.text[:50]}... -> {reply[:50]}...")
        return {"response": reply}
    except Exception as e:
        raise HTTPException(500, f"AI query failed: {e}")


@app.post("/api/ai/clear")
async def ai_clear():
    ai_client.clear_history()
    return {"status": "cleared"}


@app.get("/api/ai/models")
async def ai_models():
    return {"models": ai_client.list_models()}


@app.get("/api/audio/buffer")
async def get_audio_buffer():
    data = audio_buffer.read_all()
    return JSONResponse(content={
        "length": len(data),
        "duration_s": round(len(data) / (SAMPLE_RATE * 2), 1),
    })


@app.get("/api/audio/testtone")
async def test_tone():
    import math
    duration_s = 2.0
    freq = 440.0
    sample_rate = 16000
    n_samples = int(duration_s * sample_rate)
    pcm = bytearray(n_samples * 2)
    for i in range(n_samples):
        val = int(math.sin(2 * math.pi * freq * i / sample_rate) * 16000)
        pcm[i * 2] = val & 0xFF
        pcm[i * 2 + 1] = (val >> 8) & 0xFF
    pcm_bytes = bytes(pcm)
    audio_buffer.write(pcm_bytes)
    _safe_create_task(broadcast_audio(pcm_bytes))
    logger.info(f"Test tone generated: {len(pcm_bytes)} bytes ({duration_s}s @ {freq}Hz)")
    return {"length": len(pcm_bytes), "duration_s": duration_s, "freq": freq}


@app.get("/api/audio/testtone/raw")
async def test_tone_raw():
    import math, io, wave
    duration_s = 2.0
    freq = 440.0
    sample_rate = 16000
    n_samples = int(duration_s * sample_rate)
    pcm = bytearray(n_samples * 2)
    for i in range(n_samples):
        val = int(math.sin(2 * math.pi * freq * i / sample_rate) * 16000)
        pcm[i * 2] = val & 0xFF
        pcm[i * 2 + 1] = (val >> 8) & 0xFF

    buf = io.BytesIO()
    with wave.open(buf, 'wb') as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)
        wf.setframerate(16000)
        wf.writeframes(bytes(pcm))
    logger.info(f"Test tone WAV: {len(buf.getvalue())} bytes")
    return Response(content=buf.getvalue(), media_type="audio/wav")

# ── Session Recording (HTTP-based, no WebSocket) ──
@app.post("/api/audio/record/start")
async def record_start():
    global _session_active, _session_buffer

    # Check if BLE connection is truly alive (notifications received recently)
    ble_ok = ble_client.notifications_alive
    if not ble_ok:
        logger.warning(f"BLE not alive (connected={ble_client.is_connected}), attempting fresh connect...")
        if ble_client.is_connected:
            await ble_client.disconnect()
        try:
            await asyncio.wait_for(ble_client.connect(timeout=10.0), timeout=12.0)
            ble_ok = True
            logger.info("BLE connected for recording")
        except asyncio.TimeoutError:
            logger.error("BLE connect timed out during record start")
        except Exception as e:
            logger.error(f"BLE connect failed during record start: {e}")

    _session_active = True
    _session_buffer = bytearray()
    logger.info(f"Recording started (BLE_alive={ble_ok})")
    return {"status": "recording", "started": True, "ble_connected": ble_ok}

@app.post("/api/audio/record/stop")
async def record_stop():
    global _session_active, _session_buffer
    _session_active = False
    pcm_data = bytes(_session_buffer)
    dur = len(pcm_data) / (16000 * 2)
    logger.info(f"Recording stopped: {len(pcm_data)} bytes, {dur:.1f}s")

    import io, wave
    buf = io.BytesIO()
    with wave.open(buf, 'wb') as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)
        wf.setframerate(16000)
        wf.writeframes(pcm_data)
    wav_bytes = buf.getvalue()
    logger.info(f"WAV created: {len(wav_bytes)} bytes")

    return Response(content=wav_bytes, media_type="audio/wav",
                   headers={"Content-Disposition": "attachment; filename=recording.wav",
                            "X-Audio-Duration": str(round(dur, 2)),
                            "X-Audio-Samples": str(len(pcm_data) // 2),
                            "X-BLE-Connected": str(ble_client.is_connected).lower()})


@app.websocket("/ws/audio")
async def ws_audio(websocket: WebSocket):
    await websocket.accept()
    audio_websockets.add(websocket)
    logger.info(f"Audio WebSocket connected (total listeners: {len(audio_websockets)})")
    try:
        while True:
            await websocket.receive_text()
    except WebSocketDisconnect:
        pass
    finally:
        audio_websockets.discard(websocket)
        logger.info("Audio WebSocket disconnected")


@app.websocket("/ws/stt")
async def ws_stt(websocket: WebSocket):
    await websocket.accept()
    stt_websockets.add(websocket)
    logger.info("STT WebSocket connected")
    try:
        while True:
            await websocket.receive_text()
    except WebSocketDisconnect:
        pass
    finally:
        stt_websockets.discard(websocket)
        logger.info("STT WebSocket disconnected")


@app.websocket("/ws/events")
async def ws_events(websocket: WebSocket):
    await websocket.accept()
    event_websockets.add(websocket)
    logger.info("Events WebSocket connected")

    await websocket.send_text(json.dumps({
        "type": "status",
        "data": {
            "connected": ble_client.is_connected,
            "notifications_alive": ble_client.notifications_alive,
            "gain": ble_client.gain,
            "stt_running": stt_engine.is_running,
        }
    }))

    try:
        while True:
            await websocket.receive_text()
    except WebSocketDisconnect:
        pass
    finally:
        event_websockets.discard(websocket)
        logger.info("Events WebSocket disconnected")


@app.websocket("/ws/sensors")
async def ws_sensors(websocket: WebSocket):
    await websocket.accept()
    sensor_websockets.add(websocket)
    logger.info(f"Sensors WebSocket connected (total: {len(sensor_websockets)})")
    try:
        while True:
            await websocket.receive_text()
    except WebSocketDisconnect:
        pass
    finally:
        sensor_websockets.discard(websocket)


def main():
    parser = argparse.ArgumentParser(description="Skynet AI Audio Bridge")
    parser.add_argument("--host", default="0.0.0.0", help="Bind address")
    parser.add_argument("--port", type=int, default=8765, help="Bind port")
    parser.add_argument("--device", default="Skynet AI Beacon",
                        help="BLE device name to connect to")
    parser.add_argument("--model", default="small",
                        choices=["tiny", "base", "small", "medium", "large-v3"],
                        help="Whisper model size for STT (default: small)")
    parser.add_argument("--stt-device", default="cpu",
                        choices=["cpu", "cuda", "auto"],
                        help="Device for STT inference (default: cpu)")
    args = parser.parse_args()
    ble_client.device_name = args.device
    if args.model:
        stt_engine.model_size = args.model
    if args.stt_device:
        stt_engine.device = args.stt_device
    uvicorn.run(app, host=args.host, port=args.port)


if __name__ == "__main__":
    main()
