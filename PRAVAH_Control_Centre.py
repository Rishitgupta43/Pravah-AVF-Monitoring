from __future__ import annotations

import json
import re
import threading
import time
import webbrowser
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from queue import Queue, Empty
from typing import Any
from urllib.parse import urlparse

try:
    import serial
    from serial.tools import list_ports
except ImportError as exc:
    serial = None
    list_ports = None
    SERIAL_IMPORT_ERROR = str(exc)
else:
    SERIAL_IMPORT_ERROR = ""

APP_DIR = Path(__file__).resolve().parent
HISTORY_PATH = APP_DIR / "pravah_history.json"
BAUD = 115200
MAX_LOG_LINES = 300
MAX_HISTORY = 500
HOST_TIME_SYNC_SECONDS = 60
STATUS_POLL_SECONDS = 10

state_lock = threading.RLock()
serial_lock = threading.RLock()
serial_conn = None
serial_generation = 0
outbound: Queue[str] = Queue()


def default_state() -> dict[str, Any]:
    return {
        "connected": False, "port": None, "serial_error": SERIAL_IMPORT_ERROR or None,
        "initialized": False, "logs": [], "history": [], "live_signal": None,
        "busy": False, "activity": "Waiting for device connection",
        "clock_valid": False, "device_time": None, "day_start": "07:30", "day_end": "22:30",
        "day_interval": 45, "night_interval": 10, "baseline_valid": False,
        "baseline_mean_hz": None, "baseline_std_hz": None, "baseline_n": 0,
        "baseline_hlpr_mean": None, "baseline_hlpr_std": None,
        "baseline_high_pct_mean": None, "baseline_high_pct_std": None,
        "audio": "Unknown", "mpu6050": "Unknown", "oled": "Unknown",
        "web_control": "Unknown", "signal_status": "Waiting for a measurement",
        "last_measurement": None, "last_seen": None, "next_measurement_at": None,
        "last_host_sync": None,
    }


state = default_state()


def load_history() -> list[dict[str, Any]]:
    try:
        if HISTORY_PATH.exists():
            data = json.loads(HISTORY_PATH.read_text(encoding="utf-8"))
            if isinstance(data, list):
                return data[-MAX_HISTORY:]
    except (OSError, json.JSONDecodeError):
        pass
    return []


state["history"] = load_history()
if state["history"]:
    state["last_measurement"] = state["history"][-1]


def persist_history() -> None:
    try:
        with state_lock:
            payload = json.dumps(state["history"][-MAX_HISTORY:], indent=2, ensure_ascii=False)
        HISTORY_PATH.write_text(payload, encoding="utf-8")
    except OSError as exc:
        add_log(f"[LOCAL APP] Could not save measurement history: {exc}")


def add_log(line: str) -> None:
    line = line.rstrip("\r\n")
    if not line:
        return
    with state_lock:
        state["logs"].append({"time": datetime.now().strftime("%H:%M:%S"), "line": line})
        state["logs"] = state["logs"][-MAX_LOG_LINES:]
        state["last_seen"] = datetime.now().isoformat(timespec="seconds")


def queue_command(command: str) -> None:
    with serial_lock:
        conn = serial_conn
        if conn is None or not getattr(conn, "is_open", False):
            raise RuntimeError("Device is not connected. Select the ESP32 serial port and connect first.")
    outbound.put(command.strip())
    with state_lock:
        state["activity"] = f"Command queued: {command.strip()}"


def write_command(command: str, *, log: bool = True) -> None:
    with serial_lock:
        conn = serial_conn
        if conn is None or not getattr(conn, "is_open", False):
            raise RuntimeError("Device is not connected.")
        conn.write((command.strip() + "\n").encode("ascii", errors="strict"))
        conn.flush()
    # Time-sync commands are deliberately hidden from the dashboard log; the OLED
    # and app UI should not expose the standalone T HHMM command prompt.
    if log and not command.strip().upper().startswith("T "):
        add_log(f"[APP → ESP32] {command.strip()}")


def should_hide_local_clock_help(line: str) -> bool:
    """The standalone serial console keeps full help; the connected app hides boot-only T HHMM hints."""
    low = line.lower().strip()
    return (
        "set the clock after power-up" in low
        or low == "pravah commands"
        or re.match(r"^[tsedn]\s+hhmm", low) is not None
        or low.startswith("use t hhmm")
    )


def _finish_pending_status(pending: dict[str, Any], status: str) -> None:
    pending["status"] = status
    with state_lock:
        for item in reversed(state["history"]):
            if item.get("id") == pending.get("id"):
                item["status"] = status
                break
        last = state.get("last_measurement") or {}
        if last.get("id") == pending.get("id"):
            last["status"] = status
    persist_history()


def parse_firmware_line(line: str, pending: dict[str, Any]) -> bool:
    """Update the dashboard from the firmware's human-readable serial protocol."""
    low = line.lower()
    with state_lock:
        live = re.search(
            r"pravah live:\s*freq=([\d.]+)\s*hz,\s*high=([\d.]+)\s*hz,\s*hlpr=([\d.]+),\s*energy=([\d.]+)%",
            line, re.I,
        )
        if live:
            state["live_signal"] = {
                "frequency_hz": float(live.group(1)), "high_peak_hz": float(live.group(2)),
                "hlpr": float(live.group(3)), "high_energy_pct": float(live.group(4)),
                "timestamp": datetime.now().isoformat(timespec="seconds"),
            }
            state["busy"] = True
            state["activity"] = "Live FFT window received from ESP32"

        if any(x in low for x in (
            "automatic baseline calibration", "calibration batch", "baseline sample",
            "scheduled measurement starting", "manual measurement starting",
            "starting ~4.1 s acoustic capture", "anomaly confirmation: 10-minute",
            "web control active: keeping esp32 awake"
        )):
            if "web control active" not in low:
                state["busy"] = True
            state["activity"] = "Calibrating / capturing / processing measurement" if state["busy"] else "Connected; keeping awake"

        if "mpu6050:" in low:
            state["mpu6050"] = "Working" if re.search(r"mpu6050:\s*ok", line, re.I) else "Error"
        if re.search(r"audio i2s/adc:\s*", line, re.I) or re.search(r"audio:\s*", line, re.I):
            state["audio"] = "Working" if re.search(r"audio(?: i2s/adc)?:\s*ok", line, re.I) else "Error"
        if re.search(r"oled:\s*", line, re.I):
            state["oled"] = "Working" if re.search(r"oled:\s*ok", line, re.I) else "Error"

        if re.search(r"web control:\s*on\b", line, re.I) or "web control enabled" in low:
            state["web_control"] = "On — ESP32 awake for app control"
        elif re.search(r"web control:\s*off\b", line, re.I) or "web control disabled" in low:
            state["web_control"] = "Off — standalone sleep allowed"

        if "clock valid:" in low:
            state["clock_valid"] = "yes" in low
        m = re.search(r"current logical time:\s*(\d{2}:\d{2})", line, re.I)
        if m:
            state["device_time"] = m.group(1)
            state["clock_valid"] = True
        m = re.search(r"\[pravah\]\s*time set to\s*(\d{2}:\d{2})", line, re.I)
        if m:
            state["device_time"] = m.group(1)
            state["clock_valid"] = True
            state["last_host_sync"] = datetime.now().isoformat(timespec="seconds")
        for key, pattern in (("day_start", r"day start:\s*(\d{2}:\d{2})"), ("day_end", r"day end:\s*(\d{2}:\d{2})")):
            m = re.search(pattern, line, re.I)
            if m:
                state[key] = m.group(1)
        m = re.search(r"day interval:\s*(\d+)\s*min", line, re.I)
        if m:
            state["day_interval"] = int(m.group(1))
        m = re.search(r"night interval:\s*(\d+)\s*min", line, re.I)
        if m:
            state["night_interval"] = int(m.group(1))

        if re.search(r"baseline:\s*valid", line, re.I):
            state["baseline_valid"] = True
        elif re.search(r"baseline:\s*not set", line, re.I) or "no valid patient baseline stored" in low or "stored baseline erased" in low:
            state["baseline_valid"] = False
            if "stored baseline erased" in low:
                for key in ("baseline_mean_hz", "baseline_std_hz", "baseline_hlpr_mean", "baseline_hlpr_std", "baseline_high_pct_mean", "baseline_high_pct_std"):
                    state[key] = None
                state["baseline_n"] = 0
                state["activity"] = "Baseline erased; calibrate again before comparing readings"
                state["signal_status"] = "Baseline erased; no comparison available until recalibration"
        m = re.search(r"acoustic baseline:\s*([\d.]+)\s*\+/-\s*([\d.]+)\s*hz(?:,\s*n=(\d+))?", line, re.I)
        if m:
            state["baseline_valid"] = True
            state["baseline_mean_hz"] = float(m.group(1))
            state["baseline_std_hz"] = float(m.group(2))
            if m.group(3):
                state["baseline_n"] = int(m.group(3))
        m = re.search(r"baseline loaded:\s*acoustic=([\d.]+)\s*\+/-\s*([\d.]+)\s*hz,\s*n=(\d+)", line, re.I)
        if m:
            state["baseline_valid"] = True
            state["baseline_mean_hz"] = float(m.group(1))
            state["baseline_std_hz"] = float(m.group(2))
            state["baseline_n"] = int(m.group(3))
        m = re.search(r"hlpr baseline:\s*([\d.]+)\s*\+/-\s*([\d.]+)", line, re.I)
        if m:
            state["baseline_hlpr_mean"] = float(m.group(1))
            state["baseline_hlpr_std"] = float(m.group(2))
        m = re.search(r"high-band energy baseline:\s*([\d.]+)\s*\+/-\s*([\d.]+)\s*%", line, re.I)
        if m:
            state["baseline_high_pct_mean"] = float(m.group(1))
            state["baseline_high_pct_std"] = float(m.group(2))
        m = re.search(r"baseline saved:\s*acoustic=([\d.]+)\s*\+/-\s*([\d.]+)\s*hz,\s*retained=(\d+)/(\d+)", line, re.I)
        if m:
            state["baseline_valid"] = True
            state["baseline_mean_hz"] = float(m.group(1))
            state["baseline_std_hz"] = float(m.group(2))
            state["baseline_n"] = int(m.group(3))
            state["busy"] = False
            state["activity"] = "Patient-specific baseline saved"
        if "calibration failed" in low:
            state["busy"] = False
            state["activity"] = "Calibration failed; see device log"

        if "next scheduled measurement:" in low:
            m = re.search(r"next scheduled measurement:\s*(\d{2}:\d{2})", line, re.I)
            if m:
                state["next_measurement_at"] = m.group(1)

        if "status: within patient baseline" in low or "within patient baseline" in low:
            state["signal_status"] = "Within the stored prototype baseline (not a clinical diagnosis)"
            state["busy"] = False
            state["activity"] = "Measurement complete"
            if pending.get("id"):
                _finish_pending_status(pending, state["signal_status"])
        elif "abnormal access signal" in low:
            state["signal_status"] = "Unusual acoustic pattern — clinical verification required"
            state["busy"] = "entering 10 min confirmation" in low or "confirming" in low
            state["activity"] = "Confirming unusual signal pattern" if state["busy"] else "Measurement complete"
            if pending.get("id"):
                _finish_pending_status(pending, state["signal_status"])
        if "confirmation result: persistent abnormal pattern" in low:
            state["signal_status"] = "Persistent unusual signal pattern — clinical assessment required"
            state["busy"] = False
            state["activity"] = "Confirmation complete"
            if pending.get("id"):
                _finish_pending_status(pending, state["signal_status"])
        elif "confirmation result: readings returned within baseline limits" in low:
            state["signal_status"] = "Signal returned within prototype baseline limits"
            state["busy"] = False
            state["activity"] = "Confirmation complete"
            if pending.get("id"):
                _finish_pending_status(pending, state["signal_status"])
        elif "confirmation result: inconclusive pattern" in low or "confirmation result: insufficient valid data" in low:
            state["signal_status"] = "Inconclusive — no diagnostic conclusion"
            state["busy"] = False
            state["activity"] = "Confirmation complete — inconclusive"
            if pending.get("id"):
                _finish_pending_status(pending, state["signal_status"])

        if any(x in low for x in ("scheduled measurement failed", "scheduled measurement invalid", "manual check not completed", "persistent after retry limit", "insufficient clean acoustic data")):
            state["busy"] = False
            state["activity"] = "Measurement could not be completed"
            state["signal_status"] = "No reliable reading — check stillness, sensor connection, and coupling"

    # Assemble a completed measurement from the terminal-style firmware output.
    if "---------------- pravah reading ----------------" in low:
        pending.clear()
        pending.update({"id": str(time.time_ns()), "mode": "Scheduled", "status": "Waiting for classification"})
    m = re.search(r"measurement type:\s*(manual|scheduled)", line, re.I)
    if m:
        pending["mode"] = m.group(1).title()
    m = re.search(r"dominant acoustic frequency:\s*([\d.]+)\s*hz", line, re.I)
    if m:
        pending["frequency_hz"] = float(m.group(1))
    m = re.search(r"low-band peak:\s*([\d.]+)\s*@\s*([\d.]+)\s*hz", line, re.I)
    if m:
        pending["low_peak"] = float(m.group(1))
        pending["low_peak_hz"] = float(m.group(2))
    m = re.search(r"high-band peak:\s*([\d.]+)\s*@\s*([\d.]+)\s*hz", line, re.I)
    if m:
        pending["high_peak"] = float(m.group(1))
        pending["high_peak_hz"] = float(m.group(2))
    m = re.search(r"hlpr:\s*([\d.]+)", line, re.I)
    if m and "baseline" not in low:
        pending["hlpr"] = float(m.group(1))
    m = re.search(r"600-800 hz energy:\s*([\d.]+)\s*%", line, re.I)
    if m:
        pending["high_energy_pct"] = float(m.group(1))
    m = re.search(r"motion metric:\s*([\d.]+)\s*g delta", line, re.I)
    if m:
        pending["motion_delta_g"] = float(m.group(1))
    m = re.search(r"clean fft windows:\s*(\d+)", line, re.I)
    if m:
        pending["clean_windows"] = int(m.group(1))
    m = re.search(r"measured sample rate:\s*([\d.]+)\s*hz", line, re.I)
    if m:
        pending["sample_rate_hz"] = float(m.group(1))
        pending["timestamp"] = datetime.now().isoformat(timespec="seconds")
        record = dict(pending)
        record.setdefault("mode", "Measurement")
        with state_lock:
            state["last_measurement"] = record
            state["history"].append(record)
            state["history"] = state["history"][-MAX_HISTORY:]
            state["activity"] = f"{record['mode']} reading received"
        persist_history()
        return True
    return False


def serial_worker() -> None:
    global serial_conn, serial_generation
    pending: dict[str, Any] = {}
    seen_generation = -1
    init_at = 0.0
    next_status_at = 0.0
    next_time_sync_at = 0.0
    init_sent = False
    while True:
        with serial_lock:
            conn = serial_conn
            generation = serial_generation
        now = time.monotonic()
        if generation != seen_generation:
            seen_generation = generation
            init_sent = False
            pending.clear()
            init_at = now + 2.0
            next_status_at = now + STATUS_POLL_SECONDS
            next_time_sync_at = now + HOST_TIME_SYNC_SECONDS
            with state_lock:
                state["initialized"] = False
                state["busy"] = False
        if conn is None or not getattr(conn, "is_open", False):
            time.sleep(0.2)
            continue

        if not init_sent and time.monotonic() >= init_at:
            try:
                # Keep device awake, then set the ESP32 clock to the host time including seconds.
                write_command("W ON")
                time.sleep(0.08)
                write_command(datetime.now().strftime("T %H%M%S"))
                time.sleep(0.08)
                write_command("P")
                init_sent = True
                with state_lock:
                    state["initialized"] = True
                    state["activity"] = "Connected; device awake; clock synchronized from this computer"
            except Exception as exc:
                add_log(f"[LOCAL APP] Initialization error: {exc}")
                init_sent = True

        if init_sent:
            try:
                cmd = outbound.get_nowait()
            except Empty:
                cmd = None
            if cmd:
                try:
                    write_command(cmd)
                except Exception as exc:
                    add_log(f"[LOCAL APP] Command failed ({cmd}): {exc}")

        try:
            raw = conn.readline()
            if raw:
                line = raw.decode("utf-8", errors="replace").strip()
                if line:
                    # Hide standalone-only T HHMM boot instructions from the app UI.
                    if not should_hide_local_clock_help(line):
                        add_log(line)
                        parse_firmware_line(line, pending)
            else:
                time.sleep(0.01)
        except Exception as exc:
            with serial_lock:
                is_current = serial_conn is conn
            if is_current:
                add_log(f"[LOCAL APP] Serial read failed: {exc}")
                with state_lock:
                    state["connected"] = False
                    state["serial_error"] = str(exc)
            time.sleep(0.3)

        now = time.monotonic()
        with state_lock:
            busy = state["busy"]
        if init_sent and not busy and now >= next_time_sync_at:
            try:
                # Resync every minute while idle; seconds avoid the up-to-59-second HHMM truncation.
                write_command(datetime.now().strftime("T %H%M%S"), log=False)
                with state_lock:
                    state["last_host_sync"] = datetime.now().isoformat(timespec="seconds")
                next_time_sync_at = now + HOST_TIME_SYNC_SECONDS
            except Exception as exc:
                add_log(f"[LOCAL APP] Time sync deferred: {exc}")
                next_time_sync_at = now + 10.0
        if init_sent and not busy and now >= next_status_at:
            try:
                write_command("P", log=False)
            except Exception:
                pass
            next_status_at = now + STATUS_POLL_SECONDS


PAGE = r'''<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>PRAVAH · Control Centre</title>
<style>
:root{color-scheme:dark;--bg:#08121d;--panel:#101f2e;--panel2:#13283a;--line:#243c50;--text:#e8f1f8;--muted:#9ab0c2;--teal:#54dec4;--blue:#73b8ff;--orange:#f7b86b;--red:#ff7e87;--green:#7fddb1}
*{box-sizing:border-box}body{margin:0;background:radial-gradient(1000px 400px at 12% -10%,#143851 0%,transparent 58%),var(--bg);color:var(--text);font:14px/1.45 Inter,system-ui,-apple-system,Segoe UI,sans-serif}button,input,select{font:inherit}button{border:1px solid #31536b;border-radius:10px;padding:10px 13px;background:#19334a;color:var(--text);cursor:pointer;font-weight:650}button:hover{background:#24465f}button:disabled{opacity:.5;cursor:not-allowed}.primary{background:var(--teal);color:#071a22;border-color:var(--teal)}.danger{background:#3d202c;border-color:#74404e;color:#ffdfe2}.quiet{background:transparent}.wrap{max-width:1400px;margin:auto;padding:24px}.top{display:flex;align-items:center;justify-content:space-between;gap:18px;margin-bottom:22px}.brand{display:flex;gap:13px;align-items:center}.logo{width:48px;height:48px;border-radius:15px;background:linear-gradient(135deg,#54dec4,#3d8ddb);display:grid;place-items:center;color:#071a22;font-size:22px;font-weight:900}.eyebrow{text-transform:uppercase;letter-spacing:.15em;font-size:10px;color:var(--teal);font-weight:800}.brand h1{font-size:23px;margin:1px 0}.sub{color:var(--muted);font-size:12px}.connection{display:flex;align-items:center;gap:10px;background:var(--panel);border:1px solid var(--line);border-radius:12px;padding:9px 11px}.dot{width:9px;height:9px;border-radius:50%;background:#67798a;display:inline-block}.dot.on{background:var(--green);box-shadow:0 0 10px #7fddb177}.connectrow{display:flex;gap:6px;align-items:center;flex-wrap:wrap}.connectrow select{max-width:210px}.grid{display:grid;grid-template-columns:repeat(12,minmax(0,1fr));gap:14px}.card{background:linear-gradient(150deg,#122536,#0f1d2a);border:1px solid var(--line);border-radius:16px;padding:17px;min-width:0;box-shadow:0 12px 34px #00000014}.s3{grid-column:span 3}.s4{grid-column:span 4}.s5{grid-column:span 5}.s6{grid-column:span 6}.s7{grid-column:span 7}.s12{grid-column:span 12}.head{display:flex;align-items:center;justify-content:space-between;gap:10px;margin-bottom:13px}.card h2{font-size:14px;margin:0;font-weight:750}.tag{display:inline-flex;padding:5px 8px;border:1px solid var(--line);border-radius:99px;color:var(--muted);font-size:11px;white-space:nowrap}.good{color:var(--green)!important;border-color:#316d5a!important;background:#16382f}.warn{color:var(--orange)!important;border-color:#725333!important;background:#332a20}.bad{color:var(--red)!important;border-color:#75434a!important;background:#382127}.bigfreq{font-size:40px;font-weight:800;letter-spacing:-.04em;line-height:1.1;margin:8px 0}.unit{font-size:15px;color:var(--muted);font-weight:600}.metrics{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:9px;margin-top:17px}.metric{padding:10px;background:#0b1926;border:1px solid #1e3648;border-radius:10px}.metric .value{font-size:17px;font-weight:750;margin-top:4px;overflow-wrap:anywhere}.label{font-size:11px;color:var(--muted)}.statusmsg{border-left:3px solid var(--teal);padding:10px 12px;background:#0b1b26;border-radius:0 9px 9px 0;color:#d9e8f2;margin-top:14px}.formgrid{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:10px;margin-bottom:13px}.field{display:flex;flex-direction:column;gap:6px}.field label{color:var(--muted);font-size:11px}.field input,.connectrow select{min-height:40px;background:#091724;border:1px solid #2c465a;border-radius:9px;padding:8px 10px;color:var(--text);width:100%;outline:none}.row,.controls{display:flex;gap:8px;align-items:center;flex-wrap:wrap}.note{font-size:11px;color:var(--muted);margin-top:10px}.sensors{display:grid;gap:8px}.sensor{border:1px solid var(--line);background:#0b1926;border-radius:10px;padding:10px}.sensor strong{display:block;font-size:12px;margin-bottom:3px}.sensor span{font-size:11px;color:var(--muted)}.baseline-value{font-size:26px;font-weight:800;margin:5px 0}.divider{height:1px;background:var(--line);margin:15px 0}.history{overflow:auto;max-height:360px}table{border-collapse:collapse;width:100%;font-size:12px;min-width:760px}th{text-align:left;color:var(--muted);font-weight:600;background:#0b1926;position:sticky;top:0}th,td{padding:9px;border-bottom:1px solid #203648;white-space:nowrap}td.status{white-space:normal;min-width:200px}.logs{height:300px;overflow:auto;background:#07121c;border:1px solid #213749;border-radius:10px;padding:10px;font:11px/1.6 ui-monospace,Consolas,monospace}.logline{border-bottom:1px solid #132635;padding:2px 0;overflow-wrap:anywhere}.logtime{color:#5b8299;margin-right:9px}.alert{padding:10px 12px;margin:0 0 14px;border:1px solid #775c35;background:#30291f;color:#f4d3a2;border-radius:10px;font-size:12px}.footer{margin:17px 2px 0;color:#7f95a8;font-size:11px;display:flex;justify-content:space-between;gap:16px;flex-wrap:wrap}.empty{padding:20px;text-align:center;color:var(--muted)}
@media(max-width:1000px){.s3,.s4,.s5,.s6,.s7{grid-column:span 6}.top{align-items:flex-start;flex-direction:column}.formgrid{grid-template-columns:repeat(2,minmax(0,1fr))}}@media(max-width:650px){.wrap{padding:13px}.s3,.s4,.s5,.s6,.s7,.s12{grid-column:span 12}.metrics{grid-template-columns:repeat(2,minmax(0,1fr))}.formgrid{grid-template-columns:repeat(2,minmax(0,1fr))}.connectrow select{max-width:none}.brand h1{font-size:20px}}
</style></head><body><div class="wrap">
<header class="top"><div class="brand"><div class="logo">P</div><div><div class="eyebrow">Acoustic · Accelerometric · Prototype</div><h1>PRAVAH Control Centre</h1><div class="sub">Local USB console · saved history stays on this computer</div></div></div>
<div class="connection"><span id="connDot" class="dot"></span><div><b id="connLabel">Not connected</b><div class="sub" id="connSub">Select the ESP32 serial port</div></div><div class="connectrow"><select id="portSelect" aria-label="Serial port"><option value="">Loading ports…</option></select><button id="connectBtn" class="primary" onclick="connectDevice()">Connect</button><button class="quiet" onclick="releaseDevice()">Release / sleep</button></div></div></header>
<div class="alert">Research prototype only: PRAVAH reports deviations from a stored acoustic baseline. It cannot confirm or rule out stenosis, thrombosis, or a healthy fistula. Do not use it for treatment decisions.</div>
<div class="grid">
<section class="card s5"><div class="head"><h2>Latest acoustic signal</h2><span class="tag" id="signalTag">Awaiting reading</span></div><div class="label">Dominant acoustic peak</div><div class="bigfreq" id="freq">— <span class="unit">Hz</span></div><div class="sub" id="readMeta">No completed reading yet</div><div class="metrics"><div class="metric"><div class="label">High-band peak</div><div class="value" id="highPeak">—</div></div><div class="metric"><div class="label">HLPR</div><div class="value" id="hlpr">—</div></div><div class="metric"><div class="label">600–800 Hz energy</div><div class="value" id="highPct">—</div></div></div><div id="signalMessage" class="statusmsg">Waiting for the ESP32.</div></section>
<section class="card s4"><div class="head"><h2>Patient-specific baseline</h2><span class="tag" id="baselineTag">Not set</span></div><div class="label">Stored acoustic reference</div><div class="baseline-value" id="baselineHz">— <span class="unit">Hz</span></div><div class="sub" id="baselineDetails">No baseline has been loaded yet.</div><div class="divider"></div><div class="row"><button class="primary" onclick="sendAction('calibrate')">Calibrate baseline</button><button onclick="sendAction('refresh')">Refresh status</button></div><div class="note">Calibrate only with an appropriately assessed reference condition. Baseline agreement is not a clinical diagnosis.</div></section>
<section class="card s3"><div class="head"><h2>Device health</h2><span id="webTag" class="tag">Unknown</span></div><div class="sensors"><div class="sensor"><strong>MAX4466 acoustic input</strong><span id="audioStatus">Unknown</span></div><div class="sensor"><strong>MPU6050</strong><span id="mpuStatus">Unknown</span></div><div class="sensor"><strong>OLED</strong><span id="oledStatus">Unknown</span></div></div><div class="note" id="activityText">Waiting for connection.</div></section>
<section class="card s7"><div class="head"><h2>Monitoring schedule</h2><span class="tag" id="timeTag">Clock not synchronized</span></div><div class="formgrid"><div class="field"><label for="dayStart">Day starts</label><input type="time" id="dayStart" value="07:30"></div><div class="field"><label for="dayEnd">Day ends</label><input type="time" id="dayEnd" value="22:30"></div><div class="field"><label for="dayInterval">Day interval (min)</label><input type="number" min="1" max="720" id="dayInterval" value="45"></div><div class="field"><label for="nightInterval">Night interval (min)</label><input type="number" min="1" max="720" id="nightInterval" value="10"></div></div><div class="row"><button class="primary" onclick="saveSchedule()">Save schedule</button><button onclick="sendAction('sync_time')">Sync clock now</button><button class="quiet" onclick="sendAction('refresh')">Refresh status</button></div><div class="note">While connected, this app keeps the ESP32 awake and syncs its clock to this computer approximately once per minute while the device is idle. The computer's own clock must be correct.</div></section>
<section class="card s5"><div class="head"><h2>Manual controls</h2><span class="tag" id="phaseTag">Phase unknown</span></div><div class="controls"><button class="primary" onclick="sendAction('manual')">Run manual measurement</button><button onclick="sendAction('refresh')">Read device status</button></div><div class="divider"></div><div class="row"><button class="danger" onclick="clearSavedData()">Erase baseline + local history</button><button class="danger" onclick="resetSettings()">Reset schedule</button></div><div class="note">Reset schedule retains the baseline. Erasing baseline sends X to the ESP32 and deletes local measurement history.</div></section>
<section class="card s12"><div class="head"><div><h2>Measurement history</h2><div class="sub">Saved in pravah_history.json beside this app</div></div><span class="tag" id="historyCount">0 readings</span></div><div class="history"><table><thead><tr><th>Timestamp</th><th>Type</th><th>Peak (Hz)</th><th>High peak (Hz)</th><th>HLPR</th><th>High energy</th><th>Motion Δg</th><th>Status</th></tr></thead><tbody id="historyBody"><tr><td colspan="8" class="empty">No readings recorded yet.</td></tr></tbody></table></div></section>
<section class="card s12"><div class="head"><div><h2>Device terminal / activity log</h2><div class="sub">Firmware telemetry and app commands</div></div><button onclick="clearVisibleLogs()">Clear visible log</button></div><div id="logs" class="logs"><div class="empty">Connect the ESP32 to see logs.</div></div></section>
</div><div class="footer"><span>PRAVAH local console · binds to 127.0.0.1 only</span><span>USB control is enabled in this build; Wi-Fi control is not enabled.</span></div></div><div id="toast" style="display:none;position:fixed;right:22px;bottom:20px;background:#193b3a;border:1px solid #36766c;padding:12px 16px;border-radius:10px;box-shadow:0 10px 35px #0008;max-width:360px;z-index:3"></div>
<script>
let logFloor=0,lastLogCount=0,editingSchedule=false;
['dayStart','dayEnd','dayInterval','nightInterval'].forEach(id=>$(id).addEventListener('input',()=>editingSchedule=true));
function $(id){return document.getElementById(id)}
function toast(msg){const t=$('toast');t.textContent=msg;t.style.display='block';clearTimeout(window.toastTimer);window.toastTimer=setTimeout(()=>t.style.display='none',3500)}
async function api(url,body){const opts=body?{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)}:{};const r=await fetch(url,opts);const d=await r.json();if(!r.ok||d.ok===false)throw new Error(d.error||'Request failed');return d}
async function loadPorts(){try{const d=await api('/api/ports'),sel=$('portSelect'),old=sel.value;sel.innerHTML='';for(const p of d.ports){const o=document.createElement('option');o.value=p.device;o.textContent=p.device+(p.description?' — '+p.description:'');sel.appendChild(o)}if(!d.ports.length){const o=document.createElement('option');o.value='';o.textContent='No serial ports found';sel.appendChild(o)}if(old)sel.value=old;if(d.error)toast(d.error)}catch(e){toast(e.message)}}
async function connectDevice(){try{const port=$('portSelect').value;if(!port)throw new Error('Select the ESP32 serial port first. Close Arduino Serial Monitor before connecting.');$('connectBtn').disabled=true;await api('/api/connect',{port});toast('USB connected. Enabling app control and synchronizing time…');setTimeout(refreshState,1200)}catch(e){toast(e.message)}finally{$('connectBtn').disabled=false}}
async function releaseDevice(){if(!confirm('Release app control and allow the ESP32 to return to standalone scheduling?'))return;try{await api('/api/disconnect',{});toast('App control released. Standalone sleep scheduling requested.');refreshState()}catch(e){toast(e.message)}}
async function sendAction(action){try{await api('/api/action',{action});toast(({calibrate:'Baseline calibration requested. Keep still and wait for the result.',manual:'Manual measurement requested.',refresh:'Status request sent.',sync_time:'Clock synchronized from the computer.'})[action]||'Command sent.')}catch(e){toast(e.message)}}
async function saveSchedule(){try{const start=$('dayStart').value,end=$('dayEnd').value,di=Number($('dayInterval').value),ni=Number($('nightInterval').value);if(!start||!end||start>=end)throw new Error('Day start must be earlier than day end.');if(!Number.isInteger(di)||!Number.isInteger(ni)||di<1||di>720||ni<1||ni>720)throw new Error('Intervals must be whole minutes from 1 to 720.');await api('/api/schedule',{day_start:start,day_end:end,day_interval:di,night_interval:ni});editingSchedule=false;toast('Schedule update queued.');setTimeout(refreshState,1500)}catch(e){toast(e.message)}}
async function clearSavedData(){if(!confirm('Erase the ESP32 patient baseline AND this computer’s saved measurement history? This cannot be undone.'))return;try{await api('/api/clear-data',{});toast('Baseline erase requested; calibrate again before comparison.');refreshState()}catch(e){toast(e.message)}}
async function resetSettings(){if(!confirm('Restore 07:30–22:30 and 45/10-minute intervals? The baseline will be retained.'))return;try{$('dayStart').value='07:30';$('dayEnd').value='22:30';$('dayInterval').value=45;$('nightInterval').value=10;editingSchedule=false;await api('/api/reset-settings',{});toast('Default schedule requested.');setTimeout(refreshState,700)}catch(e){toast(e.message)}}
function clearVisibleLogs(){logFloor=lastLogCount;lastLogCount=0;$('logs').innerHTML='<div class="empty">Visible log cleared; new logs will appear here.</div>'}
function fmt(v,d=1){return v===null||v===undefined||v===''?'—':Number(v).toFixed(d)}
function esc(s){return String(s??'').replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]))}
function statusClass(s){s=(s||'').toLowerCase();if(s.includes('within')||s.includes('returned')||s.includes('working')||s.includes('on —'))return 'good';if(s.includes('abnormal')||s.includes('unusual')||s.includes('inconclusive')||s.includes('error')||s.includes('not set'))return 'warn';return ''}
function tag(id,txt,cls=''){const e=$(id);e.textContent=txt;e.className='tag '+cls}
function phaseText(now,start,end){return now>=start&&now<end?'Day phase':'Night phase'}
function updateUi(s){$('connDot').className='dot '+(s.connected?'on':'');$('connLabel').textContent=s.connected?'ESP32 connected':'Not connected';$('connSub').textContent=s.port||'Select the ESP32 serial port';$('connectBtn').textContent=s.connected?'Reconnect':'Connect';
if(!editingSchedule){$('dayStart').value=s.day_start||'07:30';$('dayEnd').value=s.day_end||'22:30';$('dayInterval').value=s.day_interval||45;$('nightInterval').value=s.night_interval||10}
const m=s.last_measurement,l=s.live_signal,live=!!l&&(!!s.busy||!m);$('freq').innerHTML=(live?fmt(l.frequency_hz):m?fmt(m.frequency_hz):'—')+' <span class="unit">Hz</span>';$('highPeak').textContent=(live?fmt(l.high_peak_hz):m?fmt(m.high_peak_hz):'—')+' Hz';$('hlpr').textContent=live?fmt(l.hlpr,3):m?fmt(m.hlpr,3):'—';$('highPct').textContent=(live?fmt(l.high_energy_pct):m?fmt(m.high_energy_pct):'—')+'%';$('readMeta').textContent=live?'Live FFT window · '+(l.timestamp||'capturing'):(m?`${m.mode||'Reading'} · ${m.timestamp||'timestamp unavailable'} · ${m.clean_windows??'—'} clean windows`:'No completed reading yet');
const sig=s.signal_status||'Waiting for measurement';$('signalMessage').textContent=sig;tag('signalTag',m?(m.status||'Reading received'):'Awaiting reading',statusClass(m?.status||sig));tag('baselineTag',s.baseline_valid?'Baseline saved':'Not set',s.baseline_valid?'good':'warn');$('baselineHz').innerHTML=s.baseline_valid?`${fmt(s.baseline_mean_hz)} <span class="unit">Hz</span>`:'— <span class="unit">Hz</span>';$('baselineDetails').textContent=s.baseline_valid?`± ${fmt(s.baseline_std_hz)} Hz · ${s.baseline_n||'—'} retained samples · HLPR ${fmt(s.baseline_hlpr_mean,3)}`:'No baseline currently loaded.';
$('audioStatus').textContent=s.audio||'Unknown';$('mpuStatus').textContent=s.mpu6050||'Unknown';$('oledStatus').textContent=s.oled||'Unknown';tag('webTag',s.web_control||'Unknown',statusClass(s.web_control));$('activityText').textContent=(s.activity||'')+(s.last_host_sync?' · Host sync '+s.last_host_sync.slice(11,19):'');tag('timeTag',s.clock_valid?'Clock '+(s.device_time||'valid'):'Clock not synchronized',s.clock_valid?'good':'warn');$('phaseTag').textContent=s.device_time?phaseText(s.device_time,s.day_start,s.day_end):'Phase unknown';$('historyCount').textContent=(s.history||[]).length+' readings';
const hist=(s.history||[]).slice().reverse();$('historyBody').innerHTML=hist.length?hist.map(r=>`<tr><td>${esc(r.timestamp||'—')}</td><td>${esc(r.mode||'—')}</td><td>${fmt(r.frequency_hz)}</td><td>${fmt(r.high_peak_hz)}</td><td>${fmt(r.hlpr,3)}</td><td>${fmt(r.high_energy_pct)}%</td><td>${fmt(r.motion_delta_g,3)}</td><td class="status">${esc(r.status||'Awaiting classification')}</td></tr>`).join(''):'<tr><td colspan="8" class="empty">No recordings stored yet.</td></tr>';
const logs=(s.logs||[]).slice(logFloor);if(logs.length!==lastLogCount){$('logs').innerHTML=logs.length?logs.map(x=>`<div class="logline"><span class="logtime">${esc(x.time)}</span>${esc(x.line)}</div>`).join(''):'<div class="empty">Visible log cleared; new logs will appear here.</div>';$('logs').scrollTop=$('logs').scrollHeight;lastLogCount=logs.length}}
async function refreshState(){try{updateUi(await api('/api/state'))}catch(e){}}
loadPorts();refreshState();setInterval(refreshState,1000);setInterval(loadPorts,12000);
</script></body></html>'''


routes: dict[tuple[str, str], Any] = {}


def route(method: str, path: str):
    def decorate(fn):
        routes[(method, path)] = fn
        return fn
    return decorate


@route("GET", "/")
def index(payload=None):
    return PAGE, "text/html; charset=utf-8", 200


@route("GET", "/api/ports")
def api_ports(payload=None):
    if list_ports is None:
        return {"ports": [], "error": f"pyserial is not installed: {SERIAL_IMPORT_ERROR}"}, "application/json; charset=utf-8", 200
    try:
        return {"ports": [{"device": p.device, "description": p.description or ""} for p in list_ports.comports()]}, "application/json; charset=utf-8", 200
    except Exception as exc:
        return {"ports": [], "error": str(exc)}, "application/json; charset=utf-8", 200


@route("GET", "/api/state")
def api_state(payload=None):
    with state_lock:
        return json.loads(json.dumps(state)), "application/json; charset=utf-8", 200


@route("POST", "/api/connect")
def api_connect(payload):
    global serial_conn, serial_generation
    if serial is None:
        return {"ok": False, "error": f"pyserial missing; install it with pip install pyserial. Details: {SERIAL_IMPORT_ERROR}"}, "application/json; charset=utf-8", 500
    port = str((payload or {}).get("port", "")).strip()
    if not port:
        return {"ok": False, "error": "Select a serial port."}, "application/json; charset=utf-8", 400
    try:
        with serial_lock:
            old = serial_conn
            if old is not None:
                try: old.close()
                except Exception: pass
            serial_conn = serial.Serial(port=port, baudrate=BAUD, timeout=0.2, write_timeout=2)
            # Most ESP32 USB-UART dev boards reboot on port open; toggle the typical auto-reset signals.
            serial_conn.dtr = False
            serial_conn.rts = True
            time.sleep(0.12)
            serial_conn.rts = False
            serial_generation += 1
        while True:
            try: outbound.get_nowait()
            except Empty: break
        with state_lock:
            state["connected"] = True; state["port"] = port; state["serial_error"] = None
            state["initialized"] = False; state["activity"] = "Serial opened; waiting for ESP32 boot output"
        add_log(f"[LOCAL APP] Opened {port} at {BAUD} baud.")
        return {"ok": True, "port": port}, "application/json; charset=utf-8", 200
    except Exception as exc:
        with state_lock:
            state["connected"] = False; state["serial_error"] = str(exc)
        return {"ok": False, "error": f"Could not open {port}: {exc}. Close Arduino Serial Monitor and other apps using this port."}, "application/json; charset=utf-8", 500


@route("POST", "/api/disconnect")
def api_disconnect(payload):
    global serial_conn, serial_generation
    try:
        with serial_lock:
            conn = serial_conn
            if conn is not None and getattr(conn, "is_open", False):
                conn.write(b"W OFF\n"); conn.flush()
        add_log("[APP → ESP32] W OFF")
        time.sleep(0.35)
        with serial_lock:
            if serial_conn is not None:
                try: serial_conn.close()
                except Exception: pass
            serial_conn = None
            serial_generation += 1
        with state_lock:
            state["connected"] = False; state["initialized"] = False
            state["web_control"] = "Off — standalone sleep allowed"
            state["activity"] = "USB released; standalone scheduling requested"
        add_log("[LOCAL APP] Serial connection released.")
        return {"ok": True}, "application/json; charset=utf-8", 200
    except Exception as exc:
        return {"ok": False, "error": str(exc)}, "application/json; charset=utf-8", 503


@route("POST", "/api/action")
def api_action(payload):
    action = (payload or {}).get("action")
    commands = {
        "calibrate": ["B"], "manual": ["M"], "refresh": ["P"],
        "sync_time": [datetime.now().strftime("T %H%M%S"), "P"],
    }
    if action not in commands:
        return {"ok": False, "error": "Unknown action."}, "application/json; charset=utf-8", 400
    try:
        for command in commands[action]:
            queue_command(command)
        return {"ok": True}, "application/json; charset=utf-8", 200
    except Exception as exc:
        return {"ok": False, "error": str(exc)}, "application/json; charset=utf-8", 503


@route("POST", "/api/schedule")
def api_schedule(payload):
    payload = payload or {}
    start, end = str(payload.get("day_start", "")), str(payload.get("day_end", ""))
    try:
        if not re.fullmatch(r"\d{2}:\d{2}", start) or not re.fullmatch(r"\d{2}:\d{2}", end) or start >= end:
            raise ValueError("Day start and end must be valid HH:MM values with start earlier than end.")
        sh, sm = map(int, start.split(":")); eh, em = map(int, end.split(":"))
        if sh > 23 or eh > 23 or sm > 59 or em > 59: raise ValueError("Invalid time.")
        di, ni = int(payload.get("day_interval")), int(payload.get("night_interval"))
        if not 1 <= di <= 720 or not 1 <= ni <= 720: raise ValueError("Intervals must be whole minutes from 1 to 720.")
    except (TypeError, ValueError) as exc:
        return {"ok": False, "error": str(exc)}, "application/json; charset=utf-8", 400
    try:
        with state_lock: current_end = state.get("day_end") or "22:30"
        start_cmd, end_cmd = f"S {start.replace(':', '')}", f"E {end.replace(':', '')}"
        ordered = [end_cmd, start_cmd] if start >= current_end else [start_cmd, end_cmd]
        ordered += [f"D {di}", f"N {ni}", "P"]
        for command in ordered: queue_command(command)
        return {"ok": True}, "application/json; charset=utf-8", 200
    except Exception as exc:
        return {"ok": False, "error": str(exc)}, "application/json; charset=utf-8", 503


@route("POST", "/api/clear-data")
def api_clear_data(payload):
    try:
        queue_command("X")
        with state_lock:
            state["history"] = []; state["last_measurement"] = None; state["baseline_valid"] = False
            for key in ("baseline_mean_hz", "baseline_std_hz", "baseline_hlpr_mean", "baseline_hlpr_std", "baseline_high_pct_mean", "baseline_high_pct_std"):
                state[key] = None
            state["baseline_n"] = 0
            state["signal_status"] = "Baseline erase requested; recalibration is required"
        persist_history()
        return {"ok": True}, "application/json; charset=utf-8", 200
    except Exception as exc:
        return {"ok": False, "error": str(exc)}, "application/json; charset=utf-8", 503


@route("POST", "/api/reset-settings")
def api_reset_settings(payload):
    try:
        queue_command("R")
        return {"ok": True}, "application/json; charset=utf-8", 200
    except Exception as exc:
        return {"ok": False, "error": str(exc)}, "application/json; charset=utf-8", 503


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    def log_message(self, fmt, *args):
        pass
    def _send(self, body, content_type, status):
        raw = body.encode("utf-8") if isinstance(body, str) else json.dumps(body, ensure_ascii=False).encode("utf-8")
        self.send_response(status); self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(raw))); self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff"); self.end_headers()
        try: self.wfile.write(raw)
        except (BrokenPipeError, ConnectionResetError): pass
    def _dispatch(self, method):
        path = urlparse(self.path).path
        fn = routes.get((method, path))
        if fn is None:
            self._send({"ok": False, "error": "Not found"}, "application/json; charset=utf-8", 404); return
        payload = {}
        if method == "POST":
            try:
                size = int(self.headers.get("Content-Length", "0"))
                if size > 1_000_000: raise ValueError("Request body too large.")
                raw = self.rfile.read(size) if size else b"{}"
                payload = json.loads(raw.decode("utf-8")) if raw else {}
                if not isinstance(payload, dict): raise ValueError("JSON body must be an object.")
            except (ValueError, UnicodeDecodeError, json.JSONDecodeError) as exc:
                self._send({"ok": False, "error": f"Invalid JSON: {exc}"}, "application/json; charset=utf-8", 400); return
        try:
            result, content_type, status = fn(payload)
            self._send(result, content_type, status)
        except Exception as exc:
            self._send({"ok": False, "error": str(exc)}, "application/json; charset=utf-8", 500)
    def do_GET(self): self._dispatch("GET")
    def do_POST(self): self._dispatch("POST")


SERIAL_THREAD = threading.Thread(target=serial_worker, daemon=True, name="pravah-serial-reader")
SERIAL_THREAD.start()


def main():
    server = ThreadingHTTPServer(("127.0.0.1", 5000), Handler)
    url = "http://127.0.0.1:5000"
    print("\nPRAVAH Control Centre")
    print(f"Open {url} in your browser. Only this computer can access the server.")
    print("Close Arduino Serial Monitor before connecting. For serial support run: pip install pyserial\n")
    threading.Timer(0.8, lambda: webbrowser.open(url)).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping PRAVAH Control Centre…")
    finally:
        with serial_lock:
            conn = serial_conn
            if conn is not None:
                try:
                    conn.write(b"W OFF\n"); conn.flush()
                except Exception: pass
                try: conn.close()
                except Exception: pass
        server.server_close()


if __name__ == "__main__":
    main()
