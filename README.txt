PRAVAH COORDINATED BUILD - OCTOBER 2026

FILES
- PRAVAH_v3_Coordinated.ino: firmware for the ESP32 Dev Module / classic ESP32 board.
- PRAVAH_Control_Centre.py: desktop-local control dashboard that communicates over USB serial.
- PRAVAH_requirements.txt: Python dependency list (pyserial).

FIRMWARE / APP PROTOCOL
The local app sends these commands automatically on connection:
  W ON       Keep the ESP32 awake while app control is active.
  T HHMMSS   Synchronize the firmware clock to the computer's local clock.
  P          Read device status.
While idle, the app resynchronizes the time about once per minute and polls status about every 10 seconds. If the app closes unexpectedly or the USB link disappears, firmware web-control mode times out after about 90 seconds without a command and falls back to standalone scheduling.

COMMANDS AVAILABLE OVER THE SERIAL CONSOLE WHEN NOT USING THE APP
  T HHMMSS   Set the time including seconds (example: T 183045).
  T HHMM     Legacy format also accepted (example: T 1830; seconds set to 00).
  S HHMM     Set daytime start.
  E HHMM     Set daytime end.
  D minutes  Set daytime interval (1 to 720).
  N minutes  Set nighttime interval (1 to 720).
  M          Manual measurement.
  B          Recalibrate the patient-specific baseline.
  X          Erase the stored baseline.
  P          Print status.
  H          Print help.
  R          Reset schedule to defaults; baseline retained.
  W ON/OFF   Enable/disable desktop-app control.

DISPLAY BEHAVIOUR
- On a cold startup, the OLED shows only PRAVAH for about 5 seconds; it does not display the T HHMM command prompt.
- Before a normal manual/scheduled measurement, PRAVAH is displayed while the MPU6050 runs its one-second pre-measurement movement check.
- After the movement check passes, the OLED displays AVF CHECK / LISTENING during microphone acquisition.
- The completed result is shown for about 5 seconds, then the OLED is switched off.
- A motion-only instruction is suppressed during the configured night phase. Genuine abnormal-signal and sensor-error messages are not suppressed.
- Confirmation mode runs repeated measurement blocks for up to 10 minutes; the final confirmation screen is held for about 5 seconds before the display is switched off.

PERSISTENCE / CLOCK
- Patient baseline values and schedule values are stored in ESP32 Preferences/NVS and should survive ordinary power-off/on cycles.
- The baseline is erased by the X command (also by the app's Erase baseline + local history control) or by deliberately erasing all flash. Do not use erase-all-flash for routine restarts.
- The current logical clock uses ESP32 RTC memory and survives deep sleep, but not a complete power removal or cold reset. The desktop app resets it from the computer clock whenever it connects. Standalone use after a full power loss requires T HHMMSS (or T HHMM) again.
- The computer's OS clock must itself be correct. This build syncs to the host computer clock; it does not obtain time directly from NTP over Wi-Fi.

HOW TO USE THE DESKTOP APP
1. Install Python 3 if not already available.
2. Install dependency: py -m pip install -r PRAVAH_requirements.txt
3. Upload PRAVAH_v3_Coordinated.ino using Arduino IDE with Tools > Board > esp32 > ESP32 Dev Module.
4. Close Arduino Serial Monitor.
5. Run: py PRAVAH_Control_Centre.py
6. In the opened browser tab, select the ESP32 COM port and click Connect.
7. The app sends W ON and clock synchronization automatically. Use the buttons for Manual measurement and Calibrate baseline.
8. Use Release / sleep when leaving the app; this sends W OFF and lets the ESP32 return to its standalone schedule.

IMPORTANT LIMITATIONS
- The firmware and application are coordinated for USB serial. The ESP32's built-in Wi-Fi is not enabled for control in this build. Wireless control is technically possible, but it requires a separate network transport, device provisioning, and end-to-end testing; do not assume Wi-Fi is active just because the board supports it.
- Python syntax and local HTTP routes were smoke-tested in this environment. The complete firmware still needs to be compiled and tested with the user's actual Arduino ESP32 core and physical hardware. This notes file does not certify clinical performance or electrical safety.
- PRAVAH is an experimental research prototype; its signal thresholds are not clinically validated and it cannot diagnose or rule out AVF stenosis or thrombosis.
