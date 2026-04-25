This repo contains firmware for the Flying Phoque 7. Used for the SSI Rockets Avionics workshop. See [here](https://drive.google.com/drive/folders/1tiUvKedx1bPB-v8GK1kLOYGjgW4yYmwD?usp=share_link) for details.

---

# Launch Day Cheat Sheet (offline, no internet required)

Every command below is copy-paste ready and assumes you just opened a **brand new terminal**. The toolchain (PlatformIO and the SHITL `.venv`) is already installed on this machine — these steps never hit the network.

> **Note:** `uv` is **not** installed on this laptop. All SHITL commands below use the pre-built virtualenv at `Airbrakes_SHITL/.venv` directly. Ignore any online docs that tell you to run `uv run ...`.

## 0. Know your serial ports

| Board | Port |
| --- | --- |
| Airbrakes flying seal | `/dev/cu.usbserial-AU0K3FHU` |
| On-sled flying seal   | `/dev/cu.usbserial-AU0K4AAD` |

To confirm what's actually plugged in:
```
ls /dev/cu.usbserial-*
```

If a port is missing, unplug/replug the USB and check again. If it still doesn't show, try a different cable (data, not charge-only).

---

## 1. Flash new firmware to the Teensy

From a blank terminal:
```
cd ~/Documents/airbrakes-teensy-controls
pio run -e teensy41 -t upload
```

/Users/noahweiss/.platformio/penv/bin/pio run -e teensy41 -t upload

That compiles + flashes in one shot. If upload hangs or errors out with "no Teensy found":
1. Physically press the small program button on the Teensy.
2. Re-run the upload command above.

Other useful firmware commands (same directory):
```
pio run -e teensy41                                      # compile only, no flash
pio run -e teensy41 -t clean                             # wipe build artifacts
pio device monitor -e teensy41 -b 115200                 # open serial monitor
pio device monitor -b 115200 --port /dev/cu.usbserial-AU0K3FHU   # force a specific port
```

To exit the serial monitor: **Ctrl-C, then Ctrl-]**.

If the monitor prints garbage, the baud is wrong — it must be `115200`.

### WiFi AirLift bringup (bench only, NOT for launch)
Replaces the main firmware with a TCP echo sketch on port 4040. Only for testing ESP32 wiring.
```
cd ~/Documents/airbrakes-teensy-controls
pio run -e wifi_bringup -t upload
pio device monitor -e wifi_bringup
```
To restore flight firmware afterward:
```
pio run -e teensy41 -t upload
```

---

## 2. Boot the SHITL dashboard

From a blank terminal — run these three lines every time:
```
cd ~/Documents/airbrakes-teensy-controls/Airbrakes_SHITL
source .venv/bin/activate
python shitl_dashboard.py
```

You should see `(.venv)` appear at the start of your prompt after the `source` line — that means the venv is active.

Then open a browser to: **http://127.0.0.1:8080**

The command above runs `demo` mode (no hardware needed). For other modes — after `source .venv/bin/activate`, run one of:

```
# Replay recorded CSV flight data through the Teensy
python shitl_dashboard.py --mode replay      --port /dev/cu.usbserial-AU0K3FHU

# Full closed-loop sim with the Teensy in the loop
python shitl_dashboard.py --mode closed-loop --port /dev/cu.usbserial-AU0K3FHU

# Live flight monitoring (real Teensy flight data)
python shitl_dashboard.py --mode flight      --port /dev/cu.usbserial-AU0K3FHU

# Bench test (Teensy reads own sensors, streams telemetry)
python shitl_dashboard.py --mode test        --port /dev/cu.usbserial-AU0K3FHU

# Bench/ground sensor movement check (real sensors + UKF, logs to SD)
python shitl_dashboard.py --mode sensor-moving --port /dev/cu.usbserial-AU0K3FHU
```

### If port 8080 is already taken
```
python shitl_dashboard.py --web-port 8081
```
Then browse to http://127.0.0.1:8081.

### `source .venv/bin/activate` fails ("no such file")
The venv got wiped. Rebuild it (requires internet — do this **before** leaving for the launch site):
```
cd ~/Documents/airbrakes-teensy-controls/Airbrakes_SHITL
python3.12 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt        # or: pip install pyserial numpy matplotlib
```

---

## 3. Kill the SHITL dashboard

**In the terminal where it's running:** press `Ctrl-C` (once; give it a second; hit it again if it doesn't exit).

**If Ctrl-C doesn't work** or you closed the terminal and it's still running, open a new terminal and nuke it by port or name:
```
# Kill anything bound to the dashboard's web port
lsof -ti tcp:8080 | xargs kill -9

# Or kill by process name
pkill -f shitl_dashboard.py
```

Verify it's actually dead:
```
lsof -i tcp:8080
```
(no output = gone)

---

## 4. Headless SHITL scripts (no web UI)

From a blank terminal:
```
cd ~/Documents/airbrakes-teensy-controls/Airbrakes_SHITL
source .venv/bin/activate
python shitl.py                    # CSV replay → Teensy, TCP out on 127.0.0.1:7000
python shitl_closed_loop.py        # closed-loop RK4 sim + matplotlib plot
python shitl_visualize.py          # replay + live matplotlib telemetry
```
Kill any of these with `Ctrl-C`.

---

## 5. Quick recovery

| Symptom | Fix |
| --- | --- |
| Teensy won't flash / "no Teensy found" | Press the program button on the Teensy, re-run upload. |
| Wrong or missing serial port | `ls /dev/cu.usbserial-*`, then pass `--port <path>` explicitly. |
| Serial monitor shows garbage | Baud must be `115200`. |
| Dashboard port 8080 in use | `--web-port 8081` (and browse to that port). |
| SHITL can't open the serial port | Close any `pio device monitor` or Serial Studio session first — only one process can hold the port. |
| Dashboard stuck / frozen | `pkill -f shitl_dashboard.py`, then relaunch. |
| `zsh: command not found: uv` | Expected — `uv` isn't installed. Use `source .venv/bin/activate` then `python ...` instead. |
| `source .venv/bin/activate: no such file` | The venv is gone. See "venv activate fails" under section 2 to rebuild (needs internet). |
