#!/usr/bin/env python3
"""Pull the highest-numbered LOGNNN.TXT off the Teensy SD card over USB serial.

Speaks the same line protocol as the dashboard's Files tab (LIST,SD then
READ,SD,<name>) — see handleSerialCommand() in src/main.cpp. Filters out
non-SD lines (telemetry, STATUS heartbeats) so they don't clutter the
download. Writes the file next to this script as <name>.

Usage:
    python tools/pull_latest_log.py [--port /dev/cu.usbmodemXXX] [--out DIR]
"""

import argparse
import re
import sys
import time
from pathlib import Path

import serial
import serial.tools.list_ports


def find_teensy_port() -> str:
    # Teensy 4.1 enumerates as a usbmodem on macOS. If multiple match, take
    # the first — the operator can override with --port.
    for p in serial.tools.list_ports.comports():
        dev = p.device
        if "usbmodem" in dev or "usbserial" in dev:
            return dev
    raise SystemExit("No Teensy serial port found. Plug in the Teensy or pass --port.")


def read_until(ser: serial.Serial, predicate, timeout_s: float):
    """Yield decoded lines until predicate(line) is True or we time out."""
    deadline = time.monotonic() + timeout_s
    buf = b""
    while time.monotonic() < deadline:
        chunk = ser.read(ser.in_waiting or 1)
        if not chunk:
            continue
        buf += chunk
        while b"\n" in buf:
            line, _, buf = buf.partition(b"\n")
            text = line.decode("utf-8", errors="replace").rstrip("\r\n")
            yield text
            if predicate(text):
                return
    raise TimeoutError(f"timed out after {timeout_s:.1f}s")


def list_logs(ser: serial.Serial) -> list[tuple[str, int]]:
    """Send LIST,SD and parse the SD_LIST_BEGIN..SD_LIST_END block."""
    ser.reset_input_buffer()
    ser.write(b"LIST,SD\n")
    ser.flush()
    in_block = False
    files: list[tuple[str, int]] = []
    for line in read_until(ser, lambda l: l.startswith("SD_LIST_END"), timeout_s=10.0):
        if line == "SD_LIST_BEGIN":
            in_block = True
            continue
        if line.startswith("SD_LIST_END"):
            break
        if in_block and "," in line:
            # Format from main.cpp: "<name>,<size>"
            name, _, size = line.rpartition(",")
            try:
                files.append((name, int(size)))
            except ValueError:
                # Not an SD entry — likely a telemetry row that snuck in.
                pass
    return files


LOG_RE = re.compile(r"^LOG(\d+)\.TXT$", re.IGNORECASE)


def latest_log(files: list[tuple[str, int]]) -> tuple[str, int] | None:
    best: tuple[int, str, int] | None = None
    for name, size in files:
        m = LOG_RE.match(name)
        if not m:
            continue
        n = int(m.group(1))
        if best is None or n > best[0]:
            best = (n, name, size)
    return (best[1], best[2]) if best else None


def download(ser: serial.Serial, name: str, expected_size: int, out_path: Path):
    """Send READ,SD,<name> and accumulate SD_LINE,<content> rows into out_path."""
    ser.reset_input_buffer()
    ser.write(f"READ,SD,{name}\n".encode())
    ser.flush()

    # Wait for SD_FILE_BEGIN — anything before it is unrelated chatter.
    actual_size = expected_size
    saw_begin = False
    deadline = time.monotonic() + 10.0
    while not saw_begin and time.monotonic() < deadline:
        for line in read_until(ser,
                               lambda l: l.startswith("SD_FILE_BEGIN,") or l.startswith("SD_FILE_ERR,"),
                               timeout_s=10.0):
            if line.startswith("SD_FILE_ERR,"):
                raise SystemExit(f"firmware refused: {line}")
            if line.startswith("SD_FILE_BEGIN,"):
                # Format: SD_FILE_BEGIN,<name>,<size>
                rest = line[len("SD_FILE_BEGIN,"):]
                parts = rest.split(",")
                if len(parts) >= 2:
                    try:
                        actual_size = int(parts[1])
                    except ValueError:
                        pass
                saw_begin = True
                break
        if saw_begin:
            break
    if not saw_begin:
        raise TimeoutError("never saw SD_FILE_BEGIN")

    out_path.parent.mkdir(parents=True, exist_ok=True)
    bytes_written = 0
    line_count = 0
    last_progress = time.monotonic()
    sentinel = f"SD_FILE_END,{name}"

    with out_path.open("w", encoding="utf-8") as out:
        for line in read_until(ser, lambda l: l == sentinel or l.startswith("SD_FILE_END,"),
                               timeout_s=300.0):
            if line.startswith("SD_LINE,"):
                content = line[len("SD_LINE,"):]
                out.write(content + "\n")
                bytes_written += len(content) + 1
                line_count += 1
                now = time.monotonic()
                if now - last_progress >= 0.5:
                    last_progress = now
                    pct = (100.0 * bytes_written / actual_size) if actual_size else 0.0
                    print(f"  {bytes_written:>9} bytes / {actual_size} "
                          f"({pct:5.1f}%, {line_count} lines)", flush=True)
            elif line.startswith("SD_FILE_END,"):
                break
            elif line.startswith("SD_FILE_ERR,"):
                raise SystemExit(f"firmware error mid-stream: {line}")
            # else: ignore (telemetry/status interleaved)

    print(f"  {bytes_written:>9} bytes / {actual_size} (done, {line_count} lines)")
    return bytes_written, line_count


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", help="Serial port (defaults to first usbmodem found)")
    ap.add_argument("--out", default="downloaded_logs", help="Output directory (default: ./downloaded_logs)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--name", help="Specific filename to download (e.g. LOG302.TXT). Default: latest LOGNNN.TXT")
    args = ap.parse_args()

    port = args.port or find_teensy_port()
    print(f"Opening {port} @ {args.baud}")
    ser = serial.Serial(port, args.baud, timeout=0.1)
    # Teensy USB CDC needs a moment after open before it processes the first
    # write — otherwise LIST,SD goes out before the loop is reading.
    time.sleep(0.3)

    print("Listing SD...")
    files = list_logs(ser)
    if not files:
        raise SystemExit("No files reported by the Teensy. Is the SD card inserted and mounted?")
    print(f"Found {len(files)} files:")
    for name, size in files:
        print(f"  {name:<16} {size:>10} bytes")

    if args.name:
        pick = next(((n, s) for n, s in files if n.upper() == args.name.upper()), None)
        if pick is None:
            raise SystemExit(f"{args.name} not found on the card.")
    else:
        pick = latest_log(files)
        if pick is None:
            raise SystemExit("No LOGNNN.TXT files on the card.")
    name, size = pick
    print(f"\nDownloading: {name} ({size} bytes)")

    out_dir = Path(args.out)
    out_path = out_dir / name
    download(ser, name, size, out_path)
    print(f"\nWrote {out_path.resolve()}")


if __name__ == "__main__":
    main()
