"""Read-only CH340 capture; never transmits debug commands."""
from __future__ import annotations

import json
from pathlib import Path
import threading
import time


class DebugUartCapture:
    def __init__(self, port_name: str, folder: Path):
        self.port_name = port_name
        self.folder = folder
        self.port = None
        self.thread = None
        self.stop_event = threading.Event()
        self.error = None
        self.bytes_received = 0
        self.last_data = time.monotonic()

    def start(self):
        import serial
        # Set modem outputs before open. No deliberate reset/download pulse.
        self.port = serial.Serial(port=None, baudrate=115200, timeout=0.02,
                                  write_timeout=0.1, rtscts=False, dsrdtr=False)
        self.port.dtr = False
        self.port.rts = False
        self.port.port = self.port_name
        self.port.open()
        self.thread = threading.Thread(target=self._read, name="CH340-read-only", daemon=True)
        self.thread.start()
        deadline = time.monotonic()+3.0
        while self.bytes_received == 0 and self.error is None and time.monotonic() < deadline:
            self.stop_event.wait(0.01)
        self.check()
        if self.bytes_received == 0:
            raise RuntimeError("CH340 opened but no debug bytes received; motion not permitted")

    def _read(self):
        try:
            with (self.folder/"ch340_raw.bin").open("wb") as raw, (self.folder/"ch340_chunks.jsonl").open("w", encoding="utf-8") as chunks:
                while not self.stop_event.is_set():
                    # Do not use in_waiting/purge/write on this read-only port.
                    data = self.port.read(512)
                    if data:
                        now = time.monotonic()
                        raw.write(data)
                        chunks.write(json.dumps(dict(host_monotonic_s=now, data_hex=data.hex()))+"\n")
                        raw.flush()
                        chunks.flush()
                        self.bytes_received += len(data)
                        self.last_data = now
        except Exception as error:
            self.error = str(error)

    def check(self):
        if self.error is not None:
            raise RuntimeError("CH340 capture failed: "+self.error)
        if self.bytes_received and time.monotonic()-self.last_data > 2.0:
            raise RuntimeError("CH340 debug stream stale (>2 seconds)")
        if self.thread is not None and not self.thread.is_alive():
            raise RuntimeError("CH340 reader stopped unexpectedly")

    def close(self):
        # Keep the reader running briefly to catch CDC close/session-end logs.
        self.stop_event.wait(0.25)
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join(timeout=2.0)
            if self.thread.is_alive():
                self.error = self.error or "CH340 reader did not terminate"
        if self.port is not None:
            try:
                self.port.close()
            except Exception as error:
                self.error = self.error or str(error)
        raw_file = self.folder/"ch340_raw.bin"
        if raw_file.exists():
            # Preserve firmware CRLF; avoid Windows translating CRLF to CRCRLF.
            with (self.folder/"ch340.log").open("w", encoding="utf-8", newline="") as text:
                text.write(raw_file.read_bytes().decode("utf-8", errors="replace"))
