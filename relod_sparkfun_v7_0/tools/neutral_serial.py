import sys
import time

import serial


port = sys.argv[1] if len(sys.argv) > 1 else "COM7"
reader = serial.Serial()
reader.port = port
reader.baudrate = 115200
reader.timeout = 0.2
reader.dtr = False
reader.rts = False
reader.open()

print(f"Neutral serial reader ready on {port}", flush=True)
deadline = time.monotonic() + 120
while time.monotonic() < deadline:
    data = reader.read(4096)
    if data:
        sys.stdout.buffer.write(data)
        sys.stdout.buffer.flush()
