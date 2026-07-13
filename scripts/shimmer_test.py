#!/usr/bin/env python3
"""
Standalone pyshimmer connection test.

Connects to a Shimmer3 over the rfcomm serial device, starts streaming,
and prints a one-line packet-count/rate metric every 5 seconds.

Usage:
    python3 shimmer_test.py [/dev/rfcomm0]
"""

import sys
import time
import threading

from serial import Serial
from pyshimmer import ShimmerBluetooth, DEFAULT_BAUDRATE, DataPacket

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/rfcomm0"

packet_count = 0
count_lock = threading.Lock()


def handler(pkt: DataPacket) -> None:
    global packet_count
    with count_lock:
        packet_count += 1


def main():
    global packet_count
    print(f"Connecting to Shimmer on {PORT}...")
    with Serial(PORT, DEFAULT_BAUDRATE) as ser, ShimmerBluetooth(ser) as shim:
        name = shim.get_device_name()
        print(f"Connected: {name}")

        shim.add_stream_callback(handler)
        shim.start_streaming()

        try:
            while True:
                time.sleep(5.0)
                with count_lock:
                    n = packet_count
                    packet_count = 0
                rate = n / 5.0
                print(f"[{time.strftime('%H:%M:%S')}] packets last 5s: {n} (~{rate:.1f} Hz)")
        except KeyboardInterrupt:
            print("\nStopping...")
        finally:
            shim.stop_streaming()


if __name__ == "__main__":
    main()