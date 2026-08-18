#!/usr/bin/env python3
"""Capture ESP32-S3 feasibility harness CSV output over USB serial.

Usage: python3 host/capture.py <outdir> [--mode B|C|H] [--port /dev/ttyACM0]
Streams everything to <outdir>/log_raw.txt and CSV lines to <outdir>/<mode>.csv.
"""
import argparse, os, re, sys, time
import serial

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("outdir")
    ap.add_argument("--mode", default="B", choices=["B", "C", "H"])
    ap.add_argument("--avg", type=int, default=1, help="mode C: ADC reads averaged per step")
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=460800)
    ap.add_argument("--timeout", type=float, default=120)
    args = ap.parse_args()

    os.makedirs(args.outdir, exist_ok=True)
    raw_path = os.path.join(args.outdir, "log_raw.txt")
    csv_path = os.path.join(args.outdir, f"{args.mode}.csv")

    ser = serial.Serial(args.port, args.baud, timeout=0.5)
    time.sleep(1.5)              # let the board settle after reset
    ser.reset_input_buffer()
    time.sleep(0.2)
    ser.write(args.mode.encode() if args.mode != "C" else f"C{args.avg}".encode())  # trigger the mode
    ser.flush()

    raw = open(raw_path, "w")
    csvf = open(csv_path, "w")
    print(f"capturing mode {args.mode} -> {csv_path}")
    t0 = time.time()
    n_csv = 0
    done = False
    while time.time() - t0 < args.timeout and not done:
        try:
            line = ser.readline().decode("utf-8", "replace").rstrip("\r\n")
        except serial.SerialException:
            break
        if not line:
            continue
        raw.write(line + "\n")
        if line.startswith("CSV,"):
            csvf.write(line + "\n"); n_csv += 1
        if line.startswith("CSV,BDONE") or line.startswith("CSV,CDONE") or line.startswith("CSV,HDONE"):
            done = True
            print(f"  {line} after {time.time()-t0:.1f}s")
            break
        sys.stdout.write(line + "\n")   # live view
        sys.stdout.flush()
    raw.close(); csvf.close()
    if not done:
        print(f"  timed out after {args.timeout}s ({n_csv} csv lines)")
    print(f"done: {n_csv} csv lines, {os.path.getsize(raw_path)} raw bytes")

if __name__ == "__main__":
    main()