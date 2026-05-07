import argparse
import csv
import queue
import sys
import threading
import time
from pathlib import Path

import numpy as np
import requests
import soundfile as sf
from scipy.signal import fftconvolve, resample_poly

try:
    import pyaudio
except ImportError:
    print("Install pyaudio: pip install pyaudio")
    sys.exit(1)

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("Install pyserial: pip install pyserial")
    sys.exit(1)


# CIPIC convention used here: 0 = front, 90 = left, 180 = back, 270 = right.
SERIAL_BAUD = 115200
SERIAL_PORT = None
CIPIC_SUBJECT = 3
BLOCK_SIZE = 1024
HRTF_STEP_DEGREES = 5
MAX_DURATION_SECONDS = 5 * 60
OUTPUT_GAIN = 0.85

# If turning left makes the sound move right, change this to -1.
YAW_SIGN = 1

# Keep 0 for "sound direction equals head yaw". Set to 180 for a rear source, etc.
SOURCE_OFFSET_DEGREES = 0


def parse_yaw_line(line):
    """Return yaw degrees from either yaw-only lines or Arduino verbose CSV."""
    try:
        return float(line) % 360
    except ValueError:
        pass

    parts = [part.strip() for part in line.split(",")]
    if len(parts) < 13:
        return None

    try:
        calibrated = int(parts[1])
        yaw_relative = float(parts[12])
    except ValueError:
        return None

    if calibrated != 1:
        return 0.0
    return yaw_relative % 360


def download_sofa(subject_id):
    sofa_dir = Path("./hrtf_cache")
    sofa_dir.mkdir(exist_ok=True)
    filename = sofa_dir / f"subject_{subject_id:03d}.sofa"
    if filename.exists():
        return str(filename)

    url = f"https://sofacoustics.org/data/database/cipic/subject_{subject_id:03d}.sofa"
    print(f"Downloading CIPIC subject {subject_id}...")
    resp = requests.get(url, timeout=60)
    if resp.status_code != 200:
        raise RuntimeError(f"Download failed (HTTP {resp.status_code}). Try a different CIPIC_SUBJECT.")
    filename.write_bytes(resp.content)
    return str(filename)


def load_hrtf(sofa_path):
    from netCDF4 import Dataset

    ds = Dataset(sofa_path, "r")
    positions = np.array(ds.variables["SourcePosition"][:])
    hrir = np.array(ds.variables["Data.IR"][:])
    sr = int(float(ds.variables["Data.SamplingRate"][:][0]))
    ds.close()
    return positions, hrir, sr


def find_nearest_hrtf(positions, hrir, target_az, target_el=0):
    az = positions[:, 0]
    el = positions[:, 1]
    az_diff = np.minimum(np.abs(az - target_az), 360 - np.abs(az - target_az))
    el_diff = np.abs(el - target_el)
    idx = np.argmin(np.sqrt(az_diff**2 + el_diff**2))
    return hrir[idx, 0, :].astype(np.float32), hrir[idx, 1, :].astype(np.float32)


def precompute_hrtfs(positions, hrir, step):
    table = {}
    for az in range(0, 360, step):
        table[az] = find_nearest_hrtf(positions, hrir, az)
    return table


def get_hrtf_for_angle(hrtf_table, angle, step):
    snapped = round(angle / step) * step % 360
    return hrtf_table[snapped]


def find_arduino_port():
    ports = serial.tools.list_ports.comports()
    for port in ports:
        desc = f"{port.description} {port.manufacturer or ''}".lower()
        if any(name in desc for name in ("arduino", "ch340", "cp210", "ftdi", "usb serial")):
            return port.device
    return ports[0].device if ports else None


class HeadTracker:
    def __init__(self, port, baud):
        self.angle = 0.0
        self.connected = False
        self.running = True
        self.samples = queue.Queue()
        self.thread = threading.Thread(target=self._read_loop, args=(port, baud), daemon=True)
        self.thread.start()

    def _read_loop(self, port, baud):
        try:
            with serial.Serial(port, baud, timeout=0.1) as ser:
                self.connected = True
                print(f"Connected to {port}")
                time.sleep(2)
                ser.reset_input_buffer()

                while self.running:
                    line = ser.readline().decode("ascii", errors="ignore").strip()
                    if not line:
                        continue
                    yaw = parse_yaw_line(line)
                    if yaw is None:
                        continue
                    self.angle = yaw
                    self.samples.put((time.time(), self.angle))
        except serial.SerialException as exc:
            print(f"Serial error: {exc}")
            self.connected = False

    def get_angle(self):
        return self.angle

    def drain_samples(self):
        drained = []
        while True:
            try:
                drained.append(self.samples.get_nowait())
            except queue.Empty:
                return drained

    def stop(self):
        self.running = False
        self.thread.join(timeout=1.0)


class ClipBinauralRenderer:
    def __init__(self, audio_path, target_sr, hrtf_table, block_size, hrtf_step, max_seconds):
        data, source_sr = sf.read(audio_path, always_2d=True, dtype="float32")
        data = np.mean(data, axis=1)

        if max_seconds is not None:
            data = data[: int(max_seconds * source_sr)]

        if source_sr != target_sr:
            gcd = np.gcd(source_sr, target_sr)
            data = resample_poly(data, target_sr // gcd, source_sr // gcd).astype(np.float32)

        self.audio = data
        self.sr = target_sr
        self.hrtf_table = hrtf_table
        self.block_size = block_size
        self.hrtf_step = hrtf_step
        self.cursor = 0

        sample_hrir = list(hrtf_table.values())[0][0]
        self.overlap_l = np.zeros(len(sample_hrir) - 1, dtype=np.float32)
        self.overlap_r = np.zeros(len(sample_hrir) - 1, dtype=np.float32)

    def done(self):
        return self.cursor >= len(self.audio)

    def next_block(self, angle):
        block = self.audio[self.cursor : self.cursor + self.block_size]
        self.cursor += len(block)

        if len(block) < self.block_size:
            block = np.pad(block, (0, self.block_size - len(block)))

        hrir_l, hrir_r = get_hrtf_for_angle(self.hrtf_table, angle, self.hrtf_step)
        conv_l = fftconvolve(block, hrir_l, mode="full").astype(np.float32)
        conv_r = fftconvolve(block, hrir_r, mode="full").astype(np.float32)

        out_l = conv_l[: self.block_size] + self.overlap_l[: self.block_size]
        out_r = conv_r[: self.block_size] + self.overlap_r[: self.block_size]
        self.overlap_l = conv_l[self.block_size :]
        self.overlap_r = conv_r[self.block_size :]

        peak = max(float(np.max(np.abs(out_l))), float(np.max(np.abs(out_r))), 1e-8)
        if peak > 1.0:
            out_l /= peak
            out_r /= peak

        return (np.column_stack((out_l, out_r)) * OUTPUT_GAIN).astype(np.float32)


def relative_source_angle(yaw_degrees):
    return (SOURCE_OFFSET_DEGREES + YAW_SIGN * yaw_degrees) % 360


def write_log_header(path):
    with path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["time_seconds", "imu_yaw_degrees", "hrtf_azimuth_degrees"])


def append_log_rows(path, start_time, samples):
    if not samples:
        return
    with path.open("a", newline="") as f:
        writer = csv.writer(f)
        for sample_time, yaw in samples:
            writer.writerow([f"{sample_time - start_time:.3f}", f"{yaw:.2f}", f"{relative_source_angle(yaw):.2f}"])


def parse_args():
    parser = argparse.ArgumentParser(
        description="Play an audio clip through HRTFs controlled by Arduino/ICM-20948 head yaw."
    )
    parser.add_argument("audio_file", help="WAV/FLAC/OGG clip to spatialize. Keep it under 5 minutes.")
    parser.add_argument("--port", default=SERIAL_PORT, help="Arduino serial port, e.g. COM3 or /dev/tty.usbserial-110.")
    parser.add_argument("--subject", type=int, default=CIPIC_SUBJECT, help="CIPIC HRTF subject ID.")
    parser.add_argument("--log", default="head_orientation_log.csv", help="CSV file for recorded head orientation.")
    parser.add_argument("--max-seconds", type=float, default=MAX_DURATION_SECONDS, help="Maximum clip length to play.")
    return parser.parse_args()


def main():
    args = parse_args()
    audio_path = Path(args.audio_file)
    if not audio_path.exists():
        raise FileNotFoundError(f"Audio file not found: {audio_path}")

    port = args.port or find_arduino_port()
    if port is None:
        print("No Arduino found. Plug it in or pass --port.")
        return

    print("Loading HRTF data...")
    sofa_path = download_sofa(args.subject)
    positions, hrir, hrtf_sr = load_hrtf(sofa_path)
    hrtf_table = precompute_hrtfs(positions, hrir, HRTF_STEP_DEGREES)

    print("Loading audio clip...")
    renderer = ClipBinauralRenderer(
        audio_path=audio_path,
        target_sr=hrtf_sr,
        hrtf_table=hrtf_table,
        block_size=BLOCK_SIZE,
        hrtf_step=HRTF_STEP_DEGREES,
        max_seconds=args.max_seconds,
    )

    tracker = HeadTracker(port, SERIAL_BAUD)
    time.sleep(3)
    if not tracker.connected:
        print("Could not connect to Arduino. Check the port, wiring, and baud rate.")
        tracker.stop()
        return

    log_path = Path(args.log)
    write_log_header(log_path)
    start_time = time.time()

    pa = pyaudio.PyAudio()

    def audio_callback(in_data, frame_count, time_info, status):
        yaw = tracker.get_angle()
        hrtf_angle = relative_source_angle(yaw)
        block = renderer.next_block(hrtf_angle)
        flag = pyaudio.paComplete if renderer.done() else pyaudio.paContinue
        return block.tobytes(), flag

    stream = pa.open(
        format=pyaudio.paFloat32,
        channels=2,
        rate=hrtf_sr,
        output=True,
        frames_per_buffer=BLOCK_SIZE,
        stream_callback=audio_callback,
    )

    print("Playing. Wear headphones and rotate the IMU.")
    print("For the requested mapping: yaw 45 deg -> sound at CIPIC azimuth 45 deg.")
    print(f"Logging head orientation to {log_path}")
    print("Press Ctrl+C to stop.")

    stream.start_stream()
    try:
        while stream.is_active():
            append_log_rows(log_path, start_time, tracker.drain_samples())
            yaw = tracker.get_angle()
            print(f"\rYaw: {yaw:6.1f} deg | HRTF azimuth: {relative_source_angle(yaw):6.1f} deg", end="", flush=True)
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\nStopping...")

    append_log_rows(log_path, start_time, tracker.drain_samples())
    stream.stop_stream()
    stream.close()
    pa.terminate()
    tracker.stop()
    print(f"\nDone. Orientation log saved to {log_path}")


if __name__ == "__main__":
    main()
