import numpy as np
import requests
import threading
import time
import sys
from scipy.signal import fftconvolve
from pathlib import Path

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

# ---- Parameters ----
TONE_FREQ = 440
CIPIC_SUBJECT = 3
SAMPLE_RATE = 44100
BLOCK_SIZE = 2048
SERIAL_BAUD = 115200
SERIAL_PORT = 'COM7'  # set to "COM3", "/dev/ttyUSB0", etc. or None for auto-detect


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
        raise RuntimeError(f"Download failed (HTTP {resp.status_code})")
    filename.write_bytes(resp.content)
    return str(filename)


def load_hrtf(sofa_path):
    from netCDF4 import Dataset
    ds = Dataset(sofa_path, 'r')
    positions = np.array(ds.variables['SourcePosition'][:])
    hrir = np.array(ds.variables['Data.IR'][:])
    sr = float(ds.variables['Data.SamplingRate'][:][0])
    ds.close()
    return positions, hrir, sr


def find_nearest_hrtf(positions, hrir, target_az, target_el=0):
    az = positions[:, 0]
    el = positions[:, 1]
    az_diff = np.minimum(np.abs(az - target_az), 360 - np.abs(az - target_az))
    el_diff = np.abs(el - target_el)
    dist = np.sqrt(az_diff**2 + el_diff**2)
    idx = np.argmin(dist)
    return hrir[idx, 0, :], hrir[idx, 1, :]


def precompute_hrtfs(positions, hrir, step=5):
    """Precompute HRTFs at every `step` degrees for fast lookup."""
    table = {}
    for az in range(0, 360, step):
        table[az] = find_nearest_hrtf(positions, hrir, az)
    return table


def get_hrtf_for_angle(hrtf_table, angle, step=5):
    """Snap angle to nearest precomputed HRTF."""
    snapped = round(angle / step) * step % 360
    return hrtf_table[snapped]


def find_arduino_port():
    """Auto-detect Arduino serial port."""
    ports = serial.tools.list_ports.comports()
    for p in ports:
        desc = (p.description + (p.manufacturer or "")).lower()
        if any(kw in desc for kw in ['arduino', 'ch340', 'cp210', 'ftdi', 'usb serial']):
            return p.device
    if ports:
        return ports[0].device
    return None


class HeadTracker:
    """Reads yaw angle from Arduino over serial in a background thread."""
    def __init__(self, port, baud):
        self.angle = 0.0
        self.running = True
        self.port = port
        self.baud = baud
        self.connected = False
        self.thread = threading.Thread(target=self._read_loop, daemon=True)
        self.thread.start()

    def _read_loop(self):
        try:
            ser = serial.Serial(self.port, self.baud, timeout=0.1)
            self.connected = True
            print(f"Connected to {self.port}")
            time.sleep(2)  # wait for Arduino reset
            ser.flushInput()

            while self.running:
                line = ser.readline().decode('ascii', errors='ignore').strip()
                if line:
                    try:
                        self.angle = float(line)
                        print(f"\rDEBUG: {self.angle}", end="")
                    except ValueError:
                        print(f"BAD LINE: {line}")
        except serial.SerialException as e:
            print(f"Serial error: {e}")
            self.connected = False

    def get_angle(self):
        return self.angle

    def stop(self):
        self.running = False


class BinauralRenderer:
    """Real-time binaural audio renderer using overlap-add convolution."""
    def __init__(self, hrtf_table, sr, block_size, tone_freq):
        self.hrtf_table = hrtf_table
        self.sr = sr
        self.block_size = block_size
        self.tone_freq = tone_freq
        self.phase = 0.0
        self.current_angle = 0.0

        # overlap-add tail from previous block
        sample_hrir = list(hrtf_table.values())[0][0]
        self.hrir_len = len(sample_hrir)
        self.overlap_l = np.zeros(self.hrir_len - 1)
        self.overlap_r = np.zeros(self.hrir_len - 1)

        # crossfade for smooth HRTF transitions
        self.prev_hrir_l = None
        self.prev_hrir_r = None

    def set_angle(self, angle):
        self.current_angle = angle

    def generate_block(self):
        # generate tone
        n = self.block_size
        t = (np.arange(n) + self.phase) / self.sr
        signal = np.sin(2 * np.pi * self.tone_freq * t) * 0.7
        self.phase += n

        # get HRTF for current angle
        hrir_l, hrir_r = get_hrtf_for_angle(self.hrtf_table, self.current_angle)

        # convolve
        conv_l = fftconvolve(signal, hrir_l, mode='full')
        conv_r = fftconvolve(signal, hrir_r, mode='full')

        # overlap-add
        out_l = conv_l[:n] + self.overlap_l[:n] if len(self.overlap_l) >= n else conv_l[:n]
        out_r = conv_r[:n] + self.overlap_r[:n] if len(self.overlap_r) >= n else conv_r[:n]

        # handle overlap sizes
        if len(conv_l) > n:
            self.overlap_l = conv_l[n:]
        else:
            self.overlap_l = np.zeros(self.hrir_len - 1)

        if len(conv_r) > n:
            self.overlap_r = conv_r[n:]
        else:
            self.overlap_r = np.zeros(self.hrir_len - 1)

        # normalize
        peak = max(np.max(np.abs(out_l)), np.max(np.abs(out_r)), 1e-10)
        if peak > 1.0:
            out_l /= peak
            out_r /= peak

        stereo = np.column_stack([out_l, out_r]).astype(np.float32)
        return stereo


def main():
    print("=" * 50)
    print("  Real-Time Binaural Head Tracker")
    print("=" * 50)
    print()
    

    # load HRTFs
    print("Loading HRTF data...")
    sofa_path = download_sofa(CIPIC_SUBJECT)
    positions, hrir, sofa_sr = load_hrtf(sofa_path)
    sr = int(sofa_sr)
    print("Precomputing HRTF lookup table...")
    hrtf_table = precompute_hrtfs(positions, hrir, step=5)

    # connect to Arduino
    port = SERIAL_PORT
    if port is None:
        print("Searching for Arduino...")
        port = find_arduino_port()
        if port is None:
            print("No Arduino found. Plug it in and try again.")
            print("Or set SERIAL_PORT manually in the script.")
            return
    print(f"Using serial port: {port}")

    tracker = HeadTracker(port, SERIAL_BAUD)
    time.sleep(3)  # let Arduino calibrate

    if not tracker.connected:
        print("Could not connect to Arduino. Check the port and wiring.")
        tracker.stop()
        return

    # set up audio
    renderer = BinauralRenderer(hrtf_table, sr, BLOCK_SIZE, TONE_FREQ)
    pa = pyaudio.PyAudio()

    def audio_callback(in_data, frame_count, time_info, status):
        renderer.set_angle(tracker.get_angle())
        block = renderer.generate_block()
        return (block.tobytes(), pyaudio.paContinue)

    stream = pa.open(
        format=pyaudio.paFloat32,
        channels=2,
        rate=sr,
        output=True,
        frames_per_buffer=BLOCK_SIZE,
        stream_callback=audio_callback
    )

    stream.start_stream()
    print()
    print("Playing! Put on headphones and rotate the IMU.")
    print("The tone stays fixed in space — turn left and it moves right, etc.")
    print("Press Ctrl+C to stop.")
    print()

    try:
        while stream.is_active():
            angle = tracker.get_angle()
            bar_pos = int(angle / 360 * 40)
            bar = "." * 40
            bar = bar[:bar_pos] + "|" + bar[bar_pos + 1:]
            print(f"\r  Yaw: {angle:6.1f}°  [{bar}]", end="", flush=True)
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\n\nStopping...")

    stream.stop_stream()
    stream.close()
    pa.terminate()
    tracker.stop()
    print("Done.")


if __name__ == "__main__":
    main()
