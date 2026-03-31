import numpy as np
import requests
import soundfile as sf
import matplotlib.pyplot as plt
from scipy.signal import fftconvolve
from pathlib import Path

# ---- Parameters (change these) ----
TONE_FREQ = 440                  # Hz
DURATION = 2.0                   # seconds per angle
AZIMUTH_ANGLES = [0, 45, 90, 135, 180, 225, 270, 315]
ELEVATION = 0
SWEEP_ENABLED = True
SWEEP_DURATION = 8.0
CIPIC_SUBJECT = 3                # try 8, 10, 15, 18, 21 for different ear shapes
SAMPLE_RATE = 44100
AMPLITUDE = 0.8

OUTPUT_DIR = Path("./output")
OUTPUT_DIR.mkdir(exist_ok=True)


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
    ds = Dataset(sofa_path, 'r')
    positions = np.array(ds.variables['SourcePosition'][:])
    hrir = np.array(ds.variables['Data.IR'][:])
    sr = float(ds.variables['Data.SamplingRate'][:][0])
    ds.close()
    return positions, hrir, sr


def find_nearest_hrtf(positions, hrir, target_az, target_el):
    az = positions[:, 0]
    el = positions[:, 1]
    az_diff = np.minimum(np.abs(az - target_az), 360 - np.abs(az - target_az))
    el_diff = np.abs(el - target_el)
    dist = np.sqrt(az_diff**2 + el_diff**2)
    idx = np.argmin(dist)
    return hrir[idx, 0, :], hrir[idx, 1, :], positions[idx, 0], positions[idx, 1]


def generate_signal(duration, sr):
    n = int(duration * sr)
    t = np.arange(n) / sr
    fade = int(0.02 * sr)
    sig = np.sin(2 * np.pi * TONE_FREQ * t)
    sig[:fade] *= np.linspace(0, 1, fade)
    sig[-fade:] *= np.linspace(1, 0, fade)
    sig = sig / (np.max(np.abs(sig)) + 1e-10)
    return sig


def render_binaural(signal, hrir_l, hrir_r, amplitude=0.8):
    left = fftconvolve(signal, hrir_l, mode='full')
    right = fftconvolve(signal, hrir_r, mode='full')
    length = min(len(left), len(right))
    left, right = left[:length], right[:length]
    peak = max(np.max(np.abs(left)), np.max(np.abs(right)), 1e-10)
    return np.column_stack([(left / peak) * amplitude, (right / peak) * amplitude])


def render_sweep(positions, hrir, sr, dur, start_az, end_az, elev, amp):
    n = int(dur * sr)
    signal = generate_signal(dur, sr)
    chunk = int(0.01 * sr)
    out_l = np.zeros(n + 512)
    out_r = np.zeros(n + 512)
    azimuths = np.linspace(start_az, end_az, n // chunk)

    for i, az in enumerate(azimuths):
        s = i * chunk
        e = min(s + chunk, n)
        if s >= n:
            break
        hl, hr, _, _ = find_nearest_hrtf(positions, hrir, az % 360, elev)
        cl = fftconvolve(signal[s:e], hl, mode='full')
        cr = fftconvolve(signal[s:e], hr, mode='full')
        out_l[s:s+len(cl)] += cl
        out_r[s:s+len(cr)] += cr

    out_l, out_r = out_l[:n], out_r[:n]
    peak = max(np.max(np.abs(out_l)), np.max(np.abs(out_r)), 1e-10)
    return np.column_stack([(out_l / peak) * amp, (out_r / peak) * amp])


def plot_direction_map(angles, actual_angles):
    fig, ax = plt.subplots(figsize=(8, 8), subplot_kw={'projection': 'polar'}, constrained_layout=True)
    req = np.deg2rad(angles)
    act = np.deg2rad(actual_angles)

    ax.scatter(req, np.ones(len(req)), s=120, c='#2196F3', zorder=5,
              label='Requested', edgecolors='white', linewidths=1.5)
    ax.scatter(act, np.ones(len(act)) * 0.85, s=80, c='#FF9800', zorder=5,
              marker='D', label='Nearest HRTF', edgecolors='white', linewidths=1)
    for r, a in zip(req, act):
        ax.annotate('', xy=(a, 0.85), xytext=(r, 1.0),
                    arrowprops=dict(arrowstyle='->', color='gray', alpha=0.5, lw=1))

    ax.set_theta_zero_location('N')
    ax.set_theta_direction(-1)
    ax.set_ylim(0, 1.3)
    ax.set_rticks([])
    ax.set_title('Source Directions (Requested vs Nearest HRTF)', fontsize=13, fontweight='bold', pad=20)
    ax.legend(loc='lower right', fontsize=10)
    for r in req:
        ax.annotate(f'{np.rad2deg(r):.0f}°', xy=(r, 1.15), ha='center', fontsize=9, color='#2196F3')

    fig.savefig(OUTPUT_DIR / "direction_map.png", dpi=150, bbox_inches='tight')
    plt.close(fig)


def main():
    print("Loading HRTF data...")
    sofa_path = download_sofa(CIPIC_SUBJECT)
    positions, hrir, sofa_sr = load_hrtf(sofa_path)
    sr = int(sofa_sr)

    print("Generating tone signal...")
    signal = generate_signal(DURATION, sr)

    print("Rendering binaural audio...")
    actual_azimuths = []
    wav_files = []

    for az in AZIMUTH_ANGLES:
        hrir_l, hrir_r, actual_az, _ = find_nearest_hrtf(positions, hrir, az, ELEVATION)
        actual_azimuths.append(actual_az)
        stereo = render_binaural(signal, hrir_l, hrir_r, AMPLITUDE)
        fname = OUTPUT_DIR / f"binaural_sound_{az}deg.wav"
        sf.write(str(fname), stereo, sr)
        wav_files.append(fname)
        print(f"  {az}° -> {fname.name}")

    silence = np.zeros((int(0.3 * sr), 2))
    combined = []
    for fname in wav_files:
        data, _ = sf.read(str(fname))
        combined.append(data)
        combined.append(silence)
    sf.write(str(OUTPUT_DIR / "binaural_all_angles.wav"), np.vstack(combined), sr)

    if SWEEP_ENABLED:
        print("Rendering sweep...")
        sweep = render_sweep(positions, hrir, sr, SWEEP_DURATION, 0, 360, ELEVATION, AMPLITUDE)
        sf.write(str(OUTPUT_DIR / "binaural_sweep.wav"), sweep, sr)

    print("Saving direction map...")
    plot_direction_map(AZIMUTH_ANGLES, actual_azimuths)

    print(f"\nDone! Files in {OUTPUT_DIR}/")


if __name__ == "__main__":
    main()
