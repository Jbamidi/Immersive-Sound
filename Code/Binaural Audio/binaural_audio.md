# Binaural Audio Experience

Generates spatialized audio using CIPIC HRTFs so you can hear sound coming from different directions through headphones.

## How It Works

The script downloads a Head-Related Transfer Function (HRTF) dataset from the CIPIC database, which captures how sound is filtered differently by each ear depending on where it comes from. It then convolves a source signal (a C major chord by default) with the left and right ear impulse responses for each specified azimuth angle. The result is stereo WAV files that trick your brain into perceiving the sound at a specific location in 3D space.

The sweep file does this continuously, interpolating between HRTFs in 10ms chunks to simulate a sound source rotating around your head.

## Setup

```
pip install numpy scipy matplotlib soundfile pysofaconventions netCDF4 requests
```

## Run

```
python binaural_experience.py
```

## Output

| File | Description |
|------|-------------|
| `binaural_sound_{angle}deg.wav` | Sound at a specific azimuth (0° = front, 90° = left, 180° = behind, 270° = right) |
| `binaural_all_angles.wav` | All angles played sequentially |
| `binaural_sweep.wav` | Sound rotating 360° around your head |
| `direction_map.png` | Polar plot showing requested vs actual HRTF directions |

## Parameters

Edit the top of `binaural_experience.py`:

| Parameter | Default | What it does |
|-----------|---------|--------------|
| `AZIMUTH_ANGLES` | `[0, 45, 90, ...]` | Which directions to render (degrees) |
| `ELEVATION` | `0` | Vertical angle (0 = ear level) |
| `CIPIC_SUBJECT` | `3` | HRTF subject ID — different ears sound different (try 8, 10, 15, 18, 21) |
| `SWEEP_DURATION` | `8.0` | How long the rotation sweep takes (seconds) |
| `DURATION` | `2.0` | Length of each individual angle file (seconds) |

## Important

Use headphones. Binaural audio does not work on speakers.
