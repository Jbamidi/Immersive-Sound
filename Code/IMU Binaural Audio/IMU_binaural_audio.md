# Real-Time Binaural Head Tracker

Uses an MPU6050 IMU on an Arduino to track head rotation and render spatialized audio in real time through headphones. When you turn your head, the sound stays fixed in space.

## How It Works

1. The Arduino reads the MPU6050's DMP (Digital Motion Processor) to get a stable yaw angle
2. The first reading is saved as 0° — all angles are relative to where you started
3. The yaw angle is sent over USB serial to your PC
4. The Python script reads the angle, picks the matching HRTF from the CIPIC dataset, and convolves a tone with it in real time using overlap-add
5. The result plays through your headphones — the tone appears to stay in one place as you rotate your head

## Wiring

```
MPU6050  ->  Arduino
VCC      ->  5V
GND      ->  GND
SDA      ->  A4
SCL      ->  A5
INT      ->  D2
```

## Arduino Setup

1. Open `head_tracker_mpu6050.ino` in the Arduino IDE
2. Install the required libraries via Sketch > Include Library > Manage Libraries:
   - **MPU6050** by Electronic Cats
   - **I2Cdev** by Jeff Rowberg
3. Upload to your Arduino
4. Open Serial Monitor at 115200 baud to verify you see angle numbers

## Python Setup

```
pip install numpy scipy pyaudio pyserial requests pysofaconventions netCDF4
```

On Windows if `pyaudio` fails: `pip install pipwin` then `pipwin install pyaudio`

## Run

```
python realtime_binaural.py
```

The script auto-detects your Arduino. If it doesn't find it, set `SERIAL_PORT` at the top of the script (e.g., `"COM3"` on Windows).

## Parameters

| Parameter | Default | What it does |
|-----------|---------|--------------|
| `TONE_FREQ` | `440` | Frequency of the tone in Hz |
| `CIPIC_SUBJECT` | `3` | HRTF ear model (try 8, 10, 15, 18, 21) |
| `SERIAL_PORT` | `None` | Set manually if auto-detect fails |
| `BLOCK_SIZE` | `2048` | Audio buffer size (lower = less latency, more CPU) |

## Important

Use headphones. Binaural audio does not work on speakers.
