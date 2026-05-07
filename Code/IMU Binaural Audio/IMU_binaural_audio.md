# Real-Time Binaural Head Tracker

Uses an Adafruit ICM-20948 IMU breakout on an Arduino Uno R3 to track head rotation and render spatialized audio in real time through headphones.

## How It Works

1. The Arduino reads the ICM-20948 accelerometer and magnetometer over I2C
2. The first reading is saved as 0° — all angles are relative to where you started
3. The yaw angle is sent over USB serial to your PC
4. The Python script reads the angle, picks the matching HRTF from the CIPIC dataset, and convolves a tone with it in real time using overlap-add
5. The result plays through your headphones with the source direction controlled by your head turn

## Wiring

```
ICM20948       ->  Arduino Uno R3
VIN            ->  5V
GND            ->  GND
SDA            ->  A4
SCL            ->  A5
INT            ->  not connected
AD0 / ADDR     ->  leave unconnected for 0x69, or tie to GND for 0x68
CS             ->  not connected for I2C
```

The Adafruit breakout includes a 1.8V regulator and level shifting, so it can be wired directly to a 5V Arduino Uno.

## Arduino Setup

1. Open `head-tracker/head-tracker.ino` in the Arduino IDE
2. Install the required libraries via Sketch > Include Library > Manage Libraries:
   - **Adafruit ICM20X**
   - **Adafruit BusIO**
   - **Adafruit Unified Sensor**
3. Upload to your Arduino
4. Open Serial Monitor at 115200 baud
5. For the first 3 seconds, slowly rotate/tilt the IMU in a figure-eight so the magnetometer can calibrate
6. The Serial Monitor prints continuous test CSV: `time_ms,calibrated,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,mag_x,mag_y,mag_z,yaw_abs,yaw_relative`
7. The Python audio player reads `yaw_relative` from the final CSV column

```
#define ICM_I2C_ADDR 0x69
```

If the Serial Monitor says it cannot find the sensor and you tied `AD0` / `ADDR` to `GND`, change it to:

```
#define ICM_I2C_ADDR 0x68
```

## Python Setup

```
pip install numpy scipy pyaudio pyserial requests pysofaconventions netCDF4
```

On Windows if `pyaudio` fails: `pip install pipwin` then `pipwin install pyaudio`

## Run

For a built-in 440 Hz test tone controlled by the 9DoF IMU:

```
python realtime_binaural.py
```

The script auto-detects your Arduino and reads `yaw_relative` from the Adafruit ICM-20948 CSV output. If it doesn't find the Arduino, set `SERIAL_PORT` at the top of the script, for example `"COM3"` on Windows or `"/dev/cu.usbmodem1101"` on macOS.

## Play a Clip and Record Head Orientation

Use `head_orientation_audio_player.py` when you want to play a short audio clip from a music video/movie/visual and move the perceived sound direction with your head.

Close Arduino Serial Monitor before running Python. The headphones plug into your computer, not the Arduino.

```
python head_orientation_audio_player.py path/to/clip.wav --port COM3 --log head_orientation_log.csv
```

On macOS/Linux, the port will look more like:

```
python head_orientation_audio_player.py path/to/clip.wav --port /dev/tty.usbserial-110
```

The clip should be less than 5 minutes. The script:

1. Reads yaw from the Arduino/ICM20948 over serial
2. Plays the audio clip through headphones
3. Maps yaw to HRTF azimuth, so a 45° head turn maps to a 45° sound direction
4. Records `time_seconds`, `imu_yaw_degrees`, and `hrtf_azimuth_degrees` to CSV

The player accepts either Arduino output format:

```
45.0
```

or verbose test CSV:

```
time_ms,calibrated,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,mag_x,mag_y,mag_z,yaw_abs,yaw_relative
```

Supported input depends on `soundfile`, but WAV/FLAC/OGG are the safest. If you have an MP4/MOV, extract a short WAV first:

```
ffmpeg -i input_video.mp4 -t 300 -vn -ac 2 clip.wav
```

If turning left makes the audio move right, open `head_orientation_audio_player.py` and set:

```
YAW_SIGN = -1
```

## Parameters

| Parameter | Default | What it does |
|-----------|---------|--------------|
| `TONE_FREQ` | `440` | Frequency of the tone in Hz |
| `CIPIC_SUBJECT` | `3` | HRTF ear model (try 8, 10, 15, 18, 21) |
| `SERIAL_PORT` | `None` | Set manually if auto-detect fails |
| `BLOCK_SIZE` | `1024` | Audio buffer size (lower = less latency, more CPU) |
| `YAW_SIGN` | `1` | Flip to `-1` if the perceived direction is reversed |
| `SOURCE_OFFSET_DEGREES` | `0` | Adds an offset to the source direction |

## Important

Use headphones. Binaural audio does not work on speakers.
