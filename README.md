# ESP32 MPU6050 FFT Analysis

Real-time FFT analysis on ESP32 with MPU6050 sensor for vibration detection and frequency analysis. Includes IoT dashboard monitoring and adaptive gimbal stabilizer concept.

## 📦 What's Included

| File | Description |
|------|-------------|
| **fft_monitor.ino** | Main FFT implementation with Thinger.io dashboard |
| **gimbal_stabilizer.ino** | Adaptive gimbal concept with frequency-based PID |
| **i2c_scanner.ino** | I2C address scanner utility (for troubleshooting) |
| **credentials_template.h** | Template for WiFi and Thinger.io credentials |

## 🛠️ Hardware Requirements

- **ESP32 DevKit** (any variant)
- **MPU6050** (6-axis IMU)
- **3x Servo Motors** (for gimbal_stabilizer.ino only)
- **3x LED + 220Ω Resistors** (for gimbal_stabilizer.ino only)
- Breadboard & jumper wires

## 📚 Required Libraries

Install via Arduino IDE Library Manager:

```
- Adafruit MPU6050
- Adafruit Unified Sensor
- arduinoFFT
- ThingerESP32 (for fft_monitor.ino)
- ESP32Servo (for gimbal_stabilizer.ino)
```

## 🔌 Wiring

### Basic Setup (All Sketches)
```
MPU6050  →  ESP32
VCC      →  3.3V
GND      →  GND
SDA      →  GPIO 21
SCL      →  GPIO 22
```

### Additional for gimbal_stabilizer.ino
```
Servo Pitch  →  GPIO 25
Servo Roll   →  GPIO 26
Servo Yaw    →  GPIO 27

LED Green    →  GPIO 2 → 220Ω → GND
LED Yellow   →  GPIO 4 → 220Ω → GND
LED Red      →  GPIO 5 → 220Ω → GND
```

## 🚀 Quick Start

### 1. Setup Credentials (fft_monitor.ino only)

Copy `credentials_template.h` to `credentials.h`:

```cpp
// credentials.h
#define USERNAME "your_thinger_username"
#define DEVICE_ID "your_device_id"
#define DEVICE_CREDENTIAL "your_credential"
#define WIFI_SSID "your_wifi_ssid"
#define WIFI_PASSWORD "your_password"
```

**⚠️ Don't commit credentials.h to Git!**

### 2. Upload Sketch

1. Open `.ino` file in Arduino IDE
2. Select **Board:** ESP32 Dev Module
3. Select **Port:** Your ESP32 COM port
4. Click **Upload**

### 3. View Results

**For fft_monitor.ino:**
- Serial Monitor: 115200 baud
- Thinger.io Dashboard: https://console.thinger.io

**For gimbal_stabilizer.ino:**
- Serial Monitor: 115200 baud
- Watch LED indicators for system status

**For i2c_scanner.ino:**
- Serial Monitor: 115200 baud
- Check detected I2C addresses

## 📊 What Each Sketch Does

### fft_monitor.ino
- Samples accelerometer at 2 kHz (256 samples)
- Performs FFT to detect dominant frequency per axis
- Calculates magnitude and amplitude
- Sends data to Thinger.io for real-time visualization
- Displays results in Serial Monitor

**Output Example:**
```
╔════════════════════════════════════════╗
║         FFT ANALYSIS RESULTS           ║
╚════════════════════════════════════════╝

🔴 X-Axis:
  Dominant Frequency: 23.44 Hz
  Magnitude: 234.67
  Amplitude: 0.456 m/s²
```

### gimbal_stabilizer.ino
- Same FFT analysis as fft_monitor
- Classifies vibration into 4 modes:
  - **COMFORT** (0-10 Hz): Green LED, soft PID
  - **STANDARD** (10-20 Hz): Green LED, balanced PID
  - **SPORT** (20-40 Hz): Yellow LED, aggressive PID
  - **EXTREME** (>40 Hz): Red LED, maximum dampening
- Auto-adjusts servo PID parameters based on frequency
- Provides vibration score (0-100) and quality rating

**Output Example:**
```
🎯 STABILIZATION MODE:
  Current Mode: SPORT (Active Movement)
  Condition: Walking

⚙️  ADAPTIVE PID PARAMETERS:
  Kp = 1.80 | Ki = 0.30 | Kd = 0.80

💡 STATUS INDICATOR:
  🟡 YELLOW - Moderate Vibration Detected
```

### i2c_scanner.ino
- Scans I2C bus (addresses 0x00 to 0x7F)
- Reports found devices
- Useful for debugging MPU6050 connection issues

**Output Example:**
```
I2C Scanner
Scanning...
I2C device found at address 0x68  !
done
```

## 🔧 Troubleshooting

### MPU6050 not found
1. Run `i2c_scanner.ino` to verify connection
2. Check wiring (especially SDA/SCL)
3. Verify VCC is 3.3V (not 5V!)
4. Try pressing EN button on ESP32

### WiFi connection failed (fft_monitor.ino)
1. Verify credentials.h exists and has correct values
2. Ensure WiFi is 2.4GHz (ESP32 doesn't support 5GHz)
3. Check signal strength

### Upload failed
- Hold BOOT button during upload
- Release when "Connecting..." appears
- Check correct COM port selected

### Servo jitter (gimbal_stabilizer.ino)
- Use external power supply for servos
- Add capacitor (100µF) across servo power
- Reduce PID gains if oscillating

## 📖 FFT Analysis Explained

**Sampling:** 256 samples at 2000 Hz (or 1000 Hz for gimbal)

**Frequency Resolution:** 
- fft_monitor: 2000 Hz / 256 = 7.8 Hz per bin
- gimbal_stabilizer: 1000 Hz / 128 = 7.8 Hz per bin

**Windowing:** Hamming window to reduce spectral leakage

**Typical Frequency Ranges:**
- 0-10 Hz: Very slow motion (smooth panning)
- 10-20 Hz: Hand tremor
- 20-40 Hz: Walking / human motion
- 40-80 Hz: Running / fast movement
- '>80 Hz: High-frequency noise

## 🎯 Use Cases

- **Vibration Monitoring:** Detect machine vibrations
- **Motion Analysis:** Classify human activities (walking, running)
- **Gimbal Stabilization:** Auto-tune stabilizer parameters
- **Fall Detection:** Detect sudden high-frequency impacts
- **Structural Monitoring:** Building vibration analysis

## 🤝 Contributing

Contributions welcome! Please:
1. Fork the repo
2. Create feature branch
3. Commit your changes
4. Push and create pull request

## 📄 License

MIT License - see [LICENSE](LICENSE) file

## 👨‍💻 Author

**I Putu Agus Alit Putra Dana**
- GitHub: [@agusalit](https://github.com/agusalit)
- Institution: ITB STIKOM Bali

## 🙏 Acknowledgments

- Adafruit MPU6050 Library
- arduinoFFT by Enrique Condes
- Thinger.io Platform

---

⭐ **Star this repo if you find it useful!** ⭐

## Issues & Questions

Found a bug or have questions? [Open an issue](https://github.com/agusalit/fft_monitor/issues)