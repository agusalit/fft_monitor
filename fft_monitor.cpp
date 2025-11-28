#include <ThingerESP32.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <arduinoFFT.h>

// Thinger.io Configuration
#define USERNAME "[replace this with your thinger.io username]"
#define DEVICE_ID "[replace this with your thinger.io device_id]"
#define DEVICE_CREDENTIAL "[replace this with your thinger.io device_credential]"
#define WIFI_SSID "[replace this with your wifi name (SSID)]"
#define WIFI_PASSWORD "[replace this with your wifi password]"

ThingerESP32 thing(USERNAME, DEVICE_ID, DEVICE_CREDENTIAL);
Adafruit_MPU6050 mpu;

// FFT Configuration
#define SAMPLES 256
#define SAMPLING_FREQUENCY 2000  // Hz

double vRealX[SAMPLES], vImagX[SAMPLES];
double vRealY[SAMPLES], vImagY[SAMPLES];
double vRealZ[SAMPLES], vImagZ[SAMPLES];

ArduinoFFT<double> FFTX = ArduinoFFT<double>(vRealX, vImagX, SAMPLES, SAMPLING_FREQUENCY);
ArduinoFFT<double> FFTY = ArduinoFFT<double>(vRealY, vImagY, SAMPLES, SAMPLING_FREQUENCY);
ArduinoFFT<double> FFTZ = ArduinoFFT<double>(vRealZ, vImagZ, SAMPLES, SAMPLING_FREQUENCY);

// Variables
float accX = 0.0, accY = 0.0, accZ = 0.0;
double dominantFrequencyX = 0.0, maxMagnitudeX = 0.0, amplitudeX = 0.0;
double dominantFrequencyY = 0.0, maxMagnitudeY = 0.0, amplitudeY = 0.0;
double dominantFrequencyZ = 0.0, maxMagnitudeZ = 0.0, amplitudeZ = 0.0;

void setup() {
  Serial.begin(115200);
  
  // Initialize WiFi and Thinger.io
  thing.add_wifi(WIFI_SSID, WIFI_PASSWORD);
  
  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) { delay(10); }
  }
  
  Serial.println("MPU6050 Found!");
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  
  // Define Thinger.io resource
  thing["fft_monitor"] >> [](pson &out) {
    out["accX"] = accX;
    out["accY"] = accY;
    out["accZ"] = accZ;
    out["dominantFrequencyX"] = dominantFrequencyX;
    out["magnitudeX"] = maxMagnitudeX;
    out["amplitudeX"] = amplitudeX;
    out["dominantFrequencyY"] = dominantFrequencyY;
    out["magnitudeY"] = maxMagnitudeY;
    out["amplitudeY"] = amplitudeY;
    out["dominantFrequencyZ"] = dominantFrequencyZ;
    out["magnitudeZ"] = maxMagnitudeZ;
    out["amplitudeZ"] = amplitudeZ;
  };
  
  Serial.println("=== Setup Complete ===");
  Serial.println("Frequency Resolution: " + String(SAMPLING_FREQUENCY / SAMPLES) + " Hz per bin");
}

void loop() {
  thing.handle();
  
  collectDataAndPerformFFT();
  
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  accX = a.acceleration.x;
  accY = a.acceleration.y;
  accZ = a.acceleration.z;
  
  printResults();
  
  delay(1000);
}

void collectDataAndPerformFFT() {
  sensors_event_t a, g, temp;
  
  // Collect 256 samples
  for (int i = 0; i < SAMPLES; i++) {
    mpu.getEvent(&a, &g, &temp);
    
    vRealX[i] = a.acceleration.x;
    vRealY[i] = a.acceleration.y;
    vRealZ[i] = a.acceleration.z;
    vImagX[i] = vImagY[i] = vImagZ[i] = 0;
    
    delayMicroseconds(500);  // 2000 Hz sampling rate
  }
  
  // Perform FFT for each axis
  performFFTandAmplitude(FFTX, vRealX, &dominantFrequencyX, &maxMagnitudeX, &amplitudeX);
  performFFTandAmplitude(FFTY, vRealY, &dominantFrequencyY, &maxMagnitudeY, &amplitudeY);
  performFFTandAmplitude(FFTZ, vRealZ, &dominantFrequencyZ, &maxMagnitudeZ, &amplitudeZ);
}

void performFFTandAmplitude(ArduinoFFT<double> &FFT, double vReal[], 
                            double *dominantFreq, double *maxMag, double *amplitude) {
  // Remove DC bias
  double meanValue = 0;
  for (int i = 0; i < SAMPLES; i++) {
    meanValue += vReal[i];
  }
  meanValue /= SAMPLES;
  for (int i = 0; i < SAMPLES; i++) {
    vReal[i] -= meanValue;
  }
  
  // Calculate amplitude (peak-to-peak)
  double minValue = vReal[0], maxValue = vReal[0];
  for (int i = 1; i < SAMPLES; i++) {
    if (vReal[i] > maxValue) maxValue = vReal[i];
    if (vReal[i] < minValue) minValue = vReal[i];
  }
  *amplitude = maxValue - minValue;
  
  // Perform FFT
  FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.compute(FFT_FORWARD);
  FFT.complexToMagnitude();
  
  // Find dominant frequency (start from bin 2 = 15.6 Hz)
  *maxMag = 0;
  *dominantFreq = 0;
  int startBin = 2;
  
  for (int i = startBin; i < SAMPLES / 2; i++) {
    if (vReal[i] > *maxMag) {
      *maxMag = vReal[i];
      *dominantFreq = (i * 1.0 * SAMPLING_FREQUENCY) / SAMPLES;
    }
  }
}

void printResults() {
  Serial.println("╔════════════════════════════════════════╗");
  Serial.println("║         FFT ANALYSIS RESULTS           ║");
  Serial.println("╚════════════════════════════════════════╝");
  
  Serial.println("\n Acceleration:");
  Serial.print("  X: "); Serial.print(accX, 3); Serial.println(" m/s²");
  Serial.print("  Y: "); Serial.print(accY, 3); Serial.println(" m/s²");
  Serial.print("  Z: "); Serial.print(accZ, 3); Serial.println(" m/s²");
  
  Serial.println("\n🔴 X-Axis:");
  Serial.print("  Dominant Frequency: "); Serial.print(dominantFrequencyX, 2); Serial.println(" Hz");
  Serial.print("  Magnitude: "); Serial.println(maxMagnitudeX, 2);
  Serial.print("  Amplitude: "); Serial.print(amplitudeX, 3); Serial.println(" m/s²");
  
  Serial.println("\n🟢 Y-Axis:");
  Serial.print("  Dominant Frequency: "); Serial.print(dominantFrequencyY, 2); Serial.println(" Hz");
  Serial.print("  Magnitude: "); Serial.println(maxMagnitudeY, 2);
  Serial.print("  Amplitude: "); Serial.print(amplitudeY, 3); Serial.println(" m/s²");
  
  Serial.println("\n🔵 Z-Axis:");
  Serial.print("  Dominant Frequency: "); Serial.print(dominantFrequencyZ, 2); Serial.println(" Hz");
  Serial.print("  Magnitude: "); Serial.println(maxMagnitudeZ, 2);
  Serial.print("  Amplitude: "); Serial.print(amplitudeZ, 3); Serial.println(" m/s²");
  
  Serial.println("\n" + String('═', 40) + "\n");
}
