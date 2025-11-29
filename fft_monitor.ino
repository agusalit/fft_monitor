#include <ThingerESP32.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <arduinoFFT.h>  // Correct library name and case
#include "credentials.h" // IoT & Wi-Fi credential stored here

ThingerESP32 thing(USERNAME, DEVICE_ID, DEVICE_CREDENTIAL);
Adafruit_MPU6050 mpu;
// FFT configuration
#define SAMPLES 256                       // Increased number of samples for better resolution
#define SAMPLING_FREQUENCY 2000           // Sampling frequency in Hz (2 kHz)
double vRealX[SAMPLES], vImagX[SAMPLES];  // Arrays for X-axis FFT
double vRealY[SAMPLES], vImagY[SAMPLES];  // Arrays for Y-axis FFT
double vRealZ[SAMPLES], vImagZ[SAMPLES];  // Arrays for Z-axis FFT

ArduinoFFT<double> FFTX = ArduinoFFT<double>(vRealX, vImagX, SAMPLES, SAMPLING_FREQUENCY);
ArduinoFFT<double> FFTY = ArduinoFFT<double>(vRealY, vImagY, SAMPLES, SAMPLING_FREQUENCY);
ArduinoFFT<double> FFTZ = ArduinoFFT<double>(vRealZ, vImagZ, SAMPLES, SAMPLING_FREQUENCY);
// Variables to store sensor values, FFT results, and amplitude
float accX = 0.0, accY = 0.0, accZ = 0.0;
double dominantFrequencyX = 0.0, maxMagnitudeX = 0.0, amplitudeX = 0.0;
double dominantFrequencyY = 0.0, maxMagnitudeY = 0.0, amplitudeY = 0.0;
double dominantFrequencyZ = 0.0, maxMagnitudeZ = 0.0, amplitudeZ = 0.0;

void setup() {
  Serial.begin(115200);
  // Initialize Thinger.io
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
  delay(100);

  // Define the resource to send data to Thinger.io
  thing["acceleration_fft"] >> [](pson &out) {
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
}

void loop() {
  // Handle Thinger.io connection
  thing.handle();
  // Collect accelerometer data and perform FFT
  collectDataAndPerformFFT();
  // Update values to send to Thinger.io
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  accX = a.acceleration.x;
  accY = a.acceleration.y;
  accZ = a.acceleration.z;
  delay(1000);  // Send data every second
}

// Function to perform FFT on acceleration data for X, Y, and Z axes
void collectDataAndPerformFFT() {
  sensors_event_t a, g, temp;
  // Collect SAMPLES number of data points for X, Y, and Z axes
  for (int i = 0; i < SAMPLES; i++) {
    mpu.getEvent(&a, &g, &temp);
    // Collect data for each axis
    vRealX[i] = a.acceleration.x;
    vRealY[i] = a.acceleration.y;
    vRealZ[i] = a.acceleration.z;
    vImagX[i] = vImagY[i] = vImagZ[i] = 0;  // Imaginary parts initialized to 0
    delay(1000 / SAMPLING_FREQUENCY);       // Maintain the sampling rate
  }
  // Perform FFT and calculate amplitude for X, Y, and Z axes
  performFFTandAmplitude(FFTX, vRealX, &dominantFrequencyX, &maxMagnitudeX, &amplitudeX);
  performFFTandAmplitude(FFTY, vRealY, &dominantFrequencyY, &maxMagnitudeY, &amplitudeY);
  performFFTandAmplitude(FFTZ, vRealZ, &dominantFrequencyZ, &maxMagnitudeZ, &amplitudeZ);
}

// Function to perform FFT, calculate dominant frequency, magnitude, and amplitude
void performFFTandAmplitude(ArduinoFFT<double> &FFT, double vReal[], double *dominantFreq,
                            double *maxMag, double *amplitude) {
  // Remove DC Bias (mean)
  double meanValue = 0;
  for (int i = 0; i < SAMPLES; i++) {
    meanValue += vReal[i];
  }
  meanValue /= SAMPLES;
  for (int i = 0; i < SAMPLES; i++) {
    vReal[i] -= meanValue;  // Remove the DC bias
  }
  // Calculate amplitude (peak-to-peak in time domain)
  double minValue = vReal[0], maxValue = vReal[0];
  for (int i = 1; i < SAMPLES; i++) {
    if (vReal[i] > maxValue) maxValue = vReal[i];
    if (vReal[i] < minValue) minValue = vReal[i];
  }
  *amplitude = maxValue - minValue;  // Peak-to-peak amplitude
  // Perform FFT
  FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.compute(FFT_FORWARD);
  FFT.complexToMagnitude();
  // Find the dominant frequency (ignore very low frequencies)
  *maxMag = 0;
  *dominantFreq = 0;
  for (int i = 5; i < SAMPLES / 2; i++) {  // Ignore frequencies below 5 Hz
    if (vReal[i] > *maxMag) {
      *maxMag = vReal[i];
      *dominantFreq = (i * SAMPLING_FREQUENCY) / SAMPLES;
    }
  }
}