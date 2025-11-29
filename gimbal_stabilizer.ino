#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <arduinoFFT.h>
#include <ESP32Servo.h>

// MPU6050 sensor
Adafruit_MPU6050 mpu;

// FFT Configuration
#define SAMPLES 128
#define SAMPLING_FREQUENCY 1000  // 1 kHz
double vRealX[SAMPLES], vImagX[SAMPLES];
double vRealY[SAMPLES], vImagY[SAMPLES];
double vRealZ[SAMPLES], vImagZ[SAMPLES];

ArduinoFFT<double> FFTX = ArduinoFFT<double>(vRealX, vImagX, SAMPLES, SAMPLING_FREQUENCY);
ArduinoFFT<double> FFTY = ArduinoFFT<double>(vRealY, vImagY, SAMPLES, SAMPLING_FREQUENCY);
ArduinoFFT<double> FFTZ = ArduinoFFT<double>(vRealZ, vImagZ, SAMPLES, SAMPLING_FREQUENCY);

// Servo motors for 3-axis gimbal
Servo servoPitch, servoRoll, servoYaw;
#define SERVO_PITCH_PIN 25
#define SERVO_ROLL_PIN 26
#define SERVO_YAW_PIN 27

// LED Status Indicators
#define LED_GREEN 2   // System OK
#define LED_YELLOW 4  // Warning
#define LED_RED 5     // High Vibration

// FFT Results
double dominantFreqX = 0, dominantFreqY = 0, dominantFreqZ = 0;
double avgDominantFreq = 0;
double magnitudeX = 0, magnitudeY = 0, magnitudeZ = 0;

float Kp = 1.0, Ki = 0.1, Kd = 0.5;

// Servo positions
int pitchPos = 90, rollPos = 90, yawPos = 90;

// Mode stabilisasi
enum StabilizationMode {
  COMFORT,    // 0-10 Hz
  STANDARD,   // 10-20 Hz
  SPORT,      // 20-40 Hz
  EXTREME     // >40 Hz
};
StabilizationMode currentMode = STANDARD;

void setup() {
  Serial.begin(115200);
  
  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050!");
    while (1) { delay(10); }
  }
  Serial.println("MPU6050 Found!");
  
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  // Initialize Servos
  servoPitch.attach(SERVO_PITCH_PIN);
  servoRoll.attach(SERVO_ROLL_PIN);
  servoYaw.attach(SERVO_YAW_PIN);
  
  // Center servos
  servoPitch.write(90);
  servoRoll.write(90);
  servoYaw.write(90);
  
  // Initialize LEDs
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  
  // Initial status: Green (OK)
  digitalWrite(LED_GREEN, HIGH);
  
  Serial.println("╔════════════════════════════════════════╗");
  Serial.println("║  GIMBAL FFT ADAPTIVE STABILIZATION    ║");
  Serial.println("║           CONCEPT DEMO                ║");
  Serial.println("╚════════════════════════════════════════╝\n");
  
  delay(1000);
}

void loop() {
  // 1. Collect data and perform FFT
  collectDataAndPerformFFT();
  
  // 2. Calculate average dominant frequency
  avgDominantFreq = (dominantFreqX + dominantFreqY + dominantFreqZ) / 3.0;
  
  // 3. Classify condition and adapt PID
  classifyAndAdapt();
  
  // 4. Update LED status
  updateLEDStatus();
  
  // 5. Simulate servo stabilization
  stabilizeGimbal();
  
  // 6. Print status
  printStatus();
  
  delay(2000);  // Update every 2 seconds for demo
}

void collectDataAndPerformFFT() {
  sensors_event_t a, g, temp;
  
  // Collect samples
  for (int i = 0; i < SAMPLES; i++) {
    mpu.getEvent(&a, &g, &temp);
    
    vRealX[i] = a.acceleration.x;
    vRealY[i] = a.acceleration.y;
    vRealZ[i] = a.acceleration.z;
    vImagX[i] = vImagY[i] = vImagZ[i] = 0;
    
    delayMicroseconds(1000);  // 1 kHz sampling
  }
  
  // Perform FFT for each axis
  performFFT(FFTX, vRealX, &dominantFreqX, &magnitudeX);
  performFFT(FFTY, vRealY, &dominantFreqY, &magnitudeY);
  performFFT(FFTZ, vRealZ, &dominantFreqZ, &magnitudeZ);
}

void performFFT(ArduinoFFT<double> &FFT, double vReal[], double *dominantFreq, double *maxMag) {
  // Remove DC bias
  double mean = 0;
  for (int i = 0; i < SAMPLES; i++) {
    mean += vReal[i];
  }
  mean /= SAMPLES;
  for (int i = 0; i < SAMPLES; i++) {
    vReal[i] -= mean;
  }
  
  // Perform FFT
  FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.compute(FFT_FORWARD);
  FFT.complexToMagnitude();
  
  // Find dominant frequency
  *maxMag = 0;
  *dominantFreq = 0;
  for (int i = 2; i < SAMPLES / 2; i++) {
    if (vReal[i] > *maxMag) {
      *maxMag = vReal[i];
      *dominantFreq = (i * 1.0 * SAMPLING_FREQUENCY) / SAMPLES;
    }
  }
}

void classifyAndAdapt() {
  // Classify based on average dominant frequency
  if (avgDominantFreq < 10) {
    currentMode = COMFORT;
    Kp = 0.8; Ki = 0.1; Kd = 0.3;  // Soft, smooth response
  } 
  else if (avgDominantFreq < 20) {
    currentMode = STANDARD;
    Kp = 1.2; Ki = 0.2; Kd = 0.5;  // Medium, balanced
  } 
  else if (avgDominantFreq < 40) {
    currentMode = SPORT;
    Kp = 1.8; Ki = 0.3; Kd = 0.8;  // Aggressive, fast response
  } 
  else {
    currentMode = EXTREME;
    Kp = 2.5; Ki = 0.4; Kd = 1.2;  // Maximum dampening
  }
}

void updateLEDStatus() {
  // Turn off all LEDs first
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_YELLOW, LOW);
  digitalWrite(LED_RED, LOW);
  
  // Light appropriate LED based on mode
  switch (currentMode) {
    case COMFORT:
      digitalWrite(LED_GREEN, HIGH);  // Green: OK
      break;
    case STANDARD:
      digitalWrite(LED_GREEN, HIGH);  // Green: OK
      break;
    case SPORT:
      digitalWrite(LED_YELLOW, HIGH); // Yellow: Warning
      break;
    case EXTREME:
      digitalWrite(LED_RED, HIGH);    // Red: High vibration
      break;
  }
}

void stabilizeGimbal() {
  // Simulate gimbal stabilization using gyroscope data
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  // Simple proportional control (demonstration only)
  // Real implementation would use full PID with integral and derivative
  
  // Pitch compensation (X-axis rotation)
  float pitchError = g.gyro.x * Kp;
  pitchPos = constrain(90 - pitchError, 0, 180);
  servoPitch.write(pitchPos);
  
  // Roll compensation (Y-axis rotation)
  float rollError = g.gyro.y * Kp;
  rollPos = constrain(90 - rollError, 0, 180);
  servoRoll.write(rollPos);
  
  // Yaw compensation (Z-axis rotation)
  float yawError = g.gyro.z * Kp;
  yawPos = constrain(90 - yawError, 0, 180);
  servoYaw.write(yawPos);
}

void printStatus() {
  Serial.println("╔════════════════════════════════════════════════╗");
  Serial.println("║         FFT ADAPTIVE GIMBAL STATUS             ║");
  Serial.println("╚════════════════════════════════════════════════╝");
  
  // FFT Results
  Serial.println("\n📊 FFT ANALYSIS:");
  Serial.print("  Dominant Frequency X: "); Serial.print(dominantFreqX, 2); Serial.println(" Hz");
  Serial.print("  Dominant Frequency Y: "); Serial.print(dominantFreqY, 2); Serial.println(" Hz");
  Serial.print("  Dominant Frequency Z: "); Serial.print(dominantFreqZ, 2); Serial.println(" Hz");
  Serial.print("  ➤ Average: "); Serial.print(avgDominantFreq, 2); Serial.println(" Hz");
  
  // Mode Classification
  Serial.println("\n🎯 STABILIZATION MODE:");
  Serial.print("  Current Mode: ");
  switch (currentMode) {
    case COMFORT:
      Serial.println("COMFORT (Smooth Operation)");
      Serial.println("  Condition: Static/Tripod");
      break;
    case STANDARD:
      Serial.println("STANDARD (Handheld)");
      Serial.println("  Condition: Hand tremor");
      break;
    case SPORT:
      Serial.println("SPORT (Active Movement)");
      Serial.println("  Condition: Walking");
      break;
    case EXTREME:
      Serial.println("EXTREME (High Vibration)");
      Serial.println("  Condition: Running/Vehicle");
      break;
  }
  
  // PID Parameters
  Serial.println("\n⚙️  ADAPTIVE PID PARAMETERS:");
  Serial.print("  Kp = "); Serial.print(Kp, 2);
  Serial.print(" | Ki = "); Serial.print(Ki, 2);
  Serial.print(" | Kd = "); Serial.println(Kd, 2);
  
  // Servo Positions
  Serial.println("\n🔧 GIMBAL SERVO POSITIONS:");
  Serial.print("  Pitch: "); Serial.print(pitchPos); Serial.println("°");
  Serial.print("  Roll:  "); Serial.print(rollPos); Serial.println("°");
  Serial.print("  Yaw:   "); Serial.print(yawPos); Serial.println("°");
  
  // Status LED
  Serial.println("\n💡 STATUS INDICATOR:");
  if (digitalRead(LED_GREEN)) {
    Serial.println("  🟢 GREEN - System Operating Normally");
  } else if (digitalRead(LED_YELLOW)) {
    Serial.println("  🟡 YELLOW - Moderate Vibration Detected");
  } else if (digitalRead(LED_RED)) {
    Serial.println("  🔴 RED - High Vibration! Check Conditions");
  }
  
  // Vibration Score (0-100)
  int vibrationScore = constrain(avgDominantFreq * 2, 0, 100);
  Serial.println("\n📈 VIBRATION SCORE:");
  Serial.print("  Score: "); Serial.print(vibrationScore); Serial.println("/100");
  
  if (vibrationScore < 20) {
    Serial.println("  Quality: ⭐⭐⭐⭐⭐ Excellent");
  } else if (vibrationScore < 40) {
    Serial.println("  Quality: ⭐⭐⭐⭐ Good");
  } else if (vibrationScore < 60) {
    Serial.println("  Quality: ⭐⭐⭐ Fair");
  } else {
    Serial.println("  Quality: ⭐⭐ Poor");
  }
  
  Serial.println("\n" + String('═', 48) + "\n");
}