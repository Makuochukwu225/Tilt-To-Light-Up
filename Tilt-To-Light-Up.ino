/*
 * MPU6050 LED Tilt Indicator
 * 
 * This sketch reads orientation data from an MPU6050 IMU sensor
 * and controls 7 LEDs based on the tilt angle:
 * - When flat, only the middle (4th) LED lights up
 * - When tilted left or right, LEDs shift accordingly
 * 
 * Connections:
 * - MPU6050: Connect VCC to 3.3V, GND to GND, SDA to A4, SCL to A5
 * - LEDs: Connect to pins 2-8 through appropriate resistors (220-330 ohm)
 */

#include <Wire.h>

// LED pins (from left to right)
const int NUM_LEDS = 7;
const int LED_PINS[NUM_LEDS] = {2, 3, 4, 5, 6, 7, 8};

// MPU6050 I2C address
const int MPU_ADDR = 0x68;

// Accelerometer registers
const int ACCEL_XOUT_H = 0x3B;
const int ACCEL_XOUT_L = 0x3C;
const int ACCEL_YOUT_H = 0x3D;
const int ACCEL_YOUT_L = 0x3E;
const int ACCEL_ZOUT_H = 0x3F;
const int ACCEL_ZOUT_L = 0x40;

// Power management register
const int PWR_MGMT_1 = 0x6B;

// Calibration values
float accelBiasX = 0.0;
float accelBiasY = 0.0;
float accelBiasZ = 0.0;

// Angle thresholds for LED transitions (degrees)
const float ANGLE_THRESHOLD = 10.0; // Degrees between LED transitions

// Function prototypes
void initMPU();
void readAccelData(float* ax, float* ay, float* az);
int16_t readWord(int regAddr);

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Initialize LED pins as outputs
  for (int i = 0; i < NUM_LEDS; i++) {
    pinMode(LED_PINS[i], OUTPUT);
    digitalWrite(LED_PINS[i], LOW);
  }
  
  // Initialize MPU6050
  initMPU();
  
  // Light up middle LED during calibration
  digitalWrite(LED_PINS[NUM_LEDS/2], HIGH);
  
  Serial.println("Calibrating accelerometer...");
  Serial.println("Keep the device flat and still for 2 seconds");

  // Collect calibration data
  delay(2000);  // Give time for positioning
  
  // Take multiple samples for averaging
  int samples = 100;
  for (int i = 0; i < samples; i++) {
    float ax, ay, az;
    readAccelData(&ax, &ay, &az);
    
    accelBiasX += ax;
    accelBiasY += ay;
    accelBiasZ += az;
    
    delay(10); 
  }

  // Calculate average bias
  accelBiasX /= samples;
  accelBiasY /= samples;
  accelBiasZ /= samples;
  
  // Normalize Z bias to 1g (using 16384 as the scale factor for ±2g range)
  float zMagnitude = accelBiasZ > 0 ? accelBiasZ : -accelBiasZ;
  if (zMagnitude > 8000) {  // Check if Z is roughly aligned with gravity
    accelBiasZ -= (accelBiasZ > 0 ? 16384.0 : -16384.0);  // Subtract 1g in the proper direction
  }

  Serial.println("Calibration complete");
  Serial.print("X Bias: "); Serial.println(accelBiasX);
  Serial.print("Y Bias: "); Serial.println(accelBiasY);
  Serial.print("Z Bias: "); Serial.println(accelBiasZ);
  
  // Turn off middle LED after calibration
  digitalWrite(LED_PINS[NUM_LEDS/2], LOW);
  
  Serial.println("Setup complete. Begin tilting the device!");
}

void loop() {
  // Get accelerometer data
  float ax, ay, az;
  readAccelData(&ax, &ay, &az);
  
  // Apply calibration offsets
  ax -= accelBiasX;
  ay -= accelBiasY;
  az -= accelBiasZ;
  
  // Calculate tilt angle (roll) using accelerometer
  // Convert from raw values to g's for calculation
  float accelX = ax / 16384.0;  // Scale factor for ±2g range
  float accelY = ay / 16384.0;
  float accelZ = az / 16384.0;
  
  // Calculate roll angle (left-right tilt)
  float rollAngle = atan2(accelX, sqrt(accelY*accelY + accelZ*accelZ)) * 180.0 / PI;
  
  // Determine which LED to light up based on tilt angle
  int activeLed = NUM_LEDS / 2;  // Start with middle LED (index 3 for 7 LEDs)
  
  // Calculate how many LEDs to shift from the middle
  int ledShift = int(rollAngle / ANGLE_THRESHOLD);
  
  // Constrain to valid range
  activeLed += ledShift;
  if (activeLed < 0) activeLed = 0;
  if (activeLed >= NUM_LEDS) activeLed = NUM_LEDS - 1;
  
  // Update LEDs
  updateLEDs(activeLed);
  
  // Print data for debugging
  Serial.print("Roll angle: ");
  Serial.print(rollAngle);
  Serial.print(" degrees, Active LED: ");
  Serial.println(activeLed);
  
  delay(50);  // Short delay to prevent excessive updates
}

void updateLEDs(int activeLed) {
  // Turn off all LEDs
  for (int i = 0; i < NUM_LEDS; i++) {
    digitalWrite(LED_PINS[i], LOW);
  }
  
  // Turn on the active LED
  digitalWrite(LED_PINS[activeLed], HIGH);
}

// Initialize the MPU6050
void initMPU() {
  // Wake up the MPU-6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(PWR_MGMT_1);  // Power management register
  Wire.write(0);           // Set to zero to wake up
  Wire.endTransmission(true);
  
  delay(100);  // Short delay after waking up
  
  // Check if sensor responds
  Wire.beginTransmission(MPU_ADDR);
  byte error = Wire.endTransmission();
  
  if (error != 0) {
    Serial.println("MPU6050 connection failed");
    while (1) {
      // Flash all LEDs if sensor connection fails
      for (int i = 0; i < NUM_LEDS; i++) {
        digitalWrite(LED_PINS[i], HIGH);
      }
      delay(200);
      for (int i = 0; i < NUM_LEDS; i++) {
        digitalWrite(LED_PINS[i], LOW);
      }
      delay(200);
    }
  }
  
  Serial.println("MPU6050 connected successfully");
}

// Read accelerometer data from sensor
void readAccelData(float* ax, float* ay, float* az) {
  *ax = readWord(ACCEL_XOUT_H);
  *ay = readWord(ACCEL_YOUT_H);
  *az = readWord(ACCEL_ZOUT_H);
}

// Read a 16-bit value from the MPU (two 8-bit registers)
int16_t readWord(int regAddr) {
  int16_t value;
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(regAddr);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 2, true);
  
  if (Wire.available() >= 2) {
    value = (Wire.read() << 8) | Wire.read();
  } else {
    value = 0;  // Return 0 if not enough bytes available
  }
  
  return value;
}
