#include <Arduino.h>
#include <LSM6DS3.h>
#include <Wire.h>

/* ===== CONFIGURATION ===== */
#define MOTOR1_PIN D6   
#define MOTOR2_PIN D9   
#define MOTOR3_PIN D10  
#define MOTOR4_PIN D0   

#define PWM_RESOLUTION 10
#define PWM_MAX 1023

// Flight Timing
#define PREFLIGHT_DELAY_MS 10000 
#define RAMP_UP_DURATION_MS 3000   
#define HOVER_DURATION_MS 20000   
#define LANDING_DURATION_MS 5000  

// GENTLE PID Gains to prevent "pirouettes" and over-correction
float Kp_pitch = 0.4, Ki_pitch = 0.001, Kd_pitch = 0.08;
float Kp_roll  = 0.4, Ki_roll  = 0.001, Kd_roll  = 0.08;

#define PID_LIMIT 150 

// Increased throttle to break ground effect (approx 44%)
int target_hover_throttle = 450; 

// TRIM
float pitch_trim = -10.0; // Negative = Tilt Backwards (fixing forward drift cuz battery is unleveled)
float roll_trim  = 0.0;  // Adjust if drifting left/right

// Tilt Safety Limit (degrees)
#define TILT_LIMIT 75.0

/* ===== STATE VARIABLES ===== */
LSM6DS3 myIMU(I2C_MODE, 0x6A); 
bool flight_completed = false;
bool emergency_stop = false;
unsigned long flight_start_time = 0;

// PID Variables
float pitch_error_int = 0, roll_error_int = 0;
float last_pitch_error = 0, last_roll_error = 0;
bool first_loop = true;

// Calibration Offsets
float gyroX_offset = 0, gyroY_offset = 0, gyroZ_offset = 0;
float pitch_offset = 0, roll_offset = 0;

// FILTER VARIABLES (EMA Filter)
float filterAlpha = 0.12; // Increased smoothing
float smoothGyroX = 0, smoothGyroY = 0;
float smoothPitch = 0, smoothRoll = 0;

void setMotors(int m1, int m2, int m3, int m4) {
  if (emergency_stop) {
    analogWrite(MOTOR1_PIN, 0);
    analogWrite(MOTOR2_PIN, 0);
    analogWrite(MOTOR3_PIN, 0);
    analogWrite(MOTOR4_PIN, 0);
    return;
  }
  analogWrite(MOTOR1_PIN, constrain(m1, 0, PWM_MAX));
  analogWrite(MOTOR2_PIN, constrain(m2, 0, PWM_MAX));
  analogWrite(MOTOR3_PIN, constrain(m3, 0, PWM_MAX));
  analogWrite(MOTOR4_PIN, constrain(m4, 0, PWM_MAX));
}

void calibrateIMU() {
  Serial.println("IMU Calibrating... KEEP LEVEL AND STILL.");
  float sx = 0, sy = 0, sz = 0;
  for (int i = 0; i < 200; i++) {
    sx += myIMU.readFloatGyroX();
    sy += myIMU.readFloatGyroY();
    sz += myIMU.readFloatGyroZ();
    delay(10);
  }
  gyroX_offset = sx / 200.0;
  gyroY_offset = sy / 200.0;
  gyroZ_offset = sz / 200.0;
  
  // Initialize filters
  smoothGyroX = 0;
  smoothGyroY = 0;
  smoothPitch = 0;
  smoothRoll = 0;
  Serial.println("Calibration Done.");
}

void setup() {
  Serial.begin(115200);
  
  pinMode(PD5, OUTPUT); 
  digitalWrite(PD5, HIGH);
  delay(500);

  if (myIMU.begin() != 0) {
    while (1) { Serial.println("IMU Error!"); delay(1000); }
  }

  analogWriteResolution(PWM_RESOLUTION);
  pinMode(MOTOR1_PIN, OUTPUT);
  pinMode(MOTOR2_PIN, OUTPUT);
  pinMode(MOTOR3_PIN, OUTPUT);
  pinMode(MOTOR4_PIN, OUTPUT);
  setMotors(0, 0, 0, 0);

  Serial.println("PRE-CALIBRATION: 5s to place drone LEVEL...");
  delay(5000);

  calibrateIMU();

  Serial.println("\n--- TILT DETECTION TEST ---");
  Serial.println("Sequence: 15s Wait -> 2s Ramp -> 10s Hover -> 3s Land");
  
  for(int i = 15; i > 0; i--) {
    float ax = myIMU.readFloatAccelX();
    float ay = myIMU.readFloatAccelY();
    float az = myIMU.readFloatAccelZ();
    float currentPitch = (atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / M_PI) - pitch_offset;
    float currentRoll  = (atan2(ay, az) * 180.0 / M_PI) - roll_offset;

    Serial.print("T-Minus: "); Serial.print(i); 
    Serial.print("s | LIVE -> P: "); Serial.print(currentPitch, 1);
    Serial.print(" R: "); Serial.println(currentRoll, 1);
    delay(1000);
  }

  Serial.println("!!! MOTOR START !!!");
  flight_start_time = millis();
}


void loop() {
  // --- RESET COMMAND ---
  // sending r or R resets full thing
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'r' || c == 'R') {
      Serial.println("RESTARTING SYSTEM...");
      delay(100);
      NVIC_SystemReset(); // Software Reset for XIAO MG24
    }
    if (c == 's' || c == 'S') {
      Serial.println("!!! MANUAL STOP TRIGGERED !!!");
      emergency_stop = true;
    }
  }

  if (flight_completed || emergency_stop) {
    setMotors(0, 0, 0, 0);
    if (emergency_stop) {
      Serial.println("EMERGENCY STOP: TILT LIMIT EXCEEDED!");
    }
    return;
  }

  unsigned long current_time = millis();
  unsigned long elapsed = current_time - flight_start_time;

  // Read Sensors
  float rawGyroPitch = myIMU.readFloatGyroX() - gyroX_offset; 
  float rawGyroRoll  = myIMU.readFloatGyroY() - gyroY_offset; 
  float ax = myIMU.readFloatAccelX();
  float ay = myIMU.readFloatAccelY();
  float az = myIMU.readFloatAccelZ();

  // Calculate Tilt Angles (Accelerometer based) AND Subtract Offsets
  float currentPitch = (atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / M_PI) - pitch_offset;
  float currentRoll  = (atan2(ay, az) * 180.0 / M_PI) - roll_offset;

  // Smooth data
  smoothGyroX = (filterAlpha * rawGyroPitch) + ((1.0 - filterAlpha) * smoothGyroX);
  smoothGyroY = (filterAlpha * rawGyroRoll) + ((1.0 - filterAlpha) * smoothGyroY);
  smoothPitch = (filterAlpha * currentPitch) + ((1.0 - filterAlpha) * smoothPitch);
  smoothRoll  = (filterAlpha * currentRoll) + ((1.0 - filterAlpha) * smoothRoll);

  // Safety Check: Tilt Kill (Relative to our zeroed orientation)
  if (abs(smoothPitch) > TILT_LIMIT || abs(smoothRoll) > TILT_LIMIT) {
    emergency_stop = true;
  }

  if (elapsed < (RAMP_UP_DURATION_MS + HOVER_DURATION_MS)) {
    // --- RAMP UP & HOVER ---
    int current_base_throttle = target_hover_throttle;
    if (elapsed < RAMP_UP_DURATION_MS) {
      current_base_throttle = map(elapsed, 0, RAMP_UP_DURATION_MS, 0, target_hover_throttle);
    }

    // PID Calculation (Rate-based to dampen violent movements)
    // INVERTED PITCH: smoothGyroX increases when forward, so we subtract it from trim
    float pitch_error = pitch_trim - smoothGyroX; 
    float roll_error  = roll_trim - smoothGyroY;

    if (first_loop) {
      last_pitch_error = pitch_error;
      last_roll_error = roll_error;
      first_loop = false;
    }

    pitch_error_int = constrain(pitch_error_int + pitch_error, -100, 100);
    roll_error_int = constrain(roll_error_int + roll_error, -100, 100);

    float pitch_output = (Kp_pitch * pitch_error) + (Ki_pitch * pitch_error_int) + (Kd_pitch * (pitch_error - last_pitch_error));
    float roll_output  = (Kp_roll * roll_error) + (Ki_roll * roll_error_int) + (Kd_roll * (roll_error - last_roll_error));

    last_pitch_error = pitch_error;
    last_roll_error = roll_error;

    pitch_output = constrain(pitch_output, -PID_LIMIT, PID_LIMIT);
    roll_output  = constrain(roll_output, -PID_LIMIT, PID_LIMIT);

    // 5. Motor Mixing (CUSTOM X-Configuration)
    // Front: D0(FL), D10(FR) | Back: D6(BL), D9(BR)
    // Left: D0(FL), D6(BL)  | Right: D10(FR), D9(BR)
    int m_fl = current_base_throttle - pitch_output - roll_output; // D0
    int m_fr = current_base_throttle - pitch_output + roll_output; // D10
    int m_br = current_base_throttle + pitch_output + roll_output; // D9
    int m_bl = current_base_throttle + pitch_output - roll_output; // D6

    // Map to user-defined pins
    analogWrite(MOTOR1_PIN, constrain(m_bl, 0, PWM_MAX)); // D6 is back-left
    analogWrite(MOTOR2_PIN, constrain(m_br, 0, PWM_MAX)); // D9 is back-right
    analogWrite(MOTOR3_PIN, constrain(m_fr, 0, PWM_MAX)); // D10 is front-right
    analogWrite(MOTOR4_PIN, constrain(m_fl, 0, PWM_MAX)); // D0 is front-left

    // Diagnostic Print
    if (elapsed % 200 < 20) {
      Serial.print("TILT -> P: "); Serial.print(smoothPitch, 1);
      Serial.print(" R: "); Serial.print(smoothRoll, 1);
      Serial.print(" | THR: "); Serial.println(current_base_throttle);
    }

  } else if (elapsed < (RAMP_UP_DURATION_MS + HOVER_DURATION_MS + LANDING_DURATION_MS)) {
    // --- LANDING ---
    unsigned long land_elapsed = elapsed - (RAMP_UP_DURATION_MS + HOVER_DURATION_MS);
    float factor = 1.0 - ((float)land_elapsed / LANDING_DURATION_MS);
    int land_throttle = target_hover_throttle * factor;
    setMotors(land_throttle, land_throttle, land_throttle, land_throttle);
    
    if (elapsed % 500 < 20) Serial.println("LANDING...");
    
  } else {
    // --- FINISHED ---
    setMotors(0, 0, 0, 0);
    flight_completed = true;
    Serial.println("TEST COMPLETED. Disconnect battery to reset.");
  }

  delay(10); 
}
