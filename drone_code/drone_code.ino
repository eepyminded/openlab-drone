#include <Arduino.h> 
#include <LSM6DS3.h> 
#include <Wire.h> 

/* ===== KONFIGURACJA ===== */ 
#define MOTOR_FL_PIN D0    
#define MOTOR_FR_PIN D10   
#define MOTOR_BR_PIN D9    
#define MOTOR_BL_PIN D6    

#define PWM_RESOLUTION 10 
#define PWM_MAX 1023 

// Timery
#define LOOP_TIME_MS 4            // petla 250Hz
#define PRE_CALIB_WAIT_SEC 10     // czas na odpiecie kabla
#define ARMING_COOLDOWN_SEC 5     // opoznienie przed startem
#define FLIGHT_TIME_MS 20000      // max czas lotu (20s)

/* ===== PID (KASKADA) ===== */ 
// Zewn: kat -> predkosc (deg/s) 
float Kp_angle = 1.5; // 0.0 na pierwszy test, najpierw tune rate

// Wewn: predkosc -> silniki 
float Kp_rate = 0.3;   
float Ki_rate = 0.04;  
float Kd_rate = 0.05;  

// Yaw 
float Kp_yaw = 1.5; 

#define RATE_LIMIT 150.0  
#define PID_LIMIT 300  
#define TILT_LIMIT 45.0 // max odchylenie

/* ===== STAN ===== */ 
LSM6DS3 myIMU(I2C_MODE, 0x6A);  
bool emergency_stop = false; 
bool flight_completed = false; 

unsigned long flight_start_time = 0; 
unsigned long last_loop_time = 0; 
unsigned long last_print_time = 0;

// Kat z filtra
float angle_pitch = 0, angle_roll = 0; 

// Pamiec PID
float pitch_rate_int = 0, roll_rate_int = 0; 
float last_pitch_rate_error = 0, last_roll_rate_error = 0; 
float last_pitch_d = 0, last_roll_d = 0; 

// Kalibracja 
float gyroX_offset = 0, gyroY_offset = 0, gyroZ_offset = 0; 

// Gaz reczny
int manual_throttle = 120;


/* ===== FUNKCJE ===== */

void setMotors(int fl, int fr, int br, int bl) { 
  if (emergency_stop || flight_completed) { 
    analogWrite(MOTOR_FL_PIN, 0); 
    analogWrite(MOTOR_FR_PIN, 0); 
    analogWrite(MOTOR_BR_PIN, 0); 
    analogWrite(MOTOR_BL_PIN, 0); 
    return; 
  } 
  analogWrite(MOTOR_FL_PIN, constrain(fl, 0, PWM_MAX)); 
  analogWrite(MOTOR_FR_PIN, constrain(fr, 0, PWM_MAX)); 
  analogWrite(MOTOR_BR_PIN, constrain(br, 0, PWM_MAX)); 
  analogWrite(MOTOR_BL_PIN, constrain(bl, 0, PWM_MAX)); 
} 

void calibrateIMU() { 
  Serial.println("\n--- KALIBRACJA IMU ---"); 
  Serial.println("NIE DOTYKAJ DRONA!"); 
  float sx = 0, sy = 0, sz = 0; 
  
  // Probkowanie tla
  for (int i = 0; i < 500; i++) { 
    sx += myIMU.readFloatGyroX(); 
    sy += myIMU.readFloatGyroY(); 
    sz += myIMU.readFloatGyroZ(); 
    delay(2); 
  } 
  gyroX_offset = sx / 500.0; 
  gyroY_offset = sy / 500.0; 
  gyroZ_offset = sz / 500.0; 
   
  // Inicjalizacja katow
  float ax = myIMU.readFloatAccelX(); 
  float ay = myIMU.readFloatAccelY(); 
  float az = myIMU.readFloatAccelZ(); 
  angle_pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / M_PI; 
  angle_roll  = atan2(ay, az) * 180.0 / M_PI; 
  
  Serial.println("Kalibracja OK.\n"); 
} 

void setup() { 
  Serial.begin(115200); 

  pinMode(PD5, OUTPUT); digitalWrite(PD5, HIGH); delay(500); 

  if (myIMU.begin() != 0) { 
    while (1) { Serial.println("Blad IMU!"); delay(1000); } 
  } 

  analogWriteResolution(PWM_RESOLUTION); 
  pinMode(MOTOR_FL_PIN, OUTPUT); pinMode(MOTOR_FR_PIN, OUTPUT); 
  pinMode(MOTOR_BR_PIN, OUTPUT); pinMode(MOTOR_BL_PIN, OUTPUT); 
  setMotors(0, 0, 0, 0); 

  // --- 1: START ---
  Serial.println("=========================================");
  Serial.println("ZASILANIE WLACZONE.");
  Serial.print("MASZ "); Serial.print(PRE_CALIB_WAIT_SEC); Serial.println(" SEKUND NA ODLACZENIE USB");
  Serial.println("I USTAWIENIE DRONA PLASKO.");
  Serial.println("=========================================");
  for (int i = PRE_CALIB_WAIT_SEC; i > 0; i--) {
    Serial.print("Kalibracja za "); Serial.print(i); Serial.println("s...");
    delay(1000);
  }

  // --- 2: KALIBRACJA ---
  calibrateIMU(); 
  
  // --- 3: UZBROJENIE ---
  Serial.println("=========================================");
  Serial.println("KALIBRACJA OK. ZARAZ START.");
  Serial.println("ODSUN SIE!");
  Serial.println("=========================================");
  for (int i = ARMING_COOLDOWN_SEC; i > 0; i--) {
    Serial.print("Uzbrajanie za "); Serial.print(i); Serial.println("s...");
    delay(1000);
  }
  
  Serial.println("\nGOTOWY DO LOTU.");
  Serial.println("'w' - gaz +, 's' - gaz -, 'SPACJA' - stop.");
   
  flight_start_time = millis(); 
  last_loop_time = micros(); 
} 

void loop() { 
  // --- RX SERIAL ---
  if (Serial.available()) { 
    char c = Serial.read(); 
    
    if (c == 'r' || c == 'R') NVIC_SystemReset(); 
    
    if (!flight_completed) {
      if (c == ' ') { // kill switch
        emergency_stop = true; 
        manual_throttle = 0;
        Serial.println("AWARYJNY STOP!");
      }
      if (c == 'w' || c == 'W') {
        manual_throttle += 20; 
        emergency_stop = false; 
      }
      if (c == 's' || c == 'S') manual_throttle -= 20;
      
      manual_throttle = constrain(manual_throttle, 0, PWM_MAX - PID_LIMIT);
    }
  } 

  // --- BLOKADA SILNIKOW ---
  if (flight_completed) {
    setMotors(0, 0, 0, 0);
    return; 
  }

  if (emergency_stop) { 
    setMotors(0, 0, 0, 0); 
    return; 
  } 

  // --- TIMEOUT LOTU ---
  if (millis() - flight_start_time >= FLIGHT_TIME_MS) {
    flight_completed = true;
    manual_throttle = 0;
    setMotors(0, 0, 0, 0);
    Serial.println("\n=========================================");
    Serial.println("LIMIT CZASU OSIAGNIETY.");
    Serial.println("SILNIKI STOP.");
    Serial.println("Wyslij 'r' lub reset zasilania.");
    Serial.println("=========================================\n");
    return; 
  }

  // --- SYNC PETLI ---
  unsigned long current_micros = micros(); 
  if (current_micros - last_loop_time < LOOP_TIME_MS * 1000) return; 
  float dt = (current_micros - last_loop_time) / 1000000.0; 
  last_loop_time = current_micros; 

  // --- 1. IMU ---
  float gx = myIMU.readFloatGyroX() - gyroX_offset; 
  float gy = myIMU.readFloatGyroY() - gyroY_offset; 
  float gz = myIMU.readFloatGyroZ() - gyroZ_offset; 
  
  float ax = myIMU.readFloatAccelX(); 
  float ay = myIMU.readFloatAccelY(); 
  float az = myIMU.readFloatAccelZ(); 

  // --- 2. FILTR ---
  float acc_pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / M_PI; 
  float acc_roll  = atan2(ay, az) * 180.0 / M_PI; 
  angle_pitch = 0.98 * (angle_pitch + gx * dt) + 0.02 * acc_pitch; 
  angle_roll  = 0.98 * (angle_roll + gy * dt) + 0.02 * acc_roll; 

  // Zabezpieczenie przed wywrotka
  if (abs(angle_pitch) > TILT_LIMIT || abs(angle_roll) > TILT_LIMIT) {
    emergency_stop = true; 
    manual_throttle = 0;
    Serial.println("KAT KRYTYCZNY - STOP");
  }

  // --- 3. PID ---
  if (manual_throttle > 50) { 

    float target_rate_pitch = constrain(Kp_angle * (0.0 - angle_pitch), -RATE_LIMIT, RATE_LIMIT); 
    float target_rate_roll  = constrain(Kp_angle * (0.0 - angle_roll), -RATE_LIMIT, RATE_LIMIT); 
    float target_rate_yaw   = 0.0; 

    float pitch_rate_error = target_rate_pitch - gx; 
    float roll_rate_error  = target_rate_roll - gy; 
    float yaw_rate_error   = target_rate_yaw - gz; 

    pitch_rate_int = constrain(pitch_rate_int + (pitch_rate_error * dt), -100, 100); 
    roll_rate_int  = constrain(roll_rate_int + (roll_rate_error * dt), -100, 100); 

    // LPF dla D
    float current_pitch_d = (pitch_rate_error - last_pitch_rate_error) / dt;
    float current_roll_d  = (roll_rate_error - last_roll_rate_error) / dt;
    last_pitch_d = (0.7 * last_pitch_d) + (0.3 * current_pitch_d);
    last_roll_d  = (0.7 * last_roll_d)  + (0.3 * current_roll_d);

    float pitch_output = (Kp_rate * pitch_rate_error) + (Ki_rate * pitch_rate_int) + (Kd_rate * last_pitch_d); 
    float roll_output  = (Kp_rate * roll_rate_error)  + (Ki_rate * roll_rate_int)  + (Kd_rate * last_roll_d); 
    float yaw_output   = (Kp_yaw * yaw_rate_error); 

    last_pitch_rate_error = pitch_rate_error; 
    last_roll_rate_error = roll_rate_error; 

    pitch_output = constrain(pitch_output, -PID_LIMIT, PID_LIMIT); 
    roll_output  = constrain(roll_output, -PID_LIMIT, PID_LIMIT); 
    yaw_output   = constrain(yaw_output, -PID_LIMIT, PID_LIMIT);

    // --- 4. MIKSOWANIE ---
    int m_fl = manual_throttle - pitch_output + roll_output + yaw_output;  
    int m_fr = manual_throttle - pitch_output - roll_output - yaw_output;  
    int m_br = manual_throttle + pitch_output - roll_output + yaw_output;  
    int m_bl = manual_throttle + pitch_output + roll_output - yaw_output;  

    setMotors(m_fl, m_fr, m_br, m_bl); 
    
  } else {
    setMotors(0, 0, 0, 0);
    pitch_rate_int = 0;
    roll_rate_int = 0;
  }

  // --- 5. LOGI ---
  if (millis() - last_print_time > 100) {
    last_print_time = millis();
    int time_left = (FLIGHT_TIME_MS - (millis() - flight_start_time)) / 1000;
    
    Serial.print("T-Minus: "); Serial.print(time_left); Serial.print("s");
    Serial.print(" | THR: "); Serial.print(manual_throttle); 
    Serial.print(" | AngP: "); Serial.print(angle_pitch, 1); 
    Serial.print(" AngR: "); Serial.println(angle_roll, 1); 
  }
}
