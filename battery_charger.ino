#include <PID_v1.h> 
// PID control variables 
double ccSetpoint, ccInput, ccOutput; 
double cvSetpoint, cvInput, cvOutput; 
PID ccPID(&ccInput, &ccOutput, &ccSetpoint, 2.0, 5.0, 1.0, DIRECT); 
PID cvPID(&cvInput, &cvOutput, &cvSetpoint, 2.0, 5.0, 1.0, DIRECT); 
enum ChargingState { 
TRICKLE, 
CC, 
PULSE, 
CV, 
DONE 
}; 
ChargingState chargingState = TRICKLE; 
const int pwmPin = 25; 
const int currentPin = 34; 
const int voltagePin = 35; 
const int tempPin = 32; 
unsigned long pulseStartTime = 0; 
bool pulseHigh = true; 
const float trickleCurrent = 1.0; 
const float ccCurrent = 4.0; 
const float pulseCurrent = 0.5; 
const float cvVoltage = 4.2; 
const float doneCurrent = 0.05; 
float readCurrent() { 
long sum = 0; 
for (int i = 0; i < 100; i++) { 
sum += analogRead(currentPin); 
delayMicroseconds(100); 
} 
int adc = sum / 100; 
float voltage = adc * (3.3 / 4095.0); 
float offsetVoltage = 3.3 / 2.357; 
float current = ((voltage - offsetVoltage) / 0.185) - 0.19; 
return current; 
} 
float readVoltage() { 
int adc = analogRead(voltagePin); 
float raw = (adc * (5.0 / 4095.0)) + 0.1; 
return raw; 
} 
int controlCharging(float vbat, float ibat) { 
int pwmValue = 0; 
if (ibat >= 0.4 && vbat >= 4.25) { 
// CV mode (PID7) 
cvSetpoint = cvVoltage; 
cvInput = vbat; 
cvPID.Compute(); 
pwmValue = cvOutput; 
chargingState = CV; 
if (ibat <= doneCurrent) chargingState = DONE; 
  } else if (vbat >= 3.54) { 
    // PULSE mode (PID6) - 100% duty with 10kHz switching 
    chargingState = PULSE; 
    if (millis() - pulseStartTime > 1000) { 
      pulseHigh = !pulseHigh; 
      pulseStartTime = millis(); 
    } 
    if (pulseHigh) { 
      ledcSetup(0, 10000, 8);  // 10kHz frequency 
      ledcAttachPin(pwmPin, 0); 
      pwmValue = 255; 
    } else { 
      pwmValue = 0; 
    } 
 
  } else if (vbat >= 3.39) { 
    // Constant Current mode (PID5) - 50% duty 
    chargingState = CC; 
    pwmValue = 128; 
 
  } else { 
    // Trickle mode (PID4) - 25% duty 
    chargingState = TRICKLE; 
    pwmValue = 64; 
  } 
 
  if (chargingState == DONE) { 
    pwmValue = 0; 
  } 
 
  return constrain(pwmValue, 0, 255); 
} 
 
void setup() { 
  Serial.begin(115200); 
  pinMode(pwmPin, OUTPUT); 
  ledcSetup(0, 5000, 8);         // Default: 5kHz, 8-bit 
  ledcAttachPin(pwmPin, 0);     // PWM channel 0 
 
  ccPID.SetMode(AUTOMATIC); 
  ccPID.SetOutputLimits(0, 255); 
  cvPID.SetMode(AUTOMATIC); 
  cvPID.SetOutputLimits(0, 255); 
 
  Serial.println("Battery Charging Controller Started"); 
} 
 
void loop() { 
  float vbat = readVoltage(); 
  float ibat = readCurrent(); 
 
  int pwmValue = controlCharging(vbat, ibat); 
  ledcWrite(0, pwmValue); 
 
  Serial.print("Vbat: "); Serial.print(vbat); Serial.print(" V, "); 
  Serial.print("Ibat: "); Serial.print(ibat); Serial.print(" A, "); 
  Serial.print("PWM: "); Serial.print(pwmValue); Serial.print(", "); 
  Serial.print("State: "); 
 
  switch (chargingState) { 
    case TRICKLE: Serial.println("Trickle (25% duty)"); break; 
    case CC:      Serial.println("Constant Current (50% duty)"); break; 
    case PULSE:   Serial.println("Pulse (100% duty @10kHz)"); break; 
    case CV:      Serial.println("Constant Voltage (PID-controlled)"); break; 
    case DONE:    Serial.println("Done"); break; 
  }
