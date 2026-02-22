#include <Arduino.h>
#include <LiquidCrystal.h>
#include <math.h>

// Structure used to implement software debouncing for push-buttons
struct DebouncedButton {
  uint8_t pin;            // Arduino pin the button is connected to
  bool lastStable;        // Last debounced stable logic level
  bool lastReading;       // Last instantaneous reading
  uint32_t lastChangeMs;  // Time of last state change (for debounce timing)
};

// Speed staging enum used for selecting motor PWM level
enum SpeedClass : uint8_t { SPEED_LOW = 0, SPEED_MED = 1, SPEED_HIGH = 2 };

// Forward declaration of button edge detection function
bool buttonPressedEdge(DebouncedButton &b);

// Encoder A and B channel input pins
const uint8_t PIN_ENC_A   = 2;
const uint8_t PIN_ENC_B   = 4;

// H-bridge motor driver pins
const uint8_t PIN_PWM     = 5; // PWM speed control
const uint8_t PIN_INB     = 6; // Direction input B
const uint8_t PIN_INA     = 7; // Direction input A

// User interface push-buttons
const uint8_t PIN_BTN_EXEC = 8;  // Start/Stop motor
const uint8_t PIN_BTN_SPD  = 9;  // Cycle speed
const uint8_t PIN_BTN_DIR  = 10; // Toggle direction

// LCD pin definitions
const uint8_t LCD_RS = 12;
const uint8_t LCD_E  = 11;
const uint8_t LCD_D4 = A0;
const uint8_t LCD_D5 = A1;
const uint8_t LCD_D6 = A2;
const uint8_t LCD_D7 = A3;

// Create LCD object (16x2 parallel interface)
LiquidCrystal lcd(LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

// Encoder counts per revolution (gearbox output shaft)
const int32_t CPR_GEARBOX = 700;

// Using x4 quadrature decoding → effective counts per rev
const int32_t COUNTS_PER_REV_GEARBOX = CPR_GEARBOX * 4;

// RPM sampling interval in microseconds (50 ms)
const uint32_t SAMPLE_US = 50000;

// Low-pass filter constant for angular velocity smoothing
const float OMEGA_ALPHA = 0.25f;

// Encoder count updated inside interrupt
volatile int32_t encCount = 0;


// Quadrature decoding lookup table
// Maps previous AB state and current AB state to count increment/decrement
static const int8_t QUAD_TABLE[16] = {
  0, -1, +1,  0,
 +1,  0,  0, -1,
 -1,  0,  0, +1,
  0, +1, -1,  0
};

// Previous encoder A/B state
volatile uint8_t prevAB = 0;


// Reads encoder A and B channel logic levels directly from PORTD register
inline uint8_t readAB() {
  uint8_t p = PIND;
  uint8_t a = (p >> PD2) & 0x01;
  uint8_t b = (p >> PD4) & 0x01;
  return (uint8_t)((a << 1) | b);
}

// Pin Change Interrupt Service Routine for encoder channels
ISR(PCINT2_vect) {
  uint8_t currAB = readAB();
  uint8_t idx = (uint8_t)((prevAB << 2) | currAB);
  encCount += QUAD_TABLE[idx];
  prevAB = currAB;
}

// Configure pin change interrupts for encoder pins
void setupEncoderPCINT() {
  pinMode(PIN_ENC_A, INPUT);
  pinMode(PIN_ENC_B, INPUT);

  prevAB = readAB();

  PCICR |= (1 << PCIE2);

  PCMSK2 |= (1 << PCINT18);
  PCMSK2 |= (1 << PCINT20);
}

// Safely read encoder count atomically
int32_t getEncCountAtomic() {
  int32_t c;
  noInterrupts();
  c = encCount;
  interrupts();
  return c;
}


// Current applied PWM command
uint8_t pwmCmd = 0;

// Current applied direction
bool dirForward = true;


// Desired staged settings from UI
bool desiredForward = true;
SpeedClass desiredSpeed = SPEED_LOW;
bool motorOn = false;

// PWM values for speed classes
const uint8_t PWM_LOW  = 50;
const uint8_t PWM_MED  = 80;
const uint8_t PWM_HIGH = 120;

// Returns PWM value based on selected speed class
uint8_t pwmFromClass(SpeedClass s) {
  switch (s) {
    case SPEED_LOW:  return PWM_LOW;
    case SPEED_MED:  return PWM_MED;
    default:         return PWM_HIGH;
  }
}

// Shaft diameter in meters (used for length estimation)
const float SHAFT_DIAMETER_M = 0.026f;

// Shaft circumference
const float SHAFT_CIRCUM_M   = PI * SHAFT_DIAMETER_M;

// Encoder count at motor start
int32_t onStartCount = 0;

// Total measured length moved
float totalLengthM = 0.0f;

// LCD refresh rate for length display
const uint32_t LCD_LEN_UPDATE_MS = 200;
uint32_t lastLcdLenUpdateMs = 0;

// Debounce timing
const uint32_t DEBOUNCE_MS = 30;

// Debounced button objects
DebouncedButton btnDir  {PIN_BTN_DIR,  false, false, 0};
DebouncedButton btnSpd  {PIN_BTN_SPD,  false, false, 0};
DebouncedButton btnExec {PIN_BTN_EXEC, false, false, 0};

// Rising-edge detection with debounce
bool buttonPressedEdge(DebouncedButton &b) {
  bool reading = digitalRead(b.pin);
  uint32_t now = millis();

  if (reading != b.lastReading) {
    b.lastReading = reading;
    b.lastChangeMs = now;
  }

  if ((now - b.lastChangeMs) > DEBOUNCE_MS && reading != b.lastStable) {
    b.lastStable = reading;
    if (b.lastStable == HIGH) return true;
  }
  return false;
}


// Apply motor direction and PWM command
void applyMotorCommand() {
  if (dirForward) {
    digitalWrite(PIN_INA, HIGH);
    digitalWrite(PIN_INB, LOW);
  } else {
    digitalWrite(PIN_INA, LOW);
    digitalWrite(PIN_INB, HIGH);
  }
  analogWrite(PIN_PWM, pwmCmd);
}

// Immediately stop motor
void stopMotor() {
  pwmCmd = 0;
  analogWrite(PIN_PWM, 0);
}

// Apply staged settings and start motor
void applyStagedSettingsAndStart() {

  dirForward = desiredForward;
  pwmCmd = pwmFromClass(desiredSpeed);

  analogWrite(PIN_PWM, 0);
  delay(20);

  applyMotorCommand();
}


// Returns string representation of speed class
const char* speedStr(SpeedClass s) {
  switch (s) {
    case SPEED_LOW:  return "LOW";
    case SPEED_MED:  return "MED";
    default:         return "HIGH";
  }
}

// Update LCD line 1 with direction, speed and motor state
void lcdPrintLine1() {
  lcd.setCursor(0, 0);

  lcd.print(desiredForward ? "IN" : "OUT");
  lcd.print(' ');


  lcd.print(' ');

  lcd.print(speedStr(desiredSpeed));
  lcd.print(' ');

  if (desiredSpeed != SPEED_HIGH) lcd.print(' ');

  lcd.print(motorOn ? "ON" : "OFF");

  lcd.print("       ");
}

// Update LCD line 2 with measured length
void lcdPrintLine2Len(float meters) {
  lcd.setCursor(0, 1);
  lcd.print("LEN:");
  lcd.print(meters, 3);
  lcd.print("m");
  lcd.print("        ");
}

// Arduino setup routine
void setup() {
  Serial.begin(115200);


  pinMode(PIN_PWM, OUTPUT);
  pinMode(PIN_INA, OUTPUT);
  pinMode(PIN_INB, OUTPUT);

  pinMode(PIN_BTN_DIR,  INPUT);
  pinMode(PIN_BTN_SPD,  INPUT);
  pinMode(PIN_BTN_EXEC, INPUT);


  setupEncoderPCINT();


  lcd.begin(16, 2);
  lcd.clear();


  motorOn = false;
  desiredForward = true;
  desiredSpeed = SPEED_LOW;
  dirForward = desiredForward;

  stopMotor();

  onStartCount = getEncCountAtomic();
  totalLengthM = 0.0f;

  lcdPrintLine1();
  lcdPrintLine2Len(totalLengthM);

  Serial.println("Motor control + LCD UI ready.");
  Serial.print("Counts/rev (gearbox, x4) = ");
  Serial.println(COUNTS_PER_REV_GEARBOX);
  Serial.print("Shaft circumference (m) = ");
  Serial.println(SHAFT_CIRCUM_M, 6);
}

// Main loop
void loop() {
  if (buttonPressedEdge(btnDir)) {
    desiredForward = !desiredForward;
    lcdPrintLine1();
  }

  if (buttonPressedEdge(btnSpd)) {
    desiredSpeed = (SpeedClass)((desiredSpeed + 1) % 3);
    lcdPrintLine1();
  }

  if (buttonPressedEdge(btnExec)) {
    motorOn = !motorOn;

    if (motorOn) {
      onStartCount = getEncCountAtomic();
      totalLengthM = 0.0f;
      lastLcdLenUpdateMs = 0;

      applyStagedSettingsAndStart();

      lcdPrintLine2Len(totalLengthM);
    } else {
      int32_t offCount = getEncCountAtomic();
      int32_t dCounts = offCount - onStartCount;
      float rotations = fabs((float)dCounts) / (float)COUNTS_PER_REV_GEARBOX;
      totalLengthM = rotations * SHAFT_CIRCUM_M;

      stopMotor();

      lcdPrintLine2Len(totalLengthM);

      Serial.print("SESSION len_m=");
      Serial.println(totalLengthM, 6);
    }

    lcdPrintLine1();
  }

  if (motorOn) {
    uint32_t nowMs = millis();
    if (nowMs - lastLcdLenUpdateMs >= LCD_LEN_UPDATE_MS) {
      lastLcdLenUpdateMs = nowMs;

      int32_t countNow = getEncCountAtomic();
      int32_t dCounts = countNow - onStartCount;
      float rotations = fabs((float)dCounts) / (float)COUNTS_PER_REV_GEARBOX;
      totalLengthM = rotations * SHAFT_CIRCUM_M;

      lcdPrintLine2Len(totalLengthM);
    }
  }


  static uint32_t lastUs = micros();
  static int32_t  lastCount = 0;
  static float    omegaFilt = 0.0f;

  uint32_t nowUs = micros();
  uint32_t dtUs = nowUs - lastUs;

  if (dtUs >= SAMPLE_US) {
    lastUs = nowUs;

    int32_t countCopy = getEncCountAtomic();
    int32_t dCount = countCopy - lastCount;
    lastCount = countCopy;

    float dtSec = (float)dtUs * 1e-6f;
    float omega = ((float)dCount / (float)COUNTS_PER_REV_GEARBOX) * (TWO_PI / dtSec);
    omegaFilt = (OMEGA_ALPHA <= 0.0f) ? omega : (omegaFilt + OMEGA_ALPHA * (omega - omegaFilt));
    float rpm = omegaFilt * (60.0f / TWO_PI);

    Serial.print("rpm=");
    Serial.print(rpm, 2);
    Serial.print(" pwm=");
    Serial.print(pwmCmd);
    Serial.print(" dir=");
    Serial.print(dirForward ? "IN" : "OUT");
    Serial.print(" staged=");
    Serial.print(desiredForward ? "IN" : "OUT");
    Serial.print(" spd=");
    Serial.print(speedStr(desiredSpeed));
    Serial.print(" motorOn=");
    Serial.print(motorOn ? "ON" : "OFF");
    Serial.print(" len_m=");
    Serial.print(totalLengthM, 4);
    Serial.print(" count=");
    Serial.print(countCopy);
    Serial.print(" dCount=");
    Serial.println(dCount);
  }
}
