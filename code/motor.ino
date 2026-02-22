#include <Arduino.h>
#include <LiquidCrystal.h>
#include <math.h>

struct DebouncedButton {
  uint8_t pin;
  bool lastStable;
  bool lastReading;
  uint32_t lastChangeMs;
};

enum SpeedClass : uint8_t { SPEED_LOW = 0, SPEED_MED = 1, SPEED_HIGH = 2 };

bool buttonPressedEdge(DebouncedButton &b);

const uint8_t PIN_ENC_A   = 2;
const uint8_t PIN_ENC_B   = 4;

const uint8_t PIN_PWM     = 5;
const uint8_t PIN_INB     = 6;
const uint8_t PIN_INA     = 7;

const uint8_t PIN_BTN_EXEC = 8;
const uint8_t PIN_BTN_SPD  = 9;
const uint8_t PIN_BTN_DIR  = 10;

const uint8_t LCD_RS = 12;
const uint8_t LCD_E  = 11;
const uint8_t LCD_D4 = A0;
const uint8_t LCD_D5 = A1;
const uint8_t LCD_D6 = A2;
const uint8_t LCD_D7 = A3;

LiquidCrystal lcd(LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

const int32_t CPR_GEARBOX = 700;
const int32_t COUNTS_PER_REV_GEARBOX = CPR_GEARBOX * 4;

const uint32_t SAMPLE_US = 50000;
const float OMEGA_ALPHA = 0.25f;

volatile int32_t encCount = 0;


static const int8_t QUAD_TABLE[16] = {
  0, -1, +1,  0,
 +1,  0,  0, -1,
 -1,  0,  0, +1,
  0, +1, -1,  0
};

volatile uint8_t prevAB = 0;


inline uint8_t readAB() {
  uint8_t p = PIND;
  uint8_t a = (p >> PD2) & 0x01;
  uint8_t b = (p >> PD4) & 0x01;
  return (uint8_t)((a << 1) | b);
}

ISR(PCINT2_vect) {
  uint8_t currAB = readAB();
  uint8_t idx = (uint8_t)((prevAB << 2) | currAB);
  encCount += QUAD_TABLE[idx];
  prevAB = currAB;
}

void setupEncoderPCINT() {
  pinMode(PIN_ENC_A, INPUT);
  pinMode(PIN_ENC_B, INPUT);

  prevAB = readAB();

  PCICR |= (1 << PCIE2);

  PCMSK2 |= (1 << PCINT18);
  PCMSK2 |= (1 << PCINT20);
}

int32_t getEncCountAtomic() {
  int32_t c;
  noInterrupts();
  c = encCount;
  interrupts();
  return c;
}


uint8_t pwmCmd = 0;
bool dirForward = true;


bool desiredForward = true;
SpeedClass desiredSpeed = SPEED_LOW;
bool motorOn = false;

const uint8_t PWM_LOW  = 50;
const uint8_t PWM_MED  = 80;
const uint8_t PWM_HIGH = 120;

uint8_t pwmFromClass(SpeedClass s) {
  switch (s) {
    case SPEED_LOW:  return PWM_LOW;
    case SPEED_MED:  return PWM_MED;
    default:         return PWM_HIGH;
  }
}

const float SHAFT_DIAMETER_M = 0.026f;
const float SHAFT_CIRCUM_M   = PI * SHAFT_DIAMETER_M;

int32_t onStartCount = 0;  
float totalLengthM = 0.0f;

const uint32_t LCD_LEN_UPDATE_MS = 200;
uint32_t lastLcdLenUpdateMs = 0;

const uint32_t DEBOUNCE_MS = 30;

DebouncedButton btnDir  {PIN_BTN_DIR,  false, false, 0};
DebouncedButton btnSpd  {PIN_BTN_SPD,  false, false, 0};
DebouncedButton btnExec {PIN_BTN_EXEC, false, false, 0};

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

void stopMotor() {
  pwmCmd = 0;
  analogWrite(PIN_PWM, 0);
}

void applyStagedSettingsAndStart() {

  dirForward = desiredForward;
  pwmCmd = pwmFromClass(desiredSpeed);

  analogWrite(PIN_PWM, 0);
  delay(20);

  applyMotorCommand();
}


const char* speedStr(SpeedClass s) {
  switch (s) {
    case SPEED_LOW:  return "LOW";
    case SPEED_MED:  return "MED";
    default:         return "HIGH";
  }
}

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

void lcdPrintLine2Len(float meters) {
  lcd.setCursor(0, 1);
  lcd.print("LEN:");
  lcd.print(meters, 3);
  lcd.print("m");
  lcd.print("        ");
}

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
