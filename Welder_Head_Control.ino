/*
  WelderHead Control

  v0.91 - 4 butoon -> Start Heat; Select Sinergy, Forward Filament; Backward Filament
  v0.9 - Stepper + sinergy select implement with Timer interrupt
  v0.8 - Start signal to robot,
  v0.7 - Implement to Nano Every, remove Analogpin.h
  v0.6 - Select Synergy implement. 1-5 and Manual mode
  v0.5 - PID Now hold stable Temp +-1°C
  v0.4 - 10k Reference Resistor, New P I D calc
  v0.3 - Implement AutoPID & NTC Thermistor
  V0.2 - New Display, PID with BangBang control,
  V0.1 - PID control two heater 12V DC -> Max 180°C
*/

float version = 0.91;

// ------------------ Thermistor  ------------------
// ------------------ Thermistor  ------------------
#include <Thermistor.h>
#include <NTC_Thermistor.h>
#include <AverageThermistor.h>

#define THERMISTOR_PIN         A7
#define REFERENCE_RESISTANCE   10000
#define NOMINAL_RESISTANCE     100000
#define NOMINAL_TEMPERATURE    25
#define B_VALUE                3950  // 3950

#define READINGS_NUMBER 5
#define DELAY_TIME 10

Thermistor* thermistor = NULL;
double Tc;

// ------------------ PID  ------------------
// ------------------ PID  ------------------
#include <AutoPID.h>
#define Heater 5
#define HeaterLed 10

#define SETPOINT_PIN A0  // Setpoint Potentiometer
int valuePoti;
uint32_t val;
double Setpoint, Output;
bool ResetPid = false;
#define resetCut 15  // Reset PID +-18°C
long startMillis;
long endMillis;
int Interval = 10000; // When reach Setpoint Temp, and keep
int StartRobotFlag = 0;

#define BANGBANG 1
#define OUTPUT_MIN 0    // 0
#define OUTPUT_MAX 255  // 255
#define KP 8.88         // 8.0;   5.0;  15.0;
#define KI 0.3          // 0.22;  0.22;  0.3;
#define KD 0.1          // 0.1;   0.1;   0.0;

//myPID.setGains(22.2, 1.08, 114.0);
AutoPID myPID(&Tc, &Setpoint, &Output, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);

// ------------------ I2C Oled ------------------
// ------------------ I2C Oled ------------------
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define minTemp 194
#define maxTemp 250

unsigned long prevMillis;
#define refreshTime 30  // Display refresh time in millisecond

// ------------------ Stepper ------------------
// ------------------ Stepper ------------------
// Cylinder Diameter = 10.6mm
// Cylinder circumference = 33.33mm
// 1 m/min = 16,66 mm/s
// 0.5 rev/sec = 1 m/min

// 200 step/rev
// 100 step/sec = 1 m/min
// 1000 millisec / 200<-(100on 100off) = 5 millisec
#include <TimerOne.h>

#define STEP_PIN PD6        // Pin 6 - 3 Green
#define DIR_PIN 7           // Pin 7 - 2 Yellow
// ----- Choose a mode
//#define STEPP_MODE 200
#define STEPP_MODE 400
//#define STEPP_MODE 800
//#define STEPP_MODE 1600

int freqTime = 1000000 / STEPP_MODE;
//int freqTime =  2500;    // Microsecond -> 1.0m/min
// 2.2m/min = 1136
// 0.8m/min = 3125
#define MIN_FEED 0.8
#define MAX_FEED 2.0
float feedRate = 1.8;
int setTime = freqTime / feedRate;
bool setTimerFlag = true;

// ------------------ Buttons ------------------
// ------------------ Buttons ------------------
#define ENTER_BUTTON 2
bool startState = false;
int ButtonState;
int lastButtonState;
#define MENU_BUTTON 3
int MenuButtonState;
int lastMenuButtonState;
int MenuValue = 6;
#define FORWARD_BUTTON 4
#define BACKWARD_BUTTON 8

// ------------------ VOID LIST ------------------
// ------------------ VOID LIST ------------------
void sensButton();
void sensMenuButton();
void readSetpoint();
void displayHEAT();
void displayValues();
void startToRobot();
void moveFilament();
void checkDir();
void setTimer(); // Set Stepper Motor speed

// ---------- SETUP ---------- SETUP ---------- SETUP ---------- SETUP ----------
// ---------- SETUP ---------- SETUP ---------- SETUP ---------- SETUP ----------
void setup() {
  Serial.begin(115200);
  delay(5);
  // --------------- Oled  ---------------
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) { // Address 0x3D or 0x3C for 128x64
    Serial.println(F("-- SSD1306 allocation failed --"));
    for (;;);
  }

  // Button
  pinMode(ENTER_BUTTON, INPUT_PULLUP);
  pinMode(MENU_BUTTON, INPUT_PULLUP);
  lastButtonState = digitalRead(ENTER_BUTTON);
  lastMenuButtonState = digitalRead(MENU_BUTTON);
  pinMode(FORWARD_BUTTON, INPUT_PULLUP);
  pinMode(BACKWARD_BUTTON, INPUT_PULLUP);

  analogReference(EXTERNAL);

  Thermistor* originThermistor = new NTC_Thermistor(
    THERMISTOR_PIN,
    REFERENCE_RESISTANCE,
    NOMINAL_RESISTANCE,
    NOMINAL_TEMPERATURE,
    B_VALUE
  );
  thermistor = new AverageThermistor(
    originThermistor,
    READINGS_NUMBER,
    DELAY_TIME
  );

  // ------------------ PID  ------------------
  //INPoti.setNoiseThreshold(5);
  //valuePoti = INPoti.read();
  pinMode(SETPOINT_PIN, INPUT);
  valuePoti = analogRead(SETPOINT_PIN);
  Setpoint = map(valuePoti, 0, 1023, minTemp, maxTemp);

  //myPID.setBangBang(BANGBANG);
  myPID.setTimeStep(22);
  //delay(5);

  pinMode(HeaterLed, OUTPUT);

  // ------------------ STEPPER  ------------------
  DDRD |= (1 << STEP_PIN); // pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  Timer1.initialize(setTime);
  Timer1.attachInterrupt(moveFilament);

  // ------------------ INITIAL DISPLAY  ------------------
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(30, 0);
  display.println("Welder");
  display.setCursor(42, 17);
  display.println("Head");
  display.setCursor(26, 34);
  display.println("Control");
  display.setTextSize(1);
  display.setCursor(50, 52);
  display.println("v" + String(version));
  //  display.setCursor(60, 52);
  //  display.println("Pr. Enter");
  display.display();

  //  while (digitalRead(ENTER_BUTTON) == HIGH) {
  //  }
  delay(2000);
  //Serial.println("Setup done....");
  display.clearDisplay();
}

// ---------- LOOP ---------- LOOP ---------- LOOP ---------- LOOP ----------
// ---------- LOOP ---------- LOOP ---------- LOOP ---------- LOOP ----------
void loop() {
  // ---------- Sens button, Read Temp, Read Setpoint
  sensButton();
  sensMenuButton();
  checkDir();
  Tc = thermistor->readCelsius();
  if (MenuValue == 6) {
    readSetpoint();
  }

  // ---------- Refresh Display
  if (millis() - prevMillis >= refreshTime) {
    // 54 millisecundum the display
    displayValues();
    prevMillis = millis();

    // ---------- Serial Print measured values
    //    Serial.print(Setpoint);
    //    Serial.print(",");
    //    Serial.print(Setpoint - resetCut);
    //    Serial.print(",");
    //    Serial.print(Setpoint + resetCut);
    //    Serial.print(",");
    //    Serial.println(Tc);
  }

  // ---------- Start Heateing
  if ( startState == true ) {

    if (myPID.atSetPoint(resetCut) && ResetPid == true) {
      myPID.reset();
      ResetPid = false;
    }

    myPID.run();
    displayHEAT();

    if ( Tc > Setpoint + 10 ) {
      //    if ( Tc > maxTemp + 15 ) {
      analogWrite(Heater, 0);  // ---------- EMERGENCY STOP ----------
    }
    else {
      analogWrite(Heater, Output);
    }
    startToRobot();
  }
  else if (startState == false) {
    analogWrite(Heater, 0);
    analogWrite(HeaterLed, 0);
  }

}
// ---------- End LOOP ---------- End LOOP ---------- End LOOP ---------- End LOOP ----------
// ---------- End LOOP ---------- End LOOP ---------- End LOOP ---------- End LOOP ----------

// ---------- Read Setpoint ---------- Read Setpoint ---------- Read Setpoint ----------
// ---------- Read Setpoint ---------- Read Setpoint ---------- Read Setpoint ----------
void readSetpoint() {
  // Read Pot
  val = 0;
  int sampleNumPot = 4;
  for (int i = 0; i < sampleNumPot; i++) {
    val += analogRead(SETPOINT_PIN);
  }
  valuePoti = int(val / sampleNumPot);
  Setpoint = map(valuePoti, 0, 1023, minTemp, maxTemp);
  if (Setpoint <= minTemp) {
    Setpoint = 0;
  }
}

// ---------- Sens Button ---------- Sens Button ---------- Sens Button ----------
// ---------- Sens Button ---------- Sens Button ---------- Sens Button ----------
void sensButton() {
  ButtonState = digitalRead(ENTER_BUTTON);
  if (ButtonState != lastButtonState)
  {
    if (ButtonState == LOW && Setpoint >= minTemp && Setpoint <= maxTemp)
    {
      startState = !startState;
      StartRobotFlag = 0;
      startMillis = millis();
      if (ResetPid == false) {
        ResetPid = true;
      }
      while (digitalRead(ENTER_BUTTON) != HIGH) {}
    }
    lastButtonState == ButtonState;
  }
}

// ---------- Sens Menu Button ---------- Sens Menu Button ---------- Sens Menu Button ----------
// ---------- Sens Menu Button ---------- Sens Menu Button ---------- Sens Menu Button ----------
void sensMenuButton() {
  MenuButtonState = digitalRead(MENU_BUTTON);
  if (MenuButtonState != lastMenuButtonState)
  {
    if (MenuButtonState == LOW && startState == false)
    {
      MenuValue++;
      setTimerFlag = true;
      if (MenuValue > 6) {
        MenuValue = 1;
      }
      while (digitalRead(MENU_BUTTON) != HIGH) {}
    }
    lastMenuButtonState == MenuButtonState;
  }
}

// ---------- Display HEAT ---------- Display HEAT ---------- Display HEAT ----------
// ---------- Display HEAT ---------- Display HEAT ---------- Display HEAT ----------
void displayHEAT() {
  // Display Text and Icon
  if (myPID.atSetPoint(2))
  {
    display.setTextColor(WHITE);
    display.fillCircle(54, 38, 3, SSD1306_WHITE);
    display.setCursor(4, 35);
    display.println("Heating");
  }
  else
  {
    //myPID.setGains(22.2, 1.08, 114.0);
    display.setTextColor(WHITE);
    display.drawCircle(54, 38, 3, SSD1306_WHITE);
    display.setCursor(4, 35);
    display.println("Heating");
  }
}

// ---------- Start to robot ---------- Start to robot ---------- Start to robot ---------- Start to robot
// ---------- Start to robot ---------- Start to robot ---------- Start to robot ---------- Start to robot
void startToRobot() {
  if (StartRobotFlag == 0) {
    endMillis = millis();
    if ( endMillis - startMillis >= Interval ) {
      StartRobotFlag = 1;
    }
  }
  if (StartRobotFlag == 1) {
    digitalWrite(HeaterLed, HIGH);  // Start signal to Robot
  }

  if (!myPID.atSetPoint(2)) {
    startMillis = endMillis;
  }
}

// ---------- Stepper forward / backward ---------- Stepper forward / backward ----------
// ---------- Stepper forward / backward ---------- Stepper forward / backward ----------
void moveFilament(void) {
  if ( !digitalRead(FORWARD_BUTTON) || !digitalRead(BACKWARD_BUTTON)) {
    PORTD ^= (1 << STEP_PIN);
  }
}

void checkDir(void) {
  if ( !digitalRead(FORWARD_BUTTON) && digitalRead(BACKWARD_BUTTON)) {
    digitalWrite(DIR_PIN, LOW);
  }
  else if ( !digitalRead(BACKWARD_BUTTON) && digitalRead(FORWARD_BUTTON)) {
    digitalWrite(DIR_PIN, HIGH);
  }
}

// ---------- Set Timer ---------- Set Timer ---------- Set Timer ----------
// ---------- Set Timer ---------- Set Timer ---------- Set Timer ----------
void setTimer(void) {
  if ( setTimerFlag == true ) {
    setTime = freqTime / feedRate;
    Timer1.initialize(setTime);
    setTimerFlag = false;
  }
}

// ---------- Display Values ---------- Display Values ---------- Display Values ----------
// ---------- Display Values ---------- Display Values ---------- Display Values ----------
void displayValues() {

  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(5, 20);
  display.println("Temp:");
  display.setCursor(67, 20);
  display.println(Tc);
  display.setCursor(115, 20);
  display.println("C");
  display.drawCircle(110, 20, 2, SSD1306_WHITE);

  display.setCursor(5, 5);
  display.println("SetPoint:");
  display.setCursor(67, 5);
  display.println(Setpoint);
  display.setCursor(115, 5);
  display.println("C");
  display.drawCircle(110, 5, 2, SSD1306_WHITE);

  // Display Values
  display.drawLine(0, 0, 127, 0, SSD1306_WHITE);
  display.drawLine(0, 15, 127, 15, SSD1306_WHITE);
  display.drawLine(0, 30, 127, 30, SSD1306_WHITE);
  display.drawRect(0, 0, 128, 64, SSD1306_WHITE);
  display.drawLine(0, 45, 127, 45, SSD1306_WHITE);

  display.drawLine(20, 45, 20, 63, SSD1306_WHITE);
  display.drawLine(40, 45, 40, 63, SSD1306_WHITE);
  display.drawLine(60, 45, 60, 63, SSD1306_WHITE);
  display.drawLine(80, 45, 80, 63, SSD1306_WHITE);
  display.drawLine(100, 45, 100, 63, SSD1306_WHITE);

  // Display Feed Rate
  display.drawLine(60, 30, 60, 45, SSD1306_WHITE);
  display.setCursor(63, 35);
  display.println("F     m/mi");
  display.setCursor(70, 35);
  display.println(feedRate);

  switch (MenuValue) {
    case 1:
      display.fillRect(2, 47, 17, 15, SSD1306_WHITE);
      display.setTextColor(BLACK);
      display.setCursor(20 - 12, 51);
      display.println(MenuValue);
      Setpoint = 195;
      feedRate = 0.8;
      display.setTextColor(WHITE);
      display.setCursor(40 - 12, 51);
      display.println("2");
      display.setCursor(60 - 12, 51);
      display.println("3");
      display.setCursor(80 - 12, 51);
      display.println("4");
      display.setCursor(100 - 12, 51);
      display.println("5");
      display.setCursor(112, 51);
      display.println("M");
      setTimer();
      break;
    case 2:
      display.fillRect(22, 47, 17, 15, SSD1306_WHITE);
      display.setTextColor(BLACK);
      display.setCursor(40 - 12, 51);
      display.println(MenuValue);
      Setpoint = 200;
      feedRate = 0.9;
      display.setTextColor(WHITE);
      display.setCursor(20 - 12, 51);
      display.println("1");
      display.setCursor(60 - 12, 51);
      display.println("3");
      display.setCursor(80 - 12, 51);
      display.println("4");
      display.setCursor(100 - 12, 51);
      display.println("5");
      display.setCursor(112, 51);
      display.println("M");
      setTimer();
      break;
    case 3:
      display.fillRect(42, 47, 17, 15, SSD1306_WHITE);
      display.setTextColor(BLACK);
      display.setCursor(60 - 12, 51);
      display.println(MenuValue);
      Setpoint = 205;
      feedRate = 1.0;
      display.setTextColor(WHITE);
      display.setCursor(20 - 12, 51);
      display.println("1");
      display.setCursor(40 - 12, 51);
      display.println("2");
      display.setCursor(80 - 12, 51);
      display.println("4");
      display.setCursor(100 - 12, 51);
      display.println("5");
      display.setCursor(112, 51);
      display.println("M");
      setTimer();
      break;
    case 4:
      display.fillRect(62, 47, 17, 15, SSD1306_WHITE);
      display.setTextColor(BLACK);
      display.setCursor(80 - 12, 51);
      display.println(MenuValue);
      Setpoint = 215;
      feedRate = 1.2;
      display.setTextColor(WHITE);
      display.setCursor(20 - 12, 51);
      display.println("1");
      display.setCursor(40 - 12, 51);
      display.println("2");
      display.setCursor(60 - 12, 51);
      display.println("3");
      display.setCursor(100 - 12, 51);
      display.println("5");
      display.setCursor(112, 51);
      display.println("M");
      setTimer();
      break;
    case 5:
      display.fillRect(82, 47, 17, 15, SSD1306_WHITE);
      display.setTextColor(BLACK);
      display.setCursor(100 - 12, 51);
      display.println(MenuValue);
      Setpoint = 235;
      feedRate = 2.0;
      display.setTextColor(WHITE);
      display.setCursor(20 - 12, 51);
      display.println("1");
      display.setCursor(40 - 12, 51);
      display.println("2");
      display.setCursor(60 - 12, 51);
      display.println("3");
      display.setCursor(80 - 12, 51);
      display.println("4");
      display.setCursor(112, 51);
      display.println("M");
      setTimer();
      break;
    case 6: // -------------------- Manual Mode
      display.fillRect(102, 47, 24, 15, SSD1306_WHITE);
      display.setTextColor(BLACK);
      display.setCursor(112, 51);
      display.println("M");
      display.setTextColor(WHITE);
      display.setCursor(20 - 12, 51);
      display.println("1");
      display.setCursor(40 - 12, 51);
      display.println("2");
      display.setCursor(60 - 12, 51);
      display.println("3");
      display.setCursor(80 - 12, 51);
      display.println("4");
      display.setCursor(100 - 12, 51);
      display.println("5");
      feedRate = 1.8;
      setTimer();
      break;
  }

  display.display();
  display.clearDisplay();
}
