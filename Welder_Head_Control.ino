/*
  WelderHead Control

  v0.3 - Implement AutoPID & NTC Thermistor
  V0.2 - New Display, PID with BangBang control,
  V0.1 - PID control two heater 12V DC -> Max 180°C
*/

float version = 0.3;

// ------------------ Thermistor  ------------------
// ------------------ Thermistor  ------------------
#include "AnalogPin.h"
#include <Thermistor.h>
#include <NTC_Thermistor.h>
#include <AverageThermistor.h>

#define THERMISTOR_PIN         A7
#define REFERENCE_RESISTANCE   100000
#define NOMINAL_RESISTANCE     100000
#define NOMINAL_TEMPERATURE    25
#define B_VALUE                3950

#define READINGS_NUMBER 5
#define DELAY_TIME 20

Thermistor* thermistor = NULL;
double Tc;

// ------------------ PID  ------------------
// ------------------ PID  ------------------
#include <AutoPID.h>
#define Heater 5
#define HeaterLed 10

AnalogPin INPoti(A2);
int valuePoti;
uint32_t val;
double Setpoint, Output;

#define BANGBANG 4
#define OUTPUT_MIN 0     // 0
#define OUTPUT_MAX 255   // 255
#define KP 22.2          // 22.2
#define KI 1.08          // 1.08
#define KD 114.0         // 114.0

//PID myPID(&Tc, &Output, &Setpoint, 22.2, 1.08, 114, DIRECT);
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
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

int minTemp = 194;
int maxTemp = 250;

unsigned long prevMillis;
int refreshTime = 100; // Display refresh time in millisecond

// ------------------ Buttons ------------------
// ------------------ Buttons ------------------
#define ENTER_BUTTON 2
bool startState = false;
unsigned long prevMillisButton;
int ButtonValue;
bool lastState = true;

// ------------------ VOID LIST ------------------
// ------------------ VOID LIST ------------------
void sensButton();
//void readTemp();
void displayHEAT();
void displayValues();

// ---------- SETUP ---------- SETUP ---------- SETUP ---------- SETUP ----------
// ---------- SETUP ---------- SETUP ---------- SETUP ---------- SETUP ----------
void setup() {
  Serial.begin(115200);
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
  INPoti.setNoiseThreshold(5);
  valuePoti = INPoti.read();
  Setpoint = map(valuePoti, 0, 1023, minTemp, maxTemp);

  myPID.setBangBang(BANGBANG);
  myPID.setTimeStep(20);

  pinMode(HeaterLed, OUTPUT);
  pinMode(ENTER_BUTTON, INPUT_PULLUP);

  // // --------------- Oled  ---------------
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D or 0x3C for 128x64
    Serial.println(F("-- SSD1306 allocation failed --"));
    for (;;);
  }

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
  display.setCursor(8, 52);
  display.println("v" + String(version));
  display.setCursor(60, 52);
  display.println("Pr. Enter");
  display.display();

  //  while (digitalRead(ENTER_BUTTON) == HIGH) {
  //  }
  delay(500);
}

// ---------- LOOP ---------- LOOP ---------- LOOP ---------- LOOP ----------
// ---------- LOOP ---------- LOOP ---------- LOOP ---------- LOOP ----------
void loop() {
  // Sens button to Start heating 
  sensButton();
  Tc = thermistor->readCelsius();

  // Read Poti
  val = 0;
  int sampleNumPot = 2;
  INPoti.setNoiseThreshold(5);
  for (int i = 0; i < sampleNumPot; i++) {
    val += INPoti.read();
  }
  valuePoti = int(val / sampleNumPot);
  //valuePoti = INPoti.read();
  Setpoint = map(valuePoti, 0, 1023, minTemp, maxTemp);
  if (Setpoint <= minTemp) {
    Setpoint = 0;
  }

  // Display Values
  if (millis() - prevMillis >= refreshTime) {
    displayValues();
    prevMillis = millis();
  }

  // Start Heateing
  if (startState == true) {
    myPID.run();
    analogWrite(Heater, Output);
    //digitalWrite(HeaterLed, Output);
    digitalWrite(HeaterLed, myPID.atSetPoint(4));
    displayHEAT();
    //Serial.println("Output: " + (String)Output);
  }
  else if (startState == false) {
    analogWrite(Heater, 0);
    analogWrite(HeaterLed, 0);
  }

}
// ---------- End LOOP ---------- End LOOP ---------- End LOOP ---------- End LOOP ----------
// ---------- End LOOP ---------- End LOOP ---------- End LOOP ---------- End LOOP ----------

// ---------- Sens Button ---------- Sens Button ---------- Sens Button ----------
// ---------- Sens Button ---------- Sens Button ---------- Sens Button ----------
void sensButton() {
  ButtonValue = digitalRead(ENTER_BUTTON);
  if (lastState == true) {
    if (ButtonValue == LOW && millis() - prevMillisButton > 50) {
      startState = !startState;
      delay(300);
      lastState == false;
    }
  }
  if (ButtonValue == HIGH) {
    lastState == true;
    prevMillisButton = millis();
  }
}

// ---------- Display HEAT ---------- Display HEAT ---------- Display HEAT ----------
// ---------- Display HEAT ---------- Display HEAT ---------- Display HEAT ----------
void displayHEAT() {

  // Display Text and Icon
  double gap = abs(Setpoint - Tc);
  if (gap < 10)
  {
    //myPID.setGains(20.1, 1.0, 100.0);
    display.fillCircle(54, 38, 3, SSD1306_WHITE);
    display.setCursor(4, 35);
    display.println("Heating");
  }
  else
  {
    //myPID.setGains(22.2, 1.08, 114.0);
    display.drawCircle(54, 38, 3, SSD1306_WHITE);
    display.setCursor(4, 35);
    display.println("Heating");
  }
}

// ---------- Display Values ---------- Display Values ---------- Display Values ----------
// ---------- Display Values ---------- Display Values ---------- Display Values ----------
void displayValues() {

  display.setTextSize(1);
  display.setCursor(5, 20);
  display.println("Temp:");
  display.setCursor(62, 20);
  display.println(Tc);
  display.setCursor(110, 20);
  display.println("C");
  display.drawCircle(105, 20, 2, SSD1306_WHITE);

  display.setCursor(5, 5);
  display.println("SetPoint:");
  display.setCursor(62, 5);
  display.println(Setpoint);
  display.setCursor(110, 5);
  display.println("C");
  display.drawCircle(105, 5, 2, SSD1306_WHITE);

  // Display Values
  display.drawLine(0, 0, 127, 0, SSD1306_WHITE);
  display.drawLine(0, 15, 127, 15, SSD1306_WHITE);
  display.drawLine(0, 30, 127, 30, SSD1306_WHITE);
  display.drawRect(0, 0, 128, 64, SSD1306_WHITE);
  display.drawLine(0, 45, 127, 45, SSD1306_WHITE);
  int n = 1;
  for (int i = 20; i < 120; i += 20) {
    display.drawLine(i, 45, i, 63, SSD1306_WHITE);
    display.setCursor(i - 12, 51);
    display.println(n);
    n++;
  }
  display.fillRect(102, 47, 24, 15, SSD1306_WHITE);
  display.setCursor(108, 51);
  display.setTextColor(BLACK);
  display.println("->");

  display.setTextColor(WHITE);
  display.drawLine(60, 30, 60, 45, SSD1306_WHITE);
  display.setCursor(63, 35);
  display.println("F 2.5m/min");

  display.setCursor(5, 5);
  display.println("SetPoint:");
  display.setCursor(62, 5);
  display.println(Setpoint);
  display.setCursor(110, 5);
  display.println("C");
  display.drawCircle(105, 5, 2, SSD1306_WHITE);

  display.display();
  display.clearDisplay();
}
