/*
  WelderHead Control

  v0.3 - Implement AutoPID
  V0.2 - New Display, PID with BangBang control,
  V0.1 - PID control two heater 12V DC -> Max 180°C
*/

float version = 0.3;

#include "AnalogPin.h"

int ThermistorPin = A7;
int Vo;
float R1 = 10000;
float logR2, R2;
double T, Tc, Tf;
float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;

// ------------------ PID  ------------------
#include <AutoPID.h>
#define Heater 5
#define HeaterLed 10 

AnalogPin INPoti(A2);
int valuePoti;
uint32_t val;
double Setpoint, Output;

#define BangBang 4
#define OUTPUT_MIN 0
#define OUTPUT_MAX 255
#define KP 22.2
#define KI 1.08
#define KD 114.0

//PID myPID(&Tc, &Output, &Setpoint, 22.2, 1.08, 114, DIRECT);
AutoPID myPID(&Tc, &Setpoint, &Output, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);

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
int refreshTime = 20; // Display refresh time in millisecond

// Buttons
#define enterButton 2
bool startState = false;
unsigned long prevMillisButton;
int ButtonValue;
bool lastState = true;

void sensButton();
void readTemp();
void displayHEAT();
void displayValues();

// ---------- SETUP ---------- SETUP ---------- SETUP ---------- SETUP ----------
// ---------- SETUP ---------- SETUP ---------- SETUP ---------- SETUP ----------
void setup() {
  Serial.begin(115200);
  
  analogReference(EXTERNAL);
  pinMode(ThermistorPin, INPUT);
  
  // ------------------ PID  ------------------
  Vo = analogRead(ThermistorPin);
  R2 = R1 * (1023.0 / (float)Vo - 1.0);
  logR2 = log(R2);
  T = (1.0 / (c1 + c2 * logR2 + c3 * logR2 * logR2 * logR2));
  Tc = T - 273.15;

  //pinMode(Poti, INPUT);
  //INPoti.setPrescaler(5);
  INPoti.setNoiseThreshold(5);
  valuePoti = INPoti.read();
  Setpoint = map(valuePoti, 0, 1023, minTemp, maxTemp);

  myPID.setBangBang(BangBang);
  myPID.setTimeStep(100);

  pinMode(HeaterLed, OUTPUT);
  pinMode(enterButton, INPUT_PULLUP);

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

  //  while (digitalRead(enterButton) == HIGH) {
  //  }
  delay(500);
}

// ---------- LOOP ---------- LOOP ---------- LOOP ---------- LOOP ----------
// ---------- LOOP ---------- LOOP ---------- LOOP ---------- LOOP ----------
void loop() {
  //myPID.run();
  
  // Sens button to Start heating state
  sensButton();
  
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

  // Read Temperature and Display
  if (millis() - prevMillis >= refreshTime) {
    readTemp();
    displayValues();
    prevMillis = millis();
  }

  // Start Heateing
  if (startState == true) {
    readTemp();
    myPID.run();
    analogWrite(Heater, Output);
    digitalWrite(HeaterLed, myPID.atSetPoint(3));
    displayHEAT();
  }
  else if (startState == false) {
    readTemp();
    analogWrite(Heater, 0);
    analogWrite(HeaterLed, 0);
  }

}
// ---------- End LOOP ---------- End LOOP ---------- End LOOP ---------- End LOOP ----------
// ---------- End LOOP ---------- End LOOP ---------- End LOOP ---------- End LOOP ----------

// ---------- Sens Button ---------- Sens Button ---------- Sens Button ----------
// ---------- Sens Button ---------- Sens Button ---------- Sens Button ----------
void sensButton() {
  ButtonValue = digitalRead(enterButton);
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
// ---------- Read Temp ---------- Read Temp ---------- Read Temp ----------
// ---------- Read Temp ---------- Read Temp ---------- Read Temp ----------
void readTemp() {
//  Vo = analogRead(ThermistorPin);

  long valIn = 0;
  int sampleNum = 10;
  for (int i = 0; i < sampleNum; i++) {
    valIn = valIn + analogRead(ThermistorPin);
  }
  Vo = int(valIn / sampleNum);
  
  R2 = R1 * (1023.0 / (float)Vo - 1.0);
  logR2 = log(R2);
  T = (1.0 / (c1 + c2 * logR2 + c3 * logR2 * logR2 * logR2));
  Tc = T - 273.15;
}

// ---------- Control PID ---------- Control PID ---------- Control PID ----------
// ---------- Control PID ---------- Control PID ---------- Control PID ----------
void displayHEAT() {

  // Display Text and Icon
  double gap = abs(Setpoint - Tc);
  if (gap < 10)
  {
    //myPID.SetTunings(consKp, consKi, consKd);
    display.fillCircle(54, 38, 3, SSD1306_WHITE);
    display.setCursor(4, 35);
    display.println("Heating");
  }
  else
  {
    //myPID.SetTunings(aggKp, aggKi, aggKd);
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
  display.setTextSize(1);
  display.setCursor(62, 20);
  if(myPID.atSetPoint(3) == 1){
    Tc = Setpoint;
  }
  display.println(Tc);
  display.setCursor(110, 20);
  display.println("C");
  display.drawCircle(105, 20, 2, SSD1306_WHITE);
  
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
  display.setTextSize(1);
  display.setCursor(62, 5);
  //display.println((int)Setpoint);
  display.println(Setpoint);
  display.setCursor(110, 5);
  display.println("C");
  display.drawCircle(105, 5, 2, SSD1306_WHITE);

  display.display();
  display.clearDisplay();
}
