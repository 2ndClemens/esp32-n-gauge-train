#include <Arduino.h>

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

#include <Adafruit_SSD1306.h>
#include <Servo.h>

#include <Adafruit_NeoPixel.h>

#define PIN 25

// When we setup the NeoPixel library, we tell it how many pixels, and which pin to use to send signals.
// Note that for older NeoPixel strips you might need to change the third parameter--see the strandtest
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(3, PIN, NEO_GRB + NEO_KHZ800);

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#define ACCELERATION_TIME 1000
#define DECELERATION_TIME 300
#define WAIT_TIME 1500

#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

#define CHARACTERISTIC_UUID_SENSOR_1_TX "6E400004-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_DIRECTION_TX "6E400005-B5A3-F393-E0A9-E50E24DCCA9E"

BLECharacteristic *pTxCharacteristicSensor1;
BLECharacteristic *pTxCharacteristicDirection;

Servo myservo1; // create servo object to control a servo
Servo myservo2; // create servo object to control a servo
Servo myservo3; // create servo object to control a servo
Servo myservo4; // create servo object to control a servo
Servo myservo5; // create servo object to control a servo
Servo myservo6; // create servo object to control a servo
// twelve servo objects can be created on most boards

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

volatile unsigned long DebounceTimer1 = millis();
volatile unsigned long DebounceTimer2 = millis();
volatile unsigned long DebounceTimer3 = millis();
volatile unsigned long DebounceTimer4 = millis();
volatile unsigned long DebounceTimer5 = millis();
volatile unsigned long DebounceTimer6 = millis();
volatile unsigned int delayTime = 1000;

uint32_t hallSensed1 = 0;
uint32_t hallSensed2 = 0;
uint32_t hallSensed3 = 0;
uint32_t hallSensed4 = 0;
uint32_t hallSensed5 = 0;
uint32_t hallSensed6 = 0;

uint32_t direction2Cycles = 0;

/* uint32_t hallSensedPrevious1 = 0;
uint32_t hallSensedPrevious2 = 0;
uint32_t hallSensedPrevious3 = 0;
uint32_t hallSensedPrevious4 = 0; */

hw_timer_t *My_timer = NULL;

int servoState1 = 0;  // double
int servoState2 = 0;  // up out
int servoState3 = 80; // down out
int servoState4 = 0;
int servoState5 = 80; // up cross
int servoState6 = 80; // down cross

int servoState1Previous = 80;
int servoState2Previous = 80;
int servoState3Previous = 0;
int servoState4Previous = 80;
int servoState5Previous = 0;
int servoState6Previous = 0;

int servo1Pin = 15;
int servo2Pin = 2;
int servo3Pin = 0;
int servo4Pin = 4;
int servo6Pin = 12;
int servo5Pin = 26;
// Motor A
int motor1Pin1 = 18;
int motor1Pin2 = 19;
int enable1Pin = 23;
// Motor B
int motor2Pin1 = 16;
int motor2Pin2 = 17;
int enable2Pin = 5;

int sensorPin1 = 13; // Digital-Pin
int sensorPin2 = 27; // Digital-Pin
int sensorPin3 = 33; // Digital-Pin
int sensorPin4 = 32; // Digital-Pin
int sensorPin5 = 35; // Digital-Pin
int sensorPin6 = 34; // Digital-Pin

int relayPin = 14;

bool twoTrains = true;

// Setting PWM properties
const int freq = 1000;
const int pwmChannel1 = 6;
const int pwmChannel2 = 7;
const int resolution = 8;
int dutyCycle1 = 0;
int dutyCycle2 = 0;
int targetDutyCycle1 = 255;
int targetDutyCycle2 = 255;

bool direction1 = false;
bool direction2 = false;
bool direction1Previous = false;
bool direction2Previous = false;
bool relayState = false;
bool relayStatePrevious = false;
volatile unsigned long direction1Schedule = millis();
volatile unsigned long relaySchedule = millis();

void reportDirection1(bool dir1)
{

  direction1 = dir1;

  Serial.print("Direction1: ");
  display.fillRect(0, 10, 64, 10, BLACK);
  Serial.println(direction1);
  display.setCursor(0, 10);
  // Display static text
  if (direction1)
  {
    display.println("fw");
  }
  else
  {
    display.println("bw");
  }
  display.setCursor(64, 10);
  // Display static text

  display.display();
}

void reportDirection2(bool dir2)
{
  direction2 = dir2;

  Serial.print("Direction2: ");
  display.fillRect(64, 10, 128, 10, BLACK);
  Serial.println(direction2);

  display.setCursor(64, 10);
  // Display static text
  if (direction2)
  {
    display.println("fw");
  }
  else
  {
    display.println("bw");
  }

  // Display static text

  display.display();
}

void reportTrainCount(bool hasTwoTrains)
{
  twoTrains = hasTwoTrains;
  display.fillRect(64, 40, 128, 10, BLACK);
  display.setCursor(64, 40);
  if (hasTwoTrains)
  {
    display.println("2 trains");
  }
  else
  {
    display.println("1 train");
  }
  display.display();
}

void setRelay(boolean state)
{
  if (state == true)
  {
    digitalWrite(relayPin, HIGH);
  }
  else
  {
    digitalWrite(relayPin, LOW);
  }
}

void setDirection1(boolean dir)
{
  // motordirection = dir;
  if (dir)
  {
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW);
    pTxCharacteristicDirection->setValue("forward");
    pTxCharacteristicDirection->notify();
  }
  else
  {
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH);
    pTxCharacteristicDirection->setValue("backward");
    pTxCharacteristicDirection->notify();
  }

  reportDirection1(dir);
}

void setDirection2(boolean dir)
{
  // motordirection = dir;
  if (dir)
  {
    digitalWrite(motor2Pin1, HIGH);
    digitalWrite(motor2Pin2, LOW);
    pTxCharacteristicDirection->setValue("forward");
    pTxCharacteristicDirection->notify();
  }
  else
  {
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, HIGH);
    pTxCharacteristicDirection->setValue("backward");
    pTxCharacteristicDirection->notify();
  }

  reportDirection2(dir);
}

void IRAM_ATTR onTimer()
{
  // dutyCycle2 = 0;
  direction2 = !direction2;
  direction2Cycles += 1;

  if ((direction2Cycles % 6) ==1 || (direction2Cycles % 6) ==2)
  {
    servoState4 = 80;
  }
  else
  {
    servoState4 = 0;
  }
  if ((direction2Cycles % 6) == 3 ||(direction2Cycles % 6) == 5)
  {
    servoState6 = 0;
  }
  else
  {
    servoState6 = 80;
  }
}

void IRAM_ATTR reportSensorRead1() // inner circle sensor
{

  if (millis() > DebounceTimer1 + delayTime)
  {
    targetDutyCycle1 = 230;
    DebounceTimer1 = millis();
    hallSensed1 += 1;

    if ((hallSensed1 % 2) == 0)
    {
      servoState1 = 80;
    }
    else
    {
      servoState1 = 0;
    }
    servoState3 = 0;
    servoState2 = 80;

    if (twoTrains)
    {
      if ((hallSensed1 % 4) == 0)
      {
        relayState = true; // Stop in inner circle
        relaySchedule = millis() + 3000;
        dutyCycle1 = 0;
        targetDutyCycle1 = 0;
        direction1Schedule = millis() + 3000;
        direction1 = false;
      }
    }
    else
    {
      relayState = false;
    }
  }
}
void IRAM_ATTR reportSensorRead3() // tunnel sensor
{

  if (millis() > DebounceTimer3 + delayTime)
  {
    targetDutyCycle1 = 220;
    DebounceTimer3 = millis();
    hallSensed3 += 1;
    if (direction1)
    {

      if ((hallSensed3 % 4) == 0)
      {
        servoState2 = 0;
      }

      servoState3 = 0;
      servoState1 = 0;
    }
    else
    {

      servoState3 = 80;
      servoState2 = 80;
      servoState1 = 0;
    }

    /*   if (!twoTrains)
      {
        digitalWrite(relayPin, LOW);
      } */
    if (direction1 == true) // ccw
    {
      relayState = false; // activate inner curve
    }
  }
}
void IRAM_ATTR reportSensorRead2() // upper exit sensor
{

  if (millis() > DebounceTimer2 + delayTime)
  {
    targetDutyCycle1 = 255;
    DebounceTimer2 = millis();

    if ((hallSensed2 % 4) == 0)
    {
      direction1Schedule = millis() + 17000;
      direction1 = true;
    }

    if ((hallSensed2 % 4) == 1)
    {
      direction1Schedule = millis() + 2000;
      direction1 = false;
    }

    if ((hallSensed2 % 4) == 3 || (hallSensed2 % 4) == 0)
    {
      servoState5 = 80;
    }
    else
    {
      servoState5 = 0;
    }

    if (direction1 == false) // driving down cw
    {
      servoState1 = 0;  // activate outer circle
      servoState2 = 0;  // upper exit switch to entering
      servoState3 = 80; // lower exit switch allow exit
    }
    else
    {
    }
    relayState = true; // activate lower entry

    hallSensed2 += 1;
    if (twoTrains)
    {
      servoState3 = 80; // lower exit switch allow exit
    }
    else
    {
      direction1Schedule = millis() + 3000;
      direction1 = false;
    }
    /* if (twoTrains)
    {
      digitalWrite(relayPin, HIGH); // two trains
      servoState3 = 80; // lower exit switch allow entry
    } */
    // direction2 = false;
  }
}
void IRAM_ATTR reportSensorRead4() // lower exit sensor
{

  if (millis() > DebounceTimer4 + delayTime)
  {
    targetDutyCycle1 = 255;
    DebounceTimer4 = millis();
    hallSensed4 += 1;

    if (twoTrains)
    {
      if (direction1 == false) // train exiting
      {
        relayState = false; // two trains
        dutyCycle1 = 0;
        targetDutyCycle1 = 0;
        relaySchedule = millis() + 3000;
        // servoState3 = 0;             // lower exit switch prevent entry
        servoState2 = 0;  // upper exit switch to exiting
        servoState1 = 80; // activate inner circle
      }
      else // train entering
      {
        // digitalWrite(relayPin, LOW);
        servoState3 = 80; // lower exit switch allow entry
        servoState2 = 80; // upper exit switch to not exiting
      }
    }
    else
    {
      relayState = true; // activate lower entry
    }

    direction1Schedule = millis() + 3000;
    direction1 = true;

    // direction2 = true;
    /* if ((hallSensed4 % 2) == 0)
    {
      servoState4 = 80;
    }
    else
    {
      servoState4 = 0;
    } */
  }
}

void IRAM_ATTR reportSensorRead5() // lower exit sensor
{
  DebounceTimer5 = millis();
  hallSensed5 += 1;
}

void IRAM_ATTR reportSensorRead6() // lower exit sensor
{
  DebounceTimer6 = millis();
  hallSensed6 += 1;
}

class MyCallbacks : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *pCharacteristic)
  {
    std::string rxValue = pCharacteristic->getValue();

    if (rxValue.length() > 0)
    {

      Serial.println("*********");
      Serial.print("Received Value: ");
      Serial.print(rxValue.c_str());
    }

    if (rxValue == "backward")
    {
      direction1 = false;
      direction2 = false;
    }
    if (rxValue == "forward")
    {
      direction1 = true;
      direction2 = true;
    }
  }
};

void reportDutyCycle(int dutyCycle)
{

  if (servoState1 != servoState1Previous)
  {
    servoState1Previous = servoState1;
    myservo1.write(servoState1);
  }

  if (servoState2 != servoState2Previous)
  {
    servoState2Previous = servoState2;
    // delay(250);
    myservo2.write(servoState2);
  }

  if (servoState3 != servoState3Previous)
  {
    servoState3Previous = servoState3;
    // delay(250);
    myservo3.write(servoState3);
  }

  if (servoState4 != servoState4Previous)
  {
    servoState4Previous = servoState4;
    // delay(250);
    myservo4.write(servoState4);
  }

  if (servoState5 != servoState5Previous)
  {
    servoState5Previous = servoState5;
    // delay(250);
    myservo5.write(servoState5);
  }

  if (servoState6 != servoState6Previous)
  {
    servoState6Previous = servoState6;
    // delay(250);
    myservo6.write(servoState6);
  }

  if (direction1 != direction1Previous)
  {
    if (millis() > direction1Schedule)
    {

      direction1Previous = direction1;
      dutyCycle1 = 0;
      ledcWrite(pwmChannel1, dutyCycle1);
      setDirection1(direction1);
      
      targetDutyCycle1 = 255;
    }
  }

  if (relayState != relayStatePrevious)
  {
    if (millis() > relaySchedule)
    {
      targetDutyCycle1 = 255;
      relayStatePrevious = relayState;
      setRelay(relayState);
    }
  }

  if (direction2 != direction2Previous)
  {
    direction2Previous = direction2;
    
    dutyCycle2 = 0;
    ledcWrite(pwmChannel2, dutyCycle2);
    setDirection2(direction2);
    targetDutyCycle2 = 255;
    
  }

  if (dutyCycle1 < targetDutyCycle1)
  {
    if (targetDutyCycle1 - dutyCycle1 > 50)
    {
      dutyCycle1 += 20;
    }
    dutyCycle1++;
  }

  if (dutyCycle1 > targetDutyCycle1)
  {
    dutyCycle1--;
  }

  if (dutyCycle2 < targetDutyCycle2)
  {
    if (targetDutyCycle2 - dutyCycle2 > 50)
    {
      dutyCycle2 += 20;
    }
    dutyCycle2++;
  }

  if (dutyCycle2 > targetDutyCycle2)
  {
    dutyCycle2--;
  }

  display.fillRect(0, 20, 64, 10, BLACK);
  display.setCursor(0, 20);

  // Display static text
  display.println(hallSensed1, DEC);
  display.display();

  display.fillRect(0, 30, 64, 10, BLACK);
  display.setCursor(0, 30);

  // Display static text
  display.println(hallSensed2, DEC);
  display.display();

  display.fillRect(0, 40, 64, 10, BLACK);
  display.setCursor(0, 40);

  // Display static text
  display.println(hallSensed3, DEC);
  display.display();

  display.fillRect(0, 50, 64, 10, BLACK);
  display.setCursor(0, 50);

  // Display static text
  display.println(hallSensed4, DEC);
  display.display();

  display.fillRect(0, 60, 64, 10, BLACK);
  display.setCursor(0, 60);

  // Display static text
  display.println(hallSensed5, DEC);
  display.display();

  ledcWrite(pwmChannel1, dutyCycle1);
  ledcWrite(pwmChannel2, dutyCycle2);
  // Serial.print("Duty cycle: ");
  // Serial.println(dutyCycle);
  display.fillRect(0, 0, 128, 10, BLACK);
  display.setCursor(0, 0);
  // Display static text

  display.println(dutyCycle1, DEC);
  display.setCursor(64, 0);
  display.println(dutyCycle2, DEC);
  display.display();
}

void setup()
{
  pinMode(relayPin, OUTPUT);
  relayState = true;

  Serial.begin(115200);

  pixels.begin(); // This initializes the NeoPixel library.

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;
  }

  delay(2000);
  display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  // Display static text
  display.println("Starting BLE work!");
  display.display();

  Serial.println("Starting BLE work!");

  BLEDevice::init("ESP32 AS A BLE");
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);

  pTxCharacteristicSensor1 = pService->createCharacteristic(
      CHARACTERISTIC_UUID_SENSOR_1_TX,
      BLECharacteristic::BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE);
  pTxCharacteristicSensor1->setCallbacks(new MyCallbacks());
  pTxCharacteristicSensor1->addDescriptor(new BLE2902());

  pTxCharacteristicDirection = pService->createCharacteristic(
      CHARACTERISTIC_UUID_DIRECTION_TX,
      BLECharacteristic::BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE);
  pTxCharacteristicDirection->setCallbacks(new MyCallbacks());
  pTxCharacteristicDirection->addDescriptor(new BLE2902());

  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID,
      BLECharacteristic::BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE);
  pCharacteristic->setCallbacks(new MyCallbacks());
  pCharacteristic->addDescriptor(new BLE2902());

  pCharacteristic->setValue("Hi,other ESP32 here is your data");
  pService->start();
  // BLEAdvertising *pAdvertising = pServer->getAdvertising();  // this still is working for backward compatibility
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  Serial.println("Characteristic defined!");

  pinMode(sensorPin1, INPUT_PULLUP);
  pinMode(sensorPin2, INPUT_PULLUP);
  pinMode(sensorPin3, INPUT_PULLUP);
  pinMode(sensorPin4, INPUT_PULLUP);
  pinMode(sensorPin5, INPUT_PULLUP);
  pinMode(sensorPin6, INPUT_PULLUP);
  attachInterrupt(sensorPin1, reportSensorRead1, RISING);
  attachInterrupt(sensorPin2, reportSensorRead2, RISING);
  attachInterrupt(sensorPin3, reportSensorRead3, RISING);
  attachInterrupt(sensorPin4, reportSensorRead4, RISING);
  attachInterrupt(sensorPin5, reportSensorRead5, RISING);
  attachInterrupt(sensorPin6, reportSensorRead6, RISING);

  // sets the pins as outputs:
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(enable1Pin, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  pinMode(enable2Pin, OUTPUT);

  // configure LED PWM functionalitites
  ledcSetup(pwmChannel1, freq, resolution);
  ledcSetup(pwmChannel2, freq, resolution);

  // attach the channel to the GPIO to be controlled
  ledcAttachPin(enable1Pin, pwmChannel1);
  ledcAttachPin(enable2Pin, pwmChannel2);

  direction2 = true;

  direction1 = true;

  reportTrainCount(true);

  My_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(My_timer, &onTimer, true);
  timerAlarmWrite(My_timer, 16000000, true);
  timerAlarmEnable(My_timer);

  myservo1.attach(servo1Pin);
  myservo2.attach(servo2Pin);
  myservo3.attach(servo3Pin);
  myservo4.attach(servo4Pin);
  myservo5.attach(servo5Pin);
  myservo6.attach(servo6Pin);
}

void loop()
{

  pixels.setPixelColor(0, pixels.Color(0, 255, 255));
  pixels.setPixelColor(2, pixels.Color(0, 255, 0));
  pixels.show(); // This sends the updated pixel color to the hardware.

  reportDutyCycle(dutyCycle1);
  reportDutyCycle(dutyCycle2);
  delay(200);
}