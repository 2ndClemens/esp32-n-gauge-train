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
// twelve servo objects can be created on most boards

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

uint32_t hallSensed1 = 0;
uint32_t hallSensed2 = 0;
uint32_t hallSensed3 = 0;
uint32_t hallSensed4 = 0;

uint32_t hallSensedPrevious1 = 0;
uint32_t hallSensedPrevious2 = 0;
uint32_t hallSensedPrevious3 = 0;
uint32_t hallSensedPrevious4 = 0;

int servoState1 = 0;  // double
int servoState2 = 0;  // up out
int servoState3 = 80; // down out
int servoState4 = 0;

int servoState1Previous = 80;
int servoState2Previous = 80;
int servoState3Previous = 0;
int servoState4Previous = 80;

int servo1Pin = 15;
int servo2Pin = 2;
int servo3Pin = 0;
int servo4Pin = 4;
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

// Setting PWM properties
const int freq = 30000;
const int pwmChannel1 = 5;
const int pwmChannel2 = 6;
const int resolution = 8;
int dutyCycle1 = 220;
int dutyCycle2 = 250;


bool direction1 = false;
bool direction2 = false;
bool direction1Previous = false;
bool direction2Previous = false;

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
  display.display();
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

void IRAM_ATTR reportSensorRead1()
{
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
}
void IRAM_ATTR reportSensorRead3()
{
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
}
void IRAM_ATTR reportSensorRead2()
{
  hallSensed2 += 1;
  direction1 = false;
  direction2 = false;

}
void IRAM_ATTR reportSensorRead4()
{
  hallSensed4 += 1;
  direction1 = true;
  direction2 = true;
  /* if ((hallSensed4 % 2) == 0)
  {
    servoState4 = 80;
  }
  else
  {
    servoState4 = 0;
  } */
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
    //delay(250);
    myservo2.write(servoState2);
  }

  if (servoState3 != servoState3Previous)
  {
    servoState3Previous = servoState3;
    //delay(250);
    myservo3.write(servoState3);
  }

  if (servoState4 != servoState4Previous)
  {
    servoState4Previous = servoState4;
    //delay(250);
    myservo4.write(servoState4);
  }

  if (direction1 != direction1Previous)
  {
    direction1Previous = direction1;
    setDirection1(direction1);
  }

    if (direction2 != direction2Previous)
  {
    direction2Previous = direction2;
    setDirection2(direction2);
  }



  display.fillRect(0, 20, 128, 10, BLACK);
  display.setCursor(0, 20);

  // Display static text
  display.println(hallSensed1, DEC);
  display.display();

  display.fillRect(0, 30, 128, 10, BLACK);
  display.setCursor(0, 30);

  // Display static text
  display.println(hallSensed2, DEC);
  display.display();

  display.fillRect(0, 40, 128, 10, BLACK);
  display.setCursor(0, 40);

  // Display static text
  display.println(hallSensed3, DEC);
  display.display();

  display.fillRect(0, 50, 128, 10, BLACK);
  display.setCursor(0, 50);

  // Display static text
  display.println(hallSensed4, DEC);
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
  pinMode(sensorPin1, INPUT);
  pinMode(sensorPin2, INPUT);
  pinMode(sensorPin3, INPUT_PULLUP);
  pinMode(sensorPin4, INPUT_PULLUP);
  attachInterrupt(sensorPin1, reportSensorRead1, RISING);
  attachInterrupt(sensorPin2, reportSensorRead2, RISING);
  attachInterrupt(sensorPin3, reportSensorRead3, RISING);
  attachInterrupt(sensorPin4, reportSensorRead4, RISING);

  myservo1.attach(servo1Pin);
  myservo2.attach(servo2Pin);
  myservo3.attach(servo3Pin);
  myservo4.attach(servo4Pin);
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

  Serial.begin(115200);

  pixels.begin(); // This initializes the NeoPixel library.

  // testing
  Serial.print("Testing DC Motor...");

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
  direction2 = true;

  direction1 = true;

  dutyCycle1 = 235;
  dutyCycle2 = 220;
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