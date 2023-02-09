/*
todo:
run a test with start bit data and collect input
improve filter
test wall angle
test align with wall
add wheel rubber
run 
tphynsth

P
*/
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <RPLidar.h>

#include <Adafruit_SSD1306.h>
#include <Adafruit_VL6180X.h>
#include "Adafruit_TSL2561_U.h"

#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

#define EN1 A0 // for controlling left
#define PH1 A1
#define EN2 A2 // for controlling right
#define PH2 A3

#define MOTOCTL A7

float pointMemory[3600];

RPLidar lidar;
struct LidarData
{
  float distance;
  float angle;
  bool startBit;
  byte quality;
};

LidarData currentPoint;

byte rightMotorSpeed;
byte leftMotorSpeed;

Adafruit_VL6180X tofFront; // connected to sd sc 3
Adafruit_VL6180X tofRight; // connected to sd sc 2
Adafruit_VL6180X tofBack;  // connected to sd sc 1
Adafruit_VL6180X tofLeft;  // connected to sd sc 0

Adafruit_TSL2561_Unified luxSensor = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345); // connected to sd sc 4
Adafruit_SSD1306 display(128, 32, &Wire, -1);

enum Direction
{
  FRONT,
  RIGHT,
  LEFT,
  BACK
};

void TCA(uint8_t bus)
{
  Wire.beginTransmission(0x70);
  Wire.write(1 << bus);
  Wire.endTransmission();
}

void printText(String s)
{
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.print(s);
  display.display();
}

int16_t readVl(Direction d)
{
  switch (d)
  {
  case FRONT:
    TCA(3);
    return tofFront.readRange(); // might need to wrap error checking
    break;
  case RIGHT:
    TCA(2);
    return tofRight.readRange();
    break;
  case BACK:
    TCA(1);
    return tofBack.readRange();
    break;
  case LEFT:
    TCA(0);
    return tofLeft.readRange();
    break;
  }
  return -1;
}

int8_t readTSL()
{
  sensors_event_t event;
  luxSensor.getEvent(&event);
  if (event.light == 0)
  {
    display.clearDisplay();
    printText(F("luxSensor overload too bright"));
    return -1;
  }
  else
    return event.light;
}

void fullScan()
{
  // does a full scan of 360 degs
  // should ignore first start bit to ensure a full rotation until the next
  while (currentPoint.startBit)
  {
    readLidar();
  }
}

uint8_t angleToWall(uint16_t theta1, uint16_t theta2)
{ // should return angle relative to east returns -90 to 90 deg
  //update lidar
  fullScan();

  uint32_t sum_x = 0;
  uint32_t sum_y = 0;
  uint32_t sum_x2 = 0;
  uint32_t sum_y2 = 0;
  uint32_t sum_xy = 0;
  uint8_t n = 0;

  for (uint16_t i = theta1; i < theta1 - theta2; i++)
  {
    if (pointMemory[i] == 0)
      continue;
    n++;
    sum_x += pointMemory[i] * cos((i / 10) * DEG_TO_RAD);
    sum_y += pointMemory[i] * sin((i / 10) * DEG_TO_RAD);
    sum_x2 += (pointMemory[i] * cos((i / 10) * DEG_TO_RAD)) * (pointMemory[i] * cos((i / 10) * DEG_TO_RAD));
    sum_y2 += (pointMemory[i] * sin((i / 10) * DEG_TO_RAD)) * (pointMemory[i] * sin((i / 10) * DEG_TO_RAD));
    sum_xy = sum_xy + (pointMemory[i] * cos((i / 10) * DEG_TO_RAD)) * (pointMemory[i] * sin((i / 10) * DEG_TO_RAD));
  }
  uint8_t y = n * sum_xy - sum_x * sum_y;
  uint8_t x = n * sum_x2 - sum_x * sum_x; // need checking to prevent x == 0;
  return atan2(y, x) * RAD_TO_DEG;
}

void calibrateToWall(uint16_t theta1, uint16_t theta2)
{
  // will adjust robot position to be as parellel to wall whichever angle is closer, 0 or 90
  uint8_t m = angleToWall(theta1, theta2);
  if (m > 45)
  {
    while (m < 90)
    {
      rightMotors(1, Direction::FRONT);
      leftMotors(1, Direction::BACK);
      m = angleToWall(theta1, theta2);
    }
  }
  else if (m <= 45 && m > 0)
  {
    while (m >= 0)
    {
      rightMotors(1, Direction::BACK);
      leftMotors(1, Direction::FRONT);
      m = angleToWall(theta1, theta2);
    }
  }
  else if (m > -45 && m <= 0)
  {
    while (m < 0)
    {
      rightMotors(1, Direction::FRONT);
      leftMotors(1, Direction::BACK);
      m = angleToWall(theta1, theta2);
    }
  }
  else if (m < -45) {
    while (m<-90) {
      rightMotors(1, Direction::BACK);
      leftMotors(1, Direction::FRONT);
      m = angleToWall(theta1, theta2);
    }
  }
}

bool filterData(LidarData d)
{ // return false if point is tossed, return false if point is accepted
  // also runs the linear estimate data

  // simple low and high pass filters, will implement smarter filters such as detecting noise and outlier data
  if (d.quality < 15)
    return false;
  if (d.distance < 150)
    return false;
  if (d.distance > 6000)
    return false;

  pointMemory[(uint8_t)(d.angle * 10)] = d.distance;

  //update currentpoint struct
  currentPoint.angle = d.angle;
  currentPoint.distance = d.distance;
  currentPoint.quality = d.quality;
  currentPoint.startBit = d.startBit;
  
  return true;
}

void linearEstimate(uint8_t a, uint8_t b)
{ // angle a * 10, angle b * 10
  // only run if the distance between the two points are low
  // estimate all points between the two angles to fill in all blank spots between angles
  // could use the least square aproximation to estimatepoints, ideally a and b should be as close together as possible

  // if we see that our point is missing, run the linear estimate
  float x1 = pointMemory[a] * cos((a / 10) * DEG_TO_RAD);
  float y1 = pointMemory[a] * sin((a / 10) * DEG_TO_RAD);
  float x2 = pointMemory[b] * cos((b / 10) * DEG_TO_RAD);
  float y2 = pointMemory[b] * sin((b / 10) * DEG_TO_RAD);

  float m = (y2 - y1) / (x2 - x1);

  float slope = y1 - (m * x1);

  for (uint8_t i = a; i < b; i++)
  {
    pointMemory[i] = slope / (sin(DEG_TO_RAD * (i / 10)) - slope * cos(DEG_TO_RAD * i / 10));
  }
}

void readLidar()
{
  if (IS_OK(lidar.waitPoint()))
  {
    LidarData d;
    d.distance = lidar.getCurrentPoint().distance; // in mm
    d.angle = lidar.getCurrentPoint().angle;       // in deg
    d.startBit = lidar.getCurrentPoint().startBit; // wheter point belongs to new scan
    d.quality = lidar.getCurrentPoint().quality;   // quality of measurement

    filterData(d);
  }
  else
  {
    printText("lidar is struggling");

    analogWrite(MOTOCTL, 0); // stop the rplidar motor

    // try to detect RPLIDAR...
    rplidar_response_device_info_t info;
    if (IS_OK(lidar.getDeviceInfo(info, 100)))
    {
      // detected...
      lidar.startScan();

      // start motor rotating at max allowed speed
      analogWrite(MOTOCTL, 255);
      delay(1000);
    }
  }
}

void stop()
{
  analogWrite(EN1, 0);
  digitalWrite(PH1, HIGH);
  analogWrite(EN2, 0);
  digitalWrite(PH2, LOW);
}

uint32_t averageDistance(uint16_t theta1, uint16_t theta2) {
  uint32_t r;
  uint16_t n;
  for (uint16_t i = theta1; i<theta2; i++) {
    if (pointMemory[i] == 0) {
      continue;
    }
    n++;
    r+=pointMemory[i];
  }

  return r/n;
}

void travel30cm() {
  //this method could be important
  //also need logic to determine which area to scan from, front or back. possibly keep track of distance change in both directions
  fullScan();
  uint16_t di = averageDistance(75,105); //because these are the degrees of north i think
  while(averageDistance(75,105)-di < 300) {
    rightMotors(1,Direction::FRONT);
    leftMotors(1,Direction::FRONT);
  }
  stop();
}

void rightMotors(uint8_t speed, Direction d)
{
  rightMotorSpeed = speed;
  switch (d)
  {
  case FRONT:
    analogWrite(EN1, speed);
    digitalWrite(PH1, HIGH);
    break;
  case BACK:
    analogWrite(EN1, speed);
    digitalWrite(PH1, LOW);
    break;
  default:
    stop();
    break;
  }
  rightMotorSpeed = 0;
  leftMotorSpeed = 0;
}

void leftMotors(uint8_t speed, Direction d)
{
  leftMotorSpeed = speed;
  switch (d)
  {
  case FRONT:
    analogWrite(EN2, speed);
    digitalWrite(PH2, HIGH);
    break;
  case BACK:
    analogWrite(EN2, speed);
    digitalWrite(PH2, LOW);
    break;
  default:
    stop();
    break;
  }
}

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS))
  {
    digitalWrite(LED_BUILTIN, HIGH);
    while (1)
      ;
  }

  display.display();
  delay(2000);
  display.clearDisplay();

  // Clear the buffer
  printText(F("starting"));
  delay(1000);

  // start the tofs
  TCA(0);
  if (!tofLeft.begin())
  {
    printText(F("tofLeft failed"));
    while (1)
      ;
  }
  printText(F("tofLeft ok"));
  TCA(1);
  if (!tofBack.begin())
  {
    printText(F("tofback failed"));
    while (1)
      ;
  }
  printText(F("tofBack ok"));
  TCA(2);
  if (!tofRight.begin())
  {
    printText(F("tofRight failed"));
    while (1)
      ;
  }
  printText(F("tofRight ok"));
  TCA(3);
  if (!tofFront.begin())
  {
    printText(F("tofFront failed"));
    while (1)
      ;
  }
  printText(F("tofFront ok"));

  // starting lux sensor
  TCA(4);
  if (!luxSensor.begin())
  {
    printText(F("luxsensor failed"));
    while (1)
      ;
  }
  printText(F("luxSensor ok"));

  pinMode(EN1, OUTPUT);
  pinMode(PH1, OUTPUT);
  pinMode(EN2, OUTPUT);
  pinMode(PH2, OUTPUT);

  Serial.begin(9600);
  lidar.begin(Serial1);

  pinMode(MOTOCTL, OUTPUT);
  analogWrite(MOTOCTL, 255);
}

void loop()
{
  rightMotors(100, Direction::FRONT);
  leftMotors(100, Direction::BACK);
  delay(1000);
  rightMotors(100, Direction::BACK);
  leftMotors(100, Direction::FRONT);
  delay(1000);
}
