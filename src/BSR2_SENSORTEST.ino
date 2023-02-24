/*
  TODO:
    TESTS:
      test wall angle
      test align with wall
      test medkit dispenser and blink
      live memory performance testing

    CODE:
      rewrite angle to wall with line library
      clean up code and remove deprecated stuff
      train pi with data
      find implementations for linear estimate
      create a function that can detect intersections and angles, we need this
  to know the input theta values for align to wall test NextGenAHRS library test
  curvefitting library

      reduce floating point math to reduce errer, might be overkill

    HARDWARE:
      add wheel rubber
      make maze
      build communication between pi and nano

*/

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_VL6180X.h>
#include <Arduino.h>
#include <RPLidar.h>
#include <RunningMedian.h>
#include <Wire.h>
#include <curveFitting.h>

#include "Adafruit_TSL2561_U.h"

#define SCREEN_ADDRESS 0x3C  ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

#define EN1 A0  // for controlling left
#define PH1 A1
#define EN2 A2  // for controlling right
#define PH2 A3

#define SOLENOID_PIN 12  // for controlling the solenoid trigger

#define MOTOCTL A7

float pointMemory[3600];

struct LidarData {
    float distance;
    float angle;
    bool startBit;
    byte quality;
};

LidarData currentPoint;

byte rightMotorSpeed;
byte leftMotorSpeed;

Adafruit_VL6180X tofFront;  // connected to sd sc 3
Adafruit_VL6180X tofRight;  // connected to sd sc 2
Adafruit_VL6180X tofBack;   // connected to sd sc 1
Adafruit_VL6180X tofLeft;   // connected to sd sc 0

Adafruit_TSL2561_Unified luxSensor = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);  // connected to sd sc 4
Adafruit_SSD1306 display(128, 32, &Wire, -1);

enum Direction { FRONT, RIGHT, LEFT, BACK };

class Robot {
    RPLidar lidar;

   private:
    const uint8_t s = 30;  // standard error change this solution sucks for encapsulation
    uint16_t e = 0;        // error
    uint16_t previousAngle = 0;

    void TCA(uint8_t bus) {
        Wire.beginTransmission(0x70);
        Wire.write(1 << bus);
        Wire.endTransmission();
    }

    void printText(String s) {
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(WHITE);
        display.setCursor(0, 0);
        display.print(s);
        display.display();
    }

    void dispenseMedkit(uint8_t n) {
        for (uint8_t i = 0; i < n; i++) {
            digitalWrite(SOLENOID_PIN, HIGH);
            delay(1000);
            digitalWrite(SOLENOID_PIN, LOW);
            delay(1000);
        }
    }

    int16_t readVl(Direction d) {
        switch (d) {
            case FRONT:
                TCA(3);
                return tofFront.readRange();  // might need to wrap error checking
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

    int8_t readTSL() {
        sensors_event_t event;
        luxSensor.getEvent(&event);
        if (event.light == 0) {
            display.clearDisplay();
            printText(F("luxSensor overload too bright"));
            return -1;
        } else
            return event.light;
    }

    void fullScan(uint16_t n) {
        // does a full scan of the first n accepted points
        // does not work based on start bit due to start bit not necessarily indicating a new rotation

        // this will run until we accept 360 points

        // clear current pointcloud
        memset(pointMemory, 0, 3600);
        uint16_t sucsess = 0;
        while (sucsess < n) {
            if (readLidar()) {
                sucsess++;
            }
        }
    }

    uint16_t angleToWall(uint16_t theta1, uint16_t theta2) {  // should return angle relative to east
        // to update with line fitting library.
        // returns -90 to 90 deg
        // update lidar
        fullScan(600);

        uint32_t sum_x = 0;
        uint32_t sum_y = 0;
        uint32_t sum_x2 = 0;
        uint32_t sum_y2 = 0;
        uint32_t sum_xy = 0;
        uint16_t n = 0;

        for (uint16_t i = theta1 * 10; i < (theta2 * 10) - (theta1 * 10); i++) {
            if (pointMemory[i] != 0) {
                n++;
                sum_x += pointMemory[i] * cos((i / 10.0) * DEG_TO_RAD);
                sum_y += pointMemory[i] * sin((i / 10.0) * DEG_TO_RAD);
                sum_x2 += (pointMemory[i] * cos((i / 10.0) * DEG_TO_RAD)) * (pointMemory[i] * cos((i / 10.0) * DEG_TO_RAD));
                sum_y2 += (pointMemory[i] * sin((i / 10.0) * DEG_TO_RAD)) * (pointMemory[i] * sin((i / 10.0) * DEG_TO_RAD));
                sum_xy = sum_xy + (pointMemory[i] * cos((i / 10.0) * DEG_TO_RAD)) * (pointMemory[i] * sin((i / 10.0) * DEG_TO_RAD));
            }
        }
        uint32_t y = n * sum_xy - sum_x * sum_y;
        uint32_t x = n * sum_x2 - sum_x * sum_x;  // need checking to prevent x == 0;
        Serial.println(String(x) + " " + String(y) + " " + String(n) + " " + String(sum_x));
        return atan2(y, x) * RAD_TO_DEG;
    }

    void calibrateToWall(uint16_t theta1, uint16_t theta2) {
        // will adjust robot position to be as parellel to wall whichever angle is
        // closer, 0 or 90
        int8_t m = angleToWall(theta1, theta2);
        if (m > 45) {
            while (m < 90) {
                rightMotors(1, Direction::FRONT);
                leftMotors(1, Direction::BACK);
                m = angleToWall(theta1, theta2);
            }
        } else if (m <= 45 && m > 0) {
            while (m >= 0) {
                rightMotors(1, Direction::BACK);
                leftMotors(1, Direction::FRONT);
                m = angleToWall(theta1, theta2);
            }
        } else if (m > -45 && m <= 0) {
            while (m < 0) {
                rightMotors(1, Direction::FRONT);
                leftMotors(1, Direction::BACK);
                m = angleToWall(theta1, theta2);
            }
        } else if (m < -45) {
            while (m < -90) {
                rightMotors(1, Direction::BACK);
                leftMotors(1, Direction::FRONT);
                m = angleToWall(theta1, theta2);
            }
        }
    }

    float angleDistance(float theta1, float theta2) {
        // calculate distance between two angles
        if (theta2 < theta1) {  // this happens when t2 is like 10, but t2 is about 350
            return (theta2 + 360) - theta1;
        }
        return (theta2 - theta1);
    }

    bool filterData(LidarData d) {  // return false if point is tossed, return
                                    // false if point is accepted
        // simple low and high pass filters, will implement smarter filters such as detecting noise and outlier data

        // filters, block:
        // distance less than 150
        // distance greater than 6000
        // angles between 123 and 125
        // distances between 3900 and 4100
        // filter out qualities that are not 15
        // filter out changes in angle that are greater than s deg

        // i realize that this will not work without some dumb variable scope magic

        if (d.quality == 15 && d.distance < 6000 && d.distance > 150 && (d.distance < 3900 || d.distance > 4100) && (d.angle < 123 || d.angle > 125)) {
            if (d.angle < 360 && angleDistance(previousAngle, d.angle) < s + e) {  // make sure that the angle can never be above 360 or else program dies
                pointMemory[(uint16_t)(d.angle * 10)] = d.distance;
                return true;
            } else
                e += s;
        } else
            e += s;

        pointMemory[(uint16_t)(d.angle * 10)] = 0;
        return false;
    }

    void linearEstimate(uint16_t a, uint16_t b) {  // currently unused method
        // angle a * 10, angle b * 10
        // only run if the distance between the two points are low
        // estimate all points between the two angles to fill in all blank spots
        // between angles could use the least square aproximation to estimatepoints,
        // ideally a and b should be as close together as possible

        // if we see that our point is missing, run the linear estimate
        float x1 = pointMemory[a] * cos((a / 10) * DEG_TO_RAD);
        float y1 = pointMemory[a] * sin((a / 10) * DEG_TO_RAD);
        float x2 = pointMemory[b] * cos((b / 10) * DEG_TO_RAD);
        float y2 = pointMemory[b] * sin((b / 10) * DEG_TO_RAD);

        float m = (y2 - y1) / (x2 - x1);

        float slope = y1 - (m * x1);

        for (uint16_t i = a; i < b; i++) {
            pointMemory[i] = slope / (sin(DEG_TO_RAD * (i / 10)) - slope * cos(DEG_TO_RAD * i / 10));
        }
    }

    boolean readLidar() {
        if (IS_OK(lidar.waitPoint())) {
            LidarData d;
            d.distance = lidar.getCurrentPoint().distance;  // in mm
            d.angle = lidar.getCurrentPoint().angle;        // in deg
            d.startBit = lidar.getCurrentPoint().startBit;  // wheter point belongs to new scan
            d.quality = lidar.getCurrentPoint().quality;    // quality of measurement

            return filterData(d);
        } else {
            printText(F("lidar is struggling"));

            analogWrite(MOTOCTL, 0);  // stop the rplidar motor

            // try to detect RPLIDAR...
            rplidar_response_device_info_t info;
            if (IS_OK(lidar.getDeviceInfo(info, 100))) {
                // detected...
                lidar.startScan();

                // start motor rotating at max allowed speed
                analogWrite(MOTOCTL, 255);
                delay(1000);
            }
        }
        return false;
    }

    void stop() {
        rightMotorSpeed = 0;
        leftMotorSpeed = 0;
        analogWrite(EN1, 0);
        digitalWrite(PH1, HIGH);
        analogWrite(EN2, 0);
        digitalWrite(PH2, LOW);
    }

    uint32_t averageDistance(uint16_t theta1, uint16_t theta2) {
        uint32_t r = 0;
        uint16_t n = 0;
        for (uint16_t i = theta1; i < theta2; i++) {
            if (pointMemory[i] == 0) {
                continue;
            }
            n++;
            r += pointMemory[i];
        }

        return r / n;
    }

    uint16_t medianDistance(uint16_t theta1, uint16_t theta2) {
        // the idea of this function is to find the distance from the theta
        // inbetween theta1 and 2 being a median, this will be more noise resistant

        RunningMedian r = RunningMedian(abs(theta2 * 10 - theta1 * 10));

        for (uint16_t i = theta1 * 10; i < theta2 * 10; i++) {
            if (pointMemory[i] != 0) {
                r.add(pointMemory[i]);
            }
        }

        if (r.getCount() == 0) {
            return 0;
        }
        return r.getMedian();
    }

    void travel30cm() {
        // this method could be important
        // also need logic to determine which area to scan from, front or back.
        // possibly keep track of distance change in both directions average
        // distance should add some level of noiseproofing to measurements, we could
        // use median to be even more resistant to outliers

        // this probably is very slow and inefficient
        fullScan(600);
        uint16_t di = medianDistance(170, 190);  // because these are the degrees of north i think
        while (di == 0) {
            fullScan(600);
            di = medianDistance(170, 190);
        }
        uint16_t dt = di;
        while (abs(dt - di) < 300) {
            rightMotors(10, Direction::FRONT);
            leftMotors(10, Direction::FRONT);
            do {
                fullScan(600);
                dt = medianDistance(170, 190);
                printText(String(di) + "\n" + String(dt) + "\n" + String(dt - di));
            } while (dt == 0);
        }
        stop();
    }

    void rightMotors(uint8_t speed, Direction d) {
        rightMotorSpeed = speed;
        switch (d) {
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
    }

    void leftMotors(uint8_t speed, Direction d) {
        leftMotorSpeed = speed;
        switch (d) {
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

   public:
    Robot() {}
    void startup() {
        pinMode(LED_BUILTIN, OUTPUT);

        if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
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
        if (!tofLeft.begin()) {
            printText(F("tofLeft failed"));
            while (1)
                ;
        }
        printText(F("tofLeft ok"));
        TCA(1);
        if (!tofBack.begin()) {
            printText(F("tofback failed"));
            while (1)
                ;
        }
        printText(F("tofBack ok"));
        TCA(2);
        if (!tofRight.begin()) {
            printText(F("tofRight failed"));
            while (1)
                ;
        }
        printText(F("tofRight ok"));
        TCA(3);
        if (!tofFront.begin()) {
            printText(F("tofFront failed"));
            while (1)
                ;
        }
        printText(F("tofFront ok"));

        // starting lux sensor
        TCA(4);
        if (!luxSensor.begin()) {
            printText(F("luxsensor failed"));
            while (1)
                ;
        }
        printText(F("luxSensor ok"));

        pinMode(EN1, OUTPUT);
        pinMode(PH1, OUTPUT);
        pinMode(EN2, OUTPUT);
        pinMode(PH2, OUTPUT);

        pinMode(SOLENOID_PIN, OUTPUT);

        Serial.begin(9600);
        lidar.begin(Serial1);

        pinMode(MOTOCTL, OUTPUT);
        analogWrite(MOTOCTL, 255);
    }

    void printCurrentPoint() {
        readLidar();
        Serial.println(String(currentPoint.angle) + " " + String(currentPoint.distance) + " " + String(currentPoint.quality) + " " + String(currentPoint.startBit));
    }

    void exportLidarData() {
        fullScan(400);
        for (uint16_t i = 0; i < 3600; i++) {
            Serial.println(String((float)i / 10) + " " + String(pointMemory[i]));
        }
        Serial.println("\n\n\n");
    }

    void fullScanTimer() {
        uint32_t t1 = millis();
        fullScan(400);
        Serial.println("time taken: " + String((millis() - t1)));
    }

    void methodTester() {
        Serial.println(angleToWall(170, 190));
    }
};

Robot robot;

void setup() { robot.startup(); }

void loop() {
    robot.methodTester();
    // delay(10000);
}
