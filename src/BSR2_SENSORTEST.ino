/*
    5/20/2023

  TODO:
    TESTS:
      **add code for multiple layered mazes

    CODE:
      clean up code and remove deprecated stuff

      blue tile logic

    HARDWARE:

*/

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_VL6180X.h>
#include <Arduino.h>
#include <Arduino_LSM9DS1.h>
#include <Dequeue.h>
#include <RPLidar.h>
#include <RunningMedian.h>
#include <TwoDTree.h>
#include <Wire.h>

#include "Adafruit_TSL2561_U.h"

#define SCREEN_ADDRESS 0x3C  ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

#define EN1 A0  // for controlling left
#define PH1 A1
#define EN2 A2  // for controlling right
#define PH2 A3

#define SOLENOID_PIN 12  // for controlling the solenoid trigger
#define BLINKER 11       // led for blink

#define MOTOCTL A7

#define stde 30  // standard error for filtering
#define LUX_CONSTANT 200
#define BLUE_TILE_CONSTANT 300

enum Direction { FRONT, RIGHT, BACK, LEFT };

struct LidarData {
    float distance;
    float angle;
    uint8_t quality;
};

struct Node {
    Node* leftNode;  // for some reason the size of a node will actually reduce when we put the pointers in the node first
    Node* rightNode;
    int8_t x;
    int8_t y;

    // these are bools confirming connectivity with the 4 possible geometrically adjacent nodes, we can then calculate each adjacent node's coordinates
    bool up : 1;
    bool down : 1;
    bool left : 1;
    bool right : 1;
};

TwoDTree<Node> nodeMap(50);  // 2 dimensional tree to store map
Dequeue<Node> hStack(50);    // stack to store movement history
LidarData currentPoint;

Adafruit_VL6180X tofFront;  // connected to sd sc 3
Adafruit_VL6180X tofRight;  // connected to sd sc 2
Adafruit_VL6180X tofBack;   // connected to sd sc 1
Adafruit_VL6180X tofLeft;   // connected to sd sc 0

Adafruit_TSL2561_Unified luxSensor = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);  // connected to sd sc 4
Adafruit_SSD1306 display(128, 32, &Wire, -1);

float pointMemory[3600];

class Robot {
   private:
    RPLidar lidar;

    uint32_t e = 0;  // error
    float previousAngle = 0;

    int8_t x = 0;
    int8_t y = 0;
    Direction currentDirection = FRONT;

    bool onFloorTwo = false;

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
        for (uint8_t i = 0; i < 5; i++) {
            digitalWrite(BLINKER, HIGH);
            delay(500);
            digitalWrite(BLINKER, LOW);
            delay(500);
        }
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

        // clear current pointcloud
        for (uint16_t i = 0; i < 3600; i++) {
            pointMemory[i] = 0;
        }

        uint16_t i = 0;
        while (i < n) {
            if (readLidar()) {
                i++;
            }
        }
    }

    double angleToWall(uint16_t theta1, uint16_t theta2) {
        double sum_x = 0;
        double sum_y = 0;
        double sum_x2 = 0;
        double sum_y2 = 0;
        double sum_xy = 0;
        uint32_t n = 0;

        fullScan(400);

        sum_x = 0;
        sum_y = 0;
        sum_x2 = 0;
        sum_y2 = 0;
        sum_xy = 0;
        uint32_t i = theta1 * 10;
        n = 0;
        while (i < theta2 * 10) {
            if (abs(sum_x) > 200000000 || abs(sum_y) > 200000000 || abs(sum_x2) > 200000000 || abs(sum_y2) > 200000000 || abs(sum_xy) > 200000000) {
                printText(F("overflow protection"));
                break;
            }

            if (pointMemory[i % 3600] != 0) {
                double y = pointMemory[i % 3600] * cos((i / 10.0) * DEG_TO_RAD);
                double x = pointMemory[i % 3600] * sin((i / 10.0) * DEG_TO_RAD);
                sum_x += x;
                sum_y += y;
                sum_x2 += x * x;
                sum_y2 += y * y;
                sum_xy = sum_xy + x * y;
                n++;
            }
            i++;
        }

        if ((n * sum_x2 - sum_x * sum_x) == 0) {
            return NULL;
        }

        double m = (n * sum_xy - sum_x * sum_y) / (n * sum_x2 - sum_x * sum_x);

        // printText(String(atan(m) * RAD_TO_DEG, 2));
        return atan(m) * RAD_TO_DEG;
    }

    double angleToWallV2(uint16_t theta1, uint16_t theta2) {
        // non working function attempt to calculate angle of wall
        // idea is to find essentially quarter 1 and quarter 3 and try to calculate slope based on two points
        // angleToWall works better
        fullScan(400);
        RunningMedian q1((uint16_t)(abs(theta2 - theta1) / 2) + 1);
        RunningMedian q3((uint16_t)(abs(theta2 - theta1) / 2) + 1);
        for (uint16_t i = theta1; i < (theta1 + abs(theta2 - theta1)) % 3600; i++) {
            // add pointss to find q1 _> convert to coordinates find slope
            if (pointMemory[i] != 0) {
                q1.add(pointMemory[i]);
            }
        }

        for (uint16_t i = (abs(theta2 - theta1)) % 3600; i % 3600 < theta2; i++) {
            if (pointMemory[i] != 0) {
                q3.add(pointMemory[i]);
            }
        }
        // y2-y1/x2-x1

        float y2 = q3.getMedian() * sin(theta1 + (angleDistance(theta1, theta2) / 4) * DEG_TO_RAD);
        float y1 = q1.getMedian() * sin(theta1 + 3 * (angleDistance(theta1, theta2) / 4) * DEG_TO_RAD);
        float x2 = q3.getMedian() * cos(theta1 + (angleDistance(theta1, theta2) / 4) * DEG_TO_RAD);
        float x1 = q1.getMedian() * cos(theta1 + 3 * (angleDistance(theta1, theta2) / 4) * DEG_TO_RAD);

        // printText(String(y2) + " " + String(y1) + " " + String(x2) + " " + String(x1));

        float m = (y2 - y1) / (x2 - x1);

        return atan(m);
    }

    Direction directionInverter(Direction d, bool i) {
        // helper function for calibrate to wall, it inverts directions so the robot turns parallel with the wall, not in the wrong direction
        if (i) {
            switch (d) {
                case FRONT:
                    return Direction::BACK;
                case RIGHT:
                    return Direction::LEFT;
                case BACK:
                    return Direction::FRONT;
                case LEFT:
                    return Direction::RIGHT;
            }
        }
        return d;
    }

    void calibrateToWallHZ(uint16_t theta1, uint16_t theta2, bool invert) {
        // will adjust robot position to be as parellel to wall whichever angle is
        // this function will only work at front and back walls as the slope will tend towards 0
        stop();
        delay(750);
        double m = angleToWall(theta1, theta2);

        if (m > 5) {
            rightMotors(15, directionInverter(Direction::FRONT, invert));
            leftMotors(15, directionInverter(Direction::BACK, invert));
            while (m >= 1) {
                m = angleToWall(theta1, theta2);
                printText(String(m));
            }
        } else if (m < -5) {
            rightMotors(15, directionInverter(Direction::BACK, invert));  //! invert
            leftMotors(15, directionInverter(Direction::FRONT, invert));
            while (m <= -1) {
                m = angleToWall(theta1, theta2);
                printText(String(m));
            }
        }
        stop();
    }

    void calibrateToWallVL(uint16_t theta1, uint16_t theta2, bool invert) {
        // functionally the same as calibrateToWallHZ but as walls to left and right of the robot tend towards 90 deg when palallel
        stop();
        delay(750);
        double m = angleToWall(theta1, theta2);

        if (m > 0 && m <= 87) {
            rightMotors(15, directionInverter(Direction::FRONT, invert));
            leftMotors(15, directionInverter(Direction::BACK, invert));
            while (m > 0 && m < 86) {
                m = angleToWall(theta1, theta2);
                printText(String(m));
            }
        } else if (m < 0 && m >= -87) {
            rightMotors(10, directionInverter(Direction::BACK, invert));
            leftMotors(10, directionInverter(Direction::FRONT, invert));
            while (m < 0 && m > -87) {
                m = angleToWall(theta1, theta2);
                printText(String(m));
            }
        }
    }

    void calibrateToWall(Direction d) {
        // aligns robot parallel with a wall given a direction
        switch (d) {
            case FRONT:
                calibrateToWallHZ(170, 190, true);
                break;
            case RIGHT:
                calibrateToWallVL(260, 280, false);
                break;
            case LEFT:
                calibrateToWallVL(80, 100, false);
                break;
            case BACK:
                calibrateToWallHZ(350, 370, true);
                break;
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

        // possible place for integer overflow, if e gets too big, set it back to 0
        if (e > 30000) {
            e = 0;
        }

        if (d.quality == 15 && d.distance < 6000 && d.distance > 150 && (d.distance < 3900 || d.distance > 4100) && (d.angle < 123 || d.angle > 125) && d.angle < 360 && d.angle >= 0) {
            if (angleDistance(previousAngle, d.angle) < stde + e) {  // make sure that the angle can never be above 360 or else program dies
                pointMemory[(uint16_t)(d.angle * 10)] = d.distance;
                previousAngle = d.angle;
                return true;
            } else
                e += stde;
        } else
            e += stde;

        if (d.angle < 360 && d.angle >= 0) {
            pointMemory[(uint16_t)(d.angle * 10)] = 0;
        } else {  // prevent possible angles greater than 360
            pointMemory[3599] = 0;
        }
        return false;
    }

    void linearEstimate(uint16_t a, uint16_t b) {
        // currently unused method
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
            d.quality = lidar.getCurrentPoint().quality;    // quality of measurement

            return filterData(d);
        } else {
            analogWrite(MOTOCTL, 0);  // stop the rplidar motor

            printText(F("lost connection to lidar"));
            // try to detect RPLIDAR...
            rplidar_response_device_info_t info;
            if (IS_OK(lidar.getDeviceInfo(info, 100))) {
                // detected...
                printText(F("detected"));
                lidar.startScan();

                // start motor rotating at max allowed speed
                analogWrite(MOTOCTL, 255);
            }
        }
        return false;
    }

    void stop() {
        analogWrite(EN1, 0);
        digitalWrite(PH1, HIGH);
        analogWrite(EN2, 0);
        digitalWrite(PH2, LOW);
    }

    void checkVictim() {
        // checks serial buffer if pi has detected any victims
        if (Serial.available() > 0) {
            stop();
            String data = Serial.readStringUntil('\n');
            printText("victim detected: " + data);
            for (uint8_t i = 0; i < 5; i++) {
                digitalWrite(BLINKER, HIGH);
                delay(500);
                digitalWrite(BLINKER, LOW);
                delay(500);
            }

            switch (data[0]) {
                case 'h':
                    dispenseMedkit(3);
                    break;
                case 's':
                    dispenseMedkit(2);
                    break;
                case 'u':
                    // dispense nothing
                    break;
                case 'r':
                    dispenseMedkit(1);
                    break;
                case 'y':
                    dispenseMedkit(1);
                    break;
                case 'g':
                    // dispense nothing
                    break;
            }
        }
    }

    uint16_t medianDistance(uint16_t theta1, uint16_t theta2) {
        // the idea of this function is to find the distance from the theta
        // inbetween theta1 and 2 being a median, this will be more noise resistant

        RunningMedian r = RunningMedian(abs(theta2 * 10 - theta1 * 10));  // max size of median is 255 we are setting this usually of size 200 //allocates total of 1kb of ram

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

    boolean travel30cm() {
        // also need logic to determine which area to scan from, front or back.
        // possibly keep track of distance change in both directions average
        // distance should add some level of noiseproofing to measurements, we could
        // use median to be even more resistant to outliers
        bool blueTrg = true;
        sensors_event_t lux;

        // center robot on tile
        if (readVl(Direction::BACK) < 200) {
            while (readVl(Direction::BACK) > 65) {
                rightMotors(15, Direction::BACK);
                leftMotors(15, Direction::BACK);
            }
            stop();
            while (readVl(Direction::BACK) < 65) {
                rightMotors(15, Direction::FRONT);
                leftMotors(15, Direction::FRONT);
            }
            stop();
            calibrateToWall();
        }

        fullScan(300);
        uint16_t di = medianDistance(170, 190);  // initial distance
        while (di == 0) {
            fullScan(300);
            di = medianDistance(170, 190);
        }
        // calculate displacement, continue moving until total displacement == 300
        uint16_t dt = di;
        while (abs(dt - di) < 300) {
            rightMotors(25, Direction::FRONT);
            leftMotors(25, Direction::FRONT);
            fullScan(300);
            dt = medianDistance(170, 190);

            // read lux sensor to detect holes, if hole is detected, reverse and returnfalse
            TCA(4);
            luxSensor.getEvent(&lux);
            if (lux.light < LUX_CONSTANT) {
                stop();
                printText(F("hole detected"));
                delay(1000);
                while (di - dt >= 0) {  // reverse to origional position
                    fullScan(300);
                    dt = medianDistance(170, 190);
                    rightMotors(20, Direction::BACK);
                    leftMotors(20, Direction::BACK);
                    printText("change in d holeMode: " + String(abs(dt - di)));
                }
                stop();
                return false;
            }

            // read lux sensor for ble tiles, if blue tile, wait a few seconds to move onto blue tile, pause for 5 seconds and continue
            if (blueTrg && lux.light > LUX_CONSTANT && lux.light < BLUE_TILE_CONSTANT) {
                blueTrg = false;
                delay(3000);  // wait for robot to move onto blue tile
                stop();
                delay(5000);
            }

            printText("change in d: " + String(abs(dt - di)));

            // if (pitch() > 15) {

            // }

            checkVictim();
        }

        stop();

        // center on tile using vl6180x
        if (readVl(Direction::FRONT) < 200) {
            while (readVl(Direction::FRONT) > 65) {
                rightMotors(15, Direction::FRONT);
                leftMotors(15, Direction::FRONT);
            }
            stop();
            while (readVl(Direction::FRONT) < 65) {
                rightMotors(15, Direction::BACK);
                leftMotors(15, Direction::BACK);
            }
            stop();
        }

        calibrateToWall();

        return true;
    }

    void calibrateToWall() {
        // function to calibrate to wall but is able to decide which wall to calibrate to to avoid minimum distance
        // walls must be at least 60mm away to allow for calibraion
        if (readVl(Direction::FRONT) < 255 && readVl(Direction::FRONT) >= 60) {
            printText(F("calibrating to wall in front"));
            delay(2000);
            fullScan(200);
            calibrateToWall(Direction::FRONT);
        } else if (readVl(Direction::BACK) < 255 && readVl(Direction::BACK) >= 60) {
            printText(F("calibrating to wall in back"));
            delay(2000);
            fullScan(200);
            calibrateToWall(Direction::BACK);
        } else if (readVl(Direction::RIGHT) < 255 && readVl(Direction::RIGHT) >= 60) {
            printText(F("calibrating to wall to right"));
            delay(2000);
            fullScan(200);
            calibrateToWall(Direction::RIGHT);
        } else if (readVl(Direction::LEFT) < 255 && readVl(Direction::LEFT) >= 60) {
            printText(F("calibrating to wall in left"));
            delay(2000);
            fullScan(200);
            calibrateToWall(Direction::LEFT);
        } else {
            printText(F("no sutiable wall detected"));
            delay(1000);
        }
        stop();
    }

    void turn90DegRight(boolean invert, uint8_t k) {
        // turn robot 90 deg right, if k is true, will turn left instead
        // turn approx 90 deg, calibrate to wall to eliminate error

        rightMotors(30, directionInverter(Direction::BACK, invert));
        leftMotors(30, directionInverter(Direction::FRONT, invert));
        for (uint8_t i = 0; i < k; i++) {
            delay(7500);  // about the time it takes for it to make a turn
        }
        stop();

        calibrateToWall();
    }

    void rightMotors(uint8_t speed, Direction d) {
        // set speed of right motors
        switch (d) {
            case BACK:
                analogWrite(EN1, speed);
                digitalWrite(PH1, HIGH);
                break;
            case FRONT:
                analogWrite(EN1, speed);
                digitalWrite(PH1, LOW);
                break;
            default:
                stop();
                break;
        }
    }

    void leftMotors(uint8_t speed, Direction d) {
        // set speed of left motors
        switch (d) {
            case BACK:
                analogWrite(EN2, speed);
                digitalWrite(PH2, HIGH);
                break;
            case FRONT:
                analogWrite(EN2, speed);
                digitalWrite(PH2, LOW);
                break;
            default:
                stop();
                break;
        }
    }

    float pitch() {
        // calculate pitch of robot, intended to calculate whether robot is on ramp but without implemention, this method has little use

        float x, y, z;
        if (IMU.accelerationAvailable()) { IMU.readAcceleration(x, y, z); }
        else return 0;
        return (180 * atan2(x, sqrt(y * y + z * z))) / PI;
    }

    void addpoint() {
        // add current point into nodeMap

        Node n;
        n.x = x;
        n.y = y;
        n.leftNode = NULL;
        n.rightNode = NULL;

        //  f r b l
        //  0 1 2 3
        // calculate correct vl6180x to read from to ensure absolute direction
        if (currentDirection == Direction::FRONT || currentDirection == Direction::BACK) {
            n.down = readVl((Direction)((currentDirection + 2) % 4)) > 200 ? true : false;
        } else {
            n.down = readVl(currentDirection) > 200 ? true : false;
        }

        n.left = readVl((Direction)(3 - currentDirection)) > 200 ? true : false;

        if (currentDirection == Direction::LEFT || currentDirection == Direction::RIGHT) {
            n.up = readVl((Direction)((currentDirection + 2) % 4)) > 200 ? true : false;
        } else {
            n.up = readVl(currentDirection) > 200 ? true : false;
        }

        if (currentDirection == Direction::FRONT || currentDirection == Direction::RIGHT) {
            n.right = readVl((Direction)abs(currentDirection - 1)) > 200 ? true : false;
        } else {
            n.right = readVl((Direction)(abs(currentDirection - 3) + 2)) > 200 ? true : false;
        }

        printText("adding point at :" + String(n.x) + ", " + String(n.y) + String(n.down) + String(n.left) + String(n.up) + String(n.right));
        nodeMap.put(n);
        delay(1000);
    }

    bool move(Direction d) {
        // traverse the robot to the geographical direction of the maze (not relative right to the robot)
        //  f r b l
        //  0 1 2 3

        // calculate the amount of turns to take and in which direction
        int k = 0;
        bool tD = false;
        if (d - currentDirection == -1 || d - currentDirection == 3) {
            // turn right once
            k = 1;
        } else if (abs(d - currentDirection) == 2) {
            k = 2;
        } else if (d - currentDirection == 1 || d - currentDirection == -3) {
            // turn left once
            k = 1;
            tD = true;
        }

        turn90DegRight(tD, k);

        currentDirection = d;

        // if there is a hole, throw this problem all the way up to run() to edit node to reflect hole
        if (!travel30cm()) {
            return false;
        }

        // update position in memeory
        switch (d) {
            case FRONT:
                y++;
                break;
            case RIGHT:
                x++;
                break;
            case BACK:
                y--;
                break;
            case LEFT:
                x--;
                break;
        }

        // add current point if not inside the nodeMap
        if (!nodeMap.contains(x, y)) {
            addpoint();
        }

        return true;
    }

    void reversal() {
        // reverse to last node with unexplored branching leaves
        // calculate and execute shortest path to target through stack
        // fill in temporary storage array with full stack reversal then calculate shortcuts
        Node* nT[50];
        uint8_t id = 0;

        Node* nC = hStack.pop();
        Node* shortcut = hStack.peek();
        printText("previous node is : " + String(shortcut->x) + ", " + String(shortcut->y));

        do {
            nC = hStack.pop();

            nT[id] = nC;
            id++;
            // check optimal node to start off
            if ((nC->y == y && nC->x == x + 1 && nC->left) || (nC->y == y && nC->x == x - 1 && nC->right) || (nC->x == x && nC->y == y - 1 && nC->up) || (nC->x == x && nC->y == y + 1 && nC->down)) {
                shortcut = nC;
                // printText("shortcut is : (" + String(nC->x) + ", " + String(nC->y) + ")");
            }
            // goofy conditional -> loop while nc adjacent node does not exist and that node would be accesible from nc
            //  stop when we have an adjacent node that has not been explored and is accesible from nc
        } while (hStack.size() > 0 && !((!nodeMap.contains(nC->x + 1, nC->y) && nC->right) || (!nodeMap.contains(nC->x - 1, nC->y) && nC->left) || (!nodeMap.contains(nC->x, nC->y + 1) && nC->up) ||
                                        (!nodeMap.contains(nC->x, nC->y - 1) && nC->down)));

        printText(String(id - 1) + "items in nT");
        printText("target node to end: " + String(nT[id - 1]->x) + ", " + String(nT[id - 1]->y));
        printText("shortcut:" + String(shortcut->x) + ", " + String(shortcut->y));

        hStack.push(*nT[id - 1]);  // this is to update the stack so it actually registers the target square

        bool trg = true;

        // iterate through queue to pathfind towards target node
        for (uint8_t i = 0; i < id; i++) {
            if (trg && nT[i] != shortcut) {
                continue;
            }
            trg = false;

            printText("moving to: " + String(nT[i]->x) + " " + String(nT[i]->y));
            delay(1000);

            if (y == nT[i]->y && x + 1 == nT[i]->x) {
                printText(F("reverse RIGHT"));
                delay(1000);
                move(Direction::RIGHT);
            } else if (y == nT[i]->y && x - 1 == nT[i]->x) {
                printText(F("reverse LEFT"));
                delay(1000);
                move(Direction::LEFT);
            } else if (x == nT[i]->x && y + 1 == nT[i]->y) {
                printText(F("reverse FRONT"));
                delay(1000);
                move(Direction::FRONT);
            } else if (x == nT[i]->x && y - 1 == nT[i]->y) {
                printText(F("reverse BACK"));
                delay(1000);
                move(Direction::BACK);
            }
        }
    }

   public:
    Robot() {}
    void startup() {
        // setup function to activate all pins, sensors, etc.

        // set up display
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

        // wait for ready response from pi
        printText(F("waiting for pi ..."));

        Serial.begin(9600);
        while (!Serial) {
            delay(1);
        }

        while (Serial.available() == 0) {
            delay(1);
        }
        Serial.readStringUntil('\n');
        printText(F("pi detected"));

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
        luxSensor.setGain(TSL2561_GAIN_1X);
        luxSensor.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);
        printText(F("luxSensor ok"));

        // start built in imu
        if (!IMU.begin()) {
            printText(F("imu failed"));
            while (1)
                ;
        }
        printText(F("imu ok"));

        // setup motor pins
        pinMode(EN1, OUTPUT);
        pinMode(PH1, OUTPUT);
        pinMode(EN2, OUTPUT);
        pinMode(PH2, OUTPUT);

        pinMode(SOLENOID_PIN, OUTPUT);
        pinMode(BLINKER, OUTPUT);

        // set up lidar
        lidar.begin(Serial1);
        // start spinning lidar
        pinMode(MOTOCTL, OUTPUT);
        analogWrite(MOTOCTL, 255);

        // push strartpoint into stack and nodeMap
        addpoint();
        hStack.push(*nodeMap.get(x, y));
    }

    void run() {
        // dfs method for deciding pathfinding

        printText(F("iterating"));
        delay(1000);
        // add current point if not on nodeMap
        if (!nodeMap.contains(x, y)) {
            addpoint();
            printText("point added! " + String(x) + " " + String(y));
            delay(1000);
        }

        printText("current location " + String(x) + String(y));
        delay(1000);

        Node* currentNode = nodeMap.get(x, y);

        printText("currentNode: " + String(currentNode->x) + ", " + String(currentNode->y) + String(currentNode->down) + String(currentNode->left) + String(currentNode->up) +
                  String(currentNode->right));
        delay(1000);

        // priorty: RIGHT, DOWN, LEFT, UP
        if (currentNode->right && !nodeMap.contains(x + 1, y)) {
            printText(F("moving absolute right"));
            delay(1000);
            if (move(Direction::RIGHT)) {
                // if movement is sucsessfull, add point into stack
                hStack.push(*nodeMap.get(x, y));
            } else {
                // if movement failed due to hole, mark currentnode direction as blocked
                currentNode->right = false;
            }
        } else if (currentNode->down && !nodeMap.contains(x, y - 1)) {
            printText(F("moving absolute back"));
            delay(1000);
            if (move(Direction::BACK)) {
                hStack.push(*nodeMap.get(x, y));
            } else {
                currentNode->down = false;
            }
        } else if (currentNode->left && !nodeMap.contains(x - 1, y)) {
            printText(F("moving absolute left"));
            delay(1000);
            if (move(Direction::LEFT)) {
                hStack.push(*nodeMap.get(x, y));
            } else {
                currentNode->left = false;
            }
        } else if (currentNode->up && !nodeMap.contains(x, y + 1)) {
            printText(F("moving absolute Front"));
            delay(1000);
            if (move(Direction::FRONT)) {
                hStack.push(*nodeMap.get(x, y));
            } else {
                currentNode->up = false;
            }
        } else if (hStack.size() > 0) {
            printText(F("reversing"));
            delay(1000);
            reversal();
        }
        stop();
    }

    void methodTester() { calibrateToWall(Direction::FRONT); }
};

Robot robot;

void setup() { robot.startup(); }

void loop() {
    // robot.methodTester();
    robot.run();
}
