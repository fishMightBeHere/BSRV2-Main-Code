/*
    5/4/2023

  TODO:
    TESTS:
      test medkit dispenser and blink
      live memory performance testing
      **add code for multiple layered mazes

    CODE:
      clean up code and remove deprecated stuff
      train pi with data

      black tile logic

      add functionality for 360 deg wraparound and calibrate to back
        if not possible, set turn time to be as close to 90 deg as possible so that in the case of entering a t intersection and turning right, the robot will not brick itself

    HARDWARE:
      build communication between pi and nano
      leds

*/

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_VL6180X.h>
#include <Arduino.h>
#include <Dequeue.h>
#include <RPLidar.h>
#include <RunningMedian.h>
#include <TwoDTree.h>
#include <Wire.h>
#include <Adafruit_LSM9DS1.h>
#include "Adafruit_TSL2561_U.h"

#define SCREEN_ADDRESS 0x3C  ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

#define EN1 A0  // for controlling left
#define PH1 A1
#define EN2 A2  // for controlling right
#define PH2 A3

#define SOLENOID_PIN 12  // for controlling the solenoid trigger

#define MOTOCTL A7

#define stde 30  // standard error for filtering

enum Direction { FRONT, RIGHT, BACK, LEFT };

struct LidarData {
    float distance;
    float angle;
    bool startBit;
    byte quality;
};

struct Node {
    Node* leftNode;//for some reason the size of a node will actually reduce when we put the pointers in the node first
    Node* rightNode;
    int8_t x;
    int8_t y;
    
    // these are bools confirming connectivity with the 4 possible geometrically adjacent nodes, we can then calculate each adjacent node's coordinates 
    bool up : 1;
    bool down : 1;
    bool left : 1;
    bool right : 1;
};

TwoDTree<Node> nodeMap(50);
//TwoDTree<Node> floorTwo(15);
Dequeue<Node> hStack(50);
//Dequeue<Node> floorTwoStack(15);
LidarData currentPoint;

byte rightMotorSpeed;
byte leftMotorSpeed;

Adafruit_LSM9DS1 imu = Adafruit_LSM9DS1();

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
    uint16_t previousAngle = 0;

    int8_t x = 0;
    int8_t y = 0;
    Direction currentDirection = FRONT;

    //const float LUX_CONSTANT;

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

        // clear current pointcloud
        memset(pointMemory, 0, sizeof(pointMemory));
        uint16_t i = 0;
        while (i < n) {
            if (readLidar()) {
                i++;
            }
        }
    }

    double angleToWall(uint16_t theta1, uint16_t theta2) {
        fullScan(200);
        double sum_x = 0;
        double sum_y = 0;
        double sum_x2 = 0;
        double sum_y2 = 0;
        double sum_xy = 0;
        uint16_t n = 0;
        for (uint16_t i = theta1 * 10; i < theta2 * 10; i++) {
            if (pointMemory[i] != 0) {
                int16_t y = pointMemory[i] * cos((i / 10.0) * DEG_TO_RAD);
                int16_t x = pointMemory[i] * sin((i / 10.0) * DEG_TO_RAD);
                sum_x += x;
                sum_y += y;
                sum_x2 += x * x;
                sum_y2 += y * y;
                sum_xy = sum_xy + x * y;
                n++;
            }
        }
        double m = (n * sum_xy - sum_x * sum_y) / (n * sum_x2 - sum_x * sum_x);
        //    int x = 10;
        // double b = (sum_x2 * sum_y - sum_x * sum_xy) / (n * sum_x2 - sum_x * sum_x);
        printText(String(atan(m) * RAD_TO_DEG));
        return atan(m) * RAD_TO_DEG;
    }

    Direction directionInverter(Direction d, bool i) {  // helper function for calibrate to wall, i determines whether to actually invert it
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

    void calibrateToWall(uint16_t theta1, uint16_t theta2, bool invert) {
        // will adjust robot position to be as parellel to wall whichever angle is
        // closer, 0 or 90
        stop();
        double m = angleToWall(theta1, theta2);

        if (m > 0) {
            rightMotors(10, directionInverter(Direction::FRONT, invert));
            leftMotors(10, directionInverter(Direction::BACK, invert));
            while (m >= 5) {
                m = angleToWall(theta1, theta2);
                printText(String(m));
            }
        } else if (m < 0) {
            rightMotors(10, directionInverter(Direction::BACK, invert));
            leftMotors(10, directionInverter(Direction::FRONT, invert));
            while (m <= -5) {
                m = angleToWall(theta1, theta2);
                printText(String(m));
            }
        }
        stop();
    }

    void calibrateToWall(Direction d) {
        switch (d) {
            case FRONT:
                calibrateToWall(170, 190, false);
                break;
            case RIGHT:
                calibrateToWall(260, 280, true);
                break;
            case LEFT:
                calibrateToWall(80, 100, true);
                break;
            default:
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

        if (d.quality == 15 && d.distance < 6000 && d.distance > 150 && (d.distance < 3900 || d.distance > 4100) && (d.angle < 123 || d.angle > 125)) {
            if (d.angle < 360 && angleDistance(previousAngle, d.angle) < stde + e) {  // make sure that the angle can never be above 360 or else program dies
                pointMemory[(uint16_t)(d.angle * 10)] = d.distance;
                return true;
            } else
                e += stde;
        } else
            e += stde;

        //possible place for integer overflow, if e gets too big, set it back to 0
        if (e > 300000) {
            e = 0;
        }
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
            analogWrite(MOTOCTL, 0);  // stop the rplidar motor

            printText(F("lost connection to lidar"));
            delay(1000);
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

    uint16_t medianDistance(uint16_t theta1, uint16_t theta2) {
        // the idea of this function is to find the distance from the theta
        // inbetween theta1 and 2 being a median, this will be more noise resistant

        RunningMedian r = RunningMedian(abs(theta2 * 10 - theta1 * 10)); //max size of median is 255 we are setting this usually of size 200 //allocates total of 1kb of ram

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

        sensors_event_t lux;

        fullScan(300);
        uint16_t di = medianDistance(170, 190);  // initial distance
        while (di == 0) {
            fullScan(300);
            di = medianDistance(170, 190);
        }
        uint16_t dt = di;
        while (abs(dt - di) < 300) {
            rightMotors(15, Direction::FRONT);
            leftMotors(15, Direction::FRONT);
            fullScan(300);
            dt = medianDistance(170, 190);

            //luxSensor.getEvent(&lux);
            /*if (lux.light > LUX_CONSTANT) { // set the point attempted to traverse to direction as false and then reverses to origional position
                while (dt > di) {
                    dt = medianDistance(170,190);
                    rightMotors(10,Direction::BACK);
                    rightMotors(10,Direction::BACK);
                }
                stop();
                return false;
            }*/

            printText("change in d: " + String(abs(dt - di)));

        }
        stop();
        return true;
    }

    void turn90DegRight(boolean invert) {
        rightMotors(30, directionInverter(Direction::BACK, invert));
        leftMotors(30, directionInverter(Direction::FRONT, invert));
        delay(7000);  // about the time it takes for it to make a turn
        stop();

        if (readVl(Direction::FRONT) < 150) {
            printText(F("calibrating to wall in front"));
            delay(2000);
            fullScan(400);
            calibrateToWall(Direction::FRONT);
        } else if (readVl(Direction::RIGHT) < 150) {
            printText(F("calibrating to wall to right"));
            delay(2000);
            fullScan(400);
            calibrateToWall(Direction::RIGHT);
        } else if (readVl(Direction::LEFT) < 150) {
            printText(F("calibrating to wall in left"));
            delay(2000);
            fullScan(400);
            calibrateToWall(Direction::LEFT);
        } 
        stop();
    }

    void rightMotors(uint8_t speed, Direction d) {
        rightMotorSpeed = speed;
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
        leftMotorSpeed = speed;
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
        imu.read(); 
        sensors_event_t a, m, g, temp;

        imu.getEvent(&a, &m, &g, &temp); 
        return (180 * atan2(a.acceleration.x, sqrt(a.acceleration.y*a.acceleration.y + a.acceleration.z*a.acceleration.z))) / PI;
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
        //sensors_event_t event;
        //luxSensor.getEvent(&event);
        //LUX_CONSTANT = event.light;

        //printText(F("luxSensor ok"));
/*
        if (!imu.begin()) {
            printText(F("imu failed"));
            while(1);
        }
        printText(F("imu ok"));*/

        pinMode(EN1, OUTPUT);
        pinMode(PH1, OUTPUT);
        pinMode(EN2, OUTPUT);
        pinMode(PH2, OUTPUT);

        pinMode(SOLENOID_PIN, OUTPUT);

        Serial.begin(9600);
        lidar.begin(Serial1);

        pinMode(MOTOCTL, OUTPUT);
        analogWrite(MOTOCTL, 255);

        //push strartpoint into stack and nodeMap
        addpoint();
        hStack.push(*nodeMap.get(x,y));
    }
    // entry point for the entire robot

    void addpoint() {
        Node n;
        n.x = x;
        n.y = y;
        n.leftNode = NULL;
        n.rightNode = NULL;
        n.down = readVl(Direction::BACK) > 150 ? true : false;
        n.left = readVl(Direction::LEFT) > 140 ? true : false;
        n.up = readVl(Direction::FRONT) > 150 ? true : false;
        n.right = readVl(Direction::RIGHT) > 150 ? true : false;
        nodeMap.put(n);
    }

    void move(Direction d) {
        // traverse the robot to the geographical direction of the maze (not relative right to the robot)
        //  f r b l
        //  0 1 2 3

        // calculate the amount of turns to take and in which direction
        int k = 0;
        bool tD = false;
        if (d - currentDirection == -1 || d - currentDirection == -3) {
            // turn right once
            k = 1;
        } else if (abs(d - currentDirection) == 2) {
            k = 2;
        } else if (d - currentDirection == 1 || d - currentDirection == 3) {
            // turn left once
            k = 1;
            tD = true;
        }

        for (uint8_t i = 0; i < k; i++) {
            turn90DegRight(tD);
        }
        currentDirection = d;

        travel30cm();

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
    }
    
    void reversal() {
        //calculate and execute shortest path to target through stack

        // fill in temporary storage array with full stack reversal then calculate shortcuts
        Node* nT[50];
        uint8_t id = 0;

        Node* shortcut = hStack.peekLast();

        Node* nC = hStack.peekLast();

        do {
            nC = hStack.pop();
            
            nT[id] = nC;
            id++;
            //check optimal node to start off
            if ((nC->y == y && nC->x == x+1) || (nC-> y == y && nC->x == x-1) || (nC->x == x && nC->y == y+1) || (nC->x == x && nC->y == y-1)) { 
                shortcut = nC;
            }
        //goofy conditional -> loop while nc adjacent node does not exist and that node would be accesible from nc
        // stop when we have an adjacent node that has not been explored and is accesible from nc
        } while (hStack.size() > 0 && !((!nodeMap.contains(nC->x + 1,nC->y) && nC->right) || (!nodeMap.contains(nC->x -1, nC->y) && nC->left) || (!nodeMap.contains(nC->x,nC->y + 1) && nC->up) || (!nodeMap.contains(nC->x,nC->y -1) && nC->down)));
        
        printText(String(id-1) + "items in nT");
        delay(1000);
        printText("shortcut :" + String(shortcut->x) + ", " + String(shortcut->y));
        delay(3000);

        hStack.push(*nT[id-1]); //this is to update the stack so it actually registers the target square
        
        bool trg = true;
        for (uint8_t i = 0; i<id; i++) {
            if (trg && nT[i] != shortcut) {
                continue;
            } 
            trg = false;

            if (y == nT[i]->y && x+1 == nT[i]->x) {
                move(Direction::RIGHT);
            } else if (y == nT[i]->y && x-1 ==nT[i]->x) {
                move(Direction::LEFT);
            } else if (x == nT[i]->x && y+1 == nT[i]->y) {
                move(Direction::FRONT);
            } else if (x == nT[i]->x && y-1 == nT[i]->y) {
                move(Direction::BACK);
            }
        }
    }

    void run() {
        printText(F("iterating"));
        delay(1000);
        if (!nodeMap.contains(x, y)) {
            addpoint();
            printText("point added!" + String(x) + " " + String(y));
            delay(1000);
        }  

        Node* currentNode = nodeMap.get(x, y);

        if (currentNode->right && !nodeMap.contains(x+1,y)) {
            printText(F("moving absolute right"));
            delay(1000);
            move(Direction::RIGHT);
            hStack.push(*nodeMap.get(x,y));
        } else if (currentNode->down && !nodeMap.contains(x,y-1)) {
            printText(F("moving absolute back"));
            delay(1000);
            move(Direction::BACK);
            hStack.push(*nodeMap.get(x,y));
        } else if (currentNode->left && !nodeMap.contains(x-1,y)) {
            printText(F("moving absolute left"));
            delay(1000);
            move(Direction::LEFT);
            hStack.push(*nodeMap.get(x,y));
        } else if (currentNode->up && !nodeMap.contains(x,y+1)) {
            printText(F("moving absolute Front"));
            delay(1000);
            move(Direction::FRONT);
            hStack.push(*nodeMap.get(x,y));
        } else if (hStack.size() > 0) {
            printText(F("reversing"));
            delay(1000);
            printText("stack size: " + String(hStack.size()));
            delay(5000);
            reversal();
        }
        stop();
    }

    void methodTester() {
        turn90DegRight(false);
        delay(10000);
    }
};

Robot robot;

void setup() { robot.startup(); }

void loop() {
    robot.run();  
}
