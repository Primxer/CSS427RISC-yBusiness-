//ARDUINO UNO CODE

#if defined (ARDUINO_AVR_UNO)
#include <SPI.h>
#include "nRF24L01.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "RF24.h"
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_MPL3115A2.h>
#include <Math.h>

#define leftAileronPin 9//left aileron servo pin
#define rightAileronPin 4 //right aileron servo pin
#define elevatorPin 3 //elevator servo pin
#define rudderPin 5
#define ESCPin 6
#define TMP36Pin A0
#define leftAileronStart 90
#define rightAileronStart 90
#define rudderStart 90
#define elevatorStart 90
#define batPin A2

struct DATA_Package {
  short yaw = 0;
  short pitch = 0;
  short roll = 0;
  short altitude;
  byte tempC = 0;
  byte VR1x1_pos;
  byte VR1y_pos;
  byte VR2y_pos;
  byte VR2x_pos;
  byte VR1sw_val;
  byte batteryLevel = 0;
};

unsigned long lastReceiveTime = 0;
unsigned long currentTime = 0;

const int LOST_CONNECTION_TIME = 1000; // last connected is over 1sec
bool localAltRequest = false;


Servo leftAileron;
Servo rightAileron;
Servo elevator;
Servo rudder;
Servo ESC;

Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2();


MPU6050 mpu;
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}


RF24 radio(7, 8); // CE, CSN
const byte addresses [][6] = {"00001", "00002"};

DATA_Package data;  //data is the package that will be read into via RF24

void setup() {
  Wire.begin();

  if (!baro.begin()) { //start the baromiter for altitude and tempurature
  }

  Serial.begin(9600);

  //setup radio
  radio.begin();
  radio.openWritingPipe(addresses[0]);
  radio.openReadingPipe(1, addresses[1]);
  radio.setPALevel(RF24_PA_LOW);

  leftAileron.attach(leftAileronPin);
  rightAileron.attach(rightAileronPin);
  elevator.attach(elevatorPin);
  rudder.attach(rudderPin);
  ESC.attach(ESCPin, 1000, 2000);

  rudder.write(rudderStart);
  elevator.write(elevatorStart);
  leftAileron.write(leftAileronStart);
  rightAileron.write(rightAileronStart);

  mpu.initialize();
  devStatus = mpu.dmpInitialize();
  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(355);
  mpu.setYGyroOffset(-90);
  mpu.setZGyroOffset(0);
  mpu.setZAccelOffset(8810); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    //Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    //Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    //Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

void loop() {
  //Read Radio transmission
  delay(3);
  radio.startListening();
  if (radio.available()) {
    while (radio.available()) {
      radio.read(&data, sizeof(DATA_Package));
    }

    lastReceiveTime = millis();
  }
  delay(3);

  //Write to all the servos
  elevator.write(data.VR1y_pos);
  leftAileron.write(data.VR1x1_pos);
  rightAileron.write(data.VR1x1_pos);
  rudder.write(data.VR2x_pos);
  ESC.write(data.VR2y_pos);

  //Send Radio Transmission
  radio.stopListening();

  //Check each sensor for request KEEP CHECKS IN ORDER BETWEEN LEADER/FOLLOWER
  int reading = analogRead(TMP36Pin);
  float voltage = reading * 5.0;
  voltage /= 1024.0;
  byte tempC = (byte) round((voltage - 0.5) * 100);
  data.tempC = tempC;

  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {
  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    //Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    data.yaw = (short)round(ypr[0] * 180 / M_PI);
    data.roll = (short)round(ypr[1] * 180 / M_PI);
    data.pitch = (short)round(ypr[2] * 180 / M_PI);
  }

  data.batteryLevel = (byte)map(analogRead(batPin), 622, 792, 0, 100);

  //data.altitude = baro.getAltitude();

  radio.write(&data, sizeof(data));

  //Check if radio lost signal
  currentTime = millis();
  if ( currentTime - lastReceiveTime > LOST_CONNECTION_TIME ) {
    lostConnection();
  }
}

void print_data() {
  Serial.print(data.VR1y_pos);
  Serial.print(", ");
  Serial.println(data.VR1x1_pos);
}

void lostConnection() {
  return;
}
#endif


// ================================================================
// ===               ARDUINO MEGA CODE                          ===
// ================================================================

#if defined(__AVR_ATmega2560__)
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_MPL3115A2.h>
#include <Adafruit_RGBLCDShield.h>
#include <utility/Adafruit_MCP23017.h>
#include <Math.h>

struct DATA_Package {
  short yaw = 0;
  short pitch = 0;
  short roll = 0;
  short altitude;
  byte tempC = 0;
  byte VR1x1_pos;
  byte VR1y_pos;
  byte VR2y_pos;
  byte VR2x_pos;
  byte VR1sw_val;
  byte batteryLevel = 0;
};

bool autoLevelOn = false;
bool MPURequest = false;
bool batteryRequest = false;
bool tempRequest = false;
bool altitudeRequest = false;

static int VR1x = A0;
static int VR1y = A1; //Joystick 1 y axis
static int VR2x = A2;
static int VR2y = A3;
static int VR1sw = 22; //Joystick 1 button
static int autolevelPin = 2;

const int LOST_CONNECTION_TIME = 1000;

bool sampleType = false;

int MPUSampleRate = 0;
int batterySampleRate = 0;
int tempSampleRate = 0;
int altitudeSampleRate = 0;

unsigned long lastTempTime = 0;
unsigned long lastMPUTime = 0;
unsigned long lastBatteryTime = 0;
unsigned long lastAltitudeTime = 0;
unsigned long lastReceiveTime = 0;
unsigned long currentTime = 0;

volatile int maxThrottle = 100;

long int terminalCount = 0;
long int leaderToFollowerCount = 0;
long int followerToLeaderCount = 0;

DATA_Package data;  //data is the package that will be sent via RF24

Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();

RF24 radio(49, 48); // CE, CSN
const byte addresses [][6] = {"00001", "00002"};

void setup() {
  Serial.begin(9600);
  Wire.begin();

  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);

  //setup radio
  radio.begin();
  radio.openWritingPipe(addresses[1]);
  radio.openReadingPipe(1, addresses[0]);
  radio.setPALevel(RF24_PA_LOW);

  //initialise data
  data.VR1x1_pos = 0;
  data.VR1y_pos = 0;
  data.VR1sw_val = 0;

  //pinMode
  pinMode(VR1x, INPUT);
  pinMode(VR1y, INPUT);
  pinMode(VR2x, INPUT);
  pinMode(VR2y, INPUT);
  pinMode(VR1sw, INPUT_PULLUP);

  //User request for manual or periodic sample rate
  Serial.println("Which sample type? 1 = manual, 0 = periodic");
  while (Serial.available() == 0) {}
  char input = Serial.read();
  if (input == '0') {
    sampleType = true;
    Serial.flush();
    Serial.read();
    Serial.println("Enter Sampling Rate for Temperature sensor (microseconds)");
    Serial.flush();
    Serial.read();
    while (Serial.available() == 0) {}
    tempSampleRate = Serial.parseInt();
    Serial.println("Enter Sampling Rate for Gyroscope sensor (microseconds)");
    Serial.flush();
    Serial.read();
    while (Serial.available() == 0) {}
    MPUSampleRate = Serial.parseInt();
    Serial.println("Enter Sampling Rate for Battery Percentage sensor (microseconds)");
    Serial.flush();
    Serial.read();
    while (Serial.available() == 0) {}
    batterySampleRate = Serial.parseInt();
    Serial.println("Enter Sampling Rate for Altitude sensor (microseconds)");
    Serial.flush();
    Serial.read();
    while (Serial.available() == 0) {}
    altitudeSampleRate = Serial.parseInt();
    Serial.flush();
    Serial.read();
  } else {
    if (input != '1') {
      Serial.println("Unknown input, defaulting to manual requests...");
    }
    Serial.println("Input 'y' into monitor for each request");
  }
  terminalCount += 1;
}
void loop() {
  //IF user input y and manual sample then send data request for ALL sensors
  if (Serial.read() == 'y' && !sampleType) {
    Serial.flush();
    Serial.read();
    tempRequest = true;
    MPURequest = true;
    batteryRequest = true;
    altitudeRequest = true;
    Serial.println("Data Request Sent");
    terminalCount += 1;
    Serial.print("Terminal Input Count: "); Serial.println(terminalCount);
    Serial.flush();
    Serial.read();
  }

  //Read Input
  autoLevelOn = digitalRead(autolevelPin);
  if (!autoLevelOn) {
    float xIn = analogRead(VR1x);
    data.VR1x1_pos = map(xIn, 0, 1023, 55, 120);
    data.VR1y_pos = map(analogRead(VR1y), 0, 1023, 125, 55);
    data.VR2x_pos = map(analogRead(VR2x), 0, 1023, 45, 100);
    data.VR2y_pos = map(constrain(analogRead(VR2y), 512, 1023), 512, 1023, 0, maxThrottle);
  }
  else {
    data.VR2x_pos = map(constrain(data.yaw, -45, 45), -45, 45, 45, 135); //yaw true range is -180 to 180. contrain down to increase sens
    data.VR1x1_pos = map(data.roll, -45, 45, 155, 45);
    data.VR1y_pos = map(data.pitch, -45, 45, 55, 125);
  }

  //Send Radio Transmission
  delay(5);
  radio.stopListening();
  radio.write(&data, sizeof(DATA_Package));
  if(radio.isAckPayloadAvailable()) {
    leaderToFollowerCount += 1;
    //Serial.print("Transmission Number: "); Serial.print(leaderToFollowerCount);Serial.println(" Data ACK");
  }
  delay(5);
  //End Send Radio Transmission

  //Read Radio Transmission
  radio.startListening();
  if (radio.available()) {
    while (radio.available()) {
      radio.read(&data, sizeof(data));
    }

    //Reset all sensor requests and read sensor data from follower KEEP RESETS IN ORDER BETWEEN LEADER/FOLLOWER
    if (tempRequest) {
      tempRequest = false;
      followerToLeaderCount += 1;
      String printTemp = String(data.tempC) + " C  ";
      Serial.println(printTemp);
      lcd.setCursor(7, 1);
      lcd.print(printTemp);
    }

    if (MPURequest) {
      MPURequest = false;
      followerToLeaderCount += 3;
      String printYPR = "YPR " + String(data.yaw) + " " + String(data.roll) + " " + String(data.pitch) + "     ";
      Serial.println(printYPR);
      lcd.setCursor(0, 0);
      lcd.print(printYPR);
    }

    if (batteryRequest) {
      batteryRequest = false;
      followerToLeaderCount += 1;
      String printBatt = String(data.batteryLevel) + "%";
      Serial.println(printBatt);
      lcd.setCursor(14, 1);
      lcd.print(printBatt);
    }
    lastReceiveTime = millis();
  }
  //End Read Radio Transmission

  //Check if radio has lost signal
  currentTime = millis();

  if ( currentTime - lastReceiveTime > LOST_CONNECTION_TIME ) {
    lostConnection();
  }

  //If in periodic mode check each sensor for if needed sample
  if (sampleType) {
    if (currentTime - lastTempTime > tempSampleRate) {
      tempRequest = true;
      lastTempTime = millis();
    }
    if (currentTime - lastMPUTime > MPUSampleRate) {
      MPURequest = true;
      lastMPUTime = millis();
    }
    if (currentTime - lastBatteryTime > batterySampleRate) {
      batteryRequest = true;
      lastBatteryTime = millis();
    }
    if (currentTime - lastAltitudeTime > altitudeSampleRate) {
      altitudeRequest = true;
      lastAltitudeTime = millis();
    }
  }

  if (data.tempC > 30) {
    maxThrottle = 60;
  } else {
    maxThrottle = 90;
  }
}

void print_data() {
  Serial.print(data.VR1y_pos);
  Serial.print(", ");
  Serial.println(data.VR1x1_pos);
}

void lostConnection() {
  return;
}
#endif
