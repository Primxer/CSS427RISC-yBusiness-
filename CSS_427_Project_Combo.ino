//ARDUINO UNO CODE

#if defined (ARDUINO_AVR_UNO)
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_MPL3115A2.h>

#define leftAileronPin 2//left aileron servo pin
#define rightAileronPin 4 //right aileron servo pin
#define elevatorPin 3 //elevator servo pin
#define rudderPin 5
#define ESCPin 6
#define leftAileronStart 90
#define rightAileronStart 90
#define rudderStart 90
#define elevatorStart 90

struct DATA_Package {
  byte VR1x1_pos;
  byte VR1x2_pos;
  byte VR1y_pos;
  byte VR2y_pos;
  byte VR2x_pos;
  byte VR1sw_val;
  bool altRequest;
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

RF24 radio(7, 8); // CE, CSN
const byte addresses [][6] = {"00001", "00002"};

DATA_Package data;  //data is the package that will be read into via RF24

void setup() {
  if(!baro.begin()) { //start the baromiter for altitude and tempurature 
  }

  Serial.begin(9600);
  
  //setup radio
  radio.begin();
  radio.openWritingPipe(addresses[0]);
  radio.openReadingPipe(1, addresses[1]);
  radio.setPALevel(RF24_PA_MIN);
  radio.setAutoAck(1);
  radio.enableAckPayload();
  
  leftAileron.attach(leftAileronPin);
  rightAileron.attach(rightAileronPin);
  elevator.attach(elevatorPin);
  rudder.attach(rudderPin);
  ESC.attach(ESCPin, 1000, 2000);
  
  rudder.write(rudderStart);
  elevator.write(elevatorStart);
  leftAileron.write(leftAileronStart);
  rightAileron.write(rightAileronStart);
  
}

void loop() {
  //Read Radio transmission
  delay(2);
  radio.startListening();
  if (radio.available()) {
    radio.read(&data, sizeof(DATA_Package));

    //Write to all the servos 
    elevator.write(data.VR1y_pos);
    leftAileron.write(data.VR1x1_pos);
    rightAileron.write(data.VR1x2_pos);
    rudder.write(data.VR2x_pos); 
    ESC.write(data.VR2y_pos);

    lastReceiveTime = millis();
  }
  delay(2);

  //Send Radio Transmission
  radio.stopListening();
  if(data.altRequest) { //THIS WILL NEED TO CHECK ALL SENSORS WITH || TO SEND DATA REQUEST ACK
    char ackMessage[20] = "Data Request ACK";
    radio.write(&ackMessage, sizeof(ackMessage));
  }

  //Check each sensor for request KEEP CHECKS IN ORDER BETWEEN LEADER/FOLLOWER
  if(data.altRequest) {
    //float altitude = baro.getAltitude();
    //radio.write(&altitude, sizeof(altitude));
  }

  //ADD OTHER SENSOR LOGIC HERE


  //Check if radio lost signal
  currentTime = millis();
  if ( currentTime - lastReceiveTime > LOST_CONNECTION_TIME ){
    lostConnection();
  }
}

void print_data() {
  Serial.print(data.VR1y_pos);
  Serial.print(", ");
  Serial.println(data.VR1x1_pos);
}

void lostConnection(){
  return;
}
#endif

//ARDUINO MEGA CODE

#if defined(__AVR_ATmega2560__)
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_MPL3115A2.h>

struct DATA_Package {
  byte VR1x1_pos;
  byte VR1x2_pos;
  byte VR1y_pos;
  byte VR2y_pos;
  byte VR2x_pos;
  byte VR1sw_val;
  bool altRequest;
};

static int VR1x = A0;
static int VR1y = A1; //Joystick 1 y axis
static int VR2x = A2;
static int VR2y = A3;
static int VR1sw = 22; //Joystick 1 button

const int LOST_CONNECTION_TIME = 1000;

bool sampleType = false;

int sampleRate = 0;
unsigned long lastAltTime = 0;
unsigned long lastReceiveTime = 0;
unsigned long currentTime = 0;

long int terminalCount = 0;
long int leaderToFollowerCount = 0;
long int followerToLeaderCount = 0;

int altitude = 0;
int tempC = 0;

DATA_Package data;  //data is the package that will be sent via RF24

RF24 radio(49, 48); // CE, CSN
const byte addresses [][6] = {"00001", "00002"};

void setup() { 
  Serial.begin(9600);
  
  //setup radio
  radio.begin();
  radio.openWritingPipe(addresses[1]);
  radio.openReadingPipe(1, addresses[0]);
  radio.setPALevel(RF24_PA_MIN);
  radio.setAutoAck(1);
  radio.enableAckPayload();

  //initialise data
  data.VR1x1_pos = 0;
  data.VR1x2_pos = 0;
  data.VR1y_pos = 0;
  data.VR1sw_val = 0;
  data.altRequest = false;

  //pinMode
  pinMode(VR1x, INPUT);
  pinMode(VR1y, INPUT);
  pinMode(VR2x, INPUT);
  pinMode(VR2y, INPUT);
  pinMode(VR1sw, INPUT_PULLUP);

  //User request for manual or periodic sample rate
  Serial.println("Which sample type? 1 = manual, 0 = periodic");
  //while(Serial.available() == 0) {}
  char input = Serial.read();
  if(input == '0') {
    sampleType = true;
    Serial.flush();
    Serial.read();
    Serial.println("Enter Sampling Rate (microseconds)");
    Serial.flush();
    Serial.read();
    while(Serial.available() == 0) {}
    sampleRate = Serial.parseInt();

    //TODO: ADD SAMPLE RATE REQUESTS FOR EACH SENSOR
    
  } else {
    if(input != '1') {
      Serial.println("Unknown input, defaulting to manual requests...");
    }
    Serial.println("Input 'y' into monitor for each request");
  }
  terminalCount += 1;
}
void loop() {

  //IF user input y and manual sample then send data request for ALL sensors 
  if(Serial.read()== 'y' && !sampleType) {
    data.altRequest = true;
    Serial.println("Data Request Sent");
    terminalCount+=1;
    Serial.print("Terminal Input Count: "); Serial.println(terminalCount);
  }

  //Read Input
  float xIn = analogRead(VR1x);
  data.VR1x1_pos = map(xIn, 0, 1023, 155, 45);
  data.VR1x2_pos = map(xIn, 0, 1023, 155, 40);
  data.VR1y_pos = map(analogRead(VR1y), 0, 1023, 55, 125);
  data.VR2x_pos = map(analogRead(VR2x), 0, 1023, 45, 135);
  data.VR2y_pos = map(constrain(analogRead(VR2y), 512, 1023), 512, 1023, 0, 90);

  //Send Radio Transmission
  delay(2);
  radio.stopListening();
  radio.write(&data, sizeof(DATA_Package));
  leaderToFollowerCount += 1;
  delay(2);
  //End Send Radio Transmission

  //Read Radio Transmission
  radio.startListening();
  if(radio.available()) {
    //Collect ACK message if exists
    radio.read(&data, sizeof(data));
    char ackMessage[20] = "a";
    radio.read(&ackMessage, sizeof(ackMessage));
    Serial.println(ackMessage);
    followerToLeaderCount += 1;
    Serial.print("Follower->Leader Count: "); Serial.println(followerToLeaderCount);

    //Reset all sensor requests and read sensor data from follower KEEP RESETS IN ORDER BETWEEN LEADER/FOLLOWER
    if(data.altRequest) {
      //radio.read(&altitude, sizeof(altitude));
      data.altRequest = false;
      followerToLeaderCount += 1;
      //Serial.print(altitude); Serial.println(" meters");
    }

    //ADD SENSOR LOGIC HERE
    
    lastReceiveTime = millis();
  }
  //End Read Radio Transmission

  print_data();
  
  //Check if radio has lost signal
  currentTime = millis();
  
  if ( currentTime - lastReceiveTime > LOST_CONNECTION_TIME ){
    lostConnection();
  }

  //If in periodic mode check each sensor for if needed sample
  //WILL ADD MORE TO THIS ONE AS WE GET MORE SENSORS WORKING
  if(sampleType) {
    if(currentTime - lastAltTime > sampleRate) {
      data.altRequest = true;
      lastAltTime = millis();
    }
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
