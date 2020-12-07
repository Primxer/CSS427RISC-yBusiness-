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
#define leftAileronStart 60
#define rightAileronStart 66
#define rudderStart 80
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
    while(radio.available()) {
      radio.read(&data, sizeof(DATA_Package));
    }
    
    //elevator.write(data.VR1y_pos);
    //leftAileron.write(data.VR1x1_pos);
    //rightAileron.write(data.VR1x2_pos);
    //rudder.write(data.VR2x_pos); 
    //ESC.write(data.VR2y_pos);

    lastReceiveTime = millis();
  }
  delay(2);

  //Send Radio Transmission
  radio.stopListening();
  if(data.altRequest) {
    float altitude = baro.getAltitude();
    radio.writeFast(&altitude, sizeof(altitude));
    Serial.println("sent alt");
  }
  
  currentTime = millis();
  if ( currentTime - lastReceiveTime > LOST_CONNECTION_TIME ){
    lostConnection();
  }
  print_data();
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

unsigned long lastReceiveTime = 0;
unsigned long currentTime = 0;

int altitude = 0;
int tempC = 0;

DATA_Package data;  //data is the package that will be sent via RF24

RF24 radio(49, 48); // CE, CSN
const byte addresses [][6] = {"00001", "00002"};
char ackMessage[] = "Data Request ";

void setup() { 
  Serial.begin(9600);
  
  //setup radio
  radio.begin();
  radio.openWritingPipe(addresses[1]);
  radio.openReadingPipe(1, addresses[0]);
  radio.setPALevel(RF24_PA_MIN);

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

  Serial.println("Which sample type? 1 = manual, 0 = periodic");
  while(Serial.available() == 0) {}
  char input = Serial.read();
  if(input == '0') {
    sampleType = true;
    Serial.println("Enter Sampling Rate");
  } else {
    if(input != '1') {
      Serial.println("Unknown input, defaulting to manual requests...");
    }
    Serial.println("Input 'y' into monitor for each request");
  }
}
void loop() {

  if(Serial.read()== 'y' && !sampleType) {
    data.altRequest = true;
  }

  //Read Input
  float xIn = analogRead(VR1x);
  data.VR1x1_pos = map(xIn, 0, 1023, 100, 40);
  data.VR1x2_pos = map(xIn, 0, 1023, 90, 40);
  data.VR1y_pos = map(analogRead(VR1y), 0, 1023, 55, 125);
  data.VR2x_pos = map(analogRead(VR2x), 0, 1023, 57, 103);
  data.VR2y_pos = map(constrain(analogRead(VR2y), 512, 1023), 512, 1023, 0, 180);

  //Send Radio Transmission
  delay(2);
  radio.stopListening();
  data.VR1sw_val = digitalRead(VR1sw); 
  
  radio.write(&data, sizeof(DATA_Package));
  delay(2);
  //End Send Radio Transmission

  //Read Radio Transmission
  radio.startListening();
  if(radio.available()) {
    if(data.altRequest) {
      radio.read(&altitude, sizeof(altitude));
      data.altRequest = false;
      Serial.print(altitude); Serial.println(" meters");
    }
    lastReceiveTime = millis();
  }
  //End Read Radio Transmission
  print_data();
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

void lostConnection() {
  return;
}
#endif
