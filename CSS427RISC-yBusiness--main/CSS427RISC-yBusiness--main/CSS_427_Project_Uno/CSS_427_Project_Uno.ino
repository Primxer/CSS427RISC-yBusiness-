 #include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_MPL3115A2.h>

#define leftAileronPin 2//left aileron servo pin
#define rightAileronPin 4 //right aileron servo pin
#define elevatorPin 3 //elevator servo pin

unsigned long lastReceiveTime = 0;
unsigned long currentTime = 0;

const bool TEST_MODE = false; // will print out data to serial monitor
const int LOST_CONNECTION_TIME = 1000; // last connected is over 1sec


Servo leftAileron; 
Servo rightAileron;
Servo elevator;

Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2();

RF24 radio(7, 8); // CE, CSN
const byte address[6] = "00001";

struct DATA_Package {
  byte VR1x_pos;
  byte VR1y_pos;
  byte VR1sw_val;
};

DATA_Package data;  //data is the package that will be read into via RF24

void setup() {
  if(TEST_MODE){
    Serial.begin(9600);
  }

  Serial.begin(9600);
  if(!baro.begin()) {
  }
  
  //setup radio
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening(); //do this before calling read()

  leftAileron.attach(leftAileronPin);
  rightAileron.attach(rightAileronPin);
  elevator.attach(elevatorPin);
  elevator.write(90);
  leftAileron.write(90);
  rightAileron.write(90);
}

void loop() {
  if (radio.available()) {
    radio.read(&data, sizeof(DATA_Package));

    elevator.write(data.VR1y_pos);
    leftAileron.write(data.VR1x_pos);
    rightAileron.write(data.VR1x_pos);
      
    if(TEST_MODE){
      print_data();
    }
  }

  float altm = baro.getAltitude();
  Serial.print(altm); Serial.println(" meters");

  float tempC = baro.getTemperature();
  Serial.print(tempC); Serial.println("*C");
  
  currentTime = millis();
  if ( currentTime - lastReceiveTime > LOST_CONNECTION_TIME ){
    lostConnection();
  }
}

void print_data(){
  Serial.print("Pod_1: ");
  Serial.print(data.VR1x_pos);
  Serial.print(", ");
  Serial.print(data.VR1y_pos);
  Serial.print(", ");
  Serial.println(data.VR1sw_val);
}

void lostConnection(){
  return;
}
