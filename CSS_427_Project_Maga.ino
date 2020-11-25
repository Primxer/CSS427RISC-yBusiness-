#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"

static int VR1x = A0;
static int VR1y = A1; //Joystick 1 y axis
static int VR1sw = 22; //Joystick 1 button

const bool TEST_MODE = false;// will print out data to serial monitor
const int LOST_CONNECTION_TIME = 1000;
bool sampleType = false;

unsigned long lastReceiveTime = 0;
unsigned long currentTime = 0;

int altitude = 0;
int tempC = 0;

struct DATA_Package {
  byte VR1x_pos;
  byte VR1y_pos;
  byte VR1sw_val;
  bool altRequest;
  bool tempCRequest;
};
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

  //initialise data
  data.VR1x_pos = 0;
  data.VR1y_pos = 0;
  data.VR1sw_val = 0;
  data.altRequest = false;
  data.tempCRequest = false;

  //pinMode
  pinMode(VR1x, INPUT);
  pinMode(VR1y, INPUT);
  pinMode(VR1sw, INPUT_PULLUP);

  Serial.println("Which sample type? 1 = manual, 0 = periodic");
  while(Serial.available() == 0);
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
  
  //Send Radio Transmission
  data.VR1x_pos = map(analogRead(VR1x), 0, 1023, 0, 180);
  data.VR1y_pos = map(analogRead(VR1y), 0, 1023, 0, 180);
  delay(2);
  radio.stopListening();
  data.VR1sw_val = digitalRead(VR1sw); 
  
  radio.write(&data, sizeof(DATA_Package));
  delay(2);

  //Read Radio Transmission
  radio.startListening();
  if(radio.available()) {
    if(data.altRequest) {
      radio.read(&altitude, sizeof(altitude));
      data.altRequest = false;
      Serial.print(altitude); Serial.println(" meters");
    }
    if(data.tempCRequest) {
      radio.read(&tempC, sizeof(altitude));
      data.tempCRequest = false;
    }

    lastReceiveTime = millis();
  }

  if(TEST_MODE){
    print_data();
  }

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
  Serial.print(data.VR1sw_val);
  Serial.print(", ");
  Serial.print(data.altRequest);
  Serial.print(", ");
  Serial.println(data.tempCRequest);
}

void lostConnection() {
  return;
}
