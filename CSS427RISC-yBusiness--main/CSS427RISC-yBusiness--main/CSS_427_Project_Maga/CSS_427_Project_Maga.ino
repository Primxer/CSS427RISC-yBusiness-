#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"

static int VR1x = A0;
static int VR1y = A1; //Joystick 1 y axis
static int VR1sw = 22; //Joystick 1 button

const bool TEST_MODE = true;// will print out data to serial monitor

struct DATA_Package {
  byte VR1x_pos;
  byte VR1y_pos;
  byte VR1sw_val;
};
DATA_Package data;  //data is the package that will be sent via RF24

RF24 radio(49, 48); // CE, CSN
const byte address[6] = "00001";

void setup() { 
  if(TEST_MODE){
    Serial.begin(9600);
  }
  
  //setup radio
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();  //do this before calling write()

  //initialise data
  data.VR1x_pos = 0;
  data.VR1y_pos = 0;
  data.VR1sw_val = 0;

  //pinMode
  pinMode(VR1x, INPUT);
  pinMode(VR1y, INPUT);
  pinMode(VR1sw, INPUT_PULLUP);
  
}
void loop() {
  //read and set data
  data.VR1x_pos = map(analogRead(VR1x), 0, 1023, 0, 180);
  data.VR1y_pos = map(analogRead(VR1y), 0, 1023, 0, 180);
  data.VR1sw_val = digitalRead(VR1sw);
  
  radio.write(&data, sizeof(DATA_Package));

  if(TEST_MODE){
    print_data();
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
