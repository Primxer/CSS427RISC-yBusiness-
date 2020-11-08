#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>

#define leftAileronPin 2 //left aileron servo pin

unsigned long lastReceiveTime = 0;
unsigned long currentTime = 0;

const bool TEST_MODE = true; // will print out data to serial monitor
const int LOST_CONNECTION_TIME = 1000; // last connected is over 1sec

Servo leftAileron; 

RF24 radio(7, 8); // CE, CSN
const byte address[6] = "00001";

struct DATA_Package {
  byte VR1x_pos;
  byte VR1y_pos;
  bool VR1sw_val;
};
DATA_Package data;  //data is the package that will be read into via RF24

void setup() {
  if(TEST_MODE){
    Serial.begin(9600);
  }

  //setup radio
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening(); //do this before calling read()

  leftAileron.attach(leftAileronPin);
}

void loop() {
  if (radio.available()) {
    radio.read(&data, sizeof(DATA_Package));
    lastReceiveTime = millis();

    leftAileron.write(data.VR1x_pos);
    
    if(TEST_MODE){
      print_data();
    }
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
  Serial.println(data.VR1sw_val);
}

void lostConnection(){
  return;
}
