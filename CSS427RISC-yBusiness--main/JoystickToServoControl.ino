#include <Servo.h> 
//LEADER GLOBAL DATA
static int VRx = A0; //joystick X output pin
static int VRy = A1; //joystick Y output pin
static int SW = 2; //joystick button state pin

//FOLLOWER GLOBAL DATA
Servo rightAileron; //servo object for the right aileron (flap)
Servo leftAileron; //servo object for the left aileron (flap) 
static int leftAileronPin = 8; //left aileron servo pin
static int rightAileronPin = 9; //right aileron servo pin
static int servoStartPos = 90; //starting postion for the servos 

//DATA COLLECTION NEEDED ON LEADER ONLY
int xPosition = 0;
int yPosition = 0;
int SW_state = 0;

//DATA COLLECTION TO BE COLLECTED ON LEADER AND SENT TO FOLLOWER
int leftAileronMap = 0;
int rightAileronMap = 0;

void setup() {

  //Setup for the servos (needed on follower only)
  leftAileron.attach(leftAileronPin);
  rightAileron.attach(rightAileronPin);
  rightAileron.write(servoStartPos);
  leftAileron.write(servoStartPos);

  //Setup for the joystick
  pinMode(VRx, INPUT);
  pinMode(VRy, INPUT);
  pinMode(SW, INPUT_PULLUP);

}

void loop() {
  xPosition = analogRead(VRx); //read xPosition from joystick pin
  //yPosition = analogRead(VRy); //future servo collection
  SW_state = digitalRead(SW); //read button state from joystick

  leftAileronMap = map(xPosition, 0, 1023, 180, 0); //map left aileron (flap) position
  rightAileronMap = map(xPosition, 0, 1023, 180, 0); //map right aileron (flap) position

  //Write to the servos the mapped position from above
  rightAileron.write(rightAileronMap);
  leftAileron.write(leftAileronMap);

}
