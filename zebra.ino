#include "src/PS2X_lib/PS2X_lib.h"
#include <Servo.h>
#include <SPI.h>

//Define variables
PS2X ps2x; // create PS2 Controller Class

// Configure digital pins to control motors on HG7881-(L9110), a 4 wire controller! (PWM PINS = 3, 5, 6, 9, 10)
int leftDirection = 4; // HG7881_B_IB ( Motor B Direction )
int leftSpeed = 6;   // HG7881_B_IA ( Motor B PWM Speed )

int rightDirection = 2; // HG7881_A_IB ( Motor A Direction )
int rightSpeed = 5; // HG7881_A_IA ( Motor A PWM Speed )

// https://www.bananarobotics.com/shop/How-to-use-the-HG7881-(L9110)-Dual-Channel-Motor-Driver-Module
int PWM_FAST_FWD = 50; // arbitrary fast speed PWM duty cycle
int PWM_SLOW_FWD = 80;  // arbitrary slow speed PWM duty cycle
//tending towards 128
int PWM_SLOW_BWD = 255 - PWM_SLOW_FWD;  // arbitrary slow speed PWM duty cycle
int PWM_FAST_BWD = 255 - PWM_FAST_FWD; // arbitrary fast speed PWM duty cycle

//configure arm pins
int leftArm = 9;
int rightArm = 10;
int armSpeed = 3;

//configure head pins
int headMovement = 3;
int leftEye = A5;
int rightEye = A4;

//default head position
int defaultHeadPos = 85;

//arm position
int leftArmPos = 90;
int rightArmPos = 90; 
 
//joysticks x positions
int joystickLHX = 128;
int joystickLHY; //Define Joystick Left  Hat  Y Variable
int joystickRHX = 128;
int joystickRHY; // Define Joystick Right Hat  X Variable

//http://lagers.org.uk/gamecontrol/api.html
String posRest = "Rest";
String posUpLeft = "Up Left";
String posUp = "Up";
String posUpRight = "Up Right";
String posRight = "Right";
String posDownRight = "Down Right";
String posDown = "Down";
String posDownLeft = "Down Left";
String posLeft = "Left";

Servo headServo;  // create servo object to control the head servo
Servo leftArmServo;  // create servo object to control the arm
Servo rightArmServo;  // create servo object to control the arm servo

int ledState = LOW;             // ledState used to set the LED

// Generally, you should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store
unsigned long previousMillis = 0;        // will store last time LED was updated

const long blinkSpeed = 1000;           // interval at which to blink (milliseconds)

bool serialPrint = false;

void setup() {

  Serial.begin(57600);

  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection

  // Configure digital pins for motor
  pinMode(leftDirection, OUTPUT);
  pinMode(leftSpeed, OUTPUT);
  pinMode(rightDirection, OUTPUT);
  pinMode(rightSpeed, OUTPUT);

  delay(1000);  //added delay to give wireless ps2 module some time to startup, before configuring it

  int error = 1;

  while (1 == 1) {

    error = ps2x.config_gamepad(13, 8, 7, 12, false, false); //setup pins and settings:  GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error

    if (error == 0) {
      print("Found Controller, configured successful");
      break;
    }
    else if (error == 1)
      print("No controller found, check wiring");
    else if (error == 2)
      print("Controller found but not accepting commands.");
    else if (error == 3)
      print("Controller refusing to enter Pressures mode, may not support it. ");

    delay(50);
  }

  int type = ps2x.readType();

  switch (type) {
    case 0:
      print("Unknown Controller type");
      break;
    case 1:
      print("DualShock Controller Found");
      break;
  }

  //connect to servos
  headServo.attach(headMovement);  // attaches the servo on pin (headMovement) to the servo object
  leftArmServo.attach(leftArm);
  rightArmServo.attach(rightArm);

  headServo.write(defaultHeadPos);  // sets the servo position to center the head
  delay(15);            // waits for the servo to get there

  leftArmServo.write(leftArmPos);  // sets the servo position to center
  delay(15);            // waits for the servo to get there

  rightArmServo.write(rightArmPos);  // sets the servo position to center
  delay(15);            // waits for the servo to get there

  pinMode( leftEye, OUTPUT); // light up left eye
  pinMode( rightEye, OUTPUT); //light up right eye
  digitalWrite( leftEye, HIGH);
  digitalWrite( rightEye, HIGH);
  delay(1000);   //leave eyes lit for one second to show working
  digitalWrite( leftEye, LOW);
  digitalWrite( rightEye, LOW);

  digitalWrite(leftDirection, LOW);
  digitalWrite(leftSpeed, LOW);

  digitalWrite(rightDirection, LOW);
  digitalWrite(rightSpeed, LOW);
}


void loop() {

  ps2x.read_gamepad(false, false);       //read controller
  //ps2x.reconfig_gamepad(); //https://stackoverflow.com/questions/46493222/why-arduino-needs-to-be-restarted-after-ps2-controller-communication-in-arduino

  delay(50);

  //movementJoystick();

  headJoystick();

  getButtonClicks();

  winkEye();

  armPosition();

  //print("Head Position on Analog(PSS_RX) @" + String(ps2x.Analog(PSS_RX)));
  //print("Head Position on Analog(PSS_LX) @" + String(ps2x.Analog(PSS_LX)));
}

void winkEye() {

  if (previousMillis > 0) {

    unsigned long currentMillis = millis();

    if (currentMillis - previousMillis >= blinkSpeed) {

      // reset
      previousMillis = 0;

      digitalWrite(rightEye, LOW);
    }
  }
}

void stopMotors() {
  //always stop motors before moving abruptly
  digitalWrite( leftDirection, LOW );
  digitalWrite( leftSpeed, LOW );
  digitalWrite( rightDirection, LOW );
  digitalWrite( rightSpeed, LOW );
}

void getButtonClicks() {
  if (ps2x.ButtonPressed(PSB_CIRCLE  )) //Circle pressed
  {
    print("CIRCLE pressed");

    //only blink if LEDs are off
    if (ledState == LOW) {
      //record time of blink & set LED to on
      previousMillis = millis();

      digitalWrite(rightEye, HIGH);
    }
  }

  stopMotors();

  if (ps2x.Button(PSB_PAD_UP)) {
    moveBackward();
  }

  if (ps2x.Button(PSB_PAD_RIGHT)) {
    moveRight();
  }

  if (ps2x.Button(PSB_PAD_LEFT)) {
    moveLeft();
  }

  if (ps2x.Button(PSB_PAD_DOWN)) {
    moveForward();
  }

  //Dead mans switch - press the xbox button to stop the car
  if (ps2x.Button(PSB_START)) //Start button pressed
  {
    print("START pressed");

    digitalWrite(rightDirection, HIGH);
    digitalWrite(rightSpeed, HIGH);

    digitalWrite(leftDirection, HIGH);
    digitalWrite(leftSpeed, HIGH);

    //print some debugging details
    String RX = String(ps2x.Analog(PSS_RX));
    String RY = String(ps2x.Analog(PSS_RY));

    print(String("STOPPED @ X:" + RX + " Y:" + RY));

    headServo.write(defaultHeadPos);  // sets the servo position to center the head
  }

  if (ps2x.ButtonPressed(PSB_CROSS)) //Cross pressed
  {
    print("CROSS pressed");

    // if the LED is off turn it on and vice-versa:
    if (ledState == LOW) {
      ledState = HIGH;
    } else {
      ledState = LOW;
    }

    // set the LED with the ledState of the variable:
    digitalWrite(leftEye, ledState);
    digitalWrite(rightEye, ledState);
  }
}

void print(String val) {
  if (serialPrint == true) {
    Serial.println(val);
  }
}


void moveForward() {
  digitalWrite(leftDirection, HIGH);   // direction = forward
  analogWrite(leftSpeed, PWM_FAST_FWD);

  digitalWrite(rightDirection, HIGH);   // direction = forward
  analogWrite(rightSpeed, PWM_FAST_FWD);
}

void moveRight() {
  digitalWrite(leftDirection, HIGH);   // direction = forward
  analogWrite(leftSpeed, PWM_FAST_FWD);

  digitalWrite(rightDirection, LOW);   // direction = backwards
  analogWrite(rightSpeed, PWM_FAST_BWD);
}

void moveLeft() {
  digitalWrite(leftDirection, LOW);   // direction = forward
  analogWrite(leftSpeed,  PWM_FAST_BWD);

  digitalWrite(rightDirection, HIGH);   // direction = backwards
  analogWrite(rightSpeed, PWM_FAST_FWD);
}

void moveBackward() {
  digitalWrite(leftDirection, LOW);   // direction = backwards
  analogWrite(leftSpeed,  PWM_FAST_BWD);

  digitalWrite(rightDirection, LOW);   // direction = backwards
  analogWrite(rightSpeed,  PWM_FAST_BWD);
}

/*
  void movementJoystick() {

  int newJoystickRHX =  ps2x.Analog(PSS_RX) - 128;
  int newJoystickRHY = 128 -  ps2x.Analog(PSS_RY) ;

  String pos = "TBD";

  if (abs(newJoystickRHX) < 23) {
    joystickRHX = 0;
  } else {
    joystickRHX = newJoystickRHX;
  }

  if (abs(newJoystickRHY) < 23) {
    joystickRHY = 0;
  } else {
    joystickRHY = newJoystickRHY;
  }

  //truth table
  if ((joystickRHX == 0) && (joystickRHY == 0)) {
    pos = posRest;
  }

  if ((joystickRHX < 0) && (joystickRHY > 0)) {
    pos = posUpLeft;
  }

  if ((joystickRHX == 0) && (joystickRHY > 0)) {
    pos = posUp;
  }

  if ((joystickRHX > 0) && (joystickRHY > 0)) {
    pos = posUpRight;
  }

  if ((joystickRHX > 0) && (joystickRHY == 0)) {
    pos = posRight;
  }

  if ((joystickRHX > 0) && (joystickRHY < 0)) {
    pos = posDownRight;
  }

  if ((joystickRHX == 0) && (joystickRHY < 0)) {
    pos = posDown;
  }

  if ((joystickRHX < 0) && (joystickRHY < 0)) {
    pos = posDownLeft;
  }

  if ((joystickRHX < 0) && (joystickRHY == 0)) {
    pos = posLeft;
  }

  print(String("RAW X:" + String(newJoystickRHX) + " RAW Y:" + String(newJoystickRHY) + " X:" + String(joystickRHX) + "  Y:" + String(joystickRHY) + " Position: " + pos));

  //always stop motors before moving abruptly
  //digitalWrite( leftDirection, LOW );
  //digitalWrite( leftSpeed, LOW );
  //digitalWrite( rightDirection, LOW );
  //digitalWrite( rightSpeed, LOW );

  //if it's changed then act
  if (pos == posUpLeft) {

    digitalWrite(leftDirection, HIGH);   // direction = forward
    analogWrite(leftSpeed, PWM_SLOW_FWD);

    digitalWrite(rightDirection, HIGH);   // direction = forward
    analogWrite(rightSpeed, PWM_FAST_FWD);

  } else if (pos == posUp) {


    moveBackward();

  } else if (pos == posUpRight) {

    digitalWrite(leftDirection, HIGH);   // direction = forward
    analogWrite(leftSpeed, PWM_FAST_FWD);

    digitalWrite(rightDirection, HIGH);   // direction = forward
    analogWrite(rightSpeed, PWM_SLOW_FWD);

  } else if (pos == posRight) {

    moveRight();

  } else if (pos == posDownRight) {

    digitalWrite(leftDirection, LOW);   // direction = backwards
    analogWrite(leftSpeed,   PWM_SLOW_BWD);

    digitalWrite(rightDirection, LOW);   // direction = backwards
    analogWrite(rightSpeed, PWM_FAST_BWD);

  } else if (pos == posDown) {

    moveForward();

  } else if (pos == posDownLeft) {

    digitalWrite(leftDirection, LOW);   // direction = backwards
    analogWrite(leftSpeed, PWM_FAST_BWD);

    digitalWrite(rightDirection, LOW);   // direction = backwards
    analogWrite(rightSpeed, PWM_SLOW_BWD);

  } else if (pos == posLeft) {

    moveLeft();

  } else if (pos == posRest) {
    //stop nicely (coast)
    stopMotors();
  } else {
    //stop nicely (coast)
    stopMotors();
  }
  }
*/

int HeadPos = defaultHeadPos;    // variable to store the servo position
unsigned long servoInterval = 10;
unsigned long previousServoMillis = millis();

void headJoystick() {

  //int newJoystickX = map(ps2x.Analog(PSS_LX) , 0, 255, 154, 30);

  int newJoystickX =  ps2x.Analog(PSS_LX);


  //newJoystickX = constrain(newJoystickX, 30, 154);


  //newJoystickX = 128 , servo position = 85
  //newJoystickX = 255 , servo position = 154
  //newJoystickX = 0 , servo position = 30

  int newHeadPos = defaultHeadPos;

  if (newJoystickX > 128) {
    newHeadPos = 30;
  }

  if (newJoystickX < 128) {
    newHeadPos = 154;
  }


  print("Head Position on Analog(PSS_RX) @" + String(newJoystickX));

  if (newHeadPos != HeadPos) {

    unsigned long currentMillis = millis();

    // its time for another move
    if (currentMillis - previousServoMillis >= servoInterval) {

      previousServoMillis = currentMillis;

      if (newHeadPos > HeadPos)
      {
        HeadPos = HeadPos + 1;
      } else {
        HeadPos = HeadPos - 1;
      }
    }

    headServo.write(HeadPos);     // sets the servo position to new position
  }
}


void armPosition() {

  //Serial.println("leftArmPos" + String(leftArmPos));

  if (ps2x.Button(PSB_L2)) {

    leftArmPos = constrain( leftArmPos + armSpeed, 0, 90);

    print("L1 pressed @" + String(leftArmPos));

    leftArmServo.write(leftArmPos);
  }

  if (ps2x.Button(PSB_L1)) {

    leftArmPos = constrain( leftArmPos - armSpeed, 0, 90);

    print("L2 pressed @" + String(leftArmPos));

    leftArmServo.write(leftArmPos);
  }

  if (ps2x.Button(PSB_R2)) {


    rightArmPos = constrain( rightArmPos - armSpeed, 90, 180);

    print("R1 pressed @" + String(rightArmPos));

    rightArmServo.write(rightArmPos);
  }

  if (ps2x.Button(PSB_R1)) {

    rightArmPos = constrain( rightArmPos + armSpeed, 90, 180);
    print("R2 pressed @" + String(rightArmPos));
    rightArmServo.write(rightArmPos);
  }
}
