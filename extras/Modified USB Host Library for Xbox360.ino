/*
  Example sketch for the Xbox Wireless Reciver library - developed by Kristian Lauszus
  It supports up to four controllers wirelessly
  For more information see the blog post: http://blog.tkjelectronics.dk/2012/12/xbox-360-receiver-added-to-the-usb-host-library/ or
  send me an e-mail:  kristianl@tkjelectronics.com

  https://github.com/felis/USB_Host_Shield_2.0/blob/master/examples/Xbox/XBOXRECV/XBOXRECV.ino
*/

#include <Servo.h>
#include <SPI.h>

//#include <XBOXRECV.h>
#include "src/USB_Host_Shield_Library_2.0_Modified/XBOXRECV.h"

// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif



//Define variables
USB Usb;
XBOXRECV Xbox(&Usb);

//THE USB SHIELD IS USING 7,8,11,12,13  !!!!!!!!!!!!!

// Configure digital pins to control motors on HG7881-(L9110), a 4 wire controller! (PWM PINS = 3, 5, 6, 9, 10)
int leftDirection = 4; // HG7881_B_IB ( Motor B Direction )
int leftSpeed = 6;   // HG7881_B_IA ( Motor B PWM Speed )

int rightDirection = 2; // HG7881_A_IB ( Motor A Direction )
int rightSpeed = 5; // HG7881_A_IA ( Motor A PWM Speed )

// https://www.bananarobotics.com/shop/How-to-use-the-HG7881-(L9110)-Dual-Channel-Motor-Driver-Module
int PWM_SLOW = 123;  // arbitrary slow speed PWM duty cycle
int PWM_FAST = 1; // arbitrary fast speed PWM duty cycle
 
//configure arm pins
int leftArm = 9;
int rightArm = 10;

//configure head pins
int headMovement = 3;
int leftEye = A5;
int rightEye = A4;

//joysticks x positions
int joystickLHX = 1400;
int joystickLHY; //Define Joystick Left  Hat  Y Variable
int joystickRHX = 1400;
int joystickRHY; // Define Joystick Right Hat  X Variable

int Xbox360Controller = 0;

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

String currentPos = "START";

Servo headServo;  // create servo object to control the head servo
Servo leftArmServo;  // create servo object to control the arm
Servo rightArmServo;  // create servo object to control the arm servo

int ledState = LOW;             // ledState used to set the LED

// Generally, you should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store
unsigned long previousMillis = 0;        // will store last time LED was updated

const long blinkSpeed = 1000;           // interval at which to blink (milliseconds)


void setup() {
  Serial.begin(115200);

  // pinMode( 7, OUTPUT); //vital to set pin 7 HIGH in order to work!
  //digitalWrite( 7, HIGH);


  //SPI.begin();

  // Configure digital pins for motor
  pinMode(leftDirection, OUTPUT);
  pinMode(leftSpeed, OUTPUT);
  pinMode(rightDirection, OUTPUT);
  pinMode(rightSpeed, OUTPUT);

  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection

  if (Usb.Init() == -1) {    // If USB shield did not initialise for whatever reason...
    while (1); //halt as long as shield is reported disconnected
  }

  print(F("\r\nXbox Wireless Receiver Library Started"));

  //connect to servos
  headServo.attach(headMovement);  // attaches the servo on pin (headMovement) to the servo object
  leftArmServo.attach(leftArm);
  rightArmServo.attach(rightArm);

  headServo.write(90);  // sets the servo position to center the head
  delay(15);            // waits for the servo to get there

  leftArmServo.write(90);  // sets the servo position to center
  delay(15);            // waits for the servo to get there

  rightArmServo.write(90);  // sets the servo position to center
  delay(15);            // waits for the servo to get there

  pinMode( leftEye, OUTPUT); // light up left eye
  pinMode( rightEye, OUTPUT); //light up right eye
  digitalWrite( leftEye, HIGH);
  digitalWrite( rightEye, HIGH);
  delay(1000);   //leave eyes lit for one second to show working
  digitalWrite( leftEye, LOW);
  digitalWrite( rightEye, LOW);

  //test motors
  //digitalWrite(leftDirection, HIGH);   // direction = forward
  //analogWrite(leftSpeed, PWM_FAST);

  //digitalWrite(rightDirection, HIGH);   // direction = forward
  //analogWrite(rightSpeed, PWM_FAST);

  //delay(1000);

  //digitalWrite(leftDirection, LOW);   // direction = backwards
  //analogWrite(leftSpeed, PWM_SLOW);

  //digitalWrite(rightDirection, LOW);   // direction = backwards
  //analogWrite(rightSpeed, PWM_SLOW);

  //delay(1000);

  digitalWrite(leftDirection, LOW);
  digitalWrite(leftSpeed, LOW);

  digitalWrite(rightDirection, LOW);
  digitalWrite(rightSpeed, LOW);
}


void loop() {

  Usb.Task();

  if (Xbox.XboxReceiverConnected) {
    if (Xbox.Xbox360Connected[Xbox360Controller]) {

      LeftJoystick();

      RightJoystick();

      getButtonClicks();

      winkEye();

      armPosition();
    }
  }
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

void getButtonClicks() {

  //  if (Xbox.getButtonClick(A, Xbox360Controller))
  //    Serial.println(F("A"));

  if (Xbox.getButtonClick(B, Xbox360Controller)) {
    //only blink if LEDs are off
    if (ledState == LOW) {
      //record time of blink & set LED to on
      previousMillis = millis();

      digitalWrite(rightEye, HIGH);
    }
  }

  if (Xbox.getButtonClick(A, Xbox360Controller)) {
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

  //  if (Xbox.getButtonClick(Y, Xbox360Controller))
  //    Serial.println(F("Y"));

  //https://felis.github.io/USB_Host_Shield_2.0/class_x_b_o_x_r_e_c_v.html
  //Dead mans switch - press the xbox button to stop the car
  if (Xbox.getButtonClick(XBOX, Xbox360Controller)) {

    digitalWrite(rightDirection, HIGH);
    digitalWrite(rightSpeed, HIGH);

    digitalWrite(leftDirection, HIGH);
    digitalWrite(leftSpeed, HIGH);

    //print some debugging details
    int JoystickLHX = map(Xbox.getAnalogHat(LeftHatX, Xbox360Controller), -32768, 32767, 0, 3880);
    int JoystickLHY = map(Xbox.getAnalogHat(LeftHatY, Xbox360Controller), -32768, 32767, 0, 3880);

    print(String("STOPPED @ X:" + String(JoystickLHX) + " Y:" + String(JoystickLHY)));

    headServo.write(90);  // sets the servo position to center the head

    Xbox.setLedMode(ROTATING, Xbox360Controller);
    delay(1000); //wait one second
    Xbox.setLedOn(LED1, Xbox360Controller);
  }

  if (Xbox.getButtonClick(SYNC, Xbox360Controller)) {
    print(F("Sync"));
    Xbox.disconnect(Xbox360Controller);
  }



}

bool serialPrint = true;

void print(String val) {
  if (serialPrint == true) {
    Serial.println(val);
  }
}


void LeftJoystick() {

  /* The sample code for the Xbox controller gave a deadzone of -7500 to 7500.
    This code maintains that dead zone for now (I would like to make it
    adjustable while the sketch is running).
    https://www.instructables.com/id/BARC-Jeep-An-XBOX-Controlled-Power-Wheels/
  */

  int newJoystickLHX = map(Xbox.getAnalogHat(LeftHatX, Xbox360Controller), -32768, 32767, 0, 3880);
  int newJoystickLHY = map(Xbox.getAnalogHat(LeftHatY, Xbox360Controller), -32768, 32767, 0, 3880);

  String pos = "TBD";

  if ((newJoystickLHX >= 1500) && (newJoystickLHX <= 2246)) {
    joystickLHX = 1400;
  }  else {
    if (newJoystickLHX != joystickLHX) {
      joystickLHX = newJoystickLHX;
    }
  }

  if ((newJoystickLHY >= 1500) && (newJoystickLHY <= 2010)) {
    joystickLHY = 1400;
  } else {
    if (newJoystickLHY != joystickLHY) {
      joystickLHY = newJoystickLHY;
    }
  }

  //truth table
  if ((joystickLHX == 1400) && (joystickLHY == 1400)) {
    pos = posRest;
  }

  if ((joystickLHX < 1400) && (joystickLHY > 1400)) {
    pos = posUpLeft;
  }

  if ((joystickLHX == 1400) && (joystickLHY > 1400)) {
    pos = posUp;
  }

  if ((joystickLHX > 1400) && (joystickLHY > 1400)) {
    pos = posUpRight;
  }

  if ((joystickLHX > 1400) && (joystickLHY == 1400)) {
    pos = posRight;
  }

  if ((joystickLHX > 1400) && (joystickLHY < 1400)) {
    pos = posDownRight;
  }

  if ((joystickLHX == 1400) && (joystickLHY < 1400)) {
    pos = posDown;
  }

  if ((joystickLHX < 1400) && (joystickLHY < 1400)) {
    pos = posDownLeft;
  }

  if ((joystickLHX < 1400) && (joystickLHY == 1400)) {
    pos = posLeft;
  }

  //determine if our position has actually changed
  if (pos != currentPos) {

    print(String("Position" + pos + " X:" + String(newJoystickLHX) + " Y:" + String(newJoystickLHY)));

    currentPos = pos;

    //always stop motors before moving abruptly
    digitalWrite( leftDirection, LOW );
    digitalWrite( leftSpeed, LOW );
    digitalWrite( rightDirection, LOW );
    digitalWrite( rightSpeed, LOW );

    //if it's changed then act
    if (pos == posUpLeft) {

      digitalWrite(leftDirection, HIGH);   // direction = forward
      analogWrite(leftSpeed, PWM_SLOW);

      digitalWrite(rightDirection, HIGH);   // direction = forward
      analogWrite(rightSpeed, PWM_FAST);

    } else if (pos == posUp) {

      digitalWrite(leftDirection, HIGH);   // direction = forward
      analogWrite(leftSpeed, PWM_FAST);

      digitalWrite(rightDirection, HIGH);   // direction = forward
      analogWrite(rightSpeed, PWM_FAST);

    } else if (pos == posUpRight) {

      digitalWrite(leftDirection, HIGH);   // direction = forward
      analogWrite(leftSpeed, PWM_FAST);

      digitalWrite(rightDirection, HIGH);   // direction = forward
      analogWrite(rightSpeed, PWM_SLOW);

    } else if (pos == posRight) {

      digitalWrite(leftDirection, HIGH);   // direction = forward
      analogWrite(leftSpeed, PWM_FAST);

      digitalWrite(rightDirection, LOW);   // direction = backwards
      analogWrite(rightSpeed, 255 - PWM_FAST);

    } else if (pos == posDownRight) {

      digitalWrite(leftDirection, LOW);   // direction = backwards
      analogWrite(leftSpeed, 255 -  PWM_FAST);

      digitalWrite(rightDirection, LOW);   // direction = backwards
      analogWrite(rightSpeed, 255 -  PWM_SLOW);

    } else if (pos == posDown) {

      digitalWrite(leftDirection, LOW);   // direction = backwards
      analogWrite(leftSpeed, 255 - PWM_FAST);

      digitalWrite(rightDirection, LOW);   // direction = backwards
      analogWrite(rightSpeed, 255 - PWM_FAST);

    } else if (pos == posDownLeft) {

      digitalWrite(leftDirection, LOW);   // direction = backwards
      analogWrite(leftSpeed, 255 - PWM_SLOW);

      digitalWrite(rightDirection, LOW);   // direction = backwards
      analogWrite(rightSpeed, 255 - PWM_FAST);

    } else if (pos == posLeft) {

      digitalWrite(leftDirection, LOW);   // direction = forward
      analogWrite(leftSpeed, 255 -  PWM_FAST);

      digitalWrite(rightDirection, HIGH);   // direction = backwards
      analogWrite(rightSpeed, PWM_FAST);

    } else {
      //stop nicely (coast)
      digitalWrite(leftDirection, LOW);
      digitalWrite(leftSpeed, LOW);
      digitalWrite(rightDirection, LOW);
      digitalWrite(rightSpeed, LOW);
    }
  }
}

int HeadPos = 90;    // variable to store the servo position
unsigned long servoInterval = 10;
unsigned long previousServoMillis = millis();

void RightJoystick() {

  /* The sample code for the Xbox controller gave a deadzone of -7500 to 7500.
    This code maintains that dead zone for now (I would like to make it
    adjustable while the sketch is running).
    https://www.instructables.com/id/BARC-Jeep-An-XBOX-Controlled-Power-Wheels/
  */

  int newJoystickRHX = map(Xbox.getAnalogHat(RightHatX, Xbox360Controller), -32768, 32767, 30, 154);

  if (newJoystickRHX != HeadPos) {

    unsigned long currentMillis = millis();

    // its time for another move
    if (currentMillis - previousServoMillis >= servoInterval) {

      previousServoMillis = currentMillis;

      if (newJoystickRHX > HeadPos)
      {
        HeadPos = HeadPos + 1;
      } else {
        HeadPos = HeadPos - 1;
      }
    }
  }

  headServo.write(HeadPos);     // sets the servo position to new position
}


void armPosition() {

 // if (Xbox.getButtonPress(L2, Xbox360Controller) || Xbox.getButtonPress(R2, Xbox360Controller)) {
 //   Serial.print("L2: ");
  //  Serial.print(Xbox.getButtonPress(L2, Xbox360Controller));
  //  Serial.print("\tR2: ");
  //  Serial.println(Xbox.getButtonPress(R2, Xbox360Controller));
  //}

  int newL2 = map(Xbox.getButtonPress(L2, Xbox360Controller), 0, 255, 90, 0);
  int newR2 = map(Xbox.getButtonPress(R2, Xbox360Controller), 0, 255, 90, 180);

  leftArmServo.write(newL2);
  rightArmServo.write(newR2);

}
