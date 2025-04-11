/*
  RavensC++, 2024-2025
  Team ID: 3991-1
  Team Members: Rudra Kumar, Ali Zain Bukhari, Dean Forsyth, Benjamin Leonard, James Rogers, and Caitlyn Volkert.
  Robofest Exhibiton
  Sunlake Academy of Math and Science
  Sphero RVR + Sparkfun Redboard Plus + Sparkfun Devices
*/

// Libraries:
#include <Wire.h>                              // Wire/Qwiic
#include <Adafruit_PWMServoDriver.h>           // Servo/PCA9685
#include <SparkFun_I2C_Mux_Arduino_Library.h>  //Qwiic Mux TCA9548A
#include <SFE_MicroOLED.h>                     //Qwiic Mini Display
#include "SparkFun_VL53L1X.h"                  //Qwiic Distance VL531X
#include <SpheroRVR.h>                         //Sphero RVR
#include <SparkFun_Qwiic_Button.h>             //Qwiic Button
#include <Servo.h>

// Constants
#define SERVO_FREQ 50  // Servo frequency
#define USMIN 600      // Min pulse width
#define USMAX 2400     // Max pulse width

QwiicButton button;
QWIICMUX myMux;
SFEVL53L1X distanceSensor;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// RVR Functions:
static DriveControl driveControl;  // Drive Control
MicroOLED oled(9);

// PROGMEM String Data
const char welcomeMessage[] PROGMEM = "RavensC++\nID:3991-1\n \nPush to\nstart\n";
const char startMessage[] PROGMEM = "RavensC++\nID:3991-1\n\n\nStarting.. \n";
const char serialMonitorMessage[] PROGMEM = "_________________________\nRavensC++ SERIAL MONITOR\n";
const char emergencyStopMessage[] PROGMEM = "RavensC++\nID:3991-1\n\n\nSTOPPED \n";

int seed_dis = 5;
int max_dis_front = 149; 
int foward = 500;
/*
QWIIC PORTS
Right Dis Port: 4    Button port : 0
Front Dis Port: 6    Display port: 1
Left Dis Port: 7     Servo Controller: 3


SERVO PORTS
ARMEntry: 0         ARMShovel: 2
ARMCenter: 1        SEEDcontrol: 3
*/

// Setup Function
void setup() {
  // Set up communication with the RVR
  rvr.configUART(&Serial);
  rvr.wake();
  delay(2000);

  // RVR drive control
  driveControl = rvr.getDriveControl();

  // Start setup:
  Serial.begin(115200);
  Wire.begin();
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);

  myMux.begin();

  setPort(1);
  // Start OLED
  oled.begin(0x3D, Wire);  // Initialize the OLED
  oled.clear(ALL);         // Clear the display's internal memory
  oled.display();          // Display what's in the buffer (splashscreen)
  oled.setFontType(0);

  delay(2000);  // Delay 1000 ms


  oledDisplayMessage(welcomeMessage);
  setServo(3, 45);

  //Checks if distance sensors are online
  myMux.setPort(6);
  distanceSensor.sensorOn();
  if (!distanceSensor.begin()) {
    Serial.println(F("Front Distance Sensor online!"));
  } else {
    Serial.println(F("Front Distance Sensor failed to begin. Please check wiring. Freezing..."));
    while (true)
      ;
  }
  myMux.setPort(7);
  distanceSensor.sensorOn();
  if (!distanceSensor.begin()) {
    Serial.println(F("Right Sensor online!"));
  } else {
    Serial.println(F("Right Distance Sensor failed to begin. Please check wiring. Freezing..."));
    while (true)
      ;
  }
  myMux.setPort(4);
  distanceSensor.sensorOn();
  if (!distanceSensor.begin()) {
    Serial.println(F("Left Sensor online!"));
  } else {
    Serial.println(F("Left Sensor Sensor failed to begin. Please check wiring. Freezing..."));
    while (true)
      ;
  }
  setPort(0);
  button.begin();
  button.LEDoff();
  while (!button.isPressed()) {
    button.LEDconfig(250, 1000, 200);
    delay(100);
  }
  button.LEDon(20);

  // END setup

  setPort(1);

  oledDisplayMessage(startMessage);
  //Moves servo to starting position
  setServo(0, 100);
  setServo(1, 180);
  setServo(2, 180);

  //void setup() END
}

//Functions:

void seedSpit() {
  // Spits seeds
  setServo(3, 0);
  delay(200);
  setServo(3, 45);
}

int dig() {
  // Picks up dirt
  turnN(90);
  delay(500);
  rawForward(500);
  setServo(0, 140);
  setServo(1, 150);
  setServo(2, 180);
  setServo(0, 10);
  delay(2000);
  setServo(1, 10);

  delay(100);
  setServo(2, 100);
  delay(3000);
  setServo(2, 30);
  setServo(1, 90);
  setServo(0, 40);
  delay(1000);
  rawBackward(500);
  // Spits seeds
  turnN(270);
  delay(500);
  turnN(270);
  delay(1000);
  rawBackward(500);
  delay(1000);
  seedSpit();
  delay(1000);
  rawForward(500);
  delay(1000);
  turnN(90);
  turnN(90);
  // Bury seed with dirt
  rawForward(500);
  setServo(2, 180);
  delay(1000);
  rawBackward(500);
  delay(1000);
  turnN(270);
}

void rawForward(unsigned long ms) {
  driveControl.setRawMotors(rawMotorModes::forward, 64, rawMotorModes::forward, 64);
  delay(ms);
  driveControl.rollStop(0);
}

void rawBackward(unsigned long ms) {
  driveControl.setRawMotors(rawMotorModes::reverse, 64, rawMotorModes::reverse, 64);
  delay(ms);
  driveControl.rollStop(0);
}

void setServo(int port, int degrees) {
  // Turns servo from 0 to 180 degrees
  degrees = constrain(degrees, 0, 180);
  int pulse = map(degrees, 0, 180, 160, 500);
  pwm.setPWM(port, 0, pulse);
}

void oledDisplayMessage(const char* message) {
  myMux.setPort(1);

  // Load the message from PROGMEM into a buffer in RAM
  char buffer[100];  // Assuming message is smaller than 100 characters
  for (int i = 0; i < 100; i++) {
    buffer[i] = pgm_read_byte(&message[i]);
    if (buffer[i] == '\0') break;  // Stop if we reach the end of the string
  }

  // Clear the OLED screen
  oled.clear(PAGE);      // Clear the display
  delay(1000);           // Optional: Delay for clearing
  oled.setCursor(0, 0);  // Start at the top-left of the screen

  // Print the buffer to the OLED
  oled.print(buffer);
  oled.display();  // Display the message

  // Clear the buffer
  memset(buffer, 0, sizeof(buffer));  // Clear the RAM buffer
}

void setPort(int muxPort) {
  // Quick change for Mux ports
  myMux.setPort(muxPort);
}

void moveDistance(int inches) {
  rvr.resetYaw();
  // Speed is 6 inches per second (from your test)
  int timeToMove = (inches / 6.0) * 1000;  // Convert seconds to milliseconds

  // Adjust the time slightly to compensate for over-movement (if it moves too far)
  timeToMove -= 250;  // Reduces the time by 200ms to make it closer to the intended distance

  // Move forward
  driveControl.rollStart(0, 25);  // Move forward at speed 25
  delay(timeToMove);              // Wait for the RVR to move the desired distance
  driveControl.rollStop(0);       // Stop moving
}

int turn(uint16_t heading, uint8_t turnSpeed) {
  // Reset the yaw (orientation) of the RVR
  rvr.resetYaw();

  // Set the initial heading to 0 (forward)
  driveControl.setHeading(0);
  delay(100);

  // Start turning with the specified heading and speed
  driveControl.rollStart(heading, turnSpeed);
  delay(2000);  // Turn for 2 seconds (you can adjust the delay as needed)

  // Reset yaw after the turn
  rvr.resetYaw();
  driveControl.setHeading(0);
  delay(100);

  // Stop the RVR after the turn is complete
  driveControl.rollStop(0);
}

int turnN(uint16_t heading) {
  // Reset yaw to ensure the starting angle is 0
  rvr.resetYaw();
  driveControl.rollStop(heading);
  delay(1000);
  driveControl.setRawMotors(rawMotorModes::reverse, 32, rawMotorModes::reverse, 32);
  delay(1);
  driveControl.rollStop(heading);
  rvr.resetYaw();
}
void getdistance(int port) {
  myMux.setPort(port);
  distanceSensor.startRanging();  // Initiate measurement

  while (!distanceSensor.checkForDataReady()) {
    delay(1);
  }

  int distance = distanceSensor.getDistance();
  distanceSensor.clearInterrupt();
  distanceSensor.stopRanging();

  Serial.print(F("Distance(mm): "));
  Serial.println(distance);

  if (distance < 250) {  // Set your threshold here
    driveControl.rollStop(0);
    delay(1000);
    oledDisplayMessage(emergencyStopMessage);
    delay(1000);
    while (true) {
  }
  } else {
    Serial.println("continue");
  }
}

void loop(){
  //Plants seeds
  dig();
  delay(1000);
  getdistance(6);
  delay(1000);
  rawForward(750);
  delay(1000);
};
