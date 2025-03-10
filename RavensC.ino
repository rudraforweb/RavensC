/*
  RavensC++, 2024-2025
  Team ID: 3991-1
  Team Members: Rudra Kumar, Ali Zain Bukhari, Dean Forsyth, Benjamin Leonard, James Rogers, and Caitlyn Volkert.
  Robofest Exhibiton
  Sunlake Academy of Math and Science
  Sphero RVR + Sparkfun Devices
*/

//Libraries:
#include <Wire.h>                              // Wire/Qwiic
#include <Adafruit_PWMServoDriver.h>           // Servo/PCA9685
#include <SparkFun_I2C_Mux_Arduino_Library.h>  //Qwiic Mux TCA9548A
#include <SFE_MicroOLED.h>                     //Qwiic Mini Display
#include "SparkFun_I2C_GPS_Arduino_Library.h"  //Qwiic GPS XA1110
#include "SparkFun_VL53L1X.h"                  //Qwiic Distance VL531X
#include <SpheroRVR.h>                         //Sphero RVR
#include <SparkFun_Qwiic_Button.h>             //Qwiic Button
#include <Servo.h>  

//Variables:
#define PIN_RESET 9

#define SERVOMIN 160   // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX 500   // This is the 'maximum' pulse length count (out of 4096)
#define USMIN 600      // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX 2400     // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50  // Analog servos run at ~50 Hz updates

QwiicButton button;
QWIICMUX myMux;
SFEVL53L1X distanceSensor;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

//RVR Functions:
static DriveControl driveControl;  //Drive Control
static uint32_t ledGroup;          //LEDs
static LedControl ledControl;
static void getBatteryVoltageStateCallback(GetBatteryVoltageStateReturn_t *batteryVoltageStateReturn);  //Battery
MicroOLED oled(PIN_RESET);//OLED


//Button Functions:
uint8_t brightness = 250;   //The maximum brightness of the pulsing LED. Can be between 0 (min) and 255 (max)
uint16_t cycleTime = 1000;  //The total time for the pulse to take. Set to a bigger number for a slower pulse, or a smaller number for a faster pulse
uint16_t offTime = 200;     //The total time to stay off between pulses. Set to 0 to be pulsing continuously.

/*
QWIIC PORTS
Right Distance Port: 4
Front Distance Port: 6
Left Distance Port: 7
Button port : 0
Display port: 1
GPS port: 3

SERVO PORTS
ARMEntry: 0
ARMCenter: 1
ARMShovel: 2
SEEDcontrol: 3
*/

// These are stored in mm
double stagel = 914;
double stagew = 609.6;
// how far each seed is apart (horizontally and vertically)
double seed_dis = 0;
// half the length of the robot L and W
double hw = 107.95;
double hl = 88.9;
// servol stands for servo length, its not from the midpoint, you would have to add both the hl and servol to get the length from midpoint to end of servos
double servol = 127;


void setup() {
  // set up communication with the RVR
  rvr.configUART(&Serial);
  rvr.wake();
  delay(2000);

  // RVR drive control
  driveControl = rvr.getDriveControl();

  // RVR led control
  ledControl = rvr.getLedControl();

  //Start setup:
  Serial.begin(115200);
  Wire.begin();
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);

  setupMux();
  setPort(1);
  setupOled();

  delay(2000);  // Delay 1000 ms
  Serial.print("_________________________\n");
  Serial.print("RavensC++ SERIAL MONITIOR\n");

  oledDisplayMessage("RavensC++\nID:3991-1\n \nPush to\nstart\n");
  setServo(3, 45);
  setPort(0);
  button.begin();
  button.LEDoff();
  while (button.isPressed() == false) {
    button.LEDconfig(brightness, cycleTime, offTime);
    if (button.isPressed() == true) {
      Serial.println("Button Pressed. Starting...");
      button.LEDon(20);
      break;
    }
  }

  //END setup.

  setPort(1);

  oledDisplayMessage("RavensC++\nID:3991-1\n\n\nStarting.. \n");

  dig();
}


void seedSpit() {
  //Spits seeds
  setServo(3, 0);
  delay(200);
  setServo(3, 45);
}
void dig() {
  //Picks up dirt
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
  delay(3000);
  //Spits seeds
  turn(90);
  turn(90);
  delay(1000);
  seedSpit();
  delay(1000);
  turn(90);
  turn(90);
  //Buries seed with dirt
  setServo(2, 180);
}




void setServo(int port, int degrees) {
  //Turns servo from 0 to 180 degrees

  // Constrain degrees between 0 and 180
  degrees = constrain(degrees, 0, 180);

  // Map degrees to pulse width
  int pulse = map(degrees, 0, 180, SERVOMIN, SERVOMAX);

  // Send command to servo
  pwm.setPWM(port, 0, pulse);
}

void oledDisplayMessage(char message[]) {
  //Displays a message on the display
  oled.clear(PAGE);
  delay(1000);
  oled.setCursor(0, 0);
  oled.print(message);
  oled.display();
}

void setupMux() {
  //Starts Mux
  myMux.begin();
  byte currentPortNumber = myMux.getPort();
}

void setPort(int muxPort) {
  //Quick change for Mux ports
  myMux.setPort(muxPort);
}

void setupOled() {
  //Starts OLED
  oled.begin(0x3D, Wire);  // Initialize the OLED
  oled.clear(ALL);         // Clear the display's internal memory
  oled.display();          // Display what's in the buffer (splashscreen)
  oled.setFontType(0);
}

void moveDistance(double distance, int speed, int heading) {
  //Moves RVR

  // Estimated time formula: time = distance / speed * 1000 (convert to ms)
  // Using double for precision with mm (distance can now be in mm)
  double time = (distance / (double)speed) * 1000.0;
  rvr.resetYaw();
  // Start moving
  driveControl.setHeading(heading);  // Assuming heading is passed as an integer
  driveControl.rollStart(heading, speed);
  delay(time);

  // Stop moving
  driveControl.rollStop(heading);
}


int turn(uint16_t heading) {
  /*90: left
    270: right
    Turns RVR*/
  rvr.resetYaw();
  driveControl.setHeading(0);
  delay(100);
  driveControl.rollStart(heading, 1);
  delay(1000);
  rvr.resetYaw();
  driveControl.setHeading(0);
  delay(100);
  driveControl.rollStop(0);
}

void stop() {
  //Halts RVR
  rvr.resetYaw();
  driveControl.setHeading(0);
  driveControl.rollStop(0);
}

void loop(){
  //Does nothing... for now
};