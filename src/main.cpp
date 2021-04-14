#include <Servo.h>            //include the library about servo https://github.com/arduino-libraries/Servo v1.1.7
#include <SoftwareSerial.h> // SoftwareSerial for serial communication with HM10 bluetooth module. https://www.arduino.cc/en/Reference/softwareSerial
#include <ArduinoBlue.h> // ArduinoBlue bluetooth library https://github.com/purwar2016/ArduinoBlue-library v3.1.01

const unsigned long BAUD_RATE = 9600;  //ble can only run at 9600 baud unless flashed AT command

Servo servo_steer;            //give a name to the servo

//motor driver pins
#define APH A2
#define AENBL 11
#define BPH A0
#define BENBL 6

// HM10 BLUETOOTH MODULE PINS
// NOTE: Not all pins on your Arduino may support SoftwareSerial.
// Please check: https://www.arduino.cc/en/Reference/softwareSerial
const int BLUETOOTH_TX = 8;
const int BLUETOOTH_RX = 7;
SoftwareSerial softSerial(BLUETOOTH_TX, BLUETOOTH_RX); // Object for serial communication to HM 10 bluetooth module using digital pins.
ArduinoBlue phone(softSerial); // Object for smartphone robot control.

#define LED 13 //not necessary

void steer()
{
  int steering = phone.getSteering();
  int mappedServo = map(abs(steering), 0, 99, 30, 150); //takes values from left/right joystick and maps to servo values
  if (steering != 49) {
     servo_steer.attach(3);
  }
  else {
    servo_steer.detach();
  }
  servo_steer.write(mappedServo); //steers using mapped values
  Serial.println(steering);
  delay(5);  //wait for servo to get to position
  //Serial.print("steering value:  "); Serial.println(steering);
  //Serial.print("mapped steering:    "); Serial.println(mappedServo);
}


void drive()
{
  int throttle = phone.getThrottle();
  int mappedSpeed = map(throttle, 0, 99, -250, 250); // Map throttle to PWM range.
  //Serial.println(mappedSpeed);

  if (mappedSpeed > -5 && mappedSpeed < 5) {
    digitalWrite(AENBL, 0);
    digitalWrite(BENBL, 0);  
    digitalWrite(APH, LOW);
    digitalWrite(BPH, LOW);
  }
  else if (mappedSpeed > 5) {
    digitalWrite(APH, LOW);
    digitalWrite(BPH, LOW);
    analogWrite(AENBL, mappedSpeed);
    analogWrite(BENBL, mappedSpeed);
  }
  else if (mappedSpeed < -5) {
    digitalWrite(APH, HIGH);
    digitalWrite(BPH, HIGH);
    analogWrite(AENBL, mappedSpeed);
    analogWrite(BENBL, mappedSpeed);
  }
}


void voltageMonitor() //still being implemented
{
  
  if (Serial.available()) {
    Serial.write("send: ");
    String str = Serial.readString();
    phone.sendMessage(str); // phone.sendMessage(str) sends the text to the phone.
    Serial.print(str);
    Serial.write('\n');
  }
}
void setup()
{
  Serial.begin(BAUD_RATE);  //make sure that if you change baud rate, that you change BLE rate with hardware commants (AT commands)
  softSerial.begin(BAUD_RATE);  //on AVR @ 16 MHz (we are at 8 MHz) minimum baud is 245
  delay(100);
  digitalWrite(LED, HIGH);
  Serial.print("BLE Setup Start");
  servo_steer.attach(9); //tell the servo object that its servo is plugged into pin 3
  Serial.println("BLE Setup Finish");
  digitalWrite(LED, LOW);
  pinMode(APH, OUTPUT);
  pinMode(AENBL, OUTPUT);
  pinMode(BPH, OUTPUT);
  pinMode(BENBL, OUTPUT);
}

void loop()
{
  steer();
  drive();
  //voltageMonitor(); Still being implemented
}