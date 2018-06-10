//#include <Servo.h> //Servo library
#include <assert.h> //assert library for debugging


#define blueToothSerial Serial1

#define RxD 6
#define TxD 7

#define SXPWM_Control 5 //servoX PWM control pin
#define SX_Control 4 //servoX control pin

#define SYPWM_Control 6 //servoY PWM control pin
#define SY_Control 7 //servoY control pin
//#define COMPARE() average_top < upperThreshold || average_bottom < upperThreshold || average_left < upperThreshold || average_right < upperThreshold
#define pwm 25
/*
   #define INTERRUPTPIN 7
   #define INTERRUPTMODE  LOW
*/
//
//Servo servoX;
//Servo servoY;


//for debugging:
int diff_reading = 0;
int reading_time = 0;

int north_south = 0; //input pin for north/south LDRs
int east_west = 1; //input pin for east/west LDRs

int* data;

float upperThreshold = 1.1;
float lowerThreshold = 3.24;

int degreesY = 90;
int degreesX = 90;

int maxY =  180;
int maxX = 180;



void setup() {
  // put your setup code here, to run once:
  for (int i = 4; i <= 7; i++)
    pinMode(i, OUTPUT);

  //  servoY.attach(SY_Control);
  //  servoY.write(degreesY);
  //  servoX.attach(SX_Control);
  //  servoX.write(degreesX);

  pinMode(RxD, INPUT);
  pinMode(TxD, OUTPUT);

  Serial.begin(9600);

  //  pinMode(4, OUTPUT);
  //  pinMode(5, OUTPUT);
}



void loop() {

  //read values from input
  int ew_read = analogRead(east_west);
  float ew = ew_read * (5.0 / 1023.0);
  Serial.println(ew);
  delay(100);

  int servoIn = digitalRead(SX_Control);
  Serial.print("servo: ");
  Serial.println(servoIn);

  //rotate
  analogWrite(SXPWM_Control, pwm); //pwm is speed control 0-225 (slow-fast)
  digitalWrite(SX_Control, HIGH);
  delay(1000);
  analogWrite(SXPWM_Control, 0);
  delay(1000);

  Serial.print("servo after: ");
  Serial.println(servoIn);






}
