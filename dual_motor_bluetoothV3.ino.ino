#define blueToothSerial Serial2  //uses digital pin 16 to shield pin 6 and digital pin 17 to shield pin 7

//servo pins
#define SXPWM 5 //servoX PWM control pin
#define SX_Control 4 //servoX control pin

#define SYPWM 6 //servoY PWM control pin
#define SY_Control 7 //servoY control pin

//voltage sensor pins
#define NORTH_SOUTH A0
#define EAST_WEST A1
#define NORTH A5
#define SOUTH A4
#define EAST A2
#define WEST A3

//threshold voltages
#define VOLTAGE_MIN 1.50
#define VOLTAGE_MAX 1.56

//threshold angles
#define ANGLE_MAX 180
#define ANGLE_MIN -180

//time taken to rotate servo 180 degrees in ms
#define ROTATE_TIME_180 4140

//time taken to rotate servo 15 degrees in ms
#define ROTATE_TIME_15 345

//current angle
int angle_y = 0;
int angle_x = 0;

//voltages from sensor
float east_west = 0;
float north_south = 0;
float east = 0;
float west = 0;
float north = 0;
float south = 0;

//if servo is delayed
bool east_west_delayed = false;
bool north_south_delayed = false;

//true corresponds to writing HIGH to the digital pin
bool x_direction = true;
bool y_direction = true;

//number of rotations
int rotations = 0;

void setup()
{

  for (int j = 4; j <= 7; j++)
    pinMode(j, OUTPUT);

  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);

  Serial.begin(9600);
  Serial.println("connected");
  
  //attachInterrupt(digitalPinToInterrupt(4), bluetoothSend, CHANGE);

  //setupBlueToothConnection();
}

void toCentre(char servo, bool rotate_direction) {

  Serial.println("sending to centre");

  //rotate in LOW direction, ensures only servo that has exceeded angle limit is rotated
  if (rotate_direction) {
    if (!east_west_delayed && servo == 'x')
      digitalWrite(SX_Control, LOW);
    if (!north_south_delayed && servo == 'y')
      digitalWrite(SY_Control, LOW);
    delay(ROTATE_TIME_180);
  }

  //rotate in HIGH DIRECTION
  else {
    if (!east_west_delayed && servo == 'x')
      digitalWrite(SX_Control, HIGH);
    if (!north_south_delayed && servo == 'y')
      digitalWrite(SY_Control, HIGH);
    delay(ROTATE_TIME_180);
  }

  angle_x = 0;
  angle_y = 0;
}
void checkVoltage() {
  //read analog pin
  east_west = analogRead(EAST_WEST) * (3.3 / 1023.0);
  north_south = analogRead(NORTH_SOUTH) * (3.3 / 1023.0);
  north = analogRead(NORTH) * (3.3 / 1023.0);
  south = analogRead(SOUTH) * (3.3 / 1023.0);
  east = analogRead(EAST) * (3.3 / 1023.0);
  west = analogRead(WEST) * (3.3 / 1023.0);


  //delay rotation if voltage is in required range
  if (abs(east_west) > VOLTAGE_MIN && abs(east_west) < VOLTAGE_MAX) {
    digitalWrite(SXPWM, LOW);
    east_west_delayed = true;
    Serial.println("rotation east/west delayed");
    rotations++;
    return;
  }

  if (abs(north_south) > VOLTAGE_MIN && abs(north_south) < VOLTAGE_MAX) {
    digitalWrite(SYPWM, LOW);
    north_south_delayed = true;
    Serial.println("rotation north/south delayed");
    rotations++;
    return;
  }

  east_west_delayed = false;
  north_south_delayed = true;
}

void loop() {

  //check voltage from sensor
  checkVoltage();
  
  //send servo to centre if exceeded max or min angle (+/- 180degrees)
  if (angle_x >= ANGLE_MAX) {
    toCentre('x', false);
    x_direction = false;
  }

  if (angle_y >= ANGLE_MIN) {
    toCentre('y', false);
    y_direction = false;
  }

  if (angle_x <= ANGLE_MIN) {
    toCentre('x', true);
    x_direction = true;
  }

  if (angle_y <= ANGLE_MIN) {
    toCentre('y', true);
    y_direction = true;
  }

  if(north -  south < 0){
    y_directon = true;
  }

  else{
    y_direction = false;
  }

  if(east - west < 0){
    x_direction = true;
  }

  else{
    x_direction = false;
  }
  
  //rotate servos 15degrees if not delayed
  if (!east_west_delayed) {
    digitalWrite(SXPWM, HIGH);
    //rotate in correct direction
    digitalWrite(SX_Control, (x_direction) ? HIGH : LOW);
    angle_x += 15;
  }

  if (!north_south_delayed) {
    digitalWrite(SYPWM, HIGH);
    digitalWrite(SY_Control, (y_direction) ? HIGH : LOW);
    angle_y += 15;
  }

  delay(ROTATE_TIME_15);
}
