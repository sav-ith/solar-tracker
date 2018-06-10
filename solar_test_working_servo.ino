//servo pins
#define SXPWM 5 //servoX PWM control pin
#define SX_Control 4 //servoX control pin

#define SYPWM 6 //servoY PWM control pin
#define SY_Control 7 //servoY control pin

//voltage sensor pins
#define NORTH_SOUTH A0
#define EAST_WEST A1
#define NORTH A4
#define SOUTH A5


//threshold voltages
#define VOLTAGE_MIN 1.50
#define VOLTAGE_MAX 1.56

//threshold angles
#define ANGLE_MAX 180
#define ANGLE_MIN -180

//time taken to rate 180 degrees in ms
#define ROTATE_TIME 4140

//current angle 
int angle_y = 0;
int angle_x = 0;

//voltages from sensor
float east_west = 0;
float north_south = 0;
float east = 0;
float west = 0;

bool east_west_delayed = false;
bool north_south_delayed = true;

void setup() {
  // put your setup code here, to run once:
  for (int j = 4; j <= 7; j++)
    pinMode(j, OUTPUT);

  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);

  Serial.begin(9600);
}


//send servo motor to centre pos
void toCentre(bool rotate_direction) {
  
  Serial.println("sending to centre");
  
  if (rotate_direction) {
    if(!east_west_delayed)
      digitalWrite(SX_Control, LOW);
    if(!north_south_delayed)
     digitalWrite(SY_Control, LOW);
    delay(ROTATE_TIME);
    
  }
  
  else {
    if(!east_west_delayed)
      digitalWrite(SX_Control, HIGH);
    if(!north_south_delayed)
      digitalWrite(SY_Control, HIGH);
    delay(ROTATE_TIME);
  }
  angle_x = 0;
  angle_y = 0;
  east_west_delayed = false;
  north_south_delayed = false;
}


//poll voltages 
void checkVoltage() {
  //read analog pin
  east_west = analogRead(EAST_WEST) * (3.3 / 1023.0);
  north_south = analogRead(NORTH_SOUTH) * (3.3 / 1023.0);
  east = analogRead(NORTH) * (3.3 / 1023.0);
  west = analogRead(SOUTH) * (3.3 / 1023.0);

  Serial.print("east/west:  ");
  Serial.println(east_west);
  Serial.print("north/south: ");
  Serial.println(north_south);
  Serial.print("east: ");
  Serial.println(east);
  Serial.print("west: ");
  Serial.println(west);

  //delay rotation if voltage is in required range
  if (abs(east_west) > VOLTAGE_MIN && abs(east_west) < VOLTAGE_MAX) {
    digitalWrite(SXPWM, LOW);
    delay(1000);
    east_west_delayed = true;
    Serial.println("rotation east/west delayed");
  }

    if (abs(north_south) > VOLTAGE_MIN && abs(north_south) < VOLTAGE_MAX) {
    digitalWrite(SYPWM, LOW);
    delay(1000);
    north_south_delayed = true;
    Serial.println("rotation north/south delayed");
  }
}


void loop() {

  //check voltage from sensor circuit

  checkVoltage();
  digitalWrite(SXPWM, HIGH);
  digitalWrite(SYPWM, HIGH);

  //while current angle is greater than max, continue rotation in current direction
  while (angle_y <= ANGLE_MAX && angle_x <= ANGLE_MAX) {
    digitalWrite(SX_Control, HIGH);
    digitalWrite(SY_Control, HIGH);
    delay(345);
    Serial.println("rotation successful");
    angle_y += 15;
    angle_x += 15;
    checkVoltage();
  }

  //once angle has reached maximum angle, return servo to centre position
  toCentre(true);
  digitalWrite(SXPWM, HIGH);
  digitalWrite(SYPWM, HIGH);
  
  //rotate in opposite direction
  while (angle_y >= ANGLE_MIN && angle_x >= ANGLE_MIN) {  
    digitalWrite(SX_Control, LOW);
    digitalWrite(SY_Control, LOW);
    delay(345);
    Serial.println("rotation successful");
    angle_y -= 15;
    angle_x -= 15;
    checkVoltage();
  }


  //once angle has reached minimum angle, return servo to centre position
  toCentre(false);

}
