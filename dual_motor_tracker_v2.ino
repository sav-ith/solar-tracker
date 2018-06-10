#define blueToothSerial Serial2  //uses digital pin 16 to shield pin 6 and digital pin 17 to shield pin 7

//servo pins
#define SXPWM 5 //servoX PWM control pin
#define SX_Control 4 //servoX control pin

#define SYPWM 6 //servoY PWM control pin
#define SY_Control 7 //servoY control pin

//voltage sensor pins
//#define NORTH_SOUTH A0
//#define EAST_WEST A1
#define NORTH A5
#define SOUTH A4
#define EAST A2
#define WEST A3

//threshold voltages
#define VOLTAGE_MIN 0.00
#define VOLTAGE_MAX 0.10

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
float north = 0;
float south = 0;


bool east_west_delayed = false;
bool north_south_delayed = true;
bool x_direction = HIGH;
bool y_direction = HIGH;

int rotations = 0;




void setup() {
  setupBlueToothConnection();

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
void toCentre() {

  Serial.println("sending to centre");

  if (!east_west_delayed) {
    digitalWrite(SX_Control, LOW);
    delay(ROTATE_TIME);
    angle_x = 0;
  }

  if (!north_south_delayed) {
    digitalWrite(SY_Control, LOW);
    delay(ROTATE_TIME);
    angle_y = 0;
  }
}


//poll voltages
void checkVoltage() {
  //read analog pin
  north = analogRead(NORTH) * (3.3 / 1023.0);
  south = analogRead(SOUTH) * (3.3 / 1023.0);
  east = analogRead(EAST) * (3.3 / 1023.0);
  west = analogRead(WEST) * (3.3 / 1023.0);

  float east_west = east - west;
  float north_south = north - south;

  Serial.print("EW_diff:  ");
  Serial.println(east_west);
  Serial.print("NS_diff: ");
  Serial.println(north_south);

  Serial.print("north: ");
  Serial.println(north);
  Serial.print("south: ");
  Serial.println(south);

  Serial.print("east: ");
  Serial.println(east);
  Serial.print("west: ");
  Serial.println(west);

  //delay rotation if voltage is in required range
  if (abs(east_west) >= VOLTAGE_MIN && abs(east_west) <= VOLTAGE_MAX) {
    digitalWrite(SXPWM, LOW);
    delay(345);
    east_west_delayed = true;
    Serial.println("rotation east/west delayed");
  }

  //choose direction to rotate in based on polarity of voltage difference
  else if ((east_west) > 0) { //east LDR more shaded than west so rotate CW
    x_direction = HIGH;
    east_west_delayed = false;
    rotations++;
  }
  else { //west LDR more shaded than east so rotate CCW
    x_direction = LOW;
    east_west_delayed = false;
    rotations++;
  }

  if (abs(north_south) >= VOLTAGE_MIN && abs(north_south) <= VOLTAGE_MAX) {
    digitalWrite(SYPWM, LOW);
    delay(345);
    north_south_delayed = true;
    Serial.println("rotation north/south delayed");
  }

  if ((north_south) > 0) { //east LDR more shaded than west so rotate CCW
    y_direction = LOW;
    north_south_delayed = false;
    rotations++;
  }

  else { //west LDR more shaded than east so rotate CW
    y_direction = HIGH;
    north_south_delayed = false;
    rotations++;
  }
}


void loop() {

  //check voltage from sensor circuit
  char recvChar;
  while (1) {
    if (blueToothSerial.available())
      digitalWrite(SXPWM, LOW);
    digitalWrite(SYPWM, LOW);
    { //check if there's any data sent from the remote bluetooth shield
      recvChar = blueToothSerial.read();
      if (recvChar == 's')
      {
        Serial2.println("\nDATA:  ");
        blueToothSerial.print("EW_diff: ");
        Serial2.println(east_west);
        Serial2.print("NS_diff: ");
        Serial2.write(north_south);
        Serial2.print("number of rotations:");
        Serial2.println(rotations);
        Serial2.print("response time: ");
        Serial2.print(345 * rotations);
        Serial2.println(" ms");
        Serial2.print("x axis angle: ");
        Serial2.println(angle_x);
        Serial2.print("y axis angle: ");
        Serial2.println(angle_y);
        Serial2.print("projected power generated: ");
        Serial2.print((float)24 * ((0.00575 * rotations) * (0.06)) * 100);
        Serial2.println("\n");

      }
      else if (recvChar == 'p')
      {
        Serial.println("pausing");
        Serial2.println("pausing");
        blueToothSerial.print("\r\npausing\r\n");
        delay(1000);
      }
      else {
        //do nothing
      }
      digitalWrite(SXPWM, HIGH);
      digitalWrite(SYPWM, HIGH);
    }
    checkVoltage();

    if (!east_west_delayed)
      digitalWrite(SXPWM, HIGH);
    if (!north_south_delayed)
      digitalWrite(SYPWM, HIGH);

    //while current angle is less than max, continue rotation in current direction
    while (angle_y <= ANGLE_MAX && angle_x <= ANGLE_MAX) {
      digitalWrite(SX_Control, x_direction);
      digitalWrite(SY_Control, y_direction);
      delay(345);
      Serial.println("rotation successful");
      angle_y += 15;
      angle_x += 15;
      checkVoltage();
    }

    //once angle has reached maximum angle, return servo to centre position
    toCentre();
  }

}

void setupBlueToothConnection()
{
  blueToothSerial.begin(38400);                           // Set BluetoothBee BaudRate to default baud rate 38400
  blueToothSerial.print("\r\n+STWMOD=0\r\n");             // set the bluetooth work in slave mode
  blueToothSerial.print("\r\n+STNA=LeahBTSlave\r\n");    // set the bluetooth name as "SeeedBTSlave"
  blueToothSerial.print("\r\n+STOAUT=1\r\n");             // Permit Paired device to connect me
  blueToothSerial.print("\r\n+STAUTO=0\r\n");             // Auto-connection should be forbidden here
  delay(2000);                                            // This delay is required.
  blueToothSerial.print("\r\n+INQ=1\r\n");                // make the slave bluetooth inquirable
  Serial.println("The slave bluetooth is inquirable!");
  delay(2000);                                            // This delay is required.
  blueToothSerial.flush();
}

