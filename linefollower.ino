//ultrasonic
#define TRIGGER 12
#define ECHO    13
//ENABLE_L
#define ENABLE1 10
//ENABLE_R
#define ENABLE2 11
#define MOTORL1 7
#define MOTORL2 6
#define MOTORR1 8
#define MOTORR2 9
const int Speed = 130;
const int Speed_turn = 100;
#define NUMBER_OF_SENSORS 5
static const uint8_t sensor_pins[] = {A0, A1, A2, A3, A4};
enum DIRECTION {FORWARD, BACKWARD};
typedef unsigned char BYTE; // Unsigned char holds 1 byte of data
int last_sensor;

/* speed - 0-255
   direction - FORWARD, BACKWARD
*/
void left_motor(BYTE speed, enum DIRECTION direction) {
  switch (direction) {
    case FORWARD:  digitalWrite(MOTORL2, LOW); digitalWrite(MOTORL1, HIGH); break;
    case BACKWARD: digitalWrite(MOTORL1, LOW); digitalWrite(MOTORL2, HIGH); break;
    default:       Serial.println("Error: wrong direction!");               break;
  }
  analogWrite(ENABLE1, speed);
}

void right_motor(BYTE speed, enum DIRECTION direction) {
  switch (direction) {
    case FORWARD:  digitalWrite(MOTORR1, LOW); digitalWrite(MOTORR2, HIGH); break;
    case BACKWARD: digitalWrite(MOTORR2, LOW); digitalWrite(MOTORR1, HIGH); break;
    default:       Serial.println("Error: wrong direction!");               break;
  }
  analogWrite(ENABLE2, speed);
}


/* Reads values of sensors
*/
int read_sensor_values(int *value_array) {
  for (BYTE i = 0; i < NUMBER_OF_SENSORS; ++i) {
    value_array[i] = analogRead(sensor_pins[i]);
  }
  return 0; // Just for now
}

int get_minimum_sensor(int *values) {
  int minimum_val = 2000;
  int minimum_sensor;
  for (int i = 0; i < NUMBER_OF_SENSORS; ++i) {
    Serial.print(values[i], DEC);
    Serial.print(", ");
    if (values[i] < minimum_val && values[i] < 300) {
      minimum_val = values[i];
      minimum_sensor = i;
    }
  }
  Serial.print('\n');
  if (minimum_val == 2000) return -1; // Jeśli nie znaleziono min, to zwraca -1
  else return minimum_sensor;
}

int measure_distance() {
  // https://howtomechatronics.com/tutorials/arduino/ultrasonic-sensor-hc-sr04/
  long duration;
  int distance;
  // Clear trigger pin
  digitalWrite(TRIGGER, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIGGER, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER, LOW);
  // Get sound wave travel time
  duration = pulseIn(ECHO, HIGH);

  // Calculate distance
  distance = duration * 0.034/2;
  return distance;
}

void test() {
  //digitalWrite(MOTORL2, LOW);
  //digitalWrite(MOTORL1, HIGH);
  //analogWrite(ENABLE1, 150);
  left_motor(255, FORWARD);
  right_motor(255, FORWARD);
}

void setup() { // Runs once on boot
  // Setup ultrasonic sensor pins
  pinMode(TRIGGER, OUTPUT);
  pinMode(ECHO, INPUT);
  // Setup motor pins
  pinMode(ENABLE1, OUTPUT);
  pinMode(ENABLE2, OUTPUT);
  pinMode(MOTORL1, OUTPUT);
  pinMode(MOTORL2, OUTPUT);
  pinMode(MOTORR1, OUTPUT);
  pinMode(MOTORR2, OUTPUT);
  // Turn off the motors
  digitalWrite(MOTORL1, LOW);
  digitalWrite(MOTORL2, LOW);
  digitalWrite(MOTORR1, LOW);
  digitalWrite(MOTORR2, LOW);
  // Setup sensor pins
  for (BYTE i = 0; i < NUMBER_OF_SENSORS; ++i) {
    pinMode(sensor_pins[i], INPUT);
  }
  // Initialize serial connection
  Serial.begin(115200);
  last_sensor = 1;
}

void loop() { // Infinite loop
  int values[NUMBER_OF_SENSORS];
  read_sensor_values(values);
  int minimum_sensor = get_minimum_sensor(values);
 
  //test();
  //Serial.println(measure_distance(), DEC);
  //Serial.println("\n");
  if(minimum_sensor == -1) {
    minimum_sensor = last_sensor;
  }
  
  switch(minimum_sensor) {
    case 0: case 1: left_motor(0,FORWARD); right_motor(Speed_turn,FORWARD);  break;
    case 2: left_motor(Speed,FORWARD); right_motor(Speed,FORWARD); break;
    case 3: case 4: left_motor(Speed_turn,FORWARD);  right_motor(0,FORWARD);  break;
  }
  last_sensor = minimum_sensor;
  //delay(100);
}
