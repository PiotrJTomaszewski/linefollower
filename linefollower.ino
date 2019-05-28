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
const float mult = 1.2;
const float Speed = 110*mult;
const float Speed_turn = 90*mult;
const float Speed_turn_fast = 105*mult;
#define NUMBER_OF_SENSORS 5
static const uint8_t sensor_pins[] = {A0, A1, A2, A3, A4};
enum DIRECTION {FORWARD, BACKWARD};
typedef unsigned char BYTE; // Unsigned char holds 1 byte of data
int last_sensor;
const int us_delay = 10;
int iteration;
int distance;
bool obstacle_detected;

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
      Serial.print(value_array[i], DEC);
      Serial.print(", ");
  }
    Serial.print('\n');

  //value_array[0] += 100;
  return 0; // Just for now
}

int get_minimum_sensor(int *values) {
  int minimum_val = 2000;
  int minimum_sensor;
  int sensors[] = {0,4,1,3,2};
  for (int i = 0; i < NUMBER_OF_SENSORS; ++i) {
    if (values[sensors[i]] < minimum_val && values[sensors[i]] < 100) {
      minimum_val = values[sensors[i]];
      minimum_sensor = sensors[i];
      return minimum_sensor;
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

void avoid() {
  left_motor(150,FORWARD); right_motor(0,FORWARD);
  delay(500);
  left_motor(150,FORWARD); right_motor(150,FORWARD);
  delay(400);
  left_motor(0,FORWARD); right_motor(150,FORWARD);
  delay(600);

  left_motor(150,FORWARD); right_motor(150,FORWARD);
  delay(400);

  left_motor(0,FORWARD); right_motor(150,FORWARD);
  delay(500);
  left_motor(150,FORWARD); right_motor(150,FORWARD);
  delay(500);
  left_motor(150, FORWARD); right_motor(0,FORWARD);
  delay(300);
  
 // left_motor(0,FORWARD); right_motor(0,FORWARD);
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
  //avoid();
  //delay(1000000);
  obstacle_detected = false;
}


void loop() { // Infinite loop
    int values[NUMBER_OF_SENSORS];
    read_sensor_values(values);
    int minimum_sensor = get_minimum_sensor(values);

    //test();
    if (!obstacle_detected) {
      distance = measure_distance();
    }
    //Serial.println(distance, DEC);
    Serial.println("\n");
    if(minimum_sensor == -1) {
      minimum_sensor = last_sensor;
    }
    if (distance < 20 && distance > 7) {
      avoid();
      obstacle_detected = true;
      distance = 2000;
      minimum_sensor = 2;
    }
    else {  
      switch(minimum_sensor) {
        case 0: left_motor(0,FORWARD); right_motor(Speed_turn_fast,FORWARD); break;
        case 1: left_motor(0,FORWARD); right_motor(Speed_turn,FORWARD);  break;
        case 2: left_motor(Speed,FORWARD); right_motor(Speed,FORWARD); break;
        case 3: left_motor(Speed_turn,FORWARD);  right_motor(0,FORWARD);  break;
        case 4: left_motor(Speed_turn_fast,FORWARD); right_motor(0,FORWARD); break;
      }
    }
    last_sensor = minimum_sensor;
    delay(10);
}
