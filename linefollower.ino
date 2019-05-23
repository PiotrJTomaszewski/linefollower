// Ultrasonic
#define TRIGGER 12
#define ECHO    13
// Left motor
#define ENABLE_LEFT 10
#define MOTOR_LEFT1 7
#define MOTOR_LEFT2 6
// Right motor
#define ENABLE_RIGHT 11
#define MOTOR_RIGHT1 8
#define MOTOR_RIGHT2 9
// Sensors
#define NUMBER_OF_SENSORS 5
static const uint8_t sensor_pins[] = {A0, A1, A2, A3, A4};
// Regulators
#define KP 50
#define TD 50

#define BASE_SPEED 180
enum DIRECTION {FORWARD, BACKWARD};
int last_sensor; // Index of the last sensor that've detected a line

void left_motor(byte speed, enum DIRECTION direction) {
  /*
  Moves the left motor
  speed - 0-255
  direction - FORWARD, BACKWARD
  */
  switch (direction) {
    case FORWARD:  digitalWrite(MOTOR_LEFT2, LOW); digitalWrite(MOTOR_LEFT1, HIGH); break;
    case BACKWARD: digitalWrite(MOTOR_LEFT1, LOW); digitalWrite(MOTOR_LEFT2, HIGH); break;
  }
  analogWrite(ENABLE_LEFT, speed);
}

void right_motor(byte speed, enum DIRECTION direction) {
  /* 
  Moves the right motor
  speed - 0-255
  direction - FORWARD, BACKWARD
  */
  switch (direction) {
    case FORWARD:  digitalWrite(MOTOR_RIGHT1, LOW); digitalWrite(MOTOR_RIGHT2, HIGH); break;
    case BACKWARD: digitalWrite(MOTOR_RIGHT2, LOW); digitalWrite(MOTOR_RIGHT1, HIGH); break;
  }
  analogWrite(ENABLE_RIGHT, speed);
}

void ride(int error, int last_error) {
  /*
  Moves the motors according to the PD controller
  error - Current error value
  last_error - Error value from the last iteration
  */
}

void read_sensor_values(int *value_array) {
  /* 
  Reads values of sensors and stores them in value_array
  value_array - a pointer to the array
  */
  for (byte i = 0; i < NUMBER_OF_SENSORS; ++i) {
    value_array[i] = analogRead(sensor_pins[i]);
  }
}

int measure_distance() {
  /* 
  Returns distance in cm read from an ultrasonic sensor
  return - distance in cm
  */
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
  /* 
  Performs a preprogrammed series of moves to avoid an obstacle
  */
  left_motor(BASE_SPEED,FORWARD); right_motor(0,FORWARD);
  delay(500);
  left_motor(BASE_SPEED,FORWARD); right_motor(BASE_SPEED,FORWARD);
  delay(300);
  left_motor(0,FORWARD); right_motor(BASE_SPEED,FORWARD);
  delay(500);
  
  left_motor(BASE_SPEED,FORWARD); right_motor(BASE_SPEED,FORWARD);
  delay(600);
  
  left_motor(0,FORWARD); right_motor(BASE_SPEED,FORWARD);
  delay(500);
  left_motor(BASE_SPEED,FORWARD); right_motor(BASE_SPEED,FORWARD);
  delay(300);
  left_motor(BASE_SPEED,FORWARD); right_motor(0,FORWARD);
  delay(500);
}

void setup() { // Runs once on boot
  // Setup the ultrasonic sensor pins
  pinMode(TRIGGER, OUTPUT);
  pinMode(ECHO, INPUT);
  // Setup motors pins
  pinMode(ENABLE_LEFT, OUTPUT);
  pinMode(MOTOR_LEFT1, OUTPUT);
  pinMode(MOTOR_LEFT2, OUTPUT);
  pinMode(ENABLE_RIGHT, OUTPUT);
  pinMode(MOTOR_RIGHT1, OUTPUT);
  pinMode(MOTOR_RIGHT2, OUTPUT);
  // Turn off the motors
  digitalWrite(MOTOR_LEFT1, LOW);
  digitalWrite(MOTOR_LEFT2, LOW);
  digitalWrite(MOTOR_RIGHT1, LOW);
  digitalWrite(MOTOR_RIGHT2, LOW);
  // Setup sensors pins
  for (byte i = 0; i < NUMBER_OF_SENSORS; ++i) {
    pinMode(sensor_pins[i], INPUT);
  }
  // Initialize serial connection
  Serial.begin(115200);
  last_sensor = 2; // The last sensor is the middle one
}

void loop() { // Infinite loop
    int values[NUMBER_OF_SENSORS];
    int minimum_sensor;
    read_sensor_values(values);
    if(minimum_sensor == -1) {
      minimum_sensor = last_sensor;
    }


    last_sensor = minimum_sensor;
}
