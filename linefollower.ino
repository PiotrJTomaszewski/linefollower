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
static const uint8_t Sensor_Pins[] = {A0, A1, A2, A3, A4};
#define THRESHOLD 600
// Controllers
#define KP 100
#define TD 20
float Error;
float Last_Error;

#define BASE_SPEED 100
enum DIRECTION {FORWARD, BACKWARD};

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

void ride() {
  /*
  Moves the motors according to the PD controller
  */
  float u; // Controller output
  if (Error == 0) // If there is no error go forward
    u = 0;
  else
    u = Error*KP + (Error-Last_Error)*TD;
  float left_speed  = BASE_SPEED+u;
  float right_speed = BASE_SPEED-u;
  if (left_speed < 0)     left_speed = 0;
  if (left_speed > 255)   left_speed = 255;
  left_motor(left_speed, FORWARD);
  if (right_speed < 0)   right_speed = 0;
  if (right_speed > 255) right_speed = 255;
  right_motor(right_speed, FORWARD);

}

void read_sensor_values(int *value_array) {
  /* 
  Reads values of sensors and stores them in value_array
  value_array - a pointer to the array to store sensors values
  */
  for (byte i = 0; i < NUMBER_OF_SENSORS; ++i) {
    value_array[i] = analogRead(Sensor_Pins[i]);
  }
  value_array[4] = 1337; // Jeden czujnik nie działa i muszę go wymienić :p
}

void calculate_error(int *value_array) {
  /*
    Calculates the current error value
    value_array - a pointer to the array holding sensors values
  */
  float local_error = 0;
  int detected_sensors = 0; // Number of sensors that've deteced a line
  for (int i=0; i<NUMBER_OF_SENSORS; ++i) {
    if (value_array[i] < THRESHOLD) {
    local_error += i - 2.f; // Gives us values from [-2,2], 0 for the middle sensor
    ++detected_sensors;
    }
  }
  if (detected_sensors > 0) {
    Last_Error = Error;
    Error = local_error/static_cast<float>(detected_sensors);
  }
  else {
     Error = Last_Error; // TODO: Probably not needed
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

void serial_debug(int *sensor_values) {
  /*
  Debug function. Sends all useful data to serial port.
  */
  for(int i=0; i<NUMBER_OF_SENSORS; ++i) {
    Serial.print(sensor_values[i], DEC);
    Serial.print(',');
  }
  Serial.print('\n');
  Serial.println(Error);
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
    pinMode(Sensor_Pins[i], INPUT);
  }
  // Initialize Error values
  Error = 0;
  Last_Error = 0;
  // Initialize serial connection
  Serial.begin(115200);
}

void loop() { // Infinite loop
    int values[NUMBER_OF_SENSORS];
    read_sensor_values(values);
    serial_debug(values);
    calculate_error(values);
    ride();
    delay(10);
}
