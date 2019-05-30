#define DEBUG 0C
//ultrasonic
#define TRIGGER 12
#define ECHO    13
#define ENABLE_LEFT 10
#define ENABLE_RIGHT 11
#define MOTORL1 7
#define MOTORL2 6
#define MOTORR1 8
#define MOTORR2 9

#define THRESHOLD 500
const float Kp;
const float Mult = 1.1;
const float Speed = 100*Mult;
const float Speed_turn = 110*Mult;
const float Speed_turn_fast = 120*Mult;
#define NUMBER_OF_SENSORS 5
static const uint8_t sensor_pins[] = {A0, A1, A2, A3, A4};
enum DIRECTION {FORWARD, BACKWARD};
int distance;
bool obstacle_detected;


inline __attribute__((always_inline)) float calculate_error(int *value_array) {
  /*
    Calculates the current error value
    value_array - a pointer to the array holding sensors values
    return - calculated error value
  */
  static float last_error = 0;
  float error = 0;
  int detected_sensors = 0; // Number of sensors that've deteced a line
  for (int i=0; i<NUMBER_OF_SENSORS; ++i) {
    if (value_array[i] < THRESHOLD) {
    error += i - 2.f; // Gives us values from [-2,2], 0 for the middle sensor
    ++detected_sensors;
    }
  }
  if (detected_sensors > 0) {
    error = error/static_cast<float>(detected_sensors);
    last_error = error;
  }
  else {
    error = last_error;
  }
  return error;
}

inline __attribute__((always_inline)) void ride(float error) {
  /*
  Moves the motors according to the PD controller
  */
  float u; // Controller output
  u = error*Kp;
  float left_speed;
  float right_speed;
  if (error < 0) {
    right_speed = Speed-u;
    left_speed = 0;
    if (right_speed < 0)   right_speed = 0;
    if (right_speed > 255) right_speed = 255;
  }
  else if (error > 0) {
    left_speed  = Speed+u;
    right_speed = 0;
    if (left_speed < 0)     left_speed = 0;
    if (left_speed > 255)   left_speed = 255;
  }
  else {
    left_speed = Speed;
    right_speed = Speed;
  }
  analogWrite(ENABLE_LEFT,  left_speed);
  analogWrite(ENABLE_RIGHT, right_speed);
}

inline __attribute__((always_inline)) int measure_distance() {
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

inline __attribute__((always_inline)) void avoid() {
  analogWrite(ENABLE_LEFT, 150); analogWrite(ENABLE_RIGHT, 0);
  delay(500);
  analogWrite(ENABLE_LEFT, 150); analogWrite(ENABLE_RIGHT, 150);
  delay(400);
  analogWrite(ENABLE_LEFT, 0); analogWrite(ENABLE_RIGHT, 150);
  delay(600);

  analogWrite(ENABLE_LEFT, 150); analogWrite(ENABLE_RIGHT, 150);
  delay(300);

  analogWrite(ENABLE_LEFT, 0); analogWrite(ENABLE_RIGHT, 150);
  delay(500);
  analogWrite(ENABLE_LEFT, 150); analogWrite(ENABLE_RIGHT, 150);
  delay(315);
  analogWrite(ENABLE_LEFT, 150); analogWrite(ENABLE_RIGHT, 0);
  delay(300);
}

#if DEBUG==1
inline __attribute__((always_inline)) void serial_debug(int *sensor_values, int distance) {
  /*
  Debug function. Sends all useful data to serial port.
  */
  for(int i=0; i<NUMBER_OF_SENSORS; ++i) {
    Serial.print(sensor_values[i], DEC);
    Serial.print(',');
  }
  Serial.print(distance, DEC);
  Serial.print('\n');
  Serial.println(millis(),DEC);
}
#endif

void setup() { // Runs once on boot
  
  // Setup ultrasonic sensor pins
  pinMode(TRIGGER, OUTPUT);
  pinMode(ECHO, INPUT);
  // Setup motor pins
  pinMode(ENABLE_LEFT, OUTPUT);
  pinMode(ENABLE_RIGHT, OUTPUT);
  pinMode(MOTORL1, OUTPUT);
  pinMode(MOTORL2, OUTPUT);
  pinMode(MOTORR1, OUTPUT);
  pinMode(MOTORR2, OUTPUT);
  // Setup direction
  digitalWrite(MOTORL2, LOW);
  digitalWrite(MOTORL1, HIGH);
  digitalWrite(MOTORR1, LOW);
  digitalWrite(MOTORR2, HIGH);
  // Setup sensor pins
  for (byte i = 0; i < NUMBER_OF_SENSORS; ++i) {
    pinMode(sensor_pins[i], INPUT);
  }
  #if DEBUG==1
  // Initialize serial connection
  Serial.begin(115200);
  #endif
  obstacle_detected = false;
}


void loop() { // Infinite loop
  int values[NUMBER_OF_SENSORS];
  float error;
  if (!obstacle_detected) {
    distance = measure_distance();
  }
  if (distance < 20 && distance > 7) {
    avoid();
    obstacle_detected = true;
    distance = 2000;
  }
  // Read sensor values
  for (byte i = 0; i < NUMBER_OF_SENSORS; ++i) {
    values[i] = analogRead(sensor_pins[i]);
  }
  // Measure distance  
  error = calculate_error(values);
  ride(error);
  #if DEBUG==1
  serial_debug(values, distance);
  #endif
}
