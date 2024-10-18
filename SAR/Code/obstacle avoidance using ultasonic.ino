#include <NewPing.h>

#define TRIGGER_PIN  19  // ESP32 GPIO pin connected to the trigger pin of HC-SR04
#define ECHO_PIN     23 // ESP32 GPIO pin connected to the echo pin of HC-SR04
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

#define MOTOR1_PIN1 15 // Example motor driver pins
#define MOTOR1_PIN2 4
#define MOTOR2_PIN1 5
#define MOTOR2_PIN2 18

#define MIN_DISTANCE_TO_OBSTACLE 20 // Minimum distance to obstacle for the robot to react (in centimeters)
#define REVERSE_TIME 1000 // Time (in milliseconds) for the robot to reverse
#define TURN_TIME 1000 // Time (in milliseconds) for the robot to turn

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

void setup() {
  Serial.begin(9600); // Open serial monitor at 9600 baud to see distance readings
  
  pinMode(MOTOR1_PIN1, OUTPUT);
  pinMode(MOTOR1_PIN2, OUTPUT);
  pinMode(MOTOR2_PIN1, OUTPUT);
  pinMode(MOTOR2_PIN2, OUTPUT);
}

void loop() {
  delay(50); // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
  
  unsigned int distance = sonar.ping_cm(); // Send ping, get distance in cm and print result (0 = outside set distance range)
  
  if (distance <= 0 || distance >= MAX_DISTANCE) {
    Serial.println("Error: Distance measurement failed.");
    return;
  }
  
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  // If an obstacle is detected within the minimum distance
  if (distance < MIN_DISTANCE_TO_OBSTACLE) {
    stopRobot();
    delay(REVERSE_TIME); // Reverse for a certain amount of time
    turnLeft(); // Turn left
    delay(TURN_TIME); // Wait for the turn to complete
  } else {
    moveForward(); // If no obstacle detected, move forward
  }
}

void moveForward() {
  digitalWrite(MOTOR1_PIN1, HIGH);
  digitalWrite(MOTOR1_PIN2, LOW);
  digitalWrite(MOTOR2_PIN1, HIGH);
  digitalWrite(MOTOR2_PIN2, LOW);
}

void stopRobot() {
  digitalWrite(MOTOR1_PIN1, LOW);
  digitalWrite(MOTOR1_PIN2, LOW);
  digitalWrite(MOTOR2_PIN1, LOW);
  digitalWrite(MOTOR2_PIN2, LOW);
}

void turnLeft() {
  digitalWrite(MOTOR1_PIN1, LOW);
  digitalWrite(MOTOR1_PIN2, HIGH);
  digitalWrite(MOTOR2_PIN1, HIGH);
  digitalWrite(MOTOR2_PIN2, LOW);
}
