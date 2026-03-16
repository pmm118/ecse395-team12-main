/*
  Read data from an ultrasonic sensor connected to the ESP32 Development 
  Board and print the distance measurement in centimeters through the 
  serial communication.

  Board: ESP32 Development Board
  Component: Ultrasonic distance Sensor(HC-SR04)
*/

//#include <Arduino.h>

// Define the pin numbers for the ultrasonic sensor
const int echoPin = 26;
const int trigPin = 25;

// Traffic light pins
const int redPin = 13;
const int yellowPin = 12;
const int greenPin = 14;

// Distance thresholds (in cm)
// Adjust these values based on your litter box size
const float redThreshold = 5.0;      // distance <= 5 cm -> red
const float yellowThreshold = 10.0;  // 5 < distance <= 10 cm -> yellow
// distance > 10 cm -> green

float readDistance();

void setTrafficLight(bool redOn, bool yellowOn, bool greenOn);
void setup() {
  Serial.begin(115200);                    // Start serial communication with a baud rate of 9600
  pinMode(echoPin, INPUT);               // Set echo pin as input
  pinMode(trigPin, OUTPUT);              // Set trig pin as output

  pinMode(redPin, OUTPUT);
  pinMode(yellowPin, OUTPUT);
  pinMode(greenPin, OUTPUT);

  // Turn all lights off at start
  setTrafficLight(false, false, false);
  Serial.println("Ultrasonic sensor + Traffic Light system:");  // Print a message indicating the ultrasonic sensor is ready with LED
}

/// @brief 
void loop() {
  float distance = readDistance();  // Call the function to read the sensor data and get the distance
  Serial.print(distance);           // Print the distance value
  Serial.println(" cm");            // Print " cm" to indicate the unit of measurement
  delay(400);                       // Delay for 400 milliseconds before repeating the loop

 // Control traffic light based on distance
if (distance <= redThreshold && distance > 0) {
    // Trash is too high, clean immediately
    setTrafficLight(true, false, false);
    Serial.println("Status: RED - Clean immediately");
  } 
else if (distance <= yellowThreshold && distance > redThreshold) {
    // Trash is getting high, cleaning recommended
    setTrafficLight(false, true, false);
    Serial.println("Status: YELLOW - Cleaning recommended");
  } 
  else if (distance > yellowThreshold) {
    // Trash level is low, no cleaning needed
    setTrafficLight(false, false, true);
    Serial.println("Status: GREEN - No cleaning needed");
  } 
  else {
    // Invalid reading
    setTrafficLight(false, false, false);
  }
}
// Function to read the sensor data and calculate the distance
float readDistance() {
  digitalWrite(trigPin, LOW);   // Set trig pin to low to ensure a clean pulse
  delayMicroseconds(2);         // Delay for 2 microseconds
  digitalWrite(trigPin, HIGH);  // Send a 10 microsecond pulse by setting trig pin to high
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);  // Set trig pin back to low

  // Measure the pulse width of the echo pin and calculate the distance value
  float distance = pulseIn(echoPin, HIGH) / 58.00;  // Formula: (340m/s * 1us) / 2
  return distance;
}

void setTrafficLight(bool redOn, bool yellowOn, bool greenOn) {
  digitalWrite(redPin, redOn ? HIGH : LOW);
  digitalWrite(yellowPin, yellowOn ? HIGH : LOW);
  digitalWrite(greenPin, greenOn ? HIGH : LOW);
}