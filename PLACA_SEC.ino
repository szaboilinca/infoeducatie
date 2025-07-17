#include <AccelStepper.h>

// Stepper setup (type 1 = DRIVER)
AccelStepper stepperY(1, 3, 6); // Step = 3, Dir = 6
AccelStepper stepperX(1, 2, 5); // Step = 3, Dir = 6

// Ultrasonic sensor pins
const int trigPin1 = A3; // Trigger
const int echoPin1 = A2; // Echo

const int trigPin2 = A0; // Trigger
const int echoPin2 = A1; // Echo

const int stopDistance1 = 270; // in mm
const int stopDistance2 = 265; // in mm

void setup() {
  Serial.begin(9600);
  Serial.println("Send '0' to start moving s1/Send '1' to start moving s2");

  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);

  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);

  stepperY.setMaxSpeed(3200);
  stepperY.setAcceleration(2000);

  stepperX.setMaxSpeed(3200);
  stepperX.setAcceleration(2000);
}

void loop() {
  if (Serial.available()) {
    char command = Serial.read();

    if (command == '0') {
      bottlemove(stopDistance1);
      platemove(stopDistance2);

      //----------------------------------------------------------

    }
  }
}

// Function to get distance from ultrasonic sensor in mm
long getDistanceMM1() {
  digitalWrite(trigPin1, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin1, LOW);

  long duration1 = pulseIn(echoPin1, HIGH, 30000); // Timeout: 30ms (5m)
  long distance1 = duration1 * 0.343 / 2; // mm (speed of sound = 343 m/s)

  return distance1;
}

long getDistanceMM2() {
  digitalWrite(trigPin2, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin2, LOW);

  long duration2 = pulseIn(echoPin2, HIGH, 30000); // Timeout: 30ms (5m)
  long distance2 = duration2 * 0.343 / 2; // mm (speed of sound = 343 m/s)

  return distance2;
}

long bottlemove(int stopDistance){
  Serial.println("Stepper moving...");

      
      stepperY.move(100000); 

      while (stepperY.isRunning()) {
        long distance1 = getDistanceMM1();

        Serial.print("Distance: ");
        Serial.print(distance1);
        Serial.println(" mm");

        if (distance1 <= stopDistance) {
          Serial.println("Object too close. Stopping.");
          stepperY.stop(); 
          break;
        }

        stepperY.run();
      }

      Serial.println("Stepper stopped.");

}

long platemove(int stopDistance){
   Serial.println("Stepper moving...");

     
      stepperX.move(-100000); 

      while (stepperX.isRunning()) {
        long distance2 = getDistanceMM2();

        Serial.print("Distance: ");
        Serial.print(distance2);
        Serial.println(" mm");

        if (distance2 <= stopDistance) {
          Serial.println("Object too close. Stopping.");
          stepperX.stop(); 
          break;
        }

        stepperX.run();
      }


}