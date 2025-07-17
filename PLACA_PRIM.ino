#include <AccelStepper.h>
#include <Servo.h>

// Enable pin for all drivers (shared)
#define EN_PIN 8

// Limit switch pins
#define X_LIMIT 9
#define Y_LIMIT 10
#define Z_LIMIT 11
#define SENSOR_PIN 14

#define PI 3.14159265
// Create stepper objects
AccelStepper stepperX(1, 2, 5);
AccelStepper stepperY(1, 3, 6);
AccelStepper stepperZ(1, 4, 7);
AccelStepper stepperE(1, 12, 13);

double x0, y0, z0;
double theta1=0, theta2=0, theta3=0;
double Targetx, Targety, Targetz;
int i=0;
int HOME_POSITION;

// robot geometry
 const double e = 103.273;     // end effector
 const double f = 259.808;     // base
 const double re = 300.0;
 const double rf = 100.0;

 // trigonometric constants
 const double sqrt3 = sqrt(3.0);
 const double pi = 3.141592653;    // PI
 const double sin120 = sqrt3/2.0;
 const double cos120 = -0.5;
 const double tan60 = sqrt3;
 const double sin30 = 0.5;
 const double tan30 = 1/sqrt3;

//servo

Servo myServo;
const int servoPin = 15;

void setup() {
   Serial.begin(9600);

  // Enable all stepper drivers
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW); // LOW = enabled on A4988

  //configure servo
  myServo.attach(servoPin);

  // Configure limit switches
  pinMode(X_LIMIT, INPUT_PULLUP); // Switch not pressed = HIGH, pressed = LOW
  pinMode(Y_LIMIT, INPUT_PULLUP);
  pinMode(Z_LIMIT, INPUT_PULLUP);

  // Set speed and acceleration for each axis
  stepperX.setMaxSpeed(400);
  stepperX.setAcceleration(200);

  stepperY.setMaxSpeed(400);
  stepperY.setAcceleration(200);

  stepperZ.setMaxSpeed(400);
  stepperZ.setAcceleration(200);

  stepperE.setMaxSpeed(1000);
  stepperE.setAcceleration(500);

  Serial.println("Setup status: Done");
  Serial.println("Use the /library && /lb command in order to access all the detail reffering to the operation of the robot");
}

void loop() {
  
  if(i==0){
    Serial.println("Type new command...");
    i=1;
  }
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n'); // Read input line
    command.trim(); // Remove whitespace
//HOME
    if (command.equalsIgnoreCase("home")) {
      Serial.println("Starting homing sequence...");

      homeAxis(stepperX, X_LIMIT, -1);
      Serial.println("X homed");

      homeAxis(stepperY, Y_LIMIT, -1);
      Serial.println("Y homed");

      homeAxis(stepperZ, Z_LIMIT, -1);
      Serial.println("Z homed");

      Serial.println("All axes homed.");
      i=0;

//NEW TARGET
    }else if (command.equalsIgnoreCase("newtarget") || command.equalsIgnoreCase("nt")) {
      Serial.println("Enter new target of end effector (X Y Z), space-separated (e.g., 100 200 -150):");

      // Wait for input
      while (!Serial.available()) {
        // Wait for user input
      }

      String input = Serial.readStringUntil('\n');
      input.trim();


      // Split input by space
      int firstSpace = input.indexOf(' ');
      int secondSpace = input.indexOf(' ', firstSpace + 1);

      if (firstSpace > 0 && secondSpace > firstSpace) {
        x0 = input.substring(0, firstSpace).toDouble();
        y0 = input.substring(firstSpace + 1, secondSpace).toDouble();
        z0 = input.substring(secondSpace + 1).toDouble();
        Serial.println("receptionat");

      }
      if (delta_calcAngleYZ(x0, y0, z0, theta1) == 0)
        Serial.println("theta1 = " + String(-theta1, 4));
      else
        Serial.println("theta1: Invalid");

      if (delta_calcAngleYZ(x0*cos120 + y0*sin120, y0*cos120-x0*sin120, z0, theta2) == 0)
        Serial.println("theta2 = " + String(-theta2, 4));
      else
        Serial.println("theta2: Invalid");

      if (delta_calcAngleYZ(x0*cos120 - y0*sin120, y0*cos120+x0*sin120, z0, theta3) == 0)
        Serial.println("theta3 = " + String(-theta3, 4));
      else
        Serial.println("theta3: Invalid");

      Targetx=-theta1*1.8*5+810;
        Serial.println("Targetx = " + String(Targetx, 4));

      Targety=-theta2*1.8*5+810;
        Serial.println("Targety = " + String(Targety, 4));

      Targetz=-theta3*1.8*5+810;
        Serial.println("Targety = " + String(Targetz, 4));
//MISCARE STEPPER X, Y, Z SIMULTANEOUSLY
int pullOffStepsX = abs(stepperX.currentPosition() - Targetx);
int pullOffStepsY = abs(stepperY.currentPosition() - Targety);
int pullOffStepsZ = abs(stepperZ.currentPosition() - Targetz);

// Determine directions and set speeds
stepperX.setSpeed(stepperX.currentPosition() > Targetx ? 700 : -700);
stepperY.setSpeed(stepperY.currentPosition() > Targety ? 700 : -700);
stepperZ.setSpeed(stepperZ.currentPosition() > Targetz ? 700 : -700);

// Find the maximum number of steps needed
int maxSteps = max(pullOffStepsX, max(pullOffStepsY, pullOffStepsZ));

// Move all motors simultaneously
for (int i = 0; i < maxSteps; i++) {
    if (i < pullOffStepsX) stepperX.runSpeed();
    if (i < pullOffStepsY) stepperY.runSpeed();
    if (i < pullOffStepsZ) stepperZ.runSpeed();
    
    delayMicroseconds(500);  // More precise timing than delay(1)
    yield();
}

// Update positions
stepperX.setCurrentPosition(Targetx);
stepperY.setCurrentPosition(Targety);
stepperZ.setCurrentPosition(Targetz);
    

    }else if (command.equalsIgnoreCase("count") || command.equalsIgnoreCase("ct")){
        dispenser();
    } else if (command.equalsIgnoreCase("library") || command.equalsIgnoreCase("lb")){
        library();
    } else if (command.equalsIgnoreCase("catch") || command.equalsIgnoreCase("ch")){
        catchobj();
    }else if (command.equalsIgnoreCase("release") || command.equalsIgnoreCase("rl")){
        release();
    }else if (command.equalsIgnoreCase("test") || command.equalsIgnoreCase("t")){
        test();
    }else if (command.equalsIgnoreCase("banda") || command.equalsIgnoreCase("bd")){
        banda();
    }
  }
}

// ---------------------------------------------
// Function to home a single axis
// ---------------------------------------------
void homeAxis(AccelStepper& stepper, int limitPin, int direction) {
  const int fastSpeed = 700;
  const int slowSpeed = 500;
  const int pullOffSteps = 150;

  //Skip if switch is already pressed at start (avoid crashing)
  if (digitalRead(limitPin) == LOW) {
    Serial.println("WARNING: Limit switch already pressed at start!");
    return;
  }

  // Step 1: Fast approach until switch is hit
  stepper.setMaxSpeed(fastSpeed);
  stepper.setSpeed(direction * fastSpeed);
  while (digitalRead(limitPin) == HIGH) {
    stepper.runSpeed();
    yield();
  }

  Serial.println("Switch hit!");
  
  // Optional: Step 2 - Pull back a bit
  stepper.setSpeed(-direction * slowSpeed);
  for (int i = 0; i < pullOffSteps; i++) {
    stepper.runSpeed();
    delay(1);
    yield();
  }

  delay(100); // Pause

  // Optional: Step 3 - Slow final approach
  stepper.setSpeed(direction * slowSpeed);
  while (digitalRead(limitPin) == HIGH) {
    stepper.runSpeed();
    yield();
  }

  // Step 4: Set position 
  stepper.setCurrentPosition(1080);

  return;
}

// ---------------------------------------------
// Function to calculate angle
// ---------------------------------------------

 int delta_calcAngleYZ(double x0, double y0, double z0, double &theta) {
     double y1 = -0.5 * 0.57735 * f; // f/2 * tg 30
     y0 -= 0.5 * 0.57735 * e;
     // z = a + b*y
     double a = (x0*x0 + y0*y0 + z0*z0 +rf*rf - re*re - y1*y1)/(2*z0);//habar nu am ce se intampla
     double b = (y1-y0)/z0;
     // discriminant
     double d = -(a+b*y1)*(a+b*y1)+rf*(b*b*rf+rf);
     if (d < 0) return -1; // non-existing point
     double yj = (y1 - a*b - sqrt(d))/(b*b + 1); // choosing outer point
     double zj = a + b*yj;
     theta = 180.0*atan(-zj/(y1 - yj))/pi + ((yj>y1)?180.0:0.0);//habar nu am ce se intampla x2
     return 0;
 }

// ---------------------------------------------
// Function to calculate number of pills
// ---------------------------------------------

void dispenser() {
  int detectionCount = 0;
  int lastSensorState = digitalRead(SENSOR_PIN);

  stepperE.setSpeed(125);  // Set speed for continuous motion
  Serial.println("Dispensing started...");

  while (detectionCount < 4) {
    stepperE.runSpeed();  // Keeps motor moving

    int currentSensorState = digitalRead(SENSOR_PIN);

    // Count falling edge (object appears)
    if (currentSensorState == LOW && lastSensorState == HIGH) {
      detectionCount++;
      Serial.print("Object Detected #");
      Serial.println(detectionCount);
    }

    lastSensorState = currentSensorState;
  }

  stepperE.setSpeed(0);  // Stop motor
  Serial.println("Dispensing complete.");
}

// ---------------------------------------------
// Function for explaining commands
// ---------------------------------------------

void library(){
  //title
  Serial.print("--------------------------LIBRARY--------------------------");

  //home
  Serial.print("/home && /hm");
  Serial.print("  ->command used for calibrating the position of the robotic arm");

  //new target
  Serial.print("/newtarget && /nt");
  Serial.print("  ->command used for moving to a new target");

  //count
  Serial.print("/count && /ct");
  Serial.print("  ->command used for counting the number of pills needed");

  //catch
  Serial.print("/catch && /ch");
  Serial.print("  ->command used for catching the bottle with the claw mechanism");
}

void catchobj() {
  // Go to "catch" position — consistent and exact
  myServo.writeMicroseconds(2050);  // Adjust this if needed for your servo
  delay(500); // Time for the servo to reach the position
}

void release() {
  // Go to "release" position — slightly different than catch
  myServo.writeMicroseconds(1200);  // Adjust for your desired release position
  delay(500); // Time for the servo to reach the position
}

void test(){
  Serial.println("Starting homing sequence...");

      homeAxis(stepperX, X_LIMIT, -1);
      Serial.println("X homed");

      homeAxis(stepperY, Y_LIMIT, -1);
      Serial.println("Y homed");

      homeAxis(stepperZ, Z_LIMIT, -1);
      Serial.println("Z homed");

      Serial.println("All axes homed.");
      i=0;

      delay(500);
      release();
      gotopoint(0,0,-278);
      delay(500);
      catchobj();
      delay(500);
      gotopoint(0,0,-255);
      delay(500);
      dispenser();
      delay(500);
      gotopoint(0,-120,-255);
      delay(1000);
      gotopoint(0,-120,-278);
      release();
      delay(500);
      gotopoint(0,-120,-255);
      delay(500);
      gotopoint(0,0,-255);

      
}

void banda(){
  Serial.println("Starting homing sequence...");

      homeAxis(stepperX, X_LIMIT, -1);
      Serial.println("X homed");

      homeAxis(stepperY, Y_LIMIT, -1);
      Serial.println("Y homed");

      homeAxis(stepperZ, Z_LIMIT, -1);
      Serial.println("Z homed");

      Serial.println("All axes homed.");
      i=0;

      delay(500);
      release();
      gotopoint(0,-130,-270);
      delay(500);
      gotopoint(0,-130,-290);
      delay(500);
      catchobj();
      delay(500);
      dispenser();
      delay(500);
      gotopoint(0,-130,-250);
      delay(500);
      gotopoint(0,150,-240);//90
      delay(500);
      gotopoint(0,140,-270);
      release();
      delay(500);
      gotopoint(0,140,-250);
}

void gotopoint(int x0, int y0, int z0){
  if (delta_calcAngleYZ(x0, y0, z0, theta1) == 0)
        Serial.println("theta1 = " + String(-theta1, 4));
      else
        Serial.println("theta1: Invalid");

      if (delta_calcAngleYZ(x0*cos120 + y0*sin120, y0*cos120-x0*sin120, z0, theta2) == 0)
        Serial.println("theta2 = " + String(-theta2, 4));
      else
        Serial.println("theta2: Invalid");

      if (delta_calcAngleYZ(x0*cos120 - y0*sin120, y0*cos120+x0*sin120, z0, theta3) == 0)
        Serial.println("theta3 = " + String(-theta3, 4));
      else
        Serial.println("theta3: Invalid");

      Targetx=-theta1*1.8*5+810;
        Serial.println("Targetx = " + String(Targetx, 4));

      Targety=-theta2*1.8*5+810;
        Serial.println("Targety = " + String(Targety, 4));

      Targetz=-theta3*1.8*5+810;
        Serial.println("Targety = " + String(Targetz, 4));
//MISCARE STEPPER X, Y, Z SIMULTANEOUSLY
int pullOffStepsX = abs(stepperX.currentPosition() - Targetx);
int pullOffStepsY = abs(stepperY.currentPosition() - Targety);
int pullOffStepsZ = abs(stepperZ.currentPosition() - Targetz);

// Determine directions and set speeds
stepperX.setSpeed(stepperX.currentPosition() > Targetx ? 700 : -700);
stepperY.setSpeed(stepperY.currentPosition() > Targety ? 700 : -700);
stepperZ.setSpeed(stepperZ.currentPosition() > Targetz ? 700 : -700);

// Find the maximum number of steps needed
int maxSteps = max(pullOffStepsX, max(pullOffStepsY, pullOffStepsZ));

// Move all motors simultaneously
for (int i = 0; i < maxSteps; i++) {
    if (i < pullOffStepsX) stepperX.runSpeed();
    if (i < pullOffStepsY) stepperY.runSpeed();
    if (i < pullOffStepsZ) stepperZ.runSpeed();
    
    delayMicroseconds(500);  // More precise timing than delay(1)
    yield();


}
stepperX.setCurrentPosition(Targetx);
stepperY.setCurrentPosition(Targety);
stepperZ.setCurrentPosition(Targetz);
}