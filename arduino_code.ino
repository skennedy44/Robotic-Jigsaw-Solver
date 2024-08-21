#include <Servo.h>
#include "BraccioRobot.h"
#include <InverseK.h>

#define INPUT_BUFFER_SIZE 50

static char inputBuffer[INPUT_BUFFER_SIZE];
Position armPosition;

// Seting up the IK model
Link base, upperarm, forearm, hand;


void setup() {
  Serial.begin(115200);  // Begin serial communication
  BraccioRobot.init();

  // Initialize the IK model
  base.init(0, b2a(0.0), b2a(180.0));
  upperarm.init(200, b2a(15.0), b2a(165.0));
  forearm.init(200, b2a(0.0), b2a(180.0));
  hand.init(270, b2a(0.0), b2a(180.0));
  
  InverseK.attach(base, upperarm, forearm, hand);
}

// Continous loop waiting for an input
void loop() {
    handleInput();
}


//First reads serial data into a buffer
// Newline character ends the command
// input is passed into interpret command function
void handleInput() {
  if (Serial.available() > 0) {
    byte result = Serial.readBytesUntil('\n', inputBuffer, INPUT_BUFFER_SIZE);
    inputBuffer[result] = 0;
    interpretCommand(inputBuffer, result);
  }
}



void interpretCommand(char* inputBuffer, byte commandLength) {

  if (inputBuffer[0] == 'I') {
    Serial.print("Received input: ");
    Serial.println(inputBuffer);
    

    // Input command should be in the format "I<x>,<y>,<z>,<wrist_rotation>,<gripper_state>"
    char* token = strtok(inputBuffer + 1, ",");  
    float x, y, z, wristRotation;
    int parsed = 0;
    bool gripper = false;

    // Command is then seperated
    if (token != NULL) {
      x = atof(token);
      token = strtok(NULL, ",");
      if (token != NULL) {
        y = atof(token);
        token = strtok(NULL, ",");
        if (token != NULL) {
          z = atof(token);
          token = strtok(NULL, ",");
          if (token != NULL) {
            wristRotation = atof(token);
            parsed = 4;
            token = strtok(NULL, ",");
            if (token != NULL) {
              for (char* p = token; *p; ++p) *p = tolower(*p);
              gripper = (strcmp(token, "true") == 0);
              parsed = 5;
            }        
          }
        }
      }
    }
    
    Serial.print("Parsed values: ");
    Serial.print(parsed);

    // print parsed values    
    if (parsed == 5) {
      Serial.print("x: ");
      Serial.print(x);
      Serial.print(", y: ");
      Serial.print(y);
      Serial.print(", z: ");
      Serial.print(z);
      Serial.print(", wrist rotation: ");
      Serial.print(wristRotation);
      Serial.print(", gripper: ");
      Serial.println(gripper);
      moveArmToPosition(x, y, z, wristRotation, gripper);
      Serial.println("OK");
    } else {
      Serial.println("Invalid input format. Use I<x>,<y>,<z>,<wrist_rotation>,<true/false>");  // Debugging step
    }
  } else {
    Serial.println("Valid command");
  }
  Serial.flush();
}


void moveArmToPosition(float x, float y, float z, float wristRotation, bool gripper) {
  float a0, a1, a2, a3;
  
  
  Serial.print("Attempting to solve for: x=");
  Serial.print(x);
  Serial.print(" y=");
  Serial.print(y);
  Serial.print(" z=");
  Serial.print(z);
  Serial.print(" wrist rotation=");
  Serial.println(wristRotation);

  if(InverseK.solve(x, y, z, a0, a1, a2, a3)) {
    // Convert angles from radians to degrees
    int baseAngle = a2b(a0);
    int shoulderAngle = a2b(a1);
    int elbowAngle = a2b(a2);
    int wristAngle = a2b(a3);

    // Ensures angles are within servo range
    baseAngle = constrain(baseAngle, 0, 180);
    shoulderAngle = constrain(shoulderAngle, 15, 165);
    elbowAngle = constrain(elbowAngle, 0, 180);
    wristAngle = constrain(wristAngle, 0, 180);

    // Apply wrist rotation less then 90 anti-clockwise, above clockwise from horizontal position
    int wristRotationAngle = constrain(wristRotation, 0, 180);

    // Set gripper position
    int gripperAngle = gripper ? 73 : 0;

    armPosition.set(baseAngle, shoulderAngle, elbowAngle, wristAngle, wristRotationAngle, gripperAngle);
    
    BraccioRobot.moveToPosition(armPosition, 100);
    
    Serial.println("Moving arm to:");
    Serial.print("Base: "); Serial.println(baseAngle);
    Serial.print("Shoulder: "); Serial.println(shoulderAngle);
    Serial.print("Elbow: "); Serial.println(elbowAngle);
    Serial.print("Wrist: "); Serial.println(wristAngle);
    Serial.print("Wrist Rotation: "); Serial.println(wristRotationAngle);
    Serial.print("Gripper: "); Serial.println(gripperAngle);
  } else {
    Serial.println("No IK solution found!");
  }
}



// Quick conversion from the Braccio angle system to radians
float b2a(float b) {
  return b / 180.0 * PI - HALF_PI;
}

// Quick conversion from radians to the Braccio angle system
float a2b(float a) {
  return (a + HALF_PI) * 180 / PI;
}