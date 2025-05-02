#ifndef SATROTATORFUNCTIONS_H
#define SATROTATORFUNCTIONS_H

#include <Arduino.h>  // Needed for Serial, delay, etc.

#include <AccelStepper.h>

// Constants for step-to-degree conversion
#define STEPS_PER_TURN 400
#define GEAR_RATIO_AZI 44.0
#define GEAR_RATIO_ELE 44.0

#define AZI_DEG_PER_STEP (360.0 / (STEPS_PER_TURN * GEAR_RATIO_AZI))
#define ELE_DEG_PER_STEP (360.0 / (STEPS_PER_TURN * GEAR_RATIO_ELE))
//
// #########################################################
// #                  PIN DEFINITIONS                      #
// #########################################################
//

// Elevation Stepper Motor Pins
const int ELE_DIR_PIN  = 25;   // Direction pin
const int ELE_STEP_PIN = 33;   // Step pin
const int ELE_EN_PIN   = 32;   // Enable pin

// Azimuth Stepper Motor Pins
const int AZI_DIR_PIN  = 14;   // Direction pin
const int AZI_STEP_PIN = 27;   // Step pin
const int AZI_EN_PIN   = 26;   // Enable pin

// Status LED Pin
const int STATUS_LED_PIN = 4;  // GPIO4

// Analog Input Pins (Bournes Potentiometers)
const int AZI_Bournes_pin = 35; // Azimuth position sensor (analog)
const int ELE_Bournes_pin = 34; // Elevation position sensor (analog)

// Bournes Target Values for Parking
int AZIBournesParked = 951;    // Target value for AZI parking
int ELEBournesParked = 2834;   // Target value for ELE parking

// Optocoupler Detection Pins
const int AZIoptocouplerSensor = 19; // AZI virtual mechanical stop sensor (digital)
const int ELEoptocouplerSensor = 23; // ELE virtual mechanical stop sensor (digital)


//
// #########################################################
// #                 STEPPER DRIVER OBJECTS                #
// #########################################################
//

// AccelStepper objects
AccelStepper eleStepper(AccelStepper::DRIVER, ELE_STEP_PIN, ELE_DIR_PIN);
AccelStepper aziStepper(AccelStepper::DRIVER, AZI_STEP_PIN, AZI_DIR_PIN);


//
// #########################################################
// #                    FUNCTION PROTOTYPES                #
// #########################################################
//

// Stepper movement control
void moveELEstepper(int stepsToMove);
void moveAZIstepper(int stepsToMove);

// Stepper movement to specific angles
void moveAzimuthToAngle(float angle);
void moveElevationToAngle(float angle);

// Conversion helpers
int stepsFromAziAngle(float angle);
int stepsFromEleAngle(float angle);

// Analog reading helpers
int averageAnalogRead(int pin, int samples = 16, int delayMs = 5);

// Parking functions
void parkAzimuth();
void parkElevation();

// Test functions
void testParallelMove();












/**
 * @brief Convert an AZIMUTH angle (in degrees) to the corresponding number of motor steps.
 *
 * @param angle Azimuth angle in degrees.
 * @return Number of steps to move.
 */
int stepsFromAziAngle(float angle)
{
    return angle / AZI_DEG_PER_STEP;
}

/**
 * @brief Convert an ELEVATION angle (in degrees) to the corresponding number of motor steps.
 *
 * @param angle Elevation angle in degrees.
 * @return Number of steps to move.
 */
int stepsFromEleAngle(float angle)
{
    return angle / ELE_DEG_PER_STEP;
}



/**
 * @brief Perform the Azimuth "parking" procedure.
 * 
 * This function moves the Azimuth stepper motor until a predefined "parked" position is reached,
 * based on a Bournes potentiometer analog reading (AZI_Bournes_pin). 
 * 
 * Steps:
 * 1. Check the current AZI Bournes reading.
 * 2. If below the park target (AZIBournesParked), move FORWARD until reaching the target.
 * 3. If above the park target, move BACKWARD until reaching the target.
 * 4. Disable motor driver after parking.
 * 5. Move slightly forward (moveAZIstepper) to "unpark" and avoid mechanical stress.
 * 6. Slowly move BACKWARD step-by-step until an optocoupler (AZIoptocouplerSensor) detects a virtual mechanical stop.
 * 7. Set the current position of the AZI stepper to zero (0) for future movements.
 * 8. Disable the AZI motor driver.
 * 
 * Serial outputs and status LED are used to indicate the procedure progress.
 */
void parkAzimuth() {
    Serial.println("🅿️ Starting Azimuth Parking Procedure...");
  
    int aziValue = averageAnalogRead(AZI_Bournes_pin);
  
    if (aziValue < AZIBournesParked) {
      Serial.println("🔼 AZI value below park target. Moving FORWARD to reach park position...");
      digitalWrite(AZI_DIR_PIN, HIGH);
      digitalWrite(AZI_EN_PIN, LOW);
      digitalWrite(STATUS_LED_PIN, HIGH); // Turn ON Status LED
  
      while (averageAnalogRead(AZI_Bournes_pin, 1, 1) < AZIBournesParked) {
        digitalWrite(AZI_STEP_PIN, HIGH);
        delayMicroseconds(1000);
        digitalWrite(AZI_STEP_PIN, LOW);
        delayMicroseconds(1000);
      }
  
      Serial.print("✅ Reached Parked Position (Bournes reading: ");
      Serial.print(averageAnalogRead(AZI_Bournes_pin));
      Serial.println(")");
    }
    else if (aziValue > AZIBournesParked) {
      Serial.println("🔽 AZI value above park target. Moving BACKWARD to reach park position...");
      digitalWrite(AZI_DIR_PIN, LOW);
      digitalWrite(AZI_EN_PIN, LOW);
      digitalWrite(STATUS_LED_PIN, HIGH); // Turn ON Status LED
  
      while (averageAnalogRead(AZI_Bournes_pin, 1, 1) > AZIBournesParked) {
        digitalWrite(AZI_STEP_PIN, HIGH);
        delayMicroseconds(1000);
        digitalWrite(AZI_STEP_PIN, LOW);
        delayMicroseconds(1000);
      }
  
      Serial.print("✅ Reached Parked Position (Bournes reading: ");
      Serial.print(averageAnalogRead(AZI_Bournes_pin));
      Serial.println(")");
    }
    else {
      Serial.println("✅ Already at Parked Position. No need to move.");
    }
  
    digitalWrite(AZI_EN_PIN, HIGH);      // Disable motor after reaching parked position
    digitalWrite(STATUS_LED_PIN, LOW);   // Turn OFF Status LED
  
    // Small move forward to "unpark"
    Serial.println("➡️ Moving slightly away from parked position...");
    moveAZIstepper(1000);
  
    // Approach virtual mechanical stop
    Serial.println("🛑 Approaching virtual mechanical stop (using optocoupler)...");
    digitalWrite(AZI_DIR_PIN, LOW);
    digitalWrite(AZI_EN_PIN, LOW);
    digitalWrite(STATUS_LED_PIN, HIGH); // Turn ON Status LED
  
    int AZIoptocouplerState = digitalRead(AZIoptocouplerSensor);
  
    while (AZIoptocouplerState == 0) {
      digitalWrite(AZI_STEP_PIN, HIGH);
      delayMicroseconds(2000);
      digitalWrite(AZI_STEP_PIN, LOW);
      delayMicroseconds(2000);
      AZIoptocouplerState = digitalRead(AZIoptocouplerSensor);
    }
  
    Serial.println("✅ virtual mechanical stop detected. Setting AZI position to 0.");
  
    aziStepper.setCurrentPosition(0);
  
    digitalWrite(AZI_EN_PIN, HIGH);      // Disable motor
    digitalWrite(STATUS_LED_PIN, LOW);   // Turn OFF Status LED
  
    Serial.println("✅ Azimuth Parking Procedure Completed.\n");
  }

/**
 * @brief Perform the Elevation "parking" procedure.
 * 
 * This function moves the Elevation stepper motor until a predefined "parked" position is reached,
 * based on a Bournes potentiometer analog reading (ELE_Bournes_pin). 
 * 
 * Steps:
 * 1. Check the current ELE Bournes reading.
 * 2. If below the park target (ELEBournesParked), move FORWARD until reaching the target.
 * 3. If above the park target, move BACKWARD until reaching the target.
 * 4. Disable motor driver after parking.
 * 5. Move slightly forward (moveELEstepper) to "unpark" and avoid mechanical stress.
 * 6. Slowly move BACKWARD step-by-step until an optocoupler (ELEoptocouplerSensor) detects a virtual mechanical stop.
 * 7. Set the current position of the ELE stepper to zero (0) for future movements.
 * 8. Disable the ELE motor driver.
 * 
 * Serial outputs and status LED are used to indicate the procedure progress.
 */
void parkElevation() {
    Serial.println("🅿️ Starting Elevation Parking Procedure...");
  
    int eleValue = averageAnalogRead(ELE_Bournes_pin);
  
    if (eleValue < ELEBournesParked) {
      Serial.println("🔼 ELE value below park target. Moving FORWARD to reach park position...");
      digitalWrite(ELE_DIR_PIN, HIGH);
      digitalWrite(ELE_EN_PIN, LOW);
      digitalWrite(STATUS_LED_PIN, HIGH); // Turn ON Status LED
  
      while (averageAnalogRead(ELE_Bournes_pin, 1, 1) < ELEBournesParked) {
        digitalWrite(ELE_STEP_PIN, HIGH);
        delayMicroseconds(1000);
        digitalWrite(ELE_STEP_PIN, LOW);
        delayMicroseconds(1000);
      }
  
      Serial.print("✅ Reached Parked Position (Bournes reading: ");
      Serial.print(averageAnalogRead(ELE_Bournes_pin));
      Serial.println(")");
    }
    else if (eleValue > ELEBournesParked) {
      Serial.println("🔽 ELE value above park target. Moving BACKWARD to reach park position...");
      digitalWrite(ELE_DIR_PIN, LOW);
      digitalWrite(ELE_EN_PIN, LOW);
      digitalWrite(STATUS_LED_PIN, HIGH); // Turn ON Status LED
  
      while (averageAnalogRead(ELE_Bournes_pin, 1, 1) > ELEBournesParked) {
        digitalWrite(ELE_STEP_PIN, HIGH);
        delayMicroseconds(1000);
        digitalWrite(ELE_STEP_PIN, LOW);
        delayMicroseconds(1000);
      }
  
      Serial.print("✅ Reached Parked Position (Bournes reading: ");
      Serial.print(averageAnalogRead(ELE_Bournes_pin));
      Serial.println(")");
    }
    else {
      Serial.println("✅ Already at Parked Position. No need to move.");
    }
  
    digitalWrite(ELE_EN_PIN, HIGH);      // Disable motor after reaching parked position
    digitalWrite(STATUS_LED_PIN, LOW);   // Turn OFF Status LED
  
    // Small move forward to "unpark"
    Serial.println("➡️ Moving slightly away from parked position...");
    moveELEstepper(1000);
  
    // Approach virtual mechanical stop
    Serial.println("🛑 Approaching virtual mechanical stop (using optocoupler)...");
    digitalWrite(ELE_DIR_PIN, LOW);
    digitalWrite(ELE_EN_PIN, LOW);
    digitalWrite(STATUS_LED_PIN, HIGH); // Turn ON Status LED
  
    int ELEoptocouplerState = digitalRead(ELEoptocouplerSensor);
  
    while (ELEoptocouplerState == 0) {
      digitalWrite(ELE_STEP_PIN, HIGH);
      delayMicroseconds(2000);
      digitalWrite(ELE_STEP_PIN, LOW);
      delayMicroseconds(2000);
      ELEoptocouplerState = digitalRead(ELEoptocouplerSensor);
    }
  
    Serial.println("✅ virtual mechanical stop detected. Setting ELE position to 0.");
  
    eleStepper.setCurrentPosition(0);
  
    digitalWrite(ELE_EN_PIN, HIGH);      // Disable motor
    digitalWrite(STATUS_LED_PIN, LOW);   // Turn OFF Status LED
  
    Serial.println("✅ Elevation Parking Procedure Completed.\n");
  }
  

/**
 * @brief Move the ELEVATION stepper motor by a specified number of steps.
 *
 * @param stepsToMove Number of steps to move. Positive = Up, Negative = Down.
 */
void moveELEstepper(int stepsToMove)
{
    Serial.print("📡 Moving ELEVATION stepper ");
    Serial.print(stepsToMove);
    Serial.println(" steps...");

    digitalWrite(STATUS_LED_PIN, HIGH); // Turn ON Status LED
    digitalWrite(ELE_EN_PIN, LOW);      // Enable ELEVATION motor driver

    eleStepper.move(stepsToMove); // Move relative to current position
    while (eleStepper.distanceToGo() != 0)
    {
        eleStepper.run();
    }

    digitalWrite(ELE_EN_PIN, HIGH);    // Disable ELEVATION motor driver
    digitalWrite(STATUS_LED_PIN, LOW); // Turn OFF Status LED

    Serial.println("✅ ELEVATION Movement completed.");
}

/**
 * @brief Move the AZIMUTH stepper motor by a specified number of steps.
 *
 * @param stepsToMove Number of steps to move. Positive = CW, Negative = CCW.
 */
void moveAZIstepper(int stepsToMove)
{
    Serial.print("🧭 Moving AZIMUTH stepper ");
    Serial.print(stepsToMove);
    Serial.println(" steps...");

    digitalWrite(STATUS_LED_PIN, HIGH); // Turn ON Status LED
    digitalWrite(AZI_EN_PIN, LOW);      // Enable AZIMUTH motor driver

    aziStepper.move(stepsToMove); // Move relative to current position
    while (aziStepper.distanceToGo() != 0)
    {
        aziStepper.run();
    }

    digitalWrite(AZI_EN_PIN, HIGH);    // Disable AZIMUTH motor driver
    digitalWrite(STATUS_LED_PIN, LOW); // Turn OFF Status LED

    Serial.println("✅ AZIMUTH Movement completed.");
}

/**
 * @brief Move the AZIMUTH stepper motor to a specified absolute angle.
 *
 * This function moves the AZIMUTH motor to the given target angle, always 
 * referring to the home (0°) position. It uses absolute movement, 
 * not relative steps. The motor stays enabled after the move to maintain 
 * holding torque. 
 * Additional debug info: current position, target position, steps to go.
 *
 * @param angle Target azimuth angle in degrees (0° = Home).
 */
void moveAzimuthToAngle(float angle)
{
    Serial.print("🧭 Moving AZIMUTH to ");
    Serial.print(angle);
    Serial.println(" degrees (absolute)...");

    int targetSteps = stepsFromAziAngle(angle);
    int currentSteps = aziStepper.currentPosition();
    int stepsToGo = targetSteps - currentSteps;

    Serial.printf("📈 AZI current pos: %d steps\n", currentSteps);
    Serial.printf("🎯 AZI target pos : %d steps\n", targetSteps);
    Serial.printf("🛤  AZI steps to go: %d steps\n", stepsToGo);

    digitalWrite(AZI_EN_PIN, LOW);       // Enable AZIMUTH motor
    digitalWrite(STATUS_LED_PIN, HIGH);  // Turn ON Status LED

    aziStepper.moveTo(targetSteps);      // Absolute move
    while (aziStepper.distanceToGo() != 0)
    {
        aziStepper.run();
    }

    digitalWrite(STATUS_LED_PIN, LOW);   // Turn OFF Status LED
    Serial.println("✅ AZIMUTH Move to Angle completed.\n");
}

/**
 * @brief Move the ELEVATION stepper motor to a specified absolute angle.
 *
 * This function moves the ELEVATION motor to the given target angle, always 
 * referring to the home (0°) position. It uses absolute movement,
 * not relative steps. The motor stays enabled after the move to maintain 
 * holding torque.
 * Additional debug info: current position, target position, steps to go.
 *
 * @param angle Target elevation angle in degrees (0° = Home).
 */
void moveElevationToAngle(float angle)
{
    Serial.print("📡 Moving ELEVATION to ");
    Serial.print(angle);
    Serial.println(" degrees (absolute)...");

    int targetSteps = stepsFromEleAngle(angle);
    int currentSteps = eleStepper.currentPosition();
    int stepsToGo = targetSteps - currentSteps;

    Serial.printf("📈 ELE current pos: %d steps\n", currentSteps);
    Serial.printf("🎯 ELE target pos : %d steps\n", targetSteps);
    Serial.printf("🛤  ELE steps to go: %d steps\n", stepsToGo);

    digitalWrite(ELE_EN_PIN, LOW);       // Enable ELEVATION motor
    digitalWrite(STATUS_LED_PIN, HIGH);  // Turn ON Status LED

    eleStepper.moveTo(targetSteps);      // Absolute move
    while (eleStepper.distanceToGo() != 0)
    {
        eleStepper.run();
    }

    digitalWrite(STATUS_LED_PIN, LOW);   // Turn OFF Status LED
    Serial.println("✅ ELEVATION Move to Angle completed.\n");
}

// Function to average ADC readings
int averageAnalogRead(int pin, int samples, int delayMs)
{

    long total = 0;
    for (int i = 0; i < samples; i++)
    {
        total += analogRead(pin);
        delay(delayMs);
    }
    return total / samples;
}

/**
 * @brief Perform a synchronized test move for both AZIMUTH and ELEVATION steppers.
 * 
 * This function moves AZIMUTH from 0° to 360° and ELEVATION from 0° to 180° simultaneously,
 * then returns both back to 0°. It adjusts the speeds proportionally so that both movements
 * complete at the same time.
 * 
 * The status LED is turned ON while motors are moving and OFF when done.
 */
void testParallelMove() {
    Serial.println("🛠 Starting synchronized AZI/ELE test...");
  
    digitalWrite(ELE_EN_PIN, LOW); // Enable ELEVATION motor
    digitalWrite(AZI_EN_PIN, LOW); // Enable AZIMUTH motor
    digitalWrite(STATUS_LED_PIN, HIGH); // Turn ON Status LED
  
    eleStepper.setCurrentPosition(0);
    aziStepper.setCurrentPosition(0);
  
    int aziTargetSteps = stepsFromAziAngle(360.0);
    int eleTargetSteps = stepsFromEleAngle(180.0);
  
    Serial.print("🔢 AZI target steps: ");
    Serial.println(aziTargetSteps);
    Serial.print("🔢 ELE target steps: ");
    Serial.println(eleTargetSteps);
  
    int maxSteps = max(abs(aziTargetSteps), abs(eleTargetSteps));
  
    const float baseSpeed = 1000.0; // Base speed in steps/second
    aziStepper.setMaxSpeed(baseSpeed * abs(aziTargetSteps) / maxSteps);
    eleStepper.setMaxSpeed(baseSpeed * abs(eleTargetSteps) / maxSteps);
  
    aziStepper.setAcceleration(500);
    eleStepper.setAcceleration(500);
  
    aziStepper.moveTo(aziTargetSteps);
    eleStepper.moveTo(eleTargetSteps);
  
    Serial.println("🚀 Moving AZI 0➔360 and ELE 0➔180...");
  
    while (aziStepper.distanceToGo() != 0 || eleStepper.distanceToGo() != 0) {
      aziStepper.run();
      eleStepper.run();
    }
  
    delay(1000); // Pause at end
  
    // Prepare return trip
    aziStepper.setMaxSpeed(baseSpeed * abs(aziTargetSteps) / maxSteps);
    eleStepper.setMaxSpeed(baseSpeed * abs(eleTargetSteps) / maxSteps);
  
    Serial.println("↩️ Returning AZI 360➔0 and ELE 180➔0...");
  
    aziStepper.moveTo(0);
    eleStepper.moveTo(0);
  
    while (aziStepper.distanceToGo() != 0 || eleStepper.distanceToGo() != 0) {
      aziStepper.run();
      eleStepper.run();
    }
  
    digitalWrite(ELE_EN_PIN, HIGH); // Disable ELEVATION motor
    digitalWrite(AZI_EN_PIN, HIGH); // Disable AZIMUTH motor
    digitalWrite(STATUS_LED_PIN, LOW); // Turn OFF Status LED
  
    Serial.println("✅ Synchronized AZI/ELE move test completed!");
  }
  

#endif
