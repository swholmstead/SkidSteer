#include <Arduino.h>
#include <ESP32Servo.h>  // by Kevin Harrington
#include <Bluepad32.h>

// Change log
// * Changed polarity of LED light bar so that all motor clips face the same way.

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

#define bucketServoPin 23
#define clawServoPin 22

#define armMotor0 19  // Used for controlling arm motor movement
#define armMotor1 21  // Used for controlling arm motor movement
#define lightPin0 18  // Used for controlling cab lights
#define lightPin1 5   // Used for controlling cab lights

#define rightMotor0 26 // Used for controlling the right motor movement
#define rightMotor1 25 // Used for controlling the right motor movement
#define leftMotor0 32  // Used for controlling the left motor movement
#define leftMotor1 33  // Used for controlling the left motor movement

#define dpadUp 1
#define dpadDown 2
#define dpadRight 4
#define dpadLeft 8

#define deadZone 30
#define bucketMin 10
#define bucketMax 140
#define bucketStart 122
#define bucketMoveSpeed 3
#define clawMin 10
#define clawMax 120
#define clawMoveSpeed 6

#define wiggleCountMax 6

Servo bucketServo;
Servo clawServo;

int lightSwitchTime = 0;
bool lightsOn = false;
unsigned long lastWiggleTime = 0;
int wiggleCount = 0;
int wiggleDirection = 1;
long wiggleDelay = 100;
bool shouldWiggle = false;
int bucketValue = bucketStart;
int clawValue = clawMax;
int steeringValue = 0;
const float DEG2RAD = PI / 180.0f;
const float RAD2DEG = 180.0f / PI;

void onConnectedController(ControllerPtr ctl) {
  bool foundEmptySlot = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
      // Additionally, you can get certain gamepad properties like:
      // Model, VID, PID, BTAddr, flags, etc.
      ControllerProperties properties = ctl->getProperties();
      Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id,
                    properties.product_id);
      myControllers[i] = ctl;
      ctl->playDualRumble(0 /* delayedStartMs */, 250 /* durationMs */, 0x80 /* weakMagnitude */, 0x40 /* strongMagnitude */);
      foundEmptySlot = true;
      shouldWiggle = true;
      processLights(true);
      break;
    }
  }
  if (!foundEmptySlot) {
    Serial.println("CALLBACK: Controller connected, but could not found empty slot");
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  bool foundController = false;

  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
      myControllers[i] = nullptr;
      foundController = true;
      break;
    }
  }

  if (!foundController) {
    Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
  }
}

void processGamepad(ControllerPtr ctl) {
  //Throttle
  processThrottle(ctl->axisX(), ctl->axisY());
  //Rasing and lowering of mast
  processArm(ctl->axisRY());
  //MastTilt
  processBucket(ctl->dpad());
  //Aux
  processLights(ctl->thumbR() | ctl->a());

  processWiggle(ctl->b());
}

void processWiggle(bool value) {
  if (value) {
    shouldWiggle = true;
    processLights(true);
  }
}

void wiggle() {                  
  unsigned long currentTime = millis();
  if (abs((int)(currentTime - lastWiggleTime)) >= wiggleDelay) {
    lastWiggleTime = currentTime;
    wiggleDirection = -wiggleDirection;
    wiggleCount++;
    moveMotor(leftMotor0, leftMotor1, wiggleDirection * 100);
    moveMotor(rightMotor0, rightMotor1, -1 * wiggleDirection * 100);
    if (wiggleCount >= wiggleCountMax) {
      moveMotor(rightMotor0, rightMotor1, 0);
      moveMotor(leftMotor0, leftMotor1, 0); 
      wiggleCount = 0;
      shouldWiggle = false;
      processLights(true);
    }
  }
}

void processThrottle(int newX, int newY) {
  float angle = atan2(newY, newX) * RAD2DEG;
  float amplitude = sqrtf(newX*newX + newY*newY);
  //Serial.printf("angle: %f amplitude: %f ", angle, amplitude);

  float rightMotorValue = ((angle >= 0 && angle <= 90) ? 1 : (angle <= -90 ? -1 : angle > 90 ? cos((180 - angle * 2) * DEG2RAD) : cos(angle * 2 * DEG2RAD))) * amplitude / 3;
  float leftMotorValue = (angle >= 90 ? 1 : (angle <= 0 && angle >= -90) ? -1 : angle > 0 ? cos((180 - angle * 2) * DEG2RAD) : cos(angle * 2 * DEG2RAD)) * amplitude / 3;
  //Serial.printf("left: %f right: %f\n", rightMotorValue, leftMotorValue);

  if (amplitude <= deadZone) {
    rightMotorValue = 0;
    leftMotorValue = 0;
  }
  moveMotor(rightMotor0, rightMotor1, rightMotorValue);
  moveMotor(leftMotor0, leftMotor1, leftMotorValue);
}

void processArm(int newValue) {
  if (abs(newValue) <= deadZone) {
    moveMotor(armMotor0, armMotor1, 0);
  }
  else {
    moveMotor(armMotor0, armMotor1, newValue / 3);
  }
}

void processBucket(int newValue) {
  if (newValue & dpadRight) {
    if (bucketValue < bucketMax) {
      bucketValue += bucketMoveSpeed;
      //Serial.printf("bucket: %d\n", bucketValue);
      bucketServo.write(bucketValue);
    }
  } else if (newValue & dpadLeft) {
    if (bucketValue > bucketMin) {
      bucketValue -= bucketMoveSpeed;
      //Serial.printf("bucket: %d\n", bucketValue);
      bucketServo.write(bucketValue);
    }
  } else if (newValue & dpadUp) {
    if (clawValue < clawMax) {
      clawValue += clawMoveSpeed;
      clawServo.write(clawValue);
    }
  } else if (newValue & dpadDown) {
    if (clawValue > clawMin) {
      clawValue -= clawMoveSpeed;
      clawServo.write(clawValue);
    }
  }
}

void processLights(bool buttonValue) {
  if (buttonValue && (millis() - lightSwitchTime) > 200) {
    if (lightsOn) {
      digitalWrite(lightPin0, LOW);
      digitalWrite(lightPin1, LOW);
      lightsOn = false;
    } else {
      digitalWrite(lightPin0, LOW);
      digitalWrite(lightPin1, HIGH);
      lightsOn = true;
    }

    lightSwitchTime = millis();
  }
}

void moveMotor(int motorPin0, int motorPin1, int velocity) {
  if (velocity > 1) {
    analogWrite(motorPin0, velocity);
    analogWrite(motorPin1, LOW);
  } else if (velocity < -1) {
    analogWrite(motorPin0, LOW);
    analogWrite(motorPin1, (-1 * velocity));
  } else {
    analogWrite(motorPin0, 0);
    analogWrite(motorPin1, 0);
  }
}

void processControllers() {
  for (auto myController : myControllers) {
    if (myController && myController->isConnected() && myController->hasData()) {
      if (myController->isGamepad()) {
        processGamepad(myController);
      } else {
        Serial.println("Unsupported controller");
      }
    }
  }
}

// Arduino setup function. Runs in CPU 1
void setup() {
  pinMode(armMotor0, OUTPUT);
  pinMode(armMotor1, OUTPUT);
  pinMode(lightPin0, OUTPUT);
  pinMode(lightPin1, OUTPUT);
  digitalWrite(lightPin0, LOW);
  digitalWrite(lightPin1, LOW);
  pinMode(rightMotor0, OUTPUT);
  pinMode(rightMotor1, OUTPUT);
  pinMode(leftMotor0, OUTPUT);
  pinMode(leftMotor1, OUTPUT);

  bucketServo.attach(bucketServoPin);
  bucketServo.write(bucketValue);
  clawServo.attach(clawServoPin);
  clawServo.write(clawValue);

  Serial.begin(115200);
  //   put your setup code here, to run once:
  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t *addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

  // Setup the Bluepad32 callbacks
  BP32.setup(&onConnectedController, &onDisconnectedController);

  // "forgetBluetoothKeys()" should be called when the user performs
  // a "device factory reset", or similar.
  // Calling "forgetBluetoothKeys" in setup() just as an example.
  // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
  // But it might also fix some connection / re-connection issues.
  BP32.forgetBluetoothKeys();

  // Enables mouse / touchpad support for gamepads that support them.
  // When enabled, controllers like DualSense and DualShock4 generate two connected devices:
  // - First one: the gamepad
  // - Second one, which is a "virtual device", is a mouse.
  // By default, it is disabled.
  BP32.enableVirtualDevice(false);
  // You could add additional error handling here,
  // such as logging the error or attempting to recover.
  // For example, you might attempt to reset the MCP23X17
  // and retry initialization before giving up completely.
  // Then, you could gracefully exit the program or continue
  // running with limited functionality.
}



// Arduino loop function. Runs in CPU 1.
void loop() {
  // This call fetches all the controllers' data.
  // Call this function in your main loop.
  bool dataUpdated = BP32.update();
  if (dataUpdated) {
    processControllers();
  }
  if (shouldWiggle) {
    wiggle();
  }
  // The main loop must have some kind of "yield to lower priority task" event.
  // Otherwise, the watchdog will get triggered.
  // If your main loop doesn't have one, just add a simple `vTaskDelay(1)`.
  // Detailed info here:
  // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time

  //     vTaskDelay(1);
  else { vTaskDelay(1); }
}
